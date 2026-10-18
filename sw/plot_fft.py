#!/usr/bin/env python3
"""
plot_fft.py - receive FFT frames from the MicroBlaze (fft_uart.c) over UART
and plot magnitude (dB), real and imaginary parts.

Packet format (matches fft_uart.c):
    b'FFT:'            4-byte sync header
    N                  uint16 little-endian, number of FFT points
    BPS                uint8, bytes per sample
                         4 -> int16 real, int16 imag
                         2 -> int8 real,  int8 imag  (8-bit scaled FFT)
    payload            N * BPS bytes
    checksum           uint16 little-endian = sum(payload bytes) & 0xFFFF

Because every frame starts with a sync header and the DMA frames are cut by
the FFT's TLAST, frames are always aligned: no --roll / --autoroll needed.

Install:  pip install pyserial numpy matplotlib

Changing the DDS frequency:
  while it runs, type a frequency in this terminal and press Enter
  (791015, 5M, 2.5k ...). It is sent to the board as "F <Hz>\\n".
  --freq 5M sets it once at start-up.

Examples:
  python plot_fft.py --port COM5
  python plot_fft.py --port COM6 --shift --fs 81e6 --freq 791015
  python plot_fft.py --port /dev/ttyUSB1 --baud 115200 --shift --fs 100e6
  python plot_fft.py --port COM5 --record capture.bin   # also save raw stream
  python plot_fft.py --file capture.bin                 # replay a saved capture
"""
import argparse
import io
import queue
import struct
import threading

import numpy as np
import matplotlib.pyplot as plt

SYNC = b"FFT:"


# --------------------------------------------------------------------------
# Packet reading (shared by live serial and --file replay)
# --------------------------------------------------------------------------
class Source:
    """Wraps a serial port or a file. Optionally records raw bytes."""

    def __init__(self, stream, stop, eof_on_empty, record=None):
        self.stream = stream
        self.stop = stop
        self.eof_on_empty = eof_on_empty  # True for files, False for serial
        self.record = record
        self.text = bytearray()           # text lines printed by the board

    def read_exact(self, n):
        """Read exactly n bytes; None on stop request or end of file."""
        buf = bytearray()
        while len(buf) < n:
            if self.stop.is_set():
                return None
            chunk = self.stream.read(n - len(buf))
            if not chunk:
                if self.eof_on_empty:
                    return None
                continue  # serial timeout, keep waiting
            if self.record:
                self.record.write(chunk)
            buf += chunk
        return bytes(buf)


def parse_payload(payload, bps):
    """Raw bytes -> complex array (real in low half, imag in high half)."""
    dtype = np.int8 if bps == 2 else np.dtype("<i2")
    data = np.frombuffer(payload, dtype=dtype).reshape(-1, 2)
    return data[:, 0].astype(np.float64) + 1j * data[:, 1].astype(np.float64)


def show_board_text(src, b):
    """Collect bytes outside packets and print complete readable lines."""
    if b == b"\n":
        line = src.text.decode("ascii", errors="ignore").strip()
        if line and all(32 <= ord(c) < 127 for c in line):
            print(f"[board] {line}")
        src.text.clear()
    elif b != b"\r":
        src.text += b
        if len(src.text) > 200:
            src.text.clear()


def parse_freq(text):
    """'5M', '791015', '2.5k', 'F 1e6' -> integer Hz (raises ValueError)."""
    t = text.strip().lstrip("Ff").strip()
    mult = 1
    if t[-1:] in ("k", "K"):
        mult, t = 1e3, t[:-1]
    elif t[-1:] in ("M", "m"):
        mult, t = 1e6, t[:-1]
    hz = round(float(t) * mult)
    if hz < 0:
        raise ValueError("frequency must be positive")
    return hz


def send_freq(ser, hz):
    ser.write(f"F {hz}\n".encode("ascii"))
    print(f"-> setting DDS to {hz} Hz")


def read_packet(src):
    """Return the next valid frame as a complex array, or None when done."""
    window = bytearray()
    while True:
        b = src.read_exact(1)
        if b is None:
            return None
        window += b
        if len(window) > len(SYNC):
            del window[0]
        if window != SYNC:
            show_board_text(src, b)  # xil_printf text between frames
            continue
        src.text.clear()

        hdr = src.read_exact(3)
        if hdr is None:
            return None
        n, bps = struct.unpack("<HB", hdr)
        if n == 0 or bps not in (2, 4):
            print(f"Bad header (N={n}, BPS={bps}), resyncing")
            window.clear()
            continue

        payload = src.read_exact(n * bps)
        ck = src.read_exact(2)
        if payload is None or ck is None:
            return None
        if (sum(payload) & 0xFFFF) != struct.unpack("<H", ck)[0]:
            print("Checksum mismatch, frame dropped")
            window.clear()
            continue

        return parse_payload(payload, bps)


# --------------------------------------------------------------------------
# Plotting
# --------------------------------------------------------------------------
def make_plot(fs):
    plt.ion()
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))
    xlabel = "Bin" if fs is None else "Frequency (Hz)"
    lines = []
    for ax, title in zip(axs, ["Magnitude (dB)", "Real", "Imag"]):
        (ln,) = ax.plot([], [], lw=1)
        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.grid(True, alpha=0.3)
        lines.append(ln)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    return fig, axs, lines


def x_axis(n, shift, fs):
    if fs is None:
        x = np.arange(n)
        return x - n // 2 if shift else x
    x = np.fft.fftfreq(n, 1 / fs)
    return np.fft.fftshift(x) if shift else x


def update(fig, axs, lines, samples, shift, fs, frame_no):
    n = len(samples)
    if shift:
        samples = np.fft.fftshift(samples)
    mag = np.abs(samples)
    mag_db = 20 * np.log10(np.maximum(mag, 1.0))  # floor at 0 dB
    x = x_axis(n, shift, fs)

    for ln, y in zip(lines, [mag_db, samples.real, samples.imag]):
        ln.set_data(x, y)
    for ax in axs:
        ax.relim()
        ax.autoscale_view()

    peak = int(np.argmax(mag))
    where = f"{x[peak]:.4g} Hz" if fs else f"bin {x[peak]}"
    fig.suptitle(f"Frame {frame_no}  (N = {n})   peak at {where}  (|X| = {mag[peak]:.0f})")
    print(f"frame {frame_no}: N={n}, peak at {where}, |X|={mag[peak]:.0f}")
    fig.canvas.draw_idle()
    plt.pause(0.001)


# --------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description="Plot FFT frames sent by fft_uart.c")
    p.add_argument("--port", help="Serial port, e.g. COM5 or /dev/ttyUSB1")
    p.add_argument("--baud", type=int, default=115200,
                   help="Must match UART_BAUD in fft_uart.c (default 115200)")
    p.add_argument("--file", help="Replay a raw capture saved with --record")
    p.add_argument("--record", help="Also save the raw serial stream to this file")
    p.add_argument("--fs", type=float, default=None, help="Sample rate in Hz for the x-axis")
    p.add_argument("--shift", action="store_true", help="fftshift (DC in the centre)")
    p.add_argument("--save", help="Save the last frame to this CSV file")
    p.add_argument("--freq", help="Set the DDS frequency at start, e.g. 791015, 5M, 2.5k")
    args = p.parse_args()

    fig, axs, lines = make_plot(args.fs)
    stop = threading.Event()
    last = None

    # ---- Replay a saved capture -------------------------------------------
    if args.file:
        with open(args.file, "rb") as f:
            src = Source(io.BytesIO(f.read()), stop, eof_on_empty=True)
        frame_no = 0
        while plt.fignum_exists(fig.number):
            samples = read_packet(src)
            if samples is None:
                break
            last = samples
            update(fig, axs, lines, samples, args.shift, args.fs, frame_no)
            frame_no += 1
            plt.pause(0.5)
        print(f"{frame_no} frames in {args.file}")
        save_csv(args.save, last)
        plt.ioff()
        plt.show()
        return

    # ---- Live serial ------------------------------------------------------
    if not args.port:
        p.error("--port is required unless --file is given")

    import serial  # pip install pyserial

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    ser.reset_input_buffer()
    rec = open(args.record, "wb") if args.record else None
    src = Source(ser, stop, eof_on_empty=False, record=rec)
    print(f"Listening on {args.port} @ {args.baud} baud. Ctrl+C to stop.")

    # Serial reads can block for a while; reading in the GUI thread would
    # freeze the window, so a background thread does the reading.
    frames = queue.Queue(maxsize=4)

    def reader():
        while not stop.is_set():
            try:
                samples = read_packet(src)
            except Exception as e:  # port closed / unplugged
                print("Reader stopped:", e)
                return
            if samples is None:
                return
            try:
                frames.put(samples, timeout=1)
            except queue.Full:
                pass  # GUI is behind; drop this frame

    t = threading.Thread(target=reader, daemon=True)
    t.start()

    if args.freq:
        send_freq(ser, parse_freq(args.freq))

    # Type a frequency + Enter in this terminal to change the DDS
    def keyboard():
        print("Type a DDS frequency and press Enter (e.g. 791015, 5M, 2.5k).")
        while not stop.is_set():
            try:
                line = input()
            except (EOFError, KeyboardInterrupt):
                return
            if not line.strip():
                continue
            try:
                send_freq(ser, parse_freq(line))
            except (ValueError, IndexError):
                print("Not a frequency. Examples: 791015   5M   2.5k")
            except Exception as e:
                print("Write failed:", e)
                return

    threading.Thread(target=keyboard, daemon=True).start()

    frame_no = 0
    try:
        while plt.fignum_exists(fig.number):
            try:
                samples = frames.get_nowait()
            except queue.Empty:
                plt.pause(0.05)  # keep the window responsive while waiting
                continue
            last = samples
            update(fig, axs, lines, samples, args.shift, args.fs, frame_no)
            frame_no += 1
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        t.join(timeout=2)
        ser.close()
        if rec:
            rec.close()
            print(f"Raw stream saved to {args.record}")
        save_csv(args.save, last)


def save_csv(path, samples):
    if path and samples is not None:
        np.savetxt(path, np.column_stack([samples.real, samples.imag, np.abs(samples)]),
                   delimiter=",", header="real,imag,magnitude", comments="", fmt="%.6g")
        print(f"Saved last frame to {path}")


if __name__ == "__main__":
    main()
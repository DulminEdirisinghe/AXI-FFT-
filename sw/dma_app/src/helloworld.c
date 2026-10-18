/******************************************************************************
 * fft_uart.c
 *
 * DDS -> AXI FFT -> AXI DMA (S2MM) -> DDR3 -> UART 16550 -> PC
 *            ^
 *            +-- DDS frequency set at run time over the same UART
 *                (MicroBlaze -> AXI4-Stream FIFO -> DDS S_AXIS_CONFIG)
 *
 * Captures one FFT frame at a time into DDR3 with the AXI DMA (Direct
 * Register mode, no Scatter Gather), then sends it over the AXI UART 16550
 * as a binary packet:
 *
 *   'F' 'F' 'T' ':'   4-byte sync header
 *   N                 uint16, little-endian (number of FFT points)
 *   BPS               uint8, bytes per sample (2 or 4)
 *   payload           N samples, little-endian
 *                     BPS = 4: [15:0] = real (int16), [31:16] = imag (int16)
 *                     BPS = 2: [7:0]  = real (int8),  [15:8]  = imag (int8)
 *   checksum          uint16, little-endian = sum of payload bytes & 0xFFFF
 *
 * Commands received on the UART (one per line, ending in Enter / '\n'):
 *   F <freq>      set DDS output frequency, e.g.  F 791015   F 5M   F 2.5k
 *                 (the leading 'F' is optional; suffixes k and M allowed)
 * The board replies with a text line:  OK F=<actual Hz> PINC=0x........
 *
 ******************************************************************************/

#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xil_types.h"
#include "xil_io.h"
#include "xstatus.h"
#include "xil_cache.h"
#include "xaxidma.h"
#include "xuartns550_l.h"


/* FFT transform length */
#define FFT_N               1024

/* Bytes per FFT output sample (= FFT output TDATA width / 8):
 *   2 =  8-bit real +  8-bit imag (16-bit TDATA, 8-bit scaled FFT) */
#define BYTES_PER_SAMPLE    2
#define FFT_FRAME_BYTES     (FFT_N * BYTES_PER_SAMPLE)

#define UART_BAUD           115200

/* --- DDS Compiler settings ---------------------------------------------- */
/* Clock driving the DDS aclk */
/*Same clock as the microblaze frequency in the xpar*/
#define DDS_CLK_HZ          81247969ULL

/* Phase Width set in the DDS Compiler */
#define DDS_PHASE_WIDTH     32
/* Frequency loaded at start-up (a programmable DDS resets to 0 Hz = DC).
 * 791015 Hz is FFT bin 10 for 81 MHz / 1024 points. */
#define DDS_DEFAULT_FREQ_HZ 79343U

/* Frames thrown away after a frequency change (they contain the old tone
 * or the switch-over). */
#define DISCARD_AFTER_CHANGE 2

/* --- AXI UART 16550 base address -------------------------------------- */
#if defined(XPAR_AXI_UART16550_0_BASEADDR)
  #define UART_BASEADDR     XPAR_AXI_UART16550_0_BASEADDR
#elif defined(STDOUT_BASEADDRESS)
  #define UART_BASEADDR     STDOUT_BASEADDRESS
#else
  #error "Set UART_BASEADDR to your AXI UART 16550 base address"
#endif

/* --- AXI UART 16550 input clock (s_axi_aclk) in Hz --------------------- */
#if defined(XPAR_XUARTNS550_CLOCK_HZ)
  #define UART_CLK_HZ       XPAR_XUARTNS550_CLOCK_HZ
#elif defined(XPAR_AXI_UART16550_0_CLOCK_FREQ_HZ)
  #define UART_CLK_HZ       XPAR_AXI_UART16550_0_CLOCK_FREQ_HZ
#elif defined(XPAR_AXI_UART16550_0_CLOCK_FREQ)
  #define UART_CLK_HZ       XPAR_AXI_UART16550_0_CLOCK_FREQ
#else
  /* Fallback: microblaze frequency */
  #define UART_CLK_HZ       81247969ULL
#endif

/* --- AXI4-Stream FIFO (axi_fifo_mm_s) feeding the DDS config channel --- */
#if defined(XPAR_AXI_FIFO_MM_S_0_BASEADDR) 
  #define DDS_FIFO_BASEADDR XPAR_AXI_FIFO_MM_S_0_BASEADDR
#elif defined(XPAR_XLLFIFO_0_BASEADDR)
  #define DDS_FIFO_BASEADDR XPAR_XLLFIFO_0_BASEADDR
#elif defined(XPAR_AXI_FIFO_0_BASEADDR)
  #define DDS_FIFO_BASEADDR XPAR_AXI_FIFO_0_BASEADDR
#else
  #error "Set DDS_FIFO_BASEADDR to your AXI4-Stream FIFO base address."
#endif

/* AXI4-Stream FIFO register offsets */
#define FIFO_ISR            0x00    /* interrupt status */
#define FIFO_TDFR           0x08    /* transmit data FIFO reset (write 0xA5) */
#define FIFO_TDFV           0x0C    /* transmit data FIFO vacancy */
#define FIFO_TDFD           0x10    /* transmit data FIFO data */
#define FIFO_TLR            0x14    /* transmit length; writing it sends the packet */

/* --- DMA receive buffer in DDR3 ----------------------------------------
 * The DMA cannot write to MicroBlaze local (LMB) BRAM, so the buffer must
 * be in DDR. A fixed address 16 MB into DDR keeps it clear of the program. */
#if defined(XPAR_MIG_0_BASEADDRESS)
  #define DDR_BASEADDR      XPAR_MIG_0_BASEADDRESS
#elif defined(XPAR_MIG_0_BASEADDR)
  #define DDR_BASEADDR      XPAR_MIG_0_BASEADDR
#elif defined(XPAR_MIG7SERIES_0_BASEADDR)
  #define DDR_BASEADDR      XPAR_MIG7SERIES_0_BASEADDR
#elif defined(XPAR_MIG_7SERIES_0_BASEADDR)
  #define DDR_BASEADDR      XPAR_MIG_7SERIES_0_BASEADDR
#else
  #error "Set DDR_BASEADDR to your MIG DDR3 base address"
#endif

#define RX_BUFFER_ADDR      ((UINTPTR)(DDR_BASEADDR + 0x01000000))

/* Busy-wait limit for one DMA transfer (loop iterations, roughly 1 s) */
#define DMA_TIMEOUT         10000000U

/* ------------------------------------------------------------------------- */

static XAxiDma AxiDma;

/* Command line being typed on the PC */
#define CMD_MAX 32
static char cmd_buf[CMD_MAX];
static u32  cmd_len;
static int  cmd_ready;

/* ===========================================================================
 * UART
 * ======================================================================== */

/* Wait until the UART has finished sending everything in its FIFO. */
static void uart_wait_tx_empty(void)
{
    while (!(XUartNs550_ReadReg(UART_BASEADDR, XUN_LSR_OFFSET) & XUN_LSR_TX_EMPTY))
        ;
}

/* Set the UART to UART_BAUD, 8 data bits, no parity, 1 stop bit, and turn
 * on its 16-byte FIFOs so incoming commands are not lost while we are busy.
 * (The default BSP setup for UART 16550 is 9600 baud, which is too slow.) */
static void uart_setup(void)
{
    uart_wait_tx_empty();
    XUartNs550_SetBaud(UART_BASEADDR, UART_CLK_HZ, UART_BAUD);
    XUartNs550_SetLineControlReg(UART_BASEADDR, XUN_LCR_8_DATA_BITS);
    XUartNs550_WriteReg(UART_BASEADDR, XUN_FCR_OFFSET,
                        XUN_FIFO_ENABLE | XUN_FIFO_RX_RESET | XUN_FIFO_TX_RESET);
}

/* Collect any received characters into cmd_buf (non-blocking).
 * Called often, including while a frame is being sent. */
static void uart_poll_rx(void)
{
    while (XUartNs550_IsReceiveData(UART_BASEADDR)) {
        char c = (char)XUartNs550_RecvByte(UART_BASEADDR);

        if (cmd_ready)
            continue;               /* previous command not handled yet */

        if (c == '\r' || c == '\n') {
            if (cmd_len > 0) {
                cmd_buf[cmd_len] = '\0';
                cmd_ready = 1;
            }
        } else if (cmd_len < CMD_MAX - 1) {
            cmd_buf[cmd_len++] = c;
        }
    }
}

static void uart_send_byte(u8 b)
{
    XUartNs550_SendByte(UART_BASEADDR, b);   /* blocking */
}

/* Send one FFT frame as a binary packet. */
static void uart_send_frame(const u8 *data, u16 n_points)
{
    u32 len = (u32)n_points * BYTES_PER_SAMPLE;
    u16 checksum = 0;

    uart_send_byte('F');
    uart_send_byte('F');
    uart_send_byte('T');
    uart_send_byte(':');

    uart_send_byte((u8)(n_points & 0xFF));
    uart_send_byte((u8)(n_points >> 8));
    uart_send_byte((u8)BYTES_PER_SAMPLE);

    for (u32 i = 0; i < len; i++) {
        uart_send_byte(data[i]);
        checksum += data[i];
        uart_poll_rx();             /* don't miss commands during a frame */
    }

    uart_send_byte((u8)(checksum & 0xFF));
    uart_send_byte((u8)(checksum >> 8));
}

/* ===========================================================================
 * DDS run-time configuration (through the AXI4-Stream FIFO)
 * ======================================================================== */

static void dds_fifo_init(void)
{
    Xil_Out32(DDS_FIFO_BASEADDR + FIFO_TDFR, 0xA5);        /* reset TX FIFO */
    Xil_Out32(DDS_FIFO_BASEADDR + FIFO_ISR, 0xFFFFFFFF);   /* clear status */
}

/* PINC = f_out * 2^PhaseWidth / f_clk  (rounded) */
static u32 dds_freq_to_pinc(u32 freq_hz)
{
    u64 num = ((u64)freq_hz << DDS_PHASE_WIDTH) + DDS_CLK_HZ / 2;
    return (u32)(num / DDS_CLK_HZ);
}

/* Frequency the DDS will actually produce for a given PINC */
static u32 dds_pinc_to_freq(u32 pinc)
{
    u64 num = (u64)pinc * DDS_CLK_HZ + (1ULL << (DDS_PHASE_WIDTH - 1));
    return (u32)(num >> DDS_PHASE_WIDTH);
}

/* Send one 32-bit config word (PINC) to the DDS S_AXIS_CONFIG channel */
static void dds_set_pinc(u32 pinc)
{
    while (Xil_In32(DDS_FIFO_BASEADDR + FIFO_TDFV) == 0)
        ;                                                  /* wait for space */
    Xil_Out32(DDS_FIFO_BASEADDR + FIFO_TDFD, pinc);
    Xil_Out32(DDS_FIFO_BASEADDR + FIFO_TLR, 4);            /* send 4 bytes */
}

static void dds_set_freq(u32 freq_hz)
{
    u32 pinc = dds_freq_to_pinc(freq_hz);
    dds_set_pinc(pinc);
    xil_printf("OK F=%d Hz PINC=0x%08x\r\n", (int)dds_pinc_to_freq(pinc), pinc);
}

/* Parse "F 5M", "791015", "2.5k", "f 1000000" -> Hz. Returns 0 on success. */
static int parse_freq(const char *s, u32 *out_hz)
{
    u64 mant = 0, mult = 1, div = 1;
    int digits = 0, frac = 0;

    while (*s == ' ' || *s == '\t') s++;
    if (*s == 'F' || *s == 'f') s++;
    while (*s == ' ' || *s == '\t') s++;

    for (; *s; s++) {
        if (*s >= '0' && *s <= '9') {
            if (digits >= 15) return -1;          /* too many digits */
            mant = mant * 10 + (u64)(*s - '0');
            digits++;
            if (frac) div *= 10;
        } else if (*s == '.' && !frac) {
            frac = 1;
        } else {
            break;
        }
    }
    if (digits == 0) return -1;

    if (*s == 'k' || *s == 'K')      { mult = 1000ULL;    s++; }
    else if (*s == 'M' || *s == 'm') { mult = 1000000ULL; s++; }
    while (*s == ' ' || *s == '\t') s++;
    if (*s != '\0') return -1;                    /* junk after the number */

    u64 hz = (mant * mult + div / 2) / div;
    if (hz > DDS_CLK_HZ / 2) return -1;           /* above Nyquist */

    *out_hz = (u32)hz;
    return 0;
}

/* Returns 1 if the DDS frequency was changed */
static int handle_command(void)
{
    u32 hz;
    int changed = 0;

    if (!cmd_ready)
        return 0;

    if (parse_freq(cmd_buf, &hz) == 0) {
        dds_set_freq(hz);
        changed = 1;
    } else {
        xil_printf("ERR bad command '%s' (use e.g. F 791015, F 5M; max %d Hz)\r\n",
                   cmd_buf, (int)(DDS_CLK_HZ / 2));
    }

    cmd_len = 0;
    cmd_ready = 0;
    return changed;
}

/* ===========================================================================
 * DMA
 * ======================================================================== */

static int dma_setup(void)
{
    XAxiDma_Config *cfg;

#ifdef SDT
    cfg = XAxiDma_LookupConfig(XPAR_XAXIDMA_0_BASEADDR);
#else
    cfg = XAxiDma_LookupConfig(XPAR_AXIDMA_0_DEVICE_ID);
#endif
    if (cfg == NULL) {
        xil_printf("ERROR: no AXI DMA config found\r\n");
        return XST_FAILURE;
    }

    if (XAxiDma_CfgInitialize(&AxiDma, cfg) != XST_SUCCESS) {
        xil_printf("ERROR: AXI DMA init failed\r\n");
        return XST_FAILURE;
    }

    if (XAxiDma_HasSg(&AxiDma)) {
        xil_printf("ERROR: DMA is in Scatter Gather mode; disable SG in the IP\r\n");
        return XST_FAILURE;
    }

    /* Polling mode: turn off DMA interrupts */
    XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);

    return XST_SUCCESS;
}

static void dma_reset(void)
{
    XAxiDma_Reset(&AxiDma);
    while (!XAxiDma_ResetIsDone(&AxiDma))
        ;
    XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
}

/* Capture one FFT frame into DDR. Returns number of bytes received, or -1. */
static int dma_capture_frame(UINTPTR buf)
{
    UINTPTR rx_base = AxiDma.RegBase + XAXIDMA_RX_OFFSET;
    u32 timeout = DMA_TIMEOUT;
    u32 status;

    /* Make sure no dirty cache lines get written over the DMA data later */
    Xil_DCacheFlushRange(buf, FFT_FRAME_BYTES);

    if (XAxiDma_SimpleTransfer(&AxiDma, buf, FFT_FRAME_BYTES,
                               XAXIDMA_DEVICE_TO_DMA) != XST_SUCCESS) {
        return -1;
    }

    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA)) {
        uart_poll_rx();
        if (--timeout == 0) {
            xil_printf("ERROR: DMA timeout (is the FFT producing data / TLAST?)\r\n");
            dma_reset();
            return -1;
        }
    }

    status = XAxiDma_ReadReg(rx_base, XAXIDMA_SR_OFFSET);
    if (status & XAXIDMA_ERR_ALL_MASK) {
        xil_printf("ERROR: DMA error, status = 0x%08x\r\n", status);
        dma_reset();
        return -1;
    }

    /* Discard stale cache lines so the CPU reads what the DMA wrote */
    Xil_DCacheInvalidateRange(buf, FFT_FRAME_BYTES);

    /* Actual number of bytes received (frame ended by TLAST) */
    return (int)XAxiDma_ReadReg(rx_base, XAXIDMA_BUFFLEN_OFFSET);
}

/* ===========================================================================
 * main
 * ======================================================================== */

int main(void)
{
    int rx_bytes;
    int discard = DISCARD_AFTER_CHANGE;

    init_platform();
    uart_setup();

    xil_printf("\r\nDDS -> FFT -> DMA -> UART  (N = %d, baud = %d)\r\n",
               FFT_N, UART_BAUD);

    dds_fifo_init();
    dds_set_freq(DDS_DEFAULT_FREQ_HZ);

    if (dma_setup() != XST_SUCCESS) {
        cleanup_platform();
        return XST_FAILURE;
    }

    while (1) {
        uart_poll_rx();
        if (handle_command())
            discard = DISCARD_AFTER_CHANGE;

        rx_bytes = dma_capture_frame(RX_BUFFER_ADDR);
        if (rx_bytes < 0)
            continue;

        /* A short frame means the DMA started mid-frame; skip it */
        if (rx_bytes != FFT_FRAME_BYTES) {
            xil_printf("Skipping partial frame (%d bytes)\r\n", rx_bytes);
            continue;
        }

        /* Frames captured right after a frequency change are mixed */
        if (discard > 0) {
            discard--;
            continue;
        }

        uart_send_frame((const u8 *)RX_BUFFER_ADDR, FFT_N);
    }

    cleanup_platform();
    return XST_SUCCESS;
}
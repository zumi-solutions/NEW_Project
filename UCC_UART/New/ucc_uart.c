/*FReescale QUICC Engine UART device driver
 *
 * Author: Timur Tabi <timur@freescale.com>
 *
 * Copyright 2007 Freescale Semiconductor, Inc.  This file is licensed under
 * the terms of the GNU General Public License version 2.  This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * This driver adds support for UART devices via Freescale's QUICC Engine
 * found on some Freescale SOCs.
 *
 * If Soft-UART support is needed but not already present, then this driver
 * will request and upload the "Soft-UART" microcode upon probe.  The
 * filename of the microcode should be fsl_qe_ucode_uart_X_YZ.bin, where "X"
 * is the name of the SOC (e.g. 8323), and YZ is the revision of the SOC,
 * (e.g. "11" for 1.1).
 */

#include <linux/module.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>

#include <linux/fs_uart_pd.h>
#include <soc/fsl/qe/ucc_slow.h>

#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/serial_core.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
# define __ENABLE_KAPI__
/*
 * The GUMR flag for Soft UART.  This would normally be defined in qe.h,
 * but Soft-UART is a hack and we want to keep everything related to it in
 * this file.
 */
#define UCC_SLOW_GUMR_H_SUART   	0x00004000      /* Soft-UART */

#define UCC_UART_ASYNC_MODE    1
#define UCC_UART_SYNC_MODE     2

//#define __ASYNC_UART__
//#define __DEBUG_UCC_UART__

//#ifndef __ASYNC_UART__

#define SCFG_QEIOCLKCR_REG        0x1570400
#define UCC1_BASE_REG             0x2402000
#define UCC3_BASE_REG             0x2402200
#define ENABLE_BRG02_CLK_MASK     0xC0
#define ENABLE_BRG02_CLK          0x80
#define ENABLE_BRG01_CLK_MASK     0x03
#define ENABLE_BRG01_CLK          0x02

#ifdef __ENABLE_KAPI__
/* Channel definitions (dispatcher expects SPx logical channels) */
#ifndef ATC_LKM_SP5S
#define ATC_LKM_SP5S 5
#endif
#ifndef ATC_LKM_SP8S
#define ATC_LKM_SP8S 8
#endif

/* Dispatcher API (provided by spxs_kapi_dispatch) */
extern int spxs_kapi_register_channel(int ch, const struct spxs_kapi_ops *ops, void *priv);
extern int spxs_kapi_unregister_channel(int ch);

#ifndef ATC_SPXS_DEFAULT_BAUD
#define ATC_SPXS_DEFAULT_BAUD        153600
#endif

#ifndef ATC_SPXS_WRITE_CONFIG
#define ATC_SPXS_WRITE_CONFIG        (0x1001)
#endif
#ifndef ATC_SPXS_READ_CONFIG
#define ATC_SPXS_READ_CONFIG         (0x1002)
#endif
#ifndef FIONREAD
#define FIONREAD                     (0x541B)
#endif

struct uart_qe_port;

typedef struct atc_spxs_config {
    int baud_rate;
    int protocol;
    int clocking;
} atc_spxs_config_t;

struct sp58_kctx {
    struct uart_qe_port *qe_port;
};

struct sp58_kapi_rx_hdr {
    u16 len;
} __packed;

#define SP58_KAPI_RX_FIFO_BYTES      (64 * 1024)
#define SP58_KAPI_PKT_MAX            (2048)
#endif /* __ENABLE_KAPI__ */


static struct class *ucc_uart_class = NULL;

//#endif

/*
 * soft_uart is 1 if we need to use Soft-UART mode
 */
static int soft_uart;
/*
 * firmware_loaded is 1 if the firmware has been loaded, 0 otherwise.
 */
static int firmware_loaded;

/* Enable this macro to configure all serial ports in internal loopback
   mode */
/* #define LOOPBACK */

/* The major and minor device numbers are defined in
 * http://www.lanana.org/docs/device-list/devices-2.6+.txt.  For the QE
 * UART, we have major number 204 and minor numbers 46 - 49, which are the
 * same as for the CPM2.  This decision was made because no Freescale part
 * has both a CPM and a QE.
 */
#define SERIAL_QE_MAJOR 204
#define SERIAL_QE_MINOR 46

/* Since we only have minor numbers 46 - 49, there is a hard limit of 4 ports */
#define UCC_MAX_UART    9

/* The number of buffer descriptors for receiving characters. */
#define RX_NUM_FIFO     4

/* The number of buffer descriptors for transmitting characters. */
#define TX_NUM_FIFO     4

/* The maximum size of the character buffer for a single RX BD. */
#define RX_BUF_SIZE     32

/* The maximum size of the character buffer for a single TX BD. */
#define TX_BUF_SIZE     32

/*
 * The number of jiffies to wait after receiving a close command before the
 * device is actually closed.  This allows the last few characters to be
 * sent over the wire.
 */
#define UCC_WAIT_CLOSING 100

struct ucc_uart_pram {
	struct ucc_slow_pram common;
	u8 res1[8];     	/* reserved */
	__be16 maxidl;  	/* Maximum idle chars */
	__be16 idlc;    	/* temp idle counter */
	__be16 brkcr;   	/* Break count register */
	__be16 parec;   	/* receive parity error counter */
	__be16 frmec;   	/* receive framing error counter */
	__be16 nosec;   	/* receive noise counter */
	__be16 brkec;   	/* receive break condition counter */
	__be16 brkln;   	/* last received break length */
	__be16 uaddr[2];	/* UART address character 1 & 2 */
	__be16 rtemp;   	/* Temp storage */
	__be16 toseq;   	/* Transmit out of sequence char */
	__be16 cchars[8];       /* control characters 1-8 */
	__be16 rccm;    	/* receive control character mask */
	__be16 rccr;    	/* receive control character register */
	__be16 rlbc;    	/* receive last break character */
	__be16 res2;    	/* reserved */
	__be32 res3;    	/* reserved, should be cleared */
	u8 res4;		/* reserved, should be cleared */
	u8 res5[3];     	/* reserved, should be cleared */
	__be32 res6;    	/* reserved, should be cleared */
	__be32 res7;    	/* reserved, should be cleared */
	__be32 res8;    	/* reserved, should be cleared */
	__be32 res9;    	/* reserved, should be cleared */
	__be32 res10;   	/* reserved, should be cleared */
	__be32 res11;   	/* reserved, should be cleared */
	__be32 res12;   	/* reserved, should be cleared */
	__be32 res13;   	/* reserved, should be cleared */
/* The rest is for Soft-UART only */
	__be16 supsmr;  	/* 0x90, Shadow UPSMR */
	__be16 res92;   	/* 0x92, reserved, initialize to 0 */
	__be32 rx_state;	/* 0x94, RX state, initialize to 0 */
	__be32 rx_cnt;  	/* 0x98, RX count, initialize to 0 */
	u8 rx_length;   	/* 0x9C, Char length, set to 1+CL+PEN+1+SL */
	u8 rx_bitmark;  	/* 0x9D, reserved, initialize to 0 */
	u8 rx_temp_dlst_qe;     /* 0x9E, reserved, initialize to 0 */
	u8 res14[0xBC - 0x9F];  /* reserved */
	__be32 dump_ptr;	/* 0xBC, Dump pointer */
	__be32 rx_frame_rem;    /* 0xC0, reserved, initialize to 0 */
	u8 rx_frame_rem_size;   /* 0xC4, reserved, initialize to 0 */
	u8 tx_mode;     	/* 0xC5, mode, 0=AHDLC, 1=UART */
	__be16 tx_state;	/* 0xC6, TX state */
	u8 res15[0xD0 - 0xC8];  /* reserved */
	__be32 resD0;   	/* 0xD0, reserved, initialize to 0 */
	u8 resD4;       	/* 0xD4, reserved, initialize to 0 */
	__be16 resD5;   	/* 0xD5, reserved, initialize to 0 */
} __attribute__ ((packed));

/* SUPSMR definitions, for Soft-UART only */
#define UCC_UART_SUPSMR_SL      	0x8000
#define UCC_UART_SUPSMR_RPM_MASK	0x6000
#define UCC_UART_SUPSMR_RPM_ODD 	0x0000
#define UCC_UART_SUPSMR_RPM_LOW 	0x2000
#define UCC_UART_SUPSMR_RPM_EVEN	0x4000
#define UCC_UART_SUPSMR_RPM_HIGH	0x6000
#define UCC_UART_SUPSMR_PEN     	0x1000
#define UCC_UART_SUPSMR_SYNC     	0x8000
#define UCC_UART_SUPSMR_TPM_MASK	0x0C00
#define UCC_UART_SUPSMR_TPM_ODD 	0x0000
#define UCC_UART_SUPSMR_TPM_LOW 	0x0400
#define UCC_UART_SUPSMR_TPM_EVEN	0x0800
#define UCC_UART_SUPSMR_TPM_HIGH	0x0C00
#define UCC_UART_SUPSMR_FRZ     	0x0100
#define UCC_UART_SUPSMR_UM_MASK 	0x00c0
#define UCC_UART_SUPSMR_UM_NORMAL       0x0000
#define UCC_UART_SUPSMR_UM_MAN_MULTI    0x0040
#define UCC_UART_SUPSMR_UM_AUTO_MULTI   0x00c0
#define UCC_UART_SUPSMR_CL_MASK 	0x0030
#define UCC_UART_SUPSMR_CL_8    	0x0030
#define UCC_UART_SUPSMR_CL_7    	0x0020
#define UCC_UART_SUPSMR_CL_6    	0x0010
#define UCC_UART_SUPSMR_CL_5    	0x0000

#define UCC_UART_TX_STATE_AHDLC 	0x00
#define UCC_UART_TX_STATE_UART  	0x01
#define UCC_UART_TX_STATE_X1    	0x00
#define UCC_UART_TX_STATE_X16   	0x80

#define UCC_UART_PRAM_ALIGNMENT 0x100

#define UCC_UART_SIZE_OF_BD     UCC_SLOW_SIZE_OF_BD
#define NUM_CONTROL_CHARS       8

/* Private per-port data structure */
struct uart_qe_port {
	struct uart_port port;
	struct ucc_slow __iomem *uccp;
	struct ucc_uart_pram __iomem *uccup;
	struct ucc_slow_info us_info;
	struct ucc_slow_private *us_private;
	struct device_node *np;
	unsigned int ucc_num;   /* First ucc is 0, not 1 */

	u16 rx_nrfifos;
	u16 rx_fifosize;
	u16 tx_nrfifos;
	u16 tx_fifosize;
	int wait_closing;
	u32 flags;
	struct qe_bd *rx_bd_base;
	struct qe_bd *rx_cur;
	struct qe_bd *tx_bd_base;
	struct qe_bd *tx_cur;
	unsigned char *tx_buf;
	unsigned char *rx_buf;
	void *bd_virt;  	/* virtual address of the BD buffers */
	dma_addr_t bd_dma_addr; /* bus address of the BD buffers     */
	unsigned int bd_size;   /* size of BD buffer space           */
        unsigned int scfg_base;/* SCFG_QEIOCLKCR Reg Base Address    */
	int          ucc_mode;
	int          ucc_port;

	/* Output (RTS) state we drive */
	bool rts_asserted;

	/* Optional: CTS input if you have it (GPIO / status reg). If not, keep false. */
	bool cts_asserted;

	/* Soft-CD (derived carrier detect) */
	bool soft_cd_asserted;
	unsigned long last_good_rx_j;

	/* Soft-CD timeout support */
	unsigned int cd_timeout_ms;      /* e.g. 1000 */
	struct delayed_work cd_drop_work;

#ifdef __ENABLE_KAPI__
	/* ---- KAPI support ---- */
	struct kfifo      kapi_rx_fifo;
	spinlock_t        kapi_rx_lock;
	struct mutex      kapi_lock;
	bool              kapi_is_open;
	atc_spxs_config_t kapi_cfg;
#endif /* __ENABLE_KAPI__ */
};

static unsigned int qe_sdlc_get_mctrl(struct uart_port *port);
static void qe_sdlc_softcd_update(struct uart_qe_port *qsp, bool cd_present);

struct qe_sdlc_port {
	struct uart_port port;

	/* Output (RTS) state we drive */
	bool rts_asserted;

	/* Optional: CTS input if you have it (GPIO / status reg). If not, keep false. */
	bool cts_asserted;

	/* Soft-CD (derived carrier detect) */
	bool soft_cd_asserted;
	unsigned long last_good_rx_j;

	/* Soft-CD timeout support */
	unsigned int cd_timeout_ms;      /* e.g. 1000 */
	struct delayed_work cd_drop_work;
};


static struct uart_driver ucc_uart_driver = {
	.owner  	= THIS_MODULE,
	.driver_name    = "ucc_uart",
	.dev_name       = "ttyQE",
	.major  	= SERIAL_QE_MAJOR,
	.minor  	= SERIAL_QE_MINOR,
	.nr     	= UCC_MAX_UART,
};

/*
 * UCC UART Mode Selection
 */
static int ucc_mode_param = 0;


#ifdef __DEBUG_UCC_UART__ 
void ucc_dump_reg(struct uart_qe_port *qe_port)
{
    struct ucc_slow __iomem *uccp = qe_port->uccp;

    printk("**********************************************\r\n");
    printk("UCCx general mode register (low)      GUMR_L  = %x \r\n", ioread32be(&uccp->gumr_l));
    printk("UCCx general mode register (high)     GUMR_H  = %x \r\n", ioread32be(&uccp->gumr_h));
    printk("UCCx protocol-specific mode register  UPSMR   = %x \r\n", ioread16be(&uccp->upsmr));
    printk("UCCx transmit on demand register      UTODR   = %x \r\n", ioread16be(&uccp->utodr));
    printk("UCCx data synchronization register    UDSR    = %x \r\n", ioread16be(&uccp->udsr));
    printk("UCCx event register                   UCCE    = %x \r\n", ioread16be(&uccp->ucce));
    printk("UCCx mask register                    UCCM    = %x \r\n", ioread16be(&uccp->uccm));
    printk("UCCx status register                  UCCS    = %x \r\n", ioread8(&uccp->uccs));
    printk("UCCx UTPT                             UTPT    = %x \r\n", ioread16be(&uccp->utpt));
    printk("UCC general extended mode register    GUEMR   = %x \r\n", ioread8(&uccp->guemr));
    printk("**********************************************\r\n");
}
#endif

static void qe_sdlc_cd_drop_workfn(struct work_struct *work)
{
	struct uart_qe_port *qsp =
		container_of(to_delayed_work(work), struct uart_qe_port, cd_drop_work);

	if (!qsp->soft_cd_asserted)
		return;

	if (time_after(jiffies,
		       qsp->last_good_rx_j + msecs_to_jiffies(qsp->cd_timeout_ms))) {
		qe_sdlc_softcd_update(qsp, false);
		return;
	}

	/* Still within window: reschedule for remaining time */
	{
		unsigned long expire = qsp->last_good_rx_j + msecs_to_jiffies(qsp->cd_timeout_ms);
		unsigned long delay = expire - jiffies;
		if ((long)delay < 1)
			delay = 1;
		schedule_delayed_work(&qsp->cd_drop_work, delay);
	}
}

/*
 * Virtual to physical address translation.
 *
 * Given the virtual address for a character buffer, this function returns
 * the physical (DMA) equivalent.
 */
static inline dma_addr_t cpu2qe_addr(void *addr, struct uart_qe_port *qe_port)
{
	if (likely((addr >= qe_port->bd_virt)) &&
	    (addr < (qe_port->bd_virt + qe_port->bd_size)))
		return qe_port->bd_dma_addr + (addr - qe_port->bd_virt);

	/* something nasty happened */
	printk(KERN_ERR "%s: addr=%p\n", __func__, addr);
	BUG();
	return 0;
}

/*
 * Physical to virtual address translation.
 *
 * Given the physical (DMA) address for a character buffer, this function
 * returns the virtual equivalent.
 */
static inline void *qe2cpu_addr(dma_addr_t addr, struct uart_qe_port *qe_port)
{
	/* sanity check */
	if (likely((addr >= qe_port->bd_dma_addr) &&
		   (addr < (qe_port->bd_dma_addr + qe_port->bd_size))))
		return qe_port->bd_virt + (addr - qe_port->bd_dma_addr);

	/* something nasty happened */
	printk(KERN_ERR "%s: addr=%llx\n", __func__, (u64)addr);
	BUG();
	return NULL;
}

/*
 * Return 1 if the QE is done transmitting all buffers for this port
 *
 * This function scans each BD in sequence.  If we find a BD that is not
 * ready (READY=1), then we return 0 indicating that the QE is still sending
 * data.  If we reach the last BD (WRAP=1), then we know we've scanned
 * the entire list, and all BDs are done.
 */
static unsigned int qe_uart_tx_empty(struct uart_port *port)
{
	struct uart_qe_port *qe_port =
		container_of(port, struct uart_qe_port, port);
	struct qe_bd *bdp = qe_port->tx_bd_base;

	while (1) {
		if (ioread16be(&bdp->status) & BD_SC_READY)
			/* This BD is not done, so return "not done" */
			return 0;

		if (ioread16be(&bdp->status) & BD_SC_WRAP)
			/*
			 * This BD is done and it's the last one, so return
			 * "done"
			 */
			return 1;

		bdp++;
	}
}

/*
 * Set the modem control lines
 *
 * Although the QE can control the modem control lines (e.g. CTS), we
 * don't need that support. This function must exist, however, otherwise
 * the kernel will panic.
 */
static void qe_sdlc_set_mctrl(struct uart_port *port, unsigned int mctrl)
{

	struct uart_qe_port *qe_port =
		container_of(port, struct uart_qe_port, port);
	struct ucc_slow __iomem *uccp = qe_port->uccp;
 
    u32 gumr_h_mask = 0;
    u32 gumr_h_set  = 0;

    /*
     * We interpret TIOCM_RTS as "enable/disable RTS/CTS HW flow control".
     * (Because you asked to reflect enable state using gumr_h bits)
     */
    bool enable_hw_fc = !!(mctrl & TIOCM_RTS);

    /* Polarity: CTSB is active-low -> set CTSP if available */
#ifdef UCC_SLOW_GUMR_H_CTSP
    gumr_h_mask |= UCC_SLOW_GUMR_H_CTSP;
    gumr_h_set  |= UCC_SLOW_GUMR_H_CTSP;
#endif

    /* Enable CTS sampling / gating only when enabled */
#ifdef UCC_SLOW_GUMR_H_CTSS
    gumr_h_mask |= UCC_SLOW_GUMR_H_CTSS;
    if (enable_hw_fc)
        gumr_h_set |= UCC_SLOW_GUMR_H_CTSS;
#endif

    /* Enable RTS management only when enabled */
#ifdef UCC_SLOW_GUMR_H_RTSM
    gumr_h_mask |= UCC_SLOW_GUMR_H_RTSM;
    if (enable_hw_fc)
	{
		// printk("Enable Hardware Flowcontrol \r\n");
        gumr_h_set |= UCC_SLOW_GUMR_H_RTSM;
	}
	else
		// printk("Disable Hardware Flowcontrol \r\n");
#endif

    /*
     * These two are “optional”. In your snippet you always set them.
     * Keep them enabled if you want CD sampling features (if supported in your mode).
     * NOTE: This is not the same as TIOCM_CD live level; that you derive separately.
     */
#ifdef UCC_SLOW_GUMR_H_CDS
    gumr_h_mask |= UCC_SLOW_GUMR_H_CDS;
    gumr_h_set  |= UCC_SLOW_GUMR_H_CDS;
#endif

#ifdef UCC_SLOW_GUMR_H_CDP
    gumr_h_mask |= UCC_SLOW_GUMR_H_CDP;
    gumr_h_set  |= UCC_SLOW_GUMR_H_CDP;
#endif

    if (gumr_h_mask)
	{
		printk("GUMR_H Register = %x \r\n", ioread32be(&uccp->gumr_l));
        qe_clrsetbits32(&uccp->gumr_h, gumr_h_mask, gumr_h_set);
    }

    /*
     * Keep a cached software state too if you want:
     * qsp->rts_asserted = enable_hw_fc;
     */
}


/*
 * Get the current modem control line status
 *
 * Although the QE can control the modem control lines (e.g. CTS), this
 * driver currently doesn't support that, so we always return Carrier
 * Detect, Data Set Ready, and Clear To Send.
 */
static unsigned int qe_sdlc_get_mctrl(struct uart_port *port)
{
	struct uart_qe_port *qe_port =
		container_of(port, struct uart_qe_port, port);
	struct ucc_slow __iomem *uccp = qe_port->uccp;
 
	u32 gumr_h = ioread32be(&uccp->gumr_h);
    unsigned int mctrl = 0;

    /*
     * Interpret “RTS/CTS enabled” from gumr_h bits:
     * - CTSS indicates CTS sampling/handshake enabled
     * - RTSM indicates RTS management enabled
     */
#ifdef UCC_SLOW_GUMR_H_CTSS
    if (gumr_h & UCC_SLOW_GUMR_H_CTSS)
        mctrl |= TIOCM_CTS;
#endif

#ifdef UCC_SLOW_GUMR_H_RTSM
    if (gumr_h & UCC_SLOW_GUMR_H_RTSM)
        mctrl |= TIOCM_RTS;
#endif

    // printk("\r\n%s : %d gumr_h = 0x%x\r\n", __func__, __LINE__, gumr_h);

    /*
     * TIOCM_CD should NOT be derived from gumr_h.
     * For your requirement, CD is derived from BD_SC_CD_LOST + timeout (soft-CD).
     * If you keep qsp->soft_cd_asserted, then:
     *
     * if (qsp->soft_cd_asserted)
     *     mctrl |= TIOCM_CD;
     */

    return mctrl;
}

/*
 * Disable the transmit interrupt.
 *
 * Although this function is called "stop_tx", it does not actually stop
 * transmission of data.  Instead, it tells the QE to not generate an
 * interrupt when the UCC is finished sending characters.
 */
static void qe_uart_stop_tx(struct uart_port *port)
{
	struct uart_qe_port *qe_port =
		container_of(port, struct uart_qe_port, port);

	qe_clrbits16(&qe_port->uccp->uccm, UCC_UART_UCCE_TX);
}

/*
 * Transmit as many characters to the HW as possible.
 *
 * This function will attempt to stuff of all the characters from the
 * kernel's transmit buffer into TX BDs.
 *
 * A return value of non-zero indicates that it successfully stuffed all
 * characters from the kernel buffer.
 *
 * A return value of zero indicates that there are still characters in the
 * kernel's buffer that have not been transmitted, but there are no more BDs
 * available.  This function should be called again after a BD has been made
 * available.
 */
static int qe_uart_tx_pump(struct uart_qe_port *qe_port)
{
	struct qe_bd *bdp;
	unsigned char *p;
	unsigned int count;
	struct uart_port *port = &qe_port->port;
	struct circ_buf *xmit = &port->state->xmit;

	bdp = qe_port->rx_cur;

	/* Handle xon/xoff */
	if (port->x_char) {
		/* Pick next descriptor and fill from buffer */
		bdp = qe_port->tx_cur;

		p = qe2cpu_addr(be32_to_cpu(bdp->buf), qe_port);

		*p++ = port->x_char;
		iowrite16be(1, &bdp->length);
		qe_setbits16(&bdp->status, BD_SC_READY);
		/* Get next BD. */
		if (ioread16be(&bdp->status) & BD_SC_WRAP)
			bdp = qe_port->tx_bd_base;
		else
			bdp++;
		qe_port->tx_cur = bdp;

		port->icount.tx++;
		port->x_char = 0;
		return 1;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		qe_uart_stop_tx(port);
		return 0;
	}

	/* Pick next descriptor and fill from buffer */
	bdp = qe_port->tx_cur;

	while (!(ioread16be(&bdp->status) & BD_SC_READY) &&
	       (xmit->tail != xmit->head)) {
		count = 0;
		p = qe2cpu_addr(be32_to_cpu(bdp->buf), qe_port);
		while (count < qe_port->tx_fifosize) {
			*p++ = xmit->buf[xmit->tail];
#ifdef __DEBUG_UCC_UART__ 
                        printk("TxD = %c \r\n", xmit->buf[xmit->tail]);
#endif
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			port->icount.tx++;
			count++;
			if (xmit->head == xmit->tail)
				break;
		}

		iowrite16be(count, &bdp->length);
		qe_setbits16(&bdp->status, BD_SC_READY);

		/* Get next BD. */
		if (ioread16be(&bdp->status) & BD_SC_WRAP)
			bdp = qe_port->tx_bd_base;
		else
			bdp++;
	}
	qe_port->tx_cur = bdp;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit)) {
		/* The kernel buffer is empty, so turn off TX interrupts.  We
		   don't need to be told when the QE is finished transmitting
		   the data. */
		qe_uart_stop_tx(port);
		return 0;
	}
	return 1;
}

/*
 * Start transmitting data
 *
 * This function will start transmitting any available data, if the port
 * isn't already transmitting data.
 */
static void qe_uart_start_tx(struct uart_port *port)
{
#ifdef __DEBUG_UCC_UART__ 
        struct qe_bd *bdp;
        int i;
#endif
	struct uart_qe_port *qe_port =
		container_of(port, struct uart_qe_port, port);

#ifdef __DEBUG_UCC_UART__ 
        printk("Entering into the qe_uart_start_tx() %x \r\n", ioread16be(&qe_port->uccp->uccm));
        ucc_dump_reg(qe_port);
       /* Print Buffer Status from BD */

        bdp = qe_port->tx_bd_base;
         for (i = 0; i < (qe_port->tx_nrfifos ); i++) {
                printk("%d. Buffer Descriptor Status = %x \r\n", i, ioread16be(&bdp->status));
                printk("%d. Buffer Descriptor len    = %d \r\n", i, ioread16be(&bdp->length));
                bdp++;
         }
#endif

	/* If we currently are transmitting, then just return */
	if (ioread16be(&qe_port->uccp->uccm) & UCC_UART_UCCE_TX)
        {
#ifdef __DEBUG_UCC_UART__ 
            printk("Consumed all the BD, THere is no BD So return from here. \r\n");
#endif
	    return;
        }

	/* Otherwise, pump the port and start transmission */
	if (qe_uart_tx_pump(qe_port))
        {
#ifdef __DEBUG_UCC_UART__ 
            printk("Pump the port and start transmission\r\n");
#endif
	    qe_setbits16(&qe_port->uccp->uccm, UCC_UART_UCCE_TX);
        }
}

/*
 * Stop transmitting data
 */
static void qe_uart_stop_rx(struct uart_port *port)
{
	struct uart_qe_port *qe_port =
		container_of(port, struct uart_qe_port, port);

	qe_clrbits16(&qe_port->uccp->uccm, UCC_UART_UCCE_RX);
}

/* Start or stop sending  break signal
 *
 * This function controls the sending of a break signal.  If break_state=1,
 * then we start sending a break signal.  If break_state=0, then we stop
 * sending the break signal.
 */
static void qe_uart_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_qe_port *qe_port =
		container_of(port, struct uart_qe_port, port);

	if (break_state)
		ucc_slow_stop_tx(qe_port->us_private);
	else
		ucc_slow_restart_tx(qe_port->us_private);
}

#ifdef __ENABLE_KAPI__
/* --------------------------------------------------------------------------
 * KAPI RX packet FIFO helper (push one received packet for kernel clients)
 * -------------------------------------------------------------------------- */
static void sp58_kapi_rx_push(struct uart_qe_port *qe_port, const u8 *payload, u16 payload_len)
{
    unsigned long flags;
    struct sp58_kapi_rx_hdr hdr;

    if (!qe_port || !payload || payload_len == 0 || payload_len > SP58_KAPI_PKT_MAX)
        return;

    hdr.len = payload_len;

    spin_lock_irqsave(&qe_port->kapi_rx_lock, flags);

    if (kfifo_avail(&qe_port->kapi_rx_fifo) < (sizeof(hdr) + payload_len)) {
        spin_unlock_irqrestore(&qe_port->kapi_rx_lock, flags);
        return;
    }

    kfifo_in(&qe_port->kapi_rx_fifo, &hdr, sizeof(hdr));
    kfifo_in(&qe_port->kapi_rx_fifo, payload, payload_len);

    spin_unlock_irqrestore(&qe_port->kapi_rx_lock, flags);

    /* Wake any waiters (re-using tty waitqueue already present in uart core). */
    if (qe_port->port.state)
        wake_up_interruptible(&qe_port->port.state->port.delta_msr_wait);
}
#endif /* __ENABLE_KAPI__ */


static void qe_sdlc_softcd_update(struct uart_qe_port *qsp, bool cd_present)
{
	if (qsp->soft_cd_asserted == cd_present)
		return;

    struct ucc_slow __iomem *uccp = qsp->uccp;
    u32 gumr_h = ioread32be(&uccp->gumr_h);

    // printk("\r\n%s : %d gumr_h = 0x%x\r\n", __func__, __LINE__, gumr_h);

	qsp->soft_cd_asserted = cd_present;

	/*
	 * This updates port->icount.dcd and triggers:
	 *  - wakeups for TIOCMIWAIT
	 *  - SIGIO async notification for O_ASYNC users
	 */
	uart_handle_dcd_change(&qsp->port, cd_present);
}

/* ISR helper function for receiving character.
 *
 * This function is called by the ISR to handling receiving characters
 */
static void qe_uart_int_rx(struct uart_qe_port *qe_port)
{
	int i;
	unsigned char ch, *cp;
	struct uart_port *port = &qe_port->port;
	struct tty_port *tport = &port->state->port;
	struct qe_bd *bdp;
	u16 status;
	unsigned int flg;
	u8 invalid_fram = 0;

	/* Just loop through the closed BDs and copy the characters into
	 * the buffer.
	 */
	bdp = qe_port->rx_cur;
#ifdef __DEBUG_UCC_UART__ 
        printk("FV : QUICC FUNC = %s ,Line =%d \r\n",__func__,__LINE__);
#endif
	while (1) {
		status = ioread16be(&bdp->status);

		/* If this one is empty, then we assume we've read them all */
		if (status & BD_SC_EMPTY)
			break;

		/* get number of characters, and check space in RX buffer */
		i = ioread16be(&bdp->length);

		/* If we don't have enough room in RX buffer for the entire BD,
		 * then we try later, which will be the next RX interrupt.
		 */
		if (tty_buffer_request_room(tport, i) < i) {
			dev_dbg(port->dev, "ucc-uart: no room in RX buffer\n");
			return;
		}

		/* get pointer */
		cp = qe2cpu_addr(be32_to_cpu(bdp->buf), qe_port);

		/* loop through the buffer */
		while (i-- > 0) {
			ch = *cp++;
#ifdef __DEBUG_UCC_UART__ 
                        printk("RxD = %c\r\n", ch);
#endif
			port->icount.rx++;
			flg = TTY_NORMAL;

			if (!i && status &
			    (BD_SC_BR | BD_SC_FR | BD_SC_PR | BD_SC_OV | BD_SC_CD))
				goto handle_error;
			if (uart_handle_sysrq_char(port, ch))
				continue;

error_return:
			tty_insert_flip_char(tport, ch, flg);

		}

		/* This BD is ready to be used again. Clear status. get next */
		qe_clrsetbits16(&bdp->status, BD_SC_BR | BD_SC_FR | BD_SC_PR |
			BD_SC_OV | BD_SC_ID | BD_SC_CD, BD_SC_EMPTY);
		if (ioread16be(&bdp->status) & BD_SC_WRAP)
			bdp = qe_port->rx_bd_base;
		else
			bdp++;
		
	}

	qe_sdlc_softcd_update(qe_port, true);

	/* Write back buffer pointer */
	qe_port->rx_cur = bdp;

	/* Activate BH processing */
	tty_flip_buffer_push(tport);

	return;

	/* Error processing */

handle_error:

	if (status & BD_SC_CD)
	{
		/* Carrier lost during reception => force CD low + notify */
		qe_sdlc_softcd_update(qe_port, false);
	}
	/* Statistics */
	if (status & BD_SC_BR)
		port->icount.brk++;
	if (status & BD_SC_PR)
		port->icount.parity++;
	if (status & BD_SC_FR)
		port->icount.frame++;
	if (status & BD_SC_OV)
		port->icount.overrun++;

	/* Mask out ignored conditions */
	status &= port->read_status_mask;

	/* Handle the remaining ones */
	if (status & BD_SC_BR)
		flg = TTY_BREAK;
	else if (status & BD_SC_PR)
		flg = TTY_PARITY;
	else if (status & BD_SC_FR)
		flg = TTY_FRAME;

	/* Overrun does not affect the current character ! */
	if (status & BD_SC_OV)
		tty_insert_flip_char(tport, 0, TTY_OVERRUN);
#ifdef SUPPORT_SYSRQ
	port->sysrq = 0;
#endif
	goto error_return;
}

/* Interrupt handler
 *
 * This interrupt handler is called after a BD is processed.
 */
static irqreturn_t qe_uart_int(int irq, void *data)
{
	struct uart_qe_port *qe_port = (struct uart_qe_port *) data;
	struct ucc_slow __iomem *uccp = qe_port->uccp;
	u16 events;

	/* Clear the interrupts */
	events = ioread16be(&uccp->ucce);
	iowrite16be(events, &uccp->ucce);

	if (events & UCC_UART_UCCE_BRKE)
		uart_handle_break(&qe_port->port);

	if (events & UCC_UART_UCCE_RX)
		qe_uart_int_rx(qe_port);

	if (events & UCC_UART_UCCE_TX)
		qe_uart_tx_pump(qe_port);

	return events ? IRQ_HANDLED : IRQ_NONE;
}

/* Initialize buffer descriptors
 *
 * This function initializes all of the RX and TX buffer descriptors.
 */
static void qe_uart_initbd(struct uart_qe_port *qe_port)
{
	int i;
	void *bd_virt;
	struct qe_bd *bdp;

	/* Set the physical address of the host memory buffers in the buffer
	 * descriptors, and the virtual address for us to work with.
	 */
	bd_virt = qe_port->bd_virt;
	bdp = qe_port->rx_bd_base;
	qe_port->rx_cur = qe_port->rx_bd_base;
	for (i = 0; i < (qe_port->rx_nrfifos - 1); i++) {
		iowrite16be(BD_SC_EMPTY | BD_SC_INTRPT, &bdp->status);
		iowrite32be(cpu2qe_addr(bd_virt, qe_port), &bdp->buf);
		iowrite16be(0, &bdp->length);
		bd_virt += qe_port->rx_fifosize;
		bdp++;
	}

	iowrite16be(BD_SC_WRAP | BD_SC_EMPTY | BD_SC_INTRPT, &bdp->status);
	iowrite32be(cpu2qe_addr(bd_virt, qe_port), &bdp->buf);
	iowrite16be(0, &bdp->length);

	/* Set the physical address of the host memory
	 * buffers in the buffer descriptors, and the
	 * virtual address for us to work with.
	 */
	bd_virt = qe_port->bd_virt +
		L1_CACHE_ALIGN(qe_port->rx_nrfifos * qe_port->rx_fifosize);
	qe_port->tx_cur = qe_port->tx_bd_base;
	bdp = qe_port->tx_bd_base;
	for (i = 0; i < (qe_port->tx_nrfifos - 1); i++) {
		iowrite16be(BD_SC_INTRPT, &bdp->status);
		iowrite32be(cpu2qe_addr(bd_virt, qe_port), &bdp->buf);
		iowrite16be(0, &bdp->length);
		bd_virt += qe_port->tx_fifosize;
		bdp++;
	}

	/* Loopback requires the preamble bit to be set on the first TX BD */
#ifdef LOOPBACK
	qe_setbits16(&qe_port->tx_cur->status, BD_SC_P);
#endif

	iowrite16be(BD_SC_WRAP | BD_SC_INTRPT, &bdp->status);
	iowrite32be(cpu2qe_addr(bd_virt, qe_port), &bdp->buf);
	iowrite16be(0, &bdp->length);
}

/*
 * Initialize a UCC for UART.
 *
 * This function configures a given UCC to be used as a UART device. Basic
 * UCC initialization is handled in qe_uart_request_port().  This function
 * does all the UART-specific stuff.
 */
static void qe_uart_init_ucc(struct uart_qe_port *qe_port)
{
	u32 cecr_subblock;
	struct ucc_slow __iomem *uccp = qe_port->uccp;
	struct ucc_uart_pram *uccup = qe_port->uccup;

	unsigned int i;

	/* First, disable TX and RX in the UCC */
	ucc_slow_disable(qe_port->us_private, COMM_DIR_RX_AND_TX);

	/* Program the UCC UART parameter RAM */
	iowrite8(UCC_BMR_GBL | UCC_BMR_BO_BE, &uccup->common.rbmr);
	iowrite8(UCC_BMR_GBL | UCC_BMR_BO_BE, &uccup->common.tbmr);
	iowrite16be(qe_port->rx_fifosize, &uccup->common.mrblr);
	iowrite16be(0x10, &uccup->maxidl);
	iowrite16be(1, &uccup->brkcr);
	iowrite16be(0, &uccup->parec);
	iowrite16be(0, &uccup->frmec);
	iowrite16be(0, &uccup->nosec);
	iowrite16be(0, &uccup->brkec);
	iowrite16be(0, &uccup->uaddr[0]);
	iowrite16be(0, &uccup->uaddr[1]);
	iowrite16be(0, &uccup->toseq);

	for (i = 0; i < 8; i++)
		iowrite16be(0xC000, &uccup->cchars[i]);
	iowrite16be(0xc0ff, &uccup->rccm);

	/* Configure the GUMR registers for UART */
	if (soft_uart) {
		/* Soft-UART requires a 1X multiplier for TX */
		qe_clrsetbits32(&uccp->gumr_l,
			UCC_SLOW_GUMR_L_MODE_MASK | UCC_SLOW_GUMR_L_TDCR_MASK |
			UCC_SLOW_GUMR_L_RDCR_MASK,
			UCC_SLOW_GUMR_L_MODE_UART | UCC_SLOW_GUMR_L_TDCR_1 |
			UCC_SLOW_GUMR_L_RDCR_16);

		qe_clrsetbits32(&uccp->gumr_h, UCC_SLOW_GUMR_H_RFW,
			UCC_SLOW_GUMR_H_TRX | UCC_SLOW_GUMR_H_TTX);
	} else {
//#ifdef __ASYNC_UART__
            if (UCC_UART_ASYNC_MODE == qe_port->ucc_mode)
            {
		qe_clrsetbits32(&uccp->gumr_l,
			UCC_SLOW_GUMR_L_MODE_MASK | UCC_SLOW_GUMR_L_TDCR_MASK |
			UCC_SLOW_GUMR_L_RDCR_MASK,
			UCC_SLOW_GUMR_L_MODE_UART | UCC_SLOW_GUMR_L_TDCR_16 |
			UCC_SLOW_GUMR_L_RDCR_16);

		qe_clrsetbits32(&uccp->gumr_h,
			UCC_SLOW_GUMR_H_TRX | UCC_SLOW_GUMR_H_TTX,
			UCC_SLOW_GUMR_H_RFW);
            }
            else // UCC_UART_SYNC_MODE
            {
//#else
		printk("SyncUCCUART : Synchronous UCC UART controller using 1× clock \r\n");
		printk("(isochronous UART operation). GUMR_L[TENC,RENC] must select NRZ \r\n");
		printk("and GUMR_L[RDCR, TDCR] select 1x mode. A bit is transferred \r\n");
		printk("with each clock and is synchronous to the clock, which can be \r\n");
		printk("internal or external.Synchronous MODE \r\n");
                
                /* Firstview */
		qe_clrsetbits32(&uccp->gumr_l,
			UCC_SLOW_GUMR_L_MODE_MASK | UCC_SLOW_GUMR_L_TDCR_MASK |
			UCC_SLOW_GUMR_L_RDCR_MASK,
			UCC_SLOW_GUMR_L_MODE_UART | UCC_SLOW_GUMR_L_TDCR_1 |
			UCC_SLOW_GUMR_L_RDCR_1);

		qe_clrsetbits32(&uccp->gumr_h,
			UCC_SLOW_GUMR_H_TRX | UCC_SLOW_GUMR_H_TTX,
			UCC_SLOW_GUMR_H_RFW);
	     }
//#endif
	}

#ifdef LOOPBACK
	qe_clrsetbits32(&uccp->gumr_l, UCC_SLOW_GUMR_L_DIAG_MASK,
		UCC_SLOW_GUMR_L_DIAG_LOOP);
	qe_clrsetbits32(&uccp->gumr_h,
		UCC_SLOW_GUMR_H_CTSP | UCC_SLOW_GUMR_H_RSYN,
		UCC_SLOW_GUMR_H_CDS);
#endif

	/* Disable rx interrupts  and clear all pending events.  */
	iowrite16be(0, &uccp->uccm);
	iowrite16be(0xffff, &uccp->ucce);
	iowrite16be(0x7e7e, &uccp->udsr);

	/* Initialize UPSMR */
	iowrite16be(0, &uccp->upsmr);

	if (soft_uart) {
		iowrite16be(0x30, &uccup->supsmr);
		iowrite16be(0, &uccup->res92);
		iowrite32be(0, &uccup->rx_state);
		iowrite32be(0, &uccup->rx_cnt);
		iowrite8(0, &uccup->rx_bitmark);
		iowrite8(10, &uccup->rx_length);
		iowrite32be(0x4000, &uccup->dump_ptr);
		iowrite8(0, &uccup->rx_temp_dlst_qe);
		iowrite32be(0, &uccup->rx_frame_rem);
		iowrite8(0, &uccup->rx_frame_rem_size);
		/* Soft-UART requires TX to be 1X */
		iowrite8(UCC_UART_TX_STATE_UART | UCC_UART_TX_STATE_X1,
			 &uccup->tx_mode);
		iowrite16be(0, &uccup->tx_state);
		iowrite8(0, &uccup->resD4);
		iowrite16be(0, &uccup->resD5);

		/* Set UART mode.
		 * Enable receive and transmit.
		 */

		/* From the microcode errata:
		 * 1.GUMR_L register, set mode=0010 (QMC).
		 * 2.Set GUMR_H[17] bit. (UART/AHDLC mode).
		 * 3.Set GUMR_H[19:20] (Transparent mode)
		 * 4.Clear GUMR_H[26] (RFW)
		 * ...
		 * 6.Receiver must use 16x over sampling
		 */
		qe_clrsetbits32(&uccp->gumr_l,
			UCC_SLOW_GUMR_L_MODE_MASK | UCC_SLOW_GUMR_L_TDCR_MASK |
			UCC_SLOW_GUMR_L_RDCR_MASK,
			UCC_SLOW_GUMR_L_MODE_QMC | UCC_SLOW_GUMR_L_TDCR_16 |
			UCC_SLOW_GUMR_L_RDCR_16);

		qe_clrsetbits32(&uccp->gumr_h,
			UCC_SLOW_GUMR_H_RFW | UCC_SLOW_GUMR_H_RSYN,
			UCC_SLOW_GUMR_H_SUART | UCC_SLOW_GUMR_H_TRX |
			UCC_SLOW_GUMR_H_TTX | UCC_SLOW_GUMR_H_TFL);

#ifdef LOOPBACK
		qe_clrsetbits32(&uccp->gumr_l, UCC_SLOW_GUMR_L_DIAG_MASK,
				UCC_SLOW_GUMR_L_DIAG_LOOP);
		qe_clrbits32(&uccp->gumr_h, UCC_SLOW_GUMR_H_CTSP |
			  UCC_SLOW_GUMR_H_CDS);
#endif

		cecr_subblock = ucc_slow_get_qe_cr_subblock(qe_port->ucc_num);
		qe_issue_cmd(QE_INIT_TX_RX, cecr_subblock,
			QE_CR_PROTOCOL_UNSPECIFIED, 0);
	} else {
		cecr_subblock = ucc_slow_get_qe_cr_subblock(qe_port->ucc_num);
		qe_issue_cmd(QE_INIT_TX_RX, cecr_subblock,
			QE_CR_PROTOCOL_UART, 0);
	}
}

/*
 * Initialize the port.
 */
static int qe_uart_startup(struct uart_port *port)
{
	struct uart_qe_port *qe_port =
		container_of(port, struct uart_qe_port, port);
	int ret;

        
	/*
	 * If we're using Soft-UART mode, then we need to make sure the
	 * firmware has been uploaded first.
	 */
	if (soft_uart && !firmware_loaded) {
		dev_err(port->dev, "Soft-UART firmware not uploaded\n");
		return -ENODEV;
	}

	qe_uart_initbd(qe_port);
	qe_uart_init_ucc(qe_port);

	/* Install interrupt handler. */
	ret = request_irq(port->irq, qe_uart_int, IRQF_SHARED, "ucc-uart",
		qe_port);
	if (ret) {
		dev_err(port->dev, "could not claim IRQ %u\n", port->irq);
		return ret;
	}

	/* Startup rx-int */
	qe_setbits16(&qe_port->uccp->uccm, UCC_UART_UCCE_RX);
	ucc_slow_enable(qe_port->us_private, COMM_DIR_RX_AND_TX);

	return 0;
}

/*
 * Shutdown the port.
 */
static void qe_uart_shutdown(struct uart_port *port)
{
	struct uart_qe_port *qe_port =
		container_of(port, struct uart_qe_port, port);
	struct ucc_slow __iomem *uccp = qe_port->uccp;
	unsigned int timeout = 20;

	/* Disable RX and TX */

	/* Wait for all the BDs marked sent */
	while (!qe_uart_tx_empty(port)) {
		if (!--timeout) {
			dev_warn(port->dev, "shutdown timeout\n");
			break;
		}
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(2);
	}

	if (qe_port->wait_closing) {
		/* Wait a bit longer */
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(qe_port->wait_closing);
	}

	/* Stop uarts */
	ucc_slow_disable(qe_port->us_private, COMM_DIR_RX_AND_TX);
	qe_clrbits16(&uccp->uccm, UCC_UART_UCCE_TX | UCC_UART_UCCE_RX);

	/* Shut them really down and reinit buffer descriptors */
	ucc_slow_graceful_stop_tx(qe_port->us_private);
	qe_uart_initbd(qe_port);

	free_irq(port->irq, qe_port);
}

/*
 * Set the serial port parameters.
 */
static void qe_uart_set_termios(struct uart_port *port,
				struct ktermios *termios, struct ktermios *old)
{
	struct uart_qe_port *qe_port =
		container_of(port, struct uart_qe_port, port);
	struct ucc_slow __iomem *uccp = qe_port->uccp;
	unsigned int baud;
	unsigned long flags;
	u16 upsmr = ioread16be(&uccp->upsmr);
	struct ucc_uart_pram __iomem *uccup = qe_port->uccup;
	u16 supsmr = ioread16be(&uccup->supsmr);
	u8 char_length = 2; /* 1 + CL + PEN + 1 + SL */

	/* Character length programmed into the mode register is the
	 * sum of: 1 start bit, number of data bits, 0 or 1 parity bit,
	 * 1 or 2 stop bits, minus 1.
	 * The value 'bits' counts this for us.
	 */

	/* byte size */
	upsmr &= UCC_UART_UPSMR_CL_MASK;
	supsmr &= UCC_UART_SUPSMR_CL_MASK;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		upsmr |= UCC_UART_UPSMR_CL_5;
		supsmr |= UCC_UART_SUPSMR_CL_5;
		char_length += 5;
		break;
	case CS6:
		upsmr |= UCC_UART_UPSMR_CL_6;
		supsmr |= UCC_UART_SUPSMR_CL_6;
		char_length += 6;
		break;
	case CS7:
		upsmr |= UCC_UART_UPSMR_CL_7;
		supsmr |= UCC_UART_SUPSMR_CL_7;
		char_length += 7;
		break;
	default:	/* case CS8 */
		upsmr |= UCC_UART_UPSMR_CL_8;
		supsmr |= UCC_UART_SUPSMR_CL_8;
		char_length += 8;
		break;
	}

	/* If CSTOPB is set, we want two stop bits */
	if (termios->c_cflag & CSTOPB) {
		upsmr |= UCC_UART_UPSMR_SL;
		supsmr |= UCC_UART_SUPSMR_SL;
		char_length++;  /* + SL */
	}

	if (termios->c_cflag & PARENB) {
		upsmr |= UCC_UART_UPSMR_PEN;
		supsmr |= UCC_UART_SUPSMR_PEN;
		char_length++;  /* + PEN */

		if (!(termios->c_cflag & PARODD)) {
			upsmr &= ~(UCC_UART_UPSMR_RPM_MASK |
				   UCC_UART_UPSMR_TPM_MASK);
			upsmr |= UCC_UART_UPSMR_RPM_EVEN |
				UCC_UART_UPSMR_TPM_EVEN;
			supsmr &= ~(UCC_UART_SUPSMR_RPM_MASK |
				    UCC_UART_SUPSMR_TPM_MASK);
			supsmr |= UCC_UART_SUPSMR_RPM_EVEN |
				UCC_UART_SUPSMR_TPM_EVEN;
		}
	}

	/*
	 * Set up parity check flag
	 */
	port->read_status_mask = BD_SC_EMPTY | BD_SC_OV;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= BD_SC_FR | BD_SC_PR;
	if (termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		port->read_status_mask |= BD_SC_BR;

	/*
	 * Characters to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= BD_SC_PR | BD_SC_FR;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= BD_SC_BR;
		/*
		 * If we're ignore parity and break indicators, ignore
		 * overruns too.  (For real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= BD_SC_OV;
	}
	/*
	 * !!! ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->read_status_mask &= ~BD_SC_EMPTY;

        if (UCC_UART_ASYNC_MODE == qe_port->ucc_mode)
        {
	    baud = uart_get_baud_rate(port, termios, old, 0, 230400);
        }
        else
	    baud = uart_get_baud_rate(port, termios, old, 0, 614400);


	/* Do we really need a spinlock here? */
	spin_lock_irqsave(&port->lock, flags);

	/* Update the per-port timeout. */
	uart_update_timeout(port, termios->c_cflag, baud);
//#ifndef __ASYNC_UART__
        if (UCC_UART_SYNC_MODE == qe_port->ucc_mode)
        {
            printk("SyncUCCUART : Setting Synchronous MODE \r\n");
	    upsmr |= UCC_UART_UPSMR_SYN;
        }
//#endif

	iowrite16be(upsmr, &uccp->upsmr);
#ifdef __DEBUG_UCC_SYNC_UART__ 
        printk("SyncUCCUART : Setting Synchronous MODE io READ16 %x : %x \r\n", upsmr, ioread16be(&uccp->upsmr));
#endif
	if (soft_uart) {
		iowrite16be(supsmr, &uccup->supsmr);
		iowrite8(char_length, &uccup->rx_length);

		/* Soft-UART requires a 1X multiplier for TX */
		qe_setbrg(qe_port->us_info.rx_clock, baud, 16);
		qe_setbrg(qe_port->us_info.tx_clock, baud, 1);
	} else {
//#ifdef __ASYNC_UART__
            if (UCC_UART_ASYNC_MODE == qe_port->ucc_mode)
            {
		qe_setbrg(qe_port->us_info.rx_clock, baud, 16);
		qe_setbrg(qe_port->us_info.tx_clock, baud, 16);
            }
            else
            {
//#else
                printk("SyncUCCUART : Setting Synchronous MODE RX/TX Clock \r\n");
		qe_setbrg(qe_port->us_info.rx_clock, baud, 1);
		qe_setbrg(qe_port->us_info.tx_clock, baud, 1);
            }
//#endif
	}

//#ifndef __ASYNC_UART__
        if (UCC_UART_SYNC_MODE == qe_port->ucc_mode)
            printk("SyncUCCUART : %s QE IO Clock Control Register (SCFG_QEIOCLKCR) = %x \r\n", __func__, *(unsigned int*)qe_port->scfg_base);
//#endif
	spin_unlock_irqrestore(&port->lock, flags);
}

/*
 * Return a pointer to a string that describes what kind of port this is.
 */
static const char *qe_uart_type(struct uart_port *port)
{
	return "QE";
}

/*
 * Allocate any memory and I/O resources required by the port.
 */
static int qe_uart_request_port(struct uart_port *port)
{
	int ret;
	struct uart_qe_port *qe_port =
		container_of(port, struct uart_qe_port, port);
	struct ucc_slow_info *us_info = &qe_port->us_info;
	struct ucc_slow_private *uccs;
	unsigned int rx_size, tx_size;
	void *bd_virt;
	dma_addr_t bd_dma_addr = 0;

	ret = ucc_slow_init(us_info, &uccs);
	if (ret) {
		dev_err(port->dev, "could not initialize UCC%u\n",
		       qe_port->ucc_num);
		return ret;
	}
        
	qe_port->us_private = uccs;
	qe_port->uccp = uccs->us_regs;
	qe_port->uccup = (struct ucc_uart_pram *) uccs->us_pram;
	qe_port->rx_bd_base = uccs->rx_bd;
	qe_port->tx_bd_base = uccs->tx_bd;

	/*
	 * Allocate the transmit and receive data buffers.
	 */

	rx_size = L1_CACHE_ALIGN(qe_port->rx_nrfifos * qe_port->rx_fifosize);
	tx_size = L1_CACHE_ALIGN(qe_port->tx_nrfifos * qe_port->tx_fifosize);

	bd_virt = dma_alloc_coherent(port->dev, rx_size + tx_size, &bd_dma_addr,
		GFP_KERNEL);
	if (!bd_virt) {
		dev_err(port->dev, "could not allocate buffer descriptors\n");
		return -ENOMEM;
	}

	qe_port->bd_virt = bd_virt;
	qe_port->bd_dma_addr = bd_dma_addr;
	qe_port->bd_size = rx_size + tx_size;

	qe_port->rx_buf = bd_virt;
	qe_port->tx_buf = qe_port->rx_buf + rx_size;

	return 0;
}

/*
 * Configure the port.
 *
 * We say we're a CPM-type port because that's mostly true.  Once the device
 * is configured, this driver operates almost identically to the CPM serial
 * driver.
 */
static void qe_uart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_CPM;
		qe_uart_request_port(port);
	}
}

/*
 * Release any memory and I/O resources that were allocated in
 * qe_uart_request_port().
 */
static void qe_uart_release_port(struct uart_port *port)
{
	struct uart_qe_port *qe_port =
		container_of(port, struct uart_qe_port, port);
	struct ucc_slow_private *uccs = qe_port->us_private;

	dma_free_coherent(port->dev, qe_port->bd_size, qe_port->bd_virt,
			  qe_port->bd_dma_addr);

	ucc_slow_free(uccs);
}

/*
 * Verify that the data in serial_struct is suitable for this device.
 */
static int qe_uart_verify_port(struct uart_port *port,
			       struct serial_struct *ser)
{
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_CPM)
		return -EINVAL;

	if (ser->irq < 0 || ser->irq >= nr_irqs)
		return -EINVAL;

	if (ser->baud_base < 9600)
		return -EINVAL;

	return 0;
}

/*      
 * Set the modem control lines
 *      
 * Although the QE can control the modem control lines (e.g. CTS), we
 * don't need that support. This function must exist, however, otherwise
 * the kernel will panic.
 */     
void qe_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{  
}
        
/*      
 * Get the current modem control line status
 *      
 * Although the QE can control the modem control lines (e.g. CTS), this
 * driver currently doesn't support that, so we always return Carrier
 * Detect, Data Set Ready, and Clear To Send.
 */     
static unsigned int qe_uart_get_mctrl(struct uart_port *port)
{       
        return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}


/* UART operations
 *
 * Details on these functions can be found in Documentation/serial/driver
 */
static struct uart_ops qe_uart_pops = {
	.tx_empty       = qe_uart_tx_empty,
	.set_mctrl      = qe_sdlc_set_mctrl,
	.get_mctrl      = qe_sdlc_get_mctrl,
	.stop_tx	= qe_uart_stop_tx,
	.start_tx       = qe_uart_start_tx,
	.stop_rx	= qe_uart_stop_rx,
	.break_ctl      = qe_uart_break_ctl,
	.startup	= qe_uart_startup,
	.shutdown       = qe_uart_shutdown,
	.set_termios    = qe_uart_set_termios,
	.type   	= qe_uart_type,
	.release_port   = qe_uart_release_port,
	.request_port   = qe_uart_request_port,
	.config_port    = qe_uart_config_port,
	.verify_port    = qe_uart_verify_port,
};

#ifdef __ENABLE_KAPI__
/* =========================================================================
 * KAPI operations (SP5S/SP8S) for kernel clients via dispatcher
 * ========================================================================= */

static void *sp58_ops_open(void *priv)
{
    struct uart_qe_port *qe_port = (struct uart_qe_port *)priv;
    struct sp58_kctx *ctx;

    if (!qe_port)
        return ERR_PTR(-ENODEV);

    if (UCC_UART_SYNC_MODE != qe_port->ucc_mode)
        return ERR_PTR(-ENODEV);

    mutex_lock(&qe_port->kapi_lock);
    if (qe_port->kapi_is_open) {
        mutex_unlock(&qe_port->kapi_lock);
        return ERR_PTR(-EBUSY);
    }

    ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
    if (!ctx) {
        mutex_unlock(&qe_port->kapi_lock);
        return ERR_PTR(-ENOMEM);
    }

    qe_port->kapi_is_open = true;
    ctx->qe_port = qe_port;

    /* Flush RX packet FIFO on open */
    spin_lock_irq(&qe_port->kapi_rx_lock);
    kfifo_reset(&qe_port->kapi_rx_fifo);
    spin_unlock_irq(&qe_port->kapi_rx_lock);

    mutex_unlock(&qe_port->kapi_lock);
    return (void *)ctx;
}

static int sp58_ops_close(void *ctxp)
{
    struct sp58_kctx *ctx = (struct sp58_kctx *)ctxp;
    struct uart_qe_port *qe_port;

    if (!ctx || !ctx->qe_port)
        return -EBADF;

    qe_port = ctx->qe_port;

    mutex_lock(&qe_port->kapi_lock);

    /* Restore defaults */
    qe_port->kapi_cfg.baud_rate = ATC_SPXS_DEFAULT_BAUD;
    qe_port->kapi_cfg.protocol  = 0;
    qe_port->kapi_cfg.clocking  = 0;

    /* Apply default baud to BRG for synchronous ports */
    qe_setbrg(qe_port->us_info.rx_clock, qe_port->kapi_cfg.baud_rate, 1);
    qe_setbrg(qe_port->us_info.tx_clock, qe_port->kapi_cfg.baud_rate, 1);

    /* Flush RX packets */
    spin_lock_irq(&qe_port->kapi_rx_lock);
    kfifo_reset(&qe_port->kapi_rx_fifo);
    spin_unlock_irq(&qe_port->kapi_rx_lock);

    qe_port->kapi_is_open = false;

    mutex_unlock(&qe_port->kapi_lock);

    kfree(ctx);
    return 0;
}

static ssize_t sp58_ops_read(void *ctxp, void *buf, ssize_t count)
{
    struct sp58_kctx *ctx = (struct sp58_kctx *)ctxp;
    struct uart_qe_port *qe_port;
    struct sp58_kapi_rx_hdr hdr;
    unsigned long flags;
    u8 *kbuf = (u8 *)buf;
    u16 pkt_len;
    u32 to_copy;

    if (!ctx || !ctx->qe_port)
        return -EBADF;
    if (!kbuf)
        return -EFAULT;
    if (count <= 0)
        return -EINVAL;

    qe_port = ctx->qe_port;

    /* Non-blocking: return 0 if no packet */
    if (kfifo_len(&qe_port->kapi_rx_fifo) < sizeof(hdr))
        return 0;

    spin_lock_irqsave(&qe_port->kapi_rx_lock, flags);

    if (kfifo_len(&qe_port->kapi_rx_fifo) < sizeof(hdr)) {
        spin_unlock_irqrestore(&qe_port->kapi_rx_lock, flags);
        return 0;
    }

    if (kfifo_out(&qe_port->kapi_rx_fifo, &hdr, sizeof(hdr)) != sizeof(hdr)) {
        spin_unlock_irqrestore(&qe_port->kapi_rx_lock, flags);
        return -EIO;
    }

    pkt_len = hdr.len;

    if ((pkt_len == 0) || (pkt_len > SP58_KAPI_PKT_MAX)) {
        kfifo_reset(&qe_port->kapi_rx_fifo);
        spin_unlock_irqrestore(&qe_port->kapi_rx_lock, flags);
        return -EFAULT;
    }

    to_copy = (count < (ssize_t)pkt_len) ? count : pkt_len;

    if (kfifo_out(&qe_port->kapi_rx_fifo, kbuf, to_copy) != to_copy) {
        spin_unlock_irqrestore(&qe_port->kapi_rx_lock, flags);
        return -EIO;
    }

    /* Discard remainder if count < pkt_len */
    if (to_copy < pkt_len) {
        u16 discard = pkt_len - to_copy;
        u8 dummy;
        while (discard--) {
            if (kfifo_out(&qe_port->kapi_rx_fifo, &dummy, 1) != 1) {
                spin_unlock_irqrestore(&qe_port->kapi_rx_lock, flags);
                return -EIO;
            }
        }
    }

    spin_unlock_irqrestore(&qe_port->kapi_rx_lock, flags);

    return to_copy;
}

static ssize_t sp58_ops_write(void *ctxp, const void *buf, ssize_t count)
{
    struct sp58_kctx *ctx = (struct sp58_kctx *)ctxp;
    struct uart_qe_port *qe_port;
    const u8 *kbuf = (const u8 *)buf;
    struct qe_bd *bdp;
    u8 *p;
    u16 frame_len;

    if (!ctx || !ctx->qe_port)
        return -EBADF;
    if (!kbuf)
        return -EFAULT;
    if (count <= 0)
        return -EINVAL;

    qe_port = ctx->qe_port;

    /* Must fit in a single packet within one BD FIFO for sync mode */
    if (count > (ssize_t)qe_port->tx_fifosize)
        return -EINVAL;

    mutex_lock(&qe_port->kapi_lock);

    /* Non-blocking: if next TX BD is still READY, would block */
    bdp = qe_port->tx_cur;
    if (ioread16be(&bdp->status) & BD_SC_READY) {
        mutex_unlock(&qe_port->kapi_lock);
        return -EAGAIN;
    }

    p = qe2cpu_addr(be32_to_cpu(bdp->buf), qe_port);

    memcpy(p, kbuf, count);

    frame_len = (u16)count;
    iowrite16be(frame_len, &bdp->length);
    qe_setbits16(&bdp->status, BD_SC_READY);

    /* Advance to next TX BD */
    if (ioread16be(&bdp->status) & BD_SC_WRAP)
        qe_port->tx_cur = qe_port->tx_bd_base;
    else
        qe_port->tx_cur = bdp + 1;

    /* Ensure TX interrupt enabled and kick TX */
    qe_setbits16(&qe_port->uccp->uccm, UCC_UART_UCCE_TX);

    mutex_unlock(&qe_port->kapi_lock);

    return count;
}

static int sp58_ops_ioctl(void *ctxp, int command, void *parameters)
{
    struct sp58_kctx *ctx = (struct sp58_kctx *)ctxp;
    struct uart_qe_port *qe_port;
    unsigned long flags;

    if (!ctx || !ctx->qe_port)
        return -EBADF;

    qe_port = ctx->qe_port;

    switch (command) {
    case ATC_SPXS_WRITE_CONFIG: {
        atc_spxs_config_t *cfg = (atc_spxs_config_t *)parameters;

        if (!cfg)
            return -EFAULT;

        if (cfg->baud_rate <= 0 || cfg->baud_rate > 614400)
            return -EINVAL;

        mutex_lock(&qe_port->kapi_lock);
        memcpy(&qe_port->kapi_cfg, cfg, sizeof(*cfg));

        /* Apply baud immediately for sync mode */
        qe_setbrg(qe_port->us_info.rx_clock, qe_port->kapi_cfg.baud_rate, 1);
        qe_setbrg(qe_port->us_info.tx_clock, qe_port->kapi_cfg.baud_rate, 1);

        mutex_unlock(&qe_port->kapi_lock);
        return 0;
    }

    case ATC_SPXS_READ_CONFIG: {
        atc_spxs_config_t *cfg = (atc_spxs_config_t *)parameters;

        if (!cfg)
            return -EFAULT;

        mutex_lock(&qe_port->kapi_lock);
        memcpy(cfg, &qe_port->kapi_cfg, sizeof(*cfg));
        mutex_unlock(&qe_port->kapi_lock);
        return 0;
    }

    case FIONREAD: {
        int *bytes_avail = (int *)parameters;
        struct sp58_kapi_rx_hdr hdr;

        if (!bytes_avail)
            return -EFAULT;

        *bytes_avail = 0;

        spin_lock_irqsave(&qe_port->kapi_rx_lock, flags);
        if (kfifo_len(&qe_port->kapi_rx_fifo) >= sizeof(hdr)) {
            if (kfifo_out_peek(&qe_port->kapi_rx_fifo, &hdr, sizeof(hdr)) == sizeof(hdr))
                *bytes_avail = (int)hdr.len;
        }
        spin_unlock_irqrestore(&qe_port->kapi_rx_lock, flags);
        return 0;
    }

    default:
        return -ENOTTY;
    }
}

static const struct spxs_kapi_ops sp58_kapi_ops = {
    .open  = sp58_ops_open,
    .close = sp58_ops_close,
    .read  = sp58_ops_read,
    .write = sp58_ops_write,
    .ioctl = sp58_ops_ioctl,
};
#endif /* __ENABLE_KAPI__ */

/*
 * Obtain the SOC model number and revision level
 *
 * This function parses the device tree to obtain the SOC model.  It then
 * reads the SVR register to the revision.
 *
 * The device tree stores the SOC model two different ways.
 *
 * The new way is:
 *
 *      	cpu@0 {
 *      		compatible = "PowerPC,8323";
 *      		device_type = "cpu";
 *      		...
 *
 *
 * The old way is:
 *      	 PowerPC,8323@0 {
 *      		device_type = "cpu";
 *      		...
 *
 * This code first checks the new way, and then the old way.
 */
static unsigned int soc_info(unsigned int *rev_h, unsigned int *rev_l)
{
	struct device_node *np;
	const char *soc_string;
#ifdef CONFIG_PPC_85xx
	unsigned int svr;
#endif
	unsigned int soc;

	/* Find the CPU node */
	np = of_find_node_by_type(NULL, "cpu");
	if (!np)
		return 0;
	/* Find the compatible property */
	soc_string = of_get_property(np, "compatible", NULL);
	if (!soc_string)
		/* No compatible property, so try the name. */
		soc_string = np->name;

	/* Extract the SOC number from the "PowerPC," string */
	if ((sscanf(soc_string, "PowerPC,%u", &soc) != 1) || !soc)
		return 0;

#ifdef CONFIG_PPC_85xx
	/* Get the revision from the SVR */
	svr = mfspr(SPRN_SVR);
	*rev_h = (svr >> 4) & 0xf;
	*rev_l = svr & 0xf;
#endif

	return soc;
}

/*
 * requst_firmware_nowait() callback function
 *
 * This function is called by the kernel when a firmware is made available,
 * or if it times out waiting for the firmware.
 */
static void uart_firmware_cont(const struct firmware *fw, void *context)
{
	struct qe_firmware *firmware;
	struct device *dev = context;
	int ret;

	if (!fw) {
		dev_err(dev, "firmware not found\n");
		return;
	}

	firmware = (struct qe_firmware *) fw->data;

	if (firmware->header.length != fw->size) {
		dev_err(dev, "invalid firmware\n");
		goto out;
	}

	ret = qe_upload_firmware(firmware);
	if (ret) {
		dev_err(dev, "could not load firmware\n");
		goto out;
	}

	firmware_loaded = 1;
 out:
	release_firmware(fw);
}

static int ucc_uart_probe(struct platform_device *ofdev)
{
	struct device_node *np = ofdev->dev.of_node;
	const char *sprop;      /* String OF properties */
	struct uart_qe_port *qe_port = NULL;
	struct resource res;
	int ret;
	u32 val;
        unsigned int reg_val;
        struct device *dev;

        uint8_t *u1clk = "clk11";
        uint8_t *u3clk = "clk10";

	/*
	 * Determine if we need Soft-UART mode
	 */
	if (of_find_property(np, "soft-uart", NULL)) {
		dev_dbg(&ofdev->dev, "using Soft-UART mode\n");
		soft_uart = 1;
	}

	/*
	 * If we are using Soft-UART, determine if we need to upload the
	 * firmware, too.
	 */
	if (soft_uart) {
		struct qe_firmware_info *qe_fw_info;

		qe_fw_info = qe_get_firmware_info();

		/* Check if the firmware has been uploaded. */
		if (qe_fw_info && strstr(qe_fw_info->id, "Soft-UART")) {
			firmware_loaded = 1;
		} else {
			char filename[32];
			unsigned int soc;
			unsigned int rev_h;
			unsigned int rev_l;

			soc = soc_info(&rev_h, &rev_l);
			if (!soc) {
				dev_err(&ofdev->dev, "unknown CPU model\n");
				return -ENXIO;
			}
			sprintf(filename, "fsl_qe_ucode_uart_%u_%u%u.bin",
				soc, rev_h, rev_l);

			dev_info(&ofdev->dev, "waiting for firmware %s\n",
				filename);

			/*
			 * We call request_firmware_nowait instead of
			 * request_firmware so that the driver can load and
			 * initialize the ports without holding up the rest of
			 * the kernel.  If hotplug support is enabled in the
			 * kernel, then we use it.
			 */
			ret = request_firmware_nowait(THIS_MODULE,
				FW_ACTION_HOTPLUG, filename, &ofdev->dev,
				GFP_KERNEL, &ofdev->dev, uart_firmware_cont);
			if (ret) {
				dev_err(&ofdev->dev,
					"could not load firmware %s\n",
					filename);
				return ret;
			}
		}
	}

	qe_port = kzalloc(sizeof(struct uart_qe_port), GFP_KERNEL);
	if (!qe_port) {
		dev_err(&ofdev->dev, "can't allocate QE port structure\n");
		return -ENOMEM;
	}
#ifdef __ENABLE_KAPI__
	/* Initialize KAPI structures (does not affect normal tty path) */
	mutex_init(&qe_port->kapi_lock);
	spin_lock_init(&qe_port->kapi_rx_lock);
	qe_port->kapi_is_open = false;
	qe_port->kapi_cfg.baud_rate = ATC_SPXS_DEFAULT_BAUD;
	qe_port->kapi_cfg.protocol = 0;
	qe_port->kapi_cfg.clocking = 0;

	if (kfifo_alloc(&qe_port->kapi_rx_fifo, SP58_KAPI_RX_FIFO_BYTES, GFP_KERNEL)) {
		dev_err(&ofdev->dev, "could not allocate KAPI RX FIFO\n");
		kfree(qe_port);
		return -ENOMEM;
	}
#endif /* __ENABLE_KAPI__ */

	/* Search for IRQ and mapbase */
	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		dev_err(&ofdev->dev, "missing 'reg' property in device tree\n");
		goto out_free;
	}
	if (!res.start) {
		dev_err(&ofdev->dev, "invalid 'reg' property in device tree\n");
		ret = -EINVAL;
		goto out_free;
	}
	qe_port->port.mapbase = res.start;

	qe_port->cd_timeout_ms = 1000; /* recommend 1000ms unless customer says otherwise */
	INIT_DELAYED_WORK(&qe_port->cd_drop_work, qe_sdlc_cd_drop_workfn);
	qe_port->soft_cd_asserted = false;
	qe_port->last_good_rx_j = jiffies;

        /*
         *
         */
         qe_port->ucc_mode = ucc_mode_param;

         printk("UCC UART Base = %x \r\n", res.start);
//#ifndef __ASYNC_UART__
         if (UCC_UART_SYNC_MODE == qe_port->ucc_mode)
         {
             printk("UCC Configuring Synchronous UART ********** \r\n");
             /* 
              * Allocate the memory
              */
             qe_port->scfg_base = ioremap(SCFG_QEIOCLKCR_REG, 0x10);

             printk("SyncUCCUART : %s QE IO Clock Control Register (SCFG_QEIOCLKCR) = %x \r\n", __func__, *(unsigned int*)qe_port->scfg_base);

             reg_val = *(unsigned int*)qe_port->scfg_base;

             if (UCC1_BASE_REG == res.start)
             {
                 /* 
                  * Clear the BRG01 bits
                  */
                 reg_val = reg_val & (~ENABLE_BRG01_CLK_MASK);

                 /* 
                  * Set the BRG01 as UCC1 TX_CLK in SCFG_QEIOCLKCR reg 
                  */
                 reg_val = reg_val | ENABLE_BRG01_CLK;
                 *(unsigned int*)qe_port->scfg_base = reg_val;
                 printk("SyncUCCUART : Enabling UCC1 BRG01-TXCLK \r\n");

                 qe_port->ucc_port = 1;
             }
             else if (UCC3_BASE_REG == res.start)
             {
                 /* 
                  * Clear the BRG02 bits
                  */
                 reg_val = reg_val & (~ENABLE_BRG02_CLK_MASK);

                 /* 
                  * Set the BRG02 as UCC3 TX_CLK in SCFG_QEIOCLKCR reg 
                  */
                 reg_val = reg_val | ENABLE_BRG02_CLK;
                 *(unsigned int*)qe_port->scfg_base = reg_val;
                 printk("SyncUCCUART : Enabling UCC3 BRG02-TXCLK \r\n");
                 
                 qe_port->ucc_port = 3;

            }
            printk("SyncUCCUART : %s QE IO Clock Control Register (SCFG_QEIOCLKCR) Set to Zero = %x \r\n", __func__, *(unsigned int*)qe_port->scfg_base);
        }
        else
        {
            printk("UCC Configuring Asynchronous UART ********** \r\n");
             if (UCC1_BASE_REG == res.start)
             {
                 qe_port->ucc_port = 1;
             }
             else if (UCC3_BASE_REG == res.start)
             {
                 qe_port->ucc_port = 3;
             }

            qe_port->ucc_mode = UCC_UART_ASYNC_MODE;
        }
//#endif
	/* Get the UCC number (device ID) */
	/* UCCs are numbered 1-7 */
	ret = of_property_read_u32_index(np, "cell-index", 0, &val);
	if (ret) {
		ret = of_property_read_u32_index(np, "device-id", 0, &val);
		if (ret) {
			dev_err(&ofdev->dev, "UCC is unspecified in "
				"device tree\n");
			ret = -EINVAL;
			goto out_free;
		}
	}

	if ((val < 1) || (val > UCC_MAX_NUM)) {
		dev_err(&ofdev->dev, "no support for UCC%u\n", val);
		ret = -ENODEV;
		goto out_free;
	}
	qe_port->ucc_num = val - 1;

	/*
	 * In the future, we should not require the BRG to be specified in the
	 * device tree.  If no clock-source is specified, then just pick a BRG
	 * to use.  This requires a new QE library function that manages BRG
	 * assignments.
	 */

	sprop = of_get_property(np, "rx-clock-name", NULL);
	if (!sprop) {
		dev_err(&ofdev->dev, "missing rx-clock-name in device tree\n");
		ret = -ENODEV;
		goto out_free;
	}

        /* 
         * If the mode is SYNC, Change the clks
         */
        if (UCC_UART_SYNC_MODE == qe_port->ucc_mode)
        {
             if (UCC1_BASE_REG == res.start)
             {
                 sprop = u1clk;
             }
             else
             {
                 sprop = u3clk;
             } 
        }


	qe_port->us_info.rx_clock = qe_clock_source(sprop);
#if 0
	if ((qe_port->us_info.rx_clock < QE_BRG1) ||
	    (qe_port->us_info.rx_clock > QE_BRG16)) {
		dev_err(&ofdev->dev, "rx-clock-name must be a BRG for UART\n");
		ret = -ENODEV;
		goto out_free;
	}
#endif
#ifdef LOOPBACK
	/* In internal loopback mode, TX and RX must use the same clock */
	qe_port->us_info.tx_clock = qe_port->us_info.rx_clock;
#else
	sprop = of_get_property(np, "tx-clock-name", NULL);
	if (!sprop) {
		dev_err(&ofdev->dev, "missing tx-clock-name in device tree\n");
		ret = -ENODEV;
		goto out_free;
	}
	qe_port->us_info.tx_clock = qe_clock_source(sprop);
#endif
#if 0
	if ((qe_port->us_info.tx_clock < QE_BRG1) ||
	    (qe_port->us_info.tx_clock > QE_BRG16)) {
		dev_err(&ofdev->dev, "tx-clock-name must be a BRG for UART\n");
		ret = -ENODEV;
		goto out_free;
	}
#endif
	/* Get the port number, numbered 0-3 */
	ret = of_property_read_u32_index(np, "port-number", 0, &val);
	if (ret) {
		dev_err(&ofdev->dev, "missing port-number in device tree\n");
		ret = -EINVAL;
		goto out_free;
	}
	qe_port->port.line = val;
	if (qe_port->port.line >= UCC_MAX_UART) {
		dev_err(&ofdev->dev, "port-number must be 0-%u\n",
			UCC_MAX_UART - 1);
		ret = -EINVAL;
		goto out_free;
	}

	qe_port->port.irq = irq_of_parse_and_map(np, 0);
	if (qe_port->port.irq == 0) {
		dev_err(&ofdev->dev, "could not map IRQ for UCC%u\n",
		       qe_port->ucc_num + 1);
		ret = -EINVAL;
		goto out_free;
	}

	/*
	 * Newer device trees have an "fsl,qe" compatible property for the QE
	 * node, but we still need to support older device trees.
	 */
	np = of_find_compatible_node(NULL, NULL, "fsl,qe");
	if (!np) {
		np = of_find_node_by_type(NULL, "qe");
		if (!np) {
			dev_err(&ofdev->dev, "could not find 'qe' node\n");
			ret = -EINVAL;
			goto out_free;
		}
	}

	ret = of_property_read_u32_index(np, "brg-frequency", 0, &val);
	if (ret) {
		dev_err(&ofdev->dev,
		       "missing brg-frequency in device tree\n");
		ret = -EINVAL;
		goto out_np;
	}

	if (val)
		qe_port->port.uartclk = val;
	else {
		/*
		 * Older versions of U-Boot do not initialize the brg-frequency
		 * property, so in this case we assume the BRG frequency is
		 * half the QE bus frequency.
		 */
		ret = of_property_read_u32_index(np, "bus-frequency", 0, &val);
		if (ret) {
			dev_err(&ofdev->dev,
				"missing QE bus-frequency in device tree\n");
			ret = -EINVAL;
			goto out_np;
		}
		if (val)
			qe_port->port.uartclk = val / 2;
		else {
			dev_err(&ofdev->dev,
				"invalid QE bus-frequency in device tree\n");
			ret = -EINVAL;
			goto out_np;
		}
	}

	spin_lock_init(&qe_port->port.lock);
	qe_port->np = np;
	qe_port->port.dev = &ofdev->dev;
	qe_port->port.ops = &qe_uart_pops;
	qe_port->port.iotype = UPIO_MEM;

	qe_port->tx_nrfifos = TX_NUM_FIFO;
	qe_port->tx_fifosize = TX_BUF_SIZE;
	qe_port->rx_nrfifos = RX_NUM_FIFO;
	qe_port->rx_fifosize = RX_BUF_SIZE;

	qe_port->wait_closing = UCC_WAIT_CLOSING;
	qe_port->port.fifosize = 512;
	qe_port->port.flags = UPF_BOOT_AUTOCONF | UPF_IOREMAP;

	qe_port->us_info.ucc_num = qe_port->ucc_num;
	qe_port->us_info.regs = (phys_addr_t) res.start;
	qe_port->us_info.irq = qe_port->port.irq;

	qe_port->us_info.rx_bd_ring_len = qe_port->rx_nrfifos;
	qe_port->us_info.tx_bd_ring_len = qe_port->tx_nrfifos;

	/* Make sure ucc_slow_init() initializes both TX and RX */
	qe_port->us_info.init_tx = 1;
	qe_port->us_info.init_rx = 1;

	/* Add the port to the uart sub-system.  This will cause
	 * qe_uart_config_port() to be called, so the us_info structure must
	 * be initialized.
	 */
	ret = uart_add_one_port(&ucc_uart_driver, &qe_port->port);
	if (ret) {
		dev_err(&ofdev->dev, "could not add /dev/ttyQE%u\n",
		       qe_port->port.line);
		goto out_np;
	}

#ifdef __ENABLE_KAPI__
	/* Register dispatcher channels for synchronous ports only */
	if (UCC_UART_SYNC_MODE == qe_port->ucc_mode) {
		if (qe_port->ucc_port == 1) {
			ret = spxs_kapi_register_channel(ATC_LKM_SP5S, &sp58_kapi_ops, qe_port);
			if (ret)
				dev_warn(&ofdev->dev, "Failed to register channel %d: %d\n", ATC_LKM_SP5S, ret);
			else
				dev_info(&ofdev->dev, "Registered channel %d for SP5S\n", ATC_LKM_SP5S);
		} else if (qe_port->ucc_port == 3) {
			ret = spxs_kapi_register_channel(ATC_LKM_SP8S, &sp58_kapi_ops, qe_port);
			if (ret)
				dev_warn(&ofdev->dev, "Failed to register channel %d: %d\n", ATC_LKM_SP8S, ret);
			else
				dev_info(&ofdev->dev, "Registered channel %d for SP8S\n", ATC_LKM_SP8S);
		}
	}
#endif /* __ENABLE_KAPI__ */

#if 0
        /*
         * Create the UCC UART device node
         */
        if (!ucc_uart_class) 
        {
            ucc_uart_class = class_create(THIS_MODULE, "ucc_uart_class");
            if (IS_ERR(ucc_uart_class)) 
            {
                uart_remove_one_port(&ucc_uart_driver, &qe_port->port);
                goto out_np;
            }
        }

#endif

#if 0
        // Remove the automatically created device node (e.g., /dev/z_uart0)
        tty_unregister_device(ucc_uart_driver.tty_driver, qe_port->port.line);
        /* 
         * Create device node for the Synchronous UCC UCART
         */
        if (UCC_UART_SYNC_MODE == qe_port->ucc_mode)
        {

            if (1 == qe_port->ucc_port)
            {
                printk("Creating device node name as sp5s\r\n");
                /*
                 * Create device node /dev/sp5s
                 */
                //device_create(ucc_uart_class, NULL, MKDEV(ucc_uart_driver.major,  qe_port->port.line), NULL, "sp5s");
                //ret = device_create(ucc_uart_class, NULL, MKDEV(ucc_uart_driver.major, 46), NULL, "sp5s");
                dev = tty_port_register_device_attr(&qe_port->port.state->port, ucc_uart_driver.tty_driver, 0, NULL, NULL, "sp5s");

            } 
            else if (3 == qe_port->ucc_port) 
            {
                printk("Creating device node name as sp8s\r\n");
                /*
                 * Create device node /dev/sp8s
                 */
                //ret = device_create(ucc_uart_class, NULL, MKDEV(ucc_uart_driver.major,  47), NULL, "sp8s");
                dev = tty_port_register_device_attr(&qe_port->port.state->port, ucc_uart_driver.tty_driver, 0, NULL, NULL, "sp8s");
            }
        }
        else  /* Asynchronous UCC UART */
        {
            if (1 == qe_port->ucc_port)
            {
                printk("Creating device node name as sp5\r\n");
                /*
                 * Create device node /dev/sp5
                 */
                device_create(ucc_uart_class, NULL, MKDEV(ucc_uart_driver.major,  qe_port->port.line), NULL, "sp5");
            } 
            else if (3 == qe_port->ucc_port) 
            {
                printk("Creating device node name as sp8\r\n");
                /*
                 * Create device node /dev/sp8
                 */
                device_create(ucc_uart_class, NULL, MKDEV(ucc_uart_driver.major,  qe_port->port.line), NULL, "sp8");
            }
        }

        if (IS_ERR(dev)) 
        {
            printk("Firstview : failed to create custom device node ret = %x = %d\n", ret, ret);
            uart_remove_one_port(&ucc_uart_driver, &qe_port->port);
            goto out_np;
       }
#endif
	platform_set_drvdata(ofdev, qe_port);
#if 1
	dev_info(&ofdev->dev, "UCC%u assigned to /dev/ttyQE%u\n",
		qe_port->ucc_num + 1, qe_port->port.line);

	/* Display the mknod command for this device */
	dev_dbg(&ofdev->dev, "mknod command is 'mknod /dev/ttyQE%u c %u %u'\n",
	       qe_port->port.line, SERIAL_QE_MAJOR,
	       SERIAL_QE_MINOR + qe_port->port.line);
#endif
	return 0;
out_np:
	of_node_put(np);
out_free:
#ifdef __ENABLE_KAPI__
	kfifo_free(&qe_port->kapi_rx_fifo);
#endif /* __ENABLE_KAPI__ */
	kfree(qe_port);
	return ret;
}

static int ucc_uart_remove(struct platform_device *ofdev)
{
	struct uart_qe_port *qe_port = platform_get_drvdata(ofdev);
#ifdef __ENABLE_KAPI__
	/* Unregister KAPI channels for synchronous ports */
	if (UCC_UART_SYNC_MODE == qe_port->ucc_mode) {
		if (qe_port->ucc_port == 1) {
			spxs_kapi_unregister_channel(ATC_LKM_SP5S);
			dev_info(&ofdev->dev, "Unregistered channel %d\n", ATC_LKM_SP5S);
		} else if (qe_port->ucc_port == 3) {
			spxs_kapi_unregister_channel(ATC_LKM_SP8S);
			dev_info(&ofdev->dev, "Unregistered channel %d\n", ATC_LKM_SP8S);
		}
	}
#endif /* __ENABLE_KAPI__ */
        unsigned int reg_val;

	dev_info(&ofdev->dev, "removing /dev/ttyQE%u\n", qe_port->port.line);
	printk("removing /dev/ttyQE%u qe_port->ucc_mode=%d qe_port->ucc_port= %d\n", qe_port->port.line, qe_port->ucc_mode, qe_port->ucc_port);

//#ifndef __ASYNC__UART__
        if (UCC_UART_SYNC_MODE == qe_port->ucc_mode)
        {
            if (1 == qe_port->ucc_port)
            {
                /* 
                 * Clear the BRG01 bits
                 */
                reg_val = reg_val & (~ENABLE_BRG01_CLK_MASK);
                *(unsigned int*)qe_port->scfg_base = reg_val;
                printk("SyncUCCUART : Clearing UCC1 BRG01-TXCLK from SCFG_QEIOCLKCR reg base = %x \r\n", qe_port->scfg_base);

                iounmap(qe_port->scfg_base);
            }
            else
            if (3 == qe_port->ucc_port)
            {
                /* 
                 * Clear the BRG02 bits
                 */
                reg_val = reg_val & (~ENABLE_BRG02_CLK_MASK);
                *(unsigned int*)qe_port->scfg_base = reg_val;
                printk("SyncUCCUART : Clearing UCC3 BRG02-TXCLK from SCFG_QEIOCLKCR reg base = %x \r\n", qe_port->scfg_base);
            
               iounmap(qe_port->scfg_base);
            }
      }
//#endif
#if 0
        if (1 == qe_port->ucc_port)
        {
            printk("Destroying sp5/sp5s node\r\n");
            /* 
             * Remove the Device node
             */
            device_destroy(ucc_uart_class, MKDEV(ucc_uart_driver.major, qe_port->port.line));
        }
        else
        if (3 == qe_port->ucc_port)
        {
            printk("Destroying sp8/sp8s node\r\n");
            /*
             * Remove the Device node
             */
            device_destroy(ucc_uart_class, MKDEV(ucc_uart_driver.major, qe_port->port.line));
        }
#endif
        qe_port->ucc_port = 0;


	uart_remove_one_port(&ucc_uart_driver, &qe_port->port);

#ifdef __ENABLE_KAPI__
	kfifo_free(&qe_port->kapi_rx_fifo);
#endif /* __ENABLE_KAPI__ */
	kfree(qe_port);

	return 0;
}

static const struct of_device_id ucc_uart_match[] = {
	{
		.type = "serial",
		.compatible = "ucc_uart",
	},
	{},
};
MODULE_DEVICE_TABLE(of, ucc_uart_match);

static struct platform_driver ucc_uart_of_driver = {
	.driver = {
		.name = "ucc_uart",
		.of_match_table    = ucc_uart_match,
	},
	.probe  	= ucc_uart_probe,
	.remove 	= ucc_uart_remove,
};

static int __init ucc_uart_init(void)
{
	int ret;

	printk(KERN_INFO "Freescale QUICC Engine UART device driver\n");
#ifdef LOOPBACK
	printk(KERN_INFO "ucc-uart: Using loopback mode\n");
#endif

	ret = uart_register_driver(&ucc_uart_driver);
	if (ret) {
		printk(KERN_ERR "ucc-uart: could not register UART driver\n");
		return ret;
	}

#if 0  
	printk(KERN_INFO "Creating the class\n");
        /*
         * Create the UCC UART device node
         */
        ucc_uart_class = class_create(THIS_MODULE, "ucc_uart_class");
        if (IS_ERR(ucc_uart_class)) 
        {
            return ret;
        }
#endif
	ret = platform_driver_register(&ucc_uart_of_driver);
	if (ret) {
		printk(KERN_ERR
		       "ucc-uart: could not register platform driver\n");
		uart_unregister_driver(&ucc_uart_driver);
	}

	return ret;
}

static void __exit ucc_uart_exit(void)
{
	printk(KERN_INFO
	       "Freescale QUICC Engine UART device driver unloading\n");

	platform_driver_unregister(&ucc_uart_of_driver);
        
        class_destroy(ucc_uart_class);

	uart_unregister_driver(&ucc_uart_driver);
}

module_init(ucc_uart_init);
module_exit(ucc_uart_exit);

MODULE_DESCRIPTION("Freescale QUICC Engine (QE) UART");
MODULE_AUTHOR("Timur Tabi <timur@freescale.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS_CHARDEV_MAJOR(SERIAL_QE_MAJOR);

module_param(ucc_mode_param, int, 0);
MODULE_PARM_DESC(ucc_mode_param, "1 : Async UCC UART "
                                 "2 : Sync UCC UART ");

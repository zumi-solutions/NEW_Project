/*
 * sp3s-uart.c - ORX-compliant U-frame only SDLC driver
 *
 * Modified to meet ORX requirements for traffic control industry:
 * - Only U-frames supported (no I-frames or S-frames)
 * - Control field always 0x83
 * - No sequence numbers (N(S)/N(R))
 * - Hardware handles bit stuffing and frame delimiting
 *
 * Macro-based mode selection:
 * - Define USE_IRQ_MODE for interrupt-driven operation
 * - Default is polling mode
 */
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/ktime.h>
#include <linux/err.h>
#include <linux/kfifo.h>
#include <linux/ioctl.h>
#include <asm/ioctl.h>
#include <linux/sched.h>  /* For signal_pending() - kernel 4.1.8 compatible */
#include <linux/types.h>  /* For __s8, __u8 etc */

// #define USE_IRQ_MODE

#define p() printk("%s : %d\r\n", __func__, __LINE__)

#ifdef USE_IRQ_MODE
#include <linux/gpio.h>
#include <linux/interrupt.h>
#endif

#include <misc/scm_core.h>
#include "spxs_kapi_dispatch.h"

/*
 * Device Identification Strings
 */
#define DEVICE_NAME       "sp3s"
#define CLASS_NAME        "sp3s_class"

/*
 * Device Memory and Buffer Size Configuration
 */
#define SP3_BASE_ADDR     0x69000000
#define SP3_REG_SIZE      (0x1000 << 1)
#define SP3_TX_FIFO_SIZE  1024
#define SP3_RX_FIFO_SIZE  1024

/* Register Offsets */
#define COMMUNICATIONS_MOD_REG        (0x03  << 1)
#define SP3_SYNC_BAUD_REG             (0x07  << 1)
#define SP3_THR_CTRL_REG              (0x0B  << 1)
#define SP3_TX_FIFO_CTRL              (0x200 << 1)
#define SP3_RX_FIFO_CTRL              (0x201 << 1)
#define SP3_TX_FIFO_CNT_MSB           (0x202 << 1)
#define SP3_TX_FIFO_CNT_LSB           (0x203 << 1)
#define SP3_RX_FIFO_CNT_MSB           (0x204 << 1)
#define SP3_RX_FIFO_CNT_LSB           (0x205 << 1)
#define SP3_TX_FIFO                   (0x400 << 1)
#define SP3_RX_FIFO                   (0x800 << 1)

/* Bit definitions */
#define SP3_TX_START_BIT              (1 << 0)
#define RELEASE_SP3_RX_FIFO_BIT       (1 << 2)
#define SP3_RX_DATA_AVAIL_BIT         (1 << 0)
#define SP3_STATUS_BUSY_BIT           (1 << 0)
#define SYNC_153600_BD_bit            (1 << 0)
#define SYNC_614400_BD_bit            (1 << 1)
#define SYNCHRONOUS_MODE_BIT          (1 << 0)

#define HW_STABILIZATION_DELAY_US 150
#define HW_STABILIZATION_153600_DELAY_US 1000
#define HW_STABILIZATION_614400_DELAY_US 500

#define SCM_TIMEOUT               100
#define MAX_RETRY_COUNT 20

/* Default local address */
#define DEFAULT_ADDRESS      0x14
/* ORX Requirement: Control field is always 0x83 in traffic control industry */
#define ORX_CONTROL_FIELD     0x83

/* GPIO for IRQ mode */
#define LPC43120_RXRDY_GPIO   500

/* Use definitions from spxs_kapi_dispatch.h */
#define ATC_SPXS_SET_ADDRESS    (0x1003)
#define ATC_SPXS_GET_ADDRESS    (0x1004)
#define ATC_SPXS_WRITE_CONFIG   (0x1001)
#define ATC_SPXS_READ_CONFIG    (0x1002)
/* Custom command for checking available bytes */
#define ATC_SPXS_GET_AVAIL      (0x1005)

#define ATC_B153600             153600
#define ATC_B614400             614400

/* Kernel-level (LKM) API channel mapping (customer requirement) */
#define ATC_LKM_SP3S            (3)
#define ATC_LKM_SP5S            (5)
#define ATC_LKM_SP8S            (8)

/* Kernel-level API packet buffering */
#define SP3_KAPI_RX_FIFO_BYTES  (64U * 1024U)
#define SP3_KAPI_PKT_MAX        (2048U)

#define RX_BUF_SIZE 0x200000  /* Adjust as needed */
static uint8_t rx_buffer[RX_BUF_SIZE];
static u32 rx_head = 0;  /* next write position */
static u32 rx_tail = 0;  /* next read position */
static spinlock_t rx_lock;

/* KAPI RX record header: [u16 payload_len][payload bytes...] */
struct sp3_kapi_rx_hdr {
    u16 len;
} __packed;

typedef struct atc_spxs_config {
    unsigned long protocol;
    unsigned long baud;
    unsigned long transmit_clock_source;
    unsigned long transmit_clock_mode;
} atc_spxs_config_t;

/*
 * sp3s_dev:
 *   - Core device structure
 */
struct sp3s_dev {
    struct cdev cdev;
    
#ifdef USE_IRQ_MODE
    /* IRQ specific fields */
    int rx_gpio;
    int rx_irq;
    int use_rx_irq;
    int rx_flag;
    wait_queue_head_t rx_busy;
#endif
    
    spinlock_t lock;
    int is_init;
    struct device *dev;
    wait_queue_head_t rx_wq;

    /* KAPI RX packet FIFO: stores payload-only packets (CRC removed). */
    struct kfifo kapi_rx_fifo;
    spinlock_t kapi_rx_lock;

    /* Default configuration to restore on close (KAPI requirement). */
    atc_spxs_config_t default_config;
    
    struct mutex write_lock;
    char rx_buf[SP3_RX_FIFO_SIZE];
    size_t rx_len;
    
    u8 sdlc_address;
    u8 sdlc_control;
    atc_spxs_config_t sp3sync_config;
    atomic_t open_count;
    
#ifdef __TX_THREAD__
    int tx_list;
#endif
    
#ifdef DEBUG_SP3S
    int total_sent;
    int tx_fifo_ovr_flo;
    int tx_crc_err;
    int tx_data_err;
    int rx_total;
    long int total_drop;
#endif
};

struct window {
    u8 *data;
    u16 len;
    struct window *next;
};
struct window *head = NULL;

#ifdef DEBUG_SP3S
struct window *head_er = NULL;
#endif

/*
 * Static driver-wide variables
 */
static struct sp3s_dev *sp3s;
static dev_t devt;
static struct class *sp3s_class;

#ifdef __TX_THREAD__
static struct task_struct *tx_thread;
#endif

/* Declare a mutex for guarding open/close */
static DEFINE_MUTEX(sp3s_mutex);

/* Lock for protecting TX linked list */
static spinlock_t tx_list_lock;

/* Waitqueue for TX thread wakeups */
static DECLARE_WAIT_QUEUE_HEAD(tx_wq);

/* Forward declarations */
static void sp3_kapi_rx_push(struct sp3s_dev *dev, const u8 *payload, u16 payload_len);
static int32_t tx_payload_with_crc(struct sp3s_dev *dev, const u8 *payload, u32 payload_len);
static void window_sp3_tx(uint8_t *buf, u16 len);
static int8_t tx_sdlc_data(int8_t *data, uint16_t len);

/*
 * crc16_ibm_sdlc - Calculate 16-bit CRC using IBM SDLC polynomial.
 */
static uint16_t crc16_ibm_sdlc(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    int i, j;
    for (i = 0; i < len; ++i) {
        crc ^= data[i];
        for (j = 0; j < 8; ++j) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0x8408;
            else
                crc >>= 1;
        }
    }
    return ~crc;
}

/*
 * free_tx_node_mem - Free all allocated TX window nodes and their data.
 */
void free_tx_node_mem(void) {
    struct window *temp = head;
    while(temp) {
        head = temp->next;
        kfree(temp->data);
        kfree(temp);
        temp = head;
    }
}

/**
 * tx_payload_with_crc() - Send one payload packet as a single SDLC frame
 * @dev: device instance
 * @payload: payload bytes (no addr/ctrl/crc)
 * @payload_len: payload length
 *
 * Frame format sent:
 *   [address][control][payload...][crc_lo][crc_hi]
 *
 * Return: payload_len on success, negative errno on failure.
 */
static int32_t tx_payload_with_crc(struct sp3s_dev *dev, const u8 *payload, u32 payload_len)
{
    u8 *frame;
    u16 crc;
    u32 frame_len;
    unsigned long flags = 0;

    if (!dev || !payload || payload_len == 0U)
        return -EINVAL;

    if (payload_len > (u32)(SP3_TX_FIFO_SIZE - 2U))
        return -EINVAL;

    frame = kmalloc(payload_len + 4U, GFP_KERNEL);
    if (!frame)
        return -ENOMEM;

    frame[0] = dev->sdlc_address;
    frame[1] = dev->sdlc_control;

    memcpy(&frame[2], payload, payload_len);

    crc = crc16_ibm_sdlc(frame, (u16)(payload_len + 2U));

    frame_len = payload_len + 2U;
    frame[frame_len++] = (u8)(crc & 0xFFU);
    frame[frame_len++] = (u8)((crc >> 8) & 0xFFU);

#ifdef __TX_THREAD__
    window_sp3_tx((uint8_t *)frame, (u16)frame_len);
#else
    spin_lock_irqsave(&tx_list_lock, flags);
    tx_sdlc_data((int8_t *)frame, (u16)frame_len);
    spin_unlock_irqrestore(&tx_list_lock, flags);
#endif

    kfree(frame);

    return (int32_t)payload_len;
}

/*
 * sending_data - Transmit data from the head TX window node to the hardware FIFO.
 */
void sending_data(void) {
    struct window *temp = head;
    u16 i  = 0;
    u8 ret = 0;

    if(temp == NULL) {
        return;
    }

#ifdef DEBUG_SP3S
    printk("List num = %d\r\n", sp3s->tx_list);
#endif
    /* Fill TX FIFO */
    for (i = 0; i < temp->len; i++) {
        udelay(HW_STABILIZATION_DELAY_US);
        scm_write8(SP3_TX_FIFO + (i << 1), temp->data[i]);
        udelay(HW_STABILIZATION_DELAY_US);
    }

    /* Set count regs */
    scm_write8(SP3_TX_FIFO_CNT_MSB, temp->len & 0xFF);
    udelay(HW_STABILIZATION_DELAY_US);
    scm_write8(SP3_TX_FIFO_CNT_LSB, (temp->len >> 8) & 0xFF);
    udelay(HW_STABILIZATION_DELAY_US);

    /* Trigger TX */
    scm_write8(SP3_TX_FIFO_CTRL, SP3_TX_START_BIT);
    udelay(HW_STABILIZATION_DELAY_US);
    ret = scm_read8(SP3_TX_FIFO_CTRL);

#ifdef DEBUG_SP3S
    sp3s->total_sent++;
    printk("TX sp3s->total_sent = %d\r\n", sp3s->total_sent);
    if(ret & 0x04) {
        sp3s->tx_fifo_ovr_flo++;
    }
    if(ret & 0x08) {
        sp3s->tx_crc_err++;
    }
    if(ret & 0x10) {
        sp3s->tx_data_err++;
    }
#endif

#ifdef USE_IRQ_MODE
    sp3s->rx_flag = 1;
    wake_up_interruptible(&sp3s->rx_busy);
#endif

    head = temp->next;
    
    kfree(temp->data);
    kfree(temp);
}

#ifdef __TX_THREAD__
/*
 * sp3_tx_thread - Kernel thread managing transmission of queued data.
 */
int sp3_tx_thread(void *data)
{
    while (!kthread_should_stop()) {
        /* Sleep until there is data or timeout */
        wait_event_interruptible_timeout(tx_wq,
                                         (head != NULL) || kthread_should_stop(),
                                         msecs_to_jiffies(10));

        if (kthread_should_stop())
            break;

loop:
        /* If no data, go back to waiting */
        spin_lock_irq(&tx_list_lock);
        if (head == NULL) {
            spin_unlock_irq(&tx_list_lock);
            continue;
        }
        spin_unlock_irq(&tx_list_lock);

        /* Check hardware FIFO status */
        {
            u8 status = scm_read8(SP3_THR_CTRL_REG);

#ifdef DEBUG_SP3S
            printk("Status = 0x%x\r\n", status);
#endif

            if ((status & SP3_STATUS_BUSY_BIT) != 0) {
                /* hardware busy, retry later */
                continue;
            }
        }

        /* ===== Remove node under lock ===== */
        {
            unsigned long flags;

            spin_lock_irqsave(&tx_list_lock, flags);
            sending_data();  /* <-- it sends 'head->data' */

#ifdef DEBUG_SP3S
            sp3s->tx_list--;
#endif

            spin_unlock_irqrestore(&tx_list_lock, flags);
        }

        /* Loop back if more nodes are queued */
        goto loop;
    }

    return 0;
}
#endif

/*
 * window_sp3_tx - Add new data buffer to the TX linked list queue.
 */
static void window_sp3_tx(uint8_t *buf, u16 len)
{
    struct window *node;
    unsigned long flags;

    node = kmalloc(sizeof(*node), GFP_KERNEL);
    if (!node)
        return;

    /* Allocate memory for data */
    node->data = kmalloc(len, GFP_KERNEL);  
    if (!node->data) {
        kfree(node);
        return;
    }

    memcpy(node->data, buf, len);
    node->len = len;
    node->next = NULL;

    spin_lock_irqsave(&tx_list_lock, flags);

    if (!head) {
        head = node;
    } else {
        struct window *tmp = head;
        while (tmp->next)
            tmp = tmp->next;
        tmp->next = node;
    }

#ifdef DEBUG_SP3S
    sp3s->tx_list++;
#endif

    spin_unlock_irqrestore(&tx_list_lock, flags);

    /* Wake up TX thread whenever a new frame is queued */
    wake_up_interruptible(&tx_wq);
}

void byte_delay(void)
{
    if (ATC_B153600 == sp3s->sp3sync_config.baud)
    {
        udelay(HW_STABILIZATION_153600_DELAY_US);
    }
    else
    {
        udelay(HW_STABILIZATION_614400_DELAY_US);
    }
}

/* Write one byte to a register and apply the SDLC byte-to-byte delay */
void sdlc_scm_write8(u8 value, u32 addr)
{
    scm_write8(addr, value);
    byte_delay();
}

/* Read one byte from a register and apply the SDLC byte-to-byte delay */
uint8_t sdlc_scm_read8(u32 addr)
{
    u8 value;
    byte_delay();
    value = scm_read8(addr);
    return value;
}

#ifndef __TX_THREAD__
static int8_t tx_sdlc_data(int8_t *data, uint16_t len)
{
    int16_t index;
    uint8_t ret;

    /* Fill TX FIFO */
    for (index = 0; index < len; index++) {
        sdlc_scm_write8(data[index],  (u32)(SP3_TX_FIFO + (index << 1)));
    }

    /* Set count regs */
    sdlc_scm_write8(len & 0xFF,  (u32)SP3_TX_FIFO_CNT_MSB);
    sdlc_scm_write8((len >> 8) & 0xFF,  (u32)SP3_TX_FIFO_CNT_LSB);

    /* Trigger TX */
    sdlc_scm_write8(SP3_TX_START_BIT,  (u32)SP3_TX_FIFO_CTRL);
    udelay(2000);
    ret = sdlc_scm_read8( (u32)SP3_TX_FIFO_CTRL);

    if(ret & 0x04) {
        pr_err("SCM: FIFO overflow detected, error code: 0x%02x\n", ret);
    }
    if(ret & 0x08) {
        pr_err("SCM: CRC error during transmission, error code: 0x%02x\n", ret);
    }
    if(ret & 0x10) {
        pr_err("SCM: FIFO size mismatch, error code: 0x%02x\n", ret);
    }

#ifdef DEBUG_SP3S
    sp3s->total_sent++;
    if(ret & 0x04) {
        sp3s->tx_fifo_ovr_flo++;
    }
    if(ret & 0x08) {
        sp3s->tx_crc_err++;
    }
    if(ret & 0x10) {
        sp3s->tx_data_err++;
    }
#endif
    
    return 0;
}
#endif

#ifdef DEBUG_SP3S
uint32_t g_count = 0;
#endif

/*
 * sp3s_write - Write callback for user-space data transmission.
 */
static ssize_t sp3s_write(struct file *file, const char __user *ubuf,
                          size_t count, loff_t *ppos)
{
    uint8_t *kbuf;
    uint16_t crc;
    uint8_t   status;
    uint8_t   iter = 0;
    unsigned long flags = 0;

#ifdef DEBUG_SP3S
    printk("Int g_count = %d\r\n", g_count);
#endif

    if (count > (SP3_TX_FIFO_SIZE - 2))
        return -EINVAL;
    
    /* Check hardware FIFO status */
    while(iter < MAX_RETRY_COUNT)
    {
        status = scm_read8(SP3_THR_CTRL_REG);
#ifdef DEBUG_SP3S
        printk("Status = 0x%x\r\n", status);
#endif
        if ((status & SP3_STATUS_BUSY_BIT) == 0) {
            break;
        }
        iter++;
        udelay(2000);
    }

    if (MAX_RETRY_COUNT == iter)
        return -EFAULT;

    kbuf = kmalloc(count + 4, GFP_KERNEL);
    if (!kbuf)
        return -ENOMEM;

    kbuf[0] = sp3s->sdlc_address;
    kbuf[1] = sp3s->sdlc_control;
    
    if (copy_from_user((kbuf + 2), ubuf, count)) {
        kfree(kbuf);
        return -EFAULT;
    }

    /* Append CRC */
    crc = crc16_ibm_sdlc(kbuf, count + 2);
    count += 2;
    kbuf[count++] = crc & 0xFF;
    kbuf[count++] = (crc >> 8) & 0xFF;
    
#ifdef __TX_THREAD__
    window_sp3_tx(kbuf, (u16)(count & 0xFFFF));
#else
    spin_lock_irqsave(&tx_list_lock, flags);
    tx_sdlc_data((int8_t *)kbuf, (u16)(count & 0xFFFF));
    spin_unlock_irqrestore(&tx_list_lock, flags);
#endif

#ifdef DEBUG_SP3S
    printk("CRC = 0x%x\r\n", crc);
    printk("Exit g_count = %d\r\n", g_count);
    g_count++;
#endif
    
    kfree(kbuf);

    /*
     * Return only the number of valid data bytes.
     * The total count excludes the following protocol overhead:
     *   - 1 byte  : Control field
     *   - 1 byte  : Address field
     *   - 2 bytes : CRC
     */
    return (count - 4);
}

/*
 * sp3s_rx_push_data - Push received data into circular buffer
 */
void sp3s_rx_push_data(u8 data)
{
    unsigned long flags;
    u32 next;

    spin_lock_irqsave(&rx_lock, flags);

    /* Compute next head index */
    next = (rx_head + 1) % RX_BUF_SIZE;

    if (next != rx_tail) {  // buffer not full
        rx_buffer[rx_head] = data;
        rx_head = next;
    } else {
        printk(KERN_WARNING "RX buffer overflow, dropping byte\n");
        // optional: handle overflow
    }

    spin_unlock_irqrestore(&rx_lock, flags);
}

/*
 * sp3s_read - Read callback for user-space data reception.
 */
static ssize_t sp3s_read(struct file *file, char __user *buf,
                         size_t count, loff_t *ppos)
{
    unsigned long flags;
    size_t copied = 0;
    size_t copy_len;

    /* Wait until data is available */
    if (rx_head == rx_tail) {
        if (file->f_flags & O_NONBLOCK)
            return -EAGAIN;

#ifdef DEBUG_SP3S
        printk("\r\nWait event Head = %d\r\nTail = %d\r\n", rx_head, rx_tail);
#endif
        if (wait_event_interruptible(sp3s->rx_wq, rx_head != rx_tail)) {
            return -EINTR;  /* Interrupted by signal before any data was read */
        }
    }

    copy_len = min((size_t)count, (size_t)RX_BUF_SIZE);

    while (copied < copy_len) {
        uint8_t byte;
        
        spin_lock_irqsave(&rx_lock, flags);

        if (rx_tail == rx_head) {
            spin_unlock_irqrestore(&rx_lock, flags);
            break; /* no more data */
        }

        byte = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % RX_BUF_SIZE;

        spin_unlock_irqrestore(&rx_lock, flags);

        if (copy_to_user(buf + copied, &byte, 1))
            return -EFAULT;

        copied++;
    }

#ifdef DEBUG_SP3S
    printk("\r\nHead = %d\r\nTail = %d copy_len = %d\r\n", rx_head, rx_tail, copied);
#endif

    return copied;
}

#ifdef DEBUG_SP3S
struct error_sp3s {
    u8 data[2000];
    u16 len;
};

#define ERR_MAX 20
struct error_sp3s err_list[ERR_MAX];
#endif

#ifdef USE_IRQ_MODE
/*
 * sp3s_rx_irq_handler - Interrupt handler for RX data
 */
static irqreturn_t sp3s_rx_irq_handler(int irq, void *dev_id)
#else
static void sp3s_rx_poll(struct work_struct *work)
#endif
{
#ifdef USE_IRQ_MODE
    struct sp3s_dev *dev = (struct sp3s_dev *)dev_id;
#else    
    struct sp3s_dev *dev = (struct sp3s_dev *)sp3s;
#endif
    unsigned long flags;
    uint16_t count, actual_crc, rx_crc;
    uint8_t count_msb, count_lsb, crc_low, crc_high;
    int i;

#ifdef DEBUG_SP3S
    static int err_index;
#endif

    if (!dev->is_init) 
    {
#ifdef USE_IRQ_MODE    
        return IRQ_HANDLED;
#else
        goto out;
#endif
    }

    /* Check if data is available */
    {
        u8 status = scm_read8(SP3_RX_FIFO_CTRL);
        if ((status & SP3_RX_DATA_AVAIL_BIT) == 0) {
#ifdef USE_IRQ_MODE    
            return IRQ_HANDLED;
#else
            goto out;
#endif
        }
    }

#ifdef DEBUG_SP3S
    sp3s->rx_total++;
#endif
    
    /* Read data count */
    udelay(HW_STABILIZATION_DELAY_US);
    count_msb = scm_read8(SP3_RX_FIFO_CNT_MSB);
    udelay(HW_STABILIZATION_DELAY_US);
    count_lsb = scm_read8(SP3_RX_FIFO_CNT_LSB);
    count = (count_lsb << 8) | count_msb;
    udelay(HW_STABILIZATION_DELAY_US);
    udelay(HW_STABILIZATION_DELAY_US);
    
    if (count > 2) {
        spin_lock_irqsave(&dev->lock, flags);
        
        /* Read data from FIFO */
        for (i = 0; i < count && i < SP3_RX_FIFO_SIZE; i++) {
            dev->rx_buf[i] = scm_read8(SP3_RX_FIFO + (i << 1));
            udelay(HW_STABILIZATION_DELAY_US);
        }
        dev->rx_len = count;
        spin_unlock_irqrestore(&dev->lock, flags);
       
        actual_crc = crc16_ibm_sdlc(dev->rx_buf, dev->rx_len - 2);
        crc_low  = dev->rx_buf[dev->rx_len - 2];
        crc_high = dev->rx_buf[dev->rx_len - 1];

        rx_crc = ((u16)crc_high << 8) | crc_low;

        /* Release RX FIFO */
        scm_write8(SP3_RX_FIFO_CTRL, RELEASE_SP3_RX_FIFO_BIT);
        udelay(HW_STABILIZATION_DELAY_US);
        
        if(actual_crc != rx_crc) {
#ifdef DEBUG_SP3S
            err_list[err_index % ERR_MAX].len = count;
            memcpy(err_list[err_index % ERR_MAX].data, dev->rx_buf, count);
            err_index++;
            sp3s->total_drop++;
#endif
            
            printk(KERN_ERR "Crc error Droping frame %04x data\n", count);
#ifdef USE_IRQ_MODE    
            return IRQ_HANDLED;
#else
            goto out;
#endif
        }

        /*
         * 1 - address 1 - Control 2 CRC
         */ 
        {
            /* Push payload packet for kernel KAPI (payload only; CRC removed). */
            u16 payload_len = (count > 4U) ? (u16)(count - 4U) : 0U;
            if (payload_len > 0U) {
                sp3_kapi_rx_push(dev, (const u8 *)&dev->rx_buf[2], payload_len);
            }
        }

        for(i = (1 + 1); i < (count - (2)); i++)
            sp3s_rx_push_data(dev->rx_buf[i]);

        /* Wake up any readers */
        wake_up_interruptible(&dev->rx_wq);
    }
#ifdef USE_IRQ_MODE    
    return IRQ_HANDLED;
#else
out:
    /* reschedule self */
    schedule_delayed_work(to_delayed_work(work), msecs_to_jiffies(10));

    return;
#endif
}

/* Declare work for polling mode */
#ifndef USE_IRQ_MODE
static DECLARE_DELAYED_WORK(rx_work, sp3s_rx_poll);
#endif

/*
 * sp3s_open - File operation: open device.
 */
static int sp3s_open(struct inode *inode, struct file *file)
{
    uint8_t val;
    unsigned long timeout;
    int ret;

    /* Use interruptible mutex lock to allow signal interruption */
    ret = mutex_lock_interruptible(&sp3s_mutex);
    if (ret) {
        if (ret == -EINTR) {
            printk(KERN_WARNING "sp3s: open interrupted by signal\n");
            return -EINTR;  /* Required by specifications */
        }
        printk(KERN_WARNING "sp3s: device is busy\n");
        return -EBUSY;
    }

    /* Save device pointer in file->private_data for later use */
    file->private_data = sp3s;

    val = scm_read8(COMMUNICATIONS_MOD_REG);
    cpu_relax();

    if(val & SYNCHRONOUS_MODE_BIT) {
        val &= (~SYNCHRONOUS_MODE_BIT);
        scm_write8(COMMUNICATIONS_MOD_REG, val);

        /*
         * Adding timeout for safety in case we are unable to successfully
         * write to SCM COMMUNICATIONS_MOD_REG to clear SYNCHRONOUS_MODE_BIT.
         */
        timeout = jiffies + msecs_to_jiffies(SCM_TIMEOUT);
        while (scm_read8(COMMUNICATIONS_MOD_REG) & SYNCHRONOUS_MODE_BIT) {
            if (time_after(jiffies, timeout)) {
                mutex_unlock(&sp3s_mutex);
                printk(KERN_ERR "Timeout waiting for SYNCHRONOUS_MODE_BIT to clear in SCM\r\n");
                return -EIO;
            }

            scm_write8(COMMUNICATIONS_MOD_REG, val);
            cpu_relax();
        }
    }

    atomic_inc(&sp3s->open_count);

#ifdef DEBUG_SP3S
    sp3s->total_sent = 0;
    sp3s->tx_fifo_ovr_flo = 0;
    sp3s->tx_crc_err = 0;
    sp3s->tx_data_err = 0;
    sp3s->total_drop = 0;
    sp3s->rx_total = 0;
#endif

#ifdef USE_IRQ_MODE
    sp3s->rx_flag = 1;
#else
    schedule_delayed_work(&rx_work, msecs_to_jiffies(10));
#endif

    return 0;
}

#ifdef DEBUG_SP3S
/*
 * print_err_tx - Print error data for debugging
 */
void print_err_tx(void) {
    struct window *temp = head_er;
    u16 i = 0;

    while (temp) {
        printk("len = %d\r\n", temp->len);

        if (temp->data) {
            printk("\r\nData = ");
            for (i = 0; i < temp->len; i++)
                printk("%02X", temp->data[i]);
            printk("\r\n\nExpected crc = 0x%04X\r\n", crc16_ibm_sdlc(temp->data, (temp->len - 2)));

            if (temp->len >= 2) {
                printk("crc = 0x%02X%02X\r\n\n", temp->data[temp->len - 2], temp->data[temp->len - 1]);
            } else {
                printk("crc = N/A (len < 2)\r\n");
            }

            kfree(temp->data);
        } else {
            printk("Data = NULL\r\n");
        }

        head_er = temp->next;
        kfree(temp);
        temp = head_er;
    }
}
#endif

/*
 * sp3s_release - File operation: release device.
 */
static int sp3s_release(struct inode *inode, struct file *file)
{
    /* Release the mutex to allow others to open the device */
    if (atomic_dec_and_test(&sp3s->open_count)) {
        /* Last close, do cleanup if needed */
    }
    mutex_unlock(&sp3s_mutex);

#ifdef DEBUG_SP3S
    int i = 0, j = 0;
    printk("Total sp3s->total_sent = %d\r\n", sp3s->total_sent);
    printk("Total sp3s->tx_fifo_ovr_flo = %d\r\n", sp3s->tx_fifo_ovr_flo);
    printk("Total sp3s->tx_crc_err = %d\r\n", sp3s->tx_crc_err);
    printk("Total sp3s->tx_data_err = %d\r\n", sp3s->tx_data_err);

    for(i = 0; i < ERR_MAX && i < sp3s->total_drop; ++i) {
        printk("\r\n\n%d row:\r\n\n\n", (i + 1));
        for(j = 0; j < err_list[i].len; ++j){
           printk("%x", err_list[i].data[j]);
        }
    }

    if(head_er)
        print_err_tx();

    printk("Total drop_rx = %ld\r\n", sp3s->total_drop);
    printk("Total rx = %ld\r\n", sp3s->rx_total);
#endif

#ifndef USE_IRQ_MODE
   cancel_delayed_work_sync(&rx_work);
#endif

    return 0;
}

/*
 * set_config - Set UART configuration
 */
void set_config(atc_spxs_config_t *new_config) {
    int baud_reg_val;
    int new_baud = new_config->baud;

    switch (new_baud) {
        case 153600:
            baud_reg_val = SYNC_153600_BD_bit;
            break;
        case 614400:
            baud_reg_val = SYNC_614400_BD_bit;
            break;
        default:
            baud_reg_val = SYNC_153600_BD_bit;
            new_baud = 153600;
    }

    scm_write8(SP3_SYNC_BAUD_REG, baud_reg_val);
    sp3s->sp3sync_config.baud = new_baud;
}

/*
 * sp3_ioctl - File operation: handle IOCTL commands.
 */
static long sp3_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    atc_spxs_config_t new_config;

#ifdef DEBUG_SP3S
    printk(KERN_DEBUG "sp3s: ioctl cmd=%u (magic=%c, nr=%u, size=%u)\n", 
           cmd, _IOC_TYPE(cmd), _IOC_NR(cmd), _IOC_SIZE(cmd));
#endif

    if (cmd == ATC_SPXS_SET_ADDRESS) {
        if (copy_from_user(&sp3s->sdlc_address, (void __user *)arg, sizeof(u8))) {
            return -EFAULT;
        }
        printk("kernel setting address = %d\r\n", sp3s->sdlc_address);
        return 0;
    }
    else if (cmd == ATC_SPXS_GET_ADDRESS) {
        if (copy_to_user((void __user *)arg, &sp3s->sdlc_address, sizeof(u8))) {
            return -EFAULT;
        }
        printk("kernel getting address = %d\r\n", sp3s->sdlc_address);
        return 0;
    }
    else if (cmd == ATC_SPXS_WRITE_CONFIG) {
        if (copy_from_user(&new_config, (void __user *)arg, sizeof(atc_spxs_config_t))) {
            return -EFAULT;
        }
        if(new_config.baud != sp3s->sp3sync_config.baud) {
            set_config(&new_config);
        }
        return 0;
    }
    else if (cmd == ATC_SPXS_READ_CONFIG) {
        if (copy_to_user((void __user *)arg, &sp3s->sp3sync_config, sizeof(atc_spxs_config_t))) {
            return -EFAULT;
        }
        return 0;
    }
    else if (cmd == ATC_SPXS_GET_AVAIL) {
        /* Check available bytes in RX buffer */
        int avail = 0;
        unsigned long flags;
        
        spin_lock_irqsave(&rx_lock, flags);
        if (rx_head >= rx_tail) {
            avail = rx_head - rx_tail;
        } else {
            avail = RX_BUF_SIZE - rx_tail + rx_head;
        }
        spin_unlock_irqrestore(&rx_lock, flags);
        
        if (copy_to_user((void __user *)arg, &avail, sizeof(int))) {
            return -EFAULT;
        }
        return 0;
    }

    return -ENOTTY;
}


/* ------------------------------------------------------------------------- */
/*                      SPxs Kernel Level Interface (KAPI)                   */
/* ------------------------------------------------------------------------- */

struct sp3_kctx {
    struct sp3s_dev *dev;
};

/**
 * sp3_kapi_open() - Open SPx synchronous port for kernel clients
 * @channel: ATC_LKM_SP3S (3) for this driver
 *
 * Return: ERR_PTR(-errno) on error; valid context pointer on success.
 *         -EINTR if interrupted by signal (per requirements)
 *         -EBUSY if device is already open
 *         -ENODEV if device not found
 *         -ENOMEM if memory allocation fails
 */
static void *sp3_kapi_open(int channel)
{
    struct sp3_kctx *ctx;
    int ret;

    if (channel != ATC_LKM_SP3S)
        return ERR_PTR(-ENODEV);

    /* Use interruptible lock to allow signal interruption */
    ret = mutex_lock_interruptible(&sp3s_mutex);
    if (ret) {
        if (ret == -EINTR) {
            printk(KERN_WARNING "sp3s: kernel open interrupted by signal\n");
            return ERR_PTR(-EINTR);  /* Required by specifications */
        }
        printk(KERN_WARNING "sp3s: kernel device is busy\n");
        return ERR_PTR(-EBUSY);
    }

    if (!sp3s || !sp3s->is_init) {
        mutex_unlock(&sp3s_mutex);
        return ERR_PTR(-ENODEV);
    }

    atomic_inc(&sp3s->open_count);

    ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
    if (!ctx) {
        atomic_dec(&sp3s->open_count);
        mutex_unlock(&sp3s_mutex);
        return ERR_PTR(-ENOMEM);
    }

    ctx->dev = sp3s;

    return (void *)ctx;
}

/**
 * sp3_kapi_close() - Close port and restore defaults immediately
 * @context: pointer returned by sdlc_kernel_open()
 *
 * Return: 0 on success, negative errno on error.
 *         -EBADF: Incorrect context specified.
 *         -EINTR: The close operation was interrupted by a signal.
 */
static int sp3_kapi_close(void *context)
{
    struct sp3_kctx *ctx = (struct sp3_kctx *)context;
    unsigned long flags;
    int ret = 0;

    if (!ctx || !ctx->dev)
        return -EBADF;

    /* Check for pending signals */
    if (signal_pending(current))
        return -EINTR;

    /* This operation should be immediate and non-blocking */
    
    /* Flush KAPI RX FIFO */
    spin_lock_irqsave(&ctx->dev->kapi_rx_lock, flags);
    kfifo_reset(&ctx->dev->kapi_rx_fifo);
    spin_unlock_irqrestore(&ctx->dev->kapi_rx_lock, flags);

    /* Restore default address/control */
    ctx->dev->sdlc_address = DEFAULT_ADDRESS;
    ctx->dev->sdlc_control = ORX_CONTROL_FIELD;

    /* Restore default configuration */
    memcpy(&ctx->dev->sp3sync_config, &ctx->dev->default_config, 
           sizeof(ctx->dev->sp3sync_config));
    set_config(&ctx->dev->sp3sync_config);

    kfree(ctx);
    
    atomic_dec(&sp3s->open_count);

    /* Try to unlock, but don't block */
    if (mutex_is_locked(&sp3s_mutex)) {
        mutex_unlock(&sp3s_mutex);
    }

    return ret;
}

/**
 * sp3_kapi_read() - Non-blocking read of next available valid packet
 * @context: pointer returned by sdlc_kernel_open()
 * @buf: destination buffer (kernel memory)
 * @count: maximum bytes to copy from next packet
 *
 * Behavior:
 * - Returns 0 if no packet available (non-blocking).
 * - Copies up to @count bytes from the next packet payload (CRC removed).
 * - If @count is less than packet size, remainder is discarded.
 * - Returns -EINTR if interrupted by signal before any data was read.
 */
static ssize_t sp3_kapi_read(void *context, void *buf, ssize_t count)
{
    struct sp3_kctx *ctx = (struct sp3_kctx *)context;
    struct sp3_kapi_rx_hdr hdr;
    unsigned long flags;
    u8 *kbuf = (u8 *)buf;
    u16 pkt_len;
    u32 to_copy;
    u32 discard_len;
    u8 dummy;
    u32 chunk;
    int i;

    if (!ctx || !ctx->dev)
        return -EBADF;

    if (!kbuf)
        return -EFAULT;

    if (count <= 0)
        return -EINVAL;

    /* Check for signals if we would need to wait */
    if (kfifo_len(&ctx->dev->kapi_rx_fifo) < sizeof(hdr)) {
        if (signal_pending(current))
            return -EINTR;  /* Interrupted by signal before any data was read */
        return 0;  /* No data available, non-blocking */
    }

    spin_lock_irqsave(&ctx->dev->kapi_rx_lock, flags);

    if (kfifo_len(&ctx->dev->kapi_rx_fifo) < sizeof(hdr)) {
        spin_unlock_irqrestore(&ctx->dev->kapi_rx_lock, flags);
        return 0;
    }

    (void)kfifo_out(&ctx->dev->kapi_rx_fifo, &hdr, sizeof(hdr));
    pkt_len = hdr.len;

    if (pkt_len == 0U || pkt_len > (u16)SP3_KAPI_PKT_MAX) {
        kfifo_reset(&ctx->dev->kapi_rx_fifo);
        spin_unlock_irqrestore(&ctx->dev->kapi_rx_lock, flags);
        return -EFAULT;
    }

    to_copy = (u32)((count < (ssize_t)pkt_len) ? count : pkt_len);

    (void)kfifo_out(&ctx->dev->kapi_rx_fifo, kbuf, to_copy);

    if (to_copy < pkt_len) {
        /* Discard remainder as per requirement */
        discard_len = pkt_len - to_copy;
        while (discard_len > 0) {
            chunk = min(discard_len, (u32)1024);
            for (i = 0; i < chunk; i++) {
                (void)kfifo_out(&ctx->dev->kapi_rx_fifo, &dummy, 1);
            }
            discard_len -= chunk;
        }
    }

    spin_unlock_irqrestore(&ctx->dev->kapi_rx_lock, flags);

    return (ssize_t)to_copy;
}

/**
 * sp3_kapi_write() - Non-blocking write of a single packet
 * @context: pointer returned by sdlc_kernel_open()
 * @buf: source buffer (kernel memory)
 * @count: payload bytes to send as ONE packet
 *
 * Behavior:
 * - Adds CRC bytes as part of sending out the packet.
 * - Returns -EINVAL if payload exceeds max packet size (no packet sent).
 * - Returns -EAGAIN immediately if write would block (port busy).
 * - Returns -EINTR if interrupted by signal before any data was written.
 */
static ssize_t sp3_kapi_write(void *context, const void *buf, ssize_t count)
{
    struct sp3_kctx *ctx = (struct sp3_kctx *)context;
    const u8 *kbuf = (const u8 *)buf;
    u8 status;
    ssize_t ret;

    if (!ctx || !ctx->dev)
        return -EBADF;

    if (!kbuf)
        return -EFAULT;

    if (count <= 0)
        return -EINVAL;

    if (count > (ssize_t)(SP3_TX_FIFO_SIZE - 2U))
        return -EINVAL;  /* EINVAL as per requirement */

    /* Check for signals before proceeding */
    if (signal_pending(current))
        return -EINTR;  /* Interrupted by signal before any data was written */

    /* Non-blocking: if busy, return immediately with EAGAIN */
    status = scm_read8(SP3_THR_CTRL_REG);
    if (status & SP3_STATUS_BUSY_BIT)
        return -EAGAIN;  /* EAGAIN as per requirement */

    /* Try to acquire lock without blocking */
    if (!mutex_trylock(&ctx->dev->write_lock)) {
        return -EAGAIN;  /* Lock contention, would block */
    }

    /* Reuse existing TX formatting: [addr][ctrl][payload][crc] */
    ret = (ssize_t)tx_payload_with_crc(ctx->dev, kbuf, (u32)count);

    mutex_unlock(&ctx->dev->write_lock);

    return ret;
}

/**
 * sp3_kapi_ioctl() - Configure/control port for kernel clients
 * @context: pointer returned by sdlc_kernel_open()
 * @command: IOCTL command
 * @parameters: integer or pointer to kernel structure depending on command
 *
 * Supported:
 * - ATC_SPXS_WRITE_CONFIG: parameters = (atc_spxs_config_t *) kernel pointer
 * - ATC_SPXS_READ_CONFIG:  parameters = (atc_spxs_config_t *) kernel pointer
 * - ATC_SPXS_GET_AVAIL:    parameters = (int *) pointer to destination count
 */
static int sp3_kapi_ioctl(void *context, int command, unsigned long parameters)
{
    struct sp3_kctx *ctx = (struct sp3_kctx *)context;
    unsigned long flags;

    if (!ctx || !ctx->dev)
        return -EBADF;

    switch (command) {
    case ATC_SPXS_WRITE_CONFIG: {
        atc_spxs_config_t *cfg = (atc_spxs_config_t *)(uintptr_t)parameters;
        if (!cfg)
            return -EFAULT;

        /* Apply only supported fields; driver currently supports baud. */
        if (cfg->baud != ctx->dev->sp3sync_config.baud) {
            set_config(cfg);
        } else {
            memcpy(&ctx->dev->sp3sync_config, cfg, sizeof(*cfg));
        }
        return 0;
    }

    case ATC_SPXS_READ_CONFIG: {
        atc_spxs_config_t *cfg = (atc_spxs_config_t *)(uintptr_t)parameters;
        if (!cfg)
            return -EFAULT;

        memcpy(cfg, &ctx->dev->sp3sync_config, sizeof(*cfg));
        return 0;
    }

    case ATC_SPXS_GET_AVAIL: {
        int *bytes_available = (int *)(uintptr_t)parameters;
        struct sp3_kapi_rx_hdr hdr;

        if (!bytes_available)
            return -EFAULT;

        *bytes_available = 0;

        spin_lock_irqsave(&ctx->dev->kapi_rx_lock, flags);
        if (kfifo_len(&ctx->dev->kapi_rx_fifo) >= sizeof(hdr)) {
            (void)kfifo_out_peek(&ctx->dev->kapi_rx_fifo, &hdr, sizeof(hdr));
            *bytes_available = (int)hdr.len;
        }
        spin_unlock_irqrestore(&ctx->dev->kapi_rx_lock, flags);

        return 0;
    }

    default:
        return -ENOTTY;
    }
}

/**
 * sp3_kapi_rx_push() - Push a received payload packet into KAPI FIFO
 * @dev: device instance
 * @payload: pointer to payload bytes (no addr/ctrl/crc)
 * @payload_len: number of payload bytes
 *
 * This function is safe to call from IRQ/softirq context.
 */
static void sp3_kapi_rx_push(struct sp3s_dev *dev, const u8 *payload, u16 payload_len)
{
    unsigned long flags;
    struct sp3_kapi_rx_hdr hdr;

    if (!dev || !payload || payload_len == 0U || payload_len > (u16)SP3_KAPI_PKT_MAX)
        return;

    hdr.len = payload_len;

    spin_lock_irqsave(&dev->kapi_rx_lock, flags);

    if (kfifo_avail(&dev->kapi_rx_fifo) < (sizeof(hdr) + payload_len)) {
        /* Policy: drop newest packet if FIFO is full. */
        spin_unlock_irqrestore(&dev->kapi_rx_lock, flags);
        return;
    }

    (void)kfifo_in(&dev->kapi_rx_fifo, &hdr, sizeof(hdr));
    (void)kfifo_in(&dev->kapi_rx_fifo, payload, payload_len);

    spin_unlock_irqrestore(&dev->kapi_rx_lock, flags);

    wake_up_interruptible(&dev->rx_wq);
}

static const struct file_operations sp3s_fops = {
    .owner   = THIS_MODULE,
    .open    = sp3s_open,
    .release = sp3s_release,
    .read    = sp3s_read,
    .write   = sp3s_write,
    .unlocked_ioctl = sp3_ioctl,
};

/* SP3 ops */
static void *sp3_ops_open(void *priv) { (void)priv; return sp3_kapi_open(ATC_LKM_SP3S); }
static int sp3_ops_close(void *ctx) { return sp3_kapi_close(ctx); }
static ssize_t sp3_ops_read(void *ctx, void *buf, ssize_t cnt) { return sp3_kapi_read(ctx, buf, cnt); }
static ssize_t sp3_ops_write(void *ctx, const void *buf, ssize_t cnt) { return sp3_kapi_write(ctx, buf, cnt); }
static int sp3_ops_ioctl(void *ctx, int cmd, void *param) { 
    return sp3_kapi_ioctl(ctx, cmd, (unsigned long)(uintptr_t)param); 
}

static const struct spxs_kapi_ops sp3_kapi_ops = {
    .open  = sp3_ops_open,
    .close = sp3_ops_close,
    .read  = sp3_ops_read,
    .write = sp3_ops_write,
    .ioctl = sp3_ops_ioctl,
};

/*
 * sp3s_init - Module initialization function.
 */
static int __init sp3s_init(void)
{
    int ret;

#ifdef USE_IRQ_MODE
    int gpio = LPC43120_RXRDY_GPIO;
    int irq;
#endif

    sp3s = kzalloc(sizeof(*sp3s), GFP_KERNEL);
    if (!sp3s)
        return -ENOMEM;
    
    sp3s->is_init = 0;
    atomic_set(&sp3s->open_count, 0);

#ifdef DEBUG_SP3S
    sp3s->total_sent = 0;
    sp3s->tx_fifo_ovr_flo = 0;
    sp3s->tx_crc_err = 0;
    sp3s->tx_data_err = 0;
#endif

    mutex_init(&sp3s->write_lock);

    ret = alloc_chrdev_region(&devt, 0, 1, DEVICE_NAME);
    if (ret)
        goto err_free;

    cdev_init(&sp3s->cdev, &sp3s_fops);
    ret = cdev_add(&sp3s->cdev, devt, 1);
    if (ret)
        goto err_chrdev;

    sp3s_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(sp3s_class)) {
        ret = PTR_ERR(sp3s_class);
        goto err_cdev;
    }

    sp3s->dev = device_create(sp3s_class, NULL, devt, NULL, DEVICE_NAME);
    if (IS_ERR(sp3s->dev)) {
        ret = PTR_ERR(sp3s->dev);
        goto err_class;
    }

#ifdef USE_IRQ_MODE
    /* Initialize IRQ mode */
    if (gpio < 0) {
        pr_info("sp3s: LPC43120_RXRDY_GPIO not defined (negative) — using polling mode\n");
    } else {
        ret = gpio_request(gpio, "sp3s_rxrdy");
        if (ret) {
            pr_err("sp3s: gpio_request(%d) failed %d\n", gpio, ret);
            /* continue — failure falls back to polling */
        } else {
            ret = gpio_direction_input(gpio);
            if (ret) {
                pr_err("sp3s: gpio_direction_input(%d) failed %d\n", gpio, ret);
                gpio_free(gpio);
            } else {
                irq = gpio_to_irq(gpio);
                if (irq < 0) {
                    pr_err("sp3s: gpio_to_irq(%d) failed %d\n", gpio, irq);
                    gpio_free(gpio);
                } else {
                    ret = request_irq(irq, sp3s_rx_irq_handler,
                                      IRQF_TRIGGER_FALLING,
                                      "sp3s_rxrdy", sp3s);
                    if (ret) {
                        pr_err("sp3s: request_irq(%d) failed %d\n", irq, ret);
                        gpio_free(gpio);
                    } else {
                        sp3s->rx_gpio = gpio;
                        sp3s->rx_irq  = irq;
                        sp3s->use_rx_irq = 1;
                        pr_info("sp3s: RXRDY -> irq %d (gpio %d)\n", irq, gpio);
                    }
                }
            }
        }
    }
#endif

    spin_lock_init(&sp3s->lock);
    init_waitqueue_head(&sp3s->rx_wq);

    spin_lock_init(&sp3s->kapi_rx_lock);
    if (kfifo_alloc(&sp3s->kapi_rx_fifo, SP3_KAPI_RX_FIFO_BYTES, GFP_KERNEL)) {
        ret = -ENOMEM;
        goto err_device;
    }

#ifdef USE_IRQ_MODE
    init_waitqueue_head(&sp3s->rx_busy);
#endif
    spin_lock_init(&rx_lock);

#ifdef __TX_THREAD__
    tx_thread = kthread_create(sp3_tx_thread, NULL, "sp3s_tx_kthread");

    if (IS_ERR(tx_thread)) {
        ret = PTR_ERR(tx_thread);
        printk("Thread creation failed: %d\n", ret);
        goto err_kfifo;
    }
    wake_up_process(tx_thread);
#endif

#ifdef DEBUG_SP3S
    sp3s->tx_list = 0;
#endif
    
    sp3s->sdlc_address = DEFAULT_ADDRESS;
    sp3s->sdlc_control = ORX_CONTROL_FIELD;
    sp3s->sp3sync_config.baud = ATC_B153600;
    /* Save defaults for restoring on close (KAPI requirement). */
    memset(&sp3s->default_config, 0, sizeof(sp3s->default_config));
    sp3s->default_config.baud = ATC_B153600;

    set_config(&sp3s->sp3sync_config);  /* Default baud rate */

    spin_lock_init(&tx_list_lock);

    sp3s->is_init = 1;

#ifdef USE_IRQ_MODE
    pr_info("sp3s: char driver loaded (IRQ mode)\n");
#else
    pr_info("sp3s: char driver loaded (Polling mode)\n");
#endif
    
    
    /* Register SP3 channel (3) for global KAPI dispatcher */
    (void)spxs_kapi_register_channel(ATC_LKM_SP3S, &sp3_kapi_ops, NULL);

    return 0;

err_kfifo:
    kfifo_free(&sp3s->kapi_rx_fifo);
err_device:
    device_destroy(sp3s_class, devt);
err_class:
    class_destroy(sp3s_class);
err_cdev:
    cdev_del(&sp3s->cdev);
err_chrdev:
    unregister_chrdev_region(devt, 1);
err_free:
    kfree(sp3s);
    return ret;
}

/*
 * sp3s_exit - Module cleanup function.
 */
static void __exit sp3s_exit(void)
{
    (void)spxs_kapi_unregister_channel(ATC_LKM_SP3S);

#ifdef __TX_THREAD__
    if (tx_thread)
        kthread_stop(tx_thread);
#endif
        
    if(head != NULL)
        free_tx_node_mem();
        
    sp3s->is_init = 0;
    
#ifdef USE_IRQ_MODE
    if (sp3s && sp3s->use_rx_irq) {
        free_irq(sp3s->rx_irq, sp3s);
        gpio_free(sp3s->rx_gpio);
    }
#else
    cancel_delayed_work_sync(&rx_work);
#endif

    device_destroy(sp3s_class, devt);
    class_destroy(sp3s_class);
    cdev_del(&sp3s->cdev);
    unregister_chrdev_region(devt, 1);
    kfifo_free(&sp3s->kapi_rx_fifo);
    kfree(sp3s);
    pr_info("sp3s: char driver unloaded\n");
}

#if 0

/* =========================================================================
 * SPxs Kernel API Dispatcher (global exported symbols)
 * =========================================================================
 */

/* One wrapper context returned to LKMs */
#define SPXS_KAPI_CTX_MAGIC   (0x53505853U) /* 'SPXS' */

struct spxs_kapi_ctx_wrap {
    u32 magic;
    int channel;
    const struct spxs_kapi_ops *ops;
    void *inner;
};

/* Channel registry (index by channel number) */
static DEFINE_MUTEX(spxs_kapi_reg_lock);
static struct spxs_kapi_reg_entry {
    const struct spxs_kapi_ops *ops;
    void *priv;
} spxs_kapi_reg[SPXS_KAPI_MAX_CHANNEL + 1];

/**
 * spxs_kapi_register_channel() - Register a channel implementation
 * @channel: channel number (3/5/8)
 * @ops: operation table for that channel
 * @priv: private pointer (typically per-port struct)
 *
 * Return: 0 on success, negative errno on failure.
 */
int spxs_kapi_register_channel(int channel, const struct spxs_kapi_ops *ops, void *priv)
{
    if ((channel <= 0) || (channel > SPXS_KAPI_MAX_CHANNEL) || !ops)
        return -EINVAL;

    mutex_lock(&spxs_kapi_reg_lock);
    if (spxs_kapi_reg[channel].ops) {
        mutex_unlock(&spxs_kapi_reg_lock);
        return -EBUSY;
    }
    spxs_kapi_reg[channel].ops = ops;
    spxs_kapi_reg[channel].priv = priv;
    mutex_unlock(&spxs_kapi_reg_lock);

    return 0;
}
EXPORT_SYMBOL(spxs_kapi_register_channel);

/**
 * spxs_kapi_unregister_channel() - Unregister a channel implementation
 */
int spxs_kapi_unregister_channel(int channel)
{
    if ((channel <= 0) || (channel > SPXS_KAPI_MAX_CHANNEL))
        return -EINVAL;

    mutex_lock(&spxs_kapi_reg_lock);
    spxs_kapi_reg[channel].ops = NULL;
    spxs_kapi_reg[channel].priv = NULL;
    mutex_unlock(&spxs_kapi_reg_lock);

    return 0;
}
EXPORT_SYMBOL(spxs_kapi_unregister_channel);

/**
 * sdlc_kernel_open() - Customer API: open channel and return context
 *
 * Return: ERR_PTR(-errno) on error, or pointer to context on success.
 *         -ENODEV: Incorrect device specified, or device cannot be found.
 *         -EBUSY: The device is already open, or the device is busy.
 *         -EINTR: The open system call was interrupted by a signal.
 */
void *sdlc_kernel_open(int channel)
{
    const struct spxs_kapi_ops *ops;
    void *priv;
    void *inner;
    struct spxs_kapi_ctx_wrap *w;

    if ((channel <= 0) || (channel > SPXS_KAPI_MAX_CHANNEL))
        return ERR_PTR(-ENODEV);

    mutex_lock(&spxs_kapi_reg_lock);
    ops = spxs_kapi_reg[channel].ops;
    priv = spxs_kapi_reg[channel].priv;
    mutex_unlock(&spxs_kapi_reg_lock);

    if (!ops || !ops->open)
        return ERR_PTR(-ENODEV);

    inner = ops->open(priv);
    if (IS_ERR(inner)) {
        /* Propagate EINTR, EBUSY, ENODEV, ENOMEM from underlying open */
        return inner;  /* Return the original error pointer */
    }

    w = kzalloc(sizeof(*w), GFP_KERNEL);
    if (!w) {
        if (ops->close)
            (void)ops->close(inner);
        return ERR_PTR(-ENOMEM);
    }

    w->magic = SPXS_KAPI_CTX_MAGIC;
    w->channel = channel;
    w->ops = ops;
    w->inner = inner;

    return (void *)w;
}
EXPORT_SYMBOL(sdlc_kernel_open);

/**
 * sdlc_kernel_close() - Close port and restore defaults immediately
 * @context: pointer returned by sdlc_kernel_open()
 *
 * Return: 0 on success, negative errno on error.
 *         -EBADF: Incorrect file descriptor specified.
 *         -EINTR: The close system call was interrupted by a signal.
 */
int sdlc_kernel_close(void *context)
{
    struct spxs_kapi_ctx_wrap *w = (struct spxs_kapi_ctx_wrap *)context;
    int ret;

    if (!w || (w->magic != SPXS_KAPI_CTX_MAGIC) || !w->ops || !w->inner)
        return -EBADF;

    /* Check for pending signals */
    if (signal_pending(current))
        return -EINTR;

    /* The close operation should be immediate and not block */
    ret = w->ops->close ? w->ops->close(w->inner) : -ENODEV;
    
    w->magic = 0;
    kfree(w);
    
    return ret;
}
EXPORT_SYMBOL(sdlc_kernel_close);

/**
 * sdlc_kernel_read() - Non-blocking read of next available valid packet
 * @context: pointer returned by sdlc_kernel_open()
 * @buf: destination buffer (kernel memory)
 * @count: maximum bytes to copy from next packet
 *
 * Return: Number of bytes read on success, negative errno on error.
 *         -EBADF: Incorrect file descriptor specified.
 *         -EFAULT: There was a problem copying data into the buffer.
 *         -EINTR: The call was interrupted by a signal before any data was read.
 */
ssize_t sdlc_kernel_read(void *context, void *buf, ssize_t count)
{
    struct spxs_kapi_ctx_wrap *w = (struct spxs_kapi_ctx_wrap *)context;
    ssize_t ret;

    if (!w || (w->magic != SPXS_KAPI_CTX_MAGIC) || !w->ops || !w->inner)
        return -EBADF;

    if (!w->ops->read)
        return -ENODEV;

    if (!buf)
        return -EFAULT;

    if (count <= 0)
        return -EINVAL;

    ret = w->ops->read(w->inner, buf, count);
    
    /* Propagate EINTR from underlying read */
    return ret;
}
EXPORT_SYMBOL(sdlc_kernel_read);

/**
 * sdlc_kernel_write() - Non-blocking write of a single packet
 * @context: pointer returned by sdlc_kernel_open()
 * @buf: source buffer (kernel memory)
 * @count: payload bytes to send as ONE packet
 *
 * Return: Number of bytes written on success, negative errno on error.
 *         -EBADF: Incorrect file descriptor specified.
 *         -EFAULT: There was a problem copying data from the buffer.
 *         -EINTR: The call was interrupted by a signal before any data was written.
 *         -EINVAL: The maximum number of bytes to be written has been exceeded.
 *         -EAGAIN: Returned if the write operation would block.
 */
ssize_t sdlc_kernel_write(void *context, const void *buf, ssize_t count)
{
    struct spxs_kapi_ctx_wrap *w = (struct spxs_kapi_ctx_wrap *)context;

    if (!w || (w->magic != SPXS_KAPI_CTX_MAGIC) || !w->ops || !w->inner)
        return -EBADF;

    if (!w->ops->write)
        return -ENODEV;

    if (!buf)
        return -EFAULT;

    if (count <= 0)
        return -EINVAL;

    /* Check if packet exceeds maximum size */
    if (count > (SP3_TX_FIFO_SIZE - 2))
        return -EINVAL;  /* EINVAL as per requirement */

    return w->ops->write(w->inner, buf, count);
}
EXPORT_SYMBOL(sdlc_kernel_write);

/**
 * sdlc_kernel_ioctl() - IOCTL control for kernel clients
 * @context: pointer returned by sdlc_kernel_open()
 * @command: IOCTL command
 * @parameters: parameters for the command
 *
 * Return: 0 on success, negative errno on error.
 */
int sdlc_kernel_ioctl(void *context, int command, void *parameters)
{
    struct spxs_kapi_ctx_wrap *w = (struct spxs_kapi_ctx_wrap *)context;

    if (!w || (w->magic != SPXS_KAPI_CTX_MAGIC) || !w->ops || !w->inner)
        return -EBADF;

    if (!w->ops->ioctl)
        return -ENOTTY;

    return w->ops->ioctl(w->inner, command, parameters);
}
EXPORT_SYMBOL(sdlc_kernel_ioctl);

#endif
module_init(sp3s_init);
module_exit(sp3s_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sudhakar M");
MODULE_DESCRIPTION("SP3S UART Driver With SDLC U-FRAME Both ISR and polling");
MODULE_VERSION("3.0");

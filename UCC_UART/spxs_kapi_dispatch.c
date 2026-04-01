/*
 * spxs_kapi_dispatch.c - SPxs SDLC kernel-level API dispatcher
 */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/sched.h>
#include "spxs_kapi_dispatch.h"

#define SPXS_KAPI_MAX_CHANNEL   (8)
#define SPXS_KAPI_CTX_MAGIC     (0x53505853U) /* 'SPXS' */

struct spxs_kapi_reg_entry {
    const struct spxs_kapi_ops *ops;
    void *priv;
};

struct spxs_kapi_ctx_wrap {
    u32 magic;
    int channel;
    const struct spxs_kapi_ops *ops;
    void *inner;
};

static struct spxs_kapi_reg_entry spxs_kapi_reg[SPXS_KAPI_MAX_CHANNEL + 1];
static DEFINE_MUTEX(spxs_kapi_reg_lock);

/* Registration function */
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
    
    pr_info("spxs_kapi: Registered channel %d\n", channel);
    return 0;
}
EXPORT_SYMBOL(spxs_kapi_register_channel);

/* Unregistration function */
int spxs_kapi_unregister_channel(int channel)
{
    if ((channel <= 0) || (channel > SPXS_KAPI_MAX_CHANNEL))
        return -EINVAL;

    mutex_lock(&spxs_kapi_reg_lock);
    spxs_kapi_reg[channel].ops = NULL;
    spxs_kapi_reg[channel].priv = NULL;
    mutex_unlock(&spxs_kapi_reg_lock);

    pr_info("spxs_kapi: Unregistered channel %d\n", channel);
    return 0;
}
EXPORT_SYMBOL(spxs_kapi_unregister_channel);

/* Public API functions */
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
    if (IS_ERR(inner))
        return inner;

    w = kzalloc(sizeof(*w), GFP_KERNEL);
    if (!w) {
        if (ops->close)
            ops->close(inner);
        return ERR_PTR(-ENOMEM);
    }

    w->magic = SPXS_KAPI_CTX_MAGIC;
    w->channel = channel;
    w->ops = ops;
    w->inner = inner;

    return (void *)w;
}
EXPORT_SYMBOL(sdlc_kernel_open);

int sdlc_kernel_close(void *context)
{
    struct spxs_kapi_ctx_wrap *w = (struct spxs_kapi_ctx_wrap *)context;
    int ret;

    if (!w || (w->magic != SPXS_KAPI_CTX_MAGIC) || !w->ops || !w->inner)
        return -EBADF;

    if (signal_pending(current))
        return -EINTR;

    ret = w->ops->close ? w->ops->close(w->inner) : -ENODEV;
    
    w->magic = 0;
    kfree(w);
    
    return ret;
}
EXPORT_SYMBOL(sdlc_kernel_close);

ssize_t sdlc_kernel_read(void *context, void *buf, ssize_t count)
{
    struct spxs_kapi_ctx_wrap *w = (struct spxs_kapi_ctx_wrap *)context;

    if (!w || (w->magic != SPXS_KAPI_CTX_MAGIC) || !w->ops || !w->inner)
        return -EBADF;

    if (!w->ops->read)
        return -ENODEV;

    if (!buf || count <= 0)
        return -EINVAL;

    return w->ops->read(w->inner, buf, count);
}
EXPORT_SYMBOL(sdlc_kernel_read);

ssize_t sdlc_kernel_write(void *context, const void *buf, ssize_t count)
{
    struct spxs_kapi_ctx_wrap *w = (struct spxs_kapi_ctx_wrap *)context;

    if (!w || (w->magic != SPXS_KAPI_CTX_MAGIC) || !w->ops || !w->inner)
        return -EBADF;

    if (!w->ops->write)
        return -ENODEV;

    if (!buf || count <= 0)
        return -EINVAL;

    return w->ops->write(w->inner, buf, count);
}
EXPORT_SYMBOL(sdlc_kernel_write);

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

static int __init spxs_kapi_init(void)
{
    pr_info("spxs_kapi: SPxs Kernel API Dispatcher loaded\n");
    memset(spxs_kapi_reg, 0, sizeof(spxs_kapi_reg));
    return 0;
}

static void __exit spxs_kapi_exit(void)
{
    pr_info("spxs_kapi: SPxs Kernel API Dispatcher unloaded\n");
}

module_init(spxs_kapi_init);
module_exit(spxs_kapi_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SPxs Kernel API Dispatcher");
MODULE_VERSION("1.0");

// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2021, Linaro Limited
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/arm-smccc.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/tee_drv.h>
#include "optee_private.h"
#include "optee_smc.h"
#include "optee_rpc_cmd.h"

struct notif_entry {
	struct list_head link;
	struct completion c;
	u_int key;
};

static struct ssp_callback {
	int (*callback)(void);
	u32 notif_value;
} *ssp_data = NULL;

int register_callback(int (*callback)(void), u32 notif_value)
{
	if (ssp_data != NULL) {
		pr_err("Registering callback failed because it already exists.\n");
		return -EPERM;
	}

	ssp_data = kmalloc(sizeof(struct ssp_callback), GFP_KERNEL);
	ssp_data->callback = callback;
	ssp_data->notif_value = notif_value;

	return 0;
}
EXPORT_SYMBOL(register_callback);

void unregister_callback(void)
{
	ssp_data = NULL;
}
EXPORT_SYMBOL(unregister_callback);

static u32 get_async_notif_value(optee_invoke_fn *invoke_fn, bool *value_valid,
				 bool *value_pending)
{
	struct arm_smccc_res res;

	invoke_fn(OPTEE_SMC_GET_ASYNC_NOTIF_VALUE, 0, 0, 0, 0, 0, 0, 0, &res);

	if (res.a0)
		return 0;
	*value_valid = (res.a2 & OPTEE_SMC_ASYNC_NOTIF_VALUE_VALID);
	*value_pending = (res.a2 & OPTEE_SMC_ASYNC_NOTIF_VALUE_PENDING);
	return res.a1;
}

#define PTA_WATCHDOG_SETUP 0
#define PTA_WATCHDOG_UPDATE 1
u32 last_value = NULL;
static bool inited = false;
static struct tee_context *ctx = NULL;
static u32 sess_id = 0;

static int optee_ctx_match(struct tee_ioctl_version_data *ver, const void *data)
{
	if (ver->impl_id == TEE_IMPL_ID_OPTEE)
		return 1;
	else
		return 0;
}

void init_pta(void)
{
	const uuid_t pta_uuid =
		UUID_INIT(0xfc93fda1, 0x6bd2, 0x4e6a, 0x89, 0x3c, 0x12, 0x2f,
			  0x6c, 0x3c, 0x8e, 0x33);
	struct tee_ioctl_open_session_arg sess_arg;
	int rc;

	memset(&sess_arg, 0, sizeof(sess_arg));

	pr_info("Open context to watchdog\n");
	ctx = tee_client_open_context(NULL, optee_ctx_match, NULL, NULL);
	if (IS_ERR(ctx)) {
		pr_err("tee_client_open_context failed for watchdog, err: %x\n", ctx);
		return;
	}

	export_uuid(sess_arg.uuid, &pta_uuid);
	sess_arg.clnt_login = TEE_IOCTL_LOGIN_PUBLIC;
	sess_arg.num_params = 0;

	pr_info("Open session to watchdog\n");
	rc = tee_client_open_session(ctx, &sess_arg, NULL);
	if ((rc < 0) || (sess_arg.ret != TEEC_SUCCESS)) {
		/* Device enumeration pseudo TA not found */
		pr_err("tee_client_open_session failed, err: %x\n",
		       sess_arg.ret);
		return;
	}

	sess_id = sess_arg.session;
	inited = true;
}

static void update(void)
{
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
	int rc;

	if (!inited)
		init_pta();

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	inv_arg.func = PTA_WATCHDOG_UPDATE;
	inv_arg.session = sess_id;
	inv_arg.num_params = 4;

	memset(&param, 0, sizeof(param));

	rc = tee_client_invoke_func(ctx, &inv_arg, param);
	if ((rc < 0) || (inv_arg.ret != 0)) {
		pr_err("PTA_WATCHDOG_UPDATE invoke error: %x\n", inv_arg.ret);
	}
}

static irqreturn_t notif_irq_handler(int irq, void *dev_id)
{
	struct optee *optee = dev_id;
	bool do_bottom_half = false;
	bool value_valid;
	bool value_pending;

	do {
		last_value = get_async_notif_value(optee->invoke_fn, &value_valid,
					      &value_pending);
		if (!value_valid)
			break;

		if (last_value == OPTEE_SMC_ASYNC_NOTIF_VALUE_DO_BOTTOM_HALF ||
		    (ssp_data != NULL && ssp_data->notif_value == last_value))
			do_bottom_half = true;
		else
			optee_notif_send(optee, last_value);
	} while (value_pending);

	if (do_bottom_half || last_value == 10)
		return IRQ_WAKE_THREAD;
	return IRQ_HANDLED;
}

static irqreturn_t notif_irq_thread_fn(int irq, void *dev_id)
{
	struct optee *optee = dev_id;

	if (ssp_data != NULL && ssp_data->notif_value == last_value) {
		ssp_data->callback();
	} else if (last_value == 10) {
		update();
	} else {
		optee_do_bottom_half(optee->notif.ctx);
	}

	return IRQ_HANDLED;
}

static bool have_key(struct optee *optee, u_int key)
{
	struct notif_entry *entry;

	list_for_each_entry(entry, &optee->notif.db, link)
		if (entry->key == key)
			return true;

	return false;
}

int optee_notif_wait(struct optee *optee, u_int key)
{
	unsigned long flags;
	struct notif_entry *entry;
	int rc = 0;

	if (key > optee->notif.max_key)
		return -EINVAL;

	entry = kmalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;
	init_completion(&entry->c);
	entry->key = key;

	spin_lock_irqsave(&optee->notif.lock, flags);

	/*
	 * If the bit is already set it means that the key has already
	 * been posted and we must not wait.
	 */
	if (test_bit(key, optee->notif.bitmap)) {
		clear_bit(key, optee->notif.bitmap);
		goto out;
	}

	/*
	 * Check if someone is already waiting for this key. If there is
	 * it's a programming error.
	 */
	if (have_key(optee, key)) {
		rc = -EBUSY;
		goto out;
	}

	list_add_tail(&entry->link, &optee->notif.db);

	/*
	 * Unlock temporarily and wait for completion.
	 */
	spin_unlock_irqrestore(&optee->notif.lock, flags);
	wait_for_completion(&entry->c);
	spin_lock_irqsave(&optee->notif.lock, flags);

	list_del(&entry->link);
out:
	spin_unlock_irqrestore(&optee->notif.lock, flags);

	kfree(entry);

	return rc;
}

int optee_notif_send(struct optee *optee, u_int key)
{
	unsigned long flags;
	struct notif_entry *entry;

	if (key > optee->notif.max_key)
		return -EINVAL;

	spin_lock_irqsave(&optee->notif.lock, flags);

	list_for_each_entry(entry, &optee->notif.db, link)
		if (entry->key == key) {
			complete(&entry->c);
			goto out;
		}

	/* Only set the bit in case there where nobody waiting */
	set_bit(key, optee->notif.bitmap);
out:
	spin_unlock_irqrestore(&optee->notif.lock, flags);

	return 0;
}

int optee_notif_init(struct optee *optee, u_int max_key, u_int irq)
{
	struct tee_context *ctx;
	int rc;

	ctx = tee_dev_open_helper(optee->teedev);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	optee->notif.ctx = ctx;

	spin_lock_init(&optee->notif.lock);
	INIT_LIST_HEAD(&optee->notif.db);
	optee->notif.bitmap = bitmap_zalloc(max_key, GFP_KERNEL);
	if (!optee->notif.bitmap) {
		rc = -ENOMEM;
		goto err_put_ctx;
	}
	optee->notif.max_key = max_key;

	rc = request_threaded_irq(irq, notif_irq_handler, notif_irq_thread_fn,
				  0, "optee_notification", optee);
	if (rc)
		goto err_free_bitmap;

	optee->notif.irq = irq;

	return 0;

err_free_bitmap:
	kfree(optee->notif.bitmap);
err_put_ctx:
	tee_dev_ctx_put(optee->notif.ctx);

	return rc;
}

void optee_notif_uninit(struct optee *optee)
{
	if (optee->notif.ctx) {
		optee_stop_async_notif(optee->notif.ctx);
		if (optee->notif.irq) {
			free_irq(optee->notif.irq, optee);
			irq_dispose_mapping(optee->notif.irq);
		}

		/*
		 * The thread normally working with optee->notif.ctx was
		 * stopped with free_irq() above.
		 *
		 * Note we're not using teedev_close_context() or
		 * tee_client_close_context() since we have already called
		 * tee_device_put() while initializing to avoid a circular
		 * reference counting.
		 */
		tee_dev_ctx_put(optee->notif.ctx);
	}

	kfree(optee->notif.bitmap);
}

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/kobject.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include "mt-plat/mtk_thermal_monitor.h"

/*<20151217> RickLiu [PSM] Porting camera thermal alert for Power Save Module 20151217 BEGIN*/
#include <linux/pid.h>
#include <linux/signal.h>
#include <linux/sched.h>

extern struct proc_dir_entry * mtk_thermal_get_proc_drv_therm_dir_entry(void);
/*<20151217> RickLiu [PSM] 20151217 END*/

#define MAX_NUM_INSTANCE_MTK_COOLER_CAM  1

#if 1
#define mtk_cooler_cam_dprintk(fmt, args...) pr_debug("thermal/cooler/cam " fmt, ##args)
#else
#define mtk_cooler_cam_dprintk(fmt, args...)
#endif

static struct thermal_cooling_device *cl_cam_dev[MAX_NUM_INSTANCE_MTK_COOLER_CAM] = { 0 };
static unsigned long cl_cam_state[MAX_NUM_INSTANCE_MTK_COOLER_CAM] = { 0 };

static unsigned int _cl_cam;

#define MAX_LEN (256)

/*<20151217> RickLiu [PSM] Porting camera thermal alert for Power Save Module 20151217 BEGIN*/
static unsigned int tm_pid;
static unsigned int tm_input_pid;
static struct task_struct g_task;
static struct task_struct *pg_task = &g_task;
static int ov_cri_happened = 0;
static kuid_t uid = KUIDT_INIT(0);
static kgid_t gid = KGIDT_INIT(1000);
/*<20151217> RickLiu [PSM] 20151217 END*/

static ssize_t _cl_cam_write(struct file *filp, const char __user *buf, size_t len, loff_t *data)
{
	int ret = 0;
	char tmp[MAX_LEN] = { 0 };
	len = (len < (MAX_LEN - 1)) ? len : (MAX_LEN - 1);
	/* write data to the buffer */
	if (copy_from_user(tmp, buf, len))
		return -EFAULT;

	ret = kstrtouint(tmp, 10, &_cl_cam);
	if (ret)
		WARN_ON(1);

	mtk_cooler_cam_dprintk("%s %s = %d\n", __func__, tmp, _cl_cam);

	return len;
}

static int _cl_cam_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", _cl_cam);
	mtk_cooler_cam_dprintk("%s %d\n", __func__, _cl_cam);

	return 0;
}

static int _cl_cam_open(struct inode *inode, struct file *file)
{
	return single_open(file, _cl_cam_read, PDE_DATA(inode));
}

static const struct file_operations _cl_cam_fops = {
	.owner = THIS_MODULE,
	.open = _cl_cam_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = _cl_cam_write,
	.release = single_release,
};

/*<20151217> RickLiu [PSM] Porting camera thermal alert for Power Save Module 20151217 BEGIN*/
static int cam_send_signal(int level)
{
	int ret = 0;
	int thro = level;

	if (tm_input_pid == 0) {
		mtk_cooler_cam_dprintk("[%s] pid is empty\n", __func__);
		ret = -1;
	}

	mtk_cooler_cam_dprintk("[%s]pid is %d, %d, %d\n", __func__, tm_pid, tm_input_pid, thro);

	if (ret == 0 && tm_input_pid != tm_pid) {
		tm_pid = tm_input_pid;
		pg_task = get_pid_task(find_vpid(tm_pid), PIDTYPE_PID);
	}

	if (ret == 0 && pg_task) {
		siginfo_t info;
		info.si_signo = SIGIO;
		info.si_errno = 0; // for md ul throttling
		info.si_code = thro;
		info.si_addr = NULL;
		ret = send_sig_info(SIGIO, &info, pg_task);
	}

	if (ret != 0) 
	    mtk_cooler_cam_dprintk("[%s] ret=%d\n", __func__, ret);

	return ret;
}
/*<20151217> RickLiu [PSM] 20151217 END*/

static int mtk_cl_cam_get_max_state(struct thermal_cooling_device *cdev, unsigned long *state)
{
	*state = 1;
	/* mtk_cooler_cam_dprintk("mtk_cl_cam_get_max_state() %s %d\n", cdev->type, *state); */
	return 0;
}

static int mtk_cl_cam_get_cur_state(struct thermal_cooling_device *cdev, unsigned long *state)
{
	*state = *((unsigned long *)cdev->devdata);
	/* mtk_cooler_cam_dprintk("mtk_cl_cam_get_cur_state() %s %d\n", cdev->type, *state); */
	return 0;
}

static int mtk_cl_cam_set_cur_state(struct thermal_cooling_device *cdev, unsigned long state)
{
	/* mtk_cooler_cam_dprintk("mtk_cl_cam_set_cur_state() %s %d\n", cdev->type, state); */

	*((unsigned long *)cdev->devdata) = state;

	if (1 == state) {
		_cl_cam = 1;
	/*<20151217> RickLiu [PSM] Porting camera thermal alert for Power Save Module 20151217 BEGIN*/
		if (0 == ov_cri_happened) {
			cam_send_signal(85);
			ov_cri_happened=1;
		}
	/*<20151217> RickLiu [PSM] 20151217 END*/
	}else{
		_cl_cam = 0;
	/*<20151217> RickLiu [PSM] Porting camera thermal alert for Power Save Module 20151217 BEGIN*/
		if (1 == ov_cri_happened) {
			cam_send_signal(76);
			ov_cri_happened=0;
		}
	/*<20151217> RickLiu [PSM] 20151217 END*/
	}
	return 0;
}

/*<20151217> RickLiu [PSM] Porting camera thermal alert for Power Save Module 20151217 BEGIN*/
static ssize_t _mtk_cl_cam_pid_write(struct file *filp, const char __user *buf, size_t len,
				    loff_t *data)
{
	int ret = 0;
	char tmp[MAX_LEN] = { 0 };

        // <<< 2016/04/20-youchihwang, Security Incident. SSIMS00000914
        // Original code could enable a local malicious application to execute arbitrary code within the context of the kernel
        // Modified code add additional bounds checks.
        len = min(len, (size_t)(MAX_LEN-1));
        // >>> 2016/04/20-youchihwang, Security Incident. SSIMS00000914

	// write data to the buffer 
	if (copy_from_user(tmp, buf, len)) {
		return -EFAULT;
	}

	ret = kstrtouint(tmp, 10, &tm_input_pid);
	if (ret)
		WARN_ON(1);

	mtk_cooler_cam_dprintk("%s %s = %d\n", __func__, tmp, tm_input_pid);

	return len;
}

static int _mtk_cl_cam_pid_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", tm_input_pid);
	mtk_cooler_cam_dprintk("%s %d\n", __func__, tm_input_pid);

	return 0;
}


static int _mtk_cl_cam_pid_open(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	return single_open(file, _mtk_cl_cam_pid_read, PDE_DATA(inode));
#else
	return single_open(file, _mtk_cl_cam_pid_read, PDE(inode)->data);
#endif
}

static const struct file_operations _cl_cam_pid_fops = {
	.owner = THIS_MODULE,
	.open = _mtk_cl_cam_pid_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = _mtk_cl_cam_pid_write,
	.release = single_release,
};
/*<20151217> RickLiu [PSM] 20151217 END*/

/* bind fan callbacks to fan device */
static struct thermal_cooling_device_ops mtk_cl_cam_ops = {
	.get_max_state = mtk_cl_cam_get_max_state,
	.get_cur_state = mtk_cl_cam_get_cur_state,
	.set_cur_state = mtk_cl_cam_set_cur_state,
};

static int mtk_cooler_cam_register_ltf(void)
{
	int i;

	mtk_cooler_cam_dprintk("register ltf\n");

	for (i = MAX_NUM_INSTANCE_MTK_COOLER_CAM; i-- > 0;) {
		char temp[20] = { 0 };

		sprintf(temp, "mtk-cl-cam%02d", i);
		cl_cam_dev[i] = mtk_thermal_cooling_device_register(temp,
								    (void *)&cl_cam_state[i],
								    &mtk_cl_cam_ops);
	}

	return 0;
}

static void mtk_cooler_cam_unregister_ltf(void)
{
	int i;

	mtk_cooler_cam_dprintk("unregister ltf\n");

	for (i = MAX_NUM_INSTANCE_MTK_COOLER_CAM; i-- > 0;) {
		if (cl_cam_dev[i]) {
			mtk_thermal_cooling_device_unregister(cl_cam_dev[i]);
			cl_cam_dev[i] = NULL;
			cl_cam_state[i] = 0;
		}
	}
}


static int __init mtk_cooler_cam_init(void)
{
	int err = 0;
	int i;

	for (i = MAX_NUM_INSTANCE_MTK_COOLER_CAM; i-- > 0;) {
		cl_cam_dev[i] = NULL;
		cl_cam_state[i] = 0;
	}

	mtk_cooler_cam_dprintk("init\n");

	{
		struct proc_dir_entry *entry;
		
		/*<20151217> RickLiu [PSM] Porting camera thermal alert for Power Save Module 20151217 BEGIN*/
		struct proc_dir_entry *dir_entry = NULL;
		dir_entry = mtk_thermal_get_proc_drv_therm_dir_entry();
		/*<20151217> RickLiu [PSM] 20151217 END*/

#if 0
		entry = create_proc_entry("driver/cl_cam", S_IRUGO | S_IWUSR, NULL);
		if (NULL != entry) {
			entry->read_proc = _cl_cam_read;
			entry->write_proc = _cl_cam_write;
		}
#endif
		/*<20151217> RickLiu [PSM] Porting camera thermal alert for Power Save Module 20150106 BEGIN*/
		//create proc to get daemon pid
		entry =
		    proc_create("cl_cam_pid", S_IRUGO | S_IWUSR | S_IWGRP, dir_entry, &_cl_cam_pid_fops);
		if (!entry) {
			mtk_cooler_cam_dprintk("%s cl_cam_pid creation failed\n", __func__);
		} else {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
			proc_set_user(entry, uid, gid);
#else
			entry->gid = 1000;
#endif
		}
		/*<20151217> RickLiu [PSM] 20150106 END*/
		entry = proc_create("driver/cl_cam", S_IRUGO | S_IWUSR, NULL, &_cl_cam_fops);
		if (!entry)
			mtk_cooler_cam_dprintk("%s driver/cl_cam creation failed\n", __func__);
	}

	err = mtk_cooler_cam_register_ltf();
	if (err)
		goto err_unreg;

	return 0;

 err_unreg:
	mtk_cooler_cam_unregister_ltf();
	return err;
}

static void __exit mtk_cooler_cam_exit(void)
{
	mtk_cooler_cam_dprintk("exit\n");

	mtk_cooler_cam_unregister_ltf();
}
module_init(mtk_cooler_cam_init);
module_exit(mtk_cooler_cam_exit);

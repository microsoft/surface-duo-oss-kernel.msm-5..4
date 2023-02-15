#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/sched/debug.h>

#ifndef CONFIG_QGKI
struct self_watchdog {
	struct task_struct      *tsk;
	struct timer_list       timer;
	void*                   data;
};

static DECLARE_RWSEM(rw_sem);
static int wd_enabled = 0;
struct self_watchdog booting_wd;


static void self_wd_handler(struct timer_list *t)
{
	struct self_watchdog *wd = from_timer(wd, t, timer);
	if (wd_enabled == 0) {
		pr_err("[boot_wd] watchdog terminated\n");
	} else {
		pr_emerg("[boot_wd] **** booting timeout ****\n");
		BUG();
	}
}

static void self_wd_set(struct self_watchdog *wd)
{
	struct timer_list *timer = &wd->timer;

	timer_setup(timer, self_wd_handler, 0);
	mod_timer(timer, jiffies + HZ * 60);
	pr_err("[boot_wd] watchdog deadline set at 60s later!\n");
}

int file_read(char* output)
{
	int ret;

	down_read(&rw_sem);
	ret = sprintf(output, "%d\n", wd_enabled);
	if (ret < 0)
		pr_err("[boot_wd] get wd_enabled failed!\n");
	up_read(&rw_sem);

	return ret;
}

int file_write(const char* input)
{
        int ret;

	down_write(&rw_sem);
	ret = sscanf(input, "%d", &wd_enabled);
	if (ret < 0)
		pr_err("[boot_wd] set wd_enabled failed!\n");
	up_write(&rw_sem);


        return ret;
}


#define FILE_NODE(_name, _mode) \
static ssize_t _name##_show(struct kobject *kobj,\
                            struct kobj_attribute *attr,\
                            char *buf)\
{\
        return file_read(buf);\
}\
static ssize_t _name##_store(struct kobject *kobj,\
                             struct kobj_attribute *attr,\
                             const char *buf, size_t count)\
{\
        int ret = file_write(buf);\
        return (ret < 0) ? ret : count;\
}\
struct kobj_attribute _name##_attr = __ATTR(_name, _mode, _name##_show, _name##_store);

#define THIS_ATTR(_name) &(_name##_attr.attr)

FILE_NODE(watchdog_enabled, 0664);

static struct attribute *attrs[] = {
        THIS_ATTR(watchdog_enabled),
        NULL,
};

static struct attribute_group attr_group = {.attrs = attrs,};
static struct kobject *this_kobj;

static int create_folder(const char* folder_name) {

	int retval = 0;

	this_kobj = kobject_create_and_add(folder_name, fs_kobj);
	if (unlikely(!this_kobj)) {
		pr_err("[boot_wd] class_compat_register failed!\n");
		retval = -ENOMEM;
		goto error;
	}

	retval = sysfs_create_group(this_kobj, &attr_group);
	if (unlikely(retval)) {
		pr_err("[boot_wd] sysfs_create_group failed!\n");
		goto reg_error;
	}
	return retval;

reg_error:
        kobject_put(this_kobj);
error:
        return retval;
}

static int __init booting_watchdog_init(void)
{
	int retval = 0;

	retval = create_folder("boot_wd");
	if (retval)
		return -1;

	wd_enabled = 1;
	self_wd_set(&booting_wd);

	return 0;
}

static void __exit booting_watchdog_exit(void)
{
	kobject_put(this_kobj);
}

#else /* CONFIG_QGKI */
static int __init booting_watchdog_init(void)
{
	return 0;
}
static void __exit booting_watchdog_exit(void)
{
	return;
}

#endif



module_init(booting_watchdog_init);
module_exit(booting_watchdog_exit);

MODULE_DESCRIPTION("boot Watchdog");
MODULE_AUTHOR("David Shih");
MODULE_LICENSE("GPL");

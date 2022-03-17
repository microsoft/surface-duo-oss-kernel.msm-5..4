// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/kthread.h>
#include <linux/memblock.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

static void *hob_va;
static struct kobject *sysfs_dir;

static ssize_t hob_mem_read(struct file *fp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t offset, size_t count)
{
	memcpy(buf, hob_va + offset, count);

	return count;
}

static BIN_ATTR_RO(hob_mem, 0);

static int hob_create_sysfs(void)
{
	int ret;

	sysfs_dir = kobject_create_and_add("hob_mem", kernel_kobj);

	if (!sysfs_dir) {
		pr_err("%s: sysfs create and add failed\n", __func__);
		return -ENOMEM;
	}

	ret = sysfs_create_bin_file(sysfs_dir, &bin_attr_hob_mem);
	if (ret) {
		pr_err("%s: failed to create sysfs: %d\n", __func__, ret);
		goto kobj_fail;
	}

	return 0;

kobj_fail:
	kobject_put(sysfs_dir);

	return ret;
}

static int find_hob_resource(struct resource *res)
{
	struct device_node *parent, *node;
	int ret = 0;

	parent = of_find_node_by_path("/reserved-memory");
	if (!parent)
		return -ENODEV;

	node = of_find_node_by_name(parent, "xbl_apps_hob_mem");
	if (!node) {
		ret = -ENODEV;
		goto node_fail;
	}

	if (of_address_to_resource(node, 0, res)) {
		ret = -ENXIO;
		goto res_fail;
	}

res_fail:
	of_node_put(node);
node_fail:
	of_node_put(parent);

	return ret;
}

static int hob_kthread(void *arg)
{
	int ret;
	struct resource res;
	void *virt;
	resource_size_t size;

	ret = find_hob_resource(&res);

	if (ret) {
		pr_err("%s: failed to find resource: %d\n", __func__, ret);
		return ret;
	}

	size = resource_size(&res);
	bin_attr_hob_mem.size = size;

	virt = memremap(res.start, size, MEMREMAP_WB);
	if (!virt) {
		pr_err("%s: failed to remap HOB region\n", __func__);
		return -ENOMEM;
	}

	hob_va = kzalloc(size, GFP_KERNEL);
	if (!hob_va) {
		ret = -ENOMEM;
		pr_err("%s: failed to allocate memory for HOB region\n", __func__);
		goto oom_fail;
	}

	memcpy(hob_va, virt, size);
	memunmap(virt);

	if (!hob_create_sysfs())
		return 0;

	kfree(hob_va);
oom_fail:
	memunmap(virt);

	return ret;
}

static int __init hob_init(void)
{
	struct task_struct *hob_task =
		kthread_run(hob_kthread, NULL, "hob_mem");

	if (PTR_ERR_OR_ZERO(hob_task))
		return PTR_ERR(hob_task);
	else
		return 0;
}
subsys_initcall(hob_init);

static void __exit hob_exit(void)
{
	if (sysfs_dir)
		sysfs_remove_bin_file(sysfs_dir, &bin_attr_hob_mem);
	if (hob_va)
		kfree(hob_va);
}
module_exit(hob_exit);

MODULE_DESCRIPTION("Export HOB memory region");
MODULE_LICENSE("GPL v2");

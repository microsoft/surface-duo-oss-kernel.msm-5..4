// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014-2015, 2017-2019, The Linux Foundation.
 * All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spmi.h>
#include <linux/regmap.h>
#include <linux/of_platform.h>
// MSCHANGE start
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
// MSCHANGE end

#define PMIC_REV2		0x101
#define PMIC_REV3		0x102
#define PMIC_REV4		0x103
#define PMIC_TYPE		0x104
#define PMIC_SUBTYPE		0x105

#define PMIC_TYPE_VALUE		0x51

#define COMMON_SUBTYPE		0x00
#define PM8941_SUBTYPE		0x01
#define PM8841_SUBTYPE		0x02
#define PM8019_SUBTYPE		0x03
#define PM8226_SUBTYPE		0x04
#define PM8110_SUBTYPE		0x05
#define PMA8084_SUBTYPE		0x06
#define PMI8962_SUBTYPE		0x07
#define PMD9635_SUBTYPE		0x08
#define PM8994_SUBTYPE		0x09
#define PMI8994_SUBTYPE		0x0a
#define PM8916_SUBTYPE		0x0b
#define PM8004_SUBTYPE		0x0c
#define PM8909_SUBTYPE		0x0d
#define PM8998_SUBTYPE		0x14
#define PMI8998_SUBTYPE		0x15
#define PM8005_SUBTYPE		0x18

// MSCHANGE start

/* SDAM2 register offsets */
#define SDAM2_REG_OFFSET		0x71A0

/* SDAM2 register bit masks */
#define SDAM2_REG_FDUMP_MASK		0X1

// MSCHANGE end

static const struct of_device_id pmic_spmi_id_table[] = {
	{ .compatible = "qcom,spmi-pmic", .data = (void *)COMMON_SUBTYPE },
	{ .compatible = "qcom,pm8941",    .data = (void *)PM8941_SUBTYPE },
	{ .compatible = "qcom,pm8841",    .data = (void *)PM8841_SUBTYPE },
	{ .compatible = "qcom,pm8019",    .data = (void *)PM8019_SUBTYPE },
	{ .compatible = "qcom,pm8226",    .data = (void *)PM8226_SUBTYPE },
	{ .compatible = "qcom,pm8110",    .data = (void *)PM8110_SUBTYPE },
	{ .compatible = "qcom,pma8084",   .data = (void *)PMA8084_SUBTYPE },
	{ .compatible = "qcom,pmi8962",   .data = (void *)PMI8962_SUBTYPE },
	{ .compatible = "qcom,pmd9635",   .data = (void *)PMD9635_SUBTYPE },
	{ .compatible = "qcom,pm8994",    .data = (void *)PM8994_SUBTYPE },
	{ .compatible = "qcom,pmi8994",   .data = (void *)PMI8994_SUBTYPE },
	{ .compatible = "qcom,pm8916",    .data = (void *)PM8916_SUBTYPE },
	{ .compatible = "qcom,pm8004",    .data = (void *)PM8004_SUBTYPE },
	{ .compatible = "qcom,pm8909",    .data = (void *)PM8909_SUBTYPE },
	{ .compatible = "qcom,pm8998",    .data = (void *)PM8998_SUBTYPE },
	{ .compatible = "qcom,pmi8998",   .data = (void *)PMI8998_SUBTYPE },
	{ .compatible = "qcom,pm8005",    .data = (void *)PM8005_SUBTYPE },
	{ }
};
// MSCHANGE start
#define REG_BLOCK_READ_SIZE   16
struct driver_data {
	struct regmap *map;
	u32 *reg_arr;
	unsigned int len;
	int sdam2_value; // this is used for sysfs of sdm2(part of pmk8350)
	struct kobject sdam2_kobj;
};

// MSCHANGE end
static void pmic_spmi_show_revid(struct regmap *map, struct device *dev)
{
	unsigned int rev2, minor, major, type, subtype;
	const char *name = "unknown";
	int ret, i;

	ret = regmap_read(map, PMIC_TYPE, &type);
	if (ret < 0)
		return;

	if (type != PMIC_TYPE_VALUE)
		return;

	ret = regmap_read(map, PMIC_SUBTYPE, &subtype);
	if (ret < 0)
		return;

	for (i = 0; i < ARRAY_SIZE(pmic_spmi_id_table); i++) {
		if (subtype == (unsigned long)pmic_spmi_id_table[i].data)
			break;
	}

	if (i != ARRAY_SIZE(pmic_spmi_id_table))
		name = pmic_spmi_id_table[i].compatible;

	ret = regmap_read(map, PMIC_REV2, &rev2);
	if (ret < 0)
		return;

	ret = regmap_read(map, PMIC_REV3, &minor);
	if (ret < 0)
		return;

	ret = regmap_read(map, PMIC_REV4, &major);
	if (ret < 0)
		return;

	/*
	 * In early versions of PM8941 and PM8226, the major revision number
	 * started incrementing from 0 (eg 0 = v1.0, 1 = v2.0).
	 * Increment the major revision number here if the chip is an early
	 * version of PM8941 or PM8226.
	 */
	if ((subtype == PM8941_SUBTYPE || subtype == PM8226_SUBTYPE) &&
	    major < 0x02)
		major++;

	if (subtype == PM8110_SUBTYPE)
		minor = rev2;

	dev_dbg(dev, "%x: %s v%d.%d\n", subtype, name, major, minor);
}

static const struct regmap_config spmi_regmap_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0xffff,
	.fast_io	= true,
};

static const struct regmap_config spmi_regmap_can_sleep_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0xffff,
	.fast_io	= false,
};

// MSCHANGE start
static int regdump_proc_show(struct seq_file *m, void *v)
{
	struct driver_data *data = m->private;
	int i = 0, j, k, ret = 0;
	unsigned int start_reg, count_reg;
	u8 value[REG_BLOCK_READ_SIZE];

	for(i = 0; i < data->len; i = i + 2) {
		start_reg = data->reg_arr[i];
		count_reg = data->reg_arr[i + 1];
		count_reg = count_reg % REG_BLOCK_READ_SIZE == 0 ? count_reg : count_reg + REG_BLOCK_READ_SIZE - (count_reg % REG_BLOCK_READ_SIZE);
		for(j = 0; j < count_reg; j = j + REG_BLOCK_READ_SIZE) {
			memset(value, 0, sizeof(value));
			ret = regmap_bulk_read(data->map, start_reg + j, &value[0], sizeof(value));
			if (ret == 0) {
				for(k = 0; k < sizeof(value); k++)
					seq_printf(m , "%.4x: %.2x ",(start_reg + j + k), value[k]);
				seq_printf(m , "\n");
			}
		}
	}
	return 0;
}

static int regdump_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, regdump_proc_show, PDE_DATA(inode));
}


static const struct file_operations regdump_proc_fops = {
	.open		= regdump_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static ssize_t sdam2_default_sysfs_show(struct kobject *kobj, struct attribute *attr,
			     char *buf)
{
	struct kobj_attribute *kattr;
	ssize_t ret = -EIO;

	kattr = container_of(attr, struct kobj_attribute, attr);
	if (kattr->show)
		ret = kattr->show(kobj, kattr, buf);
	return ret;
}

static ssize_t sdam2_default_sysfs_store(struct kobject *kobj, struct attribute *attr, const char* buf, size_t n)
{
	struct kobj_attribute *kattr;
	ssize_t ret = -EIO;

	kattr = container_of(attr, struct kobj_attribute, attr);
	if(kattr->store)
		ret = kattr->store(kobj, kattr, buf, n);
	return ret;
}

static ssize_t sdam2_sysfs_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct driver_data *driver_data = container_of(kobj, struct driver_data, sdam2_kobj);
	return sprintf(buf, "%d\n", driver_data->sdam2_value);
}

static ssize_t sdam2_sysfs_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	struct driver_data *driver_data = container_of(kobj, struct driver_data, sdam2_kobj);
    sscanf(buf, "%d", &(driver_data->sdam2_value));

	if (driver_data->sdam2_value == 1)
	{
		ret = regmap_write_bits(driver_data->map, SDAM2_REG_OFFSET, SDAM2_REG_FDUMP_MASK, 0x01);

		if (ret)
			pr_err("Not able to set SDAM2 register bit-0 properly\n");
	}
	else if (driver_data->sdam2_value == 0)
	{
		ret = regmap_write_bits(driver_data->map, SDAM2_REG_OFFSET, SDAM2_REG_FDUMP_MASK, 0x00);

		if (ret)
			pr_err("Not able to set SDAM2 register bit-0 properly\n");
	}
	else {
		pr_warn("SDAM2 register bit-0 is not changed\n");
	}

    return count;
}

static struct kobj_attribute sdm2_attr = __ATTR(sdam2_value, 0664, sdam2_sysfs_show, sdam2_sysfs_store);

static const struct sysfs_ops sdam2_sysfs_ops = {
	.show	= sdam2_default_sysfs_show,
	.store = sdam2_default_sysfs_store,
};

static struct kobj_type sdam2_ktype = {
	.sysfs_ops = &sdam2_sysfs_ops,
};

// MSCHANGE end

static int pmic_spmi_probe(struct spmi_device *sdev)
{
	struct device_node *root = sdev->dev.of_node;
	struct regmap *regmap;
	struct driver_data *driver_data;
	const char *node_name;
	int ret = 0,reg_arr_len;
	int retval;

	if (of_property_read_bool(root, "qcom,can-sleep"))
		regmap = devm_regmap_init_spmi_ext(sdev,
						&spmi_regmap_can_sleep_config);
	else
		regmap = devm_regmap_init_spmi_ext(sdev, &spmi_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	/* Only the first slave id for a PMIC contains this information */
	if (sdev->usid % 2 == 0)
		pmic_spmi_show_revid(regmap, &sdev->dev);

	ret = devm_of_platform_populate(&sdev->dev);

// MSCHANGE start
	if (ret != 0)
		return ret;

	driver_data = devm_kzalloc(&sdev->dev, sizeof(*driver_data), GFP_KERNEL);

	if (!driver_data) {
		dev_err(&sdev->dev, "did not get memory for driver_data\n");
		return -ENOMEM;
	}
	driver_data->map = regmap;

	if (of_property_read_bool(root, "qcom,create-regdump-node")) {

		ret = of_property_read_string(root, "qcom,regdump-node-name", &node_name);
		if (ret < 0) {
			dev_err(&sdev->dev, "did not find qcom,regdump-node-name properity in dtsi, ret = %d\n", ret);
			goto err_free_driver_data;
		}

		reg_arr_len = of_property_count_u32_elems(root,
							 "qcom,reg-array");
		if (reg_arr_len < 0) {
			dev_err(&sdev->dev, "error in qcom,reg-array properity properity, reg_arr_len = %d \n", reg_arr_len);
			goto err_free_driver_data;
		}

		driver_data->len = reg_arr_len;

		driver_data->reg_arr = devm_kzalloc(&sdev->dev, sizeof(u32) * reg_arr_len, GFP_KERNEL);

		if (!driver_data->reg_arr) {
			ret = -ENOMEM;
			goto err_free_driver_data;
		}

		ret = of_property_read_u32_array(root, "qcom,reg-array",
						driver_data->reg_arr, reg_arr_len);

		if (ret < 0) {
			dev_err(&sdev->dev, "did not find qcom,reg-array properity in dtsi, ret = %d\n", ret);
			goto err_free_reg_arr;
		}
		proc_create_data(node_name, S_IRUGO, NULL,
				&regdump_proc_fops, driver_data);

	}

	if (sdev->usid == 0) {
		retval = kobject_init_and_add(&driver_data->sdam2_kobj, &sdam2_ktype, kernel_kobj, "sdam2_sysfs");

		if (retval)
	{
		pr_err("failed to create sdam2_sysfs parent obj");
		kobject_put(&driver_data->sdam2_kobj); //decrease reference count
	}

	if (sysfs_create_file(&driver_data->sdam2_kobj, &sdm2_attr.attr)) {
		pr_err("cannot create sysfs for SDAM2 register bit-0\n");
			kobject_put(&driver_data->sdam2_kobj);
	}
	}

	return 0;

	err_free_reg_arr:
			devm_kfree(&sdev->dev, driver_data->reg_arr);
	err_free_driver_data:
			devm_kfree(&sdev->dev, driver_data);

	return ret;
// MSCHANGE end

}

MODULE_DEVICE_TABLE(of, pmic_spmi_id_table);

static void pmic_spmi_remove(struct spmi_device *sdev) {}

static struct spmi_driver pmic_spmi_driver = {
	.probe = pmic_spmi_probe,
	.remove = pmic_spmi_remove,
	.driver = {
		.name = "pmic-spmi",
		.of_match_table = pmic_spmi_id_table,
	},
};

static int __init pmic_spmi_init(void)
{
	return spmi_driver_register(&pmic_spmi_driver);
}
arch_initcall(pmic_spmi_init);

static void __exit pmic_spmi_exit(void)
{
	spmi_driver_unregister(&pmic_spmi_driver);
}
module_exit(pmic_spmi_exit);

MODULE_DESCRIPTION("Qualcomm SPMI PMIC driver");
MODULE_ALIAS("spmi:spmi-pmic");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Josh Cartwright <joshc@codeaurora.org>");
MODULE_AUTHOR("Stanimir Varbanov <svarbanov@mm-sol.com>");

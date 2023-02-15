/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ms_gpio_verifier.c
 *
 * Copyright (c) 2022 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/sysfs.h>
#include <linux/suspend.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/ms_gpio_verifier.h>

#define MAX_GPIO_PINS 256
static int NUM_GPIO_PINS = 0;
static bool gpio_is_secure[MAX_GPIO_PINS];

#define NUM_GPIO_STATES 2

static struct platform_driver ms_gpio_verifier_driver;

static struct kobject* ms_gpio_verifier_kobject;
static struct kobj_attribute ms_one_gpio_attr;
static struct kobj_attribute ms_all_gpio_init_attr;
static struct kobj_attribute ms_all_gpio_sleep_attr;
static struct attribute_group ms_gpio_verifier_attr_group;

struct my_gpio {
	u8 gpio_pull;
	u8 func_sel;
	u8 drv_strength;
	u8 gpio_dir;
	u8 gpio_hihys_en;
	u8 egpio_present;
	u8 egpio_enable;

	u8 gpio_in;
	u8 gpio_out;

	// Derived attribute
	u8 gpio_low_high;
};

static struct my_gpio my_gpio_pin; // Specific GPIO pin
static struct my_gpio my_gpio_pins[NUM_GPIO_STATES][MAX_GPIO_PINS];

static u64 ms_gpio_addr[MAX_GPIO_PINS];

static bool all_gpio_were_read = 0;
static int gpio_pin_requested = -1;
static int last_single_gpio_read = -1;

static const char * const gpio_dir_str[] = {
	"in", "out"
};

static const char * const gpio_low_high_str[] = {
	"low", "high"
};

static const char * const drv_strength_str[] = {
	"2mA", "4mA", "6mA", "8mA", "10mA", "12mA", "14mA", "16mA"
};

static const char * const gpio_pull_str[] = {
	"no pull", "pull down", "keeper", "pull up"
};


// ----------------------------------------------------------------------------
// Function prototypes
// ----------------------------------------------------------------------------
static int ms_gpio_verifier_probe (struct platform_device* pdev);
static int ms_gpio_verifier_remove (struct platform_device* pdev);

static void read_one_gpio (int pin, struct my_gpio* pin_val);

static ssize_t ms_one_gpio_show (struct kobject* kobj, struct kobj_attribute* attr, char* buf);
static ssize_t ms_one_gpio_store (struct kobject* kobj, struct kobj_attribute* attr, const char* buf, size_t size);

static ssize_t ms_all_gpio_show (int state, struct kobject* kobj, struct kobj_attribute* attr, char* buf);

static ssize_t ms_all_gpio_sleep_show (struct kobject* kobj, struct kobj_attribute* attr, char* buf);

static ssize_t ms_all_gpio_init_show (struct kobject* kobj, struct kobj_attribute* attr, char* buf);

static bool gpio_pin_is_valid (int pin);
static int ms_gpio_verifier_parse_dt (struct device* dev);


// ----------------------------------------------------------------------------
// Checks if provided pin is a valid GPIO pin (only range-checking)
// ----------------------------------------------------------------------------
static bool gpio_pin_is_valid (int pin) {
	return (gpio_is_valid(pin) && (0 <= pin) && (pin < NUM_GPIO_PINS));
}


// ----------------------------------------------------------------------------
// Helper method for ms_all_gpio_init_show and ms_all_gpio_sleep_show
// ----------------------------------------------------------------------------
static ssize_t ms_all_gpio_show (int state, struct kobject* kobj, struct kobj_attribute* attr, char* buf)
{
	int i = 0;
	ssize_t bytes_written = 0;

	if (!all_gpio_were_read) { // GPIO were read in both states
		bytes_written += snprintf(buf, PAGE_SIZE, "Device has not gone into sleep\n");
		pr_warn("[%s] All GPIO pins were never read. Device has not gone into sleep\n", __func__);
		return bytes_written;
	}

	if (state < 0 || state >= NUM_GPIO_STATES) {
		pr_err("[%s] state: %d is invalid\n", __func__, state);
		return 0;
	}

	for (i = 0; i < NUM_GPIO_PINS; ++i) {
		if (bytes_written + 48 > PAGE_SIZE) {
			pr_warn("[%s] bytes_written: %ld + 48 > PAGE_SIZE: %ld\n", __func__, bytes_written, PAGE_SIZE);
			break;
		}

		bytes_written += snprintf(buf+bytes_written, 48, "%3d: %x%x%x%x%x%x%x%x%x%x\n", i,
			my_gpio_pins[state][i].gpio_dir,
			my_gpio_pins[state][i].gpio_low_high,
			my_gpio_pins[state][i].drv_strength,
			my_gpio_pins[state][i].gpio_pull,
			my_gpio_pins[state][i].func_sel,
			my_gpio_pins[state][i].gpio_hihys_en,
			my_gpio_pins[state][i].egpio_present,
			my_gpio_pins[state][i].egpio_enable,
			my_gpio_pins[state][i].gpio_in,
			my_gpio_pins[state][i].gpio_out);
	}

	return bytes_written;
}


// ----------------------------------------------------------------------------
// When ms_all_gpio_init is read
// ----------------------------------------------------------------------------
static ssize_t ms_all_gpio_init_show (struct kobject* kobj, struct kobj_attribute* attr, char* buf)
{
	pr_info("[%s] Reading pins in INIT state\n", __func__);
	return ms_all_gpio_show(MS_GPIO_INIT_STATE, kobj, attr, buf);
}


// ----------------------------------------------------------------------------
// When ms_all_gpio_sleep is read
// ----------------------------------------------------------------------------
static ssize_t ms_all_gpio_sleep_show (struct kobject* kobj, struct kobj_attribute* attr, char* buf)
{
	pr_info("[%s] Reading pins in SLEEP state\n", __func__);
	return ms_all_gpio_show(MS_GPIO_SLEEP_STATE, kobj, attr, buf);
}


// ----------------------------------------------------------------------------
// When ms_one_gpio is read (e.g., cat), what data (buf) to show
// ----------------------------------------------------------------------------
static ssize_t ms_one_gpio_show (struct kobject* kobj, struct kobj_attribute* attr, char* buf)
{
	ssize_t bytes_written = 0;

	if (last_single_gpio_read == -1) {
		bytes_written += snprintf(buf, PAGE_SIZE, "No GPIO pin number written to ms_one_gpio\n");
		pr_warn("[%s] No GPIO pin number written to ms_one_gpio\n", __func__);
		return bytes_written;
	}

	if (!gpio_pin_is_valid(last_single_gpio_read)) {
		bytes_written += snprintf(buf, PAGE_SIZE, "Requested GPIO: %d is invalid\n", last_single_gpio_read);
		pr_err("[%s] last_single_gpio_read is invalid\n", __func__);
		return bytes_written;
	}

	bytes_written += snprintf(buf+bytes_written, 48, "PIN: %d\n", last_single_gpio_read);

	bytes_written += snprintf(buf+bytes_written, 48, "gpio_dir      : 0x%x  %s\n",
		my_gpio_pin.gpio_dir, gpio_dir_str[my_gpio_pin.gpio_dir]);

	bytes_written += snprintf(buf+bytes_written, 48, "gpio_low_high : 0x%x  %s\n",
		my_gpio_pin.gpio_low_high, gpio_low_high_str[my_gpio_pin.gpio_low_high]);

	bytes_written += snprintf(buf+bytes_written, 48, "drv_strength  : 0x%x  %s\n",
		my_gpio_pin.drv_strength, drv_strength_str[my_gpio_pin.drv_strength]);

	bytes_written += snprintf(buf+bytes_written, 48, "gpio_pull     : 0x%x  %s\n",
		my_gpio_pin.gpio_pull, gpio_pull_str[my_gpio_pin.gpio_pull]);

	bytes_written += snprintf(buf+bytes_written, 48, "func_sel      : 0x%x\n", my_gpio_pin.func_sel);
	bytes_written += snprintf(buf+bytes_written, 48, "gpio_hihys_en : 0x%x\n", my_gpio_pin.gpio_hihys_en);
	bytes_written += snprintf(buf+bytes_written, 48, "egpio_present : 0x%x\n", my_gpio_pin.egpio_present);
	bytes_written += snprintf(buf+bytes_written, 48, "egpio_enable  : 0x%x\n", my_gpio_pin.egpio_enable);
	bytes_written += snprintf(buf+bytes_written, 48, "gpio_in       : 0x%x\n", my_gpio_pin.gpio_in);
	bytes_written += snprintf(buf+bytes_written, 48, "gpio_out      : 0x%x\n", my_gpio_pin.gpio_out);

	if (bytes_written > PAGE_SIZE) {
		pr_warn("[%s]  bytes_written: %ld > PAGE_SIZE: %ld\n", __func__, bytes_written, PAGE_SIZE);
	}

	return bytes_written;
}


// ----------------------------------------------------------------------------
// When ms_one_gpio is written to (e.g., echo ".." > ), the data written is buf
// ----------------------------------------------------------------------------
static ssize_t ms_one_gpio_store (struct kobject* kobj, struct kobj_attribute* attr, const char* buf, size_t size)
{
	int err = 0;
	int number_read = 0;
	ssize_t bytes_read = 0;

	pr_info("[%s] Reading GPIO file\n", __func__);

	bytes_read = strlen(buf);
	if (bytes_read > PAGE_SIZE)	{
		pr_warn("[%s] bytes_read: %ld > PAGE_SIZE: %ld\n", __func__, bytes_read, PAGE_SIZE);
	}

	pr_debug("[%s] Got: %s of size: %lu", __func__, buf, size);

	err = kstrtoint(buf, 10, &number_read); // kstrtoint_from_user for user buf (__user)
	if (err < 0) {
		pr_err("[%s] kstrtoint failed\n", __func__);
	}
	else {
		pr_info("[%s] kstrtoint successful\n", __func__);
		pr_info("[%s] gpio_pin_requested: %d\n", __func__, number_read);

		if (!gpio_pin_is_valid(number_read)) {
			pr_err("[%s] Requested pin is invalid\n", __func__);
		}
		else {
			gpio_pin_requested = number_read;
			read_one_gpio(gpio_pin_requested, &my_gpio_pin);
			last_single_gpio_read = gpio_pin_requested;
		}
	}

	pr_info("[%s] Read %ld bytes (excluding null character)\n", __func__, bytes_read);
	return bytes_read;
}


// ----------------------------------------------------------------------------
// Read configuration of a single GPIO pin and store it in pin_val
// ----------------------------------------------------------------------------
static void read_one_gpio (int pin, struct my_gpio* pin_val)
{
	void __iomem* addr = NULL;
	u32 temp = 0;

	if (!gpio_pin_is_valid(pin)) {
		pr_err("[%s] GPIO %d is invalid\n", __func__, pin);
		return ;
	}

	if (ms_gpio_addr[pin] == 0) {
		pr_err("[%s] Pin: %d has 0 address, was never set\n", __func__, pin);
		return ;
	}

	if (gpio_is_secure[pin]) {
		pr_debug("[%s] Pin %d is secure. Cannot read pin. Disable XPU first\n", __func__, pin);
		return ;
	}

	if (!pin_val) {
		pr_err("[%s] pin_val is 0\n", __func__);
		return ;
	}

	addr = ioremap(ms_gpio_addr[pin], 0x8);
	if (!addr) {
		pr_err("[%s] addr is 0. ioremap failed\n", __func__);
		iounmap(addr);
		return ;
	}

	if (IS_ERR(addr)) { // Does not check for NULL (see IS_ERR_OR_NULL)
		pr_err("[%s] IS_ERR: %d. ioremap failed\n", __func__, IS_ERR(addr));
		iounmap(addr);
		return ;
	}

	temp = readl_relaxed(addr);
	pin_val->gpio_pull 		= (temp)		& 0b11u;	// 1:0
	pin_val->func_sel 		= (temp >> 2) 	& 0b1111u;	// 5:2
	pin_val->drv_strength 	= (temp >> 6) 	& 0b111u;	// 8:6
	pin_val->gpio_dir 		= (temp >> 9) 	& 0b1u;		// 9
	pin_val->gpio_hihys_en 	= (temp >> 10) 	& 0b1u;		// 10
	pin_val->egpio_present 	= (temp >> 11) 	& 0b1u;		// 11
	pin_val->egpio_enable 	= (temp >> 12) 	& 0b1u;		// 12

	temp = readl_relaxed(addr + 0x4);
	pin_val->gpio_in		= (temp) 		& 0b1u;		// 0
	pin_val->gpio_out 		= (temp >> 1)	& 0b1u;		// 1
	pin_val->gpio_low_high 	= (pin_val->gpio_dir == 0) ? (pin_val->gpio_in) : (pin_val->gpio_out);

	// Use this if you want OUTPUT_HIGH and OUTPUT_LOW instead of only OUTPUT
	// if (pin_val->func_sel == 0 && pin_val->dir) {
	// 	pin_val->gpio_dir = (pin_val->gpio_out) ? 0x1 : 0x2;
	// 	// 0x1: OUTPUT_HIGH        0x2: OUTPUT_LOW
	// }

	iounmap(addr);
}


// ----------------------------------------------------------------------------
// Reads the following attributes of GPIO pins from 0 till NUM_GPIO_PINS
// ----------------------------------------------------------------------------
void read_all_gpio (int state)
{
	int i = 0;

	if (NUM_GPIO_PINS == 0) {
		pr_err("[%s] NUM_GPIO_PINS is 0. Returning\n", __func__);
		return ;
	}

	if ((state < 0) || (state >= NUM_GPIO_STATES)) {
		pr_err("[%s] state: %d is invalid\n", __func__, state);
		return ;
	}

	pr_info("[%s] STATE: %d | Reading all GPIO pins\n", __func__, state);
	for (i = 0; i < NUM_GPIO_PINS; ++i) {
		read_one_gpio(i, &my_gpio_pins[state][i]);
	}

	if (state == MS_GPIO_SLEEP_STATE) {
		all_gpio_were_read = 1;
	}
}


// Takes more than 1kB of size
static u32 secure_gpio[MAX_GPIO_PINS];
static u32 gpio_ranges[MAX_GPIO_PINS];


// ----------------------------------------------------------------------------
// Parse Device Tree (DT) entry for ms_gpio_verifier in soc
// ----------------------------------------------------------------------------
static int ms_gpio_verifier_parse_dt (struct device* dev)
{
	struct device_node* np = NULL;
	struct device_node* region_node = NULL;
	struct device_node* secure_gpio_node = NULL;
	struct property* prop = NULL; // Only for error detection, not used

	u32 num_gpio_regions;
	u32 num_gpio_pins;
	u32 single_gpio_config_size;
	u32 num_secure_gpio;
	u32 base_addr;
	u32 len_gpio_range;
	char region_name[10]; // regionXYZ
	u32 xpu_status;

	u32 region; // North, East, South or West
	u32 range_idx; // Which range we are considering (the index)
	u32 range_start, range_end;
	u32 pin; // Current pin

	int i;
	int err = 0;

	if (!dev) {
		pr_err("[%s] dev is 0\n", __func__);
		return 1;
	}

	if (!(dev->of_node)) {
		pr_err("[%s] dev->of_node is 0\n", __func__);
		return 1;
	}

	np = dev->of_node;

	pr_debug("[%s] np->name: %s\n", __func__, np->name);
	pr_debug("[%s] np->full_name: %s\n", __func__, np->full_name);

	pr_info("[%s] Setting addresses of every GPIO pin\n", __func__);


	// Number of GPIO regions
	// ----------------------
	err = device_property_read_u32(dev, "num-gpio-regions", &num_gpio_regions);
	if (err) {
		pr_err("[%s] device_property_read_u32 for num-regions failed: %d\n", __func__, err);
		return err;
	}

	pr_info("[%s] num_gpio_regions: %u\n", __func__, num_gpio_regions);


	// Number of GPIOs
	// ---------------
	err = device_property_read_u32(dev, "num-gpio-pins", &num_gpio_pins);
	if (err) {
		pr_err("[%s] device_property_read_u32 for num-gpio-pins failed: %d\n", __func__, err);
		return err;
	}

	if (num_gpio_pins < 0 || num_gpio_pins > MAX_GPIO_PINS) {
		pr_err("[%s] num_gpio_pins: %d is invalid\n", __func__, num_gpio_pins);
		return 1;
	}

	NUM_GPIO_PINS = num_gpio_pins;
	pr_info("[%s] NUM_GPIO_PINS: %d\n", __func__, NUM_GPIO_PINS);


	// Single GPIO Configuration Size
	// ------------------------------
	err = device_property_read_u32(dev, "single-gpio-config-size", &single_gpio_config_size);
	if (err) {
		pr_err("[%s] device_property_read_u32 for single-gpio-config-size failed: %d\n", __func__, err);
		return err;
	}

	pr_info("[%s] single_gpio_config_size: %d\n", __func__, single_gpio_config_size);


	// Secure GPIOs
	// ------------
	for (pin = 0; pin < NUM_GPIO_PINS; ++pin) {
		gpio_is_secure[pin] = 0;
	}

	secure_gpio_node = of_get_child_by_name(np, "secure-gpio");
	if (!secure_gpio_node) {
		pr_err("[%s] of_get_child_by_name for secure-gpio failed\n", __func__);
		return 1;
	}

	err = of_property_read_u32(secure_gpio_node, "xpu-status", &xpu_status);
	if (err) {
		pr_err("[%s] of_property_read_u32 for xpu-status failed: %d\n", __func__, err);
		return 1;
	}

	pr_info("[%s] xpu_status: %u\n", __func__, xpu_status);

	if (xpu_status == 0) {
		pr_info("[%s] Skipping reading secure-gpio pins (xpu-status == 0)\n", __func__);
		of_node_put(secure_gpio_node);
		goto iterate_over_gpio_ranges;
	}

	prop = of_find_property(secure_gpio_node, "pins", &num_secure_gpio);
	if (!prop) {
		pr_err("[%s] of_find_property for secure-gpio : pins failed\n", __func__);
		return 1;
	}

	pr_info("[%s] num_secure_gpio (before): %d\n", __func__, num_secure_gpio);
	num_secure_gpio /= sizeof(u32);
	pr_info("[%s] num_secure_gpio (after): %d\n", __func__, num_secure_gpio);

	if (num_secure_gpio < 0 || num_secure_gpio > NUM_GPIO_PINS) {
		pr_err("[%s] num_secure_gpio: %d is invalid\n", __func__, num_secure_gpio);
		return 1;
	}

	err = of_property_read_u32_array(secure_gpio_node, "pins", (u32*)secure_gpio, num_secure_gpio);
	if (err) {
		pr_err("[%s] device_property_read_u32_array failed for secure-gpio: %d\n", __func__, err);
		return err;
	}

	for (i = 0; i < num_secure_gpio; ++i) {
		pin = secure_gpio[i];
		pr_info("[%s] Pin %d is secure\n", __func__, pin);
		gpio_is_secure[pin] = 1;
	}

	of_node_put(secure_gpio_node);

	// Iterate over all GPIO regions
	// -----------------------------
iterate_over_gpio_ranges:

	for (region = 0; region < num_gpio_regions; ++region) {
		snprintf(region_name, sizeof(region_name), "region%u", region);

		region_node = of_get_child_by_name(np, region_name); // np = dev->of_node
		if (!region_node) {
			pr_err("[%s] [Region number mismatch] of_get_child_by_name could not find: %s\n", __func__, region_name);
			continue;
		}


		// Read all GPIO ranges of region
		// ------------------------------
		prop = of_find_property(region_node, "gpio-ranges", &len_gpio_range);
		if (!prop) {
			pr_err("[%s] of_find_property for gpio-ranges failed (%s)\n", __func__, region_name);
			of_node_put(region_node);
			continue;
		}

		pr_info("[%s] len_gpio_range (before): %d\n", __func__, len_gpio_range);
		len_gpio_range /= sizeof(u32);
		pr_info("[%s] len_gpio_range (after): %d\n", __func__, len_gpio_range);

		if (len_gpio_range == 0) {
			pr_warn("[%s] len_gpio_range is 0 (%s)\n", __func__, region_name);
			of_node_put(region_node);
			continue;
		}

		if (len_gpio_range % 2 == 1) {
			pr_warn("[%s] len_gpio_range: %d is not even (%s)\n", __func__, len_gpio_range, region_name);
			of_node_put(region_node);
			continue;
		}

		err = of_property_read_u32_array(region_node, "gpio-ranges", (u32*)gpio_ranges, len_gpio_range);
		if (err) {
			pr_err("[%s] of_property_read_u32_array for gpio-ranges failed: %d\n", __func__, err);
			of_node_put(region_node);
			continue;
		}


		// Base address of region
		// ----------------------
		err = of_property_read_u32(region_node, "base-addr", &base_addr); // Change to u64 in final version
		if (err) {
			pr_err("[%s] of_property_read_u32 for base-addr failed: %d (%s)\n", __func__, err, region_name);
			of_node_put(region_node);
			continue;
		}

		pr_debug("[%s] base_addr of %s: 0x%x\n", __func__, region_name, base_addr);


		// Set address of every pin
		// ------------------------
		for (range_idx = 0; range_idx + 1 < len_gpio_range; range_idx += 2) {
			range_start = gpio_ranges[range_idx];
			range_end   = gpio_ranges[range_idx+1];
			pr_info("[%s] Range: [%u, %u]\n", __func__, range_start, range_end);

			if ((range_start > range_end) || (range_end >= NUM_GPIO_PINS)) {
				pr_warn("[%s] Invalid range\n", __func__);
				continue;
			}

			for (pin = range_start; pin <= range_end; ++pin) {
				ms_gpio_addr[pin] = base_addr + (pin * single_gpio_config_size);
				pr_debug("[%s] addr of pin %d: 0x%llx\n", __func__, pin, ms_gpio_addr[pin]);
			}
		}

		of_node_put(region_node);
	}

	return 0;
}

// ----------------------------------------------------------------------------
// sysfs initialization
// ----------------------------------------------------------------------------
static struct kobj_attribute ms_one_gpio_attr =
__ATTR(ms_one_gpio, 0644, ms_one_gpio_show, ms_one_gpio_store);


static struct kobj_attribute ms_all_gpio_init_attr =
__ATTR(ms_all_gpio_init, 0444, ms_all_gpio_init_show, NULL);


static struct kobj_attribute ms_all_gpio_sleep_attr =
__ATTR(ms_all_gpio_sleep, 0444, ms_all_gpio_sleep_show, NULL);


static struct attribute* ms_gpio_verifier_kobj_attr[] = {
	&ms_one_gpio_attr.attr,
	&ms_all_gpio_init_attr.attr,
	&ms_all_gpio_sleep_attr.attr,
	NULL,
};


static struct attribute_group ms_gpio_verifier_attr_group = {
	.attrs = ms_gpio_verifier_kobj_attr,
};


// ----------------------------------------------------------------------------
// Probe
// ----------------------------------------------------------------------------
static int ms_gpio_verifier_probe (struct platform_device* pdev)
{
	int i = 0, err = 0;
	pr_info("[%s] Probing\n", __func__);

	if (pdev) {
		dev_info(&(pdev->dev), "[ms_gpio_verifier_probe] dev_info\n");
	}
	else {
		pr_err("[%s] pdev is 0\n", __func__);
	}


	// Fill GPIO base addresses for all pins
	// -------------------------------------
	err = ms_gpio_verifier_parse_dt(&(pdev->dev));
	if (err) {
		pr_err("[%s] ms_gpio_verifier_parse_dt failed: %d\n", __func__, err);
		goto ms_gpio_verifier_parse_dt_failed;
	}

	ms_gpio_verifier_kobject = kobject_create_and_add("ms_gpio_verifier", NULL); //pdev->dev.kobj
	if (!ms_gpio_verifier_kobject) {
		pr_err("[%s] kobject_create_and_add failed\n", __func__);
		goto kobject_create_and_add_fail;
	}

	err = sysfs_create_group(ms_gpio_verifier_kobject, &ms_gpio_verifier_attr_group);
	if (err) {
		pr_err("[%s] sysfs_create_group failed: %d\n", __func__, err);
		goto sys_create_group_fail;
	}

	return 0;

	// Error handling
sys_create_group_fail:
	sysfs_remove_group(ms_gpio_verifier_kobject, &ms_gpio_verifier_attr_group);

kobject_create_and_add_fail:
	kobject_del(ms_gpio_verifier_kobject);

ms_gpio_verifier_parse_dt_failed:
	// platform_driver_unregister(&ms_gpio_verifier_driver);
	pr_info("[%s] Setting address of all GPIO pins to 0\n", __func__);
	for (i = 0; i < MAX_GPIO_PINS; ++i) {
		ms_gpio_addr[i] = 0;
	}

	return err;
}

// ----------------------------------------------------------------------------
// Remove
// ----------------------------------------------------------------------------
static int ms_gpio_verifier_remove (struct platform_device* pdev)
{
	int err = 0;
	pr_info("[%s] Removing\n", __func__);

	sysfs_remove_group(ms_gpio_verifier_kobject, &ms_gpio_verifier_attr_group);

	kobject_del(ms_gpio_verifier_kobject); // kobject_put

	return err;
}


// ----------------------------------------------------------------------------
// Init
// ----------------------------------------------------------------------------
static int __init ms_gpio_verifier_init (void)
{
	int err = 0;
	pr_info("[%s] Initting\n", __func__);

	err = platform_driver_register(&ms_gpio_verifier_driver);
	if (err) {
		pr_err("[%s] platform_driver_register failed: %d\n", __func__, err);
		platform_driver_unregister(&ms_gpio_verifier_driver);
		return err;
	}

	return 0;
}


// ----------------------------------------------------------------------------
// Exit
// ----------------------------------------------------------------------------
static void __exit ms_gpio_verifier_exit (void)
{
	pr_info("[%s] Exiting\n", __func__);
	platform_driver_unregister(&ms_gpio_verifier_driver);
}

// ----------------------------------------------------------------------------
// Platform Driver
// ----------------------------------------------------------------------------
static const struct of_device_id ms_gpio_verifier_match_table[] = {
	{ .compatible = "qcom,ms_gpio_verifier", },
	{},
};


static struct platform_driver ms_gpio_verifier_driver = {
	.driver = {
		.name = "ms_gpio_verifier",
		.of_match_table = ms_gpio_verifier_match_table,
		.owner = THIS_MODULE,
	},
	.probe = ms_gpio_verifier_probe,
	.remove = ms_gpio_verifier_remove,
};


module_init(ms_gpio_verifier_init);
module_exit(ms_gpio_verifier_exit);


MODULE_AUTHOR("Karan Tandon <t-katand@microsoft.com>");
MODULE_DESCRIPTION("Saves status of all GPIO pins just before device suspends");
MODULE_LICENSE("GPL");

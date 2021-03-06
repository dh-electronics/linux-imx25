/*
 * Generic pwmlib implementation
 *
 * Copyright (C) 2011 Sascha Hauer <s.hauer@pengutronix.de>
 * Copyright (C) 2011-2012 Avionic Design GmbH
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; see the file COPYING.  If not, write to
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/radix-tree.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

static DEFINE_MUTEX(pwm_lookup_lock);
static LIST_HEAD(pwm_lookup_list);
static DEFINE_MUTEX(pwm_lock);
static LIST_HEAD(pwm_chips);
static DECLARE_BITMAP(allocated_pwms, MAX_PWMS);
static RADIX_TREE(pwm_tree, GFP_KERNEL);


#ifdef CONFIG_PWM_SYSFS

/* lock protects against unexport_pwm() being called while
 * sysfs files are active.
 */
static DEFINE_MUTEX(sysfs_lock);
static struct class pwm_class;
static struct pwm_device *pwm_table[MAX_PWMS];

/*
 * /sys/class/pwm/pwm... only for PWMs that are exported
 *   /polarity
 *      * only visible if the underlying driver has registered a
 *        set_polarity function
 *      * always readable
 *      * may be written as "1" for inverted polarity or "0" for
 *        normal polarity
 *      * can only be written if PWM is not running
 *   /period_ns
 *      * always readable
 *      * write with desired pwm period in nanoseconds
 *      * may return with error depending on duty_ns
 *   /duty_ns
 *      * always readable
 *      * write with desired duty portion in nanoseconds
 *      * may return with error depending on period_ns
 *   /run
 *      * always readable
 *      * write with "1" to start generating pwm signal, "0" to stop it
 */
static ssize_t pwm_polarity_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	const struct pwm_device *pwm = dev_get_drvdata(dev);
	ssize_t	status;

	mutex_lock(&sysfs_lock);

	if (!test_bit(PWMF_EXPORT, &pwm->flags))
		status = -EIO;
	else
		status = sprintf(buf, "%d\n", pwm->polarity);

	mutex_unlock(&sysfs_lock);

	return status;
}

static ssize_t pwm_polarity_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pwm_device *pwm = dev_get_drvdata(dev);
	ssize_t status;

	mutex_lock(&sysfs_lock);

	if (!test_bit(PWMF_EXPORT, &pwm->flags))
		status = -EIO;
	else {
		long value;

		status = kstrtol(buf, 0, &value);
		if (status == 0) {
			if (value == 0) {
				if (pwm->polarity == PWM_POLARITY_NORMAL)
					goto fail_unlock;
				status = pwm_set_polarity(pwm,
							PWM_POLARITY_NORMAL);
			} else {
				if (pwm->polarity == PWM_POLARITY_INVERSED)
					goto fail_unlock;
				status = pwm_set_polarity(pwm,
							PWM_POLARITY_INVERSED);
			}
		}
	}

fail_unlock:
	mutex_unlock(&sysfs_lock);
	return status ? : size;
}

static const DEVICE_ATTR(polarity, 0644,
		pwm_polarity_show, pwm_polarity_store);


static ssize_t pwm_period_ns_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	const struct pwm_device *pwm = dev_get_drvdata(dev);
	ssize_t	status;

	mutex_lock(&sysfs_lock);

	if (!test_bit(PWMF_EXPORT, &pwm->flags))
		status = -EIO;
	else
		status = sprintf(buf, "%d\n", pwm->period);

	mutex_unlock(&sysfs_lock);

	return status;
}

static ssize_t pwm_period_ns_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pwm_device *pwm = dev_get_drvdata(dev);
	ssize_t status;

	mutex_lock(&sysfs_lock);

	if (!test_bit(PWMF_EXPORT, &pwm->flags))
		status = -EIO;
	else {
		long value;

		status = kstrtol(buf, 0, &value);
		if (status == 0) {
			if (pwm->duty < 0)
				pwm->period = value;
			else
				status = pwm_config(pwm, pwm->duty, value);
		}
	}

	mutex_unlock(&sysfs_lock);

	return status ? : size;
}

static const DEVICE_ATTR(period_ns, 0644,
		pwm_period_ns_show, pwm_period_ns_store);

static ssize_t pwm_duty_ns_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	const struct pwm_device *pwm = dev_get_drvdata(dev);
	ssize_t	status;

	mutex_lock(&sysfs_lock);

	if (!test_bit(PWMF_EXPORT, &pwm->flags))
		status = -EIO;
	else
		status = sprintf(buf, "%d\n", pwm->duty);

	mutex_unlock(&sysfs_lock);

	return status;
}

static ssize_t pwm_duty_ns_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pwm_device *pwm = dev_get_drvdata(dev);
	ssize_t status;

	mutex_lock(&sysfs_lock);

	if (!test_bit(PWMF_EXPORT, &pwm->flags))
		status = -EIO;
	else {
		long value;

		status = kstrtol(buf, 0, &value);
		if (status == 0) {
			if (pwm->period <= 0)
				pwm->duty = value;
			else
				status = pwm_config(pwm, value, pwm->period);
		}
	}

	mutex_unlock(&sysfs_lock);

	return status ? : size;
}

static const DEVICE_ATTR(duty_ns, 0644,
		pwm_duty_ns_show, pwm_duty_ns_store);


static ssize_t pwm_run_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	const struct pwm_device *pwm = dev_get_drvdata(dev);
	ssize_t	status;

	mutex_lock(&sysfs_lock);

	if (!test_bit(PWMF_EXPORT, &pwm->flags))
		status = -EIO;
	else
		status = sprintf(buf, "%d\n",
				!!test_bit(PWMF_ENABLED, &pwm->flags));

	mutex_unlock(&sysfs_lock);

	return status;
}

static ssize_t pwm_run_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pwm_device *pwm = dev_get_drvdata(dev);
	ssize_t	status;

	mutex_lock(&sysfs_lock);

	if (!test_bit(PWMF_EXPORT, &pwm->flags))
		status = -EIO;
	else {
		long		value;

		status = kstrtol(buf, 0, &value);
		if (status == 0) {
			if (value)
				status = pwm_enable(pwm);
			else
				pwm_disable(pwm);
		}
	}

	mutex_unlock(&sysfs_lock);

	return status ? : size;
}

static DEVICE_ATTR(run, 0644, pwm_run_show, pwm_run_store);

static const struct attribute *pwm_attrs[] = {
	&dev_attr_period_ns.attr,
	&dev_attr_duty_ns.attr,
	&dev_attr_run.attr,
	NULL,
};

static const struct attribute_group pwm_attr_group = {
	.attrs = (struct attribute **) pwm_attrs,
};

/*
 * /sys/class/pwm/pwmchipN/
 *   /base ... matching pwm_chip.base (N)
 *   /ngpio ... matching pwm_chip.npwm
 */

static ssize_t chip_base_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	const struct pwm_chip	*chip = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", chip->base);
}
static DEVICE_ATTR(base, 0444, chip_base_show, NULL);

static ssize_t chip_npwm_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	const struct pwm_chip	*chip = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", chip->npwm);
}
static DEVICE_ATTR(npwm, 0444, chip_npwm_show, NULL);

static const struct attribute *pwmchip_attrs[] = {
	&dev_attr_base.attr,
	&dev_attr_npwm.attr,
	NULL,
};

static const struct attribute_group pwmchip_attr_group = {
	.attrs = (struct attribute **) pwmchip_attrs,
};

static int pwm_export(struct pwm_device *pwm)
{
	struct device *dev;
	int status;

	mutex_lock(&sysfs_lock);

	if (!test_bit(PWMF_REQUESTED, &pwm->flags) ||
		test_bit(PWMF_EXPORT, &pwm->flags)) {
		pr_debug("pwm %d unavailable (requested=%d, exported=%d)\n",
			pwm->pwm,
			test_bit(PWMF_REQUESTED, &pwm->flags),
			test_bit(PWMF_EXPORT, &pwm->flags));

		status = -EPERM;
		goto fail_unlock;
	}

	dev = device_create(&pwm_class, pwm->chip->dev, MKDEV(0, 0),
			pwm, "pwm%u", pwm->pwm);
	if (IS_ERR(dev)) {
		status = PTR_ERR(dev);
		goto fail_unlock;
	}

	status = sysfs_create_group(&dev->kobj, &pwm_attr_group);
	if (status)
		goto fail_unregister_device;

	if (pwm->chip->ops->set_polarity) {
		status = device_create_file(dev, &dev_attr_polarity);
		if (status)
			goto fail_unregister_device;
	}

	set_bit(PWMF_EXPORT, &pwm->flags);
	mutex_unlock(&sysfs_lock);
	return 0;

fail_unregister_device:
	device_unregister(dev);
fail_unlock:
	mutex_unlock(&sysfs_lock);
	return status;
}

static int match_export(struct device *dev, void *data)
{
	return dev_get_drvdata(dev) == data;
}

/*
 * /sys/class/pwm/export ... write-only
 *	integer N ... number of pwm to export (full access)
 * /sys/class/pwm/unexport ... write-only
 *	integer N ... number of pwm to unexport
 */
static ssize_t export_store(struct class *class,
				struct class_attribute *attr,
				const char *buf, size_t len)
{
	long pwm;
	int status;
	struct pwm_device *dev;
	struct pwm_chip *chip;

	status = kstrtol(buf, 0, &pwm);
	if (status < 0)
		goto done;

	if (!pwm_is_valid(pwm) || !pwm_table[pwm]) {
		status = -ENODEV;
		goto done;
	}
	chip = pwm_table[pwm]->chip;
	if (!chip) {
		status = -ENODEV;
		goto done;
	}
	dev = pwm_request_from_chip(chip, pwm - chip->base, "sysfs");
	if (IS_ERR(dev)) {
		status = -ENODEV;
		goto done;
	}
	status = pwm_export(dev);
	if (status < 0)
		pwm_free(dev);
done:
	if (status)
		pr_debug("%s: status %d\n", __func__, status);
	return status ? : len;
}

static ssize_t unexport_store(struct class *class,
				struct class_attribute *attr,
				const char *buf, size_t len)
{
	long pwm;
	int status;
	struct pwm_device *dev;
	struct device *d;

	status = kstrtol(buf, 0, &pwm);
	if (status < 0)
		goto done;

	status = -EINVAL;

	/* reject bogus pwms */
	if (!pwm_is_valid(pwm))
		goto done;

	dev = pwm_table[pwm];
	if (dev && test_and_clear_bit(PWMF_EXPORT, &dev->flags)) {
		d = class_find_device(&pwm_class, NULL, dev, match_export);
		if (d)
			device_unregister(d);
		status = 0;
		pwm_put(dev);
	}
done:
	if (status)
		pr_debug("%s: status %d\n", __func__, status);
	return status ? : len;
}

static struct class_attribute pwm_class_attrs[] = {
	__ATTR(export, 0200, NULL, export_store),
	__ATTR(unexport, 0200, NULL, unexport_store),
	__ATTR_NULL,
};

static struct class pwm_class = {
	.name =		"pwm",
	.owner =	THIS_MODULE,
	.class_attrs =	pwm_class_attrs,
};

static int pwmchip_export(struct pwm_chip *chip)
{
	int status;
	struct device *dev;

	mutex_lock(&sysfs_lock);
	dev = device_create(&pwm_class, chip->dev, MKDEV(0, 0), chip,
				"pwmchip%d", chip->base);
	if (!IS_ERR(dev))
		status = sysfs_create_group(&dev->kobj, &pwmchip_attr_group);
	else
		status = PTR_ERR(dev);

	chip->exported = (status == 0);
	mutex_unlock(&sysfs_lock);

	if (status) {
		unsigned i;

		mutex_lock(&pwm_lock);
		i = chip->base;
		while (pwm_table[i]->chip == chip)
			pwm_table[i++]->chip = NULL;

		mutex_unlock(&pwm_lock);

		pr_debug("%s: pwm chip status %d\n", __func__, status);
	}

	return status;
}

static void pwmchip_unexport(struct pwm_chip *chip)
{
	int status, i;
	struct device *dev;

	mutex_lock(&sysfs_lock);
	dev = class_find_device(&pwm_class, NULL, chip, match_export);
	if (dev) {
		put_device(dev);
		device_unregister(dev);
		for (i = chip->base; i < chip->base + chip->npwm; i++)
			pwm_table[i] = NULL;
		chip->exported = 0;
		status = 0;
	} else
		status = -ENODEV;
	mutex_unlock(&sysfs_lock);

	if (status)
		pr_debug("%s: pwm chip status %d\n", __func__, status);
}

static int __init pwmlib_sysfs_init(void)
{
	int status;

	status = class_register(&pwm_class);
	return status;
}
postcore_initcall(pwmlib_sysfs_init);

#else
static inline int pwmchip_export(struct pwm_chip *chip)
{
	return 0;
}

static inline void pwmchip_unexport(struct pwm_chip *chip)
{
}

#endif /* CONFIG_PWM_SYSFS */


static struct pwm_device *pwm_to_device(unsigned int pwm)
{
	return radix_tree_lookup(&pwm_tree, pwm);
}

static int alloc_pwms(int pwm, unsigned int count)
{
	unsigned int from = 0;
	unsigned int start;

	if (pwm >= MAX_PWMS)
		return -EINVAL;

	if (pwm >= 0)
		from = pwm;

	start = bitmap_find_next_zero_area(allocated_pwms, MAX_PWMS, from,
					   count, 0);

	if (pwm >= 0 && start != pwm)
		return -EEXIST;

	if (start + count > MAX_PWMS)
		return -ENOSPC;

	return start;
}

static void free_pwms(struct pwm_chip *chip)
{
	unsigned int i;

	for (i = 0; i < chip->npwm; i++) {
		struct pwm_device *pwm = &chip->pwms[i];
		radix_tree_delete(&pwm_tree, pwm->pwm);
#ifdef CONFIG_PWM_SYSFS
		pwm_table[i + chip->base]->chip = NULL;
#endif
	}
	bitmap_clear(allocated_pwms, chip->base, chip->npwm);

	kfree(chip->pwms);
	chip->pwms = NULL;
}

static struct pwm_chip *pwmchip_find_by_name(const char *name)
{
	struct pwm_chip *chip;

	if (!name)
		return NULL;

	mutex_lock(&pwm_lock);

	list_for_each_entry(chip, &pwm_chips, list) {
		const char *chip_name = dev_name(chip->dev);

		if (chip_name && strcmp(chip_name, name) == 0) {
			mutex_unlock(&pwm_lock);
			return chip;
		}
	}

	mutex_unlock(&pwm_lock);

	return NULL;
}

static int pwm_device_request(struct pwm_device *pwm, const char *label)
{
	int err;

	if (test_bit(PWMF_REQUESTED, &pwm->flags))
		return -EBUSY;

	if (!try_module_get(pwm->chip->ops->owner))
		return -ENODEV;

	if (pwm->chip->ops->request) {
		err = pwm->chip->ops->request(pwm->chip, pwm);
		if (err) {
			module_put(pwm->chip->ops->owner);
			return err;
		}
	}

	set_bit(PWMF_REQUESTED, &pwm->flags);
	pwm->label = label;

	return 0;
}

/**
 * pwm_set_chip_data() - set private chip data for a PWM
 * @pwm: PWM device
 * @data: pointer to chip-specific data
 */
int pwm_set_chip_data(struct pwm_device *pwm, void *data)
{
	if (!pwm)
		return -EINVAL;

	pwm->chip_data = data;

	return 0;
}

/**
 * pwm_get_chip_data() - get private chip data for a PWM
 * @pwm: PWM device
 */
void *pwm_get_chip_data(struct pwm_device *pwm)
{
	return pwm ? pwm->chip_data : NULL;
}

/**
 * pwmchip_add() - register a new PWM chip
 * @chip: the PWM chip to add
 *
 * Register a new PWM chip. If chip->base < 0 then a dynamically assigned base
 * will be used.
 */
int pwmchip_add(struct pwm_chip *chip)
{
	struct pwm_device *pwm;
	unsigned int i;
	int ret;

	if (!chip || !chip->dev || !chip->ops || !chip->ops->config ||
	    !chip->ops->enable || !chip->ops->disable)
		return -EINVAL;

	mutex_lock(&pwm_lock);

	ret = alloc_pwms(chip->base, chip->npwm);
	if (ret < 0)
		goto out;

	chip->pwms = kzalloc(chip->npwm * sizeof(*pwm), GFP_KERNEL);
	if (!chip->pwms) {
		ret = -ENOMEM;
		goto out;
	}

	chip->base = ret;

	for (i = 0; i < chip->npwm; i++) {
		pwm = &chip->pwms[i];

		pwm->chip = chip;
		pwm->pwm = chip->base + i;
		pwm->hwpwm = i;
#ifdef CONFIG_PWM_SYSFS
		pwm_table[i + chip->base] = pwm;
#endif

		radix_tree_insert(&pwm_tree, pwm->pwm, pwm);
	}

	bitmap_set(allocated_pwms, chip->base, chip->npwm);

	INIT_LIST_HEAD(&chip->list);
	list_add(&chip->list, &pwm_chips);

	ret = 0;

	ret = pwmchip_export(chip);

out:
	mutex_unlock(&pwm_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(pwmchip_add);

/**
 * pwmchip_remove() - remove a PWM chip
 * @chip: the PWM chip to remove
 *
 * Removes a PWM chip. This function may return busy if the PWM chip provides
 * a PWM device that is still requested.
 */
int pwmchip_remove(struct pwm_chip *chip)
{
	unsigned int i;
	int ret = 0;

	mutex_lock(&pwm_lock);

	for (i = 0; i < chip->npwm; i++) {
		struct pwm_device *pwm = &chip->pwms[i];

		if (test_bit(PWMF_REQUESTED, &pwm->flags)) {
			ret = -EBUSY;
			goto out;
		}
	}

	list_del_init(&chip->list);
	free_pwms(chip);
	pwmchip_unexport(chip);

out:
	mutex_unlock(&pwm_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(pwmchip_remove);

/**
 * pwm_request() - request a PWM device
 * @pwm_id: global PWM device index
 * @label: PWM device label
 *
 * This function is deprecated, use pwm_get() instead.
 */
struct pwm_device *pwm_request(int pwm, const char *label)
{
	struct pwm_device *dev;
	int err;

	if (pwm < 0 || pwm >= MAX_PWMS)
		return ERR_PTR(-EINVAL);

	mutex_lock(&pwm_lock);

	dev = pwm_to_device(pwm);
	if (!dev) {
		dev = ERR_PTR(-EPROBE_DEFER);
		goto out;
	}

	err = pwm_device_request(dev, label);
	if (err < 0)
		dev = ERR_PTR(err);

out:
	mutex_unlock(&pwm_lock);

	return dev;
}
EXPORT_SYMBOL_GPL(pwm_request);

/**
 * pwm_request_from_chip() - request a PWM device relative to a PWM chip
 * @chip: PWM chip
 * @index: per-chip index of the PWM to request
 * @label: a literal description string of this PWM
 *
 * Returns the PWM at the given index of the given PWM chip. A negative error
 * code is returned if the index is not valid for the specified PWM chip or
 * if the PWM device cannot be requested.
 */
struct pwm_device *pwm_request_from_chip(struct pwm_chip *chip,
					 unsigned int index,
					 const char *label)
{
	struct pwm_device *pwm;
	int err;

	if (!chip || index >= chip->npwm)
		return ERR_PTR(-EINVAL);

	mutex_lock(&pwm_lock);
	pwm = &chip->pwms[index];

	err = pwm_device_request(pwm, label);
	if (err < 0)
		pwm = ERR_PTR(err);

	mutex_unlock(&pwm_lock);
	return pwm;
}
EXPORT_SYMBOL_GPL(pwm_request_from_chip);

/**
 * pwm_free() - free a PWM device
 * @pwm: PWM device
 *
 * This function is deprecated, use pwm_put() instead.
 */
void pwm_free(struct pwm_device *pwm)
{
	pwm_put(pwm);
}
EXPORT_SYMBOL_GPL(pwm_free);

/**
 * pwm_config() - change a PWM device configuration
 * @pwm: PWM device
 * @duty_ns: "on" time (in nanoseconds)
 * @period_ns: duration (in nanoseconds) of one cycle
 */
int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	int status;

	if (!pwm || duty_ns < 0 || period_ns <= 0 || duty_ns > period_ns)
		return -EINVAL;

	status = pwm->chip->ops->config(pwm->chip, pwm, duty_ns, period_ns);
#ifdef CONFIG_PWM_SYSFS
	if (status == 0) {
		pwm->period = period_ns;
		pwm->duty = duty_ns;
	}
#endif
	return status;
}
EXPORT_SYMBOL_GPL(pwm_config);

/**
 * pwm_get_config() - get a PWM device configuration
 * @pwm: PWM device
 * @duty_ns: "on" time (in nanoseconds)
 * @period_ns: duration (in nanoseconds) of one cycle
 */
int pwm_get_config(struct pwm_device *pwm, int *duty_ns, int *period_ns)
{
	if (!pwm || !pwm->chip->ops)
		return -EINVAL;

	if (!pwm->chip->ops->get_config)
		return -ENOSYS;

	return pwm->chip->ops->get_config(pwm->chip, duty_ns, period_ns);
}
EXPORT_SYMBOL_GPL(pwm_get_config);

/**
 * pwm_set_polarity() - configure the polarity of a PWM signal
 * @pwm: PWM device
 * @polarity: new polarity of the PWM signal
 *
 * Note that the polarity cannot be configured while the PWM device is enabled
 */
int pwm_set_polarity(struct pwm_device *pwm, enum pwm_polarity polarity)
{
	int status;

	if (!pwm || !pwm->chip->ops)
		return -EINVAL;

	if (!pwm->chip->ops->set_polarity)
		return -ENOSYS;

	if (test_bit(PWMF_ENABLED, &pwm->flags))
		return -EBUSY;

	status = pwm->chip->ops->set_polarity(pwm->chip, pwm, polarity);
#ifdef CONFIG_PWM_SYSFS
	if (!status)
		pwm->polarity = polarity;
#endif
	return status;
}
EXPORT_SYMBOL_GPL(pwm_set_polarity);

/**
 * pwm_enable() - start a PWM output toggling
 * @pwm: PWM device
 */
int pwm_enable(struct pwm_device *pwm)
{
	if (pwm && !test_and_set_bit(PWMF_ENABLED, &pwm->flags))
		return pwm->chip->ops->enable(pwm->chip, pwm);

	return pwm ? 0 : -EINVAL;
}
EXPORT_SYMBOL_GPL(pwm_enable);

/**
 * pwm_disable() - stop a PWM output toggling
 * @pwm: PWM device
 */
void pwm_disable(struct pwm_device *pwm)
{
	if (pwm && test_and_clear_bit(PWMF_ENABLED, &pwm->flags))
		pwm->chip->ops->disable(pwm->chip, pwm);
}
EXPORT_SYMBOL_GPL(pwm_disable);

/**
 * pwm_add_table() - register PWM device consumers
 * @table: array of consumers to register
 * @num: number of consumers in table
 */
void __init pwm_add_table(struct pwm_lookup *table, size_t num)
{
	mutex_lock(&pwm_lookup_lock);

	while (num--) {
		list_add_tail(&table->list, &pwm_lookup_list);
		table++;
	}

	mutex_unlock(&pwm_lookup_lock);
}

/**
 * pwm_get() - look up and request a PWM device
 * @dev: device for PWM consumer
 * @con_id: consumer name
 *
 * Look up a PWM chip and a relative index via a table supplied by board setup
 * code (see pwm_add_table()).
 *
 * Once a PWM chip has been found the specified PWM device will be requested
 * and is ready to be used.
 */
struct pwm_device *pwm_get(struct device *dev, const char *con_id)
{
	struct pwm_device *pwm = ERR_PTR(-EPROBE_DEFER);
	const char *dev_id = dev ? dev_name(dev): NULL;
	struct pwm_chip *chip = NULL;
	unsigned int index = 0;
	unsigned int best = 0;
	struct pwm_lookup *p;
	unsigned int match;

	/*
	 * We look up the provider in the static table typically provided by
	 * board setup code. We first try to lookup the consumer device by
	 * name. If the consumer device was passed in as NULL or if no match
	 * was found, we try to find the consumer by directly looking it up
	 * by name.
	 *
	 * If a match is found, the provider PWM chip is looked up by name
	 * and a PWM device is requested using the PWM device per-chip index.
	 *
	 * The lookup algorithm was shamelessly taken from the clock
	 * framework:
	 *
	 * We do slightly fuzzy matching here:
	 *  An entry with a NULL ID is assumed to be a wildcard.
	 *  If an entry has a device ID, it must match
	 *  If an entry has a connection ID, it must match
	 * Then we take the most specific entry - with the following order
	 * of precedence: dev+con > dev only > con only.
	 */
	mutex_lock(&pwm_lookup_lock);

	list_for_each_entry(p, &pwm_lookup_list, list) {
		match = 0;

		if (p->dev_id) {
			if (!dev_id || strcmp(p->dev_id, dev_id))
				continue;

			match += 2;
		}

		if (p->con_id) {
			if (!con_id || strcmp(p->con_id, con_id))
				continue;

			match += 1;
		}

		if (match > best) {
			chip = pwmchip_find_by_name(p->provider);
			index = p->index;

			if (match != 3)
				best = match;
			else
				break;
		}
	}

	if (chip)
		pwm = pwm_request_from_chip(chip, index, con_id ?: dev_id);

	mutex_unlock(&pwm_lookup_lock);

	return pwm;
}
EXPORT_SYMBOL_GPL(pwm_get);

/**
 * pwm_put() - release a PWM device
 * @pwm: PWM device
 */
void pwm_put(struct pwm_device *pwm)
{
	if (!pwm)
		return;

	mutex_lock(&pwm_lock);

	if (!test_and_clear_bit(PWMF_REQUESTED, &pwm->flags)) {
		pr_warning("PWM device already freed\n");
		goto out;
	}

	if (pwm->chip->ops->free)
		pwm->chip->ops->free(pwm->chip, pwm);

	pwm->label = NULL;

	module_put(pwm->chip->ops->owner);
out:
	mutex_unlock(&pwm_lock);
}
EXPORT_SYMBOL_GPL(pwm_put);

#ifdef CONFIG_DEBUG_FS
static void pwm_dbg_show(struct pwm_chip *chip, struct seq_file *s)
{
	unsigned int i;

	for (i = 0; i < chip->npwm; i++) {
		struct pwm_device *pwm = &chip->pwms[i];

		seq_printf(s, " pwm-%-3d (%-20.20s):", i, pwm->label);

		if (test_bit(PWMF_REQUESTED, &pwm->flags))
			seq_printf(s, " requested");

		if (test_bit(PWMF_ENABLED, &pwm->flags))
			seq_printf(s, " enabled");

		if (test_bit(PWMF_EXPORT, &pwm->flags))
			seq_printf(s, " sysfs_exported");

		seq_printf(s, "\n");
	}
}

static void *pwm_seq_start(struct seq_file *s, loff_t *pos)
{
	mutex_lock(&pwm_lock);
	s->private = "";

	return seq_list_start(&pwm_chips, *pos);
}

static void *pwm_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	s->private = "\n";

	return seq_list_next(v, &pwm_chips, pos);
}

static void pwm_seq_stop(struct seq_file *s, void *v)
{
	mutex_unlock(&pwm_lock);
}

static int pwm_seq_show(struct seq_file *s, void *v)
{
	struct pwm_chip *chip = list_entry(v, struct pwm_chip, list);

	seq_printf(s, "%s%s/%s, %d PWM device%s\n", (char *)s->private,
		   chip->dev->bus ? chip->dev->bus->name : "no-bus",
		   dev_name(chip->dev), chip->npwm,
		   (chip->npwm != 1) ? "s" : "");

	if (chip->ops->dbg_show)
		chip->ops->dbg_show(chip, s);
	else
		pwm_dbg_show(chip, s);

	return 0;
}

static const struct seq_operations pwm_seq_ops = {
	.start = pwm_seq_start,
	.next = pwm_seq_next,
	.stop = pwm_seq_stop,
	.show = pwm_seq_show,
};

static int pwm_seq_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &pwm_seq_ops);
}

static const struct file_operations pwm_debugfs_ops = {
	.owner = THIS_MODULE,
	.open = pwm_seq_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static int __init pwm_debugfs_init(void)
{
	debugfs_create_file("pwm", S_IFREG | S_IRUGO, NULL, NULL,
			    &pwm_debugfs_ops);

	return 0;
}

subsys_initcall(pwm_debugfs_init);
#endif /* CONFIG_DEBUG_FS */

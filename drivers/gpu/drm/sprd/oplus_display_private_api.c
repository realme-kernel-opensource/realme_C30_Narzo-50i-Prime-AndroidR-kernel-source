/***************************************************************
** Copyright (C),  2018,  oplus Mobile Comm Corp.,  Ltd
** File : oplus_display_private_api.h
** Description : oplus display private api implement
** Version : 1.0
** Date : 2018/03/20
******************************************************************/
#include "oplus_display_private_api.h"

#include <linux/fb.h>
#include <linux/time.h>
#include <linux/timekeeping.h>
#include <linux/moduleparam.h>

/*
 * we will create a sysfs which called /sys/kernel/oplus_display,
 * In that directory, oplus display private api can be called
 */
unsigned int tp_gesture = 0;

#if 0
unsigned long CABC_mode = 0;
unsigned int flag_bl;


// extern int sprd_panel_cabc_mode(unsigned int level);
extern int sprd_panel_cabc(unsigned int level);
static ssize_t cabc_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
    printk("%s CABC_mode=%ld\n", __func__, CABC_mode);
    return sprintf(buf, "%ld\n", CABC_mode);
}

static ssize_t cabc_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t num)
{
    int ret = 0;
    ret = kstrtoul(buf, 10, &CABC_mode);
    printk("%s CABC_mode=%ld\n", __func__, CABC_mode);
    if (flag_bl ==0) {
        return 0;
	}
	ret = sprd_panel_cabc((unsigned int)CABC_mode);
	return num;
}

static struct kobj_attribute dev_attr_cabc =
	__ATTR(cabc, 0664, cabc_show, cabc_store);
#endif


/*add /sys/kernel/oplus_display/tp_gesture*/
static ssize_t tp_gesture_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
    printk("%s tp_gesture=%d\n", __func__, tp_gesture);
    return sprintf(buf, "%d\n", tp_gesture);
}

static ssize_t tp_gesture_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t num)
{
    int ret = 0;
    ret = kstrtouint(buf, 10, &tp_gesture);
    printk("%s tp_gesture_mode=%d\n", __func__, tp_gesture);

	return num;
}

static struct kobj_attribute dev_attr_tp_gesture =
	__ATTR(tp_gesture, 0664, tp_gesture_show, tp_gesture_store);


static struct kobject *oplus_display_kobj;

//static DEVICE_ATTR_RW(cabc);

static bool support_mipi_cmd = true;
module_param(support_mipi_cmd, bool, 0644);
unsigned long lcd_id;
static int __init lcd_id_get(char *str)
{
	int ret;
	if (str != NULL)
		ret = kstrtoul(str, 16, &lcd_id);

	return 0;
}
__setup("lcd_id=ID", lcd_id_get);

static ssize_t lcd_id_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	int ret;

	if (support_mipi_cmd) {
		ret = scnprintf(buf, PAGE_SIZE, "LCM ID[%x]: 0x%x 0x%x\n", 0, lcd_id, 0);
	} else {
		ret = scnprintf(buf, PAGE_SIZE, "LCM ID[00]: 0x00 0x00\n");
	}

	return ret;
}

static ssize_t lcd_id_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t num)
{
	return num;
}
//static DEVICE_ATTR(panel_id, S_IRUGO|S_IWUSR, lcd_id_show, lcd_id_store);
static struct kobj_attribute dev_attr_panel_id =
	__ATTR(panel_id, 0664, lcd_id_show, lcd_id_store);
/*
 * Create a group of attributes so that we can create and destroy them all
 * at once.
 */
static struct attribute *oplus_display_attrs[] = {
//	&dev_attr_cabc.attr,
	&dev_attr_panel_id.attr,
	&dev_attr_tp_gesture.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group oplus_display_attr_group = {
	.attrs = oplus_display_attrs,
};

static int __init oplus_display_private_api_init(void)
{
	int retval;

	oplus_display_kobj = kobject_create_and_add("oplus_display", kernel_kobj);
	if (!oplus_display_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(oplus_display_kobj, &oplus_display_attr_group);
	if (retval)
		kobject_put(oplus_display_kobj);

	return retval;
}

static void __exit oplus_display_private_api_exit(void)
{
	kobject_put(oplus_display_kobj);
}

module_init(oplus_display_private_api_init);
module_exit(oplus_display_private_api_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Hujie <hujie@oplus.com>");

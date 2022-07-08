/*
 * Copyright (C) 2018 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drm_atomic_helper.h>
#include <linux/backlight.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <video/mipi_display.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>
#include "sprd_dpu.h"
#include "sprd_panel.h"
#include "dsi/sprd_dsi_api.h"
#include "sysfs/sysfs_display.h"
#include <linux/notifier.h>
#include <linux/tp_pannel_notifier.h>

#define SPRD_MIPI_DSI_FMT_DSC 0xff
static DEFINE_MUTEX(panel_lock);

int g_black_test = 0;

extern unsigned int tp_gesture;

static int pre_brightness = 0;

extern void ili_resume_by_ddi(void);

extern void lp6280_set_voltage(unsigned int vol);
const char *lcd_name;
const char *bootmode = "normal";
int control_ibus_brightness;
static int __init lcd_name_get(char *str)
{
	if (str != NULL)
		lcd_name = str;
	DRM_INFO("lcd name from uboot: %s\n", lcd_name);
	return 0;
}
__setup("lcd_name=", lcd_name_get);
static int __init bootmode_get(char *str)
{
        if (str != NULL)
             bootmode = str;
        DRM_INFO("bootmode from uboot: %s\n", bootmode);
        return 0;
}
__setup("androidboot.mode=", bootmode_get);

static inline struct sprd_panel *to_sprd_panel(struct drm_panel *panel)
{
	return container_of(panel, struct sprd_panel, base);
}

static int sprd_panel_send_cmds(struct mipi_dsi_device *dsi,
				const void *data, int size)
{
	struct sprd_panel *panel;
	const struct dsi_cmd_desc *cmds = data;
	u16 len;

	if ((cmds == NULL) || (dsi == NULL))
		return -EINVAL;

	panel = mipi_dsi_get_drvdata(dsi);

	while (size > 0) {
		len = (cmds->wc_h << 8) | cmds->wc_l;

		if (panel->info.use_dcs)
			mipi_dsi_dcs_write_buffer(dsi, cmds->payload, len);
		else
			mipi_dsi_generic_write(dsi, cmds->payload, len);

		if (cmds->wait)
			msleep(cmds->wait);
		cmds = (const struct dsi_cmd_desc *)(cmds->payload + len);
		size -= (len + 4);
	}

	return 0;
}

static int sprd_panel_unprepare(struct drm_panel *p)
{
	struct sprd_panel *panel = to_sprd_panel(p);
	struct gpio_timing *timing;
	int items, i;

    DRM_INFO("%s(): g_black_test and tp_gesture = 0 will turn off power. g_black_test is %d, tp_gesture is %d\n",__func__, g_black_test,tp_gesture);
	if (tp_gesture == 0 && g_black_test == 0){
		DRM_INFO("%s(): g_black_test is %d, tp_gesture is %d\n",__func__, g_black_test,tp_gesture);
		if (panel->info.avee_gpio) {
			gpiod_direction_output(panel->info.avee_gpio, 0);
			mdelay(panel->info.power_gpio_delay);
		}

		if (panel->info.avdd_gpio) {
			gpiod_direction_output(panel->info.avdd_gpio, 0);
			mdelay(5);
		}
	}
		if (panel->info.reset_gpio) {
			items = panel->info.rst_off_seq.items;
			timing = panel->info.rst_off_seq.timing;
			for (i = 0; i < items; i++) {
				gpiod_direction_output(panel->info.reset_gpio,
							timing[i].level);
				mdelay(timing[i].delay);
			}
		}

		regulator_disable(panel->supply);
		
	return 0;
}

static int sprd_panel_prepare(struct drm_panel *p)
{
	struct sprd_panel *panel = to_sprd_panel(p);
	struct gpio_timing *timing;
	int items, i, ret;

	DRM_INFO("%s()\n", __func__);
	if (tp_gesture == 0 && g_black_test == 0){
		DRM_INFO("%s(): power on g_black_test is %d, tp_gesture is %d.\n", __func__, g_black_test,tp_gesture);
		ret = regulator_enable(panel->supply);
		if (ret < 0)
			DRM_ERROR("enable lcd regulator failed\n");

		if (panel->info.avdd_gpio) {
			gpiod_direction_output(panel->info.avdd_gpio, 1);
			mdelay(panel->info.power_gpio_delay);
		}

		if (panel->info.avee_gpio) {
			gpiod_direction_output(panel->info.avee_gpio, 1);
			mdelay(5);
		}
		lp6280_set_voltage(6000);
	}
	if (panel->info.reset_gpio) {
		items = panel->info.rst_on_seq.items;
		timing = panel->info.rst_on_seq.timing;
		for (i = 0; i < items; i++) {
			gpiod_direction_output(panel->info.reset_gpio,
						timing[i].level);
			mdelay(timing[i].delay);
		}
	}

	//ili9882q tp add resume
	if (strncmp("lcd_ili9882q_txd_mipi_hd", lcd_name, 28) == 0) {
		//ili_resume_by_ddi();
		DRM_INFO("%s()notify tp into resume\n", __func__);
		tp_pannel_notifier_call_chain(TOUCHSCREEN_RESUME,NULL);
	}

	return 0;
}

void  sprd_panel_enter_doze(struct drm_panel *p)
{
	struct sprd_panel *panel = to_sprd_panel(p);

	DRM_INFO("%s() enter\n", __func__);

	mutex_lock(&panel_lock);

	if (panel->esd_work_pending) {
		cancel_delayed_work_sync(&panel->esd_work);
		panel->esd_work_pending = false;
	}

	sprd_panel_send_cmds(panel->slave,
	       panel->info.cmds[CMD_CODE_DOZE_IN],
	       panel->info.cmds_len[CMD_CODE_DOZE_IN]);

	mutex_unlock(&panel_lock);
}

void  sprd_panel_exit_doze(struct drm_panel *p)
{
	struct sprd_panel *panel = to_sprd_panel(p);

	DRM_INFO("%s() enter\n", __func__);

	mutex_lock(&panel_lock);

	sprd_panel_send_cmds(panel->slave,
		panel->info.cmds[CMD_CODE_DOZE_OUT],
		panel->info.cmds_len[CMD_CODE_DOZE_OUT]);

	if (panel->info.esd_check_en) {
		schedule_delayed_work(&panel->esd_work,
				      msecs_to_jiffies(1000));
		panel->esd_work_pending = true;
	}

	mutex_unlock(&panel_lock);
}

static int sprd_panel_disable(struct drm_panel *p)
{
	struct sprd_panel *panel = to_sprd_panel(p);

	DRM_INFO("%s()\n", __func__);

	/*
	 * FIXME:
	 * The cancel work should be executed before DPU stop,
	 * otherwise the esd check will be failed if the DPU
	 * stopped in video mode and the DSI has not change to
	 * CMD mode yet. Since there is no VBLANK timing for
	 * LP cmd transmission.
	 */
	if (panel->esd_work_pending) {
		cancel_delayed_work_sync(&panel->esd_work);
		panel->esd_work_pending = false;
	}

	mutex_lock(&panel_lock);
	if (panel->backlight) {
		panel->backlight->props.power = FB_BLANK_POWERDOWN;
		panel->backlight->props.state |= BL_CORE_FBBLANK;
		backlight_update_status(panel->backlight);
	}

	//ili9882q tp add suspend
	if (strncmp("lcd_ili9882q_txd_mipi_hd", lcd_name, 28) == 0) {
		  DRM_INFO("%s() notify tp into suspend\n", __func__);
		  tp_pannel_notifier_call_chain(TOUCHSCREEN_SUSPEND,NULL);
		  msleep(5);
	}

	sprd_panel_send_cmds(panel->slave,
			     panel->info.cmds[CMD_CODE_SLEEP_IN],
			     panel->info.cmds_len[CMD_CODE_SLEEP_IN]);

	panel->is_enabled = false;
	mutex_unlock(&panel_lock);

	return 0;
}

static int sprd_panel_enable(struct drm_panel *p)
{
	struct sprd_panel *panel = to_sprd_panel(p);

	DRM_INFO("%s()\n", __func__);

	mutex_lock(&panel_lock);
	sprd_panel_send_cmds(panel->slave,
			     panel->info.cmds[CMD_CODE_INIT],
			     panel->info.cmds_len[CMD_CODE_INIT]);

	if (panel->backlight) {
		panel->backlight->props.power = FB_BLANK_UNBLANK;
		panel->backlight->props.state &= ~BL_CORE_FBBLANK;
		backlight_update_status(panel->backlight);
	}

	if (panel->info.esd_check_en) {
		schedule_delayed_work(&panel->esd_work,
				      msecs_to_jiffies(1000));
		panel->esd_work_pending = true;
		panel->esd_work_backup = false;
	}

	panel->is_enabled = true;
	mutex_unlock(&panel_lock);

	return 0;
}

static int sprd_panel_get_modes(struct drm_panel *p)
{
	struct drm_display_mode *mode;
	struct sprd_panel *panel = to_sprd_panel(p);
	struct device_node *np = panel->slave->dev.of_node;
	u32 surface_width = 0, surface_height = 0;
	int i, mode_count = 0;

	DRM_INFO("%s()\n", __func__);

	/*
	 * Only include timing0 for preferred mode. if it defines "native-mode"
	 * property in dts, whether lcd timing in dts is in order or reverse
	 * order. it can parse timing0 about func "of_get_drm_display_mode".
	 * so it all matches correctly timimg0 for perferred mode.
	 */
	mode = drm_mode_duplicate(p->drm, &panel->info.mode);
	if (!mode) {
		DRM_ERROR("failed to alloc mode %s\n", panel->info.mode.name);
		return 0;
	}
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(p->connector, mode);
	mode_count++;

	/*
	 * Don't include timing0 for default mode. if lcd timing in dts is in
	 * order, timing0 is the fist one. if lcd timing in dts is reserve
	 * order, timing0 is the last one.
	 */
	for (i = 0; i < panel->info.num_buildin_modes - 1; i++)	{
		mode = drm_mode_duplicate(p->drm,
			&(panel->info.buildin_modes[i]));
		if (!mode) {
			DRM_ERROR("failed to alloc mode %s\n",
				panel->info.buildin_modes[i].name);
			return 0;
		}
		mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_DEFAULT;
		drm_mode_probed_add(p->connector, mode);
		mode_count++;
	}

	of_property_read_u32(np, "sprd,surface-width", &surface_width);
	of_property_read_u32(np, "sprd,surface-height", &surface_height);
	if (surface_width && surface_height) {
		struct videomode vm = {};

		vm.hactive = surface_width;
		vm.vactive = surface_height;
		vm.pixelclock = surface_width * surface_height * 60;

		mode = drm_mode_create(p->drm);

		mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_BUILTIN |
			DRM_MODE_TYPE_CRTC_C;
		mode->vrefresh = 60;
		drm_display_mode_from_videomode(&vm, mode);
		drm_mode_probed_add(p->connector, mode);
		mode_count++;
	}

	p->connector->display_info.width_mm = panel->info.mode.width_mm;
	p->connector->display_info.height_mm = panel->info.mode.height_mm;

	return mode_count;
}

static const struct drm_panel_funcs sprd_panel_funcs = {
	.get_modes = sprd_panel_get_modes,
	.enable = sprd_panel_enable,
	.disable = sprd_panel_disable,
	.prepare = sprd_panel_prepare,
	.unprepare = sprd_panel_unprepare,
};

static int sprd_panel_esd_check(struct sprd_panel *panel)
{
	struct panel_info *info = &panel->info;
	u8 read_val = 0;
	struct sprd_dpu *dpu;

	if (!panel->base.connector ||
	    !panel->base.connector->encoder ||
	    !panel->base.connector->encoder->crtc) {
		return 0;
	}

	mutex_lock(&panel_lock);
	if (!panel->is_enabled) {
		DRM_INFO("panel is not enabled, skip esd check");
		mutex_unlock(&panel_lock);
		return 0;
	}

	dpu = container_of(panel->base.connector->encoder->crtc,
		struct sprd_dpu, crtc);

	mutex_lock(&dpu->ctx.vrr_lock);

	/* FIXME: we should enable HS cmd tx here */
	mipi_dsi_set_maximum_return_packet_size(panel->slave, 1);
	mipi_dsi_dcs_read(panel->slave, info->esd_check_reg,
			  &read_val, 1);

	mutex_unlock(&dpu->ctx.vrr_lock);
	/*
	 * TODO:
	 * Should we support multi-registers check in the future?
	 */
	if (read_val != info->esd_check_val) {
		DRM_ERROR("esd check failed, read value = 0x%02x\n",
			  read_val);
		mutex_unlock(&panel_lock);
		return -EINVAL;
	}
	mutex_unlock(&panel_lock);

	return 0;
}

static int sprd_panel_te_check(struct sprd_panel *panel)
{
	static int te_wq_inited;
	struct sprd_dpu *dpu;
	int ret;
	bool irq_occur = false;

	if (!panel->base.connector ||
	    !panel->base.connector->encoder ||
	    !panel->base.connector->encoder->crtc) {
		return 0;
	}

	dpu = container_of(panel->base.connector->encoder->crtc,
		struct sprd_dpu, crtc);

	if (!te_wq_inited) {
		init_waitqueue_head(&dpu->ctx.te_wq);
		te_wq_inited = 1;
		dpu->ctx.evt_te = false;
		DRM_INFO("%s init te waitqueue\n", __func__);
	}

	/* DPU TE irq maybe enabled in kernel */
	if (!dpu->ctx.is_inited)
		return 0;

	dpu->ctx.te_check_en = true;

	/* wait for TE interrupt */
	ret = wait_event_interruptible_timeout(dpu->ctx.te_wq,
		dpu->ctx.evt_te, msecs_to_jiffies(500));
	if (!ret) {
		/* double check TE interrupt through dpu_int_raw register */
		if (dpu->core && dpu->core->check_raw_int) {
			down(&dpu->ctx.refresh_lock);
			if (dpu->ctx.is_inited)
				irq_occur = dpu->core->check_raw_int(&dpu->ctx,
					DISPC_INT_TE_MASK);
			up(&dpu->ctx.refresh_lock);
			if (!irq_occur) {
				DRM_ERROR("TE esd timeout.\n");
				ret = -1;
			} else
				DRM_WARN("TE occur, but isr schedule delay\n");
		} else {
			DRM_ERROR("TE esd timeout.\n");
			ret = -1;
		}
	}

	dpu->ctx.te_check_en = false;
	dpu->ctx.evt_te = false;

	return ret < 0 ? ret : 0;
}

static void sprd_panel_esd_work_func(struct work_struct *work)
{
	struct sprd_panel *panel = container_of(work, struct sprd_panel,
						esd_work.work);
	struct panel_info *info = &panel->info;
	int ret;
	u8 buf[] = {0x39, 0x00, 0x00, 0x03, 0x51, 0x00, 0x00};
	struct dsi_cmd_desc *brightness_cmd = (struct dsi_cmd_desc *)buf;

	if (info->esd_check_mode == ESD_MODE_REG_CHECK)
		ret = sprd_panel_esd_check(panel);
	else if (info->esd_check_mode == ESD_MODE_TE_CHECK)
		ret = sprd_panel_te_check(panel);
	else {
		DRM_ERROR("unknown esd check mode:%d\n", info->esd_check_mode);
		return;
	}

	if (ret && panel->base.connector && panel->base.connector->encoder) {
		const struct drm_encoder_helper_funcs *funcs;
		struct drm_encoder *encoder;

		encoder = panel->base.connector->encoder;
		funcs = encoder->helper_private;
		panel->esd_work_pending = false;

		if (!encoder->crtc || (encoder->crtc->state &&
		    !encoder->crtc->state->active)) {
			DRM_INFO("skip esd recovery during panel suspend\n");
			panel->esd_work_backup = true;
			return;
		}

		DRM_INFO("====== esd recovery start ========\n");
		funcs->disable(encoder);
		funcs->enable(encoder);
		brightness_cmd->payload[1] = (pre_brightness >> 8) & 0xFF;
		brightness_cmd->payload[2] = pre_brightness & 0xFF;
		sprd_panel_send_cmds(panel->slave, brightness_cmd, 3);
		if (!panel->esd_work_pending && panel->is_enabled)
			schedule_delayed_work(&panel->esd_work,
					msecs_to_jiffies(info->esd_check_period));
		DRM_INFO("======= esd recovery end =========\n");
	} else
		schedule_delayed_work(&panel->esd_work,
			msecs_to_jiffies(info->esd_check_period));
}

static int sprd_panel_gpio_request(struct device *dev,
			struct sprd_panel *panel)
{
	panel->info.avdd_gpio = devm_gpiod_get_optional(dev,
					"avdd", GPIOD_ASIS);
	if (IS_ERR_OR_NULL(panel->info.avdd_gpio))
		DRM_WARN("can't get panel avdd gpio: %ld\n",
				 PTR_ERR(panel->info.avdd_gpio));

	panel->info.avee_gpio = devm_gpiod_get_optional(dev,
					"avee", GPIOD_ASIS);
	if (IS_ERR_OR_NULL(panel->info.avee_gpio))
		DRM_WARN("can't get panel avee gpio: %ld\n",
				 PTR_ERR(panel->info.avee_gpio));

	panel->info.reset_gpio = devm_gpiod_get_optional(dev,
					"reset", GPIOD_ASIS);
	if (IS_ERR_OR_NULL(panel->info.reset_gpio))
		DRM_WARN("can't get panel reset gpio: %ld\n",
				 PTR_ERR(panel->info.reset_gpio));

	return 0;
}

static int of_parse_reset_seq(struct device_node *np,
				struct panel_info *info)
{
	struct property *prop;
	int bytes, rc;
	u32 *p;

	prop = of_find_property(np, "sprd,reset-on-sequence", &bytes);
	if (!prop) {
		DRM_ERROR("sprd,reset-on-sequence property not found\n");
		return -EINVAL;
	}

	p = kzalloc(bytes, GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	rc = of_property_read_u32_array(np, "sprd,reset-on-sequence",
					p, bytes / 4);
	if (rc) {
		DRM_ERROR("parse sprd,reset-on-sequence failed\n");
		kfree(p);
		return rc;
	}

	info->rst_on_seq.items = bytes / 8;
	info->rst_on_seq.timing = (struct gpio_timing *)p;

	prop = of_find_property(np, "sprd,reset-off-sequence", &bytes);
	if (!prop) {
		DRM_ERROR("sprd,reset-off-sequence property not found\n");
		return -EINVAL;
	}

	p = kzalloc(bytes, GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	rc = of_property_read_u32_array(np, "sprd,reset-off-sequence",
					p, bytes / 4);
	if (rc) {
		DRM_ERROR("parse sprd,reset-off-sequence failed\n");
		kfree(p);
		return rc;
	}

	info->rst_off_seq.items = bytes / 8;
	info->rst_off_seq.timing = (struct gpio_timing *)p;

	return 0;
}

static int sprd_get_drm_display_mode(struct panel_info *info,
					struct device_node *lcd_node)
{
	int i, ret, num_timings;
	struct device_node *timings_np;
	struct videomode *vm;
	struct display_timings *disp;

	timings_np = of_get_child_by_name(lcd_node, "display-timings");
	if (!timings_np) {
		DRM_ERROR("%s: can not find display-timings node\n", lcd_node->name);
		return -ENODEV;
	}

	num_timings = of_get_child_count(timings_np);
	if (num_timings == 0) {
		/* should never happen, as entry was already found above */
		DRM_ERROR("%s: no timings specified\n", lcd_node->name);
		return -EINVAL;
	}
	info->num_buildin_modes = num_timings;

	info->buildin_modes = kzalloc(sizeof(struct drm_display_mode) * num_timings, GFP_KERNEL);
	if (!info->buildin_modes) {
		DRM_ERROR("kzalloc buildin_modes failed!\n");
		ret = -ENOMEM;
		goto buildin_mode_fail;
	}

	vm = kzalloc(sizeof(struct videomode) * num_timings, GFP_KERNEL);
	if (!vm) {
		DRM_ERROR("kzalloc vm failed!\n");
		ret = -ENOMEM;
		goto vm_fail;
	}

	disp = of_get_display_timings(lcd_node);
	if (!disp) {
		DRM_ERROR("%pOF: no timings specified\n", lcd_node);
		ret = -ENODEV;
		goto disp_fail;
	}

	for (i = 0; i < num_timings; i ++) {
		ret = videomode_from_timings(disp, &vm[i], i);
		if (ret) {
			DRM_ERROR("get display timing failed\n");
			goto timings_fail;
		}

		drm_display_mode_from_videomode(&vm[i], &info->buildin_modes[i]);
		DRM_INFO("%pOF: got %dx%d display mode from %s\n",
				lcd_node, vm[i].hactive, vm[i].vactive, lcd_node->name);

		info->buildin_modes[i].width_mm = info->mode.width_mm;
		info->buildin_modes[i].height_mm = info->mode.height_mm;
		info->buildin_modes[i].vrefresh = drm_mode_vrefresh(&info->buildin_modes[i]);

		/* Config native mode */
		if (i == disp->native_mode) {
			memcpy(&info->mode, &info->buildin_modes[i],
						sizeof(struct drm_display_mode));
			info->mode.vrefresh = drm_mode_vrefresh(&info->buildin_modes[i]);
		}
	}

	kfree(vm);
	display_timings_release(disp);
	return 0;

timings_fail:
	display_timings_release(disp);
disp_fail:
	kfree(vm);
vm_fail:
	kfree(info->buildin_modes);
buildin_mode_fail:
	return ret;
}

static int of_parse_buildin_modes(struct panel_info *info,
	struct device_node *lcd_node)
{
	int i, rc, num_timings;
	char timing_check_ct = 0;

	rc = sprd_get_drm_display_mode(info, lcd_node);
	if(rc)
		return rc;

	num_timings = info->num_buildin_modes;
	if(num_timings > 1) {
		for(i = 1; i < num_timings; i++)
			if(info->buildin_modes[0].htotal
					== info->buildin_modes[i].htotal)
				timing_check_ct ++;
		if(timing_check_ct == num_timings - 1)
			vrr_mode = true;
	}
	DRM_INFO("info->num_buildin_modes = %d, vrr_mode = %d\n",
					info->num_buildin_modes, vrr_mode);
	return 0;
}

static int of_parse_oled_cmds(struct sprd_oled *oled,
		const void *data, int size)
{
	const struct dsi_cmd_desc *cmds = data;
	struct dsi_cmd_desc *p;
	u16 len;
	int i, total;

	if (cmds == NULL)
		return -EINVAL;

	/*
	 * TODO:
	 * Currently, we only support the same length cmds
	 * for oled brightness level. So we take the first
	 * cmd payload length as all.
	 */
	len = (cmds->wc_h << 8) | cmds->wc_l;
	total =  size / (len + 4);

	p = (struct dsi_cmd_desc *)kzalloc(size, GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	memcpy(p, cmds, size);
	for (i = 0; i < total; i++) {
		oled->cmds[i] = p;
		p = (struct dsi_cmd_desc *)(p->payload + len);
	}

	oled->cmds_total = total;
	oled->cmd_len = len + 4;

	return 0;
}
static int map_exp[4096] = {0};
static int oplus_max_normal_brightness =3768;
static void init_global_exp_backlight(void)
{
	int lut_index[41] = {0, 4, 99, 144, 187, 227, 264, 300, 334, 366, 397, 427, 456, 484, 511, 537, 563, 587, 611, 635, 658, 680,
						702, 723, 744, 764, 784, 804, 823, 842, 861, 879, 897, 915, 933, 950, 967, 984, 1000, 1016, 1023};
	int lut_value1[41] = {0, 4, 6, 14, 24, 37, 52, 69, 87, 107, 128, 150, 173, 197, 222, 248, 275, 302, 330, 358, 387, 416, 446, 
						477, 507, 539, 570, 602, 634, 667, 700, 733, 767, 801, 835, 869, 903, 938, 973, 1008, 1023};
	int index_start = 0, index_end = 0;
	int value1_start = 0, value1_end = 0;
	int i,j;
	int index_len = sizeof(lut_index) / sizeof(int);
	int value_len = sizeof(lut_value1) / sizeof(int);
	if (index_len == value_len) {
		for (i = 0; i < index_len - 1; i++) {
			index_start = lut_index[i] * oplus_max_normal_brightness / 1023;
			index_end = lut_index[i+1] * oplus_max_normal_brightness / 1023;
			value1_start = lut_value1[i] * oplus_max_normal_brightness / 1023;
			value1_end = lut_value1[i+1] * oplus_max_normal_brightness / 1023;
			for (j = index_start; j <= index_end; j++) {
				map_exp[j] = value1_start + (value1_end - value1_start) * (j - index_start) / (index_end - index_start);
			}
		}
	}
}

int sprd_oled_set_brightness(struct backlight_device *bdev)
{
	int brightness;
	int bl_map;
	struct sprd_oled *oled = bl_get_data(bdev);
	struct sprd_panel *panel = oled->panel;

	mutex_lock(&panel_lock);
	if (!panel->is_enabled) {
		mutex_unlock(&panel_lock);
		DRM_WARN("oled panel has been powered off\n");
		return -ENXIO;
	}

	if(!pre_brightness && strstr(lcd_name, "ili9882q_txd"))
		msleep(15);
	if(!pre_brightness && strstr(lcd_name, "icnl9911c_hkc"))
		msleep(14);

	brightness = bdev->props.brightness;
	control_ibus_brightness = brightness;
	pre_brightness = brightness;
	bl_map = brightness;
	if ((brightness > 0) && (brightness < oplus_max_normal_brightness)) {
		bl_map = map_exp[brightness];
	}
	DRM_INFO("%s brightness: %d, bl_map: %d\n", __func__, brightness, bl_map);

	sprd_panel_send_cmds(panel->slave,
			     panel->info.cmds[CMD_OLED_REG_LOCK],
			     panel->info.cmds_len[CMD_OLED_REG_LOCK]);

	if (oled->cmds_total == 1) {
		if (oled->cmds[0]->wc_l == 3) {
			oled->cmds[0]->payload[1] = bl_map >> 8;
			oled->cmds[0]->payload[2] = bl_map & 0xFF;
		} else
			oled->cmds[0]->payload[1] = bl_map;

		sprd_panel_send_cmds(panel->slave,
			     oled->cmds[0],
			     oled->cmd_len);
	} else
		sprd_panel_send_cmds(panel->slave,
			     oled->cmds[bl_map],
			     oled->cmd_len);

	sprd_panel_send_cmds(panel->slave,
			     panel->info.cmds[CMD_OLED_REG_UNLOCK],
			     panel->info.cmds_len[CMD_OLED_REG_UNLOCK]);

	mutex_unlock(&panel_lock);

	return 0;
}

static const struct backlight_ops sprd_oled_backlight_ops = {
	.update_status = sprd_oled_set_brightness,
};

static int sprd_oled_backlight_init(struct sprd_panel *panel)
{
	struct sprd_oled *oled;
	struct device_node *oled_node;
	struct panel_info *info = &panel->info;
	const void *p;
	int bytes, rc;
	u32 temp;

	oled_node = of_get_child_by_name(info->of_node,
				"oled-backlight");
	if (!oled_node)
		return 0;

	oled = devm_kzalloc(&panel->dev,
			sizeof(struct sprd_oled), GFP_KERNEL);
	if (!oled)
		return -ENOMEM;

	oled->bdev = devm_backlight_device_register(&panel->dev,
			"sprd_backlight", &panel->dev, oled,
			&sprd_oled_backlight_ops, NULL);
	if (IS_ERR(oled->bdev)) {
		DRM_ERROR("failed to register oled backlight ops\n");
		return PTR_ERR(oled->bdev);
	}
	panel->oled_bdev = oled->bdev;
	p = of_get_property(oled_node, "brightness-levels", &bytes);
	if (p) {
		info->cmds[CMD_OLED_BRIGHTNESS] = p;
		info->cmds_len[CMD_OLED_BRIGHTNESS] = bytes;
	} else
		DRM_ERROR("can't find brightness-levels property\n");

	p = of_get_property(oled_node, "sprd,reg-lock", &bytes);
	if (p) {
		info->cmds[CMD_OLED_REG_LOCK] = p;
		info->cmds_len[CMD_OLED_REG_LOCK] = bytes;
	} else
		DRM_INFO("can't find sprd,reg-lock property\n");

	p = of_get_property(oled_node, "sprd,reg-unlock", &bytes);
	if (p) {
		info->cmds[CMD_OLED_REG_UNLOCK] = p;
		info->cmds_len[CMD_OLED_REG_UNLOCK] = bytes;
	} else
		DRM_INFO("can't find sprd,reg-unlock property\n");

	rc = of_property_read_u32(oled_node, "default-brightness-level", &temp);
	if (!rc)
		oled->bdev->props.brightness = temp;
	else
		oled->bdev->props.brightness = 25;

	rc = of_property_read_u32(oled_node, "sprd,max-level", &temp);
	if (!rc)
		oled->max_level = temp;
	else
		oled->max_level = 255;

	oled->bdev->props.max_brightness = oled->max_level;
	oled->panel = panel;
	of_parse_oled_cmds(oled,
			panel->info.cmds[CMD_OLED_BRIGHTNESS],
			panel->info.cmds_len[CMD_OLED_BRIGHTNESS]);

	DRM_INFO("%s() ok\n", __func__);

	return 0;
}

int sprd_panel_parse_lcddtb(struct device_node *lcd_node,
	struct sprd_panel *panel)
{
	u32 val;
	struct panel_info *info = &panel->info;
	int bytes, rc;
	const void *p;
	const char *str;

	if (!lcd_node) {
		DRM_ERROR("Lcd node from dtb is Null\n");
		return -ENODEV;
	}
	info->of_node = lcd_node;

	rc = of_property_read_u32(lcd_node, "sprd,dsi-work-mode", &val);
	if (!rc) {
		if (val == SPRD_DSI_MODE_CMD)
			info->mode_flags = 0;
		else if (val == SPRD_DSI_MODE_VIDEO_BURST)
			info->mode_flags = MIPI_DSI_MODE_VIDEO |
					   MIPI_DSI_MODE_VIDEO_BURST;
		else if (val == SPRD_DSI_MODE_VIDEO_SYNC_PULSE)
			info->mode_flags = MIPI_DSI_MODE_VIDEO |
					   MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
		else if (val == SPRD_DSI_MODE_VIDEO_SYNC_EVENT)
			info->mode_flags = MIPI_DSI_MODE_VIDEO;
	} else {
		DRM_ERROR("dsi work mode is not found! use video mode\n");
		info->mode_flags = MIPI_DSI_MODE_VIDEO |
				   MIPI_DSI_MODE_VIDEO_BURST;
	}

	if (of_property_read_bool(lcd_node, "sprd,dsi-non-continuous-clock"))
		info->mode_flags |= MIPI_DSI_CLOCK_NON_CONTINUOUS;

	rc = of_property_read_u32(lcd_node, "sprd,dsi-lane-number", &val);
	if (!rc)
		info->lanes = val;
	else
		info->lanes = 4;

	rc = of_property_read_string(lcd_node, "sprd,dsi-color-format", &str);
	if (rc)
		info->format = MIPI_DSI_FMT_RGB888;
	else if (!strcmp(str, "rgb888"))
		info->format = MIPI_DSI_FMT_RGB888;
	else if (!strcmp(str, "rgb666"))
		info->format = MIPI_DSI_FMT_RGB666;
	else if (!strcmp(str, "rgb666_packed"))
		info->format = MIPI_DSI_FMT_RGB666_PACKED;
	else if (!strcmp(str, "rgb565"))
		info->format = MIPI_DSI_FMT_RGB565;
	else if (!strcmp(str, "dsc"))
		info->format = SPRD_MIPI_DSI_FMT_DSC;
	else
		DRM_ERROR("dsi-color-format (%s) is not supported\n", str);

	rc = of_property_read_u32(lcd_node, "sprd,width-mm", &val);
	if (!rc)
		info->mode.width_mm = val;
	else
		info->mode.width_mm = 68;

	rc = of_property_read_u32(lcd_node, "sprd,height-mm", &val);
	if (!rc)
		info->mode.height_mm = val;
	else
		info->mode.height_mm = 121;

	rc = of_property_read_u32(lcd_node, "sprd,esd-check-enable", &val);
	if (!rc)
		info->esd_check_en = val;

	rc = of_property_read_u32(lcd_node, "sprd,esd-check-mode", &val);
	if (!rc)
		info->esd_check_mode = val;
	else
		info->esd_check_mode = 1;

	rc = of_property_read_u32(lcd_node, "sprd,esd-check-period", &val);
	if (!rc)
		info->esd_check_period = val;
	else
		info->esd_check_period = 1000;

	rc = of_property_read_u32(lcd_node, "sprd,esd-check-register", &val);
	if (!rc)
		info->esd_check_reg = val;
	else
		info->esd_check_reg = 0x0A;

	rc = of_property_read_u32(lcd_node, "sprd,esd-check-value", &val);
	if (!rc)
		info->esd_check_val = val;
	else
		info->esd_check_val = 0x9C;

	rc = of_property_read_u32(lcd_node, "sprd,power-gpio-delay", &val);
	if (!rc)
		info->power_gpio_delay = val;
	else
		info->power_gpio_delay = 5;

	if (of_property_read_bool(lcd_node, "sprd,use-dcs-write"))
		info->use_dcs = true;
	else
		info->use_dcs = false;

	rc = of_parse_reset_seq(lcd_node, info);
	if (rc)
		DRM_ERROR("parse lcd reset sequence failed\n");

	p = of_get_property(lcd_node, "sprd,initial-command", &bytes);
	if (p) {
		info->cmds[CMD_CODE_INIT] = p;
		info->cmds_len[CMD_CODE_INIT] = bytes;
	} else
		DRM_ERROR("can't find sprd,initial-command property\n");

	p = of_get_property(lcd_node, "sprd,sleep-in-command", &bytes);
	if (p) {
		info->cmds[CMD_CODE_SLEEP_IN] = p;
		info->cmds_len[CMD_CODE_SLEEP_IN] = bytes;
	} else
		DRM_ERROR("can't find sprd,sleep-in-command property\n");

	p = of_get_property(lcd_node, "sprd,sleep-out-command", &bytes);
	if (p) {
		info->cmds[CMD_CODE_SLEEP_OUT] = p;
		info->cmds_len[CMD_CODE_SLEEP_OUT] = bytes;
	} else
		DRM_ERROR("can't find sprd,sleep-out-command property\n");

	p = of_get_property(lcd_node, "sprd,doze-in-command", &bytes);
	if (p) {
		info->cmds[CMD_CODE_DOZE_IN] = p;
		info->cmds_len[CMD_CODE_DOZE_IN] = bytes;
	} else
		DRM_INFO("can't find sprd,doze-in-command property\n");

	p = of_get_property(lcd_node, "sprd,doze-out-command", &bytes);
	if (p) {
		info->cmds[CMD_CODE_DOZE_OUT] = p;
		info->cmds_len[CMD_CODE_DOZE_OUT] = bytes;
	} else
		DRM_INFO("can't find sprd,doze-out-command property\n");

	of_parse_buildin_modes(info, lcd_node);

	return 0;
}

static int sprd_panel_parse_dt(struct device_node *np, struct sprd_panel *panel)
{
	struct device_node *lcd_node;
	int rc;
	const char *str;
	char lcd_path[60];

	rc = of_property_read_string(np, "sprd,force-attached", &str);
	if (!rc)
		lcd_name = str;

	sprintf(lcd_path, "/lcds/%s", lcd_name);
	lcd_node = of_find_node_by_path(lcd_path);
	if (!lcd_node) {
		DRM_ERROR("%pOF: could not find %s node\n", np, lcd_name);
		return -ENODEV;
	}
	rc = sprd_panel_parse_lcddtb(lcd_node, panel);
	if (rc)
		return rc;

	return 0;
}

static int sprd_panel_device_create(struct device *parent,
				    struct sprd_panel *panel)
{
	panel->dev.class = display_class;
	panel->dev.parent = parent;
	panel->dev.of_node = panel->info.of_node;
	dev_set_name(&panel->dev, "panel0");
	dev_set_drvdata(&panel->dev, panel);

	return device_register(&panel->dev);
}

static int sprd_panel_probe(struct mipi_dsi_device *slave)
{
	int ret;
	struct sprd_panel *panel;
	struct device_node *bl_node;

	panel = devm_kzalloc(&slave->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	bl_node = of_parse_phandle(slave->dev.of_node,
					"sprd,backlight", 0);
	if (bl_node) {
		panel->backlight = of_find_backlight_by_node(bl_node);
		of_node_put(bl_node);

		if (panel->backlight) {
			panel->backlight->props.state &= ~BL_CORE_FBBLANK;
			panel->backlight->props.power = FB_BLANK_UNBLANK;
			backlight_update_status(panel->backlight);
		} else {
			DRM_WARN("backlight is not ready, panel probe deferred\n");
			return -EPROBE_DEFER;
		}
	} else
		DRM_WARN("backlight node not found\n");

	panel->supply = devm_regulator_get(&slave->dev, "power");
	if (IS_ERR(panel->supply)) {
		if (PTR_ERR(panel->supply) == -EPROBE_DEFER)
			DRM_ERROR("regulator driver not initialized, probe deffer\n");
		else
			DRM_ERROR("can't get regulator: %ld\n", PTR_ERR(panel->supply));

		return PTR_ERR(panel->supply);
	}

	INIT_DELAYED_WORK(&panel->esd_work, sprd_panel_esd_work_func);

	ret = sprd_panel_parse_dt(slave->dev.of_node, panel);
	if (ret) {
		DRM_ERROR("parse panel info failed\n");
		return ret;
	}

	ret = sprd_panel_gpio_request(&slave->dev, panel);
	if (ret) {
		DRM_WARN("gpio is not ready, panel probe deferred\n");
		return -EPROBE_DEFER;
	}

	ret = sprd_panel_device_create(&slave->dev, panel);
	if (ret) {
		DRM_ERROR("panel device create failed\n");
		return ret;
	}

	ret = sprd_oled_backlight_init(panel);
	if (ret) {
		DRM_ERROR("oled backlight init failed\n");
		return ret;
	}

	panel->base.dev = &panel->dev;
	panel->base.funcs = &sprd_panel_funcs;
	drm_panel_init(&panel->base);

	ret = drm_panel_add(&panel->base);
	if (ret) {
		DRM_ERROR("drm_panel_add() failed\n");
		return ret;
	}

	slave->lanes = panel->info.lanes;
	slave->format = panel->info.format;
	slave->mode_flags = panel->info.mode_flags;

	ret = mipi_dsi_attach(slave);
	if (ret) {
		DRM_ERROR("failed to attach dsi panel to host\n");
		drm_panel_remove(&panel->base);
		return ret;
	}
	panel->slave = slave;

	sprd_panel_sysfs_init(&panel->dev);
	mipi_dsi_set_drvdata(slave, panel);

	/*
	 * FIXME:
	 * The esd check work should not be scheduled in probe
	 * function. It should be scheduled in the enable()
	 * callback function. But the dsi encoder will not call
	 * drm_panel_enable() the first time in encoder_enable().
	 */
	if (panel->info.esd_check_en) {
		schedule_delayed_work(&panel->esd_work,
				      msecs_to_jiffies(2000));
		panel->esd_work_pending = true;
	}

	init_global_exp_backlight();
	if (!strcmp(bootmode, "silent")){
		panel->is_enabled = false;
	} else {
		panel->is_enabled = true;
	}

	DRM_INFO("panel driver probe success\n");

	return 0;
}

static int sprd_panel_remove(struct mipi_dsi_device *slave)
{
	struct sprd_panel *panel = mipi_dsi_get_drvdata(slave);
	int ret;

	DRM_INFO("%s()\n", __func__);

	sprd_panel_disable(&panel->base);
	sprd_panel_unprepare(&panel->base);

	ret = mipi_dsi_detach(slave);
	if (ret < 0)
		DRM_ERROR("failed to detach from DSI host: %d\n", ret);

	drm_panel_detach(&panel->base);
	drm_panel_remove(&panel->base);

	return 0;
}

static void sprd_panel_shutdown(struct mipi_dsi_device *slave)
{
	struct sprd_panel *panel = mipi_dsi_get_drvdata(slave);

	DRM_INFO("%s()\n", __func__);

   if (tp_gesture == 1) {
		if (strstr(lcd_name, "ili9882q_txd") != NULL){
			DRM_INFO("%s(): %s avee avdd power off when tp_gesture is enable.\n",__func__, lcd_name);
			if (panel->info.avee_gpio) {
				gpiod_direction_output(panel->info.avee_gpio, 0);
				mdelay(panel->info.power_gpio_delay);
			}

			if (panel->info.avdd_gpio) {
				gpiod_direction_output(panel->info.avdd_gpio, 0);
				mdelay(5);
			}

     	   /* if (panel->info.reset_gpio) {
	                   gpiod_direction_output(panel->info.reset_gpio, 0);
                 }*/
       } else if ((strstr(lcd_name, "icnl9911c_hkc") != NULL)||(strstr(lcd_name, "icnl9911c_auo") != NULL)){
			DRM_INFO("%s(): %s lcd_rst avee avdd power off when tp_gesture is enable.\n",__func__, lcd_name);
			if (panel->info.reset_gpio) {
				 gpiod_direction_output(panel->info.reset_gpio, 0);
				 mdelay(5);
			}

			if (panel->info.avee_gpio) {
				gpiod_direction_output(panel->info.avee_gpio, 0);
				mdelay(panel->info.power_gpio_delay);
			}

			if (panel->info.avdd_gpio) {
				gpiod_direction_output(panel->info.avdd_gpio, 0);
				mdelay(5);
			}
       }
   }
}

static const struct of_device_id panel_of_match[] = {
	{ .compatible = "sprd,generic-mipi-panel", },
	{ }
};
MODULE_DEVICE_TABLE(of, panel_of_match);

static struct mipi_dsi_driver sprd_panel_driver = {
	.driver = {
		.name = "sprd-mipi-panel-drv",
		.of_match_table = panel_of_match,
	},
	.probe = sprd_panel_probe,
	.remove = sprd_panel_remove,
	.shutdown = sprd_panel_shutdown,
};
module_mipi_dsi_driver(sprd_panel_driver);

MODULE_AUTHOR("Leon He <leon.he@unisoc.com>");
MODULE_DESCRIPTION("SPRD MIPI DSI Panel Driver");
MODULE_LICENSE("GPL v2");

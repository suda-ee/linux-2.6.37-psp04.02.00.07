/*
 * Innolux AT70TN83 panel support
 *
 * Copyright (C) 2008 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>

#include <plat/display.h>

static struct omap_video_timings innolux_panel_timings = {
	/* 800 x 480 @ 68 Hz  Reduced blanking VESA CVT 0.31M3-R */
	.x_res		= 800,
	.y_res		= 480,
	.pixel_clock	= 28800,
	.hfp		= 40,
	.hsw		= 48,
	.hbp		= 40,
	.vfp		= 13,
	.vsw		= 3,
	.vbp		= 29,
};

static int innolux_panel_power_on(struct omap_dss_device *dssdev)
{
	int r;

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err1;
	}

	return 0;
err1:
	omapdss_dpi_display_disable(dssdev);
err0:
	return r;
}

static void innolux_panel_power_off(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	omapdss_dpi_display_disable(dssdev);
}

static int innolux_panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config |= OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS
                                | OMAP_DSS_LCD_IHS;
	dssdev->panel.timings = innolux_panel_timings;

	return 0;
}

static void innolux_panel_remove(struct omap_dss_device *dssdev)
{
}

static int innolux_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = innolux_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void innolux_panel_disable(struct omap_dss_device *dssdev)
{
	innolux_panel_power_off(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int innolux_panel_suspend(struct omap_dss_device *dssdev)
{
	innolux_panel_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	return 0;
}

static int innolux_panel_resume(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = innolux_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void innolux_panel_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dpi_set_timings(dssdev, timings);
}

static void innolux_panel_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int innolux_panel_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	return dpi_check_timings(dssdev, timings);
}

static struct omap_dss_driver innolux_driver = {
	.probe		= innolux_panel_probe,
	.remove		= innolux_panel_remove,

	.enable		= innolux_panel_enable,
	.disable	= innolux_panel_disable,
	.suspend	= innolux_panel_suspend,
	.resume		= innolux_panel_resume,

	.set_timings	= innolux_panel_set_timings,
	.get_timings	= innolux_panel_get_timings,
	.check_timings	= innolux_panel_check_timings,

	.driver         = {
		.name   = "innolux_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init innolux_panel_drv_init(void)
{
	return omap_dss_register_driver(&innolux_driver);
}

static void __exit innolux_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&innolux_driver);
}

module_init(innolux_panel_drv_init);
module_exit(innolux_panel_drv_exit);
MODULE_LICENSE("GPL");

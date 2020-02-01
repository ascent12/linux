// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2012 Red Hat
 * based in parts on udlfb.c:
 * Copyright (C) 2009 Roberto De Ioris <roberto@unbit.it>
 * Copyright (C) 2009 Jaya Kumar <jayakumar.lkml@gmail.com>
 * Copyright (C) 2009 Bernie Thompson <bernie@plugable.com>
 */

#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_probe_helper.h>

#include "udl_drv.h"

static int udl_get_edid_block(void *data, u8 *buf, unsigned int block,
			       size_t len)
{
	int ret, i;
	u8 *read_buff;
	struct udl_device *udl = data;

	read_buff = kmalloc(2, GFP_KERNEL);
	if (!read_buff)
		return -1;

	for (i = 0; i < len; i++) {
		int bval = (i + block * EDID_LENGTH) << 8;
		ret = usb_control_msg(udl->udev,
				      usb_rcvctrlpipe(udl->udev, 0),
					  (0x02), (0x80 | (0x02 << 5)), bval,
					  0xA1, read_buff, 2, HZ);
		if (ret < 1) {
			DRM_ERROR("Read EDID byte %d failed err %x\n", i, ret);
			kfree(read_buff);
			return -1;
		}
		buf[i] = read_buff[1];
	}

	kfree(read_buff);
	return 0;
}

static int udl_get_modes(struct drm_connector *connector)
{
	struct udl_device *udl = connector->dev->dev_private;

	drm_connector_update_edid_property(connector, udl->edid);
	if (udl->edid)
		return drm_add_edid_modes(connector, udl->edid);
	return 0;
}

static int udl_connector_detect(struct drm_connector *connector,
				struct drm_modeset_acquire_ctx *ctx,
				bool force)
{
	struct udl_device *udl = connector->dev->dev_private;

	/* cleanup previous edid */
	if (udl->edid != NULL) {
		kfree(udl->edid);
		udl->edid = NULL;
	}

	udl->edid = drm_do_get_edid(connector, udl_get_edid_block, udl);
	if (!udl->edid)
		return connector_status_disconnected;

	return connector_status_connected;
}

static enum drm_mode_status udl_mode_valid(struct drm_connector *connector,
			  struct drm_display_mode *mode)
{
	struct udl_device *udl = connector->dev->dev_private;
	if (!udl->sku_pixel_limit)
		return 0;

	if (mode->vdisplay * mode->hdisplay > udl->sku_pixel_limit)
		return MODE_VIRTUAL_Y;

	return 0;
}

static const struct drm_connector_helper_funcs udl_connector_helper_funcs = {
	.get_modes = udl_get_modes,
	.detect_ctx = udl_connector_detect,
	.mode_valid = udl_mode_valid,
};

static const struct drm_connector_funcs udl_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

int udl_connector_init(struct udl_device *udl)
{
	drm_connector_helper_add(&udl->conn, &udl_connector_helper_funcs);
	return drm_connector_init(&udl->drm, &udl->conn, &udl_connector_funcs,
				  DRM_MODE_CONNECTOR_DVII);
}

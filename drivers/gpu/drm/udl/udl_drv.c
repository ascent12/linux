// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2012 Red Hat
 */

#include <linux/module.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_file.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include "udl_drv.h"

/*
 * All DisplayLink bulk operations start with 0xAF, followed by specific code
 * All operations are written to buffers which then later get sent to device
 */
static char *udl_set_register(char *buf, u8 reg, u8 val)
{
	*buf++ = 0xAF;
	*buf++ = 0x20;
	*buf++ = reg;
	*buf++ = val;
	return buf;
}

static char *udl_vidreg_lock(char *buf)
{
	return udl_set_register(buf, 0xFF, 0x00);
}

static char *udl_vidreg_unlock(char *buf)
{
	return udl_set_register(buf, 0xFF, 0xFF);
}

/*
 * On/Off for driving the DisplayLink framebuffer to the display
 *  0x00 H and V sync on
 *  0x01 H and V sync off (screen blank but powered)
 *  0x07 DPMS powerdown (requires modeset to come back)
 */
static char *udl_set_blank(char *buf, int dpms_mode)
{
	u8 reg;
	switch (dpms_mode) {
	case DRM_MODE_DPMS_OFF:
		reg = 0x07;
		break;
	case DRM_MODE_DPMS_STANDBY:
		reg = 0x05;
		break;
	case DRM_MODE_DPMS_SUSPEND:
		reg = 0x01;
		break;
	case DRM_MODE_DPMS_ON:
		reg = 0x00;
		break;
	}

	return udl_set_register(buf, 0x1f, reg);
}

static char *udl_set_color_depth(char *buf, u8 selection)
{
	return udl_set_register(buf, 0x00, selection);
}

static char *udl_set_base16bpp(char *wrptr, u32 base)
{
	/* the base pointer is 16 bits wide, 0x20 is hi byte. */
	wrptr = udl_set_register(wrptr, 0x20, base >> 16);
	wrptr = udl_set_register(wrptr, 0x21, base >> 8);
	return udl_set_register(wrptr, 0x22, base);
}

/*
 * DisplayLink HW has separate 16bpp and 8bpp framebuffers.
 * In 24bpp modes, the low 323 RGB bits go in the 8bpp framebuffer
 */
static char *udl_set_base8bpp(char *wrptr, u32 base)
{
	wrptr = udl_set_register(wrptr, 0x26, base >> 16);
	wrptr = udl_set_register(wrptr, 0x27, base >> 8);
	return udl_set_register(wrptr, 0x28, base);
}

static char *udl_set_register_16(char *wrptr, u8 reg, u16 value)
{
	wrptr = udl_set_register(wrptr, reg, value >> 8);
	return udl_set_register(wrptr, reg+1, value);
}

/*
 * This is kind of weird because the controller takes some
 * register values in a different byte order than other registers.
 */
static char *udl_set_register_16be(char *wrptr, u8 reg, u16 value)
{
	wrptr = udl_set_register(wrptr, reg, value);
	return udl_set_register(wrptr, reg+1, value >> 8);
}

/*
 * LFSR is linear feedback shift register. The reason we have this is
 * because the display controller needs to minimize the clock depth of
 * various counters used in the display path. So this code reverses the
 * provided value into the lfsr16 value by counting backwards to get
 * the value that needs to be set in the hardware comparator to get the
 * same actual count. This makes sense once you read above a couple of
 * times and think about it from a hardware perspective.
 */
static u16 udl_lfsr16(u16 actual_count)
{
	u32 lv = 0xFFFF; /* This is the lfsr value that the hw starts with */

	while (actual_count--) {
		lv =	 ((lv << 1) |
			(((lv >> 15) ^ (lv >> 4) ^ (lv >> 2) ^ (lv >> 1)) & 1))
			& 0xFFFF;
	}

	return (u16) lv;
}

/*
 * This does LFSR conversion on the value that is to be written.
 * See LFSR explanation above for more detail.
 */
static char *udl_set_register_lfsr16(char *wrptr, u8 reg, u16 value)
{
	return udl_set_register_16(wrptr, reg, udl_lfsr16(value));
}

/*
 * This takes a standard fbdev screeninfo struct and all of its monitor mode
 * details and converts them into the DisplayLink equivalent register commands.
  ERR(vreg(dev,               0x00, (color_depth == 16) ? 0 : 1));
  ERR(vreg_lfsr16(dev,        0x01, xDisplayStart));
  ERR(vreg_lfsr16(dev,        0x03, xDisplayEnd));
  ERR(vreg_lfsr16(dev,        0x05, yDisplayStart));
  ERR(vreg_lfsr16(dev,        0x07, yDisplayEnd));
  ERR(vreg_lfsr16(dev,        0x09, xEndCount));
  ERR(vreg_lfsr16(dev,        0x0B, hSyncStart));
  ERR(vreg_lfsr16(dev,        0x0D, hSyncEnd));
  ERR(vreg_big_endian(dev,    0x0F, hPixels));
  ERR(vreg_lfsr16(dev,        0x11, yEndCount));
  ERR(vreg_lfsr16(dev,        0x13, vSyncStart));
  ERR(vreg_lfsr16(dev,        0x15, vSyncEnd));
  ERR(vreg_big_endian(dev,    0x17, vPixels));
  ERR(vreg_little_endian(dev, 0x1B, pixelClock5KHz));

  ERR(vreg(dev,               0x1F, 0));

  ERR(vbuf(dev, WRITE_VIDREG_UNLOCK, DSIZEOF(WRITE_VIDREG_UNLOCK)));
 */
static char *udl_set_vid_cmds(char *wrptr, struct drm_display_mode *mode)
{
	u16 xds, yds;
	u16 xde, yde;
	u16 yec;

	/* x display start */
	xds = mode->crtc_htotal - mode->crtc_hsync_start;
	wrptr = udl_set_register_lfsr16(wrptr, 0x01, xds);
	/* x display end */
	xde = xds + mode->crtc_hdisplay;
	wrptr = udl_set_register_lfsr16(wrptr, 0x03, xde);

	/* y display start */
	yds = mode->crtc_vtotal - mode->crtc_vsync_start;
	wrptr = udl_set_register_lfsr16(wrptr, 0x05, yds);
	/* y display end */
	yde = yds + mode->crtc_vdisplay;
	wrptr = udl_set_register_lfsr16(wrptr, 0x07, yde);

	/* x end count is active + blanking - 1 */
	wrptr = udl_set_register_lfsr16(wrptr, 0x09,
					mode->crtc_htotal - 1);

	/* libdlo hardcodes hsync start to 1 */
	wrptr = udl_set_register_lfsr16(wrptr, 0x0B, 1);

	/* hsync end is width of sync pulse + 1 */
	wrptr = udl_set_register_lfsr16(wrptr, 0x0D,
					mode->crtc_hsync_end - mode->crtc_hsync_start + 1);

	/* hpixels is active pixels */
	wrptr = udl_set_register_16(wrptr, 0x0F, mode->hdisplay);

	/* yendcount is vertical active + vertical blanking */
	yec = mode->crtc_vtotal;
	wrptr = udl_set_register_lfsr16(wrptr, 0x11, yec);

	/* libdlo hardcodes vsync start to 0 */
	wrptr = udl_set_register_lfsr16(wrptr, 0x13, 0);

	/* vsync end is width of vsync pulse */
	wrptr = udl_set_register_lfsr16(wrptr, 0x15, mode->crtc_vsync_end - mode->crtc_vsync_start);

	/* vpixels is active pixels */
	wrptr = udl_set_register_16(wrptr, 0x17, mode->crtc_vdisplay);

	wrptr = udl_set_register_16be(wrptr, 0x1B,
				      mode->clock / 5);

	return wrptr;
}

static char *udl_dummy_render(char *wrptr)
{
	*wrptr++ = 0xAF;
	*wrptr++ = 0x6A; /* copy */
	*wrptr++ = 0x00; /* from addr */
	*wrptr++ = 0x00;
	*wrptr++ = 0x00;
	*wrptr++ = 0x01; /* one pixel */
	*wrptr++ = 0x00; /* to address */
	*wrptr++ = 0x00;
	*wrptr++ = 0x00;
	return wrptr;
}

static int udl_crtc_write_mode_to_hw(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct udl_device *udl = dev->dev_private;
	struct urb *urb;
	char *buf;
	int retval;

	urb = udl_get_urb(dev);
	if (!urb)
		return -ENOMEM;

	buf = (char *)urb->transfer_buffer;

	memcpy(buf, udl->mode_buf, udl->mode_buf_len);
	retval = udl_submit_urb(dev, urb, udl->mode_buf_len);
	DRM_DEBUG("write mode info %d\n", udl->mode_buf_len);
	return retval;
}

static const struct drm_mode_config_funcs udl_mode_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static void udl_pipe_enable(struct drm_simple_display_pipe *pipe,
			    struct drm_crtc_state *crtc_state,
			    struct drm_plane_state *plane_state)
{
	struct udl_device *udl = pipe->crtc.dev->dev_private;
	struct drm_framebuffer *fb = plane_state->fb;
	struct drm_display_mode *mode = &crtc_state->mode;
	char *buf;
	char *wrptr;
	int color_depth = 0;

	buf = (char *)udl->mode_buf;

	/* for now we just clip 24 -> 16 - if we fix that fix this */
	/*if  (crtc->fb->bits_per_pixel != 16)
	  color_depth = 1; */

	/* This first section has to do with setting the base address on the
	* controller * associated with the display. There are 2 base
	* pointers, currently, we only * use the 16 bpp segment.
	*/
	wrptr = udl_vidreg_lock(buf);
	wrptr = udl_set_color_depth(wrptr, color_depth);
	/* set base for 16bpp segment to 0 */
	wrptr = udl_set_base16bpp(wrptr, 0);
	/* set base for 8bpp segment to end of fb */
	wrptr = udl_set_base8bpp(wrptr, 2 * mode->vdisplay * mode->hdisplay);

	wrptr = udl_set_vid_cmds(wrptr, mode);
	wrptr = udl_set_blank(wrptr, DRM_MODE_DPMS_ON);
	wrptr = udl_vidreg_unlock(wrptr);

	wrptr = udl_dummy_render(wrptr);

	udl->mode_buf_len = wrptr - buf;

	/* damage all of it */
	udl_handle_damage(fb, 0, 0, fb->width, fb->height);

	udl_crtc_write_mode_to_hw(&pipe->crtc);
}

static void udl_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct udl_device *udl = pipe->crtc.dev->dev_private;
	struct urb *urb;
	char *buf;

	urb = udl_get_urb(&udl->drm);
	if (!urb)
		return;

	buf = (char *)urb->transfer_buffer;
	buf = udl_vidreg_lock(buf);
	buf = udl_set_blank(buf, DRM_MODE_DPMS_OFF);
	buf = udl_vidreg_unlock(buf);

	buf = udl_dummy_render(buf);
	udl_submit_urb(&udl->drm, urb, buf - (char *) urb->transfer_buffer);
}

static int udl_pipe_check(struct drm_simple_display_pipe *pipe,
			  struct drm_plane_state *plane_state,
			  struct drm_crtc_state *crtc_state)
{
	crtc_state->no_vblank = true;
	return 0;
}

static void udl_pipe_update(struct drm_simple_display_pipe *pipe,
			    struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_atomic_helper_damage_iter iter;
	struct drm_rect rect;

	drm_atomic_helper_damage_iter_init(&iter, old_state, state);

	drm_atomic_for_each_plane_damage(&iter, &rect)
		udl_handle_damage(state->fb,
				  rect.x1, rect.y1,
				  rect.x2 - rect.x1, rect.y2 - rect.y1);
}

static const struct drm_simple_display_pipe_funcs udl_pipe_funcs = {
	.enable	= udl_pipe_enable,
	.disable = udl_pipe_disable,
	.check = udl_pipe_check,
	.update	= udl_pipe_update,
	.prepare_fb = drm_gem_fb_simple_display_pipe_prepare_fb,
};

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

static int udl_connector_init(struct udl_device *udl)
{
	drm_connector_helper_add(&udl->conn, &udl_connector_helper_funcs);
	return drm_connector_init(&udl->drm, &udl->conn, &udl_connector_funcs,
				  DRM_MODE_CONNECTOR_DVII);
}

static const uint32_t udl_pipe_formats[] = {
	DRM_FORMAT_XRGB8888,
};

static const uint64_t udl_pipe_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

static int udl_modeset_init(struct udl_device *udl)
{
	struct drm_device *dev = &udl->drm;
	int ret;

	drm_mode_config_init(dev);

	dev->mode_config.min_width = 640;
	dev->mode_config.min_height = 480;

	dev->mode_config.max_width = 2048;
	dev->mode_config.max_height = 2048;

	dev->mode_config.prefer_shadow = 0;
	dev->mode_config.preferred_depth = 24;

	dev->mode_config.funcs = &udl_mode_funcs;

	ret = udl_connector_init(udl);
	if (ret)
		return ret;

	ret = drm_simple_display_pipe_init(dev,
					   &udl->pipe,
					   &udl_pipe_funcs,
					   udl_pipe_formats,
					   ARRAY_SIZE(udl_pipe_formats),
					   udl_pipe_modifiers,
					   &udl->conn);
	if (ret)
		return ret;

	drm_plane_enable_fb_damage_clips(&udl->pipe.plane);

	drm_mode_config_reset(dev);

	return 0;
}

static void udl_modeset_cleanup(struct drm_device *dev)
{
	drm_mode_config_cleanup(dev);
}

DEFINE_DRM_GEM_FOPS(udl_fops);

static void udl_driver_release(struct drm_device *dev)
{
	drm_kms_helper_poll_fini(dev);
	udl_usb_fini(dev);
	udl_modeset_cleanup(dev);
	drm_dev_fini(dev);
	kfree(dev);
}

static struct drm_driver driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM,

	.release = udl_driver_release,
	.fops = &udl_fops,

	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,

	DRM_GEM_SHMEM_DRIVER_OPS,
};

static struct udl_device *udl_driver_create(struct usb_interface *interface)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	struct udl_device *udl;
	int ret;

	udl = kzalloc(sizeof(*udl), GFP_KERNEL);
	if (!udl)
		return ERR_PTR(-ENOMEM);

	ret = drm_dev_init(&udl->drm, &driver, &interface->dev);
	if (ret)
		goto err_free;

	udl->udev = udev;
	udl->drm.dev_private = udl;

	ret = udl_modeset_init(udl);
	if (ret)
		goto err_dev;

	drm_kms_helper_poll_init(&udl->drm);

	ret = udl_usb_init(udl);
	if (ret)
		goto err_dev;

	usb_set_intfdata(interface, udl);
	return udl;

err_dev:
	drm_dev_fini(&udl->drm);
err_free:
	kfree(udl);
	return ERR_PTR(ret);
}

static int udl_usb_probe(struct usb_interface *interface,
			 const struct usb_device_id *id)
{
	int r;
	struct udl_device *udl;

	udl = udl_driver_create(interface);
	if (IS_ERR(udl))
		return PTR_ERR(udl);

	r = drm_dev_register(&udl->drm, 0);
	if (r)
		goto err_free;

	drm_fbdev_generic_setup(&udl->drm, 0);

	DRM_INFO("Initialized udl on minor %d\n", udl->drm.primary->index);

	return 0;

err_free:
	drm_dev_put(&udl->drm);
	return r;
}

static void udl_usb_disconnect(struct usb_interface *interface)
{
	struct drm_device *dev = usb_get_intfdata(interface);

	drm_kms_helper_poll_disable(dev);
	udl_drop_usb(dev);
	drm_dev_unplug(dev);
	drm_dev_put(dev);
}

static int udl_usb_suspend(struct usb_interface *interface,
			   pm_message_t message)
{
	struct drm_device *dev = usb_get_intfdata(interface);

	return drm_mode_config_helper_suspend(dev);
}

static int udl_usb_resume(struct usb_interface *interface)
{
	struct drm_device *dev = usb_get_intfdata(interface);

	drm_mode_config_helper_resume(dev);

	return 0;
}

/*
 * There are many DisplayLink-based graphics products, all with unique PIDs.
 * So we match on DisplayLink's VID + Vendor-Defined Interface Class (0xff)
 * We also require a match on SubClass (0x00) and Protocol (0x00),
 * which is compatible with all known USB 2.0 era graphics chips and firmware,
 * but allows DisplayLink to increment those for any future incompatible chips
 */
static const struct usb_device_id id_table[] = {
	{.idVendor = 0x17e9, .bInterfaceClass = 0xff,
	 .bInterfaceSubClass = 0x00,
	 .bInterfaceProtocol = 0x00,
	 .match_flags = USB_DEVICE_ID_MATCH_VENDOR |
			USB_DEVICE_ID_MATCH_INT_CLASS |
			USB_DEVICE_ID_MATCH_INT_SUBCLASS |
			USB_DEVICE_ID_MATCH_INT_PROTOCOL,},
	{},
};
MODULE_DEVICE_TABLE(usb, id_table);

static struct usb_driver udl_driver = {
	.name = "udl",
	.probe = udl_usb_probe,
	.disconnect = udl_usb_disconnect,
	.suspend = udl_usb_suspend,
	.resume = udl_usb_resume,
	.id_table = id_table,
};
module_usb_driver(udl_driver);
MODULE_LICENSE("GPL");

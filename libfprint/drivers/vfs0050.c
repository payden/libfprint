#define FP_COMPONENT "vfs0050"

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

#include <fp_internal.h>

#include "vfs0050.h"
#include "driver_ids.h"

enum {
    M_INIT_START,
    M_INIT_1_ONGOING,
    M_INIT_1_STEP2,
    M_INIT_1_STEP3,
    M_INIT_1_STEP4,
    M_INIT_2_ONGOING,
    M_INIT_2_RECV_EP1_ONGOING,
    M_INIT_2_RECV_EP2_ONGOING,
    M_INIT_2_COMPLETE,
    M_INIT_NUMSTATES,
};

static void state_init_cb(struct libusb_transfer *transfer)
{
    struct fpi_ssm *ssm = transfer->user_data;
    struct fp_img_dev *dev = ssm->priv;
    struct vfs0050_dev *vfs_dev = dev->priv;
    //TODO: check for transfer errors in tranfer->status
    switch (ssm->cur_state) {
    case M_INIT_1_ONGOING:
        vfs_dev->init1_offset += transfer->actual_length;
        libusb_free_transfer(transfer);
        if (vfs_dev->init1_offset == sizeof(vfs0050_init1)) { //done transferring this initialization section
            fpi_ssm_next_state(ssm);
            break;
        }
        fpi_ssm_jump_to_state(ssm, M_INIT_1_ONGOING);
        break;
    case M_INIT_2_ONGOING:
        vfs_dev->init2_offset += transfer->actual_length;
        libusb_free_transfer(transfer);
        if (vfs_dev->init2_offset == sizeof(vfs0050_init2)) { //done transferring this initialization section
            fpi_ssm_next_state(ssm);
            break;
        }
        fpi_ssm_jump_to_state(ssm, M_INIT_2_ONGOING);
        break;
    case M_INIT_2_RECV_EP1_ONGOING:
        //we wait for a timeout here indicating no more data to receive.
        fprintf(stderr, "Transfer status in recv ongoing: %d\n", transfer->status);
        fprintf(stderr, "actual length: %d\n", transfer->actual_length);
        if (transfer->actual_length == 0 || transfer->status == LIBUSB_TRANSFER_TIMED_OUT) {
            fpi_ssm_next_state(ssm);
            break;
        }
        fpi_ssm_jump_to_state(ssm, M_INIT_2_RECV_EP1_ONGOING);
        break;
    case M_INIT_2_RECV_EP2_ONGOING:
        if (transfer->actual_length == 0 || transfer->status == LIBUSB_TRANSFER_TIMED_OUT) {
            fpi_ssm_next_state(ssm);
            break;
        }
        fpi_ssm_jump_to_state(ssm, M_INIT_2_RECV_EP2_ONGOING);
        break;
    default:
        libusb_free_transfer(transfer);
        fpi_ssm_next_state(ssm);
        break;
    }
    
}

static void state_init(struct fpi_ssm *ssm)
{
    int transferred;
    int ret;
    int to_send;
    struct libusb_transfer *transfer;
    struct fp_img_dev *dev = ssm->priv;
    struct vfs0050_dev *vfs_dev = dev->priv;
    switch (ssm->cur_state) {
    case M_INIT_START:
        //couple of synchronous transfers here in the beginning, don't think this hurts much.
        vfs_dev->tmpbuf[0] = 0x1a;
        ret = libusb_bulk_transfer(dev->udev, EP1_OUT, vfs_dev->tmpbuf, 1, &transferred, BULK_TIMEOUT);
        ret = libusb_bulk_transfer(dev->udev, EP1_IN, vfs_dev->tmpbuf, 64, &transferred, BULK_TIMEOUT);
        fpi_ssm_next_state(ssm);
        break;
    case M_INIT_1_ONGOING:
        to_send = sizeof(vfs0050_init1) - vfs_dev->init1_offset;
        to_send = to_send >= 64 ? 64 : to_send;
        assert(to_send > 0);
        transfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(transfer, dev->udev, EP1_OUT, vfs0050_init1 + vfs_dev->init1_offset, to_send, state_init_cb, ssm, BULK_TIMEOUT);
        libusb_submit_transfer(transfer);
        break;
    case M_INIT_1_STEP2:
        transfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(transfer, dev->udev, EP1_IN, vfs_dev->tmpbuf, 64, state_init_cb, ssm, BULK_TIMEOUT);
        libusb_submit_transfer(transfer);
        break;
    case M_INIT_1_STEP3:
        vfs_dev->tmpbuf[0] = 0x01;
        transfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(transfer, dev->udev, EP1_OUT, vfs_dev->tmpbuf, 1, state_init_cb, ssm, BULK_TIMEOUT);
        libusb_submit_transfer(transfer);
        break;
    case M_INIT_1_STEP4:
        transfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(transfer, dev->udev, EP1_IN, vfs_dev->tmpbuf, 64, state_init_cb, ssm, BULK_TIMEOUT);
        libusb_submit_transfer(transfer);
        break;
    case M_INIT_2_ONGOING:
        to_send = sizeof(vfs0050_init2) - vfs_dev->init2_offset;
        to_send = to_send >= 64 ? 64 : to_send;
        assert(to_send > 0);
        transfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(transfer, dev->udev, EP1_OUT, vfs0050_init2 + vfs_dev->init2_offset, to_send, state_init_cb, ssm, BULK_TIMEOUT);
        libusb_submit_transfer(transfer);
        break;
    case M_INIT_2_RECV_EP1_ONGOING:
        transfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(transfer, dev->udev, EP1_IN, vfs_dev->tmpbuf, 64, state_init_cb, ssm, BULK_TIMEOUT);
        libusb_submit_transfer(transfer);
        break;
    case M_INIT_2_RECV_EP2_ONGOING:
        transfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(transfer, dev->udev, EP2_IN, vfs_dev->tmpbuf, 64, state_init_cb, ssm, BULK_TIMEOUT);
        libusb_submit_transfer(transfer);
        break;
    default:
        fpi_imgdev_open_complete(dev, 0);
        fpi_ssm_mark_completed(ssm);
        break;
    }
}

static void state_init_complete(struct fpi_ssm *ssm)
{
    fp_dbg("state_init_complete called");
}


static void dev_deactivate(struct fp_img_dev *dev)
{
    fp_dbg("dev_deactivate called");
    fpi_imgdev_deactivate_complete(dev);
}

static int dev_activate(struct fp_img_dev *dev, enum fp_imgdev_state state)
{
    fp_dbg("dev_activate called");
    //maybe use ssm here
    fpi_imgdev_activate_complete(dev, 0);
    return 0;
}

static int dev_open(struct fp_img_dev *dev, unsigned long driver_data)
{
    struct fpi_ssm *init_ssm;
    struct vfs0050_dev *vdev = NULL;
    int r;
    fp_dbg("dev_open called");
    
    r = libusb_claim_interface(dev->udev, 0);
    if (r < 0) {
        fp_err("could not claim interface 0");
        return r;
    }
    libusb_control_transfer(dev->udev, 0x00, 0x09, 0x0001, 0, NULL, 0, 100);


    vdev = g_malloc0(sizeof(struct vfs0050_dev));
    dev->priv = vdev;
    init_ssm = fpi_ssm_new(dev->dev, state_init, M_INIT_NUMSTATES);
    init_ssm->priv = dev;
    fpi_ssm_start(init_ssm, state_init_complete);

    return 0;
}

static void dev_close(struct fp_img_dev *dev)
{
    fp_dbg("dev_close called");
    //release private structure
    g_free(dev->priv);

    libusb_release_interface(dev->udev, 0);

    fpi_imgdev_close_complete(dev);
}


static const struct usb_id id_table[] =
{
    { .vendor = 0x138a, .product = 0x0050 },
    { 0, 0, 0 },
};

struct fp_img_driver vfs0050_driver =
{
    .driver =
    {
        .id = VFS0050_ID,
        .name = FP_COMPONENT,
        .full_name = "Validity 0050",
        .id_table = id_table,
        .scan_type = FP_SCAN_TYPE_SWIPE,
    },

    .flags = 0,
    .img_width = VFS0050_IMG_WIDTH,
    .img_height = -1,
    .bz3_threshold = 24,

    .open = dev_open,
    .close = dev_close,
    .activate = dev_activate,
    .deactivate = dev_deactivate,
};

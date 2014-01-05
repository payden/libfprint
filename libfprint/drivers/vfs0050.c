#define FP_COMPONENT "vfs0050"

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <pixman.h>
#include <math.h>

#include <fp_internal.h>

#include "vfs0050.h"
#include "driver_ids.h"

static int is_noise(struct vfs0050_line *a)
{
    int i;
    int total_white = 0;
    int total_black = 0;
    for (i = 0; i < VFS0050_IMG_WIDTH; i++) {
        if (a->row[i] >= WHITE_THRESH)
            total_white++;
        if (a->row[i] <= BLACK_THRESH)
            total_black++;
    }
    if (total_black < 5 && total_white < 5) {
        return 1;
    }
    return 0;
}

static void process_image_data(struct fp_img_dev *dev, char **output, int *output_height)
{
    //pixman stuff taken from libfprint/pixman.c, adapted for my purposes.
    pixman_image_t *orig, *resized;
    pixman_transform_t transform;
    struct vfs0050_dev *vfs_dev = dev->priv;
    struct vfs0050_line *line, *calibration_line;
    char *buf = malloc(vfs_dev->scanbuf_idx);
    int lines = vfs_dev->scanbuf_idx / VFS0050_FRAME_SIZE;
    int i, x, sum, last_sum, diff;
    int new_height;
    //just grab one around middle, there should be 100
    calibration_line = (struct vfs0050_line *) ((char *) vfs_dev->calbuf + (50 * VFS0050_FRAME_SIZE));

    new_height = 0;
    for (i = 0; i < lines; i++) {
        line = (struct vfs0050_line *) ((char *) vfs_dev->scanbuf + (i * VFS0050_FRAME_SIZE));
        if (!is_noise(line))
            memcpy(buf + (new_height++ * VFS0050_IMG_WIDTH), line->row, VFS0050_IMG_WIDTH);
        else
            fp_dbg("removed noise at line: %d\n", i);
    }

    orig = pixman_image_create_bits(PIXMAN_a8, VFS0050_IMG_WIDTH, new_height, (uint32_t *) buf, VFS0050_IMG_WIDTH);
    new_height *= VFS0050_SCALE_FACTOR; //scale for resized image
    resized = pixman_image_create_bits(PIXMAN_a8, VFS0050_IMG_WIDTH, new_height, NULL, VFS0050_IMG_WIDTH);
    pixman_transform_init_identity(&transform);
    pixman_transform_scale(NULL, &transform, pixman_int_to_fixed(1), pixman_double_to_fixed(0.2));
    pixman_image_set_transform(orig, &transform);
    pixman_image_set_filter(orig, PIXMAN_FILTER_BEST, NULL, 0);
    pixman_image_composite32(PIXMAN_OP_SRC,
            orig,
            NULL,
            resized,
            0, 0,
            0, 0,
            0, 0,
            VFS0050_IMG_WIDTH, new_height
            );
    memcpy(buf, pixman_image_get_data(resized), VFS0050_IMG_WIDTH * new_height);

    pixman_image_unref(orig);
    pixman_image_unref(resized);

    *output_height = new_height;
    *output = buf;
}

static void tmp_writeout_buf(struct fp_img_dev *dev)
{
    struct vfs0050_dev *vfs_dev = dev->priv;
    FILE *fp = fopen("/tmp/test.pgm", "w");
    if (!fp) {
        return;
    }
    struct vfs0050_line *line;
    int i, x;
    char tmpbuf[100];
    fwrite("P2\n", 3, 1, fp);
    sprintf(tmpbuf, "%d %d\n", VFS0050_IMG_WIDTH + 32, (vfs_dev->scanbuf_idx / VFS0050_FRAME_SIZE));
    fwrite(tmpbuf, strlen(tmpbuf), 1, fp);
    fwrite("255\n", 4, 1, fp);
    for (i = 0; i < (vfs_dev->scanbuf_idx / VFS0050_FRAME_SIZE); i++) {
        line = (struct vfs0050_line *) ((char *) vfs_dev->scanbuf + (i * VFS0050_FRAME_SIZE));
        for (x = 0; x < VFS0050_IMG_WIDTH + 32; x++) {
            sprintf(tmpbuf, "%d\t", line->row[x] & 0xff);
            fwrite(tmpbuf, strlen(tmpbuf), 1, fp);
        }
        fwrite("\n", 1, 1, fp);
    }
    fclose(fp);
}

static int submit_image(struct fp_img_dev *dev)
{
    struct vfs0050_dev *vfs_dev = dev->priv;
    struct fp_img *img = NULL;
    int final_height;
    char *processed_image;
    tmp_writeout_buf(dev);
    
    process_image_data(dev, &processed_image, &final_height); //the fun part

    img = fpi_img_new(VFS0050_IMG_WIDTH * final_height);
    if (img == NULL)
        return 0;

    memcpy(img->data, processed_image, final_height * VFS0050_IMG_WIDTH);
    free(processed_image);
    img->width = VFS0050_IMG_WIDTH;
    img->height = final_height;
    img->flags = FP_IMG_V_FLIPPED;
    fpi_imgdev_image_captured(dev, img);
    return 1;
}

//activate ssm states
enum {
    M_ACTIVATE_START,
    M_ACTIVATE_1_STEP2,
    M_ACTIVATE_1_SINGLE_READ,
    M_ACTIVATE_2_SEND,
    M_ACTIVATE_EP1_DRAIN,
    M_ACTIVATE_EP3_INT1,
    M_ACTIVATE_AWAIT_FINGER,
    M_ACTIVATE_RECEIVE_FINGERPRINT,
    M_ACTIVATE_POST_RECEIVE,
    M_ACTIVATE_NUMSTATES
};

static void async_sleep_cb(void *data)
{
    struct fpi_ssm *ssm = data;
    struct fp_img_dev *dev = ssm->priv;
    struct vfs0050_dev *vfs_dev = dev->priv;
    if (vfs_dev->is_active) {
        fpi_ssm_jump_to_state(ssm, M_ACTIVATE_START);
    } else {
        fpi_ssm_next_state(ssm);
    }
}

static void state_activate_cb(struct libusb_transfer *transfer)
{
    struct fpi_ssm *ssm = transfer->user_data;
    struct fp_img_dev *dev = ssm->priv;
    struct vfs0050_dev *vfs_dev = dev->priv;
    switch (ssm->cur_state) {
    case M_ACTIVATE_1_SINGLE_READ:
        //check bytes are 0x00 and 0x00
        if (vfs_dev->tmpbuf[0] != 0x00 || vfs_dev->tmpbuf[1] != 0x00) {
            fp_dbg("unexpected bytes back from endpoint in M_ACTIVATE_1_SINGLE_READ");
            libusb_free_transfer(transfer);
            fpi_ssm_jump_to_state(ssm, M_ACTIVATE_START);
            break;
        }
        libusb_free_transfer(transfer);
        fpi_ssm_next_state(ssm);
        break;
    case M_ACTIVATE_2_SEND:
        vfs_dev->activate_offset += transfer->actual_length;
        if (vfs_dev->activate_offset == sizeof(vfs0050_activate2)) { //finished sending this activation section
            fpi_ssm_next_state(ssm);
            break;
        }
        fpi_ssm_jump_to_state(ssm, M_ACTIVATE_2_SEND);
        break;
    case M_ACTIVATE_EP1_DRAIN:
        //just draining this data, not sure what it does yet.
        if (transfer->actual_length < 64) {
            libusb_free_transfer(transfer);
            fpi_ssm_next_state(ssm);
            break;
        }
        libusb_free_transfer(transfer);
        fpi_ssm_jump_to_state(ssm, M_ACTIVATE_EP1_DRAIN);
        break;
    case M_ACTIVATE_EP3_INT1:
        if (transfer->actual_length != 5) {
            fp_dbg("unexpected length for interrupt transfer in M_ACTIVATE_EP3_INT1");
            //TODO: fail here, exit ssm
        }
        fpi_ssm_next_state(ssm);
        break;
    case M_ACTIVATE_AWAIT_FINGER:
        //if we got here, it's time to read fingerprint data.
        //interrupt data should be 02 00 0e 00 f0
        if (vfs_dev->is_active) {
            if (transfer->actual_length != 5) {
                fp_dbg("unexpected length for interrupt transfer in M_ACTIVATE_AWAIT_FINGER");
                //TODO: fail here, exit ssm.
            }
            if (memcmp(vfs_dev->tmpbuf, vfs0050_valid_interrupt, 5) != 0) {
                fp_dbg("invalid interrupt data in M_ACTIVATE_AWAIT_FINGER");
                //TODO: fail here, exit ssm.
            }
            fpi_imgdev_report_finger_status(dev, TRUE); //report finger on to libfprint
            fpi_ssm_next_state(ssm);
        } else {
            fpi_ssm_mark_completed(ssm);
            //break fall through
        }
        break;
    case M_ACTIVATE_RECEIVE_FINGERPRINT:
        if (transfer->actual_length == 0) {
            libusb_free_transfer(transfer);
            fpi_ssm_next_state(ssm);
            break;
        }
        vfs_dev->scanbuf_idx += transfer->actual_length;
        libusb_free_transfer(transfer);
        fpi_ssm_jump_to_state(ssm, M_ACTIVATE_RECEIVE_FINGERPRINT);
        break;
    default:
        libusb_free_transfer(transfer);
        fpi_ssm_next_state(ssm);
        break;
    }
}

static void state_activate(struct fpi_ssm *ssm)
{
    struct libusb_transfer *transfer;
    struct fp_img_dev *dev = ssm->priv;
    struct vfs0050_dev *vfs_dev = dev->priv;
    int to_send;
    switch (ssm->cur_state) {
    case M_ACTIVATE_START:
        vfs_dev->activate_offset = 0;
        transfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(transfer, dev->udev, EP1_OUT, vfs0050_activate1, 64, state_activate_cb, ssm, BULK_TIMEOUT);
        libusb_submit_transfer(transfer);
        break;
    case M_ACTIVATE_1_STEP2:
        transfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(transfer, dev->udev, EP1_OUT, vfs0050_activate1 + 64, 61, state_activate_cb, ssm, BULK_TIMEOUT);
        libusb_submit_transfer(transfer);
        break;
    case M_ACTIVATE_1_SINGLE_READ:
        transfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(transfer, dev->udev, EP1_IN, vfs_dev->tmpbuf, 64, state_activate_cb, ssm, BULK_TIMEOUT);
        libusb_submit_transfer(transfer);
        break;
    case M_ACTIVATE_2_SEND:
        to_send = sizeof(vfs0050_activate2) - vfs_dev->activate_offset;
        to_send = to_send >= 64 ? 64 : to_send;
        transfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(transfer, dev->udev, EP1_OUT, vfs0050_activate2 + vfs_dev->activate_offset, to_send, state_activate_cb, ssm, BULK_TIMEOUT);
        libusb_submit_transfer(transfer);
        break;
    case M_ACTIVATE_EP1_DRAIN:
        transfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(transfer, dev->udev, EP1_IN, vfs_dev->tmpbuf, 64, state_activate_cb, ssm, BULK_TIMEOUT);
        libusb_submit_transfer(transfer);
        break;
    case M_ACTIVATE_EP3_INT1: //first interrupt, should be 5 x 0x00
        transfer = libusb_alloc_transfer(0);
        libusb_fill_interrupt_transfer(transfer, dev->udev, EP3_IN, vfs_dev->tmpbuf, 8, state_activate_cb, ssm, INTERRUPT_TIMEOUT1);
        libusb_submit_transfer(transfer);
        break;
    case M_ACTIVATE_AWAIT_FINGER:
        //this sets up infinite wait for interrupt.  When the interrupt occurs, we're ready to read data on EP2.
        if (!vfs_dev->is_active) {
            vfs_dev->is_active = 1;
            fpi_imgdev_activate_complete(dev, 0); //notify libfprint activation complete.
        }
        transfer = libusb_alloc_transfer(0);
        libusb_fill_interrupt_transfer(transfer, dev->udev, EP3_IN, vfs_dev->tmpbuf, 8, state_activate_cb, ssm, INTERRUPT_TIMEOUT_NONE);
        libusb_submit_transfer(transfer);
        break;
    case M_ACTIVATE_RECEIVE_FINGERPRINT:
        if (vfs_dev->scanbuf_idx + 64 >= vfs_dev->scanbuf_sz) {
            vfs_dev->scanbuf_sz <<= 1;
            vfs_dev->scanbuf = g_realloc(vfs_dev->scanbuf, vfs_dev->scanbuf_sz);
        }
        transfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(transfer, dev->udev, EP2_IN, vfs_dev->scanbuf + vfs_dev->scanbuf_idx, 64, state_activate_cb, ssm, 500);
        libusb_submit_transfer(transfer);
        break;
    case M_ACTIVATE_POST_RECEIVE:
        submit_image(dev);
        fpi_imgdev_report_finger_status(dev, FALSE);
        vfs_dev->activate_offset = 0;
        vfs_dev->scanbuf_idx = 0;
        fpi_timeout_add(300, async_sleep_cb, ssm); //wait a bit and see if we're swiping again.
        break;

    }
}

static void state_activate_complete(struct fpi_ssm *ssm)
{
    fpi_ssm_free(ssm);
}

//init ssm states
enum {
    M_INIT_START,
    M_INIT_1_ONGOING,
    M_INIT_1_STEP2,
    M_INIT_1_STEP3,
    M_INIT_1_STEP4,
    M_INIT_2_ONGOING,
    M_INIT_2_RECV_EP1_ONGOING,
    M_INIT_2_RECV_EP2_ONGOING, //TODO: this looks like some sensor calibration data, could be useful later.
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
        if (transfer->actual_length < 64) {
            fpi_ssm_next_state(ssm);
            break;
        }
        fpi_ssm_jump_to_state(ssm, M_INIT_2_RECV_EP1_ONGOING);
        break;
    case M_INIT_2_RECV_EP2_ONGOING:
        vfs_dev->calbuf_idx += transfer->actual_length;
        if (transfer->actual_length < 64) {
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
    int to_send;
    struct libusb_transfer *transfer;
    struct fp_img_dev *dev = ssm->priv;
    struct vfs0050_dev *vfs_dev = dev->priv;
    switch (ssm->cur_state) {
    case M_INIT_START:
        //couple of synchronous transfers here in the beginning, don't think this hurts much.
        vfs_dev->tmpbuf[0] = 0x1a;
        libusb_bulk_transfer(dev->udev, EP1_OUT, vfs_dev->tmpbuf, 1, &transferred, BULK_TIMEOUT);
        libusb_bulk_transfer(dev->udev, EP1_IN, vfs_dev->tmpbuf, 64, &transferred, BULK_TIMEOUT);
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
        if (vfs_dev->calbuf_idx + 64 >= vfs_dev->calbuf_sz) {
            vfs_dev->calbuf_sz <<= 1;
            vfs_dev->calbuf = g_realloc(vfs_dev->calbuf, vfs_dev->calbuf_sz);
        }
        transfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(transfer, dev->udev, EP2_IN, vfs_dev->calbuf + vfs_dev->calbuf_idx, 64, state_init_cb, ssm, BULK_TIMEOUT);
        libusb_submit_transfer(transfer);
        break;
    default:
        fpi_ssm_mark_completed(ssm);
        break;
    }
}

static void state_init_complete(struct fpi_ssm *ssm)
{
    struct fp_img_dev *dev = ssm->priv;
    fpi_imgdev_open_complete(dev, 0);
    fpi_ssm_free(ssm);
}

static void generic_async_cb(struct libusb_transfer *t)
{
    libusb_free_transfer(t);
}

static void dev_deactivate(struct fp_img_dev *dev)
{
    struct vfs0050_dev *vfs_dev = dev->priv;
    int err = 0;
    int tmpoffset;
    int to_send;
    struct libusb_transfer *t;
    vfs_dev->is_active = 0;
    //EP2 IN
    t = libusb_alloc_transfer(0);
    libusb_fill_bulk_transfer(t, dev->udev, EP2_IN, vfs_dev->tmpbuf, 64, generic_async_cb, NULL, 100);
    libusb_submit_transfer(t);
    //EP1_IN
    t = libusb_alloc_transfer(0);
    libusb_fill_bulk_transfer(t, dev->udev, EP1_IN, vfs_dev->tmpbuf, 64, generic_async_cb, NULL, 100);
    libusb_submit_transfer(t);
    //EP1_OUT
    t = libusb_alloc_transfer(0);
    vfs_dev->tmpbuf[0] = 0x04;
    libusb_fill_bulk_transfer(t, dev->udev, EP1_OUT, vfs_dev->tmpbuf, 1, generic_async_cb, NULL, 100);
    libusb_submit_transfer(t);
    //EP1_IN
    t = libusb_alloc_transfer(0);
    libusb_fill_bulk_transfer(t, dev->udev, EP1_IN, vfs_dev->tmpbuf, 64, generic_async_cb, NULL, 100);
    libusb_submit_transfer(t);
    //EP1_OUT
    t = libusb_alloc_transfer(0);
    libusb_fill_bulk_transfer(t, dev->udev, EP1_OUT, vfs0050_deactivate1, 64, generic_async_cb, NULL, 100);
    libusb_submit_transfer(t);
    //EP1_OUT
    t = libusb_alloc_transfer(0);
    libusb_fill_bulk_transfer(t, dev->udev, EP1_OUT, &vfs0050_deactivate1[64], 61, generic_async_cb, NULL, 100);
    libusb_submit_transfer(t);
    tmpoffset = 0;
    do {
        to_send = sizeof(vfs0050_activate2) - tmpoffset;
        to_send = to_send >= 64 ? 64 : to_send;
        t = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(t, dev->udev, EP1_OUT, vfs0050_activate2 + tmpoffset, to_send, generic_async_cb, NULL, 100);
        libusb_submit_transfer(t);
        tmpoffset += err;
    } while (err == 64);
    do {
        t = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(t, dev->udev, EP1_IN, vfs_dev->tmpbuf, 64, generic_async_cb, NULL, 100);
        libusb_submit_transfer(t);
    } while(err == 64);
    t = libusb_alloc_transfer(0);
    libusb_fill_bulk_transfer(t, dev->udev, EP1_IN, vfs_dev->tmpbuf, 64, generic_async_cb, NULL, 100);
    libusb_submit_transfer(t);
    //TODO: finish this, leaves device in inconsistent state.
    fpi_imgdev_deactivate_complete(dev);
}

static int dev_activate(struct fp_img_dev *dev, enum fp_imgdev_state state)
{
    fp_dbg("dev_activate called");
    struct fpi_ssm *activate_ssm;
    activate_ssm = fpi_ssm_new(dev->dev, state_activate, M_ACTIVATE_NUMSTATES);
    activate_ssm->priv = dev;
    fpi_ssm_start(activate_ssm, state_activate_complete);
    return 0;
}

static int dev_open(struct fp_img_dev *dev, unsigned long driver_data)
{
    struct fpi_ssm *init_ssm;
    struct vfs0050_dev *vdev = NULL;
    int r;
    fp_dbg("dev_open called");
    //TODO: this is probably frowned upon :/ but right now a reset is the only
    // way for me to know I'm working with a clean slate and the rest of this will work.
    // Will be trying to reverse engineer more of the protocol so I can avoid resetting
    // the device each time it's opened.
    libusb_reset_device(dev->udev);

    r = libusb_claim_interface(dev->udev, 0);
    if (r < 0) {
        fp_err("could not claim interface 0");
        return r;
    }
    libusb_control_transfer(dev->udev, 0x00, 0x09, 0x0001, 0, NULL, 0, 100);


    vdev = g_malloc0(sizeof(struct vfs0050_dev));
    vdev->scanbuf = g_malloc0(VFS0050_INITIAL_SCANBUF_SIZE);
    vdev->scanbuf_sz = VFS0050_INITIAL_SCANBUF_SIZE;
    vdev->calbuf = g_malloc0(VFS0050_INITIAL_SCANBUF_SIZE);
    vdev->calbuf_sz = VFS0050_INITIAL_SCANBUF_SIZE;
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
    g_free(((struct vfs0050_dev *)dev->priv)->calbuf);
    g_free(((struct vfs0050_dev *)dev->priv)->scanbuf);
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

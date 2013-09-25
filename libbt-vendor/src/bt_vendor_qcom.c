/*
 * Copyright 2012 The Android Open Source Project
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 * Not a Contribution.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/******************************************************************************
 *
 *  Filename:      bt_vendor_qcom.c
 *
 *  Description:   vendor specific library implementation
 *
 ******************************************************************************/

#define LOG_TAG "bt_vendor"

#include <utils/Log.h>
#include <cutils/properties.h>
#include <fcntl.h>
#include <termios.h>
#include "bt_vendor_qcom.h"
#include "hci_uart.h"
#include "hci_smd.h"

/******************************************************************************
**  Externs
******************************************************************************/
extern int hw_config(int nState);

extern int is_hw_ready();
extern int rome_soc_init(int fd, char *bdaddr);
/******************************************************************************
**  Variables
******************************************************************************/
int pFd[2] = {0,};
bt_vendor_callbacks_t *bt_vendor_cbacks = NULL;
uint8_t vnd_local_bd_addr[6]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static int btSocType = BT_SOC_DEFAULT;
static int rfkill_id = -1;
static char *rfkill_state = NULL;


static const tUSERIAL_CFG userial_init_cfg =
{
    (USERIAL_DATABITS_8 | USERIAL_PARITY_NONE | USERIAL_STOPBITS_1),
    USERIAL_BAUD_115200
};

#if (HW_NEED_END_WITH_HCI_RESET == TRUE)
void hw_epilog_process(void);
#endif


/******************************************************************************
**  Local type definitions
******************************************************************************/


/******************************************************************************
**  Functions
******************************************************************************/

/** Get Bluetooth SoC type from system setting */
static int get_bt_soc_type()
{
    int ret = 0;
    char bt_soc_type[PROPERTY_VALUE_MAX];

    ALOGI("bt-vendor : get_bt_soc_type");

    ret = property_get("qcom.bluetooth.soc", bt_soc_type, NULL);
    if (ret != 0) {
        ALOGI("qcom.bluetooth.soc set to %s\n", bt_soc_type);
        if (!strncasecmp(bt_soc_type, "rome", sizeof("rome"))) {
            return BT_SOC_ROME;
        }
        else if (!strncasecmp(bt_soc_type, "ath3k", sizeof("ath3k"))) {
            return BT_SOC_AR3K;
        }
        else {
            ALOGI("qcom.bluetooth.soc not set, so using default.\n");
            return BT_SOC_DEFAULT;
        }
    }
    else {
        ALOGE("%s: Failed to get soc type", __FUNCTION__);
        ret = -1;
    }

    return ret;
}

/** Bluetooth Controller power up or shutdown */
static int bt_powerup(int en )
{
    char rfkill_type[64];
    char type[16];
    int fd, size, i;

    char disable[PROPERTY_VALUE_MAX];
    char on = (en)?'1':'0';

    ALOGI("bt_powerup: %c", on);

    /* Check if rfkill has been disabled */
    property_get("ro.rfkilldisabled", disable, "0");

    /* In case rfkill disabled, then no control power*/
    if (strcmp(disable, "1") == 0) {
        ALOGI("ro.rfkilldisabled : %s", disable);
        return -1;
    }

    /* Assign rfkill_id and find bluetooth rfkill state path*/
    for(i=0;(rfkill_id == -1) && (rfkill_state == NULL);i++)
    {
        snprintf(rfkill_type, sizeof(rfkill_type), "/sys/class/rfkill/rfkill%d/type", i);
        if ((fd = open(rfkill_type, O_RDONLY)) < 0)
        {
            ALOGE("open(%s) failed: %s (%d)\n", rfkill_type, strerror(errno), errno);
            return -1;
        }

        size = read(fd, &type, sizeof(type));
        close(fd);

        if ((size >= 9) && !memcmp(type, "bluetooth", 9))
        {
            asprintf(&rfkill_state, "/sys/class/rfkill/rfkill%d/state", rfkill_id = i);
            break;
        }
    }

    /* Get rfkill State to control */
    if ((fd = open(rfkill_state, O_WRONLY)) < 0)
    {
        ALOGE("open(%s) for write failed: %s (%d)",rfkill_state, strerror(errno), errno);
        return -1;
    }

    /* Write value to control rfkill */
    if ((size = write(fd, &on, 1)) < 0) {
        ALOGE("write(%s) failed: %s (%d)",rfkill_state, strerror(errno),errno);
        return -1;
    }

    if (fd >= 0)
        close(fd);

    return 0;
}

/*****************************************************************************
**
**   BLUETOOTH VENDOR INTERFACE LIBRARY FUNCTIONS
**
*****************************************************************************/

static int init(const bt_vendor_callbacks_t* p_cb, unsigned char *local_bdaddr)
{
    int i;

    ALOGI("bt-vendor : init");

    if (p_cb == NULL)
    {
        ALOGE("init failed with no user callbacks!");
        return -1;
    }

    if ((btSocType = get_bt_soc_type()) < 0) {
        ALOGE("%s: Failed to detect BT SOC Type", __FUNCTION__);
        return -1;
    }

    switch(btSocType)
    {
        case BT_SOC_ROME:
        case BT_SOC_AR3K:
            ALOGI("bt-vendor : Initializing UART transport layer");
            userial_vendor_init();
            break;
        case BT_SOC_DEFAULT:
            break;
        default:
            ALOGE("Unknown btSocType: 0x%x", btSocType);
            break;
    }

    /* store reference to user callbacks */
    bt_vendor_cbacks = (bt_vendor_callbacks_t *) p_cb;

    /* Copy BD Address as little-endian byte order */
    if(local_bdaddr)
        for(i=0;i<6;i++)
            vnd_local_bd_addr[i] = *(local_bdaddr + (5-i));

    ALOGI("%s: Local BD Address : %.2x:%.2x:%.2x:%.2x:%.2x:%.2x", __FUNCTION__,
                                                vnd_local_bd_addr[0],
                                                vnd_local_bd_addr[1],
                                                vnd_local_bd_addr[2],
                                                vnd_local_bd_addr[3],
                                                vnd_local_bd_addr[4],
                                                vnd_local_bd_addr[5]);
    return 0;
}

/** Requested operations */
static int op(bt_vendor_opcode_t opcode, void *param)
{
    int retval = 0;
    int nCnt = 0;
    int nState = -1;

    ALOGV("bt-vendor : op for %d", opcode);

    switch(opcode)
    {
        case BT_VND_OP_POWER_CTRL:
            {
                nState = *(int *) param;
                ALOGI("bt-vendor : BT_VND_OP_POWER_CTRL: %s",
                        (nState == BT_VND_PWR_ON)? "On" : "Off" );

                switch(btSocType)
                {
                    case BT_SOC_DEFAULT:
                        retval = hw_config(nState);
                        if(nState == BT_VND_PWR_ON
                           && retval == 0
                           && is_hw_ready() == TRUE){
                            retval = 0;
                        }
                        else {
                            retval = -1;
                        }
                    case BT_SOC_ROME:
                    case BT_SOC_AR3K:
                        /* BT Chipset Power Control through Device Tree Node */
                        retval = bt_powerup(nState);
                    default:
                        break;
                }
            }
            break;

        case BT_VND_OP_FW_CFG:
            {
                // call hciattach to initalize the stack
                if(bt_vendor_cbacks){
                   ALOGI("Bluetooth Firmware and transport layer are initialized");
                   bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
                }
                else{
                   ALOGE("Error : hci, smd initialization Error");
                   bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
                }
            }
            break;

        case BT_VND_OP_SCO_CFG:
            {
                bt_vendor_cbacks->scocfg_cb(BT_VND_OP_RESULT_SUCCESS); //dummy
            }
            break;

        case BT_VND_OP_USERIAL_OPEN:
            {
                ALOGI("bt-vendor : BT_VND_OP_USERIAL_OPEN");
                switch(btSocType)
                {
                    case BT_SOC_DEFAULT:
                        {
                            if(bt_hci_init_transport(pFd) != -1){
                                int (*fd_array)[] = (int (*) []) param;

                                    (*fd_array)[CH_CMD] = pFd[0];
                                    (*fd_array)[CH_EVT] = pFd[0];
                                    (*fd_array)[CH_ACL_OUT] = pFd[1];
                                    (*fd_array)[CH_ACL_IN] = pFd[1];
                            }
                            else {
                                retval = -1;
                                break;
                            }
                            retval = 2;
                        }
                        break;
                    case BT_SOC_ROME:
                    case BT_SOC_AR3K:
                        {
                            int (*fd_array)[] = (int (*)[]) param;
                            int idx, fd;
                            fd = userial_vendor_open((tUSERIAL_CFG *) &userial_init_cfg);
                            if (fd != -1) {
                                for (idx=0; idx < CH_MAX; idx++)
                                    (*fd_array)[idx] = fd;
                                     retval = 1;
                            }
                            else
                                retval = -1;

                            /* Vendor Specific Process should happened during userial_open process
                                After userial_open, rx read thread is running immediately,
                                so it will affect VS event read process.
                            */
                            switch (btSocType)
                            {
                                case BT_SOC_ROME:
                                    if(rome_soc_init(fd,vnd_local_bd_addr)<0)
                                        retval = -1;
                                    break;
                                case BT_SOC_AR3K:
                                    if(ath3k_init(fd,3000000,115200,NULL,&vnd_userial.termios)<0)
                                        retval = -1;
                                    break;
                            }
                        }
                        break;
                    default:
                        ALOGE("Unknown btSocType: 0x%x", btSocType);
                        break;
                }
            }
            break;

        case BT_VND_OP_USERIAL_CLOSE:
            {
                switch(btSocType)
                {
                    case BT_SOC_DEFAULT:
                         bt_hci_deinit_transport(pFd);
                         break;

                     case BT_SOC_ROME:
                     case BT_SOC_AR3K:
                        userial_vendor_close();
                        break;
                    default:
                        ALOGE("Unknown btSocType: 0x%x", btSocType);
                        break;
                }
            }
            break;

        case BT_VND_OP_GET_LPM_IDLE_TIMEOUT:
            if (btSocType ==  BT_SOC_AR3K) {
                uint32_t *timeout_ms = (uint32_t *) param;
                *timeout_ms = 1000;
            }
            break;

        case BT_VND_OP_LPM_SET_MODE:
            if(btSocType ==  BT_SOC_AR3K) {
                uint8_t *mode = (uint8_t *) param;

                if (*mode) {
                    lpm_set_ar3k(UPIO_LPM_MODE, UPIO_ASSERT, 0);
                }
                else {
                    lpm_set_ar3k(UPIO_LPM_MODE, UPIO_DEASSERT, 0);
                }

                bt_vendor_cbacks->lpm_cb(BT_VND_OP_RESULT_SUCCESS);
            }
            else {
                bt_vendor_cbacks->lpm_cb(BT_VND_OP_RESULT_SUCCESS); //dummy
            }
            break;

        case BT_VND_OP_LPM_WAKE_SET_STATE:
            {
                switch(btSocType)
                {
                    case BT_SOC_ROME:
                        {
                            uint8_t *state = (uint8_t *) param;
                            uint8_t wake_assert = (*state == BT_VND_LPM_WAKE_ASSERT) ? \
                                BT_VND_LPM_WAKE_ASSERT : BT_VND_LPM_WAKE_DEASSERT;

                            if (wake_assert == 0)
                                ALOGI("ASSERT: Waking up BT-Device");
                            else if (wake_assert == 1)
                                ALOGI("DEASSERT: Allowing BT-Device to Sleep");

#ifdef QCOM_BT_SIBS_ENABLE
                            if(bt_vendor_cbacks){
                                ALOGI("Invoking HCI H4 callback function");
                               bt_vendor_cbacks->lpm_set_state_cb(wake_assert);
                            }
#endif
                        }
                        break;
                    case BT_SOC_AR3K:
                        {
                            uint8_t *state = (uint8_t *) param;
                            uint8_t wake_assert = (*state == BT_VND_LPM_WAKE_ASSERT) ? \
                                                        UPIO_ASSERT : UPIO_DEASSERT;
                            lpm_set_ar3k(UPIO_BT_WAKE, wake_assert, 0);
                        }
                    case BT_SOC_DEFAULT:
                        break;
                    default:
                        ALOGE("Unknown btSocType: 0x%x", btSocType);
                        break;
                    }
            }
            break;
        case BT_VND_OP_EPILOG:
            {
#if (HW_NEED_END_WITH_HCI_RESET == FALSE)
                if (bt_vendor_cbacks)
                {
                    bt_vendor_cbacks->epilog_cb(BT_VND_OP_RESULT_SUCCESS);
                }
#else
                hw_epilog_process();
#endif
            }
            break;
    }

    return retval;
}

/** Closes the interface */
static void cleanup( void )
{
    ALOGI("cleanup");
    bt_vendor_cbacks = NULL;
}

// Entry point of DLib
const bt_vendor_interface_t BLUETOOTH_VENDOR_LIB_INTERFACE = {
    sizeof(bt_vendor_interface_t),
    init,
    op,
    cleanup
};

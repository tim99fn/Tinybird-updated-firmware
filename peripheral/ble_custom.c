/**
 * Copyright (c) 2012 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_NUS)
#include "ble.h"
#include "ble_custom.h"
#include "ble_srv_common.h"

#define NRF_LOG_MODULE_NAME ble_nus
#if BLE_NUS_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       BLE_NUS_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  BLE_NUS_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR BLE_NUS_CONFIG_DEBUG_COLOR
#else // BLE_NUS_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       0
#endif // BLE_NUS_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();


#define BLE_UUID_NUS_TX_CHARACTERISTIC 0x0003               /**< The UUID of the TX Characteristic. */
#define BLE_UUID_NUS_RX_CHARACTERISTIC 0x0002               /**< The UUID of the RX Characteristic. */

#define BLE_NUS_MAX_RX_CHAR_LEN        BLE_NUS_MAX_DATA_LEN /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_NUS_MAX_TX_CHAR_LEN        BLE_NUS_MAX_DATA_LEN /**< Maximum length of the TX Characteristic (in bytes). */

#define NUS_BASE_UUID                  {{0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}} /**< Used vendor specific UUID. */


//Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the SoftDevice.
 
static void on_connect(ble_nus_t * p_nus, ble_evt_t const * p_ble_evt)
{
    ret_code_t                 err_code;
    ble_nus_evt_t              evt;
    ble_gatts_value_t          gatts_val;
    uint8_t                    cccd_value[2];
    ble_nus_client_context_t * p_client = NULL;
 err_code = blcm_link_ctx_get(p_nus->p_link_ctx_storage,
                                 p_ble_evt->evt.gatts_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gatts_evt.conn_handle);
    }



    /* Check the hosts CCCD value to inform of readiness to send data using the RX characteristic */
    memset(&gatts_val, 0, sizeof(ble_gatts_value_t));
    gatts_val.p_value = cccd_value;
    gatts_val.len     = sizeof(cccd_value);
    gatts_val.offset  = 0;



        memset(&evt, 0, sizeof(ble_nus_evt_t));
        evt.type        = BLE_NUS_EVT_COMM_STARTED;
        evt.p_nus       = p_nus;
        evt.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        evt.p_link_ctx  = p_client;

        p_nus->data_handler(&evt);
    
}


// Function for handling the @ref BLE_GATTS_EVT_WRITE event from the SoftDevice.
//triggers the event communication started if the initialisation of the service went as planned
static void on_write(ble_nus_t * p_nus, ble_evt_t const * p_ble_evt)
{
    ret_code_t                    err_code;
    ble_nus_evt_t                 evt;
    ble_nus_client_context_t    * p_client;
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    err_code = blcm_link_ctx_get(p_nus->p_link_ctx_storage,
                                 p_ble_evt->evt.gatts_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gatts_evt.conn_handle);
    }

    memset(&evt, 0, sizeof(ble_nus_evt_t));
    evt.p_nus       = p_nus;
    evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
    evt.p_link_ctx  = p_client;

    if ((p_evt_write->handle == p_nus->tx_handles.cccd_handle) &&
        (p_evt_write->len == 2))
    {
        if (p_client != NULL)
        {
            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                p_client->is_notification_enabled = true;
                evt.type                          = BLE_NUS_EVT_COMM_STARTED;
            }
            else
            {
                p_client->is_notification_enabled = false;
                evt.type                          = BLE_NUS_EVT_COMM_STOPPED;
            }

            if (p_nus->data_handler != NULL)
            {
                p_nus->data_handler(&evt);
            }

        }
    }
   
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
   
}



void ble_custom_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_nus_t * p_nus = (ble_nus_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_nus, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_nus, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

//initialisation of the implemented service and the characteristics it contains
uint32_t ble_custom_init(ble_nus_t * p_nus, ble_nus_init_t const * p_nus_init)
{
    ret_code_t            err_code;
    ble_uuid_t            ble_uuid;
    ble_uuid128_t         nus_base_uuid = NUS_BASE_UUID;
    ble_add_char_params_t add_char_params;

    VERIFY_PARAM_NOT_NULL(p_nus);
    VERIFY_PARAM_NOT_NULL(p_nus_init);

    // Initialize the service structure.
    p_nus->data_handler = p_nus_init->data_handler;

    //Adding proprietary Service to the SoftDevice] 
    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&nus_base_uuid, &p_nus->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_nus->uuid_type;
    ble_uuid.uuid = BLE_UUID_NUS_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_nus->service_handle);
   
    VERIFY_SUCCESS(err_code);

    // rx characterisitc implementd since it receives epty packet as confirmation from central
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = BLE_UUID_NUS_RX_CHARACTERISTIC;
    add_char_params.uuid_type                = p_nus->uuid_type;
    add_char_params.max_len                  = BLE_NUS_MAX_RX_CHAR_LEN;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.write         = 1;
    add_char_params.char_props.write_wo_resp = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;

    err_code = characteristic_add(p_nus->service_handle, &add_char_params, &p_nus->rx_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the TX Characteristic.
  
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLE_UUID_NUS_TX_CHARACTERISTIC;
    add_char_params.uuid_type         = p_nus->uuid_type;
    add_char_params.max_len           = BLE_NUS_MAX_TX_CHAR_LEN;
    add_char_params.init_len          = sizeof(uint8_t);
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;

    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access      = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    return characteristic_add(p_nus->service_handle, &add_char_params, &p_nus->tx_handles);
    //add characteristic to bluetooth stack
}

//implementing the hvx notification that was previously initialized to make it callable from the main
uint32_t ble_custom_data_send(ble_nus_t * p_nus,
                           uint8_t   * p_data,
                           uint16_t  * p_length,
                           uint16_t    conn_handle)
{
    ret_code_t                 err_code;
    ble_gatts_hvx_params_t     hvx_params;
    ble_nus_client_context_t * p_client;

 
    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_nus->tx_handles.value_handle;
    hvx_params.p_data = p_data;
    hvx_params.p_len  = p_length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(conn_handle, &hvx_params);
}


#endif // NRF_MODULE_ENABLED(BLE_NUS)

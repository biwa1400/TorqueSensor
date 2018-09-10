
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_lbs.h"
#include "nrf_ble_gatt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_delay.h"
#include "nrf_drv_twi.h"

#include "nrf_drv_timer.h"

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2    /**< Reply when unsupported features are requested. */


#define DEVICE_NAME                     "torque_miun"                         /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(25, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(50, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(1000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_ALL;                        /**< Handle of the current connection. */


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/*****************self start**************************/

void callbackFunc(ble_evt_t const * p_ble_evt, void * p_context)
{
	NRF_LOG_INFO("CallBack %d",*((uint8_t*)p_context+2));
	switch (p_ble_evt->header.evt_id)
	{
		case BLE_GATTS_EVT_WRITE:
			NRF_LOG_INFO("%.2x",p_ble_evt->evt.gatts_evt.params.write.data[0])
		break;
        default:
            // No implementation needed.
            break;
	}
}


ret_code_t addCallFunc()
{
	static uint8_t parameters[]={4,5,6};
	static void (*callbackFuncPtr)(ble_evt_t const * p_ble_evt, void * p_context); 
	NRF_LOG_INFO("calling now!");
	NRF_SDH_BLE_OBSERVER(null,BLE_LBS_BLE_OBSERVER_PRIO,callbackFunc, parameters);
}


ret_code_t change_Char_Value(int size,uint8_t* valueBuf,uint16_t handle)
{
		ble_gatts_value_t ble_gatts_value;
		ble_gatts_value.p_value=valueBuf;
		ble_gatts_value.len=size;
		uint32_t   err_code = sd_ble_gatts_value_set(-1,handle,&ble_gatts_value);
	  //NRF_LOG_INFO("error code: %d",err_code);
		NRF_LOG_INFO("");
}

ret_code_t sendNotify(uint16_t size,uint8_t* valueBuf,uint16_t characterHandle)
{
	ble_gatts_hvx_params_t params;
	params.handle = characterHandle;
	params.type=BLE_GATT_HVX_NOTIFICATION;
	params.p_data = valueBuf;
	params.p_len = &size;
	uint32_t err_code = sd_ble_gatts_hvx(m_conn_handle, &params);
//	NRF_LOG_INFO("conn handle: %d",m_conn_handle);
	//NRF_LOG_INFO("error code2: %d",err_code);
	NRF_LOG_INFO("");
}


ret_code_t get_Char_Value(uint16_t* len,uint8_t* valueBuf,uint16_t handle)
{
		ble_gatts_value_t ble_gatts_value;
		uint32_t err_code = sd_ble_gatts_value_get(-1, handle, &ble_gatts_value);
		*len=ble_gatts_value.len;
		memcpy(valueBuf,ble_gatts_value.p_value,ble_gatts_value.len);
	
		//NRF_LOG_INFO("error code: %d",err_code);
	

}

ret_code_t setUUID128(uint8_t* uuid128)
{
	ble_uuid128_t base_uuid = {LBS_UUID_BASE};
	memcpy(base_uuid.uuid128,uuid128,16);
	uint8_t UUID_type = BLE_UUID_TYPE_VENDOR_BEGIN;
	uint8_t err_code = sd_ble_uuid_vs_add(&base_uuid, &UUID_type);
}


ret_code_t addService(uint16_t uuid, uint8_t uuid_type, uint16_t* service_handle)
{
	uint32_t   err_code;
	ble_uuid_t ble_uuid;
	
	// set UUID
	//ble_uuid.type = BLE_UUID_TYPE_BLE;     //para
  //ble_uuid.uuid = BLE_UUID_HEALTH_THERMOMETER_SERVICE; //para
	ble_uuid.type = uuid_type;
	ble_uuid.uuid = uuid;
	
	// add Service 
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, service_handle);
	VERIFY_SUCCESS(err_code);
}

ret_code_t addCharacteristic(uint16_t service_handle,uint16_t uuid, uint8_t uuid_type, uint16_t* char_value_handle)
{  
		ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read  = 1;
    char_md.char_props.write = 1;
		char_md.char_props.notify = 1;

	

		ble_uuid_t ble_uuid;
    //ble_uuid.type = BLE_UUID_TYPE_BLE;
    //ble_uuid.uuid = BLE_UUID_TEMPERATURE_MEASUREMENT_CHAR;
	  ble_uuid.type = uuid_type;
    ble_uuid.uuid = uuid;

	  ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 0;

		ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 7*sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 7*sizeof(uint8_t);
		uint8_t valueBuf[]={0x01};
    attr_char_value.p_value   = valueBuf;


		ble_gatts_char_handles_t gatts_char_handle;
    uint32_t err_code = sd_ble_gatts_characteristic_add(service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &gatts_char_handle);
		*char_value_handle = gatts_char_handle.value_handle;
		NRF_LOG_INFO("error code: %d",err_code);																			 
		
    VERIFY_SUCCESS(err_code);

}

/******************self end*************************/
/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

	  // security
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;

    ble_uuid_t adv_uuids[] = {{LBS_UUID_SERVICE, BLE_UUID_TYPE_BLE}};

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;
    err_code = ble_advdata_set(&advdata, &srdata);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t           err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						NRF_LOG_INFO("Connect handle %d",m_conn_handle);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

#ifndef S140
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
#endif

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}




/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
	   //NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, callbackFunc, NULL);
}




static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}



/**@brief Sensors.
 */
#define TWI_FREQUENCY_FREQUENCY_K50 (0x00cc0000UL) /*!< 50 kbps */

static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(0);
void sensorInit()
{
    ret_code_t ret;
    const nrf_drv_twi_config_t config =
    {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = TWI_FREQUENCY_FREQUENCY_K100,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    ret = nrf_drv_twi_init(&m_twi_master, &config, NULL, NULL);

    if (NRF_SUCCESS == ret)
    {
        nrf_drv_twi_enable(&m_twi_master);
    }
}

#define SENSOR_I2C_ADDR                   0x29    
void sensorWrite(uint8_t* buf,uint8_t length)
{
	ret_code_t ret;
	do
	{
		 ret = nrf_drv_twi_tx(&m_twi_master, SENSOR_I2C_ADDR, buf,length, false);
		 if (NRF_SUCCESS != ret)
		 {
				 break;
		 }
		 //ret = nrf_drv_twi_rx(&m_twi_master, EEPROM_SIM_ADDR, pdata, size);
	}while (0);
}

void sensorRead(uint8_t* buf,uint8_t length)
{
	ret_code_t ret;
	do
	{
		 ret = nrf_drv_twi_rx(&m_twi_master, SENSOR_I2C_ADDR, buf,length);
		 if (NRF_SUCCESS != ret)
		 {
				 break;
		 }
	}while (0);
}



/**@brief Function for application main entry.
 */
uint8_t valueBuf[7];
uint16_t torque_tempeature_handle=0;
uint8_t refleshTag = false;


void sensor_timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
		sensorRead(valueBuf,7);
		refleshTag = true;
		uint8_t writeBuf[1]={0xaa};
		sensorWrite(writeBuf,1);
}

void start_sensor_sample_timer(){
		uint32_t err_code = NRF_SUCCESS;
	
	  uint32_t time_ms = 5; //Time(in miliseconds) between consecutive compare events.

	  nrf_drv_timer_t timer_1 = NRF_DRV_TIMER_INSTANCE(1);
	  nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
	

		err_code = nrf_drv_timer_init(&timer_1, &timer_cfg, sensor_timer_event_handler);
		NRF_LOG_INFO("error: %d",err_code);
    APP_ERROR_CHECK(err_code);
	
	  uint32_t time_ticks = nrf_drv_timer_ms_to_ticks(&timer_1, time_ms);

    nrf_drv_timer_extended_compare(
         &timer_1, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&timer_1);
}


int main(void)
{
	
    // Initialize.
    timers_init();
    log_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
	
	
	// add service
	uint8_t uuid128[]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
	setUUID128(uuid128);
	
	// torque_service
		uint16_t torque_service_handle=0;
		uint16_t uuid=0x5857;
		addService(uuid, BLE_UUID_TYPE_BLE, &torque_service_handle);
		addCharacteristic(torque_service_handle,uuid+1, BLE_UUID_TYPE_BLE,&torque_tempeature_handle);
	
    advertising_init();
    conn_params_init();

	
		sensorInit();
		start_sensor_sample_timer();
		
		sd_ble_gap_tx_power_set(-4);
    advertising_start();

    for (;;)
    {	
				if(refleshTag == true){
					change_Char_Value(7,valueBuf,torque_tempeature_handle);
					sendNotify(7,valueBuf,torque_tempeature_handle);
				}


    }
}


/**
 * @}
 */

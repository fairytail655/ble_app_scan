#include <stdio.h>
#include "boards.h"
#include "bsp.h"

#include "app_timer.h"
#include "app_uart.h"

#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_qwr.h"
#include "nrf_ble_scan.h"

#define BTN_ID_SLEEP                0   /**< ID of the button used to put the application into sleep/system OFF mode. */
#define BTN_ID_WAKEUP               0   /**< ID of the button used to wake up the application. */

#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */

#define UART_TX_BUF_SIZE        32768                                   /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

// #define RECORDS_COUNT_MAX       500
#define RECORD_NAME_MAX         22

typedef union{
    struct {
        uint8_t mac[6];
        uint8_t adv_data[31];
        uint8_t name[RECORD_NAME_MAX];
        int8_t rssi;
    } section;
    uint8_t data[38+RECORD_NAME_MAX];
} scan_record_t;

NRF_BLE_SCAN_DEF(m_scan);

// static scan_record_t records[RECORDS_COUNT_MAX];
// static uint8_t records_count = 0;
static int records_rssi;

static void bsp_evt_handler(bsp_event_t evt);
static void bsp_configuration(void);
static void uart_init(void);
static void log_init(void);
static void timer_init(void);
static void power_management_init(void);
static void idle_state_handle(void);
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
static void ble_stack_init(void);
static void scan_evt_handler(scan_evt_t const * p_scan_evt);
static void scan_init(void);
static void scan_start(void);
static void scan_stop(void);
static uint32_t find_adv_name(const ble_gap_evt_adv_report_t *p_adv_report, uint8_t *name);

int main()
{
    log_init();
    timer_init();
    uart_init();
    power_management_init();
    ble_stack_init();
    scan_init();
    bsp_configuration();

    NRF_LOG_INFO("BLE UART central example started.");
    printf("Hello world!\r\n");

    for (;;)
    {
        idle_state_handle();
    }
}

static void bsp_evt_handler(bsp_event_t evt)
{
    switch (evt)
    {
    case BSP_EVENT_WAKEUP:
        NRF_LOG_INFO("Scan started");
        // NRF_LOG_INFO("wake");
        break;
    case BSP_EVENT_SLEEP:
        NRF_LOG_INFO("Scan stoped");
        // NRF_LOG_INFO("sleep");
        break;
    default:
        NRF_LOG_ERROR("Unknow bsp_evt");
        return; // no implementation needed
    }
}

static void bsp_configuration(void)
{
    ret_code_t err_code;

    err_code = bsp_init(BSP_INIT_LEDS|BSP_INIT_BUTTONS, bsp_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_event_to_button_action_assign(BTN_ID_SLEEP,
                                                 BSP_BUTTON_ACTION_RELEASE,
                                                 BSP_EVENT_SLEEP);
    APP_ERROR_CHECK(err_code);
    err_code = bsp_event_to_button_action_assign(BTN_ID_WAKEUP,
                                                 BSP_BUTTON_ACTION_PUSH,
                                                 BSP_EVENT_WAKEUP);
    APP_ERROR_CHECK(err_code);
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
            break;

        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
            NRF_LOG_ERROR("Communication error occurred while handling UART.");
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the UART. */
static void uart_init(void)
{
    ret_code_t err_code;

    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code= nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        
            NRF_LOG_INFO("Connected.");

            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            break;

        case BLE_GAP_EVT_DISCONNECTED:

            NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported.
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

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

        default:
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
}

/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    // scan_record_t record;
    ble_gap_evt_adv_report_t const * p_adv;
    // uint8_t i;

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_NOT_FOUND:
        {
            p_adv = p_scan_evt->params.p_not_found;
            // if ((p_adv->rssi >= records_rssi) && (check_update(p_adv->peer_addr.addr) == true))
            // {
            //     for (i = 0; i < 6; i++)
            //     {
            //         record.section.mac[i] = p_adv->peer_addr.addr[5-i];
            //     }
            //     for (i = 0; i < p_adv->data.len; i++)
            //     {
            //         record.section.adv_data[i] = p_adv->data.p_data[i];
            //     }
            //     if (find_adv_name(p_adv, record.section.name) == NRF_ERROR_NOT_FOUND)
            //     {
            //         record.section.name[0] = 'N';
            //         record.section.name[1] = '/';
            //         record.section.name[2] = 'A';
            //         record.section.name[3] = '\0';
            //     }
            //     record.section.rssi = p_adv->rssi;
            // }
        } break;

        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
            NRF_LOG_INFO("Scan timed out.");
        } break;

        default:
            break;
    }
}

/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;
    const ble_gap_scan_params_t m_scan_param = {
        .active = 0x00,
        // .interval = NRF_BLE_SCAN_SCAN_INTERVAL,
        // .interval = 320,
        .interval = 160,
        .window = NRF_BLE_SCAN_SCAN_WINDOW,
        .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
        .timeout = NRF_BLE_SCAN_SCAN_DURATION,
        .scan_phys = BLE_GAP_PHY_1MBPS,
    };
    // nrf_drv_gpiote_in_config_t in_config_toggle = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);


    memset(&init_scan, 0, sizeof(init_scan));
    init_scan.connect_if_match = false;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;
    init_scan.p_scan_param = &m_scan_param;
    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function to start scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}

/**@brief Function to start scanning. */
static void scan_stop(void)
{
    ret_code_t ret;

    nrf_ble_scan_stop();
    APP_ERROR_CHECK(ret);


    ret = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(ret);
}

static uint32_t find_adv_name(const ble_gap_evt_adv_report_t *p_adv_report, uint8_t *name)
{
    uint8_t *p_data = p_adv_report->data.p_data;
    uint8_t index = 0, i;
    uint8_t field_len, field_type;

    while (index < p_adv_report->data.len)
    {
        field_len = p_data[index];
        field_type = p_data[index + 1];
        if ((field_type == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME) ||
            (field_type == BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME))
        {
            for (i = 0; i < field_len-1 && i < RECORD_NAME_MAX-1; i++)
            {
                name[i] = p_adv_report->data.p_data[index + 2 + i];
            }
            name[i] = '\0';
            return NRF_SUCCESS;
        }
        index += field_len + 1;
    }

    return NRF_ERROR_NOT_FOUND;
}

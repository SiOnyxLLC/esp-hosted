// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#ifdef CONFIG_BT_ENABLED
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/lldesc.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "esp_system.h"
#include "slave_bt.h"

#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32S3)
    #include "esp_private/gdma.h"
    #if BLUETOOTH_UART
        #include "hal/uhci_ll.h"
    #endif
#endif

static const char BT_TAG[] = "FW_BT";
extern QueueHandle_t to_host_queue[MAX_PRIORITY_QUEUES];

#if BLUETOOTH_HCI
/* ***** HCI specific part ***** */

#define VHCI_MAX_TIMEOUT_MS 	2000
static SemaphoreHandle_t vhci_send_sem;

static void controller_rcv_pkt_ready(void)
{
	if (vhci_send_sem)
		xSemaphoreGive(vhci_send_sem);
}

/* Set just before a runtime esp_bt_controller_enable(): the controller then announces itself with
 * a Command Complete for opcode 0x0000 (no command). Swallow that one event, so the host sees exactly
 * one Command Complete for its 0xFFA0 switch command (hcitool otherwise takes the NOP as the reply). */
static volatile bool bt_swallow_nop_cc;

static int host_rcv_pkt(uint8_t *data, uint16_t len)
{
	esp_err_t ret = ESP_OK;
	interface_buffer_handle_t buf_handle;
	uint8_t *buf = NULL;

	if (bt_swallow_nop_cc && len >= 6 && data[0] == 0x04 && data[1] == 0x0E &&
	    data[4] == 0x00 && data[5] == 0x00) {
		bt_swallow_nop_cc = false;
		return 0;
	}

	buf = (uint8_t *) malloc(len);

	if (!buf) {
		ESP_LOGE(BT_TAG, "HCI Send packet: memory allocation failed");
		return ESP_FAIL;
	}

	memcpy(buf, data, len);

	memset(&buf_handle, 0, sizeof(buf_handle));

	buf_handle.if_type = ESP_HCI_IF;
	buf_handle.if_num = 0;
	buf_handle.payload_len = len;
	buf_handle.payload = buf;
	buf_handle.wlan_buf_handle = buf;
	buf_handle.free_buf_handle = free;

#if CONFIG_ESP_BT_DEBUG
	ESP_LOG_BUFFER_HEXDUMP("bt_tx", data, len, ESP_LOG_INFO);
#endif
	ret = xQueueSend(to_host_queue[PRIO_Q_MID], &buf_handle, portMAX_DELAY);

	if (ret != pdTRUE) {
		ESP_LOGE(BT_TAG, "HCI send packet: Failed to send buffer\n");
		free(buf);
		return ESP_FAIL;
	}

	return 0;
}

static esp_vhci_host_callback_t vhci_host_cb = {
	controller_rcv_pkt_ready,
	host_rcv_pkt
};

/*
 * Runtime BT controller switch, driven by the host over the existing HCI channel (no SDIO
 * protocol or host-driver change; esp_fw_version.h untouched, so the host still matches
 * NG-1.0.3.0.1).
 *
 * HCI vendor command 0xFFA0 (OGF 0x3F, OCF 0x3A0), one parameter byte:
 *   0x00 = esp_bt_controller_disable()   0x01 = esp_bt_controller_enable(ESP_BT_MODE_BLE)
 *   0x02 = query only
 * Reply: Command Complete, return parameters = status (0x00 ok, 0x12 bad parameter,
 * 0x1F unspecified error) and esp_bt_controller_status_t after the operation (1 off, 2 on).
 *   e.g. from the host:  hcitool -i hci0 cmd 0x3f 0x3a0 00
 *
 * disable()/enable() only, never deinit(): init-after-deinit is unsupported on C6 (esp_bt.h),
 * and esp_bt_mem_release() is irreversible. disable() leaves coex initialised (Wi-Fi owns it).
 * While the controller is off, every other HCI command is answered at once with Command Status
 * 0x0C (Command Disallowed) instead of timing out on the host, and ACL data is dropped.
 * The controller restarts with no host state: after "on" the host must re-run its BLE setup
 * (on KL730: systemctl restart sips-bluetooth).
 */
#define HCI_H4_CMD			0x01
#define HCI_H4_EVT			0x04
#define HCI_EV_CMD_COMPLETE		0x0E
#define HCI_EV_CMD_STATUS		0x0F
#define SNX_VS_BT_SWITCH_OPCODE		0xFFA0
#define HCI_ERR_CMD_DISALLOWED		0x0C
#define HCI_ERR_INVALID_PARAM		0x12
#define HCI_ERR_UNSPECIFIED		0x1F

static volatile bool bt_runtime_off;

static void hci_reply_to_host(uint8_t *ev, uint16_t len)
{
	if (host_rcv_pkt(ev, len))
		ESP_LOGE(BT_TAG, "BT switch: could not queue HCI event to host");
}

static void bt_switch_handle(uint16_t opcode, const uint8_t *param, uint8_t plen)
{
	uint8_t status = 0;
	esp_err_t r = ESP_OK;
	uint8_t ev[8];

	if (plen < 1 || param[0] > 2) {
		status = HCI_ERR_INVALID_PARAM;
	} else if (param[0] == 0 && esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
		r = esp_bt_controller_disable();
		if (r == ESP_OK)
			bt_runtime_off = true;
	} else if (param[0] == 1 && esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
		bt_swallow_nop_cc = true;
		r = esp_bt_controller_enable(ESP_BT_MODE_BLE);
		if (r == ESP_OK)
			bt_runtime_off = false;
	}
	if (r != ESP_OK)
		status = HCI_ERR_UNSPECIFIED;

	ESP_LOGW(BT_TAG, "BT switch op %d -> %s, controller status %d, heap free %lu min %lu",
			plen ? param[0] : -1, esp_err_to_name(r), esp_bt_controller_get_status(),
			(unsigned long)esp_get_free_heap_size(), (unsigned long)esp_get_minimum_free_heap_size());

	ev[0] = HCI_H4_EVT;
	ev[1] = HCI_EV_CMD_COMPLETE;
	ev[2] = 5;			/* plen */
	ev[3] = 1;			/* Num_HCI_Command_Packets */
	ev[4] = opcode & 0xff;
	ev[5] = opcode >> 8;
	ev[6] = status;
	ev[7] = (uint8_t)esp_bt_controller_get_status();
	hci_reply_to_host(ev, sizeof(ev));
}

static void hci_reply_disallowed(uint16_t opcode)
{
	uint8_t ev[] = { HCI_H4_EVT, HCI_EV_CMD_STATUS, 4, HCI_ERR_CMD_DISALLOWED, 1,
			 opcode & 0xff, opcode >> 8 };

	hci_reply_to_host(ev, sizeof(ev));
}

void process_hci_rx_pkt(uint8_t *payload, uint16_t payload_len) {
	/* VHCI needs one extra byte at the start of payload */
	/* that is accomodated in esp_payload_header */
#if CONFIG_ESP_BT_DEBUG
	ESP_LOG_BUFFER_HEXDUMP("bt_rx", payload, payload_len, ESP_LOG_INFO);
#endif
	payload--;
	payload_len++;

	/* payload[0] = H4 packet type; a command is opcode(2) + plen(1) + params */
	if (payload[0] == HCI_H4_CMD && payload_len >= 4) {
		uint16_t opcode = payload[1] | (payload[2] << 8);

		bt_swallow_nop_cc = false;	/* only the NOP right after our own enable is swallowed */

		if (opcode == SNX_VS_BT_SWITCH_OPCODE) {
			bt_switch_handle(opcode, &payload[4], payload_len - 4 < payload[3] ? payload_len - 4 : payload[3]);
			return;
		}
		if (bt_runtime_off) {
			hci_reply_disallowed(opcode);
			return;
		}
	}
	if (bt_runtime_off)
		return;		/* ACL while the controller is off: drop */

	if (!esp_vhci_host_check_send_available()) {
		ESP_LOGD(BT_TAG, "VHCI not available");
	}

#if SOC_ESP_NIMBLE_CONTROLLER
	esp_vhci_host_send_packet(payload, payload_len);
#else
	if (vhci_send_sem) {
		if (xSemaphoreTake(vhci_send_sem, VHCI_MAX_TIMEOUT_MS) == pdTRUE) {
			esp_vhci_host_send_packet(payload, payload_len);
		} else {
			ESP_LOGI(BT_TAG, "VHCI sem timeout");
		}
	}
#endif
}

#elif BLUETOOTH_UART
/* ***** UART specific part ***** */

#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32S3)
// Operation functions for HCI UART Transport Layer
static bool hci_uart_tl_init(void);
static void hci_uart_tl_deinit(void);
static void hci_uart_tl_recv_async(uint8_t *buf, uint32_t size, esp_bt_hci_tl_callback_t callback, void *arg);
static void hci_uart_tl_send_async(uint8_t *buf, uint32_t size, esp_bt_hci_tl_callback_t callback, void *arg);
static void hci_uart_tl_flow_on(void);
static bool hci_uart_tl_flow_off(void);
static void hci_uart_tl_finish_transfers(void);

struct uart_txrxchannel {
	esp_bt_hci_tl_callback_t callback;
	void *arg;
	lldesc_t link;
};

struct uart_env_tag {
	struct uart_txrxchannel tx;
	struct uart_txrxchannel rx;
};

struct uart_env_tag uart_env;

static volatile uhci_dev_t *s_uhci_hw = &UHCI0;
static gdma_channel_handle_t s_rx_channel;
static gdma_channel_handle_t s_tx_channel;

static esp_bt_hci_tl_t s_hci_uart_tl_funcs = {
	._magic = ESP_BT_HCI_TL_MAGIC_VALUE,
	._version = ESP_BT_HCI_TL_VERSION,
	._reserved = 0,
	._open = (void *)hci_uart_tl_init,
	._close = (void *)hci_uart_tl_deinit,
	._finish_transfers = (void *)hci_uart_tl_finish_transfers,
	._recv = (void *)hci_uart_tl_recv_async,
	._send = (void *)hci_uart_tl_send_async,
	._flow_on = (void *)hci_uart_tl_flow_on,
	._flow_off = (void *)hci_uart_tl_flow_off,
};

static bool hci_uart_tl_init(void)
{
	return true;
}

static void hci_uart_tl_deinit(void)
{
}

static IRAM_ATTR void hci_uart_tl_recv_async(uint8_t *buf, uint32_t size,
						esp_bt_hci_tl_callback_t callback, void *arg)
{
	assert(buf != NULL);
	assert(size != 0);
	assert(callback != NULL);
	uart_env.rx.callback = callback;
	uart_env.rx.arg = arg;

	memset(&uart_env.rx.link, 0, sizeof(lldesc_t));
	uart_env.rx.link.buf = buf;
	uart_env.rx.link.size = size;

	s_uhci_hw->pkt_thres.thrs = size;

	gdma_start(s_rx_channel, (intptr_t)(&uart_env.rx.link));
}

static IRAM_ATTR void hci_uart_tl_send_async(uint8_t *buf, uint32_t size,
						esp_bt_hci_tl_callback_t callback, void *arg)
{
	assert(buf != NULL);
	assert(size != 0);
	assert(callback != NULL);

	uart_env.tx.callback = callback;
	uart_env.tx.arg = arg;

	memset(&uart_env.tx.link, 0, sizeof(lldesc_t));
	uart_env.tx.link.length = size;
	uart_env.tx.link.buf = buf;
	uart_env.tx.link.eof = 1;

	gdma_start(s_tx_channel, (intptr_t)(&uart_env.tx.link));
}

static void hci_uart_tl_flow_on(void)
{
}

static bool hci_uart_tl_flow_off(void)
{
	return true;
}

static void hci_uart_tl_finish_transfers(void)
{
}

static IRAM_ATTR bool hci_uart_tl_rx_eof_callback(gdma_channel_handle_t dma_chan,
								gdma_event_data_t *event_data, void *user_data)
{
	assert(dma_chan == s_rx_channel);
	assert(uart_env.rx.callback != NULL);
	esp_bt_hci_tl_callback_t callback = uart_env.rx.callback;
	void *arg = uart_env.rx.arg;

	// clear callback pointer
	uart_env.rx.callback = NULL;
	uart_env.rx.arg = NULL;

	// call handler
	callback(arg, ESP_BT_HCI_TL_STATUS_OK);

	// send notification to Bluetooth Controller task
	esp_bt_h4tl_eif_io_event_notify(1);

	return true;
}

static IRAM_ATTR bool hci_uart_tl_tx_eof_callback(gdma_channel_handle_t dma_chan,
								gdma_event_data_t *event_data, void *user_data)
{
	assert(dma_chan == s_tx_channel);
	assert(uart_env.tx.callback != NULL);
	esp_bt_hci_tl_callback_t callback = uart_env.tx.callback;
	void *arg = uart_env.tx.arg;

	// clear callback pointer
	uart_env.tx.callback = NULL;
	uart_env.tx.arg = NULL;

	// call handler
	callback(arg, ESP_BT_HCI_TL_STATUS_OK);

	// send notification to Bluetooth Controller task
	esp_bt_h4tl_eif_io_event_notify(1);

	return true;
}
#endif

#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32S3)
static void init_uart(void)
{
	ESP_LOGD(BT_TAG, "Set-up BLE for ESP32-C3/ESP32-S3");
#if BLUETOOTH_UART == 1
	periph_module_enable(PERIPH_UART1_MODULE);
	periph_module_reset(PERIPH_UART1_MODULE);
#elif BLUETOOTH_UART == 2
	periph_module_enable(PERIPH_UART2_MODULE);
	periph_module_reset(PERIPH_UART2_MODULE);
#endif

	periph_module_enable(PERIPH_UHCI0_MODULE);
	periph_module_reset(PERIPH_UHCI0_MODULE);

	gpio_config_t io_output_conf = {
		.intr_type = GPIO_INTR_DISABLE,	//disable interrupt
		.mode = GPIO_MODE_OUTPUT,	// output mode
		.pin_bit_mask = GPIO_OUTPUT_PIN_SEL,	// bit mask of the output pins
		.pull_down_en = 0,	// disable pull-down mode
		.pull_up_en = 0,	// disable pull-up mode
	};
	gpio_config(&io_output_conf);

	gpio_config_t io_input_conf = {
		.intr_type = GPIO_INTR_DISABLE,	//disable interrupt
		.mode = GPIO_MODE_INPUT,	// input mode
		.pin_bit_mask = GPIO_INPUT_PIN_SEL,  // bit mask of the input pins
		.pull_down_en = 0,	// disable pull-down mode
		.pull_up_en = 0,	// disable pull-down mode
	};
	gpio_config(&io_input_conf);

	ESP_ERROR_CHECK( uart_set_pin(BLUETOOTH_UART, BT_TX_PIN,
				BT_RX_PIN, BT_RTS_PIN, BT_CTS_PIN) );
	ESP_LOGI(BT_TAG, "UART Pins: Tx:%u Rx:%u RTS:%u CTS:%u",
			BT_TX_PIN, BT_RX_PIN, BT_RTS_PIN, BT_CTS_PIN);

	// configure UART1
	ESP_LOGI(BT_TAG, "baud rate for HCI uart :: %d\n",
					CONFIG_EXAMPLE_HCI_UART_BAUDRATE);

	uart_config_t uart_config = {
		.baud_rate = CONFIG_EXAMPLE_HCI_UART_BAUDRATE,

		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
		.rx_flow_ctrl_thresh = UART_RX_THRS,
		.source_clk = UART_SCLK_APB,
	};
	ESP_ERROR_CHECK(uart_param_config(BLUETOOTH_UART, &uart_config));

	// install DMA driver
	gdma_channel_alloc_config_t tx_channel_config = {
		.flags.reserve_sibling = 1,
		.direction = GDMA_CHANNEL_DIRECTION_TX,
	};
	ESP_ERROR_CHECK(gdma_new_channel(&tx_channel_config, &s_tx_channel));
	gdma_channel_alloc_config_t rx_channel_config = {
		.direction = GDMA_CHANNEL_DIRECTION_RX,
		.sibling_chan = s_tx_channel,
	};
	ESP_ERROR_CHECK(gdma_new_channel(&rx_channel_config, &s_rx_channel));

	gdma_connect(s_tx_channel, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_UHCI, 0));
	gdma_connect(s_rx_channel, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_UHCI, 0));

	gdma_strategy_config_t strategy_config = {
		.auto_update_desc = false,
		.owner_check = false
	};
	gdma_apply_strategy(s_tx_channel, &strategy_config);
	gdma_apply_strategy(s_rx_channel, &strategy_config);

	gdma_rx_event_callbacks_t rx_cbs = {
		.on_recv_eof = hci_uart_tl_rx_eof_callback
	};
	gdma_register_rx_event_callbacks(s_rx_channel, &rx_cbs, NULL);

	gdma_tx_event_callbacks_t tx_cbs = {
		.on_trans_eof = hci_uart_tl_tx_eof_callback
	};
	gdma_register_tx_event_callbacks(s_tx_channel, &tx_cbs, NULL);

	// configure UHCI
	uhci_ll_init(s_uhci_hw);
	uhci_ll_set_eof_mode(s_uhci_hw, UHCI_RX_LEN_EOF);
	// disable software flow control
	s_uhci_hw->escape_conf.val = 0;
	uhci_ll_attach_uart_port(s_uhci_hw, 1);
}
#elif CONFIG_IDF_TARGET_ESP32
static void init_uart(void)
{
	ESP_LOGD(BT_TAG, "Set-up BLE for ESP32");
#if BLUETOOTH_UART == 1
	periph_module_enable(PERIPH_UART1_MODULE);
	periph_module_reset(PERIPH_UART1_MODULE);
#elif BLUETOOTH_UART == 2
	periph_module_enable(PERIPH_UART2_MODULE);
	periph_module_reset(PERIPH_UART2_MODULE);
#endif

	periph_module_enable(PERIPH_UHCI0_MODULE);
	periph_module_reset(PERIPH_UHCI0_MODULE);


	ESP_ERROR_CHECK( uart_set_pin(BLUETOOTH_UART, BT_TX_PIN,
		BT_RX_PIN, BT_RTS_PIN, BT_CTS_PIN) );
	ESP_LOGI(BT_TAG, "UART Pins: Tx:%u Rx:%u RTS:%u CTS:%u",
			BT_TX_PIN, BT_RX_PIN, BT_RTS_PIN, BT_CTS_PIN);

}
#elif defined(CONFIG_IDF_TARGET_ESP32C2) || defined(CONFIG_IDF_TARGET_ESP32C6)
static void init_uart(void)
{
	ESP_LOGD(BT_TAG, "Set-up BLE for ESP32-C2/C6");

#if defined(CONFIG_IDF_TARGET_ESP32C2)
	ESP_LOGI(BT_TAG, "UART Pins: Tx:%u Rx:%u", BT_TX_PIN, BT_RX_PIN);
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
	ESP_LOGI(BT_TAG, "UART Pins: Tx:%u Rx:%u", BT_TX_PIN, BT_RX_PIN);
#endif
}
#endif
#endif

#if BLUETOOTH_HCI
#if SOC_ESP_NIMBLE_CONTROLLER
#include "nimble/ble_hci_trans.h"

typedef enum {
    DATA_TYPE_COMMAND = 1,
    DATA_TYPE_ACL     = 2,
    DATA_TYPE_SCO     = 3,
    DATA_TYPE_EVENT   = 4
} serial_data_type_t;

/* Host-to-controller command. */
#define BLE_HCI_TRANS_BUF_CMD       3

/* ACL_DATA_MBUF_LEADINGSPCAE: The leadingspace in user info header for ACL data */
#define ACL_DATA_MBUF_LEADINGSPCAE    4

void esp_vhci_host_send_packet(uint8_t *data, uint16_t len)
{
    if (*(data) == DATA_TYPE_COMMAND) {
        struct ble_hci_cmd *cmd = NULL;
        cmd = (struct ble_hci_cmd *) ble_hci_trans_buf_alloc(BLE_HCI_TRANS_BUF_CMD);
	if (!cmd) {
		ESP_LOGE(BT_TAG, "Failed to allocate memory for HCI transport buffer");
		return;
	}
        memcpy((uint8_t *)cmd, data + 1, len - 1);
        ble_hci_trans_hs_cmd_tx((uint8_t *)cmd);
    }

    if (*(data) == DATA_TYPE_ACL) {
        struct os_mbuf *om = os_msys_get_pkthdr(len, ACL_DATA_MBUF_LEADINGSPCAE);
        assert(om);
        os_mbuf_append(om, &data[1], len - 1);
        ble_hci_trans_hs_acl_tx(om);
    }
}

bool esp_vhci_host_check_send_available() {
    // not need to check in esp new controller
    return true;
}

int ble_hs_hci_rx_evt(uint8_t *hci_ev, void *arg)
{
    uint16_t len = hci_ev[1] + 3;
    uint8_t *data = (uint8_t *)malloc(len);
    data[0] = 0x04;
    memcpy(&data[1], hci_ev, len - 1);
    ble_hci_trans_buf_free(hci_ev);
    vhci_host_cb.notify_host_recv(data, len);
    free(data);
    return 0;
}


int ble_hs_rx_data(struct os_mbuf *om, void *arg)
{
    uint16_t len = om->om_len + 1;
    uint8_t *data = (uint8_t *)malloc(len);
    data[0] = 0x02;
    os_mbuf_copydata(om, 0, len - 1, &data[1]);
    vhci_host_cb.notify_host_recv(data, len);
    free(data);
    os_mbuf_free_chain(om);
    return 0;
}

#endif
#endif

esp_err_t initialise_bluetooth(void)
{
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();


#ifdef BLUETOOTH_UART
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32S3)
	bt_cfg.hci_tl_funcs = &s_hci_uart_tl_funcs;
#endif

	init_uart();
#endif
	ESP_ERROR_CHECK( esp_bt_controller_init(&bt_cfg) );
#if BLUETOOTH_BLE
	ESP_ERROR_CHECK( esp_bt_controller_enable(ESP_BT_MODE_BLE) );
#elif BLUETOOTH_BT
	ESP_ERROR_CHECK( esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT) );
#elif BLUETOOTH_BT_BLE
	ESP_ERROR_CHECK( esp_bt_controller_enable(ESP_BT_MODE_BTDM) );
#endif

#if BLUETOOTH_HCI
	esp_err_t ret = ESP_OK;

#if SOC_ESP_NIMBLE_CONTROLLER
	ble_hci_trans_cfg_hs((ble_hci_trans_rx_cmd_fn *)ble_hs_hci_rx_evt,NULL,
				(ble_hci_trans_rx_acl_fn *)ble_hs_rx_data,NULL);
#else
	ret = esp_vhci_host_register_callback(&vhci_host_cb);
#endif

	if (ret != ESP_OK) {
		ESP_LOGE(BT_TAG, "Failed to register VHCI callback");
		return ret;
	}

	vhci_send_sem = xSemaphoreCreateBinary();
	if (vhci_send_sem == NULL) {
		ESP_LOGE(BT_TAG, "Failed to create VHCI send sem");
		return ESP_ERR_NO_MEM;
	}

	xSemaphoreGive(vhci_send_sem);
#endif

	return ESP_OK;
}

void deinitialize_bluetooth(void)
{
#if BLUETOOTH_HCI
	if (vhci_send_sem) {
		/* Dummy take and give sema before deleting it */
		xSemaphoreTake(vhci_send_sem, portMAX_DELAY);
		xSemaphoreGive(vhci_send_sem);
		vSemaphoreDelete(vhci_send_sem);
		vhci_send_sem = NULL;
	}
	esp_bt_controller_disable();
	esp_bt_controller_deinit();
#endif
}

uint8_t get_bluetooth_capabilities(void)
{
	uint8_t cap = 0;
	ESP_LOGI(BT_TAG, "- BT/BLE");
#if BLUETOOTH_HCI
#if CONFIG_ESP_SPI_HOST_INTERFACE
	ESP_LOGI(BT_TAG, "   - HCI Over SPI");
	cap |= ESP_BT_SPI_SUPPORT;
#else
	ESP_LOGI(BT_TAG, "   - HCI Over SDIO");
	cap |= ESP_BT_SDIO_SUPPORT;
#endif
#elif BLUETOOTH_UART
	ESP_LOGI(BT_TAG, "   - HCI Over UART");
	cap |= ESP_BT_UART_SUPPORT;
#endif

#if BLUETOOTH_BLE
	ESP_LOGI(BT_TAG, "   - BLE only");
	cap |= ESP_BLE_ONLY_SUPPORT;
#elif BLUETOOTH_BT
	ESP_LOGI(BT_TAG, "   - BR_EDR only");
	cap |= ESP_BR_EDR_ONLY_SUPPORT;
#elif BLUETOOTH_BT_BLE
	ESP_LOGI(BT_TAG, "   - BT/BLE dual mode");
	cap |= ESP_BLE_ONLY_SUPPORT | ESP_BR_EDR_ONLY_SUPPORT;
#endif
	return cap;
}


#endif

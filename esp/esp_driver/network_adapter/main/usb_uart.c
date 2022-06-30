/* USB Example

	 This example code is in the Public Domain (or CC0 licensed, at your option.)

	 Unless required by applicable law or agreed to in writing, this
	 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
	 CONDITIONS OF ANY KIND, either express or implied.
*/

// DESCRIPTION:
// This example contains minimal code to make ESP32-S2 based device
// recognizable by USB-host devices as a USB Serial Device.

#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include <stdio.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include <string.h>

#define ECHO_TEST_TXD (37)
#define ECHO_TEST_RXD (39)
#define ECHO_TEST_RTS (-1)
#define ECHO_TEST_CTS (-1)

#define ECHO_UART_PORT_NUM (1)
#define ECHO_UART_BAUD_RATE (115200)
#define ECHO_TASK_STACK_SIZE (2048)

#define BUF_SIZE (1024)

#define GPIO_IO_EN (6)
#define GPIO_IO_RTS (18)
#define GPIO_IO_LED (15)

static const char *TAG = "example";
static uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];

static bool dtr = 0;
static bool rts = 0;
static uint32_t bit_rate = 0;
static uint8_t stop_bits = 0;
static uint8_t parity = 0;
static uint8_t data_bits = 0;

void gpio_config_input_pull_up(int mask)
{
	gpio_config_t io_conf = {
			.intr_type = GPIO_INTR_DISABLE,
			.mode = GPIO_MODE_INPUT,
			.pin_bit_mask = mask,
			.pull_down_en = 0,
			.pull_up_en = 1,
	};
	gpio_config(&io_conf);
}

void gpio_config_output(int gpio_num, uint32_t value)
{
	gpio_config_t io_conf = {
			.intr_type = GPIO_INTR_DISABLE,
			.mode = GPIO_MODE_OUTPUT,
			.pin_bit_mask = (1ULL << gpio_num),
			.pull_down_en = 0,
			.pull_up_en = 0,
	};
	gpio_config(&io_conf);
	gpio_set_level(gpio_num, value);
}

void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
	size_t rx_size = 0;
	esp_err_t ret = tinyusb_cdcacm_read(itf, buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
	if (ret == ESP_OK)
	{
		buf[rx_size] = '\0';
		uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)buf, rx_size);
		// ESP_LOGI(TAG, "Got data (%d bytes): %s", rx_size, buf);
	}
	else
	{
		ESP_LOGE(TAG, "Read error");
	}
}

void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
	if (dtr != event->line_state_changed_data.dtr || rts != event->line_state_changed_data.rts)
	{
		dtr = event->line_state_changed_data.dtr;
		rts = event->line_state_changed_data.rts;
		if (dtr == rts)
		{
			gpio_config_input_pull_up((1ULL << GPIO_IO_EN) | (1ULL << GPIO_IO_RTS));
		}
		else
		{
			if (rts)
			{
				gpio_config_input_pull_up(1ULL << GPIO_IO_EN);
				gpio_config_output(GPIO_IO_RTS, 0);
			}
			else
			{
				gpio_config_output(GPIO_IO_EN, 0);
				gpio_config_input_pull_up(1ULL << GPIO_IO_RTS);
			}
		}
	}
}

void tinyusb_cdc_line_coding_changed_callback(int itf, cdcacm_event_t *event)
{
	if (bit_rate != event->line_coding_changed_data.p_line_coding->bit_rate ||
			stop_bits != event->line_coding_changed_data.p_line_coding->stop_bits ||
			parity != event->line_coding_changed_data.p_line_coding->parity ||
			data_bits != event->line_coding_changed_data.p_line_coding->data_bits)
	{
		bit_rate = event->line_coding_changed_data.p_line_coding->bit_rate;
		stop_bits = event->line_coding_changed_data.p_line_coding->stop_bits;
		parity = event->line_coding_changed_data.p_line_coding->parity;
		data_bits = event->line_coding_changed_data.p_line_coding->data_bits;

		gpio_config_output(GPIO_IO_LED, 1);

		ESP_LOGI(TAG, "bit_rate: %d, stop_bits: %d, parity: %d, data_bits: %d", bit_rate, stop_bits, parity, data_bits);

		uart_set_baudrate(ECHO_UART_PORT_NUM, bit_rate);

		if (stop_bits == 1)
			uart_set_stop_bits(ECHO_UART_PORT_NUM, UART_STOP_BITS_1_5);
		else if (stop_bits == 2)
			uart_set_stop_bits(ECHO_UART_PORT_NUM, UART_STOP_BITS_2);
		else
			uart_set_stop_bits(ECHO_UART_PORT_NUM, UART_STOP_BITS_1);

		if (parity == 1)
			uart_set_parity(ECHO_UART_PORT_NUM, UART_PARITY_ODD);
		else if (parity == 2)
			uart_set_parity(ECHO_UART_PORT_NUM, UART_PARITY_EVEN);
		else
			uart_set_parity(ECHO_UART_PORT_NUM, UART_PARITY_DISABLE);

		if (data_bits == 5)
			uart_set_word_length(ECHO_UART_PORT_NUM, UART_DATA_5_BITS);
		else if (data_bits == 6)
			uart_set_word_length(ECHO_UART_PORT_NUM, UART_DATA_6_BITS);
		else if (data_bits == 7)
			uart_set_word_length(ECHO_UART_PORT_NUM, UART_DATA_7_BITS);
		else
			uart_set_word_length(ECHO_UART_PORT_NUM, UART_DATA_8_BITS);
	}
}

void usb_uart_task(void *pvParameters)
{
	gpio_config_input_pull_up((1ULL << GPIO_IO_EN) | (1ULL << GPIO_IO_RTS));

	uart_config_t uart_config = {
			.baud_rate = ECHO_UART_BAUD_RATE,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.source_clk = UART_SCLK_APB,
	};
	
	ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, /* ESP_INTR_FLAG_IRAM */0));
	ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

	ESP_LOGI(TAG, "USB initialization");
	tinyusb_config_t tusb_cfg = {}; // the configuration using default values
	ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

	tinyusb_config_cdcacm_t amc_cfg = {
			.usb_dev = TINYUSB_USBDEV_0,
			.cdc_port = TINYUSB_CDC_ACM_0,
			.rx_unread_buf_sz = 64,
			.callback_rx = &tinyusb_cdc_rx_callback, // the first way to register a callback
			.callback_rx_wanted_char = NULL,
			.callback_line_state_changed = &tinyusb_cdc_line_state_changed_callback,
			.callback_line_coding_changed = &tinyusb_cdc_line_coding_changed_callback};

	ESP_ERROR_CHECK(tusb_cdc_acm_init(&amc_cfg));
	ESP_LOGI(TAG, "USB initialization DONE");

	uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
	while (1)
	{
		int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, BUF_SIZE, 1 / portTICK_RATE_MS);
		if (len)
		{
			// ESP_LOGI(TAG, "len: %d", len);
			tinyusb_cdcacm_write_queue(0, data, len);
			tinyusb_cdcacm_write_flush(0, 0);
		}
	}
}

// void app_main(void)
// {
// 	assert(xTaskCreate(usb_uart_task, "usb_uart_task", ECHO_TASK_STACK_SIZE, NULL, 2, NULL) == pdTRUE);
// 	// usb_uart_task(NULL);
// }

/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileCopyrightText: 2026 Ignacy Oziero
 * SPDX-License-Identifier: Unlicense
 */
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include "common.h"
#include "driver/rmt_types_legacy.h"
#include "esp_err.h"
#include "esp_check.h"
#include "events.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "hal/uart_types.h"
#include "driver/gpio.h"
#include "ir_nec_encoder.h"
#include "lwip/err.h"
#include "portmacro.h"
#include "soc/gpio_num.h"
#include "uart.h"
#include "ir_nec.h"

#define IR_RX_GPIO GPIO_NUM_27
#define IR_TX_GPIO GPIO_NUM_26
#define IR_NEC_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us
#define TAG "Main"
#define CFG_ERR_CORR (1 << 0)
#define CFG_BIG_ENDIAN (1 << 1)
#define DEFAULT_CFG_FLAG 0


uart_m_t uart_m;
ir_nec_t ir_nec;
task_arg_t *ir_nec_task_arg;
task_arg_t *uart_task_arg;
int cfg_flags = DEFAULT_CFG_FLAG;

static esp_err_t parse_ir_nec_tx_data(uint8_t *buf, size_t len, nec_scan_code_t *res) {
	if (len < sizeof(nec_scan_code_t)) {
		return ESP_ERR_INVALID_SIZE;
	}
	uint16_t addr = *(uint16_t*)buf;
	uint16_t command = *(uint16_t*)(buf + 2);
	
	bool err_cor = cfg_flags & CFG_ERR_CORR;
	bool big_endian = cfg_flags & CFG_BIG_ENDIAN;
	
	if (big_endian) {
		addr = (addr << 8) | (addr >> 8);
	}
	
	if (err_cor) {
		if (addr & 0xFF00 || command & 0xFF00) {
				return ERR_VAL;
			}
		addr = ~addr | (addr << 8);
		command = ~command | (command << 8);
	}
	// use err_cor and lsb arguments to format frame
	memcpy(res, buf, sizeof(nec_scan_code_t));
	return ESP_OK;
}

static esp_err_t parse_event(event_t *event) {
	
	switch (event->event_type) {
		case EVENT_UART_RX_DATA:
		{
			uart_frame_t uart_frame;
			size_t res_len = uart_read(&uart_m, &uart_frame);
			if (res_len != sizeof(uart_frame)) {
				ESP_LOGE(TAG, "Error while parsing uart frame");
				return ESP_ERR_INVALID_RESPONSE;
			}
			switch (uart_frame.hdr) {
				case CMD_CFG_SET:
					
				
				case CMD_IR_NEC_TX_DATA:
				{
					nec_scan_code_t scan;
					if (parse_ir_nec_tx_data(uart_frame.data, sizeof(uart_frame.data), &scan) != ESP_OK) {
						ESP_LOGE(TAG, "Error while parsing ir nec tx data\n");
					};
					ir_nec_write(&ir_nec, &scan);
				}
					 break;
				case CMD_IR_NEC_TX_START:
				{
					ir_nec_tx_run(ir_nec_task_arg);
				}
					break;
				case CMD_IR_NEC_TX_STOP:
				{
					ir_nec_tx_stop(&ir_nec);
					
				}
					break;
				case CMD_IR_NEC_RX_START:
				{
					ir_nec_rx_run(ir_nec_task_arg);
				}
					break;
				case CMD_IR_NEC_RX_STOP:
				{
					ir_nec_rx_stop(&ir_nec);
				}
					break;
				default:
				{
					ESP_LOGE(TAG, "Unknown command in uart frame hdr");
				}
					break;
			}
			
		}
			break;
			
		case EVENT_NEC_RX_DATA:
		{
			nec_scan_code_t scan_code;
			size_t res_len = ir_nec_read(&ir_nec, &scan_code);
			if (res_len != sizeof(scan_code)) {
				ESP_LOGE(TAG, "Error while reading from ir nec tx interm buffer");
			}
			uart_frame_t frame;
			frame.hdr = CMD_IR_NEC_RX_DATA;
			memcpy(frame.data, &scan_code, sizeof(nec_scan_code_t));
			uart_write(&uart_m, &frame);
		}
			break;
		default:
			ESP_LOGE(TAG, "Unknown event");
			return ESP_ERR_INVALID_ARG;
	}
	return ESP_OK;
}


void app_main(void)
{	
	uart_m_config_t uart_m_config = {
		.uart_event_queue_size = 20,
		.event_addr = 16,
		.rx_buffer_size = 1024,
		.tx_buffer_size = 1024,
		.baud_rate = 115200,
		.uart_port = UART_NUM_0,
		.tx_interm_buffer_size = 1024,
		.rx_interm_buffer_size = 1024,
		.tx_event_queue_size = 20
	};
	vTaskDelay(pdMS_TO_TICKS(1000*5));

	QueueHandle_t uart_q_in = xQueueCreate(128, sizeof(event_t));
	ESP_ERROR_CHECK(uart_init(&uart_m, &uart_m_config));
	
	uart_task_arg = malloc(sizeof(task_arg_t));
	uart_task_arg->dev_ptr = &uart_m;
	uart_task_arg->event_queue = uart_q_in;
	
	ESP_ERROR_CHECK(uart_rx_run(uart_task_arg));
	ESP_ERROR_CHECK(uart_tx_run(uart_task_arg));
	
	
	ir_nec_config_t ir_nec_config = {
		.tx_carrier_config = {
			.duty_cycle = 0.33, // 33 %
			.frequency_hz = 38000 // 38kHz
		},
		.resolution_hz = IR_NEC_RESOLUTION_HZ,
		.rmt_rx_queue_size = 64,
		.rx_gpio = IR_RX_GPIO,
		.tx_gpio = IR_TX_GPIO,
		.rx_interm_buf_sz = sizeof(nec_scan_code_t)*64,
		.tx_interm_buf_sz = sizeof(nec_scan_code_t)*64,
		.invert_in = 1
	};
	ir_nec_init(&ir_nec, &ir_nec_config);
	
	ir_nec_task_arg = malloc(sizeof(task_arg_t));
	ir_nec_task_arg->dev_ptr = &ir_nec;
	ir_nec_task_arg->event_queue = uart_q_in;

	
	event_t event;	
    while (true) {
		if (xQueueReceive(uart_q_in, (void*)&event, 10)) {
			ESP_LOGI(TAG, "[Event] Type: %d | Payload len: %hu", event.event_type, event.payload_len);
			parse_event(&event);
		}
		vTaskDelay(pdMS_TO_TICKS(100));
    }
}

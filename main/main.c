/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileCopyrightText: 2026 Ignacy Oziero
 * SPDX-License-Identifier: Unlicense
 */
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
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
#include "portmacro.h"
#include "soc/gpio_num.h"
#include "uart.h"
#include "ir_nec.h"

#define IR_RX_GPIO GPIO_NUM_27
#define IR_TX_GPIO GPIO_NUM_26
#define IR_NEC_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us
#define TAG "Example"

uart_m_t uart_m;
ir_nec_t ir_nec;

static inline esp_err_t parse_event(event_t *event) {
	
	switch (event->event_type) {
		case EVENT_UART_DATA:
		{
			uart_frame_t uart_frame;
			size_t res_len = uart_read(&uart_m, &uart_frame);
			//ESP_LOGI("", "%zu %d %d", res_len, uart_frame.hdr, uart_frame.data[0]);
		}
			break;
		case EVENT_NEC_RX_DATA:
		{
			ir_nec_scan_code_t scan_code;
			size_t res_len = ir_nec_read(&ir_nec, &scan_code);
			//ESP_LOGI("DUPA Z NEC", "%zu %d %d", res_len, scan_code.address, scan_code.command);
		}
			break;
		default:
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
	ESP_ERROR_CHECK(uart_rx_run(&uart_m, uart_q_in));
	ESP_ERROR_CHECK(uart_tx_run(&uart_m));
	
	
	ir_nec_config_t ir_nec_config = {
		.tx_carrier_config = {
			.duty_cycle = 0.33, // 33 %
			.frequency_hz = 38000 // 38kHz
		},
		.resolution_hz = IR_NEC_RESOLUTION_HZ,
		.rmt_rx_queue_size = 64,
		.rx_gpio = GPIO_NUM_16,
		.tx_gpio = GPIO_NUM_17,
		.rx_interm_buf_sz = sizeof(ir_nec_scan_code_t)*64,
		.tx_interm_buf_sz = sizeof(ir_nec_scan_code_t)*64,
		.invert_in = 1
	};
	ir_nec_init(&ir_nec, &ir_nec_config);
	
	task_arg_t *ir_nec_task_arg = malloc(sizeof(task_arg_t));
	ir_nec_task_arg->dev_ptr = &ir_nec;
	ir_nec_task_arg->event_queue = uart_q_in;

	ir_nec_tx_run(ir_nec_task_arg);
	
	event_t event;
	int c = 0;
	
    while (true) {
		if (xQueueReceive(uart_q_in, (void*)&event, 10)) {
			ESP_LOGI("SOME", "I GOT SOME DATA FROM UART: %d %d", event.event_type, event.payload_len);
			parse_event(&event);
		}
		if (c % 3 == 0) {
			/*uart_frame_t frame;
			frame.hdr = 69;
			ESP_LOGI("GGG", "UART TX EVENT");
			uart_write(&uart_m, &frame);*/
			ir_nec_scan_code_t scan;
			scan.address = 07;
			scan.command = 01;
			ir_nec_write(&ir_nec, &scan);
		}
		vTaskDelay(pdMS_TO_TICKS(100));
		c++;
    }
}

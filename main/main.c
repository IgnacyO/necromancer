/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileCopyrightText: 2026 Ignacy Oziero
 * SPDX-License-Identifier: Unlicense
 */
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "esp_err.h"
#include "esp_check.h"
#include "events.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "hal/uart_types.h"
#include "driver/gpio.h"
#include "portmacro.h"
#include "uart.h"

#define IR_RX_GPIO GPIO_NUM_27
#define IR_TX_GPIO GPIO_NUM_26
#define IR_NEC_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us
#define TAG "Example"

uart_m_t uart_m;

/*void ir_nec_init(rmt_channel_handle_t *rmt_rx_chan) {
	static const rmt_rx_channel_config_t rmt_rx_chan_config = {
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.gpio_num = IR_RX_GPIO,
		.resolution_hz = IR_NEC_RESOLUTION_HZ,
		.mem_block_symbols = 64,
		.flags.invert_in = 1
	};
	ESP_ERROR_CHECK(rmt_new_rx_channel(&rmt_rx_chan_config, rmt_rx_chan));
}

static bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *event_data, void *user_data) {
	// BaseType_t is the most efficient data type for the architecture
	BaseType_t high_task_wakeup = pdFALSE;
	QueueHandle_t rx_queue = (QueueHandle_t)user_data;
	xQueueSendFromISR(rx_queue, event_data, &high_task_wakeup);
	return high_task_wakeup == pdTRUE;
}

void parse_received_symbols_to_nec(rmt_symbol_word_t *rmt_symbols, size_t num_symbols) {
	 printf("NEC frame start---\r\n");
    for (size_t i = 0; i < num_symbols; i++) {
        printf("{%d:%d},{%d:%d}\r\n", rmt_symbols[i].level0, rmt_symbols[i].duration0,
               rmt_symbols[i].level1, rmt_symbols[i].duration1);
    }
    printf("---NEC frame end: ");
	uint16_t nec_addr = 0;
	uint16_t nec_cmd = 0;
    // decode RMT symbols
    switch (num_symbols) {
    case 34: // NEC normal frame
        if (nec_parse_frame(rmt_symbols, &nec_addr, &nec_cmd)) {
            printf("Address=%04X, Command=%04X\r\n\r\n", nec_addr, nec_cmd);
        }
        break;
    case 2: // NEC repeat frame
        if (nec_parse_frame_repeat(rmt_symbols)) {
            printf("Repeat\n");
        }
        break;
    default:
        printf("Unknown NEC frame\r\n\r\n");
        break;
    }
}
*/

static inline esp_err_t parse_event(event_t *event) {
	
	switch (event->event_type) {
		case EVENT_UART_DATA:
		{
			uart_frame_t uart_frame;
			size_t res_len = uart_read(&uart_m, &uart_frame);
			ESP_LOGI("DUPA", "%zu %d %d", res_len, uart_frame.hdr, uart_frame.data[0]);
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
	
	event_t event;
	int c = 0;
	
    while (true) {
		if (xQueueReceive(uart_q_in, (void*)&event, 10)) {
			ESP_LOGI("SOME", "I GOT SOME DATA FROM UART: %d %d", event.event_type, event.payload_len);
			parse_event(&event);
		}
		if (c % 3 == 0) {
			uart_frame_t frame;
			frame.hdr = 69;
			ESP_LOGI("GGG", "UART TX EVENT");
			uart_write(&uart_m, &frame);
		}
		vTaskDelay(pdMS_TO_TICKS(100));
		c++;
        /*if (xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(1000)) == pdPASS) {
            // parse the receive symbols and print the result
        	parse_received_symbols_to_nec(rx_data.received_symbols, rx_data.num_symbols);
            // start receive again
            ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
        } else {
			const ir_nec_scan_code_t scan_code = {
				.address = 0x0440,
				.command = 0x3003
			};
			ESP_ERROR_CHECK(rmt_transmit(tx_channel, nec_encoder, &scan_code, sizeof(scan_code), &transmit_config));
		}*/
    }
}

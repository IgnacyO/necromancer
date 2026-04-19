/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileCopyrightText: 2026 Ignacy Oziero
 * SPDX-License-Identifier: Unlicense
 */
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "driver/rmt_rx.h"
#include "driver/rmt_types.h"
#include "esp_err.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "ir_nec_parser.h"
#include "driver/gpio.h"

#define IR_RX_GPIO GPIO_NUM_27
#define IR_NEC_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us


void ir_nec_init(rmt_channel_handle_t *rmt_rx_chan) {
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


void app_main(void)
{
	rmt_channel_handle_t rx_channel;
	ir_nec_init(&rx_channel);
	ESP_ERROR_CHECK(rmt_enable(rx_channel));
	
	rmt_symbol_word_t raw_symbols[64];
	rmt_rx_done_event_data_t rx_data;
	QueueHandle_t receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(receive_queue);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, receive_queue));
	
	rmt_receive_config_t receive_config = {
	    .signal_range_min_ns = 1250,     // the shortest duration for NEC signal is 560us, 1250ns < 560us, valid signal won't be treated as noise
	    .signal_range_max_ns = 12000000, // the longest duration for NEC signal is 9000us, 12000000ns > 9000us, the receive won't stop early
	};
	
	ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
	
	
    while (true) {
        if (xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(1000)) == pdPASS) {
            // parse the receive symbols and print the result
        	parse_received_symbols_to_nec(rx_data.received_symbols, rx_data.num_symbols);
            // start receive again
            ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
        } 
    }
}

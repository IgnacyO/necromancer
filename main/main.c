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
#include "hal/uart_types.h"
#include "ir_nec_encoder.h"
#include "ir_nec_parser.h"
#include "driver/gpio.h"
#include "uart.h"

#define IR_RX_GPIO GPIO_NUM_27
#define IR_TX_GPIO GPIO_NUM_26
#define IR_NEC_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us
#define TAG "Example"

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
	
	 rmt_tx_channel_config_t tx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = IR_NEC_RESOLUTION_HZ,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
        .trans_queue_depth = 4,  // number of transactions that allowed to pending in the background, this example won't queue multiple transactions, so queue depth > 1 is sufficient
        .gpio_num = IR_TX_GPIO,
    };
    rmt_channel_handle_t tx_channel = NULL;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &tx_channel));

    rmt_carrier_config_t carrier_cfg = {
        .duty_cycle = 0.33,
        .frequency_hz = 38000, // 38KHz
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(tx_channel, &carrier_cfg));

    // this example won't send NEC frames in a loop
    rmt_transmit_config_t transmit_config = {
        .loop_count = 0, // no loop
    };

    ESP_LOGI(TAG, "install IR NEC encoder");
    ir_nec_encoder_config_t nec_encoder_cfg = {
        .resolution = IR_NEC_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t nec_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_ir_nec_encoder(&nec_encoder_cfg, &nec_encoder));

    ESP_LOGI(TAG, "enable RMT TX and RX channels");
    //ESP_ERROR_CHECK(rmt_enable(tx_channel));
    //ESP_ERROR_CHECK(rmt_enable(rx_channel));
	
	//ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
	
	uart_m_config_t uart_m_config = {
		.uart_event_queue_size = 20,
		.event_addr = 16,
		.rx_buffer_size = 1024,
		.tx_buffer_size = 1024,
		.baud_rate = 115200,
		.uart_port = UART_NUM_0
	};
	vTaskDelay(pdMS_TO_TICKS(1000*5));

	QueueHandle_t uart_q_in = xQueueCreate(128, 1);
	uart_m_t uart_m;
	ESP_ERROR_CHECK(uart_init(&uart_m, &uart_m_config, uart_q_in, NULL, NULL));
	ESP_ERROR_CHECK(uart_run_rx(&uart_m));
	
    while (true) {
		vTaskDelay(pdMS_TO_TICKS(100));
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

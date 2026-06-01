#include "ir_nec.h"
#include <string.h>
#include "driver/rmt_common.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_types.h"
#include "esp_log.h"
#include "freertos/projdefs.h"
#include "freertos/ringbuf.h"
#include "freertos/queue.h"
#include "hal/rmt_types.h"
#include "ir_nec_encoder.h"
#include "ir_nec_parser.h"
#include "events.h"

#define RX_SYMBOL_NUM 64
#define RMT_BLOCK_SYMBOLS_NUM 128

static const char* IR_NEC_TAG = "ir_nec";

static const rmt_receive_config_t receive_config = {
    .signal_range_min_ns = IR_NEC_SIG_RANGE_MIN_NS,     
    .signal_range_max_ns = IR_NEC_SIG_RANGE_MAX_NS,
};

static const rmt_transmit_config_t transmit_config = {
    .loop_count = 0,
};

static void ir_nec_tx_task(void *pvParameters) {
	task_arg_t *arg = pvParameters; 
	ir_nec_t *ir_nec = arg->dev_ptr;
	size_t received_frame_len;
	ir_nec_scan_code_t *frame_to_be_sent;
	for (;;) {
		if ((frame_to_be_sent = (ir_nec_scan_code_t*)xRingbufferReceive(ir_nec->tx_interm_buf, &received_frame_len, 10)) != NULL) {
			ESP_LOGI(IR_NEC_TAG, "GOT SOMETHING TX %d %d\n", frame_to_be_sent->address, frame_to_be_sent->command);
			rmt_transmit(ir_nec->rmt_tx_chan, ir_nec->nec_encoder, frame_to_be_sent, sizeof(ir_nec_scan_code_t), &transmit_config);
			vRingbufferReturnItem(ir_nec->tx_interm_buf, frame_to_be_sent);
		}
		else {
			vTaskDelay(pdMS_TO_TICKS(10));
		}
	}
	vTaskDelete(NULL);
}

static void ir_nec_rx_task(void *pvParameters) {
    task_arg_t *arg = pvParameters; 
	ir_nec_t *ir_nec = arg->dev_ptr;

    rmt_rx_done_event_data_t evt_data;

    static rmt_symbol_word_t rx_symbols[RX_SYMBOL_NUM];

    ESP_ERROR_CHECK(
        rmt_receive(
            ir_nec->rmt_rx_chan,
            rx_symbols,
            sizeof(rx_symbols),
            &receive_config
        )
    );

    for (;;) {

        if (xQueueReceive(
                ir_nec->rmt_rx_queue,
                &evt_data,
                portMAX_DELAY
            ) == pdTRUE) {

            ir_nec_scan_code_t scan_code = {0, 0};
			
            if (parse_received_symbols_to_nec(evt_data.received_symbols, evt_data.num_symbols, &scan_code)) {
               ESP_LOGI(IR_NEC_TAG, "NEC: %d %d", scan_code.address, scan_code.command);
            }

            ESP_ERROR_CHECK(
                rmt_receive(
                    ir_nec->rmt_rx_chan,
                    rx_symbols,
                    sizeof(rx_symbols),
                    &receive_config
                )
            );
        }
        else {
			vTaskDelay(pdMS_TO_TICKS(10));
		}
    }
    vTaskDelete(NULL);
}

static bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *event_data, void *user_data) {
	BaseType_t high_task_wakeup = pdFALSE;
	QueueHandle_t rx_queue = *(QueueHandle_t*)user_data;
	xQueueSendFromISR(rx_queue, event_data, &high_task_wakeup);
	return high_task_wakeup == pdTRUE;
}

esp_err_t ir_nec_init(ir_nec_t *ir_nec, const ir_nec_config_t *config) {
    ir_nec->initialized = true;
    ir_nec->rx_running = false;
    ir_nec->tx_running = false;
    ir_nec->rx_interm_buf = xRingbufferCreate(config->rx_interm_buf_sz, RINGBUF_TYPE_NOSPLIT);
    ir_nec->tx_interm_buf = xRingbufferCreate(config->tx_interm_buf_sz, RINGBUF_TYPE_NOSPLIT);
    ir_nec->rmt_rx_queue = xQueueCreate(config->rmt_rx_queue_size, sizeof(rmt_rx_done_event_data_t));

	rmt_rx_channel_config_t rmt_rx_chan_config = {
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.gpio_num = config->rx_gpio,
		.resolution_hz = config->resolution_hz,
		.mem_block_symbols = RMT_BLOCK_SYMBOLS_NUM,
		.flags.invert_in = config->invert_in
	};
	ESP_ERROR_CHECK(rmt_new_rx_channel(&rmt_rx_chan_config, &ir_nec->rmt_rx_chan));
    
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(ir_nec->rmt_rx_chan, &cbs, &ir_nec->rmt_rx_queue));
	
	rmt_tx_channel_config_t tx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = config->resolution_hz,
        .mem_block_symbols = RMT_BLOCK_SYMBOLS_NUM,
        .trans_queue_depth = 4,
        .gpio_num = config->tx_gpio,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &ir_nec->rmt_tx_chan));
    ESP_ERROR_CHECK(rmt_apply_carrier(ir_nec->rmt_tx_chan, &config->tx_carrier_config));

    ir_nec_encoder_config_t nec_encoder_cfg = {
        .resolution = config->resolution_hz,
    };
    ESP_ERROR_CHECK(rmt_new_ir_nec_encoder(&nec_encoder_cfg, &ir_nec->nec_encoder));
    ESP_ERROR_CHECK(rmt_enable(ir_nec->rmt_rx_chan));
    ESP_LOGI(IR_NEC_TAG, "ENABLED RX CHAN");
    ESP_ERROR_CHECK(rmt_enable(ir_nec->rmt_tx_chan));
    
	return ESP_OK;
}

esp_err_t ir_nec_tx_run(task_arg_t *task_arg) {
	ir_nec_t *ir_nec = task_arg->dev_ptr;
	if (ir_nec->tx_running || !ir_nec->initialized) return ESP_ERR_INVALID_STATE;
	xTaskCreate(ir_nec_tx_task, "ir_nec_tx_task", 3072, task_arg, 12, &ir_nec->tx_task_handle);
	ir_nec->tx_running = true;
	return ESP_OK;
}

esp_err_t ir_nec_rx_run(task_arg_t *task_arg) {
	ir_nec_t *ir_nec = task_arg->dev_ptr;
	if (ir_nec->rx_running || !ir_nec->initialized) return ESP_ERR_INVALID_STATE;
	xTaskCreate(ir_nec_rx_task, "ir_nec_rx_task", 3072, task_arg, 12, &ir_nec->rx_task_handle);
	ir_nec->rx_running = true;
	return ESP_OK;
}

esp_err_t ir_nec_stop_tx(ir_nec_t *ir_nec) {
	if (ir_nec->tx_running && ir_nec->initialized) {
		vTaskSuspend(ir_nec->tx_task_handle);
		ir_nec->tx_running = false;
		reset_ring_buffer(ir_nec->tx_interm_buf);
		return ESP_OK;		
	}
	return ESP_ERR_INVALID_STATE;
}

esp_err_t ir_nec_stop_rx(ir_nec_t *ir_nec) {
	if (ir_nec->rx_running && ir_nec->initialized) {
		vTaskSuspend(ir_nec->rx_task_handle);
		ir_nec->rx_running = false;
		reset_ring_buffer(ir_nec->rx_interm_buf);
        xQueueReset(ir_nec->rmt_rx_queue);
		return ESP_OK;
	}
	return ESP_ERR_INVALID_STATE;
}

size_t ir_nec_read(ir_nec_t *ir_nec, ir_nec_scan_code_t* buf) {
	size_t res_size;
	void *res = xRingbufferReceive(ir_nec->rx_interm_buf, &res_size, 10);
	if (res == NULL) {
		ESP_LOGE(IR_NEC_TAG, "RX: Failed to receive data from intermediate buffer");
		return 0;
	}
	memcpy(buf, res, res_size);
	vRingbufferReturnItem(ir_nec->rx_interm_buf, res);
	return res_size;
}

void ir_nec_write(ir_nec_t *ir_nec, ir_nec_scan_code_t* buf) {
	if (xRingbufferSend(ir_nec->tx_interm_buf, buf, sizeof(ir_nec_scan_code_t), 10) == pdFALSE) {
		ESP_LOGE(IR_NEC_TAG, "TX: Failed to send data to intermediate buffer");
		return;
	}
}


esp_err_t ir_nec_deinit(ir_nec_t *ir_nec) {
	vRingbufferDelete(ir_nec->rx_interm_buf);
	vRingbufferDelete(ir_nec->tx_interm_buf);
    vQueueDelete(ir_nec->rmt_rx_queue);
	ESP_ERROR_CHECK(rmt_disable(ir_nec->rmt_rx_chan));
    ESP_ERROR_CHECK(rmt_disable(ir_nec->rmt_tx_chan));
    rmt_del_channel(ir_nec->rmt_rx_chan);
    rmt_del_channel(ir_nec->rmt_tx_chan);
    rmt_del_encoder(ir_nec->nec_encoder);
	ir_nec->rx_running = false;
	ir_nec->tx_running = false;
	ir_nec->initialized = false;
	return ESP_OK;
}
#include "ir_nec.h"
#include "driver/rmt_common.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_types.h"
#include "esp_log.h"
#include "freertos/ringbuf.h"
#include "ir_nec_encoder.h"
#include <string.h>
#define MIN(a, b) ((a) <= (b) ? (a) : (b))

static const char* IR_NEC_TAG = "ir_nec"; /*
static const rmt_receive_config_t receive_config = {
    .signal_range_min_ns = IR_NEC_SIG_RANGE_MIN_NS,     
    .signal_range_max_ns = IR_NEC_SIG_RANGE_MAX_NS,
};
// Dont send NEC frames in a loop
static const rmt_transmit_config_t transmit_config = {
    .loop_count = 0,
};*/

void reset_ring_buffer(RingbufHandle_t buf_handle) {
    if (buf_handle == NULL) {
        return;
    }

    size_t item_size;
    void *item;

    while ((item = xRingbufferReceive(buf_handle, &item_size, 0)) != NULL) {
    	vRingbufferReturnItem(buf_handle, item);
    }
}

static bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *event_data, void *user_data) {
	// BaseType_t is the most efficient data type for the architecture
	BaseType_t high_task_wakeup = pdFALSE;
	RingbufHandle_t rx_buf = *(RingbufHandle_t*)user_data;
	xRingbufferSendFromISR(rx_buf, event_data, sizeof(rmt_rx_done_event_data_t), &high_task_wakeup);
	return high_task_wakeup == pdTRUE;
}

esp_err_t ir_nec_init(ir_nec_m_t *ir_nec_m, const ir_nec_m_config_t *config) {
	rmt_rx_channel_config_t rmt_rx_chan_config = {
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.gpio_num = config->rx_gpio,
		.resolution_hz = config->resolution_hz,
		.mem_block_symbols = 64,
		.flags.invert_in = 1
	};
	ESP_ERROR_CHECK(rmt_new_rx_channel(&rmt_rx_chan_config, &ir_nec_m->rmt_rx_chan));
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(ir_nec_m->rmt_rx_chan, &cbs, &ir_nec_m->rx_interm_buf));
	
	 rmt_tx_channel_config_t tx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = config->resolution_hz,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,  // number of transactions that allowed to pending in the background, this example won't queue multiple transactions, so queue depth > 1 is sufficient
        .gpio_num = config->tx_gpio,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &ir_nec_m->rmt_tx_chan));

    /*rmt_carrier_config_t carrier_cfg = {
        .duty_cycle = 0.33,
        .frequency_hz = 38000, // 38KHz
    };*/
    ESP_ERROR_CHECK(rmt_apply_carrier(ir_nec_m->rmt_tx_chan, &config->tx_carrier_config));


    ir_nec_encoder_config_t nec_encoder_cfg = {
        .resolution = config->resolution_hz,
    };
    ESP_ERROR_CHECK(rmt_new_ir_nec_encoder(&nec_encoder_cfg, &ir_nec_m->nec_encoder));
    ESP_ERROR_CHECK(rmt_enable(ir_nec_m->rmt_rx_chan));
    ESP_ERROR_CHECK(rmt_enable(ir_nec_m->rmt_tx_chan));
    
    ir_nec_m->initialized = true;
    ir_nec_m->rx_running = false;
    ir_nec_m->tx_running = false;
    ir_nec_m->rx_interm_buf = xRingbufferCreate(config->rx_interm_buf_sz, RINGBUF_TYPE_NOSPLIT);
    ir_nec_m->tx_interm_buf = xRingbufferCreate(config->tx_interm_buf_sz, RINGBUF_TYPE_NOSPLIT);
    
	return ESP_OK;
}

size_t ir_nec_read(ir_nec_m_t *ir_nec_m, ir_nec_scan_code_t* buf) {
	size_t res_size;
	void *res = xRingbufferReceive(ir_nec_m->rx_interm_buf, &res_size, 10);
	if (res == NULL) {
		ESP_LOGE(IR_NEC_TAG, "RX: Failed to receive data from intermediate buffer");
		return 0;
	}
	memcpy(buf, res, res_size);
	vRingbufferReturnItem(ir_nec_m->rx_interm_buf, res);
	return res_size;
}

void ir_nec_write(ir_nec_m_t *ir_nec_m, ir_nec_scan_code_t* buf) {
	if (xRingbufferSend(ir_nec_m->tx_interm_buf, buf, sizeof(ir_nec_scan_code_t), 10) == pdFALSE) {
		ESP_LOGE(IR_NEC_TAG, "TX: Failed to send data to intermediate buffer");
		return;
	}
}

esp_err_t ir_nec_deinit(ir_nec_m_t *ir_nec_m) {
/*	reset_ring_buffer(ir_nec_m->rx_interm_buf);
	reset_ring_buffer(ir_nec_m->tx_interm_buf);*/
	vRingbufferDelete(ir_nec_m->rx_interm_buf);
	vRingbufferDelete(ir_nec_m->tx_interm_buf);
	ESP_ERROR_CHECK(rmt_disable(ir_nec_m->rmt_rx_chan));
    ESP_ERROR_CHECK(rmt_disable(ir_nec_m->rmt_tx_chan));
    rmt_del_channel(ir_nec_m->rmt_rx_chan);
    rmt_del_channel(ir_nec_m->rmt_tx_chan);
    rmt_del_encoder(ir_nec_m->nec_encoder);
	ir_nec_m->rx_running = false;
	ir_nec_m->tx_running = false;
	ir_nec_m->initialized = false;
	return ESP_OK;
}
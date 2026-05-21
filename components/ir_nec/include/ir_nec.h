#ifndef COMPONENTS_IR_NEC_INCLUDE_IR_NEC_H_
#define COMPONENTS_IR_NEC_INCLUDE_IR_NEC_H_

#include "driver/rmt_common.h"
#include "esp_err.h"
#include "ir_nec_encoder.h"
#include "ir_nec_parser.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_types.h"
#include "freertos/ringbuf.h"
#include "soc/gpio_num.h"

// the shortest duration for NEC signal is 560us, 1250ns < 560us, valid signal won't be treated as noise
#define IR_NEC_SIG_RANGE_MIN_NS 1250 
// the longest duration for NEC signal is 9000us, 12000000ns > 9000us, the receive won't stop early
#define IR_NEC_SIG_RANGE_MAX_NS 1200000

typedef struct {
	size_t rmt_rx_queue_size;
	uint16_t rx_interm_buf_sz;
	uint16_t tx_interm_buf_sz;
	gpio_num_t rx_gpio;
	gpio_num_t tx_gpio;
	size_t resolution_hz;
	rmt_carrier_config_t tx_carrier_config;
} ir_nec_m_config_t;

typedef struct {
	rmt_channel_handle_t rmt_rx_chan;
	rmt_channel_handle_t rmt_tx_chan;
	rmt_encoder_handle_t nec_encoder;
	bool initialized;
	bool rx_running;
	bool tx_running;
	TaskHandle_t rx_task_handle;
	TaskHandle_t tx_task_handle;
	RingbufHandle_t rx_interm_buf;
	RingbufHandle_t tx_interm_buf;
	QueueHandle_t rmt_rx_queue;
} ir_nec_m_t;

esp_err_t ir_nec_init(ir_nec_m_t *ir_nec_m, const ir_nec_m_config_t *config);
esp_err_t ir_nec_tx_run(ir_nec_m_t *ir_nec_m);
esp_err_t ir_nec_rx_run(ir_nec_m_t *ir_nec_m);
esp_err_t ir_nec_tx_stop(ir_nec_m_t *ir_nec_m);
esp_err_t ir_nec_rx_stop(ir_nec_m_t *ir_nec_m);
esp_err_t ir_nec_tx_notify(ir_nec_m_t *ir_nec_m);
size_t ir_nec_read(ir_nec_m_t *ir_nec_m, ir_nec_scan_code_t* buf);
void ir_nec_write(ir_nec_m_t *ir_nec_m, ir_nec_scan_code_t* buf);
esp_err_t ir_nec_deinit(ir_nec_m_t *ir_nec_m);

#endif /* COMPONENTS_IR_NEC_INCLUDE_IR_NEC_H_ */
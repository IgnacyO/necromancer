#ifndef COMPONENT_IR_NEC_H
#define COMPONENT_IR_NEC_H

#include "driver/rmt_common.h"
#include "esp_err.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_types.h"
#include "freertos/ringbuf.h"
#include "soc/gpio_num.h"
#include "common.h"
#include "ir_nec_encoder.h"
#include "ir_nec_parser.h"

// the shortest duration for NEC signal is 560us, 1250ns < 560us, valid signal won't be treated as noise
#define IR_NEC_SIG_RANGE_MIN_NS 1000 
// the longest duration for NEC signal is 9000us, 12000000ns > 9000us, the receive won't stop early
#define IR_NEC_SIG_RANGE_MAX_NS 25000000

//@brief IR NEC device configuration struct for both rx and tx
typedef struct {
	size_t rmt_rx_queue_size;
	uint16_t rx_interm_buf_sz;
	uint16_t tx_interm_buf_sz;
	gpio_num_t rx_gpio;
	gpio_num_t tx_gpio;
	size_t resolution_hz;
	rmt_carrier_config_t tx_carrier_config;
	bool invert_in; ///< whether to invert input ir signal
	bool error_correction;
} ir_nec_config_t;

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
	task_arg_t tx_task_arg;
	task_arg_t rx_task_arg;
	bool data_lsb_format;
	bool error_correction;
} ir_nec_t;

esp_err_t ir_nec_init(ir_nec_t *ir_nec, const ir_nec_config_t *config);
esp_err_t ir_nec_tx_run(task_arg_t *task_arg);
esp_err_t ir_nec_rx_run(task_arg_t *task_arg);
esp_err_t ir_nec_tx_stop(ir_nec_t *ir_nec);
esp_err_t ir_nec_rx_stop(ir_nec_t *ir_nec);
size_t ir_nec_read(ir_nec_t *ir_nec, nec_scan_code_t* buf);
void ir_nec_write(ir_nec_t *ir_nec, nec_scan_code_t* buf);
esp_err_t ir_nec_deinit(ir_nec_t *ir_nec);

#endif /* COMPONENT_IR_NEC_H */
#ifndef COMPONENT_COMMON_H
#define COMPONENT_COMMON_H
#include "freertos/ringbuf.h"
#include "freertos/queue.h"
#include <stdint.h>

///@brief device task argument type
typedef struct {
	void *dev_ptr; /*< ptr to device struct */
	QueueHandle_t event_queue; /*< event queue handler */
} task_arg_t;

typedef enum : uint8_t {
	CMD_IR_NEC_TX_DATA,
	CMD_IR_NEC_TX_START,
	CMD_IR_NEC_TX_STOP,
	CMD_IR_NEC_RX_START,
	CMD_IR_NEC_RX_STOP,
	CMD_IR_NEC_RX_DATA,
	CMD_CFG_SET,
	CMD_CFG_UNSET
} uart_hdr_t;

///@brief Empties idf ring buffer
void reset_ring_buffer(RingbufHandle_t buf_handle);
#endif 
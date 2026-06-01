#include "freertos/ringbuf.h"
#include "freertos/queue.h"

///@brief device task argument type
typedef struct {
	void *dev_ptr; /*< ptr to device struct */
	QueueHandle_t event_queue; /*< event queue handler */
} task_arg_t;

///@brief Empties idf ring buffer
void reset_ring_buffer(RingbufHandle_t buf_handle);
#include "common.h"

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
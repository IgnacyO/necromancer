#ifndef COMPONENT_EVENTS_H
#define COMPONENT_EVENTS_H
#include <stdint.h>

typedef enum event_type_t {
	EVENT_UART_RX_DATA,
	EVENT_NEC_RX_DATA
} event_type_t;

typedef struct event_t {
	uint16_t event_type;
	uint16_t payload_len;
} event_t;
#endif
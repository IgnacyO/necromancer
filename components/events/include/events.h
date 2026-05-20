#ifndef COMPONENT_EVENTS_H
#define COMPONENT_EVENTS_H
#include <stdint.h>

typedef enum event_type_t {
	EVENT_UART_DATA,
	EVENT_UART_INV_STATE,
	EVENT_UART_RST_STATE,
	EVENT_NEC_TX,
	EVENT_NEC_RX
} event_type_t;

typedef struct event_t {
	uint16_t event_type;
	uint16_t payload_len;
} event_t;
#endif
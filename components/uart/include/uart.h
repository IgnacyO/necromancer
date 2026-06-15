#ifndef COMPONENT_UART_H
#define COMPONENT_UART_H
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "soc/uart_struct.h"
#include <stdint.h>
#include "events.h"
#include "common.h"

typedef struct uart_frame_t {
	uint8_t magic;
	uint8_t hdr;
	uint8_t crc;
	uint8_t data[13];
} uart_frame_t;

typedef struct uart_m_config_t {
	uart_port_t uart_port;
	uint16_t event_addr;
	uint16_t uart_event_queue_size;
	uint32_t baud_rate;
	uint16_t rx_buffer_size;
	uint16_t tx_buffer_size;
	uint16_t rx_interm_buffer_size;
	uint16_t tx_interm_buffer_size;
	uint16_t tx_event_queue_size;
} uart_m_config_t;

typedef struct uart_m_t {
	uint16_t event_addr;
	bool initialized;
	bool rx_running;
	bool tx_running;
	uart_port_t uart_port;
	QueueHandle_t uart_internal_queue;
	uart_dev_t uart_dev;
	TaskHandle_t rx_task_handle;
	TaskHandle_t tx_task_handle;
	RingbufHandle_t rx_interm_buf;
	RingbufHandle_t tx_interm_buf;
	QueueHandle_t tx_event_queue;
} uart_m_t;

typedef struct uart_m_rx_task_arg_t {
	uart_m_t *uart;
	QueueHandle_t event_queue;
} uart_m_rx_task_arg_t;

esp_err_t uart_init(uart_m_t *uart_m, uart_m_config_t *uart_m_config);
esp_err_t uart_tx_run(task_arg_t *task_arg);
esp_err_t uart_rx_run(task_arg_t *task_arg);
esp_err_t uart_tx_stop(uart_m_t *uart_m);
esp_err_t uart_rx_stop(uart_m_t *uart_m);
esp_err_t uart_tx_notify(uart_m_t *uart_m);
size_t uart_read(uart_m_t *uart_m, uart_frame_t *buffer);
size_t uart_write(uart_m_t *uart_m, uart_frame_t *buffer);
esp_err_t uart_deinit(uart_m_t *uart_m);

#endif
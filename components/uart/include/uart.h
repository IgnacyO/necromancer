#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "soc/uart_struct.h"
#include <stdint.h>
#include "events.h"

typedef struct uart_frame_t {
	uint8_t hdr;
	uint8_t crc;
	uint8_t data[14];
} uart_frame_t;

typedef struct uart_m_config_t {
	uart_port_t uart_port;
	uint16_t event_addr;
	uint16_t uart_event_queue_size;
	uint32_t baud_rate;
	uint16_t rx_buffer_size;
	uint16_t tx_buffer_size;
} uart_m_config_t;

typedef struct uart_m_t {
	uint16_t event_addr;
	bool initialized;
	bool rx_running;
	bool tx_running;
	uart_port_t uart_port;
	QueueHandle_t event_queue_in;
	QueueHandle_t uart_event_queue;
	uart_dev_t uart_dev;
	TaskHandle_t rx_task_handle;
	TaskHandle_t tx_task_handle;
	void *event_queue_out_impl;
	esp_err_t (*event_send_impl)(void *, event_t);
} uart_m_t;

static const char *UART_TAG = "uart_module"; 

esp_err_t uart_init(uart_m_t *uart_m, uart_m_config_t *uart_m_config, QueueHandle_t event_queue_in, void *event_queue_out_impl, esp_err_t (*event_send_impl)(void *, event_t));
esp_err_t uart_run_tx(uart_m_t *uart_m);
esp_err_t uart_run_rx(uart_m_t *uart_m);
esp_err_t uart_stop_tx(uart_m_t *uart_m);
esp_err_t uart_stop_rx(uart_m_t *uart_m);
esp_err_t uart_read(uart_m_t *uart_m, char *buffer, size_t size, size_t *len);
esp_err_t uart_write(uart_m_t *uart_m, char *buffer, size_t size, size_t *len);
esp_err_t uart_deinit(uart_m_t *uart_m);
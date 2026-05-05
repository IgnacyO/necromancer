#include "uart.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdint.h>

static void uart_rx_task(void *pvParameters) {
	uart_m_t *uart_m_ptr = pvParameters;
    uart_event_t uart_event;
    event_t event;
    
    char buffer[sizeof(uart_frame_t)];

    for (;;) {
		/*if (uart_m_ptr->event_queue_out_impl == NULL) {
			vTaskDelay(pdMS_TO_TICKS(10));
			continue;
		}*/
        if (xQueueReceive(uart_m_ptr->uart_event_queue, (void *)&uart_event, (TickType_t)portMAX_DELAY)) {
            switch (uart_event.type) {
            case UART_DATA:
                uart_read_bytes(uart_m_ptr->uart_port,buffer, sizeof(uart_frame_t), portMAX_DELAY);
                ESP_LOGI(UART_TAG, "UART: %c %c %c\n", buffer[0], buffer[1], buffer[2]);
                event.event_type = EVENT_UART;
                event.payload_len = sizeof(uart_frame_t);
                //UBaseType_t res =  xRingbufferSend(uart_m_ptr->event_ring_buffer, &event, sizeof(event_t),(TickType_t)portMAX_DELAY);
			    //if (res != pdTRUE) {
			    //    ESP_LOGE(UART_TAG, "could not send event to dispatcher's ring buffer");
			    //}
			    if (uart_m_ptr->event_send_impl != NULL) {
					ESP_LOGI(UART_TAG, "SEND IMPL IS NOT NULL");
					uart_m_ptr->event_send_impl(uart_m_ptr->event_queue_out_impl, event);
				}
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGW(UART_TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(uart_m_ptr->uart_port);
                xQueueReset(uart_m_ptr->uart_event_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGW(UART_TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(uart_m_ptr->uart_port);
                xQueueReset(uart_m_ptr->uart_event_queue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGW(UART_TAG, "uart rx break");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGE(UART_TAG, "uart parity error");
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGE(UART_TAG, "uart frame error");
                break;
            //Others
            default:
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

esp_err_t uart_init(uart_m_t *uart_m, uart_m_config_t *uart_m_config, QueueHandle_t event_queue_in, void *event_queue_out_impl, esp_err_t (*event_send_impl)(void *, event_t)) {
	esp_err_t ret;
	
	assert(uart_m != NULL && uart_m_config);
	
	// Install driver if not already
	if (!uart_is_driver_installed(uart_m_config->uart_port)) {
		ESP_GOTO_ON_ERROR(uart_driver_install(uart_m_config->uart_port, uart_m_config->rx_buffer_size, uart_m_config->tx_buffer_size, uart_m_config->uart_event_queue_size, &uart_m->uart_event_queue, 0), err, UART_TAG, "installing uart driver failed.");
	}
	
	uart_config_t uart_config = {
	    .baud_rate = uart_m_config->baud_rate,
	    .data_bits = UART_DATA_8_BITS,
	    .parity = UART_PARITY_DISABLE,
	    .stop_bits = UART_STOP_BITS_1,
	    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
	
	ESP_GOTO_ON_ERROR(uart_param_config(uart_m_config->uart_port, &uart_config), err, UART_TAG, "configuring uart failed.");
	uart_m->event_queue_in = event_queue_in;
	uart_m->event_addr = uart_m_config->event_addr;
	uart_m->initialized = true;
	uart_m->tx_running = false;
	uart_m->rx_running = false;
	uart_m->uart_port = uart_m_config->uart_port;
	uart_m->event_send_impl = event_send_impl;
	uart_m->event_queue_out_impl = event_queue_out_impl;
	
	
	switch(uart_m->uart_port) {
		case UART_NUM_0:
			uart_m->uart_dev = UART0;
			break;
		case UART_NUM_1:
			uart_m->uart_dev = UART1;
			break;
		case UART_NUM_2:
			uart_m->uart_dev = UART2;
			break;
		default:
			ESP_LOGI(UART_TAG, "invalid uart port: %d", uart_m_config->uart_port);
			return ESP_ERR_INVALID_ARG;
	}
	
	return ESP_OK;

	err:
		return ESP_FAIL;
}

esp_err_t uart_run_tx(uart_m_t *uart_m) {
	if (uart_m->tx_running || !uart_m->initialized) return ESP_ERR_INVALID_STATE;
	// run tx task
	return ESP_OK;
}

esp_err_t uart_run_rx(uart_m_t *uart_m) {
	if (uart_m->tx_running || !uart_m->initialized) return ESP_ERR_INVALID_STATE;
	xTaskCreate(uart_rx_task, "uart_rx_task", 3072, uart_m, 12, &uart_m->rx_task_handle);
	uart_m->rx_running = true;
	return ESP_OK;
}

esp_err_t uart_stop_tx(uart_m_t *uart_m) {
	if (uart_m->tx_running && uart_m->initialized); // suspend tx
	return ESP_ERR_INVALID_STATE;
}

esp_err_t uart_stop_rx(uart_m_t *uart_m) {
	if (uart_m->tx_running && uart_m->initialized) {
		vTaskSuspend(uart_m->rx_task_handle);
		uart_m->rx_running = false;
		return ESP_OK;
	}
	return ESP_ERR_INVALID_STATE;
}

esp_err_t uart_read(uart_m_t *uart_m, char *buffer, size_t size, size_t *len) {
	return ESP_OK;
}

esp_err_t uart_write(uart_m_t *uart_m, char *buffer, size_t size, size_t *len){
	return ESP_OK;
}

esp_err_t uart_deinit(uart_m_t *uart) {
	if (!uart->initialized) return ESP_ERR_INVALID_STATE;
	uart_stop_rx(uart);
	uart_stop_tx(uart);
	esp_err_t ret;
	ESP_ERROR_CHECK(uart_driver_delete(uart->uart_port));
	uart->initialized = false;
	xQueueReset(uart->event_queue_in);
	xQueueReset(uart->uart_event_queue);
	return ESP_OK;
}
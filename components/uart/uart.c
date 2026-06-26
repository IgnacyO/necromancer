#include "uart.h"
#include "common.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "events.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "portmacro.h"
#include <stdint.h>
#include <string.h>

static const char *UART_TAG = "UART";

static void uart_rx_task(void *pvParameters) {
  task_arg_t *arg = pvParameters;
  uart_m_t *uart_m_ptr = arg->dev_ptr;
  QueueHandle_t event_queue = arg->event_queue;

  uart_event_t uart_event;
  event_t event;

  for (;;) {
    if (xQueueReceive(uart_m_ptr->uart_internal_queue, (void *)&uart_event,
                      10)) {
      switch (uart_event.type) {
      case UART_DATA: {
        static uint8_t rx_accum[128];
        static size_t rx_len = 0;

        uint8_t tmp[64]; // temporary read buffer

        int len = uart_event.size;
        int read =
            uart_read_bytes(uart_m_ptr->uart_port, tmp, len, portMAX_DELAY);

        // Protect against overflow
        if (rx_len + read > sizeof(rx_accum)) {
          ESP_LOGE(UART_TAG, "RX: Buffer overflow. Resetting...");
          rx_len = 0;
        }

        // Append new data
        memcpy(rx_accum + rx_len, tmp, read);
        rx_len += read;

        size_t offset = 0;

        while (rx_len - offset >= sizeof(uart_frame_t)) {

          uart_frame_t *frame = (uart_frame_t *)(rx_accum + offset);

          // Sync check (magic byte)
          if (frame->magic != 0xAA) {
            if (g_debug_enabled)
              ESP_LOGW(UART_TAG, "Desync (0x%02X), skipping byte",
                       frame->magic);
            offset++; // shift by one byte to resync
            continue;
          }

          if (g_debug_enabled)
            ESP_LOGI(UART_TAG, "Parsed frame cmd: %d", frame->hdr);

          // Push event
          event.event_type = EVENT_UART_RX_DATA;
          event.payload_len = sizeof(uart_frame_t);

          if (xQueueSend(event_queue, &event, 10) != pdPASS) {
            ESP_LOGE(UART_TAG, "Rx: Failed to push event");
            continue;
          }

          if (xRingbufferSend(uart_m_ptr->rx_interm_buf, frame,
                              sizeof(uart_frame_t), 10) != pdPASS) {
            ESP_LOGE(UART_TAG, "Rx: Failed to push data to interm buf.");
            continue;
          }

          offset += sizeof(uart_frame_t);
        }

        // Move leftover bytes to front
        if (offset > 0) {
          memmove(rx_accum, rx_accum + offset, rx_len - offset);
          rx_len -= offset;
        }

        break;
      }
      // Event of HW FIFO overflow detected
      case UART_FIFO_OVF:
        ESP_LOGW(UART_TAG, "Rx: hw fifo overflow");
        // If fifo overflow happened, you should consider adding flow control
        // for your application. The ISR has already reset the rx FIFO, As an
        // example, we directly flush the rx buffer here in order to read more
        // data.
        uart_flush_input(uart_m_ptr->uart_port);
        xQueueReset(uart_m_ptr->uart_internal_queue);
        break;
      // Event of UART ring buffer full
      case UART_BUFFER_FULL:
        ESP_LOGW(UART_TAG, "Rx: ring buffer full");
        // If buffer full happened, you should consider increasing your buffer
        // size As an example, we directly flush the rx buffer here in order to
        // read more data.
        uart_flush_input(uart_m_ptr->uart_port);
        xQueueReset(uart_m_ptr->uart_internal_queue);
        break;
      // Event of UART RX break detected
      case UART_BREAK:
        ESP_LOGW(UART_TAG, "Rx: uart rx break");
        break;
      // Event of UART parity check error
      case UART_PARITY_ERR:
        ESP_LOGE(UART_TAG, "Rx: uart parity error");
        break;
      // Event of UART frame error
      case UART_FRAME_ERR:
        ESP_LOGE(UART_TAG, "Rx: uart frame error");
        break;
      // Others
      default:
        break;
      }
    }
  }
  vTaskDelete(NULL);
}

static void uart_tx_task(void *pvParameters) {
  task_arg_t *arg = pvParameters;
  uart_m_t *uart_m_ptr = arg->dev_ptr;
  size_t received_frame_len;
  uart_frame_t *frame_to_be_sent;
  for (;;) {
    if ((frame_to_be_sent = (uart_frame_t *)xRingbufferReceive(
             uart_m_ptr->tx_interm_buf, &received_frame_len, 10)) != NULL) {
      uart_write_bytes(uart_m_ptr->uart_port, frame_to_be_sent,
                       received_frame_len);
      vRingbufferReturnItem(uart_m_ptr->tx_interm_buf, frame_to_be_sent);
    }
  }
  vTaskDelete(NULL);
}

esp_err_t uart_init(uart_m_t *uart_m, uart_m_config_t *uart_m_config) {
  ESP_LOGI(UART_TAG, "Initializing module...");
  esp_err_t ret;

  assert(uart_m != NULL && uart_m_config);

  // Install driver if not already
  if (!uart_is_driver_installed(uart_m_config->uart_port)) {
    ESP_GOTO_ON_ERROR(uart_driver_install(uart_m_config->uart_port,
                                          uart_m_config->rx_buffer_size,
                                          uart_m_config->tx_buffer_size,
                                          uart_m_config->uart_event_queue_size,
                                          &uart_m->uart_internal_queue, 0),
                      err, UART_TAG, "installing uart driver failed.");
  }
  ESP_LOGI(UART_TAG, "Done installing uart driver.");

  uart_config_t uart_config = {.baud_rate = uart_m_config->baud_rate,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

  ESP_GOTO_ON_ERROR(uart_param_config(uart_m_config->uart_port, &uart_config),
                    err, UART_TAG, "configuring uart failed.");
  ESP_LOGI(UART_TAG, "Done configuring uart.");

  uart_m->event_addr = uart_m_config->event_addr;
  uart_m->initialized = true;
  uart_m->tx_running = false;
  uart_m->rx_running = false;
  uart_m->uart_port = uart_m_config->uart_port;
  uart_m->tx_event_queue =
      xQueueCreate(uart_m_config->tx_event_queue_size, sizeof(event_t));
  uart_m->rx_interm_buf = xRingbufferCreate(
      uart_m_config->rx_interm_buffer_size, RINGBUF_TYPE_NOSPLIT);
  uart_m->tx_interm_buf = xRingbufferCreate(
      uart_m_config->tx_interm_buffer_size, RINGBUF_TYPE_NOSPLIT);

  switch (uart_m->uart_port) {
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
    ESP_LOGI(UART_TAG, "Invalid uart port: %d", uart_m_config->uart_port);
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(UART_TAG, "Done initializing module.");
  return ESP_OK;

err:
  return ESP_FAIL;
}

esp_err_t uart_tx_run(task_arg_t *task_arg) {
  ESP_LOGI(UART_TAG, "Running tx task...");
  uart_m_t *uart_m = task_arg->dev_ptr;
  if (uart_m->tx_running || !uart_m->initialized)
    return ESP_ERR_INVALID_STATE;
  xTaskCreate(uart_tx_task, "uart_tx_task", 3072, task_arg, 12,
              &uart_m->tx_task_handle);
  uart_m->tx_running = true;
  ESP_LOGI(UART_TAG, "Tx task is running.");
  return ESP_OK;
}

esp_err_t uart_rx_run(task_arg_t *task_arg) {

  ESP_LOGI(UART_TAG, "Running tx task...");

  uart_m_t *uart_m = task_arg->dev_ptr;
  if (uart_m->rx_running || !uart_m->initialized)
    return ESP_ERR_INVALID_STATE;
  xTaskCreate(uart_rx_task, "uart_rx_task", 3072, task_arg, 12,
              &uart_m->rx_task_handle);
  uart_m->rx_running = true;
  ESP_LOGI(UART_TAG, "Rx task is running.");

  return ESP_OK;
}

esp_err_t uart_stop_tx(uart_m_t *uart_m) {
  ESP_LOGI(UART_TAG, "Stopping tx task...");

  if (uart_m->tx_running && uart_m->initialized) {
    vTaskSuspend(uart_m->tx_task_handle);
    uart_m->tx_running = false;
    reset_ring_buffer(uart_m->tx_interm_buf);
    ESP_LOGI(UART_TAG, "Tx task is stopped.");

    return ESP_OK;
  }
  return ESP_ERR_INVALID_STATE;
}

esp_err_t uart_stop_rx(uart_m_t *uart_m) {
  ESP_LOGI(UART_TAG, "Stopping rx task...");

  if (uart_m->rx_running && uart_m->initialized) {
    vTaskSuspend(uart_m->rx_task_handle);
    uart_m->rx_running = false;
    reset_ring_buffer(uart_m->rx_interm_buf);
    ESP_LOGI(UART_TAG, "Rx task is stopped.");

    return ESP_OK;
  }
  return ESP_ERR_INVALID_STATE;
}

size_t uart_read(uart_m_t *uart_m, uart_frame_t *buffer) {
  size_t res_size;
  void *res = xRingbufferReceive(uart_m->rx_interm_buf, &res_size, 10);
  if (res == NULL) {
    ESP_LOGE(UART_TAG, "Rx: failed to receive data from interm buf.");
    return 0;
  }
  memcpy(buffer, res, res_size);
  vRingbufferReturnItem(uart_m->rx_interm_buf, res);
  return res_size;
}

size_t uart_write(uart_m_t *uart_m, uart_frame_t *buffer) {
  if (xRingbufferSend(uart_m->tx_interm_buf, buffer, sizeof(uart_frame_t),
                      10) == pdFALSE) {
    ESP_LOGE(UART_TAG, "Tx: failed to send data to interm buf");
    return 0;
  }
  return sizeof(uart_frame_t);
}

esp_err_t uart_deinit(uart_m_t *uart) {
  ESP_LOGI(UART_TAG, "Deinitializing module...");

  if (!uart->initialized)
    return ESP_ERR_INVALID_STATE;
  uart_stop_rx(uart);
  uart_stop_tx(uart);
  esp_err_t ret;
  ESP_ERROR_CHECK(uart_driver_delete(uart->uart_port));
  uart->initialized = false;
  vRingbufferDelete(uart->rx_interm_buf);
  vRingbufferDelete(uart->tx_interm_buf);
  xQueueReset(uart->tx_event_queue);
  xQueueReset(uart->uart_internal_queue);
  ESP_LOGI(UART_TAG, "Done deinitializing module.");

  return ESP_OK;
}
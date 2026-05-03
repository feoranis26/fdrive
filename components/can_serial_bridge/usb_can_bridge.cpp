#include "usb_can_bridge.h"

#include <ctype.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart_vfs.h"
#include "esp_check.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const int USB_CAN_BRIDGE_LINE_MAX = 128;
static const char *TAG = "usb_can_bridge";

struct usb_can_bridge_t {
    uart_port_t uart_port;
    can_queue_t *can_queue;
    can_queue_handle_t *sniffer_handle;
    QueueHandle_t tx_tap_queue;
    TaskHandle_t input_task;
    TaskHandle_t output_task;
    volatile bool running;
    volatile bool mirroring_enabled;
    bool use_stdio;
};

static void usb_can_bridge_write_line(usb_can_bridge_t *bridge, const char *line)
{
    if (bridge == NULL || line == NULL) {
        return;
    }

    if (bridge->use_stdio) {
        fputs(line, stdout);
        fflush(stdout);
        return;
    }

    (void)uart_write_bytes(bridge->uart_port, line, strlen(line));
}

static int usb_can_bridge_read_byte(usb_can_bridge_t *bridge, uint8_t *out_byte)
{
    if (bridge == NULL || out_byte == NULL) {
        return -1;
    }

    if (bridge->use_stdio) {
        int ch = fgetc(stdin);
        if (ch == EOF) {
            return 0;
        }
        *out_byte = (uint8_t)ch;
        return 1;
    }

    return uart_read_bytes(bridge->uart_port, out_byte, 1U, pdMS_TO_TICKS(20));
}

static void usb_can_bridge_emit_frame_line(usb_can_bridge_t *bridge, const char *direction, const twai_frame_t *frame)
{
    if (bridge == NULL || frame == NULL || !bridge->mirroring_enabled) {
        return;
    }

    char line[USB_CAN_BRIDGE_LINE_MAX] = {0};
    int offset = snprintf(line, sizeof(line), "SEND %s ID:%03" PRIX32, direction, frame->header.id);
    for (uint32_t i = 0; i < frame->buffer_len && offset > 0 && offset < (int)sizeof(line); ++i) {
        offset += snprintf(&line[offset], sizeof(line) - (size_t)offset, " %02X", frame->buffer[i]);
    }
    if (offset > 0 && offset < (int)sizeof(line) - 1) {
        line[offset++] = '\n';
        line[offset] = '\0';
    }
    usb_can_bridge_write_line(bridge, line);
}

static bool usb_can_bridge_parse_hex_byte(const char *token, uint8_t *out_value)
{
    char *end = NULL;
    unsigned long value = strtoul(token, &end, 16);
    if (token == end || value > 0xFFUL) {
        return false;
    }
    while (*end != '\0') {
        if (!isspace((unsigned char)*end)) {
            return false;
        }
        ++end;
    }
    *out_value = (uint8_t)value;
    return true;
}

static bool usb_can_bridge_parse_send_line(const char *line, twai_frame_t *out_frame)
{
    if (line == NULL || out_frame == NULL) {
        return false;
    }

    if (strncmp(line, "SEND ", 5) != 0) {
        return false;
    }

    const char *payload = line + 5;
    if (strcmp(payload, "ENABLE") == 0 || strcmp(payload, "DISABLE") == 0) {
        return false;
    }

    if (strncmp(payload, "ID:", 3) != 0) {
        return false;
    }

    char scratch[USB_CAN_BRIDGE_LINE_MAX] = {0};
    strncpy(scratch, payload, sizeof(scratch) - 1U);

    char *save_ptr = NULL;
    char *token = strtok_r(scratch, " ", &save_ptr);
    if (token == NULL || strncmp(token, "ID:", 3) != 0) {
        return false;
    }

    char *end = NULL;
    unsigned long id = strtoul(token + 3, &end, 16);
    if ((token + 3) == end || id > 0x1FFFFFFFUL) {
        return false;
    }

    uint8_t payload_bytes[8] = {0};
    uint32_t payload_len = 0U;

    while ((token = strtok_r(NULL, " ", &save_ptr)) != NULL) {
        if (payload_len >= 8U) {
            return false;
        }
        if (!usb_can_bridge_parse_hex_byte(token, &payload_bytes[payload_len])) {
            return false;
        }
        payload_len++;
    }

    uint8_t *buffer = NULL;
    if (payload_len > 0U) {
        buffer = (uint8_t *)malloc(payload_len);
        if (buffer == NULL) {
            return false;
        }
        memcpy(buffer, payload_bytes, payload_len);
    }

    out_frame->header.id = (uint32_t)id;
    out_frame->header.ide = (id > TWAI_STD_ID_MASK);
    out_frame->header.rtr = 0;
    out_frame->header.fdf = 0;
    out_frame->header.brs = 0;
    out_frame->header.esi = 0;
    out_frame->header.dlc = payload_len;
    out_frame->buffer = buffer;
    out_frame->buffer_len = payload_len;
    return true;
}

static void usb_can_bridge_input_task(void *arg)
{
    usb_can_bridge_t *bridge = (usb_can_bridge_t *)arg;
    char line[USB_CAN_BRIDGE_LINE_MAX] = {0};
    size_t offset = 0U;

    while (bridge->running) {
        uint8_t byte = 0;
        const int read = usb_can_bridge_read_byte(bridge, &byte);
        if (read <= 0) {
            continue;
        }

        if (byte == '\r') {
            continue;
        }

        if (byte == '\n') {
            line[offset] = '\0';

            if (strcmp(line, "SEND ENABLE") == 0) {
                bridge->mirroring_enabled = true;
                usb_can_bridge_write_line(bridge, "SEND ENABLED\n");
            } else if (strcmp(line, "SEND DISABLE") == 0) {
                bridge->mirroring_enabled = false;
                usb_can_bridge_write_line(bridge, "SEND DISABLED\n");
            } else if (offset > 0U) {
                twai_frame_t frame = {};
                if (usb_can_bridge_parse_send_line(line, &frame)) {
                    bool injected_locally = bridge->can_queue->inject_local_rx(frame, 0);
                    if (!bridge->can_queue->enqueue_tx(frame, 0)) {
                        usb_can_bridge_write_line(bridge, "NAK\n");
                    } else if (!injected_locally) {
                        usb_can_bridge_write_line(bridge, "NAK\n");
                    }
                    free(frame.buffer);
                } else {
                    usb_can_bridge_write_line(bridge, "NAK\n");
                }
            }

            offset = 0U;
            continue;
        }

        if (offset + 1U < sizeof(line)) {
            line[offset++] = (char)byte;
        } else {
            offset = 0U;
            usb_can_bridge_write_line(bridge, "NAK\n");
        }
    }

    vTaskDelete(NULL);
}

static void usb_can_bridge_output_task(void *arg)
{
    usb_can_bridge_t *bridge = (usb_can_bridge_t *)arg;

    while (bridge->running) {
        twai_frame_t frame = {};
        if (bridge->sniffer_handle->dequeue_rx(&frame, pdMS_TO_TICKS(10))) {
            usb_can_bridge_emit_frame_line(bridge, "RX", &frame);
            free(frame.buffer);
        }

        memset(&frame, 0, sizeof(frame));
        if (xQueueReceive(bridge->tx_tap_queue, &frame, 0) == pdTRUE) {
            usb_can_bridge_emit_frame_line(bridge, "TX", &frame);
            free(frame.buffer);
        }
    }

    vTaskDelete(NULL);
}

esp_err_t usb_can_bridge_init(usb_can_bridge_t **out_bridge, const usb_can_bridge_config_t *config)
{
    if (out_bridge == NULL || config == NULL || config->can_queue == NULL || config->sniffer_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    usb_can_bridge_t *bridge = (usb_can_bridge_t *)calloc(1, sizeof(*bridge));
    if (bridge == NULL) {
        return ESP_ERR_NO_MEM;
    }

    bridge->uart_port = config->uart_port;
    bridge->can_queue = config->can_queue;
    bridge->sniffer_handle = config->sniffer_handle;
    bridge->running = true;
    bridge->use_stdio = (config->uart_port == UART_NUM_0);

    bridge->tx_tap_queue = xQueueCreate(32, sizeof(twai_frame_t));
    if (bridge->tx_tap_queue == NULL) {
        free(bridge);
        return ESP_ERR_NO_MEM;
    }

    uart_config_t uart_config = {};
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    if (bridge->use_stdio) {
        ESP_RETURN_ON_ERROR(uart_param_config(bridge->uart_port, &uart_config), TAG, "uart_param_config failed");
        ESP_RETURN_ON_ERROR(uart_driver_install(bridge->uart_port, 2048, 0, 0, NULL, 0), TAG, "uart_driver_install failed");
        uart_vfs_dev_use_driver(bridge->uart_port);
        (void)uart_vfs_dev_port_set_rx_line_endings(bridge->uart_port, ESP_LINE_ENDINGS_CR);
        (void)uart_vfs_dev_port_set_tx_line_endings(bridge->uart_port, ESP_LINE_ENDINGS_CRLF);
        setvbuf(stdin, NULL, _IONBF, 0);
        setvbuf(stdout, NULL, _IONBF, 0);
    } else {
        ESP_RETURN_ON_ERROR(uart_param_config(bridge->uart_port, &uart_config), TAG, "uart_param_config failed");
        ESP_RETURN_ON_ERROR(uart_driver_install(bridge->uart_port, 2048, 0, 0, NULL, 0), TAG, "uart_driver_install failed");
    }

    bridge->can_queue->set_tx_tap_queue(bridge->tx_tap_queue);

    if (xTaskCreate(usb_can_bridge_input_task, "usb_can_in", 4096, bridge, 5, &bridge->input_task) != pdPASS ||
        xTaskCreate(usb_can_bridge_output_task, "usb_can_out", 4096, bridge, 5, &bridge->output_task) != pdPASS) {
        bridge->can_queue->set_tx_tap_queue(NULL);
        if (bridge->tx_tap_queue != NULL) {
            vQueueDelete(bridge->tx_tap_queue);
        }
        free(bridge);
        return ESP_ERR_NO_MEM;
    }

    *out_bridge = bridge;
    return ESP_OK;
}

void usb_can_bridge_deinit(usb_can_bridge_t *bridge)
{
    if (bridge == NULL) {
        return;
    }

    bridge->running = false;
    bridge->can_queue->set_tx_tap_queue(NULL);
    if (bridge->input_task != NULL) {
        vTaskDelete(bridge->input_task);
    }
    if (bridge->output_task != NULL) {
        vTaskDelete(bridge->output_task);
    }
    if (bridge->tx_tap_queue != NULL) {
        twai_frame_t frame = {};
        while (xQueueReceive(bridge->tx_tap_queue, &frame, 0) == pdTRUE) {
            free(frame.buffer);
        }
        vQueueDelete(bridge->tx_tap_queue);
    }
    free(bridge);
}
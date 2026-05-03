#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
#include "can_queue.hpp"
typedef can_queue_handle_t command_router_queue_handle_t;
#else
typedef void command_router_queue_handle_t;
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct command_router_t command_router_t;

typedef enum {
    COMMAND_ROUTER_RESPONSE_ACK = 0x80,
    COMMAND_ROUTER_RESPONSE_IN_PROGRESS = 0x81,
    COMMAND_ROUTER_RESPONSE_OK = 0x82,
    COMMAND_ROUTER_RESPONSE_ERROR = 0xFF,
} command_router_response_type_t;

typedef enum {
    COMMAND_ROUTER_HANDLER_RESULT_COMPLETE = 0,
    COMMAND_ROUTER_HANDLER_RESULT_ASYNC = 1,
} command_router_handler_result_t;

typedef struct {
    uint32_t command_rx_id;
    uint32_t response_tx_id;
    command_router_queue_handle_t *rx_handle;
    command_router_queue_handle_t *tx_handle;
    uint32_t task_stack_size;
    UBaseType_t task_priority;
} command_router_config_t;

typedef struct {
    uint8_t opcode;
    const uint8_t *payload;
    size_t payload_len;
    uint32_t arbitration_id;
} command_router_request_t;

typedef struct {
    command_router_t *router;
    uint8_t opcode;
} command_router_response_context_t;

typedef esp_err_t (*command_router_handler_fn)(
    void *handler_ctx,
    const command_router_request_t *request,
    command_router_response_context_t *response_ctx,
    command_router_handler_result_t *out_result);

typedef struct {
    uint8_t opcode;
    command_router_handler_fn handler;
    void *handler_ctx;
    bool allow_while_busy;
} command_router_handler_entry_t;

esp_err_t command_router_init(command_router_t **out_router, const command_router_config_t *config);
void command_router_deinit(command_router_t *router);

esp_err_t command_router_register_handlers(command_router_t *router, const command_router_handler_entry_t *entries, size_t entry_count);

esp_err_t command_router_send_ack(command_router_response_context_t *ctx);
esp_err_t command_router_send_in_progress(command_router_response_context_t *ctx, const uint8_t *payload, size_t payload_len);
esp_err_t command_router_send_ok(command_router_response_context_t *ctx, const uint8_t *payload, size_t payload_len);
esp_err_t command_router_send_error(command_router_response_context_t *ctx, uint8_t error_code);

#ifdef __cplusplus
}
#endif

#include "command_router.h"

#include <cstring>
#include <vector>

#include "esp_check.h"
#include "freertos/task.h"

namespace {

constexpr uint32_t kDefaultTaskStackSize = 4096;
constexpr UBaseType_t kDefaultTaskPriority = 8;

} // namespace

struct command_router_t {
    explicit command_router_t(const command_router_config_t &config)
        : config(config)
    {
    }

    command_router_config_t config;
    std::vector<command_router_handler_entry_t> handlers;
    TaskHandle_t task = nullptr;
    volatile bool stop_requested = false;
};

static bool command_router_is_extended_id(uint32_t id)
{
    return id > TWAI_STD_ID_MASK;
}

static esp_err_t command_router_send_response_internal(command_router_t *router,
                                                       uint8_t opcode,
                                                       command_router_response_type_t response_type,
                                                       const uint8_t *payload,
                                                       size_t payload_len)
{
    if (router == nullptr || router->config.tx_handle == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (payload_len > 6U) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t frame_payload[8] = {0};
    frame_payload[0] = (uint8_t)response_type;
    frame_payload[1] = opcode;
    if (payload != nullptr && payload_len > 0U) {
        memcpy(&frame_payload[2], payload, payload_len);
    }

    twai_frame_t frame = {};
    frame.header.id = router->config.response_tx_id;
    frame.header.ide = command_router_is_extended_id(router->config.response_tx_id);
    frame.header.rtr = 0;
    frame.header.fdf = 0;
    frame.header.brs = 0;
    frame.header.esi = 0;
    frame.header.dlc = 2U + payload_len;
    frame.buffer = frame_payload;
    frame.buffer_len = frame.header.dlc;

    return router->config.tx_handle->enqueue_tx(frame, 0) ? ESP_OK : ESP_FAIL;
}

static const command_router_handler_entry_t *command_router_find_handler(const command_router_t *router, uint8_t opcode)
{
    if (router == nullptr) {
        return nullptr;
    }

    for (const command_router_handler_entry_t &entry : router->handlers) {
        if (entry.opcode == opcode) {
            return &entry;
        }
    }

    return nullptr;
}

static void command_router_task_entry(void *arg)
{
    command_router_t *router = static_cast<command_router_t *>(arg);

    while (!router->stop_requested) {
        twai_frame_t frame = {};
        if (router->config.rx_handle == nullptr || !router->config.rx_handle->dequeue_rx(&frame, portMAX_DELAY)) {
            continue;
        }

        if (frame.header.rtr == 0U && frame.header.fdf == 0U && frame.buffer != nullptr && frame.buffer_len > 0U) {
            const uint8_t opcode = frame.buffer[0];
            const command_router_handler_entry_t *handler = command_router_find_handler(router, opcode);
            if (handler != nullptr && handler->handler != nullptr) {
                command_router_request_t request = {
                    .opcode = opcode,
                    .payload = frame.buffer + 1,
                    .payload_len = frame.buffer_len - 1U,
                    .arbitration_id = frame.header.id,
                };
                command_router_response_context_t response_ctx = {
                    .router = router,
                    .opcode = opcode,
                };
                command_router_handler_result_t result = COMMAND_ROUTER_HANDLER_RESULT_COMPLETE;
                esp_err_t err = handler->handler(handler->handler_ctx, &request, &response_ctx, &result);
                if (err != ESP_OK && err != ESP_ERR_NOT_SUPPORTED) {
                    const uint8_t error_code = 1U;
                    (void)command_router_send_response_internal(router, opcode, COMMAND_ROUTER_RESPONSE_ERROR, &error_code, 1U);
                }
            } else {
                const uint8_t error_code = 1U;
                (void)command_router_send_response_internal(router, opcode, COMMAND_ROUTER_RESPONSE_ERROR, &error_code, 1U);
            }
        }

        free(frame.buffer);
        frame.buffer = nullptr;
    }

    router->task = nullptr;
    vTaskDelete(nullptr);
}

extern "C" esp_err_t command_router_init(command_router_t **out_router, const command_router_config_t *config)
{
    if (out_router == nullptr || config == nullptr || config->rx_handle == nullptr || config->tx_handle == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    command_router_config_t normalized = *config;
    if (normalized.task_stack_size == 0U) {
        normalized.task_stack_size = kDefaultTaskStackSize;
    }
    if (normalized.task_priority == 0U) {
        normalized.task_priority = kDefaultTaskPriority;
    }

    command_router_t *router = new command_router_t(normalized);
    if (router == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    BaseType_t task_ok = xTaskCreate(command_router_task_entry,
                                     "command_router",
                                     router->config.task_stack_size,
                                     router,
                                     router->config.task_priority,
                                     &router->task);
    if (task_ok != pdPASS) {
        delete router;
        return ESP_ERR_NO_MEM;
    }

    *out_router = router;
    return ESP_OK;
}

extern "C" void command_router_deinit(command_router_t *router)
{
    if (router == nullptr) {
        return;
    }

    router->stop_requested = true;
    for (int attempt = 0; attempt < 5 && router->task != nullptr; ++attempt) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (router->task != nullptr) {
        vTaskDelete(router->task);
        router->task = nullptr;
    }

    delete router;
}

extern "C" esp_err_t command_router_register_handlers(command_router_t *router,
                                                        const command_router_handler_entry_t *entries,
                                                        size_t entry_count)
{
    if (router == nullptr || (entry_count > 0U && entries == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }

    router->handlers.assign(entries, entries + entry_count);
    return ESP_OK;
}

extern "C" esp_err_t command_router_send_ack(command_router_response_context_t *ctx)
{
    if (ctx == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    return command_router_send_response_internal(ctx->router, ctx->opcode, COMMAND_ROUTER_RESPONSE_ACK, nullptr, 0U);
}

extern "C" esp_err_t command_router_send_in_progress(command_router_response_context_t *ctx,
                                                       const uint8_t *payload,
                                                       size_t payload_len)
{
    if (ctx == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    return command_router_send_response_internal(ctx->router, ctx->opcode, COMMAND_ROUTER_RESPONSE_IN_PROGRESS, payload, payload_len);
}

extern "C" esp_err_t command_router_send_ok(command_router_response_context_t *ctx,
                                              const uint8_t *payload,
                                              size_t payload_len)
{
    if (ctx == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    return command_router_send_response_internal(ctx->router, ctx->opcode, COMMAND_ROUTER_RESPONSE_OK, payload, payload_len);
}

extern "C" esp_err_t command_router_send_error(command_router_response_context_t *ctx, uint8_t error_code)
{
    if (ctx == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    return command_router_send_response_internal(ctx->router, ctx->opcode, COMMAND_ROUTER_RESPONSE_ERROR, &error_code, 1U);
}

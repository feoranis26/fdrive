#pragma once

#include "config_store.h"
#include "esp_err.h"

typedef struct command_router_t command_router_t;
typedef struct drive_management_service_t drive_management_service_t;
typedef struct sense_service_t sense_service_t;
typedef struct drive_control_t drive_control_t;

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t drive_management_service_init(drive_management_service_t **out_service,
                                        command_router_t *router,
                                        config_store_t *config_store,
                                        sense_service_t *sense_service,
                                        drive_control_t *drive_control);
void drive_management_service_deinit(drive_management_service_t *service);
void drive_management_service_poll(drive_management_service_t *service);

#ifdef __cplusplus
}
#endif
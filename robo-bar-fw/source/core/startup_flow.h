#ifndef STARTUP_FLOW_H
#define STARTUP_FLOW_H

#include <stdint.h>
#include <stdbool.h>
#include "../config/system_config.h"
#include "../shared/system_types.h"

/*******************************************************************************
 * Startup Flow States (Simplified)
 ******************************************************************************/
typedef enum {
    STARTUP_STATE_INIT = 0,
    STARTUP_STATE_INITIALIZING,
    STARTUP_STATE_COMPLETE,
    STARTUP_STATE_ERROR
} startup_flow_state_t;

/*******************************************************************************
 * Startup Flow Control Structure
 ******************************************************************************/
typedef struct {
    startup_flow_state_t current_state;
    bool hardware_initialized;
} startup_flow_t;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
operation_status_t StartupFlow_Run(void);
bool StartupFlow_Init(void);
operation_status_t StartupFlow_Execute(void);  // Cambiado el tipo de retorno
void StartupFlow_ShowInitScreen(void);
void StartupFlow_ShowError(const char* error_msg);
const char* StartupFlow_StateToString(startup_flow_state_t state);

#endif /* STARTUP_FLOW_H */
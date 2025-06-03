/*
 * uart_config.h
 * 
 * UART configuration header for debugging and communication
 * FRDM-KL25Z Development Board - Robo-Bar project
 */

#ifndef UART_CONFIG_H
#define UART_CONFIG_H

#include "system_config.h"
#include "fsl_uart.h"
#include "fsl_port.h"
#include "fsl_clock.h"
#include <stdarg.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* UART instance and configuration */
#define DEBUG_UART                  UART0
#define DEBUG_UART_CLKSRC           UART0_CLK_SRC
#define DEBUG_UART_CLK_FREQ         CLOCK_GetFreq(UART0_CLK_SRC)
#define DEBUG_UART_IRQ              UART0_IRQn
#define DEBUG_UART_IRQ_HANDLER      UART0_IRQHandler

/* UART pins (FRDM-KL25Z default) */
#define DEBUG_UART_RX_PORT          PORTA
#define DEBUG_UART_RX_GPIO          GPIOA
#define DEBUG_UART_RX_PIN           1U
#define DEBUG_UART_RX_PIN_MUX       kPORT_MuxAlt2

#define DEBUG_UART_TX_PORT          PORTA
#define DEBUG_UART_TX_GPIO          GPIOA
#define DEBUG_UART_TX_PIN           2U
#define DEBUG_UART_TX_PIN_MUX       kPORT_MuxAlt2

/* UART configuration parameters */
#define DEBUG_UART_BAUDRATE         115200U
#define DEBUG_UART_DATA_BITS        kUART_EightDataBits
#define DEBUG_UART_PARITY           kUART_ParityDisabled
#define DEBUG_UART_STOP_BITS        kUART_OneStopBit

/* Buffer sizes */
#define UART_TX_BUFFER_SIZE         512U
#define UART_RX_BUFFER_SIZE         256U
#define UART_PRINTF_BUFFER_SIZE     256U

/* Debug levels */
typedef enum {
    DEBUG_LEVEL_NONE = 0,
    DEBUG_LEVEL_ERROR,
    DEBUG_LEVEL_WARNING,
    DEBUG_LEVEL_INFO,
    DEBUG_LEVEL_DEBUG,
    DEBUG_LEVEL_VERBOSE
} debug_level_t;

/* UART status */
typedef enum {
    UART_STATUS_OK = 0,
    UART_STATUS_ERROR,
    UART_STATUS_BUSY,
    UART_STATUS_TIMEOUT,
    UART_STATUS_BUFFER_FULL
} uart_status_t;

/* UART configuration structure */
typedef struct {
    UART_Type *base;
    uint32_t baudrate;
    uart_data_bits_t data_bits;
    uart_parity_mode_t parity;
    uart_stop_bit_count_t stop_bits;
    bool enable_tx;
    bool enable_rx;
    bool enable_interrupt;
} uart_config_extended_t;

/* Statistics structure */
typedef struct {
    uint32_t bytes_transmitted;
    uint32_t bytes_received;
    uint32_t tx_errors;
    uint32_t rx_errors;
    uint32_t buffer_overruns;
} uart_statistics_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Initialize UART configuration
 */
void uart_config_init(void);

/*!
 * @brief Initialize debug UART
 */
void uart_debug_init(void);

/*!
 * @brief Deinitialize UART
 */
void uart_config_deinit(void);

/*!
 * @brief Send data via UART
 * @param data Pointer to data buffer
 * @param length Number of bytes to send
 * @return UART status
 */
uart_status_t uart_send_data(const uint8_t *data, size_t length);

/*!
 * @brief Send string via UART
 * @param str Null-terminated string to send
 * @return UART status
 */
uart_status_t uart_send_string(const char *str);

/*!
 * @brief Receive data via UART
 * @param data Pointer to receive buffer
 * @param length Maximum number of bytes to receive
 * @param received Pointer to store actual bytes received
 * @return UART status
 */
uart_status_t uart_receive_data(uint8_t *data, size_t length, size_t *received);

/*!
 * @brief Printf-style formatted output via UART
 * @param format Format string
 * @param ... Variable arguments
 * @return Number of characters printed
 */
int uart_printf(const char *format, ...);

/*!
 * @brief Debug printf with level filtering
 * @param level Debug level
 * @param format Format string
 * @param ... Variable arguments
 */
void debug_printf(debug_level_t level, const char *format, ...);

/*!
 * @brief Set debug level
 * @param level New debug level
 */
void uart_set_debug_level(debug_level_t level);

/*!
 * @brief Get debug level
 * @return Current debug level
 */
debug_level_t uart_get_debug_level(void);

/*!
 * @brief Check if UART is ready for transmission
 * @return true if ready, false otherwise
 */
bool uart_is_tx_ready(void);

/*!
 * @brief Check if UART has received data
 * @return true if data available, false otherwise
 */
bool uart_is_rx_ready(void);

/*!
 * @brief Get UART statistics
 * @return Pointer to statistics structure
 */
const uart_statistics_t* uart_get_statistics(void);

/*!
 * @brief Reset UART statistics
 */
void uart_reset_statistics(void);

/*!
 * @brief Enable/disable UART interrupts
 * @param enable true to enable, false to disable
 */
void uart_set_interrupt_enable(bool enable);

/*!
 * @brief Flush TX buffer
 */
void uart_flush_tx(void);

/*!
 * @brief Flush RX buffer
 */
void uart_flush_rx(void);

/*!
 * @brief UART IRQ handler
 */
void DEBUG_UART_IRQ_HANDLER(void);

/*******************************************************************************
 * Debug Macros
 ******************************************************************************/

#if DEBUG_UART_ENABLED
    #define DEBUG_PRINT_ERROR(fmt, ...)    debug_printf(DEBUG_LEVEL_ERROR, "[ERROR] " fmt "\r\n", ##__VA_ARGS__)
    #define DEBUG_PRINT_WARNING(fmt, ...)  debug_printf(DEBUG_LEVEL_WARNING, "[WARN] " fmt "\r\n", ##__VA_ARGS__)
    #define DEBUG_PRINT_INFO(fmt, ...)     debug_printf(DEBUG_LEVEL_INFO, "[INFO] " fmt "\r\n", ##__VA_ARGS__)
    #define DEBUG_PRINT_DEBUG(fmt, ...)    debug_printf(DEBUG_LEVEL_DEBUG, "[DEBUG] " fmt "\r\n", ##__VA_ARGS__)
    #define DEBUG_PRINT_VERBOSE(fmt, ...)  debug_printf(DEBUG_LEVEL_VERBOSE, "[VERBOSE] " fmt "\r\n", ##__VA_ARGS__)
#else
    #define DEBUG_PRINT_ERROR(fmt, ...)    ((void)0)
    #define DEBUG_PRINT_WARNING(fmt, ...)  ((void)0)
    #define DEBUG_PRINT_INFO(fmt, ...)     ((void)0)
    #define DEBUG_PRINT_DEBUG(fmt, ...)    ((void)0)
    #define DEBUG_PRINT_VERBOSE(fmt, ...)  ((void)0)
#endif

/* System status debug macros */
#define DEBUG_SYSTEM_STATE(state)       DEBUG_PRINT_INFO("System State: %s", system_state_to_string(state))
#define DEBUG_SYSTEM_ERROR(error)       DEBUG_PRINT_ERROR("System Error: %s", system_error_to_string(error))

#ifdef __cplusplus
}
#endif

#endif /* UART_CONFIG_H */
/*
 * uart_config.c
 * 
 * UART configuration implementation for debugging and communication
 * FRDM-KL25Z Development Board - Robo-Bar project
 */

#include "uart_config.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

/*******************************************************************************
 * Private Variables
 ******************************************************************************/

/* UART buffers */
static uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static char printf_buffer[UART_PRINTF_BUFFER_SIZE];

/* Buffer pointers */
static volatile size_t tx_buffer_head = 0;
static volatile size_t tx_buffer_tail = 0;
static volatile size_t rx_buffer_head = 0;
static volatile size_t rx_buffer_tail = 0;

/* UART state */
static bool uart_initialized = false;
static debug_level_t current_debug_level = DEBUG_LEVEL_INFO;
static uart_statistics_t uart_stats = {0};

/* FreeRTOS synchronization */
static SemaphoreHandle_t uart_tx_mutex = NULL;
static SemaphoreHandle_t uart_rx_mutex = NULL;

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

static void uart_configure_pins(void);
static size_t uart_tx_buffer_available(void);
static size_t uart_rx_buffer_available(void);
static void uart_tx_buffer_put(uint8_t data);
static uint8_t uart_rx_buffer_get(void);
static void uart_start_transmission(void);

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/*!
 * @brief Configure UART pins
 */
static void uart_configure_pins(void)
{
    /* Enable PORTA clock */
    CLOCK_EnableClock(kCLOCK_PortA);
    
    /* Configure RX pin */
    PORT_SetPinMux(DEBUG_UART_RX_PORT, DEBUG_UART_RX_PIN, DEBUG_UART_RX_PIN_MUX);
    
    /* Configure TX pin */
    PORT_SetPinMux(DEBUG_UART_TX_PORT, DEBUG_UART_TX_PIN, DEBUG_UART_TX_PIN_MUX);
}

/*!
 * @brief Get available space in TX buffer
 * @return Number of available bytes
 */
static size_t uart_tx_buffer_available(void)
{
    size_t head = tx_buffer_head;
    size_t tail = tx_buffer_tail;
    
    if (head >= tail)
    {
        return (UART_TX_BUFFER_SIZE - 1) - (head - tail);
    }
    else
    {
        return (tail - head - 1);
    }
}

/*!
 * @brief Get available data in RX buffer
 * @return Number of available bytes
 */
static size_t uart_rx_buffer_available(void)
{
    size_t head = rx_buffer_head;
    size_t tail = rx_buffer_tail;
    
    if (head >= tail)
    {
        return (head - tail);
    }
    else
    {
        return (UART_RX_BUFFER_SIZE - tail + head);
    }
}

/*!
 * @brief Put data into TX buffer
 * @param data Data byte to put
 */
static void uart_tx_buffer_put(uint8_t data)
{
    size_t next_head = (tx_buffer_head + 1) % UART_TX_BUFFER_SIZE;
    
    if (next_head != tx_buffer_tail)
    {
        uart_tx_buffer[tx_buffer_head] = data;
        tx_buffer_head = next_head;
    }
    else
    {
        uart_stats.buffer_overruns++;
    }
}

/*!
 * @brief Get data from RX buffer
 * @return Data byte
 */
static uint8_t uart_rx_buffer_get(void)
{
    uint8_t data = 0;
    
    if (rx_buffer_head != rx_buffer_tail)
    {
        data = uart_rx_buffer[rx_buffer_tail];
        rx_buffer_tail = (rx_buffer_tail + 1) % UART_RX_BUFFER_SIZE;
    }
    
    return data;
}

/*!
 * @brief Start UART transmission
 */
static void uart_start_transmission(void)
{
    if (tx_buffer_head != tx_buffer_tail && 
        (UART_GetStatusFlags(DEBUG_UART) & kUART_TxDataRegEmptyFlag))
    {
        uint8_t data = uart_tx_buffer[tx_buffer_tail];
        tx_buffer_tail = (tx_buffer_tail + 1) % UART_TX_BUFFER_SIZE;
        UART_WriteByte(DEBUG_UART, data);
        uart_stats.bytes_transmitted++;
    }
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/*!
 * @brief Initialize UART configuration
 */
void uart_config_init(void)
{
    if (uart_initialized)
        return;
    
    /* Create mutexes */
    uart_tx_mutex = xSemaphoreCreateMutex();
    uart_rx_mutex = xSemaphoreCreateMutex();
    
    if (uart_tx_mutex == NULL || uart_rx_mutex == NULL)
    {
        /* Handle mutex creation failure */
        return;
    }
    
    /* Initialize UART for debugging */
    uart_debug_init();
    
    uart_initialized = true;
}

/*!
 * @brief Initialize debug UART
 */
void uart_debug_init(void)
{
    uart_config_t uart_config;
    
    /* Configure UART pins */
    uart_configure_pins();
    
    /* Get default UART configuration */
    UART_GetDefaultConfig(&uart_config);
    uart_config.baudRate_Bps = DEBUG_UART_BAUDRATE;
    uart_config.enableTx = true;
    uart_config.enableRx = true;
    
    /* Initialize UART */
    UART_Init(DEBUG_UART, &uart_config, DEBUG_UART_CLK_FREQ);
    
    /* Enable UART interrupts */
    UART_EnableInterrupts(DEBUG_UART, kUART_RxDataRegFullInterruptEnable | 
                                     kUART_TxDataRegEmptyInterruptEnable);
    
    /* Enable UART IRQ */
    EnableIRQ(DEBUG_UART_IRQ);
    
    /* Initialize buffer pointers */
    tx_buffer_head = tx_buffer_tail = 0;
    rx_buffer_head = rx_buffer_tail = 0;
    
    /* Reset statistics */
    memset((void*)&uart_stats, 0, sizeof(uart_stats));
}

/*!
 * @brief Deinitialize UART
 */
void uart_config_deinit(void)
{
    if (!uart_initialized)
        return;
    
    /* Disable UART interrupts */
    DisableIRQ(DEBUG_UART_IRQ);
    UART_DisableInterrupts(DEBUG_UART, kUART_RxDataRegFullInterruptEnable | 
                                      kUART_TxDataRegEmptyInterruptEnable);
    
    /* Deinitialize UART */
    UART_Deinit(DEBUG_UART);
    
    /* Delete mutexes */
    if (uart_tx_mutex != NULL)
    {
        vSemaphoreDelete(uart_tx_mutex);
        uart_tx_mutex = NULL;
    }
    
    if (uart_rx_mutex != NULL)
    {
        vSemaphoreDelete(uart_rx_mutex);
        uart_rx_mutex = NULL;
    }
    
    uart_initialized = false;
}

/*!
 * @brief Send data via UART
 * @param data Pointer to data buffer
 * @param length Number of bytes to send
 * @return UART status
 */
uart_status_t uart_send_data(const uint8_t *data, size_t length)
{
    if (!uart_initialized || data == NULL || length == 0)
        return UART_STATUS_ERROR;
    
    if (xSemaphoreTake(uart_tx_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
        return UART_STATUS_TIMEOUT;
    
    uart_status_t status = UART_STATUS_OK;
    
    for (size_t i = 0; i < length; i++)
    {
        if (uart_tx_buffer_available() == 0)
        {
            status = UART_STATUS_BUFFER_FULL;
            break;
        }
        
        uart_tx_buffer_put(data[i]);
    }
    
    /* Start transmission */
    uart_start_transmission();
    
    xSemaphoreGive(uart_tx_mutex);
    return status;
}

/*!
 * @brief Send string via UART
 * @param str Null-terminated string to send
 * @return UART status
 */
uart_status_t uart_send_string(const char *str)
{
    if (str == NULL)
        return UART_STATUS_ERROR;
    
    return uart_send_data((const uint8_t*)str, strlen(str));
}

/*!
 * @brief Receive data via UART
 * @param data Pointer to receive buffer
 * @param length Maximum number of bytes to receive
 * @param received Pointer to store actual bytes received
 * @return UART status
 */
uart_status_t uart_receive_data(uint8_t *data, size_t length, size_t *received)
{
    if (!uart_initialized || data == NULL || length == 0)
        return UART_STATUS_ERROR;
    
    if (xSemaphoreTake(uart_rx_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
        return UART_STATUS_TIMEOUT;
    
    size_t bytes_read = 0;
    size_t available = uart_rx_buffer_available();
    size_t to_read = (length < available) ? length : available;
    
    for (size_t i = 0; i < to_read; i++)
    {
        data[i] = uart_rx_buffer_get();
        bytes_read++;
    }
    
    if (received != NULL)
        *received = bytes_read;
    
    xSemaphoreGive(uart_rx_mutex);
    return UART_STATUS_OK;
}

/*!
 * @brief Printf-style formatted output via UART
 * @param format Format string
 * @param ... Variable arguments
 * @return Number of characters printed
 */
int uart_printf(const char *format, ...)
{
    if (!uart_initialized || format == NULL)
        return -1;
    
    va_list args;
    va_start(args, format);
    
    int length = vsnprintf(printf_buffer, UART_PRINTF_BUFFER_SIZE, format, args);
    
    va_end(args);
    
    if (length > 0)
    {
        uart_send_data((const uint8_t*)printf_buffer, (size_t)length);
    }
    
    return length;
}

/*!
 * @brief Debug printf with level filtering
 * @param level Debug level
 * @param format Format string
 * @param ... Variable arguments
 */
void debug_printf(debug_level_t level, const char *format, ...)
{
    if (!uart_initialized || level > current_debug_level || format == NULL)
        return;
    
    va_list args;
    va_start(args, format);
    
    int length = vsnprintf(printf_buffer, UART_PRINTF_BUFFER_SIZE, format, args);
    
    va_end(args);
    
    if (length > 0)
    {
        uart_send_data((const uint8_t*)printf_buffer, (size_t)length);
    }
}

/*!
 * @brief Set debug level
 * @param level New debug level
 */
void uart_set_debug_level(debug_level_t level)
{
    current_debug_level = level;
}

/*!
 * @brief Get debug level
 * @return Current debug level
 */
debug_level_t uart_get_debug_level(void)
{
    return current_debug_level;
}

/*!
 * @brief Check if UART is ready for transmission
 * @return true if ready, false otherwise
 */
bool uart_is_tx_ready(void)
{
    return uart_initialized && (uart_tx_buffer_available() > 0);
}

/*!
 * @brief Check if UART has received data
 * @return true if data available, false otherwise
 */
bool uart_is_rx_ready(void)
{
    return uart_initialized && (uart_rx_buffer_available() > 0);
}

/*!
 * @brief Get UART statistics
 * @return Pointer to statistics structure
 */
const uart_statistics_t* uart_get_statistics(void)
{
    return &uart_stats;
}

/*!
 * @brief Reset UART statistics
 */
void uart_reset_statistics(void)
{
    memset((void*)&uart_stats, 0, sizeof(uart_stats));
}

/*!
 * @brief Enable/disable UART interrupts
 * @param enable true to enable, false to disable
 */
void uart_set_interrupt_enable(bool enable)
{
    if (!uart_initialized)
        return;
    
    if (enable)
    {
        UART_EnableInterrupts(DEBUG_UART, kUART_RxDataRegFullInterruptEnable | 
                                         kUART_TxDataRegEmptyInterruptEnable);
        EnableIRQ(DEBUG_UART_IRQ);
    }
    else
    {
        DisableIRQ(DEBUG_UART_IRQ);
        UART_DisableInterrupts(DEBUG_UART, kUART_RxDataRegFullInterruptEnable | 
                                          kUART_TxDataRegEmptyInterruptEnable);
    }
}

/*!
 * @brief Flush TX buffer
 */
void uart_flush_tx(void)
{
    if (xSemaphoreTake(uart_tx_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        tx_buffer_head = tx_buffer_tail = 0;
        xSemaphoreGive(uart_tx_mutex);
    }
}

/*!
 * @brief Flush RX buffer
 */
void uart_flush_rx(void)
{
    if (xSemaphoreTake(uart_rx_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        rx_buffer_head = rx_buffer_tail = 0;
        xSemaphoreGive(uart_rx_mutex);
    }
}

/*!
 * @brief UART IRQ handler
 */
void DEBUG_UART_IRQ_HANDLER(void)
{
    uint32_t status = UART_GetStatusFlags(DEBUG_UART);
    
    /* Handle RX interrupt */
    if (status & kUART_RxDataRegFullFlag)
    {
        uint8_t data = UART_ReadByte(DEBUG_UART);
        
        size_t next_head = (rx_buffer_head + 1) % UART_RX_BUFFER_SIZE;
        if (next_head != rx_buffer_tail)
        {
            uart_rx_buffer[rx_buffer_head] = data;
            rx_buffer_head = next_head;
            uart_stats.bytes_received++;
        }
        else
        {
            uart_stats.buffer_overruns++;
        }
    }
    
    /* Handle TX interrupt */
    if (status & kUART_TxDataRegEmptyFlag)
    {
        if (tx_buffer_head != tx_buffer_tail)
        {
            uint8_t data = uart_tx_buffer[tx_buffer_tail];
            tx_buffer_tail = (tx_buffer_tail + 1) % UART_TX_BUFFER_SIZE;
            UART_WriteByte(DEBUG_UART, data);
            uart_stats.bytes_transmitted++;
        }
        else
        {
            /* No more data to send, disable TX interrupt */
            UART_DisableInterrupts(DEBUG_UART, kUART_TxDataRegEmptyInterruptEnable);
        }
    }
    
    /* Handle errors */
    if (status & (kUART_RxOverrunFlag | kUART_ParityErrorFlag | kUART_FramingErrorFlag))
    {
        /* Clear error flags */
        UART_ClearStatusFlags(DEBUG_UART, status);
        
        if (status & kUART_RxOverrunFlag)
            uart_stats.rx_errors++;
        if (status & (kUART_ParityErrorFlag | kUART_FramingErrorFlag))
            uart_stats.tx_errors++;
    }
}
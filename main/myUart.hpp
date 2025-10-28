#include "driver/uart.h"

class MyUart {
public:
    void begin(uart_port_t uart_port, int pin_tx, int pin_rx, int BAUDRATE) {
        uart_num = uart_port;

        // Install UART driver using an event queue
        uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0);

        uart_config_t uart_config = {
            .baud_rate = BAUDRATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT
        };
        // Configure UART parameters
        ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

        // Set your TX and RX pins
        ESP_ERROR_CHECK(uart_set_pin(uart_num, pin_tx, pin_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    }

    void print(const char* data) {
        uart_write_bytes(uart_num, (const char*)data, strlen(data));
    }

    void printf(const char* format, ...) {
        uart_write_bytes(uart_num, (const char*)format, strlen(format));
    }

    int read() {
        uint8_t c = 0;
        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
        if(uart_read_bytes(uart_num, &c, 1, 0) == 1) {
            return c;
        } else {
            return -1;
        }
    }

    int available() {
        size_t available;
        uart_get_buffered_data_len(uart_num, &available);

        return available;
    }

    void write(const uint8_t *data, size_t len) {
        if (data == NULL || !len) {
            return;
        }

        uart_write_bytes(uart_num, data, len);
    }

private:
    const int uart_buffer_size = (1024 * 2);
    uart_port_t uart_num;
    QueueHandle_t uart_queue;
};
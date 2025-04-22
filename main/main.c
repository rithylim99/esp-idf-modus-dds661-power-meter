/***********************************************
 
    ESP-IDF Nanomodbus master example 

    ESP-IDF 5.3.2
    Author: Rithy Lim
    Date: 2025-04-22
    Description:
        ESP-IDF Nanomodbus master with DDS661 example
    Hardware:
        ESP32

************************************************/


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nanomodbus.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

static const char *TAG = "esp_nmbs_master";

static const int RX_BUF_SIZE = 1024;

#define RTU_SERVER_ADDRESS  (0x01) // DDS661 ID
#define MODBUS_UART_NUM     (UART_NUM_1)
#define MODBUS_TXD_PIN      (GPIO_NUM_17)
#define MODBUS_RXD_PIN      (GPIO_NUM_16)
#define MODBUS_EN_PIN       (GPIO_NUM_4)

#define RELAY_1             (GPIO_NUM_21)
#define RELAY_2             (GPIO_NUM_19)

esp_err_t uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = 0x2, // UART_PARITY_EVEN enable
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // We won't use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(MODBUS_UART_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MODBUS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(MODBUS_UART_NUM, MODBUS_TXD_PIN, MODBUS_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    return ESP_OK;
}

int32_t uart_read(uint8_t* buf, uint16_t count, int32_t byte_timeout_ms, void* arg)
{
    ESP_UNUSED(arg);
    int len = uart_read_bytes(MODBUS_UART_NUM, buf, count, byte_timeout_ms / portTICK_PERIOD_MS);
    return len;
} 

int32_t uart_write(const uint8_t* buf, uint16_t count, int32_t byte_timeout_ms, void* arg)
{
    ESP_UNUSED(arg);
    gpio_set_level(MODBUS_EN_PIN, 1); // Enable transmission
    int len = uart_write_bytes(MODBUS_UART_NUM, (const char*)buf, count);
    uart_wait_tx_done(MODBUS_UART_NUM, pdMS_TO_TICKS(byte_timeout_ms));
    gpio_set_level(MODBUS_EN_PIN, 0); // Disable transmission (switch to RX)
    return len;
}

float regs_to_float(uint16_t hi, uint16_t lo)
{
    uint32_t raw = ((uint32_t)hi << 16) | lo;
    float value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}


void app_main(void)
{
    gpio_set_direction(MODBUS_EN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELAY_1, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(uart_init());
    // gpio_init();
    
    nmbs_platform_conf platform_conf;
    platform_conf.transport = NMBS_TRANSPORT_RTU;
    platform_conf.read = uart_read;
    platform_conf.write = uart_write;
    platform_conf.arg = NULL;

    // Create the modbus client
    nmbs_t nmbs;
    nmbs_error err = nmbs_client_create(&nmbs, &platform_conf);
    if (err != NMBS_ERROR_NONE) {
        ESP_LOGE(TAG, "Error creating modbus client\n");
    }

    // Set only the response timeout. Byte timeout will be handled by the TCP connection
    nmbs_set_read_timeout(&nmbs, 1000);
    nmbs_set_byte_timeout(&nmbs, 100);

    nmbs_set_destination_rtu_address(&nmbs, RTU_SERVER_ADDRESS);
    
    uint16_t address [6] = {0, 8, 18, 42, 54, 256}; // Register address with following param

    /* 

    Register Parameter Description

    In this case, I've convert from Hex of Address to Dec for easier reading

    Address (Hex)  | Description         | Unit             | Format         | Mode
    -------------------------------------------------------------------------------
    0x0000         | Voltage             | Volt             | Floating Point | Read Only
    0x0008         | Electric Current    | Amperage         | Floating Point | Read Only
    0x0012         | Active Power        | Kilowatts        | Floating Point | Read Only
    0x002A         | Power Factor        | COS              | Floating Point | Read Only
    0x0036         | Frequency           | Hertz (Physics)  | Floating Point | Read Only
    0x0100         | Total Active Power  | Kilowatt-hours   | Floating Point | Read Only
    
    */


    const char *label[] = {"Voltage", "Current", "Active Power", "pF", "F", "Total Power"};
    const char *unit[]  = {"V", "A", "kW", "COS", "Hz", "kWh"};

    gpio_set_level(RELAY_1,1);

    while (1)
    {
        uint16_t r_regs[2];
        ESP_LOGI(TAG, "=============================================");
        for (int i = 0; i < 6; i++)
        {
            err = nmbs_read_input_registers(&nmbs, address[i], 2, r_regs);
            // ESP_LOGI(TAG, "r_regs={%d, %d}", r_regs[0], r_regs[1]);
            float real_value = regs_to_float(r_regs[0], r_regs[1]);
            
            ESP_LOGI(TAG, "%s = %.2f %s",label[i], real_value, unit[i]);
        }
        ESP_LOGI(TAG, "=============================================");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
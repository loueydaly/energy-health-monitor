#include <Arduino.h>
#include "driver/twai.h"

// Hardware Pin Configuration
#define CAN_TX_PIN      GPIO_NUM_5
#define CAN_RX_PIN      GPIO_NUM_4

// Protocol Constants
#define FRAME_SIZE      14
#define STX_BYTE        0x02
#define ETX_BYTE        0x03

// Global buffers
uint8_t uart_rx_buffer[FRAME_SIZE];

// Helper: Calculate XOR CRC across bytes 1 to 11
uint8_t calculate_crc(const uint8_t *data) {
    uint8_t crc = 0;
    for (int i = 1; i <= 11; i++) {
        crc ^= data[i];
    }
    return crc;
}

void setup() {
    // Initialize USB UART to LabVIEW
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    // Configure TWAI (CAN) Peripheral for 500 kbps
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config  = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config  = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();

        // Enable alerts for status and recovery monitoring
        uint32_t alerts_to_enable = TWAI_ALERT_TX_SUCCESS    | 
                                    TWAI_ALERT_TX_FAILED     | 
                                    TWAI_ALERT_BUS_ERROR     |
                                    TWAI_ALERT_BUS_OFF       |
                                    TWAI_ALERT_BUS_RECOVERED;
        twai_reconfigure_alerts(alerts_to_enable, NULL);
    }
}

// -----------------------------------------------------------------------------
// Direction 1: LabVIEW UART -> ESP32 -> STM32 CAN (CAN ID 0x100)
// -----------------------------------------------------------------------------
void process_uart_to_can() {
    if (Serial.available() > 0) {
        // Resynchronize if buffer head is not STX (0x02)
        if (Serial.peek() != STX_BYTE) {
            Serial.read(); // Discard unaligned byte
            return;
        }

        // Wait until full 14-byte frame is available
        if (Serial.available() >= FRAME_SIZE) {
            Serial.readBytes(uart_rx_buffer, FRAME_SIZE);

            // Validate ETX framing
            if (uart_rx_buffer[FRAME_SIZE - 1] != ETX_BYTE) {
                return; // Corrupt frame alignment
            }

            // Validate Checksum
            uint8_t computed_crc = calculate_crc(uart_rx_buffer);
            if (computed_crc != uart_rx_buffer[12]) {
                Serial.write(0xEE); // NACK: CRC Error
                return;
            }

            // Build CAN Message
            twai_message_t tx_msg;
            tx_msg.identifier = ((uint32_t)uart_rx_buffer[1] << 8) | uart_rx_buffer[2];
            tx_msg.data_length_code = uart_rx_buffer[3];
            if (tx_msg.data_length_code > 8) {
                tx_msg.data_length_code = 8;
            }
            tx_msg.extd = 0; // Standard 11-bit ID
            tx_msg.rtr = 0;  // Data frame

            for (int i = 0; i < tx_msg.data_length_code; i++) {
                tx_msg.data[i] = uart_rx_buffer[4 + i];
            }

            // Transmit to CAN bus
            esp_err_t tx_result = twai_transmit(&tx_msg, pdMS_TO_TICKS(10));

            if (tx_result == ESP_OK) {
                uint32_t alerts_triggered = 0;
                twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(5));

                if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
                    Serial.write(0xAA); // ACK: Transmitted & Acknowledged on CAN
                } else if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
                    Serial.write(0xF1); // NACK: No ACK received from STM32
                } else if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
                    Serial.write(0xF2); // NACK: Bus error
                } else if (alerts_triggered & TWAI_ALERT_BUS_OFF) {
                    Serial.write(0xF3); // Critical: Bus-off condition
                    twai_initiate_recovery();
                } else {
                    Serial.write(0xAA); // Queued successfully
                }
            } else {
                Serial.write(0xF5); // TX buffer full / driver error
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Direction 2: STM32 CAN -> ESP32 -> LabVIEW UART (CAN ID 0x200 Quality Metrics)
// -----------------------------------------------------------------------------
void process_can_to_uart() {
    twai_message_t rx_msg;

    // Non-blocking read from CAN buffer
    if (twai_receive(&rx_msg, 0) == ESP_OK) {
        // Forward standard CAN frames (specifically 0x200)
        if (!rx_msg.extd) {
            uint8_t tx_uart_frame[FRAME_SIZE];

            tx_uart_frame[0] = STX_BYTE;                                      // 0x02
            tx_uart_frame[1] = (uint8_t)((rx_msg.identifier >> 8) & 0xFF);     // ID High (0x02)
            tx_uart_frame[2] = (uint8_t)(rx_msg.identifier & 0xFF);            // ID Low  (0x00)
            tx_uart_frame[3] = rx_msg.data_length_code;                       // DLC (8)

            // Copy 8 payload bytes
            for (int i = 0; i < 8; i++) {
                tx_uart_frame[4 + i] = (i < rx_msg.data_length_code) ? rx_msg.data[i] : 0x00;
            }

            // Compute XOR CRC across bytes 1 to 11
            tx_uart_frame[12] = calculate_crc(tx_uart_frame);

            // Framing end byte
            tx_uart_frame[13] = ETX_BYTE;                                     // 0x03

            // Stream to LabVIEW
            Serial.write(tx_uart_frame, FRAME_SIZE);
        }
    }

    // Check for automatic CAN recovery
    uint32_t bg_alerts = 0;
    twai_read_alerts(&bg_alerts, 0);
    if (bg_alerts & TWAI_ALERT_BUS_RECOVERED) {
        twai_start(); // Restart driver
    }
}

void loop() {
    process_uart_to_can();
    process_can_to_uart();
}
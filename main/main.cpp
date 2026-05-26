#include "uart_serial_comm.h"

#include "esp_log.h"

#include "driver/gpio.h"

static const char* TAG = "SERIAL_COMM_MAIN";


static void on_ping(void* ctx, const SerialCommProtoPacket& packet) {
    auto* serial_comm = static_cast<SerialComm*>(ctx);
    if (serial_comm == nullptr) {
        ESP_LOGE(TAG, "SerialComm context is null");
        return;
    }
    SerialCommProtoPacket reply;
    SerialCommProtocol::clear_packet(reply);
    reply.header.command = SerialCommProtoCommand::REPLY;
    reply.header.payload_len = 0;
    if (serial_comm->send(reply) != SerialCommResult_Codes::errCode::OK) {
        ESP_LOGE(TAG, "Failed to send PING reply");
        return;
    }
    ESP_LOGI(
        TAG,
        "PING Recv & REPLY Send (len=%u)",
        static_cast<unsigned>(packet.header.payload_len)
    );
}


static void on_reply(void* ctx, const SerialCommProtoPacket& packet) {
    (void)ctx;
    ESP_LOGI(
        TAG,
        "REPLY Recv (len=%u)",
        static_cast<unsigned>(packet.header.payload_len)
    );
}


static void on_read(void* ctx, const SerialCommProtoPacket& packet) {
    auto* serial_comm = static_cast<SerialComm*>(ctx);
    if (serial_comm == nullptr) {
        ESP_LOGE(TAG, "SerialComm context is null");
        return;
    }
    uint32_t ticks = static_cast<uint32_t>(xTaskGetTickCount());
    SerialCommProtoPacket reply;
    SerialCommProtocol::clear_packet(reply);
    reply.header.command = SerialCommProtoCommand::REPLY;
    reply.header.payload_len = sizeof(ticks);
    reply.payload[0] = static_cast<uint8_t>(ticks & 0xFF);
    reply.payload[1] = static_cast<uint8_t>((ticks >> 8) & 0xFF);
    reply.payload[2] = static_cast<uint8_t>((ticks >> 16) & 0xFF);
    reply.payload[3] = static_cast<uint8_t>((ticks >> 24) & 0xFF);
    if (serial_comm->send(reply) != SerialCommResult_Codes::errCode::OK) {
        ESP_LOGE(TAG, "Failed to send READ reply");
        return;
    }
    // ESP_LOGI(TAG, "READ - Ticks=%u seq=%u", static_cast<unsigned>(ticks), static_cast<unsigned>(seq));
}


extern "C" void app_main(void) {    

    static UARTTransport::HardwareConfig hw_cfg;
    hw_cfg.uart_port = UART_NUM_2;
    hw_cfg.tx_pin = GPIO_NUM_25;
    hw_cfg.rx_pin = GPIO_NUM_26;

    static UARTTransport transport(hw_cfg);

    static ISerialCommTransport::Config transport_cfg;
    transport_cfg.baudrate = SERIAL_COMM_UART_BAUDRATE;
    transport_cfg.rx_buffer_size = SERIAL_COMM_UART_RX_BUFFER_SIZE;
    transport_cfg.tx_buffer_size = SERIAL_COMM_UART_TX_BUFFER_SIZE;
    transport_cfg.half_duplex = false;

    static SerialComm::Config comm_cfg;
    comm_cfg.enable_inter_byte_timeout = true;
    comm_cfg.inter_byte_timeout_us = uart_interbyte_timeout_us(
        transport_cfg.baudrate
    );

    static SerialComm serial_comm(&transport);

    if (transport.init(transport_cfg) != SerialCommResult_Codes::errCode::OK) {
        ESP_LOGE(TAG, "Falha ao inicializar o transporte UART");
        return;
    }

    if (serial_comm.init(comm_cfg) != SerialCommResult_Codes::errCode::OK) {
        ESP_LOGE(TAG, "Falha ao inicializar o SerialComm");
        return;
    }

    gpio_set_direction( GPIO_NUM_2, GPIO_MODE_OUTPUT );
    gpio_set_level( GPIO_NUM_2, false );

    serial_comm.register_callback(
        SerialCommProtoCommand::PING,
        on_ping,
        &serial_comm
    );

    serial_comm.register_callback(
        SerialCommProtoCommand::REPLY,
        on_reply,
        nullptr
    );

    serial_comm.register_callback(
        SerialCommProtoCommand::READ,
        on_read,
        &serial_comm
    );

    if (serial_comm.start() != SerialCommResult_Codes::errCode::OK) {
        ESP_LOGE(TAG, "Falha ao iniciar o SerialComm");
        return;
    }

    // Encerra o Main
    vTaskDelete(nullptr);
}

/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @file esp_rom_bootloader.c
 * Driver for communicating with the ESP32 ROM bootloader
 *  
 */

#define DEBUG_MODULE "ESPROMBL"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include "debug.h"
#include "queue.h"
#include "deck.h"
#include "uart2.h"
#include "esp_rom_bootloader.h"
#include "esp_slip.h"

static esp_uart_send_packet sender_pckt;
static esp_uart_receive_packet receiver_pckt;

void espblInit()
{
    pinMode(DECK_GPIO_IO1, OUTPUT);
    digitalWrite(DECK_GPIO_IO1, LOW);
    pinMode(DECK_GPIO_IO4, OUTPUT);
    digitalWrite(DECK_GPIO_IO4, LOW);
    vTaskDelay(10);
    digitalWrite(DECK_GPIO_IO4, HIGH);
    pinMode(DECK_GPIO_IO4, INPUT_PULLUP);
    vTaskDelay(100);
    digitalWrite(DECK_GPIO_IO1, HIGH);
    pinMode(DECK_GPIO_IO1, INPUT_PULLUP);
}

bool espblSync()
{
    sender_pckt.command = SYNC;
    sender_pckt.data_size = 0x24;
    sender_pckt.data[0] = 0x07;
    sender_pckt.data[1] = 0x07;
    sender_pckt.data[2] = 0x12;
    sender_pckt.data[3] = 0x20;
    for (int i = 0; i < 32; i++)
    {
        sender_pckt.data[4 + i] = 0x55;
    }

    bool sync = false;
    for (int i = 0; i < 10 && !sync; i++) // maximum 10 sync attempts
    {
        sync = espblExchange(&receiver_pckt, &sender_pckt, uart2Putchar, uart2GetDataWithTimeout, 100);
    }

    // ESP32 responds multiple times upon succesful SYNC. Wait until all responses are received, so they can be cleared before next transmission.
    vTaskDelay(M2T(100));

    return sync;
}

bool spiAttach()
{
    sender_pckt.command = SPI_ATTACH;
    sender_pckt.data_size = 0x4;
    sender_pckt.data[0] = 0x00;
    sender_pckt.data[1] = 0x00;
    sender_pckt.data[2] = 0x00;
    sender_pckt.data[3] = 0x00;
    sender_pckt.data[4] = 0x00;
    sender_pckt.data[5] = 0x00;
    sender_pckt.data[6] = 0x00;
    sender_pckt.data[7] = 0x00;

    return espblExchange(&receiver_pckt, &sender_pckt, uart2Putchar, uart2GetDataWithTimeout, 100);
}

bool espblFlashBegin(uint32_t number_of_data_packets, uint32_t firmware_size, uint32_t flash_offset)
{
    sender_pckt.command = FLASH_BEGIN;
    sender_pckt.data_size = 0x10;
    sender_pckt.data[0] = (uint8_t)((firmware_size >> 0) & 0x000000FF);
    sender_pckt.data[1] = (uint8_t)((firmware_size >> 8) & 0x000000FF);
    sender_pckt.data[2] = (uint8_t)((firmware_size >> 16) & 0x000000FF);
    sender_pckt.data[3] = (uint8_t)((firmware_size >> 24) & 0x000000FF);

    sender_pckt.data[4] = (uint8_t)((number_of_data_packets >> 0) & 0x000000FF);
    sender_pckt.data[5] = (uint8_t)((number_of_data_packets >> 8) & 0x000000FF);
    sender_pckt.data[6] = (uint8_t)((number_of_data_packets >> 16) & 0x000000FF);
    sender_pckt.data[7] = (uint8_t)((number_of_data_packets >> 24) & 0x000000FF);

    sender_pckt.data[8] = (uint8_t)((ESP_MTU >> 0) & 0x000000FF);
    sender_pckt.data[9] = (uint8_t)((ESP_MTU >> 8) & 0x000000FF);
    sender_pckt.data[10] = (uint8_t)((ESP_MTU >> 16) & 0x000000FF);
    sender_pckt.data[11] = (uint8_t)((ESP_MTU >> 24) & 0x000000FF);
    sender_pckt.data[12] = (uint8_t)((flash_offset >> 0) & 0x000000FF);
    sender_pckt.data[13] = (uint8_t)((flash_offset >> 8) & 0x000000FF);
    sender_pckt.data[14] = (uint8_t)((flash_offset >> 16) & 0x000000FF);
    sender_pckt.data[15] = (uint8_t)((flash_offset >> 24) & 0x000000FF);

    return espblExchange(&receiver_pckt, &sender_pckt, uart2Putchar, uart2GetDataWithTimeout, 10000);
}

bool espblFlashData(uint8_t *esp_fw, uint32_t flash_data_size, uint32_t sequence_number)
{
    sender_pckt.command = FLASH_DATA;
    sender_pckt.data_size = ESP_MTU + 16; // set data size to the data size including the additional header

    sender_pckt.data[0] = (uint8_t)((ESP_MTU >> 0) & 0x000000FF);
    sender_pckt.data[1] = (uint8_t)((ESP_MTU >> 8) & 0x000000FF);
    sender_pckt.data[2] = (uint8_t)((ESP_MTU >> 16) & 0x000000FF);
    sender_pckt.data[3] = (uint8_t)((ESP_MTU >> 24) & 0x000000FF);

    sender_pckt.data[4] = (uint8_t)((sequence_number >> 0) & 0x000000FF);
    sender_pckt.data[5] = (uint8_t)((sequence_number >> 8) & 0x000000FF);
    sender_pckt.data[6] = (uint8_t)((sequence_number >> 16) & 0x000000FF);
    sender_pckt.data[7] = (uint8_t)((sequence_number >> 24) & 0x000000FF);

    sender_pckt.data[8] = 0x00;
    sender_pckt.data[9] = 0x00;
    sender_pckt.data[10] = 0x00;
    sender_pckt.data[11] = 0x00;
    sender_pckt.data[12] = 0x00;
    sender_pckt.data[13] = 0x00;
    sender_pckt.data[14] = 0x00;
    sender_pckt.data[15] = 0x00;

    memcpy(&sender_pckt.data[16], esp_fw, flash_data_size);
    if (flash_data_size < ESP_MTU)
    {
        // pad the data with 0xFF
        memset(&sender_pckt.data[16 + flash_data_size], 0xFF, ESP_MTU - flash_data_size);
    }
    return espblExchange(&receiver_pckt, &sender_pckt, uart2Putchar, uart2GetDataWithTimeout, 100);
}


bool espblFlashWrite(uint8_t *esp_fw, uint32_t esp_fw_size, uint32_t sequence_number)
{
    if (!espblFlashData(esp_fw, esp_fw_size, sequence_number))
    {
        DEBUG_PRINT("Failed to flash page %lu\n", sequence_number);
        return 0;
    }
    return 1;
}


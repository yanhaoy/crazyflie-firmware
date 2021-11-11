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
 * @file esp_slip.c
 * Protocol for assembling, sending, receiving and decoding SLIP packets to/from the ESP32 ROM bootloader
 *  
 */

#include <string.h>
#include "esp_slip.h"

#define ESP_OVERHEAD_LEN 8

static uint8_t sendBuffer[ESP_MTU + 10 + 16];
static uint32_t sendSize;

static uint8_t generateChecksum(esp_uart_send_packet *sender_pckt)
{
    uint8_t checksum = 0xEF; // seed
    for (int i = 0; i < sender_pckt->data_size - 16; i++)
    {
        checksum ^= sender_pckt->data[16 + i];
    }
    return checksum;
}

static void sendSLIPPacket(uint32_t size, uint8_t *data, coms_putchar_t putCharFnc)
{
    uint32_t i;

    for (i = 0; i < size; i++)
    {
        if ((data[i] == 0xC0 && i != 0 && i != size - 1) || (data[i] == 0xDB && i != 0 && i != size - 1))
        {
            for (int j = 0; j < 2; j++)
            {
                j == 0 ? putCharFnc(0xDB) : data[i] == 0xC0 ? putCharFnc(0xDC)
                                                            : putCharFnc(0xDD);
            }
        }
        else
        {
            putCharFnc(data[i]);
        }
    }
}
static void espblAssembleBuffer(esp_uart_send_packet *sender_pckt)
{
    sendSize = sender_pckt->data_size + ESP_OVERHEAD_LEN + 2;

    sendBuffer[0] = 0xC0;
    sendBuffer[1] = DIR_CMD;
    sendBuffer[2] = sender_pckt->command;
    sendBuffer[3] = (uint8_t)((sender_pckt->data_size >> 0) & 0x000000FF);
    sendBuffer[4] = (uint8_t)((sender_pckt->data_size >> 8) & 0x000000FF);

    if (sender_pckt->command == FLASH_DATA) // or MEM_DATA
    {
        uint32_t checksum = (uint32_t)generateChecksum(sender_pckt);
        sendBuffer[5] = (uint8_t)((checksum >> 0) & 0x000000FF);
        sendBuffer[6] = (uint8_t)((checksum >> 8) & 0x000000FF);
        sendBuffer[7] = (uint8_t)((checksum >> 16) & 0x000000FF);
        sendBuffer[8] = (uint8_t)((checksum >> 24) & 0x000000FF);
    }
    else
    {
        sendBuffer[5] = 0x00;
        sendBuffer[6] = 0x00;
        sendBuffer[7] = 0x00;
        sendBuffer[8] = 0x00;
    }

    if (sender_pckt->data_size)
    {
        memcpy(&sendBuffer[9], sender_pckt->data, sender_pckt->data_size);
    }

    sendBuffer[9 + sender_pckt->data_size] = 0xC0;
}

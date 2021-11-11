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

typedef enum
{
    receiveStart,
    receiveDirection,
    receiveCommand,
    receiveSize,
    receiveValue,
    receiveData,
    receiveEnd,
    error,
} ESPblReceiveState;
static ESPblReceiveState espblReceiveState = receiveStart;

static uint8_t size[2];
static uint8_t value[4];
static uint8_t data_index = 0;
static uint8_t size_index = 0;
static uint8_t value_index = 0;
static uint8_t previous_db = 0;

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

static slipDecoderStatus_t receiveSLIPPacket(uint8_t c, esp_uart_receive_packet *receiver_pckt, esp_uart_send_packet *sender_pckt)
{

    slipDecoderStatus_t decoderStatus = SLIP_DECODING;
    switch (espblReceiveState)
    {
    case receiveStart:
        receiver_pckt->status = 1;
        espblReceiveState = (c == 0xC0) ? receiveDirection : receiveStart;
        break;
    case receiveDirection:
        if (c == 0x01)
        {
            espblReceiveState = receiveCommand;
        }
        else
        {
            espblReceiveState = error;
            decoderStatus = SLIP_ERROR;
        }
        break;
    case receiveCommand:
        receiver_pckt->command = c;
        size_index = 0;
        if (c == sender_pckt->command)
        {
            espblReceiveState = receiveSize;
        }
        else
        {
            espblReceiveState = error;
            decoderStatus = SLIP_ERROR;
        }
        break;
    case receiveSize:
        size[size_index] = c;
        if (size_index == 1)
        {
            receiver_pckt->data_size = ((uint16_t)size[0] + ((uint16_t)size[1] << 8));
            value_index = 0;
            if (receiver_pckt->data_size > 0 && receiver_pckt->data_size < ESP_MTU)
            {
                espblReceiveState = receiveValue;
            }
            else
            {
                espblReceiveState = error;
                decoderStatus = SLIP_ERROR;
            }
        }
        size_index++;
        break;
    case receiveValue: // only used for READ_REG
        value[value_index] = c;
        if (value_index == 3)
        {
            receiver_pckt->value = ((uint32_t)value[0] + ((uint32_t)value[1] << 8) + ((uint32_t)value[2] << 16) + ((uint32_t)value[3] << 24));
            data_index = 0;
            previous_db = 0;
            espblReceiveState = receiveData;
        }
        value_index++;
        break;
    case receiveData:
    {
        const uint16_t payloadSize = receiver_pckt->data_size - 4;
        if (data_index < payloadSize)
        {
            if (c == 0xDB)
            {
                previous_db = 1;
            }
            else if (previous_db)
            {
                previous_db = 0;
                if (c == 0xDC)
                {
                    receiver_pckt->data[data_index] = 0xC0;
                    data_index++;
                }
                else if (c == 0xDD)
                {
                    receiver_pckt->data[data_index] = 0xDB;
                    data_index++;
                }
                else
                {
                    espblReceiveState = error;
                }
            }
            else
            {
                receiver_pckt->data[data_index] = c;
                data_index++;
            }
        }
        else
        {
            if (data_index == payloadSize)
            {
                receiver_pckt->status = c;
            }
            if (data_index == payloadSize + 1)
            {
                receiver_pckt->error = c;
            }
            if (data_index == payloadSize + 3)
            {
                espblReceiveState = receiveEnd;
            }
            data_index++;
        }
    }
    break;

    case receiveEnd:
        if (c == 0xC0)
        {
            espblReceiveState = receiveStart;
            decoderStatus = SLIP_SUCCESS;
        }
        else
        {
            espblReceiveState = error;
            decoderStatus = SLIP_ERROR;
        }
        break;

    case error:
        decoderStatus = SLIP_ERROR;
        break;
    default:
        break;
    }
    return decoderStatus;
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

static void clearUart2Buffer(coms_getDataWithTimeout_t getDataWithTimeout)
{
    uint8_t c;
    bool success = true;
    while (success)
    {
        success = getDataWithTimeout(&c, 1);
    }
    return;
}

#pragma once

// RPdenBoer July 2024
// Encapsulate reading and writing some relevant registers of the MT6816 encoder IC

#include <Arduino.h>
#include <SPI.h>

// Slow(er) speed helps with interference?
#define MT6816_SPI_SPEED 100000
#define MT6816_SPI_MODE SPI_MODE3
#define MT6816_SPI_ORDER MSBFIRST

#define MT6816_ANGLE_MSB 0x03
#define MT6816_ANGLE_LSB 0x04

#define MT6816_ZERO_MSB 0x32
#define MT6816_ZERO_LSB 0x33

#define MT6816_DIRECTION 0x1B
#define MT6816_MTP_STATUS 0x01

#define MT6816_KEY_ADDRESS 0x09
#define MT6816_KEY_VALUE 0xB3

#define MT6816_COMMAND_ADDRESS 0x0A
#define MT6816_COMMAND_VALUE 0x05

SPISettings settings(MT6816_SPI_SPEED, MT6816_SPI_ORDER, MT6816_SPI_MODE);

class MT6816
{
public:
    MT6816(uint32_t selectPin)
        : _selectPin(selectPin) {}

    void begin()
    {
        pinMode(_selectPin, OUTPUT);
        digitalWrite(_selectPin, HIGH);

        SPI.begin();

        SPI.beginTransaction(settings);
    }

    uint16_t readAngle()
    {
        // 14 bit number
        uint8_t highByte = readRegister(MT6816_ANGLE_MSB);
        uint8_t lowByte = readRegister(MT6816_ANGLE_LSB);

        uint16_t combined = (highByte << 8) | lowByte;

        if (!parityCheck(combined))
        {
            return 0;
        }

        // Downshift two bits (discard them)
        return combined >> 2;
    }

    uint16_t readZero()
    {
        // 12 bit number
        uint8_t highByte = readRegister(MT6816_ZERO_MSB);
        uint8_t lowByte = readRegister(MT6816_ZERO_LSB);

        uint16_t combined = (highByte << 8) | lowByte;

        // Discard top 4 bits (if there are any)
        return combined &= 0x0FFF;
    }

    uint8_t readWrites()
    {
        uint8_t status = readRegister(MT6816_MTP_STATUS);

        switch (status)
        {
        case 0:
            status = 1;
        case 1:
            status = 2;
        case 3:
            status = 3;
        case 7:
            status = 4;
        default:
            status = 5;
        }

        return status;
    }

private:
    uint8_t readRegister(uint8_t address)
    {
        uint16_t command = (0x80 | address) << 8;

        digitalWrite(_selectPin, LOW);

        uint16_t data = SPI.transfer16(command);
        Serial.println(data, BIN);

        digitalWrite(_selectPin, HIGH);

        return (data & 0xFF);
    }

    void writeRegister(uint8_t address, uint8_t data)
    {
        uint16_t command = (address & 0x7F) << 8 | data;

        digitalWrite(_selectPin, LOW);

        SPI.transfer16(command);

        digitalWrite(_selectPin, HIGH);
    }

    bool parityCheck(uint16_t data)
    {
        data ^= data >> 8;
        data ^= data >> 4;
        data ^= data >> 2;
        data ^= data >> 1;

        return (~data) & 1;
    }

    uint32_t _selectPin;
};
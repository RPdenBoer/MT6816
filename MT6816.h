#pragma once

// RPdenBoer July 2024
// To encapsulate reading and writing registers of the MT6816 encoder IC
// TODO: Add error handling and party checking

#include <Arduino.h>
#include <SPI.h>

#define SPI_SPEED 1000000
#define SPI_ORDER MSBFIRST
#define SPI_MODE SPI_MODE0

#define REG_ANGLE_MSB 0x03
#define REG_ANGLE_LSB 0x04

#define REG_ZERO_MSB 0x32
#define REG_ZERO_LSB 0x33

#define REG_DIRECTION 0x1B
#define REG_MTP_STATUS 0x01

#define REG_KEY_ADDRESS 0x09
#define REG_KEY_VALUE 0xB3

#define REG_COMMAND_ADDRESS 0x0A
#define REG_COMMAND_VALUE 0x05

class MT6816
{
public:
    MT6816(uint32_t selectPin)
        : _selectPin(selectPin) {}

    void init()
    {
        pinMode(_selectPin, OUTPUT);
        digitalWrite(_selectPin, HIGH);

        SPI.begin();
    }

    uint16_t readAngle()
    {
        uint8_t highByte = readRegister(REG_ANGLE_MSB);
        uint8_t lowByte = readRegister(REG_ANGLE_LSB);
        uint16_t combined = (highByte << 8) | lowByte;

        return combined >> 2;
    }

    uint16_t readZero()
    {
        uint8_t highByte = readRegister(REG_ZERO_MSB);
        uint8_t lowByte = readRegister(REG_ZERO_LSB);
        uint16_t combined = (highByte << 8) | lowByte;

        return combined &= 0x0FFF;
    }

    uint8_t readWrites()
    {
        uint8_t status = readRegister(REG_MTP_STATUS);

        switch (status)
        {
        case 0:
            status = 1;
            break;
        case 1:
            status = 2;
            break;
        case 3:
            status = 3;
            break;
        case 7:
            status = 4;
            break;
        // Default to 5, the declared MAX writes
        default:
            status = 5;
        }

        return status;
    }

    void writeZero(uint16_t value)
    {
        uint8_t highByte = (value >> 8) & 0xFF;
        uint8_t lowByte = value & 0xFF;

        writeRegister(REG_ANGLE_MSB, 0x00);
        writeRegister(REG_ANGLE_LSB, 0x00);

        writeRegister(REG_ANGLE_MSB, highByte);
        writeRegister(REG_ANGLE_LSB, lowByte);

        writeRegister(REG_KEY_ADDRESS, REG_KEY_VALUE);
        writeRegister(REG_COMMAND_ADDRESS, REG_COMMAND_VALUE);
    }

private:
    uint32_t _selectPin;

    uint8_t readRegister(uint8_t address)
    {
        uint16_t command = (0x80 | address) << 8;

        digitalWrite(_selectPin, LOW);
        SPI.beginTransaction(SPISettings(SPI_SPEED, SPI_ORDER, SPI_MODE));

        uint16_t data = SPI.transfer16(command);

        SPI.endTransaction();
        digitalWrite(_selectPin, HIGH);

        return (data & 0xFF);
    }

    void writeRegister(uint8_t address, uint8_t data)
    {
        uint16_t command = (address & 0x7F) << 8 | data;

        digitalWrite(_selectPin, LOW);
        SPI.beginTransaction(SPISettings(SPI_SPEED, SPI_ORDER, SPI_MODE));

        SPI.transfer16(command);

        SPI.endTransaction();
        digitalWrite(_selectPin, HIGH);
    }
};

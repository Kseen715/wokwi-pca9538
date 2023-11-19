// SPDX-License-Identifier: MIT
// Copyright 2023 Kseen715

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>

#ifndef DEBUG
// #define DEBUG
// #undef DEBUG
#endif

#define PCA9538_IN_REG 0x00
#define PCA9538_OUT_REG 0x01
#define PCA9538_POL_REG 0x02
#define PCA9538_CONF_REG 0x03

#define CHIP_ON (pin_read(chip->PIN_VCC)==HIGH)

void printBits(size_t const size, void  *ptr)
{
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;
    printf("0b");
    for (i = size-1; i >= 0; i--) {
        for (j = 7; j >= 0; j--) {
            byte = (b[i] >> j) & 1;
            printf("%u", byte);
        }
    }
}

typedef struct {
    pin_t PIN_VCC;
    pin_t PIN_SDA;
    pin_t PIN_SCL;
    pin_t PIN_INT;
    pin_t PIN_GND;
    pin_t PIN_RESET;
    pin_t PIN_A0;
    pin_t PIN_A1;
    pin_t PIN_P0;
    pin_t PIN_P1;
    pin_t PIN_P2;
    pin_t PIN_P3;
    pin_t PIN_P4;
    pin_t PIN_P5;
    pin_t PIN_P6;
    pin_t PIN_P7;
    uint8_t buffer;         // Буфер данных.
    uint8_t address;        // I2C адрес.
    uint8_t command;        // Текущяя команда.
    uint8_t config_dup;     // Копия конфигурации,
                            // т.к. мы не можем читать ее.
    uint8_t polarity_dup;   // Копия полярностей,
                            // т.к. мы не можем читать их.
    uint8_t last_input;     // Последнее обнаруженное
                            // состояние вводов.
} chip_state_t;

static bool on_i2c_connect(void *user_data, uint32_t address, bool connect);
static uint8_t on_i2c_read(void *user_data);
static bool on_i2c_write(void *user_data, uint8_t data);
static void on_i2c_disconnect(void *user_data);

bool i2c_address_init(chip_state_t *chip){
  uint32_t a0 = pin_read(chip->PIN_A0);
  uint32_t a1 = pin_read(chip->PIN_A1);
  uint8_t mode = 0;
  mode |= (a0 << 0) | (a1 << 1);
  switch (mode) {
      case 0b00:
          chip->address = 0x70;
          break;
      case 0b01:
          chip->address = 0x71;
          break;
      case 0b10:
          chip->address = 0x72;
          break;
      case 0b11:
          chip->address = 0x73;
          break;
      default:
          printf("PCA9538: Error! Wrong address configuration.\n");
          return false;
  }
  return true;
}

void chip_init() {
  chip_state_t *chip = malloc(sizeof(chip_state_t));

  chip->PIN_VCC = pin_init("VCC", INPUT_PULLDOWN);
  chip->PIN_SCL = pin_init("SCL", INPUT_PULLDOWN);
  chip->PIN_SDA = pin_init("SDA", INPUT_PULLDOWN);
  chip->PIN_INT = pin_init("INT", OUTPUT);
  chip->PIN_GND = pin_init("GND", INPUT_PULLDOWN);
  chip->PIN_RESET = pin_init("RESET", INPUT_PULLDOWN);

  chip->PIN_A0 = pin_init("A0", INPUT_PULLDOWN);
  chip->PIN_A1 = pin_init("A1", INPUT_PULLDOWN);

  chip->PIN_P0 = pin_init("P0", INPUT_PULLDOWN);
  chip->PIN_P1 = pin_init("P1", INPUT_PULLDOWN);
  chip->PIN_P2 = pin_init("P2", INPUT_PULLDOWN);
  chip->PIN_P3 = pin_init("P3", INPUT_PULLDOWN);
  chip->PIN_P4 = pin_init("P4", INPUT_PULLDOWN);
  chip->PIN_P5 = pin_init("P5", INPUT_PULLDOWN);
  chip->PIN_P6 = pin_init("P6", INPUT_PULLDOWN);
  chip->PIN_P7 = pin_init("P7", INPUT_PULLDOWN);

  chip->polarity_dup = 0x00;  // Все с прямой полярностью.
  chip->config_dup = 0xFF;    // Все на ввод.
  chip->buffer = 0;
  chip->last_input = 0;
  chip->command = 0xFF; // Будем считать, что в таком
                        // состоянии плата ждет команды.

  i2c_address_init(chip);

  const i2c_config_t i2c_config = {
    .user_data = chip,
    .address = chip->address,
    .scl = chip->PIN_SCL,
    .sda = chip->PIN_SDA,
    .connect = on_i2c_connect,
    .read = on_i2c_read,
    .write = on_i2c_write,
    .disconnect = on_i2c_disconnect,
  };

  i2c_init(&i2c_config);

  if (CHIP_ON) {
    printf("PCA9538 initialized on I2C with address 0x%02X!\n", chip->address);
  }
}

bool getNbit(uint8_t byte, uint8_t bit){
  return (bool)(byte & (0b1 << bit));
}

uint8_t shiftBit(uint8_t byte, uint8_t bit){
  return (byte & (0b1 << bit));
}

bool on_i2c_connect(void *user_data, uint32_t address, bool connect) {
  return true;
}

uint8_t on_i2c_read(void *user_data) {
  chip_state_t *chip = user_data;
  if (CHIP_ON) {
    chip->buffer = 0;

    if (getNbit(chip->config_dup, 7) == 1)
      chip->buffer |= pin_read(chip->PIN_P7)
        ^ getNbit(chip->polarity_dup, 7);
    chip->buffer<<=1;

    if (getNbit(chip->config_dup, 6) == 1)
      chip->buffer |= pin_read(chip->PIN_P6)
        ^ getNbit(chip->polarity_dup, 6);
    chip->buffer<<=1;

    if (getNbit(chip->config_dup, 5) == 1)
      chip->buffer |= pin_read(chip->PIN_P5)
        ^ getNbit(chip->polarity_dup, 5);
    chip->buffer<<=1;

    if (getNbit(chip->config_dup, 4) == 1)
      chip->buffer |= pin_read(chip->PIN_P4)
        ^ getNbit(chip->polarity_dup, 4);
    chip->buffer<<=1;

    if (getNbit(chip->config_dup, 3) == 1)
      chip->buffer |= pin_read(chip->PIN_P3)
        ^ getNbit(chip->polarity_dup, 3);
    chip->buffer<<=1;

    if (getNbit(chip->config_dup, 2) == 1)
      chip->buffer |= pin_read(chip->PIN_P2)
        ^ getNbit(chip->polarity_dup, 2);
    chip->buffer<<=1;

    if (getNbit(chip->config_dup, 1) == 1)
      chip->buffer |= pin_read(chip->PIN_P1)
        ^ getNbit(chip->polarity_dup, 1);
    chip->buffer<<=1;

    if (getNbit(chip->config_dup, 0) == 1)
      chip->buffer |= pin_read(chip->PIN_P0)
        ^ getNbit(chip->polarity_dup, 0);

    if (chip->last_input != chip->buffer){
      chip->last_input = chip->buffer;
      pin_write(chip->PIN_INT, HIGH);
      pin_write(chip->PIN_INT, LOW);
    }

#ifdef DEBUG
    printf("Sent from I2C^0x%02X: ", chip->address);
    printf("0x%02X (", chip->buffer);
    printBits(1, &chip->buffer);
    printf(")\n");
#endif // DEBUG
    chip->command = 0xFF;
    return chip->buffer;
  }
  return 0;
}

static void update_state(chip_state_t *chip)
{
  if (chip->command == 0xFF){
    // Записываем команду
    chip->command = chip->buffer;
  } else {
    // Записываем данные
    switch (chip->command) {
      case PCA9538_IN_REG: // 0x00
        chip->command = 0xFF;
        break;

      case PCA9538_OUT_REG: // 0x01
        if (getNbit(chip->config_dup, 0) == 0)
          pin_write(chip->PIN_P0, (getNbit(chip->polarity_dup, 0)
            ^ getNbit(chip->buffer, 0)) ? HIGH : LOW);

        if (getNbit(chip->config_dup, 1) == 0)
          pin_write(chip->PIN_P1, (getNbit(chip->polarity_dup, 1) 
            ^ getNbit(chip->buffer, 1)) ? HIGH : LOW);

        if (getNbit(chip->config_dup, 2) == 0)
          pin_write(chip->PIN_P2, (getNbit(chip->polarity_dup, 2) 
            ^ getNbit(chip->buffer, 2)) ? HIGH : LOW);

        if (getNbit(chip->config_dup, 3) == 0)       
          pin_write(chip->PIN_P3, (getNbit(chip->polarity_dup, 3) 
            ^ getNbit(chip->buffer, 3)) ? HIGH : LOW);

        if (getNbit(chip->config_dup, 4) == 0)
          pin_write(chip->PIN_P4, (getNbit(chip->polarity_dup, 4) 
            ^ getNbit(chip->buffer, 4)) ? HIGH : LOW);

        if (getNbit(chip->config_dup, 5) == 0)
          pin_write(chip->PIN_P5, (getNbit(chip->polarity_dup, 5) 
            ^ getNbit(chip->buffer, 5)) ? HIGH : LOW);

        if (getNbit(chip->config_dup, 6) == 0)
          pin_write(chip->PIN_P6, (getNbit(chip->polarity_dup, 6) 
            ^ getNbit(chip->buffer, 6)) ? HIGH : LOW);

        if (getNbit(chip->config_dup, 7) == 0)
          pin_write(chip->PIN_P7, (getNbit(chip->polarity_dup, 7) 
            ^ getNbit(chip->buffer, 7)) ? HIGH : LOW);

        chip->command = 0xFF;
        break;

      case PCA9538_POL_REG: // 0x02
        chip->polarity_dup = chip->buffer;
        chip->command = 0xFF;
        break;

      case PCA9538_CONF_REG: // 0x03
        pin_mode(chip->PIN_P0, getNbit(chip->buffer, 0) 
          ? INPUT_PULLDOWN : OUTPUT); 
        pin_mode(chip->PIN_P1, getNbit(chip->buffer, 1) 
          ? INPUT_PULLDOWN : OUTPUT); 
        pin_mode(chip->PIN_P2, getNbit(chip->buffer, 2) 
          ? INPUT_PULLDOWN : OUTPUT); 
        pin_mode(chip->PIN_P3, getNbit(chip->buffer, 3) 
          ? INPUT_PULLDOWN : OUTPUT); 
        pin_mode(chip->PIN_P4, getNbit(chip->buffer, 4) 
          ? INPUT_PULLDOWN : OUTPUT); 
        pin_mode(chip->PIN_P5, getNbit(chip->buffer, 5) 
          ? INPUT_PULLDOWN : OUTPUT); 
        pin_mode(chip->PIN_P6, getNbit(chip->buffer, 6) 
          ? INPUT_PULLDOWN : OUTPUT); 
        pin_mode(chip->PIN_P7, getNbit(chip->buffer, 7) 
          ? INPUT_PULLDOWN : OUTPUT); 

        chip->config_dup = chip->buffer;
        chip->command = 0xFF;
        break;

      default:
        printf("PCA9538: Error! Wrong command.\n");
    }
  }
}

bool on_i2c_write(void *user_data, uint8_t data) {
  chip_state_t *chip = user_data;
  if (CHIP_ON) {
#ifdef DEBUG
    printf("Sent to I2C^0x%02X: ", chip->address);
    printf("0x%02X (", data);
    printBits(1, &data);
    printf(")\n");
#endif // DEBUG
    chip->buffer = data;
    update_state(chip);
    
    return true; // ACK
  }
  return false; // NACK
}

void on_i2c_disconnect(void *user_data) {
  // Do nothing
}
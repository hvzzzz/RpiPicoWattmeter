#include "ina219.h"

#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "stdio.h"

ina219::ina219(uint8_t addr, i2c_inst_t *i2c) {
  ina219_i2caddr = addr;
  ina219_i2cChan = i2c;
  ina219_currentDivider_mA = 0;
  ina219_powerMultiplier_mW = 0;
  ina219_shuntVoltageMultiplier_mV = 0;
  ina219_busVoltageMultiplier_V = 0;
}
ina219::~ina219() {}

void ina219::setCalibration(bool triggered) {

  // Calibration value
  // uint16_t ina219_calValue = 10240;
  uint16_t ina219_calValue = 8192;
  wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue);

  // Configuration register
  if (triggered) {
    // uint16_t config = 0x0000 | 0x1000 | 0x0480 | 0x0070 | 0x0001; // 25Hz
    //  uint16_t config = 0x0000 | 0x1000 | 0x0480 | 0x0050 | 0x0001; // 250Hz
    //  uint16_t config = 0x0000 | 0x1000 | 0x0480 | 0x0068 | 0x0001; // 50Hz
    uint16_t config = 0x0000 | 0x1000 | 0x0480 | 0x0050 | 0x0001; // 300Hz
    // uint16_t config = 0x0000 | 0x1000 | 0x0480 | 0x0048 | 0x0001; // 500Hz
    // uint16_t config = 0x0000 | 0x1000 | 0x0480 | 0x0018 | 0x0001; // 1kHz
    // uint16_t config = 0x0000 | 0x1000 | 0x0480 | 0x0060 | 0x0001; // 100Hz
    // uint16_t config = 0x0000 | 0x1000 | 0x0780 | 0x0058 | 0x0001; //200Hz
    // uint16_t config = 0x0000 | 0x0000 | 0x0780 | 0x0058 | 0x0001;
    wireWriteRegister(INA219_REG_CONFIG, config);

  } else {
    uint16_t config = 0x2000 | 0x1000 | 0x0780 | 0x01F8 | 0x0007;
    wireWriteRegister(INA219_REG_CONFIG, config);
  }

  i2c_write_blocking(ina219_i2cChan, INA219_ADDRESS << 1, NULL, 0,
                     true); // Start condition

  // Write high-speed master code (00001XXX)
  i2c_write_blocking(ina219_i2cChan, 0b00001000, NULL, 0, true);

  // Repeated start condition
  i2c_write_blocking(ina219_i2cChan, INA219_ADDRESS << 1, NULL, 0,
                     true); // Repeated start

  // Multipliers
  // ina219_currentDivider_mA = 25;
  ina219_currentDivider_mA = 20;
  ina219_powerMultiplier_mW = 0.8f;
  ina219_shuntVoltageMultiplier_mV = 0.01;
  ina219_busVoltageMultiplier_V = 0.003986;
}

int16_t ina219::getBusVoltage_raw() {
  return wireReadRegister(INA219_REG_BUSVOLTAGE) >> 3;
}

int16_t ina219::getShuntVoltage_raw() {
  return wireReadRegister(INA219_REG_SHUNTVOLTAGE);
}

int16_t ina219::getCurrent_raw() {
  return wireReadRegister(INA219_REG_CURRENT);
}

int16_t ina219::getPower_raw() { return wireReadRegister(INA219_REG_POWER); }

float ina219::getShuntVoltage_mV() {
  return getShuntVoltage_raw() * ina219_shuntVoltageMultiplier_mV;
}

float ina219::getBusVoltage_V() {
  return getBusVoltage_raw() * ina219_busVoltageMultiplier_V;
}

float ina219::getCurrent_mA() {
  return getCurrent_raw() / ina219_currentDivider_mA;
}

float ina219::getPower_mW() {
  return getPower_raw() * ina219_powerMultiplier_mW;
}

void ina219::triggerMeasurement() {
  uint16_t val = wireReadRegister(INA219_REG_CONFIG);
  wireWriteRegister(INA219_REG_CONFIG, val);
  uint16_t convReady = 0x0000;
  while (!convReady) {
    convReady = ((wireReadRegister(INA219_REG_BUSVOLTAGE)) &
                 0x0002); // checks if sampling is completed
  }
}

uint64_t ina219::triggerMeasurement(uint64_t call_timestamp) {
  uint16_t val = wireReadRegister(INA219_REG_CONFIG);
  wireWriteRegister(INA219_REG_CONFIG, val);
  uint64_t convertion_timestamp = time_us_64();
  uint16_t convReady = 0x0000;
  while (!convReady) {
    convReady = ((wireReadRegister(INA219_REG_BUSVOLTAGE)) &
                 0x0002); // checks if sampling is completed
  }
  return convertion_timestamp;
}

void ina219::wireWriteRegister(uint8_t reg, uint16_t value) {
  uint8_t data[] = {reg, (value >> 8) & 0xFF, value & 0xFF};
  i2c_write_blocking(ina219_i2cChan, ina219_i2caddr, data, sizeof(data), false);
}

uint16_t ina219::wireReadRegister(uint8_t reg) {
  uint16_t value = 0;
  i2c_write_blocking(ina219_i2cChan, ina219_i2caddr, &reg, 1, true);
  i2c_read_blocking(ina219_i2cChan, ina219_i2caddr,
                    reinterpret_cast<uint8_t *>(&value), sizeof(value), false);
  return (value >> 8) | ((value & 0xFF) << 8); // Swap bytes for little-endian
}

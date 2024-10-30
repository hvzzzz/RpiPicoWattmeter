#ifndef INA219_H_
#define INA219_H_

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define INA219_ADDRESS 0x40
#define INA219_REG_BUSVOLTAGE 0x02
#define INA219_REG_SHUNTVOLTAGE 0x01
#define INA219_REG_CURRENT 0x04
#define INA219_REG_POWER 0x03
#define INA219_REG_CALIBRATION 0x05
#define INA219_REG_CONFIG 0x00

#define I2C_CHAN i2c0

class ina219 {
public:
  ina219(uint8_t addr = INA219_ADDRESS, i2c_inst_t *i2c = I2C_CHAN);

  ~ina219();

  void setCalibration(bool triggered = false);
  int16_t getBusVoltage_raw();
  int16_t getShuntVoltage_raw();
  int16_t getCurrent_raw();
  int16_t getPower_raw();
  float getShuntVoltage_mV();
  float getBusVoltage_V();
  float getCurrent_mA();
  float getPower_mW();
  void triggerMeasurement();
  uint64_t triggerMeasurement(uint64_t call_timestamp);
  void wireWriteRegister(uint8_t reg, uint16_t value);
  uint16_t wireReadRegister(uint8_t reg);

private:
  uint8_t ina219_i2caddr;
  float ina219_currentDivider_mA;
  float ina219_powerMultiplier_mW;
  float ina219_shuntVoltageMultiplier_mV;
  float ina219_busVoltageMultiplier_V;
  i2c_inst_t *ina219_i2cChan;
};

#endif

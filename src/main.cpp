#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"

#include "ina219.h"
#include "pico/stdlib.h"

#include "math.h"
#include "stdio.h"

#define BAUDRATE 2560000
#define I2C_SDA_PIN 16
#define I2C_SCL_PIN 17

#define ADC_CHAN 0
#define ADC_PIN 26
#define OFFSET 4.36
#define GAIN_AMP 2.99
#define GAIN_SUBS 1.107
#define GAIN_VOLT_DIVIDER 0.31577256020406114

#define m 9.33613445
#define b 13.25273109

#define GPIO_ON 1
#define GPIO_OFF 0

i2c_inst_t *i2c = i2c0;

ina219 INA219(0x40, i2c);
uint16_t adcData;
// const float adc_conversion_factor = 3.251f / ((1 << 8) - 1);
const float adc_conversion_factor = 0.012603482716692722;
// set this to determine sample rate
// 0     = 500,000 Hz
// 960   = 50,000 Hz
// 9600  = 5,000 Hz
#define CLOCK_DIV 1
// #define NSAMP 400 //200Hz
// #define NSAMP 8 //1kHz
// #define NSAMP 128 // 500Hz
#define NSAMP 128 // 333Hz
// #define NSAMP 800 // 50Hz
// #define NSAMP 400 // 250Hz
// #define NSAMP 2048 // 25Hz
dma_channel_config cfg;
uint dma_chan;
float freqs[NSAMP];

void adc_dma_sample(uint16_t *capture_buf) {
  adc_fifo_drain();
  adc_run(false);

  dma_channel_configure(dma_chan, &cfg,
                        capture_buf,   // dst
                        &adc_hw->fifo, // src
                        NSAMP,         // transfer count
                        true           // start immediately
  );
  adc_run(true);
  dma_channel_wait_for_finish_blocking(dma_chan);
}
void init_adc_dma() {
  adc_gpio_init(ADC_PIN);
  adc_init();
  adc_select_input(ADC_CHAN);
  gpio_set_dir(23, GPIO_OUT);
  gpio_put(23, GPIO_ON);

  adc_fifo_setup(
      true,  // Write each completed conversion to the sample FIFO
      true,  // Enable DMA data request (DREQ)
      1,     // DREQ (and IRQ) asserted when at least 1 sample present
      false, // We won't see the ERR bit because of 8 bit reads; disable.
      true   // Shift each sample to 8 bits when pushing to FIFO
  );

  // set sample rate
  adc_set_clkdiv(CLOCK_DIV);

  sleep_ms(1000);
  // Set up the DMA to start transferring data as soon as it appears in FIFO
  uint dma_chan = dma_claim_unused_channel(true);
  cfg = dma_channel_get_default_config(dma_chan);

  // Reading from constant address, writing to incrementing byte addresses
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);

  // Pace transfers based on availability of ADC samples
  channel_config_set_dreq(&cfg, DREQ_ADC);
}

void init_ina219() {

  i2c_init(i2c, BAUDRATE);

  gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA_PIN);
  gpio_pull_up(I2C_SCL_PIN);

  INA219.setCalibration(true);
}

float adc2busVoltageOpamp(double adc_reading) {
  float adcVoltage = adc_reading * adc_conversion_factor;

  float busVoltage = (adcVoltage + b) / (m * GAIN_VOLT_DIVIDER);

  return busVoltage;
}

bool repeating_timer_callback(struct repeating_timer *t) {

  uint64_t start_time_us = time_us_64();

  uint16_t cap_buf[NSAMP];
  INA219.triggerMeasurement();
  // uint64_t current_time_us_after_ina219 =
  // INA219.triggerMeasurement(start_time_us);

  uint64_t current_time_us_after_ina219 = time_us_64();

  adc_dma_sample(cap_buf);
  double adcData = 0.0;
  for (int i = 0; i < NSAMP; i++) {
    adcData += cap_buf[i];
  }

  adcData = adcData / NSAMP;
  adcData = round(adcData);

  uint64_t current_time_us_after_adc = time_us_64();

  float ina219_convertion_time = current_time_us_after_ina219 - start_time_us;
  float current = INA219.getCurrent_mA();
  float voltage = adc2busVoltageOpamp(adcData);
  // float Voltage = INA219.getBusVoltage_V();
  // float mean_voltage = (Voltage + voltage) / 2;
  float power = current * voltage;

  float adc_convertion_time =
      current_time_us_after_adc - current_time_us_after_ina219;

  printf("%lld,%.1f,%.4f,%.4f\n", current_time_us_after_ina219, current,
         voltage, power);
  //   printf("%.4f\n", current);
  // printf("%.3f,%.4f,%.4f,%.0f,%.0f\n", current, voltage, power,
  //       ina219_convertion_time, adc_convertion_time);
  //   printf("%.4f %.5f\n",voltage, current / 100);

  return true;
}

int main() {
  stdio_init_all();
  init_ina219();

  init_adc_dma();

  struct repeating_timer timer;
  add_repeating_timer_us(-3000, repeating_timer_callback, NULL, &timer);

  while (true) {
  }

  return 0;
}

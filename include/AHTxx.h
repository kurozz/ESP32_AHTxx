/***************************************************************************************************/
/*
   ESP-IDF port of AHTxx library for Aosong ASAIR AHT10/AHT15/AHT20/AHT21/AHT25/AM2301B/AM2311B.

   Adapted by : Kuro
   Source code: https://github.com/kurozz/ESP32_AHTxx

   Forked from https://github.com/enjoyneering/AHTxx written by : enjoyneering

   Aosong ASAIR AHT1x/AHT2x features:
   - AHT1x +1.8v..+3.6v, AHT2x +2.2v..+5.5v
   - AHT1x 0.25uA..320uA, AHT2x 0.25uA..980uA
   - temperature range -40C..+85C
   - humidity range 0%..100%
   - typical accuracy T +-0.3C, RH +-2%
   - typical resolution T 0.01C, RH 0.024%
   - normal operating range T -20C..+60C, RH 10%..80%
   - maximum operating rage T -40C..+80C, RH 0%..100%
   - response time 8..30sec*
   - I2C bus speed 100KHz..400KHz, 10KHz recommended minimum
     *measurement with high frequency leads to heating of the
      sensor, interval must be > 1 second to keep self-heating below 0.1C

   GNU GPL license
   see link for details - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/

#ifndef AHTXX_h
#define AHTXX_h

#include "i2c_bus.h"

/* list of I2C addresses */
#define AHTXX_ADDRESS_X38 0x38 // AHT15/AHT20/AHT21/AHT25 I2C address, AHT10 I2C address if address pin to GND
#define AHT10_ADDRESS_X39 0x39 // AHT10 I2C address, if address pin to Vcc

/* list of command registers */
#define AHT1X_INIT_REG 0xE1              // initialization register, for AHT1x only
#define AHT2X_INIT_REG 0xBE              // initialization register, for AHT2x only
#define AHTXX_STATUS_REG 0x71            // read status byte register
#define AHTXX_START_MEASUREMENT_REG 0xAC // start measurement register
#define AHTXX_SOFT_RESET_REG 0xBA        // soft reset register

/* calibration register controls */
#define AHT1X_INIT_CTRL_NORMAL_MODE 0x00 // normal mode on/off       bit[6:5], for AHT1x only
#define AHT1X_INIT_CTRL_CYCLE_MODE 0x20  // cycle mode on/off        bit[6:5], for AHT1x only
#define AHT1X_INIT_CTRL_CMD_MODE 0x40    // command mode  on/off     bit[6:5], for AHT1x only
#define AHTXX_INIT_CTRL_CAL_ON 0x08      // calibration coeff on/off bit[3]
#define AHTXX_INIT_CTRL_NOP 0x00         // NOP control, send after any "AHT1X_INIT_CTRL..."

/* status byte register controls */
#define AHTXX_STATUS_CTRL_BUSY 0x80        // busy                      bit[7]
#define AHT1X_STATUS_CTRL_NORMAL_MODE 0x00 // normal mode status        bit[6:5], for AHT1x only
#define AHT1X_STATUS_CTRL_CYCLE_MODE 0x20  // cycle mode status         bit[6:5], for AHT1x only
#define AHT1X_STATUS_CTRL_CMD_MODE 0x40    // command mode status       bit[6:5], for AHT1x only
#define AHTXX_STATUS_CTRL_CRC 0x10         // CRC8 status               bit[4], no info in datasheet
#define AHTXX_STATUS_CTRL_CAL_ON 0x08      // calibration coeff status  bit[3]
#define AHTXX_STATUS_CTRL_FIFO_ON 0x04     // FIFO on status            bit[2], no info in datasheet
#define AHTXX_STATUS_CTRL_FIFO_FULL 0x02   // FIFO full status          bit[1], no info in datasheet
#define AHTXX_STATUS_CTRL_FIFO_EMPTY 0x02  // FIFO empty status         bit[1], no info in datasheet

/* measurement register controls */
#define AHTXX_START_MEASUREMENT_CTRL 0x33     // measurement controls, suspect this is temperature & humidity DAC resolution
#define AHTXX_START_MEASUREMENT_CTRL_NOP 0x00 // NOP control, send after any "AHTXX_START_MEASUREMENT_CTRL..."

/* sensor delays */
#define AHTXX_CMD_DELAY 10         // delay between commands, in milliseconds
#define AHTXX_MEASUREMENT_DELAY 80 // wait for measurement to complete, in milliseconds
#define AHT1X_POWER_ON_DELAY 40    // wait for AHT1x to initialize after power-on, in milliseconds
#define AHT2X_POWER_ON_DELAY 100   // wait for AHT2x to initialize after power-on, in milliseconds
#define AHTXX_SOFT_RESET_DELAY 20  // less than 20 milliseconds

/* misc */
#define AHTXX_I2C_SPEED_HZ 100000   // sensor I2C speed 100KHz..400KHz, in Hz
#define AHTXX_I2C_STRETCH_USEC 1000 // I2C stretch time, in usec
#define AHTXX_FORCE_READ_DATA true  // force to read data via I2C
#define AHTXX_USE_READ_DATA false   // force to use data from previous read

#define AHTXX_NO_ERROR 0x00   // success, no errors
#define AHTXX_BUSY_ERROR 0x01 // sensor is busy
#define AHTXX_ACK_ERROR 0x02  // sensor didn't return ACK (not connected, broken, long wires (reduce speed), bus locked by slave (increase stretch limit))
#define AHTXX_DATA_ERROR 0x03 // received data smaller than expected
#define AHTXX_CRC8_ERROR 0x04 // computed CRC8 not match received CRC8, for AHT2x only
#define AHTXX_ERROR 0xFF      // other errors

typedef enum : uint8_t
{
    AHT1x_SENSOR = 0x00,
    AHT2x_SENSOR = 0x01,
} AHTXX_I2C_SENSOR;

class AHTxx
{
public:
    AHTxx(uint8_t address = AHTXX_ADDRESS_X38, AHTXX_I2C_SENSOR = AHT1x_SENSOR);

    bool begin(i2c_bus_handle_t i2c_bus);

    float readHumidity(bool readAHT = AHTXX_FORCE_READ_DATA);
    float readTemperature(bool readAHT = AHTXX_FORCE_READ_DATA);
    bool setNormalMode();
    bool setCycleMode();
    bool setComandMode();
    bool softReset();
    uint8_t getStatus();
    void setType(AHTXX_I2C_SENSOR = AHT1x_SENSOR);

private:
    i2c_bus_device_handle_t _aht;

    AHTXX_I2C_SENSOR _sensorType;
    uint8_t _address;
    uint8_t _status;
    uint8_t _rawData[7] = {0, 0, 0, 0, 0, 0, 0}; //{status, RH, RH, RH+T, T, T, CRC}, CRC for AHT2x only

    void _readMeasurement();
    bool _setInitializationRegister(uint8_t value);
    uint8_t _readStatusRegister();
    uint8_t _getCalibration();
    uint8_t _getBusy(bool readAHT = AHTXX_FORCE_READ_DATA);
    bool _checkCRC8();
};

#endif

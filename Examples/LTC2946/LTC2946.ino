/*!
 *  LTC2946 Arduino Example
 *
 *  LTC2946: 12-Bit Wide Range Power, Charge and Energy Monitor
 *
 *  vbshightime (ThingHz).
 *  https://github.com/ThingHz
 *
 */

#include <Arduino.h>
#include "LTC2946.h"
#include <Wire.h>
#define I2C_ADDRESS 0x67


const float resistor = .02;         //!< resistor value on demo board
const float CLK_FREQ = 4E6;         //!< CLK Frequency

LTC2946    ltc(&Wire);
// LSB Weights
const float LTC2946_ADIN_lsb = 5.001221E-04;                      //!< Typical ADIN lsb weight in volts
const float LTC2946_DELTA_SENSE_lsb = 2.5006105E-05;              //!< Typical Delta lsb weight in volts
const float LTC2946_VIN_lsb = 2.5006105E-02;                      //!< Typical VIN lsb weight in volts
const float LTC2946_Power_lsb = 6.25305E-07;                      //!< Typical POWER lsb weight in V^2 VIN_lsb * DELTA_SENSE_lsb
const float LTC2946_ADIN_DELTA_SENSE_lsb = 1.25061E-08;           //!< Typical sense lsb weight in V^2  *ADIN_lsb * DELTA_SENSE_lsb
const float LTC2946_INTERNAL_TIME_lsb = 4101.00/250000.00;        //!< Internal TimeBase lsb. Use LTC2946_TIME_lsb if an external CLK is used. See Settings menu for how to calculate Time LSB.


static uint8_t CTRLA = LTC2946_CHANNEL_CONFIG_V_C_3|LTC2946_SENSE_PLUS|LTC2946_OFFSET_CAL_EVERY|LTC2946_ADIN_GND;  //! Set Control A register to default value.
static uint8_t VOLTAGE_SEL = LTC2946_SENSE_PLUS;                                                //! Set Voltage selection to default value.
int8_t ack = 0;

//! Initialize Linduino
void setup()
{
  Serial.begin(115200);
  ltc.init(I2C_ADDRESS);
  uint8_t LTC2946_mode;

  LTC2946_mode = CTRLA;                                                         //! Set the configuration of the CTRLA Register.
  Serial.println();
  ack |= ltc.LTC2946_write(LTC2946_CTRLA_REG, LTC2946_mode);   //! Sets the LTC2946 to continuous mode

}

//! Repeats Linduino loop
void loop()
{   
    uint32_t power_code, max_power_code, min_power_code;
    ack |= ltc.LTC2946_read_24_bits(LTC2946_POWER_MSB2_REG, &power_code);                 
    ack |= ltc.LTC2946_read_24_bits(LTC2946_MAX_POWER_MSB2_REG, &max_power_code);        
    ack |= ltc.LTC2946_read_24_bits(LTC2946_MIN_POWER_MSB2_REG, &min_power_code);       

    float power, max_power, min_power;  // Store power results
    power = ltc.LTC2946_code_to_power(power_code, resistor, LTC2946_Power_lsb);                                
    max_power = ltc.LTC2946_code_to_power(max_power_code, resistor, LTC2946_Power_lsb);                       
    min_power = ltc.LTC2946_code_to_power(min_power_code, resistor, LTC2946_Power_lsb);                       

    Serial.print(F("****Power: "));
    Serial.print(power, 4);
    Serial.print(F(" W\n"));

    Serial.print(F("Max Power: "));
    Serial.print(max_power, 4);
    Serial.print(F(" W\n"));

    Serial.print(F("Min Power: "));
    Serial.print(min_power, 4);
    Serial.print(F(" W\n"));
    uint16_t current_code, max_current_code, min_current_code;
    ack |= ltc.LTC2946_read_12_bits(LTC2946_DELTA_SENSE_MSB_REG, &current_code);
    ack |= ltc.LTC2946_read_12_bits(LTC2946_MAX_DELTA_SENSE_MSB_REG, &max_current_code);
    ack |= ltc.LTC2946_read_12_bits(LTC2946_MIN_DELTA_SENSE_MSB_REG, &min_current_code);

    float current, max_current, min_current;
    current = ltc.LTC2946_code_to_current(current_code, resistor, LTC2946_DELTA_SENSE_lsb);
    max_current = ltc.LTC2946_code_to_current(max_current_code, resistor, LTC2946_DELTA_SENSE_lsb);
    min_current = ltc.LTC2946_code_to_current(min_current_code, resistor, LTC2946_DELTA_SENSE_lsb);

    Serial.print(F("\n****Current: "));
    Serial.print(current, 4);
    Serial.print(F(" A\n"));

    Serial.print(F("Max Current: "));
    Serial.print(max_current, 4);
    Serial.print(F(" A\n"));

    Serial.print(F("Min Current: "));
    Serial.print(min_current, 4);
    Serial.print(F(" A\n"));

    uint16_t VIN_code, max_VIN_code, min_VIN_code;
    ack |= ltc.LTC2946_read_12_bits(LTC2946_VIN_MSB_REG, &VIN_code);
    ack |= ltc.LTC2946_read_12_bits(LTC2946_MAX_VIN_MSB_REG, &max_VIN_code);
    ack |= ltc.LTC2946_read_12_bits(LTC2946_MIN_VIN_MSB_REG, &min_VIN_code);

    float VIN, max_VIN, min_VIN;
    VIN = ltc.LTC2946_VIN_code_to_voltage(VIN_code , LTC2946_VIN_lsb);
    max_VIN = ltc.LTC2946_VIN_code_to_voltage(max_VIN_code, LTC2946_VIN_lsb);
    min_VIN = ltc.LTC2946_VIN_code_to_voltage(min_VIN_code, LTC2946_VIN_lsb);

    Serial.print(F("\n****VIN: "));
    Serial.print(VIN, 4);
    Serial.print(F(" V\n"));

    Serial.print(F("Max VIN: "));
    Serial.print(max_VIN, 4);
    Serial.print(F(" V\n"));

    Serial.print(F("Min VIN: "));
    Serial.print(min_VIN, 4);
    Serial.print(F(" V\n"));
    delay(2000);

}



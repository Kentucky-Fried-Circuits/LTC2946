/*!
 *  @file LTC2946.h
 *
 *  @mainpage LTC2946: 12-Bit Wide Range Power, Charge and Energy Monitor
 *
 *  @section author Author
 *
 *  vbshightime (ThingHz).
 *  https://github.com/ThingHz
 *
 *
 */

#ifndef LTC2946_H
#define LTC2946_H

#include <Arduino.h>
#include "Wire.h"


#define LTC2946_I2C_MASS_WRITE      0xCC
#define LTC2946_I2C_ALERT_RESPONSE  0x19


// Registers

#define LTC2946_CTRLA_REG                           0x00
#define LTC2946_CTRLB_REG                           0x01
#define LTC2946_ALERT1_REG                          0x02
#define LTC2946_STATUS1_REG                         0x03
#define LTC2946_FAULT1_REG                          0x04

#define LTC2946_POWER_MSB2_REG                      0x05
#define LTC2946_POWER_MSB1_REG                      0x06
#define LTC2946_POWER_LSB_REG                       0x07
#define LTC2946_MAX_POWER_MSB2_REG                  0x08
#define LTC2946_MAX_POWER_MSB1_REG                  0x09
#define LTC2946_MAX_POWER_LSB_REG                   0x0A
#define LTC2946_MIN_POWER_MSB2_REG                  0x0B
#define LTC2946_MIN_POWER_MSB1_REG                  0x0C
#define LTC2946_MIN_POWER_LSB_REG                   0x0D
#define LTC2946_MAX_POWER_THRESHOLD_MSB2_REG        0x0E
#define LTC2946_MAX_POWER_THRESHOLD_MSB1_REG        0x0F
#define LTC2946_MAX_POWER_THRESHOLD_LSB_REG         0x10
#define LTC2946_MIN_POWER_THRESHOLD_MSB2_REG        0x11
#define LTC2946_MIN_POWER_THRESHOLD_MSB1_REG        0x12
#define LTC2946_MIN_POWER_THRESHOLD_LSB_REG         0x13

#define LTC2946_DELTA_SENSE_MSB_REG                 0x14
#define LTC2946_DELTA_SENSE_LSB_REG                 0x15
#define LTC2946_MAX_DELTA_SENSE_MSB_REG             0x16
#define LTC2946_MAX_DELTA_SENSE_LSB_REG             0x17
#define LTC2946_MIN_DELTA_SENSE_MSB_REG             0x18
#define LTC2946_MIN_DELTA_SENSE_LSB_REG             0x19
#define LTC2946_MAX_DELTA_SENSE_THRESHOLD_MSB_REG   0x1A
#define LTC2946_MAX_DELTA_SENSE_THRESHOLD_LSB_REG   0x1B
#define LTC2946_MIN_DELTA_SENSE_THRESHOLD_MSB_REG   0x1C
#define LTC2946_MIN_DELTA_SENSE_THRESHOLD_LSB_REG   0x1D

#define LTC2946_VIN_MSB_REG                         0x1E
#define LTC2946_VIN_LSB_REG                         0x1F
#define LTC2946_MAX_VIN_MSB_REG                     0x20
#define LTC2946_MAX_VIN_LSB_REG                     0x21
#define LTC2946_MIN_VIN_MSB_REG                     0x22
#define LTC2946_MIN_VIN_LSB_REG                     0x23
#define LTC2946_MAX_VIN_THRESHOLD_MSB_REG           0x24
#define LTC2946_MAX_VIN_THRESHOLD_LSB_REG           0x25
#define LTC2946_MIN_VIN_THRESHOLD_MSB_REG           0x26
#define LTC2946_MIN_VIN_THRESHOLD_LSB_REG           0x27

#define LTC2946_ADIN_MSB_REG                        0x28
#define LTC2946_ADIN_LSB_REG_REG                    0x29
#define LTC2946_MAX_ADIN_MSB_REG                    0x2A
#define LTC2946_MAX_ADIN_LSB_REG                    0x2B
#define LTC2946_MIN_ADIN_MSB_REG                    0x2C
#define LTC2946_MIN_ADIN_LSB_REG                    0x2D
#define LTC2946_MAX_ADIN_THRESHOLD_MSB_REG          0x2E
#define LTC2946_MAX_ADIN_THRESHOLD_LSB_REG          0x2F
#define LTC2946_MIN_ADIN_THRESHOLD_MSB_REG          0x30
#define LTC2946_MIN_ADIN_THRESHOLD_LSB_REG          0x31

#define LTC2946_ALERT2_REG                          0x32
#define LTC2946_GPIO_CFG_REG                        0x33

#define LTC2946_TIME_COUNTER_MSB3_REG               0x34
#define LTC2946_TIME_COUNTER_MSB2_REG               0x35
#define LTC2946_TIME_COUNTER_MSB1_REG               0x36
#define LTC2946_TIME_COUNTER_LSB_REG                0x37

#define LTC2946_CHARGE_MSB3_REG                     0x38
#define LTC2946_CHARGE_MSB2_REG                     0x39
#define LTC2946_CHARGE_MSB1_REG                     0x3A
#define LTC2946_CHARGE_LSB_REG                      0x3B

#define LTC2946_ENERGY_MSB3_REG                     0x3C
#define LTC2946_ENERGY_MSB2_REG                     0x3D
#define LTC2946_ENERGY_MSB1_REG                     0x3E
#define LTC2946_ENERGY_LSB_REG                      0x3F

#define LTC2946_STATUS2_REG                         0x40
#define LTC2946_FAULT2_REG                          0x41
#define LTC2946_GPIO3_CTRL_REG                      0x42
#define LTC2946_CLK_DIV_REG                         0x43


// Voltage Selection Command
#define LTC2946_DELTA_SENSE                         0x00
#define LTC2946_VDD                                 0x08
#define LTC2946_ADIN                                0x10
#define LTC2946_SENSE_PLUS                          0x18

// Command Codes

#define LTC2946_ADIN_INTVCC                     0x80
#define LTC2946_ADIN_GND                        0x00

#define LTC2946_OFFSET_CAL_LAST                 0x60
#define LTC2946_OFFSET_CAL_128                  0x40
#define LTC2946_OFFSET_CAL_16                   0x20
#define LTC2946_OFFSET_CAL_EVERY                0x00

#define LTC2946_CHANNEL_CONFIG_SNAPSHOT         0x07
#define LTC2946_CHANNEL_CONFIG_V_C              0x06
#define LTC2946_CHANNEL_CONFIG_A_V_C_1          0x05  
#define LTC2946_CHANNEL_CONFIG_A_V_C_2          0x04  
#define LTC2946_CHANNEL_CONFIG_A_V_C_3          0x03  
#define LTC2946_CHANNEL_CONFIG_V_C_1            0x02  
#define LTC2946_CHANNEL_CONFIG_V_C_2            0x01  
#define LTC2946_CHANNEL_CONFIG_V_C_3            0x00  


#define LTC2946_ENABLE_ALERT_CLEAR              0x80
#define LTC2946_ENABLE_SHUTDOWN                 0x40
#define LTC2946_ENABLE_CLEARED_ON_READ          0x20   
#define LTC2946_ENABLE_STUCK_BUS_RECOVER        0x10

#define LTC2946_DISABLE_ALERT_CLEAR             0x7F
#define LTC2946_DISABLE_SHUTDOWN                0xBF
#define LTC2946_DISABLE_CLEARED_ON_READ         0xDF
#define LTC2946_DISABLE_STUCK_BUS_RECOVER       0xEF              

#define LTC2946_ACC_PIN_CONTROL                 0x08
#define LTC2946_DISABLE_ACC                     0x04
#define LTC2946_ENABLE_ACC                      0x00

#define LTC2946_RESET_ALL                       0x03
#define LTC2946_RESET_ACC                       0x02
#define LTC2946_ENABLE_AUTO_RESET               0x01
#define LTC2946_DISABLE_AUTO_RESET              0x00


#define LTC2946_MAX_POWER_MSB2_RESET            0x00
#define LTC2946_MIN_POWER_MSB2_RESET            0xFF
#define LTC2946_MAX_DELTA_SENSE_MSB_RESET       0x00
#define LTC2946_MIN_DELTA_SENSE_MSB_RESET       0xFF
#define LTC2946_MAX_VIN_MSB_RESET               0x00
#define LTC2946_MIN_VIN_MSB_RESET               0xFF
#define LTC2946_MAX_ADIN_MSB_RESET              0x00
#define LTC2946_MIN_ADIN_MSB_RESET              0xFF

#define LTC2946_ENABLE_MAX_POWER_ALERT          0x80
#define LTC2946_ENABLE_MIN_POWER_ALERT          0x40
#define LTC2946_DISABLE_MAX_POWER_ALERT         0x7F
#define LTC2946_DISABLE_MIN_POWER_ALERT         0xBF

#define LTC2946_ENABLE_MAX_I_SENSE_ALERT        0x20
#define LTC2946_ENABLE_MIN_I_SENSE_ALERT        0x10
#define LTC2946_DISABLE_MAX_I_SENSE_ALERT       0xDF
#define LTC2946_DISABLE_MIN_I_SENSE_ALERT       0xEF

#define LTC2946_ENABLE_MAX_VIN_ALERT            0x08
#define LTC2946_ENABLE_MIN_VIN_ALERT            0x04
#define LTC2946_DISABLE_MAX_VIN_ALERT           0xF7
#define LTC2946_DISABLE_MIN_VIN_ALERT           0xFB

#define LTC2946_ENABLE_MAX_ADIN_ALERT           0x02
#define LTC2946_ENABLE_MIN_ADIN_ALERT           0x01
#define LTC2946_DISABLE_MAX_ADIN_ALERT          0xFD
#define LTC2946_DISABLE_MIN_ADIN_ALERT          0xFE

#define LTC2946_ENABLE_ADC_DONE_ALERT           0x80
#define LTC2946_DISABLE_ADC_DONE_ALERT          0x7F

#define LTC2946_ENABLE_GPIO_1_ALERT             0x40
#define LTC2946_DISABLE_GPIO_1_ALERT            0xBF

#define LTC2946_ENABLE_GPIO_2_ALERT             0x20
#define LTC2946_DISABLE_GPIO_2_ALERT            0xDF

#define LTC2946_ENABLE_STUCK_BUS_WAKE_ALERT     0x08
#define LTC2946_DISABLE_STUCK_BUS_WAKE_ALERT    0xF7

#define LTC2946_ENABLE_ENERGY_OVERFLOW_ALERT    0x04
#define LTC2946_DISABLE_ENERGY_OVERFLOW_ALERT   0xFB

#define LTC2946_ENABLE_CHARGE_OVERFLOW_ALERT    0x02
#define LTC2946_DISABLE_CHARGE_OVERFLOW_ALERT   0xFD

#define LTC2946_ENABLE_COUNTER_OVERFLOW_ALERT   0x01
#define LTC2946_DISABLE_COUNTER_OVERFLOW_ALERT  0xFE

#define LTC2946_GPIO1_IN_ACTIVE_HIGH            0xC0
#define LTC2946_GPIO1_IN_ACTIVE_LOW             0x80
#define LTC2946_GPIO1_OUT_HIGH_Z                0x40
#define LTC2946_GPIO1_OUT_LOW                   0x00

#define LTC2946_GPIO2_IN_ACTIVE_HIGH            0x30
#define LTC2946_GPIO2_IN_ACTIVE_LOW             0x20
#define LTC2946_GPIO2_OUT_HIGH_Z                0x10
#define LTC2946_GPIO2_OUT_LOW                   0x12
#define LTC2946_GPIO2_IN_ACC                    0x00


#define LTC2946_GPIO3_IN_ACTIVE_HIGH            0x0C
#define LTC2946_GPIO3_IN_ACTIVE_LOW             0x08
#define LTC2946_GPIO3_OUT_REG_42                0x04
#define LTC2946_GPIO3_OUT_ALERT                 0x00
#define LTC2946_GPIO3_OUT_LOW                   0x40
#define LTC2946_GPIO3_OUT_HIGH_Z                0x00
#define LTC2946_GPIO_ALERT_CLEAR                0x00


// Register Mask Command
#define LTC2946_CTRLA_ADIN_MASK                0x7F
#define LTC2946_CTRLA_OFFSET_MASK              0x9F  
#define LTC2946_CTRLA_VOLTAGE_SEL_MASK         0xE7  
#define LTC2946_CTRLA_CHANNEL_CONFIG_MASK      0xF8  
#define LTC2946_CTRLB_ACC_MASK                 0xF3  
#define LTC2946_CTRLB_RESET_MASK               0xFC  
#define LTC2946_GPIOCFG_GPIO1_MASK             0x3F  
#define LTC2946_GPIOCFG_GPIO2_MASK             0xCF  
#define LTC2946_GPIOCFG_GPIO3_MASK             0xF3
#define LTC2946_GPIOCFG_GPIO2_OUT_MASK         0xFD
#define LTC2946_GPIO3_CTRL_GPIO3_MASK          0xBF

extern TwoWire Wire;

class LTC2946 {
public:
	
/**
   *  Constructor.
   */	
LTC2946(TwoWire *theWire = &Wire);	

 /**
   * Initialises the I2C bus, and assigns the I2C address to us.
   *
   * @param i2caddr   The I2C address to use for the sensor.
   *
   * @return True if initialisation was successful, otherwise False.
   */
bool init(uint8_t i2c_addr);
	
 /**
   * Writes an 8-bit adc_code to LTC2946
   *
   * @param 8 bit command       8 bit value to write to register.
   *
   * @return state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
   */
int8_t LTC2946_write(uint8_t adc_command, //!< The "command byte" for the LTC2946
                     uint8_t code         //!< Value that will be written to the register.
                    );
 /**
   * Reads an 8-bit adc_code from LTC2946
   *
   * @param 8 bit command       value to read from register.
   *
   * @return state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
   */
							
int8_t LTC2946_read(uint8_t adc_command, //!< The "command byte" for the LTC2946
                    uint8_t *adc_code    //!< Value that will be read from the register.
                   );

/**
   * Reads a 12-bit adc_code from LTC2946
   *
   * @param 8 bit command       value to read from register.
   *
   * @return state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
   */

int8_t LTC2946_read_12_bits(uint8_t adc_command, //!< The "command byte" for the LTC2946
                            uint16_t *adc_code   //!< Value that will be read from the register.
                           );

/**
   * Reads a 24-bit adc_code from LTC2946
   *
   * @param 8 bit command       value to read from register.
   *
   * @return state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
   */
int8_t LTC2946_read_24_bits(uint8_t adc_command, //!< The "command byte" for the LTC2946
                            uint32_t *adc_code    //!< Value that will be read from the register.
                           );
/**
   * Calculate the LTC2946 VIN voltage
   *
   * @param The ADC value       VIN lsb weight
   *
   * @return Returns the VIN Voltage in Volts
   */

					   					   
float LTC2946_VIN_code_to_voltage(uint16_t adc_code,        //!< The ADC value
                                  float LTC2946_VIN_lsb     //!< VIN lsb weight
                                 );

/**
   * Calculate the LTC2946 ADIN voltage
   *
   * @param The ADC value       ADIN lsb weight
   *
   * @return Returns the ADIN Voltage in Volts
   */                                 

float LTC2946_ADIN_code_to_voltage(uint16_t adc_code,       //!< The ADC value
                                   float LTC2946_ADIN_lsb   //!< ADIN lsb weight
                                  );

/**
   * Calculate the LTC2946 current with a sense resistor
   *
   * @param The ADC value    The resistor value    Delta sense lsb weight
   *
   * @return The LTC2946 current in Amps
   */ 

float LTC2946_code_to_current(uint16_t adc_code,                //!< The ADC value
                              float resistor,                   //!< The resistor value
                              float LTC2946_DELTA_SENSE_lsb     //!< Delta sense lsb weight
                             );

/**
   * Calculate the LTC2946 power
   *
   * @param The ADC value    The resistor value    Power lsb weight
   *
   * @return The LTC2946 power in Watts
   */ 

float LTC2946_code_to_power(int32_t adc_code,           //!< The ADC value
							float resistor,				//!< The resistor value
                            float LTC2946_Power_lsb);     //!< Power lsb weight


/**
   * Calculate the LTC2946 energy
   *
   * @param The ADC value    The resistor value    Power lsb weight     Time lsb weight 
   *
   * @return The LTC2946 energy in Joules
   */ 

float LTC2946_code_to_energy(int32_t adc_code,                      //!< The ADC value
								 float resistor,					//!< The resistor value
                                 float LTC2946_Power_lsb, 		    //!< Power lsb weight
                                 float LTC2946_TIME_lsb             //!< Time lsb weight                
                                );


/**
   * Calculate the LTC2946 coulombs
   *
   * @param The ADC value    The resistor value    Power lsb weight     Time lsb weight 
   *
   * @return The LTC2946 charge in coulombs
   */ 

float LTC2946_code_to_coulombs(int32_t adc_code,                      //!< The ADC value
                                 float resistor,                      //!< The resistor value
                                 float LTC2946_DELTA_SENSE_lsb,       //!< Delta sense lsb weight
                                 float LTC2946_Time_lsb              //!< Time lsb weight
                                );

/**
   * Calculate the LTC2946 coulombs
   *
   * @param Time adc code    Time lsb weight    
   *
   * @return The internal time base in seconds
   */ 	

float LTC2946_code_to_time(float time_code,							//!< Time adc code
                           float LTC2946_Time_lsb					//!< Time lsb weight
                                 );

TwoWire *_wire; /**< Wire object */

private:
  byte I2C_ADDRESS;	
};

#endif  // LTC2946_H

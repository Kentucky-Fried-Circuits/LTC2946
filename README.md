# LTC2946: 12-Bit Wide Range Power, Charge and Energy Monitor

The LTC®2946 is a rail-to-rail system monitor that measures
current, voltage, power, charge and energy. It features an
operating range of 2.7V to 100V and includes a shunt regulator
for supplies above 100V. The current measurement common mode
range of 0V to 100V is independent of the input supply. 
A 12-bit ADC measures load current, input voltage and an 
auxiliary external voltage. Load current and internally 
calculated power are integrated over an external clock or
crystal or internal oscillator time base for charge and energy.
An accurate time base allows the LTC2946 to provide measurement
accuracy of better than ±0.6% for charge and ±1% for power and
energy. Minimum and maximum values are stored and an overrange
alert with programmable thresholds minimizes the need for software
polling. Data is reported via a standard I2C interface.
Shutdown mode reduces power consumption to 15uA. 


## I2C Address for LTC2946
| LTC2946 I2C Address Assignment    | Value |   AD1    |   AD0    |
| --------------------------------- | ----- | -------- | -------- |
| LTC2946_I2C_ADDRESS               | 0x67  |   High   |   Low    |
| LTC2946_I2C_ADDRESS               | 0x68  |   NC     |   High   |
| LTC2946_I2C_ADDRESS               | 0x69  |   High   |   High   |
| LTC2946_I2C_ADDRESS               | 0x6A  | 	NC     |   NC     |
| LTC2946_I2C_ADDRESS               | 0x6B  | 	NC     |   Low    |
| LTC2946_I2C_ADDRESS               | 0x6C  | 	Low    |   High   |
| LTC2946_I2C_ADDRESS               | 0x6D  | 	High   |   NC     |
| LTC2946_I2C_ADDRESS               | 0x6E  | 	Low    |   NC     |
| LTC2946_I2C_ADDRESS               | 0x6F  | 	Low    |   Low    |
|                                   |       |          |          |
| LTC2946_I2C_MASS_WRITE            | 0xCC  | X        | X        |
| LTC2946_I2C_ALERT_RESPONSE        | 0x19  | X        | X        |

## Register Addresses

| Name                                              | Value |
| :------------------------------------------------ | :---: |
| LTC2946_CTRLA_REG                                 | 0x00  |
| LTC2946_CTRLB_REG                                 | 0x01  |
| LTC2946_ALERT1_REG                                | 0x02  |
| LTC2946_STATUS1_REG                               | 0x03  |
| LTC2946_FAULT1_REG                                | 0x04  |
| LTC2946_POWER_MSB2_REG                            | 0x05  |
| LTC2946_POWER_MSB1_REG                            | 0x06  |
| LTC2946_POWER_LSB_REG                             | 0x07  |
| LTC2946_MAX_POWER_MSB2_REG                        | 0x08  |
| LTC2946_MAX_POWER_MSB1_REG                        | 0x09  |
| LTC2946_MAX_POWER_LSB_REG                         | 0x0A  |
| LTC2946_MIN_POWER_MSB2_REG                        | 0x0B  |
| LTC2946_MIN_POWER_MSB1_REG                        | 0x0C  |
| LTC2946_MIN_POWER_LSB_REG                         | 0x0D  |
| LTC2946_MAX_POWER_THRESHOLD_MSB2_REG              | 0x0E  |
| LTC2946_MAX_POWER_THRESHOLD_MSB1_REG              | 0x0F  |
| LTC2946_MAX_POWER_THRESHOLD_LSB_REG               | 0x10  |
| LTC2946_MIN_POWER_THRESHOLD_MSB2_REG              | 0x11  |
| LTC2946_MIN_POWER_THRESHOLD_MSB1_REG              | 0x12  |
| LTC2946_MIN_POWER_THRESHOLD_LSB_REG               | 0x13  |
| LTC2946_DELTA_SENSE_MSB_REG                       | 0x14  |
| LTC2946_DELTA_SENSE_LSB_REG                       | 0x15  |
| LTC2946_MAX_DELTA_SENSE_MSB_REG                   | 0x16  |
| LTC2946_MAX_DELTA_SENSE_LSB_REG                   | 0x17  |
| LTC2946_MIN_DELTA_SENSE_MSB_REG                   | 0x18  |
| LTC2946_MIN_DELTA_SENSE_LSB_REG                   | 0x19  |
| LTC2946_MAX_DELTA_SENSE_THRESHOLD_MSB_REG         | 0x1A  |
| LTC2946_MAX_DELTA_SENSE_THRESHOLD_LSB_REG         | 0x1B  |
| LTC2946_MIN_DELTA_SENSE_THRESHOLD_MSB_REG         | 0x1C  |
| LTC2946_MIN_DELTA_SENSE_THRESHOLD_LSB_REG         | 0x1D  |
| LTC2946_VIN_MSB_REG                               | 0x1E  |
| LTC2946_VIN_LSB_REG                               | 0x1F  |
| LTC2946_MAX_VIN_MSB_REG                           | 0x20  |
| LTC2946_MAX_VIN_LSB_REG                           | 0x21  |
| LTC2946_MIN_VIN_MSB_REG                           | 0x22  |
| LTC2946_MIN_VIN_LSB_REG                           | 0x23  |
| LTC2946_MAX_VIN_THRESHOLD_MSB_REG                 | 0x24  |
| LTC2946_MAX_VIN_THRESHOLD_LSB_REG                 | 0x25  |
| LTC2946_MIN_VIN_THRESHOLD_MSB_REG                 | 0x26  |
| LTC2946_MIN_VIN_THRESHOLD_LSB_REG                 | 0x27  |
| LTC2946_ADIN_MSB_REG                              | 0x28  |
| LTC2946_ADIN_LSB_REG_REG                          | 0x29  |
| LTC2946_MAX_ADIN_MSB_REG                          | 0x2A  |
| LTC2946_MAX_ADIN_LSB_REG                          | 0x2B  |
| LTC2946_MIN_ADIN_MSB_REG                          | 0x2C  |
| LTC2946_MIN_ADIN_LSB_REG                          | 0x2D  |
| LTC2946_MAX_ADIN_THRESHOLD_MSB_REG                | 0x2E  |
| LTC2946_MAX_ADIN_THRESHOLD_LSB_REG                | 0x2F  |
| LTC2946_MIN_ADIN_THRESHOLD_MSB_REG                | 0x30  |
| LTC2946_MIN_ADIN_THRESHOLD_LSB_REG                | 0x31  |
| LTC2946_ALERT2_REG                                | 0x32  |
| LTC2946_GPIO_CFG_REG                              | 0x33  |
| LTC2946_TIME_COUNTER_MSB3_REG                     | 0x34  |
| LTC2946_TIME_COUNTER_MSB2_REG                     | 0x35  |
| LTC2946_TIME_COUNTER_MSB1_REG                     | 0x36  |
| LTC2946_TIME_COUNTER_LSB_REG                      | 0x37  |
| LTC2946_CHARGE_MSB3_REG                           | 0x38  |
| LTC2946_CHARGE_MSB2_REG                           | 0x39  |
| LTC2946_CHARGE_MSB1_REG                           | 0x3A  |
| LTC2946_CHARGE_LSB_REG                            | 0x3B  |
| LTC2946_ENERGY_MSB3_REG                           | 0x3C  |
| LTC2946_ENERGY_MSB2_REG                           | 0x3D  |
| LTC2946_ENERGY_MSB1_REG                           | 0x3E  |
| LTC2946_ENERGY_LSB_REG                            | 0x3F  |
| LTC2946_STATUS2_REG                               | 0x40  |
| LTC2946_FAULT2_REG                                | 0x41  |
| LTC2946_GPIO3_CTRL_REG                            | 0x42  |
| LTC2946_CLK_DIV_REG                               | 0x43  |


## Voltage Selection Command

| Voltage Selection Command            | Value |
| :------------------------------------| :---: |
| LTC2946_DELTA_SENSE                  | 0x00  |
| LTC2946_VDD                          | 0x08  |
| LTC2946_ADIN                         | 0x10  |
| LTC2946_SENSE_PLUS                   | 0x18  |

## Register Addresses

| Command Codes                                 | Value     |
| :-------------------------------------------- | :-------: |
| LTC2946_ADIN_INTVCC                     		|	0x80	|
| LTC2946_ADIN_GND         		                |	0x00	|
| LTC2946_OFFSET_CAL_LAST			            |   0x60	|
| LTC2946_OFFSET_CAL_128    	                | 	0x40	|
| LTC2946_OFFSET_CAL_16                   		|	0x20	|
| LTC2946_OFFSET_CAL_EVERY                		|	0x00	|
| LTC2946_CHANNEL_CONFIG_SNAPSHOT         		|	0x07	|
| LTC2946_CHANNEL_CONFIG_V_C              		|	0x06	|
| LTC2946_CHANNEL_CONFIG_A_V_C_1          		|	0x05	|
| LTC2946_CHANNEL_CONFIG_A_V_C_2          		|	0x04  	|
| LTC2946_CHANNEL_CONFIG_A_V_C_3          		|	0x03  	|
| LTC2946_CHANNEL_CONFIG_V_C_1          		|	0x02 	|
| LTC2946_CHANNEL_CONFIG_V_C_2          		|	0x01 	|
| LTC2946_CHANNEL_CONFIG_V_C_3          		|	0x00 	|
| LTC2946_ENABLE_ALERT_CLEAR             		|	0x80	|
| LTC2946_ENABLE_SHUTDOWN                		|	0x40    |
| LTC2946_ENABLE_CLEARED_ON_READ         		|	0x20    |
| LTC2946_ENABLE_STUCK_BUS_RECOVER       		|	0x10    |
| LTC2946_DISABLE_ALERT_CLEAR            		|	0x7F    |
| LTC2946_DISABLE_SHUTDOWN               		|	0xBF    |
| LTC2946_DISABLE_CLEARED_ON_READ        		|	0xDF    |
| LTC2946_DISABLE_STUCK_BUS_RECOVER      		|	0xEF    |          
| LTC2946_ACC_PIN_CONTROL                		|	0x08    |
| LTC2946_DISABLE_ACC                    		|	0x04    |
| LTC2946_ENABLE_ACC                     		|	0x00    |
| LTC2946_RESET_ALL                      		|	0x03    |
| LTC2946_RESET_ACC                      		|	0x02    |
| LTC2946_ENABLE_AUTO_RESET              		|	0x01    |
| LTC2946_DISABLE_AUTO_RESET             		|	0x00    |
| LTC2946_MAX_POWER_MSB2_RESET           		|	0x00    |
| LTC2946_MIN_POWER_MSB2_RESET           		|	0xFF    |
| LTC2946_MAX_DELTA_SENSE_MSB_RESET      		|	0x00    |
| LTC2946_MIN_DELTA_SENSE_MSB_RESET      		|	0xFF    |
| LTC2946_MAX_VIN_MSB_RESET              		|	0x00    |
| LTC2946_MIN_VIN_MSB_RESET              		|	0xFF    |
| LTC2946_MAX_ADIN_MSB_RESET             		|	0x00    |
| LTC2946_MIN_ADIN_MSB_RESET             		|	0xFF    |
| LTC2946_ENABLE_MAX_POWER_ALERT         		|	0x80    |
| LTC2946_ENABLE_MIN_POWER_ALERT         		|	0x40    |
| LTC2946_DISABLE_MAX_POWER_ALERT        		|	0x7F    |
| LTC2946_DISABLE_MIN_POWER_ALERT        		|	0xBF    |
| LTC2946_ENABLE_MAX_I_SENSE_ALERT       		|	0x20    |
| LTC2946_ENABLE_MIN_I_SENSE_ALERT       		|	0x10    |
| LTC2946_DISABLE_MAX_I_SENSE_ALERT      		|	0xDF    |
| LTC2946_DISABLE_MIN_I_SENSE_ALERT      		|	0xEF    |
| LTC2946_ENABLE_MAX_VIN_ALERT           		|	0x08    |
| LTC2946_ENABLE_MIN_VIN_ALERT           		|	0x04    |
| LTC2946_DISABLE_MAX_VIN_ALERT          		|	0xF7    |
| LTC2946_DISABLE_MIN_VIN_ALERT          		|	0xFB    |
| LTC2946_ENABLE_MAX_ADIN_ALERT          		|	0x02    |
| LTC2946_ENABLE_MIN_ADIN_ALERT          		|	0x01    |
| LTC2946_DISABLE_MAX_ADIN_ALERT         		|	0xFD    |
| LTC2946_DISABLE_MIN_ADIN_ALERT         		|	0xFE    |
| LTC2946_ENABLE_ADC_DONE_ALERT          		|	0x80    |
| LTC2946_DISABLE_ADC_DONE_ALERT         		|	0x7F    |
| LTC2946_ENABLE_GPIO_1_ALERT            		|	0x40    |
| LTC2946_DISABLE_GPIO_1_ALERT           		|	0xBF    |
| LTC2946_ENABLE_GPIO_2_ALERT            		|	0x20    |
| LTC2946_DISABLE_GPIO_2_ALERT           		|	0xDF    |
| LTC2946_ENABLE_STUCK_BUS_WAKE_ALERT    		|	0x08    |
| LTC2946_DISABLE_STUCK_BUS_WAKE_ALERT   		|	0xF7    |
| LTC2946_ENABLE_ENERGY_OVERFLOW_ALERT   		|	0x04    |
| LTC2946_DISABLE_ENERGY_OVERFLOW_ALERT  		|	0xFB    |
| LTC2946_ENABLE_CHARGE_OVERFLOW_ALERT   		|	0x02    |
| LTC2946_DISABLE_CHARGE_OVERFLOW_ALERT  		|	0xFD    |
| LTC2946_ENABLE_COUNTER_OVERFLOW_ALERT  		|	0x01    |
| LTC2946_DISABLE_COUNTER_OVERFLOW_ALERT 		|	0xFE    |
| LTC2946_GPIO1_IN_ACTIVE_HIGH           		|	0xC0    |
| LTC2946_GPIO1_IN_ACTIVE_LOW            		|	0x80    |
| LTC2946_GPIO1_OUT_HIGH_Z               		|	0x40    |
| LTC2946_GPIO1_OUT_LOW                  		|	0x00    |
| LTC2946_GPIO2_IN_ACTIVE_HIGH           		|	0x30    |
| LTC2946_GPIO2_IN_ACTIVE_LOW            		|	0x20    |
| LTC2946_GPIO2_OUT_HIGH_Z               		|	0x10    |
| LTC2946_GPIO2_OUT_LOW                  		|	0x12    |
| LTC2946_GPIO2_IN_ACC                   		|	0x00    |
| LTC2946_GPIO3_IN_ACTIVE_HIGH           		|	0x18    |
| LTC2946_GPIO3_IN_ACTIVE_LOW            		|	0x10    |
| LTC2946_GPIO3_OUT_REG_42               		|	0x04    |
| LTC2946_GPIO3_OUT_ALERT                		|	0x00    |
| LTC2946_GPIO3_OUT_LOW                  		|	0x40    |
| LTC2946_GPIO3_OUT_HIGH_Z               		|	0x00    |
| LTC2946_GPIO_ALERT_CLEAR               		|	0x00    |

## Register Mask Commands

| Register Mask Command                | Value |
| :------------------------------------| :---: |
| LTC2946_CTRLA_ADIN_MASK              |  0x7F |
| LTC2946_CTRLA_OFFSET_MASK            |  0x9F |
| LTC2946_CTRLA_VOLTAGE_SEL_MASK       |  0xE7 |
| LTC2946_CTRLA_CHANNEL_CONFIG_MASK    |  0xF8 |
| LTC2946_CTRLB_ACC_MASK               |  0xF3 |
| LTC2946_CTRLB_RESET_MASK             |  0xFC |
| LTC2946_GPIOCFG_GPIO1_MASK           |  0x3F |
| LTC2946_GPIOCFG_GPIO2_MASK           |  0xCF |
| LTC2946_GPIOCFG_GPIO3_MASK           |  0xF3 |
| LTC2946_GPIOCFG_GPIO2_OUT_MASK       |  0xFD |
| LTC2946_GPIO3_CTRL_GPIO3_MASK        |  0xBF |
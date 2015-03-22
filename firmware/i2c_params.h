#ifndef i2c_params
#define i2c_params

enum { 
  INST_PASS = 0,
  INST_LED_HIGH = 1,
  INST_LED_LOW = 2,
  INST_LED_PWM = 3,
  INST_GET_LED_STATUS = 4
};

void execute();

extern const uint8_t I2C_INST_ARGS_MAX = 2;
extern const uint8_t I2C_RESP_LEN_MAX = 2;

#define I2C_ADDR            7            
#define TWI_FREQ_SETTING    400000L       // 400KHz for I2C
#define CPU_FREQ            16000000L     // 16MHz

extern const uint8_t supportedI2Ccmd[] = { 0, 1, 2, 3, 4 };

#endif

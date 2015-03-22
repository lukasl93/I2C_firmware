#include <Wire.h>

#define I2C_REQ_DELAY_MS         50  // used for IO reads - from node's memory (fast)
#define I2C_REQ_LONG_DELAY_MS    100  //used for configuration etc.

#define TWI_FREQ_SETTING         400000L
#define CPU_FREQ                 16000000L

enum { 
  INST_PASS = 0,
  INST_LED_HIGH = 1,
  INST_LED_LOW = 2,
  INST_LED_PWM = 3,
  INST_GET_LED_STATUS = 4
};

int i2cSlaveAddr = 7;


void setup(){
  // joining i2c as a master
  //TWBR = ((CPU_FREQ / TWI_FREQ_SETTING) - 16) / 2;
  Wire.begin(); 
  Serial.begin(9600);
  delay(2000);
}

void loop(){
  
  uint8_t cmd[3]; 

  cmd[0] = 2;
  Wire.beginTransmission(i2cSlaveAddr); 
  Wire.write(cmd[0]);  
  Wire.endTransmission();  
  Serial.println(cmd[0], HEX);

  delay(5000);

  cmd[0] = 1;
  Wire.beginTransmission(i2cSlaveAddr); 
  Wire.write(cmd[0]);  
  Wire.endTransmission(); 
  Serial.println(cmd[0], HEX); 

  delay(5000);

  cmd[0] = 3;
  cmd[1] = 0x00;
  cmd[2] = 0x80;
  Wire.beginTransmission(i2cSlaveAddr); 
  Wire.write(cmd, 3);  
  Wire.endTransmission();  
  Serial.print(cmd[0], HEX);
  Serial.print(cmd[1], HEX);
  Serial.println(cmd[2], HEX);

  delay(5000);

  cmd[0] = 3;
  cmd[1] = 0x00;
  cmd[2] = 0xF0;
  Wire.beginTransmission(i2cSlaveAddr); 
  Wire.write(cmd, 3);  
  Wire.endTransmission();  
  Serial.print(cmd[0], HEX);
  Serial.print(cmd[1], HEX);
  Serial.println(cmd[2], HEX);

  delay(5000);

  cmd[0] = 3;
  cmd[1] = 0x00;
  cmd[2] = 0x20;
  Wire.beginTransmission(i2cSlaveAddr); 
  Wire.write(cmd, 3);  
  Wire.endTransmission();  
  Serial.print(cmd[0], HEX);
  Serial.print(cmd[1], HEX);
  Serial.println(cmd[2], HEX);

  delay(5000);

  cmd[0] = 4;
  Wire.beginTransmission(i2cSlaveAddr); 
  Wire.write(cmd[0]);  
  Wire.endTransmission();  
  Serial.println(cmd[0], HEX);

  delay(I2C_REQ_DELAY_MS);


  // master knows slave should return 1 bytes to the INST_GET_SOME_STATUS command
    uint8_t respVal;

  Wire.requestFrom(i2cSlaveAddr, 1);

  uint8_t respIoIndex = 0;

 
      if(Wire.available()){ 
        respVal = (uint8_t)Wire.read();     
      }else{
        Serial.print("noresp");
      }
      Serial.print("Responce: ");
      Serial.println(respVal, HEX);
      delay(5000);
}

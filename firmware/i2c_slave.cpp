#include <Wire.h>
#include "i2c_params.h"


int argsCnt = 0;                        // how many arguments were passed with given command
int requestedCmd = 0;                   // which command was requested (if any)

byte i2cArgs[I2C_INST_ARGS_MAX];         // array to store args received from master
int i2cArgsLen = 0;                     // how many args passed by master to given command

uint8_t i2cResponse[I2C_RESP_LEN_MAX];  // array to store response
int i2cResponseLen = 0;                 // response length

void setup()
{
  // >> starting i2c
  //TWBR = ((CPU_FREQ / TWI_FREQ_SETTING) - 16) / 2;

  Wire.begin(I2C_ADDR);                        // join i2c bus 
  Wire.onRequest(requestEvent);                // register event
  Wire.onReceive(receiveEvent);    
  // << starting i2c
  Serial.begin(9600);
  pinMode(P1_2, OUTPUT);
}

void loop()
{	
	
	switch(requestedCmd){
		case INST_LED_HIGH:
			analogWrite(P1_2, 255);
			requestedCmd = INST_PASS;
			break;
		case INST_LED_LOW:
			analogWrite(P1_2, 0);
			requestedCmd = INST_PASS;
			break;
		case INST_LED_PWM:
			analogWrite(P1_2, i2cArgs[1]);
			requestedCmd = INST_PASS;
			break;	
		case INST_GET_SOME_STATUS:
			i2cResponseLen = 1;
			i2cResponse[i2cResponseLen -1] = 0xFA;
			requestedCmd = INST_PASS;
                        break;
		case INST_PASS:
			break;
		default:
			//command not recognized
			break;		
	}

} 


// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent(){

  Wire.write(i2cResponse, i2cResponseLen);
  i2cResponseLen = 0;
  memset(i2cResponse, 0, I2C_RESP_LEN_MAX);
}


// function that executes when master sends data (begin-end transmission)
// this function is registered as an event, see setup()
void receiveEvent(int nBytesRcvd)
{
  //evt toggle a LED
  
  int cmdRcvd = -1;
  int argIndex = -1; 
  argsCnt = 0;

  if (Wire.available()){
    cmdRcvd = Wire.read();                 // receive first byte - command
    while(Wire.available()){               // receive rest of tramsmission from master - arguments to the command
      if (argIndex < I2C_INST_ARGS_MAX){
        i2cArgs[++argIndex] = Wire.read();
      }
      else{
        ; // too many arguments
      }
      argsCnt = argIndex+1;  
    }
  }
  else{
    // empty request
    return;
  }
  // validating command is supported by slave
  int fcnt = -1;
  for (int i = 0; i < sizeof(supportedI2Ccmd); i++) {
    if (supportedI2Ccmd[i] == cmdRcvd) {
      fcnt = i;
    }
  }

  if (fcnt<0){
    // command not supported
    return;
  }
  requestedCmd = cmdRcvd;
  // the main loop executes this command
}

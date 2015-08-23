 /*
  * NOTE: 100k extrnal pull-ups are needed on SDA/SDC.
  */

#include <msp430.h>
#include <legacymsp430.h>

#include "TI_USCI_I2C_slave.h"
//#include "drive.h"

/* callback for start condition */
void start_cb();

/* callback for bytes received */
void receive_cb(unsigned char receive);

/* callback to transmit bytes */
void transmit_cb(unsigned char volatile *receive);

/* Commands */
#define CMD_PASS  			0x00

#define CMD_TEST_LED_ON				0x01
#define CMD_TEST_LED_OFF			0x02
#define CMD_TEST_MOTORS_ON			0x03
#define CMD_TEST_MOTORS_OFF			0x04

#define CMD_GPIO_DIR  		0x05

#define CMD_GPIO_SET 		0x0A
#define CMD_GPIO_GET  		0x0B

#define CMD_GPIO_SETPWM 	0x10
#define CMD_GPIO_GETPWM 	0x11

#define CMD_GPIO_GETFREQ	0x20

#define CMD_SET_PORT  		0x30
#define CMD_GET_PORT 		0x31

#define CMD_SET_SEL			0x3A
#define CMD_GET_SEL			0x3B

#define CMD_SET_INTERRUPT 	0x50

#define CMD_SPI_WRITE  		0x70
#define CMD_SPI_READ  		0x71

/* Parameters */
#define PAR_UNKNOWN 0xFF

#define PAR_PORT1  	0x01
#define PAR_PORT2  	0x02
#define PAR_PORT3  	0x03

#define PAR_PIN0 	0x00
#define PAR_PIN1 	0x01
#define PAR_PIN2 	0x02
#define PAR_PIN3 	0x03
#define PAR_PIN4 	0x04
#define PAR_PIN5 	0x05
#define PAR_PIN6 	0x06
#define PAR_PIN7 	0x07

#define PAR_DIR_IN 	0x00
#define PAR_DIR_OUT	0x01

#define PAR_STATE_LOW 	0x00
#define PAR_STATE_HIGH 	0x01


/* Responses */
#define RES_ERROR   	0xFF

#define RES_STATE_LOW   0x00
#define RES_STATE_HIGH  0x01
#define RISING		0x01
#define FALLING		0x00

/* command */
unsigned char cmd = CMD_PASS;

/* parameter */
unsigned char par[MAX_PARAMETERS] =
	{PAR_UNKNOWN, PAR_UNKNOWN, PAR_UNKNOWN, PAR_UNKNOWN};

unsigned char parameter_len = 0;

/* response to send out on read req. */
unsigned char res[MAX_RESPONSE_LENGTH] =
	{RES_ERROR, RES_ERROR, RES_ERROR, RES_ERROR};

unsigned char last_byte = 0x00;

unsigned char response_len = 0;

unsigned char flstop=0; //stop flag for motors

//pwm input listener variables
struct TMSTP {
	unsigned int time;
	unsigned char edge;
};
struct TMSTP time_stamp[3];


volatile unsigned char n = 0;
unsigned long duty_cycle = 0;
unsigned long period = 0;
unsigned char percent_on = 0;

unsigned char received_buffer[10];

void drive();
void stop();

void dir_left(unsigned char dir);
void dir_right(unsigned char dir);
void start_left();
void start_right();
void stop_left();
void stop_right();
void mid_speed();
void left_curve();
void right_curve();

void process_cmd(unsigned char cmd, unsigned char par[])
{
	unsigned char i;
    for (i=0; i<MAX_RESPONSE_LENGTH; i++) {
    	res[i] = RES_ERROR;
    }
    response_len = 0;
    unsigned char pin_num = 0x00;
    unsigned int pwm_freq = 0;
    unsigned params_dummy[3];
    switch(cmd) {
    case CMD_TEST_LED_ON:
//    	P2SEL &= ~BIT0;
//    	P2DIR |= BIT0;
//    	P2OUT |= BIT0;
//    	cmd = CMD_PASS;
    	cmd = CMD_GPIO_DIR;
    	par[0] = PAR_PORT2;
    	par[1] = PAR_PIN0;
    	par[2] = PAR_DIR_OUT;
    	process_cmd(cmd, par);
    	cmd = CMD_GPIO_SET;
    	par[0] = PAR_PORT2;
    	par[1] = PAR_PIN0;
    	par[2] = PAR_STATE_HIGH;
    	process_cmd(cmd, par);
    	cmd = CMD_PASS;
        break;
    case CMD_TEST_LED_OFF:
    	P2SEL &= ~BIT0;
    	P2DIR |= BIT0;
    	P2OUT &= ~BIT0;
    	cmd = CMD_PASS;
    	break;
    case CMD_TEST_MOTORS_ON:
    		flstop=0;
    		drive();
            break;
    case CMD_TEST_MOTORS_OFF:
    	stop();
            break;
    case CMD_PASS:
    	cmd = CMD_PASS;
            break;
    case CMD_GPIO_DIR:
    {
    	pin_num = (0x01 << par[1]);
        if(par[0] == PAR_PORT1) {
        	//set bit to 0 for i/o
        	P1SEL &= ~pin_num;
            if (par[2] == PAR_DIR_IN) {
            	//set bit to 0 for input
            	P1DIR &= ~pin_num;
            } else {
            	//set bit to 1 for output
            	P1DIR |= pin_num;
            }
        } else if (par[0] == PAR_PORT2) {
        	P2SEL &= ~pin_num;
            if (par[2] == PAR_DIR_IN) {
            	//set bit to 0 for input
            	P2DIR &= ~pin_num;
            } else {
            	//set bit to 1 for output
            	P2DIR |= pin_num;
            }
        } else if (par[0] == PAR_PORT3) {
        	P3SEL &= ~pin_num;
            if (par[2] == PAR_DIR_IN) {
            	//set bit to 0 for input
            	P3DIR &= ~pin_num;
            } else {
            	//set bit to 1 for output
            	P3DIR |= pin_num;
            }
        } else {
        	//bad port number
        }
        cmd = CMD_PASS;
        break;
    }
    case CMD_GPIO_SET:
    	pin_num = (0x01 << par[1]);
        if(par[0] == PAR_PORT1) {
        	P1SEL &= ~pin_num;
        	if (par[2] == PAR_STATE_LOW) {
        		P1OUT &= ~pin_num;
        	} else if (par[0] == PAR_STATE_HIGH) {
        		P1OUT |=  pin_num;
        	} else {
        		//bad digital state
        	}
        } else if (par[0] == PAR_PORT2) {
        	P2SEL &= ~pin_num;
        	if (par[2] == PAR_STATE_LOW) {
        		P2OUT &= ~pin_num;
        	} else if (par[2] == PAR_STATE_HIGH) {
        		P2OUT |=  pin_num;
        	} else {
        		//bad digital state
        	}
        } else if (par[0] == PAR_PORT3) {
        	P3SEL &= ~pin_num;
        	if (par[2] == PAR_STATE_LOW) {
        		P3OUT &= ~pin_num;
        	} else if (par[0] == PAR_STATE_HIGH) {
        		P3OUT |=  pin_num;
        	} else {
        		//bad digital state
        	}
        } else {
        	//bad port number
        }
        cmd = CMD_PASS;
        break;
    case CMD_GPIO_GET:
    	pin_num = (0x01 << par[1]);
    	if (par[0] == PAR_PORT1) {
    		P1SEL &= ~pin_num;
    		if (P1IN & pin_num == 0x00) {
    			res[++response_len - 1] = 0x00;
    		} else {
    			res[++response_len - 1] = 0x01;
    		}
    	} else if (par[0] == PAR_PORT2) {
    		P2SEL &= ~pin_num;
    		if (P2IN & pin_num == 0x00) {
    			res[++response_len - 1] = 0x00;
    		} else {
    			res[++response_len - 1] = 0x01;
    		}
    	} else if (par[0] == PAR_PORT3) {
    		P3SEL &= ~pin_num;
    		if (P3IN & pin_num == 0x00) {
    			res[++response_len - 1] = 0x00;
    		} else {
    			res[++response_len - 1] = 0x01;
    		}
    	} else {
    		// bad port number
    	}
    	cmd = CMD_PASS;
    	break;
    case CMD_GPIO_SETPWM:
    	pin_num = (0x01 << par[1]);
    	pwm_freq = 1020; //>=1.1kHz
    	//max or min pwm = high or low
    	if (par[2] == 0x00 || par[2] == 0xFF) {
    		if (par[2] == 0xFF) {
    			par[2] = 0x01;
    		}
    		process_cmd(CMD_GPIO_SET, par);
    		break;
    	}
    	if (par[0] == PAR_PORT1) {
    		//set the pin as output
    		P1SEL |= pin_num;
    		P1SEL2 &= ~pin_num;
    		P1DIR |= pin_num;
    	} else if (par[0] == PAR_PORT2) {
    		//set the pin as output
    		P2SEL |= pin_num;
    		P2SEL2 &= ~pin_num;
    		P2DIR |= pin_num;
    	} else if (par[0] == PAR_PORT3) {
    		//set the pin as output
    		P3SEL |= pin_num;
    		P3SEL2 &= ~pin_num;
    		P3DIR |= pin_num;
    	} else {
    		// bad port number
    	}
    	//set pwm period in respect to SMCLK
    	TA0CCR0 = pwm_freq - 1;
    	//output is reset when timer counts to TACCRx value
    	TA0CCTL1 = OUTMOD_7;
    	//pwm duty cycle
    	TA0CCR1 = par[2]*4;
    	//choose SMCLK and up mode
    	TA0CTL = TASSEL_2 + MC_1;
    	cmd = CMD_PASS;
    	break;
    case CMD_GPIO_GETPWM:
    	pin_num = (0x01 << par[1]);

    	n = 0;
    	duty_cycle = 0;
    	percent_on = 0;
    	time_stamp[0].time = 0;
    	time_stamp[1].time = 0;
    	time_stamp[2].time = 0;
    	time_stamp[0].edge = 0;
    	time_stamp[1].edge = 0;
    	time_stamp[2].edge = 0;

    	if (par[0] == PAR_PORT1) {
    		P1SEL |= pin_num;                            // TA0.1 option select
    	} else if (par[0] == PAR_PORT2) {
    		P2SEL |= pin_num;
    	} else if (par[0] == PAR_PORT3) {
    		P3SEL |= pin_num;
    	} else {
    		// bad port number
    	}
    	TACTL |= TASSEL_2 + MC_2 + ID_0;    // SMCLK + Continuous mode
    	TACCTL0 |= CM_3 + CCIS_0 + SCS + CAP + CCIE ;    // Raising Edge/Falling + CCI0A + Sync + Capture Mode + Interrupt enable
    	//__enable_interrupt();
    	unsigned char loc_err_cnt = 0;
    	while(loc_err_cnt < 3){
        	if (n > 2){
        		if (time_stamp[0].edge == RISING &&
        				time_stamp[1].edge == FALLING &&
						time_stamp[2].edge == RISING) {
        			duty_cycle = time_stamp[1].time - time_stamp[0].time;
        			period = time_stamp[2].time - time_stamp[0].time;
        			break;
        		} else if (time_stamp[0].edge == FALLING &&
        				time_stamp[1].edge == RISING &&
						time_stamp[2].edge == FALLING) {
        			duty_cycle = (unsigned long)(time_stamp[2].time - time_stamp[1].time);
        			period = (unsigned long)(time_stamp[2].time - time_stamp[0].time);
        			break;
        		} else {
        			//error handler implement!
        			loc_err_cnt++;
        			continue;
        			}
        			n = 0;
        		}
        		n = 0;
        	}
		if (!period){
			percent_on = 0;
	    	cmd = CMD_PASS;
	    	break;
		}
    	n = 0;
    	TACCTL0 &= ~CCIE;
    	duty_cycle = period = 0;
        percent_on = (unsigned char)((duty_cycle*100)/period);
    	//res[++response_len - 1] = percent_on;
    	cmd = CMD_PASS;
    	break;
    case CMD_GPIO_GETFREQ:
        //TODO and check
//    	pwm_freq = CCR0;
//    	int i;
//    	for(i=0; i < sizeof(unsigned int); i++) {
//    		res[++response_len - 1] = (unsigned char)((pwm_freq));
//    	}
    	cmd = CMD_PASS;
    	break;
    case CMD_SET_PORT:
        if (par[0] == PAR_PORT1) {
        	P1DIR |= 0xFF;
        	P1OUT = par[1];
        } else if (par[0] == PAR_PORT2) {
        	P2DIR |= 0xFF;
        	P2OUT = par[1];
        } else if (par[0] == PAR_PORT3) {
        	P3DIR |= 0xFF;
        	P3OUT = par[1];
        } else {
        	//bad port number
        }
        cmd = CMD_PASS;
        break;
    case CMD_GET_PORT:
        if (par[0] == PAR_PORT1) {
        	P1DIR &= 0x00;
        	res[++response_len - 1] = P1IN;
        } else if (par[0] == PAR_PORT2) {
        	P2DIR &= 0x00;
        	res[++response_len - 1] = P2IN;
        } else if (par[0] == PAR_PORT3) {
        	P3DIR &= 0x00;
        	res[++response_len - 1] = P3IN;
        } else {
        	//bad port number
        }
        cmd = CMD_PASS;
        break;
    //be carefull with pins 1.6 und 1.7 (SDA and SCL)
    case CMD_SET_SEL:
        if (par[0] == PAR_PORT1) {
        	P1SEL = par[1];
        } else if (par[0] == PAR_PORT2) {
        	P2SEL = par[1];
        } else if (par[0] == PAR_PORT3) {
        	P3SEL = par[1];
        } else {
        	//bad port number
        }
        cmd = CMD_PASS;
        break;
    case CMD_GET_SEL:
        if (par[0] == PAR_PORT1) {
        	res[++response_len - 1] = P1SEL;
        } else if (par[0] == PAR_PORT2) {
        	res[++response_len - 1] = P2SEL;
        } else if (par[0] == PAR_PORT3) {
        	res[++response_len - 1] = P3SEL;
        } else {
        	//bad port number
        }
        cmd = CMD_PASS;
        break;
    case CMD_SET_INTERRUPT:
    	//TODO
        cmd = CMD_PASS;
        break;
    case CMD_SPI_WRITE:
    	//TODO
        cmd = CMD_PASS;
        break;
    case CMD_SPI_READ:
    	//TODO
        cmd = CMD_PASS;
        break;
    default:
    	//TODO
        cmd = CMD_PASS;
        break;
    }
}

void start_cb()
{
    cmd = CMD_PASS;
    unsigned char i;
    for (i = 0; i < MAX_PARAMETERS; i++) {
    	par[i] = PAR_UNKNOWN;
    }
    last_byte = 0x00;
    parameter_len = 0x00;
}

//called on each received byte
void receive_cb(unsigned char receive)
{
    if(cmd == CMD_PASS) {

        cmd = receive;
        //process commands with no paramethers (if any)
        flstop=1;
        process_cmd(cmd, par);
    } else {
        par[++parameter_len - 1] = receive;
        //execute command <=> all parameters received
        if (parameter_len == 3){
        	if (cmd == CMD_GPIO_DIR ||
        			cmd == CMD_GPIO_SET ||
					cmd == CMD_GPIO_SETPWM ||
					cmd == CMD_SET_INTERRUPT) {
        		process_cmd(cmd, par);
        	}
        } else if (parameter_len == 2) {
        	if (cmd == CMD_GPIO_GET ||
        			cmd == CMD_SET_PORT ||
					cmd == CMD_SET_SEL ||
					cmd == CMD_GPIO_GETFREQ ||
					cmd == CMD_GPIO_GETPWM ||
					cmd == CMD_SPI_WRITE ||
					cmd == CMD_SPI_READ) {
        		process_cmd(cmd, par);
        	}
        } else if (parameter_len == 1) {
        	if (cmd == CMD_GET_PORT ||
        			cmd == CMD_GET_SEL) {
        		process_cmd(cmd, par);
        	}
        } else {
        	//error  : too many parameters

        }
    }
}

void transmit_cb(unsigned char volatile *byte)
{

    *byte = *(res + last_byte);
    last_byte++;
}



void stop(){
	//turn zumo power on
	    cmd = CMD_GPIO_DIR;
	    par[0] = PAR_PORT3;
	    par[1] = PAR_PIN6;
	    par[2] = PAR_DIR_OUT;
	    process_cmd(cmd, par);

	    cmd = CMD_GPIO_SET;
	    par[0] = PAR_PORT3;
	    par[1] = PAR_PIN6;
	    par[2] = PAR_STATE_HIGH;
	    process_cmd(cmd, par);
	//dir motor left = forward
	    cmd = CMD_GPIO_DIR;
	    par[0] = PAR_PORT2;
	    par[1] = PAR_PIN1;
	    par[2] = PAR_DIR_OUT;
	    process_cmd(cmd, par);

		cmd = CMD_GPIO_SET;
		par[0] = PAR_PORT2;
		par[1] = PAR_PIN1;
		par[2] = PAR_STATE_LOW;
		process_cmd(cmd, par);
	//dir motor right = backward
	    cmd = CMD_GPIO_DIR;
	    par[0] = PAR_PORT1;
	    par[1] = PAR_PIN0;
	    par[2] = PAR_DIR_OUT;
	    process_cmd(cmd, par);

		cmd = CMD_GPIO_SET;
		par[0] = PAR_PORT1;
		par[1] = PAR_PIN0;
		par[2] = PAR_STATE_LOW;
		process_cmd(cmd, par);

	//left motor speed
		P1SEL &= ~BIT5;
		P1DIR |= BIT5;
		P1OUT &= ~BIT5;
//	    cmd = CMD_GPIO_DIR;
//	    par[0] = PAR_PORT1;
//	    par[1] = PAR_PIN5;
//	    par[2] = PAR_DIR_OUT;
//	    process_cmd(cmd, par);
//
//		cmd = CMD_GPIO_SETPWM;
//		par[0] = PAR_PORT1;
//		par[1] = PAR_PIN5;
//		par[2] = PAR_STATE_HIGH;
//		process_cmd(cmd, par);

	//right motor speed
	    cmd = CMD_GPIO_DIR;
	    par[0] = PAR_PORT2;
	    par[1] = PAR_PIN6;
	    par[2] = PAR_DIR_OUT;
	    process_cmd(cmd, par);

		cmd = CMD_GPIO_SET;
		par[0] = PAR_PORT2;
		par[1] = PAR_PIN6;
		par[2] = PAR_STATE_LOW;
		process_cmd(cmd, par);
}

void drive(){
	unsigned long i;
	//turn zumo power on
	    cmd = CMD_GPIO_DIR;
	    par[0] = PAR_PORT3;
	    par[1] = PAR_PIN6;
	    par[2] = PAR_DIR_OUT;
	    process_cmd(cmd, par);

	    cmd = CMD_GPIO_SET;
	    par[0] = PAR_PORT3;
	    par[1] = PAR_PIN6;
	    par[2] = PAR_STATE_HIGH;
	    process_cmd(cmd, par);

	//left motor init
	    cmd = CMD_GPIO_DIR;
	    par[0] = PAR_PORT2;
	    par[1] = PAR_PIN1;
	    par[2] = PAR_DIR_OUT;
	    process_cmd(cmd, par);

	    cmd = CMD_GPIO_DIR;
	    par[0] = PAR_PORT1;
	    par[1] = PAR_PIN5;
	    par[2] = PAR_DIR_OUT;
	    process_cmd(cmd, par);


	//right motor init
		cmd = CMD_GPIO_DIR;
		par[0] = PAR_PORT1;
		par[1] = PAR_PIN0;
		par[2] = PAR_DIR_OUT;
		process_cmd(cmd, par);

	    cmd = CMD_GPIO_DIR;
	    par[0] = PAR_PORT2;
	    par[1] = PAR_PIN6;
	    par[2] = PAR_DIR_OUT;
	    process_cmd(cmd, par);
dir_left(0);
dir_right(0);
while(1){
	for(i=0; i<1000000; i++){
		mid_speed();
	}
	for(i=0; i<1000000; i++){
		left_curve();
	}
	for(i=0; i<1000000; i++){
		mid_speed();
	}
	for(i=0; i<1000000; i++){
		right_curve();
	}
	if(flstop) break;
}
stop();


}

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;                      // Stop WDT


    TI_USCI_I2C_slaveinit(start_cb, transmit_cb, receive_cb, 0x23);

    //_EINT();
//    BCSCTL1 = CALBC1_16MHZ;
//    DCOCTL = CALDCO_16MHZ;
//    LPM0;
    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL  = CALDCO_16MHZ;
    volatile unsigned long i;

//    __enable_interrupt();
    __bis_SR_register(GIE);



    while(1) __asm__("nop");

    return 0;
}


#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
    {
		time_stamp[n].time = TA0CCR0;
		if ((TA0CCTL0 & CCI) == CCI) { //rising
			time_stamp[n++].edge = RISING;
		}else {
			time_stamp[n++].edge = FALLING;
		}
    }


//interrupt on input pwm capturing
//#pragma vector=TIMER0_A0_VECTOR
//__interrupt void Timer_A (void)
//    {
//		if (flag_very_fst_time) {
//			if ((TA0CCTL0 & CCI) == CCI) { //rising
//				rising1 = TA0CCR0;
//				flag_captured1 = 1;
//				flag_very_fst_time = 0;
//			}
//		} else {
//			if ((TA0CCTL0 & CCI) == CCI) { //rising
//				rising2 = TA0CCR0;
//				flag_fst_rising = 1;
//				flag_captured3 = 1;
//			} else {						//falling
//				falling = TA0CCR0;
//				flag_captured2 = 1;
//			}
//		}
//    }


void dir_left(unsigned char dir){
	//dir motor left = forward


			cmd = CMD_GPIO_SET;
			par[0] = PAR_PORT2;
			par[1] = PAR_PIN1;
			if (dir){
				par[2] = PAR_STATE_LOW;
			}
			else {
				par[2] = PAR_STATE_HIGH;
			}
			process_cmd(cmd, par);
}

void dir_right(unsigned char dir) {


			cmd = CMD_GPIO_SET;
			par[0] = PAR_PORT1;
			par[1] = PAR_PIN0;
			if (dir){
							par[2] = PAR_STATE_LOW;
						}
						else {
							par[2] = PAR_STATE_HIGH;
						}
			process_cmd(cmd, par);

}

void start_left() {

	cmd = CMD_GPIO_SET;
	par[0] = PAR_PORT1;
	par[1] = PAR_PIN5;
	par[2] = PAR_STATE_HIGH;
	process_cmd(cmd, par);
}
void start_right() {


	cmd = CMD_GPIO_SET;
	par[0] = PAR_PORT2;
	par[1] = PAR_PIN6;
	par[2] = PAR_STATE_HIGH;
	process_cmd(cmd, par);
}
void stop_left() {


	cmd = CMD_GPIO_SET;
	par[0] = PAR_PORT1;
	par[1] = PAR_PIN5;
	par[2] = PAR_STATE_LOW;
	process_cmd(cmd, par);
}
void stop_right() {


	cmd = CMD_GPIO_SET;
	par[0] = PAR_PORT2;
	par[1] = PAR_PIN6;
	par[2] = PAR_STATE_LOW;
	process_cmd(cmd, par);
}

void mid_speed(){
	start_left();
	start_right();
	stop_left();
	stop_right();
}

void right_curve(){
	char n = 0;
	while(n<10){
	start_left();
	unsigned long i;
	for(i=0; i<1000000;i++);
	start_right();
	stop_left();
	stop_right();
	n++;
	}
}

void left_curve(){
	char n = 0;
	while(n<10){
	start_right();
	unsigned long i;
	for(i=0; i<1000000;i++);
	start_left();
	stop_left();
	stop_right();
	n++;
	}
}

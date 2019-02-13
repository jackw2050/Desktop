/* 
https://stackoverflow.com/questions/13046624/how-to-permanently-export-a-variable-in-linux
Look into custom , perminant device tree overlays

Beam    16 bit 0 - 65535    Zero - 32767 (0x7FFF)   Voltage swing - 4V?  1 - 5.0    Zero - 2V  (Frequency range?)
Levels  16 bit 0 - 65535    Zero - 32767 (0x7FFF)   Voltage swing - V?
Lid Thermistor  8 bit - 255 (What is range and normal?)

*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <queue>
#include <fcntl.h> 
#include <errno.h>
#include <iostream>
#include <fstream>
#include <unistd.h> //for usleep
#include "GPIO.h"
#include "PWM.h" 
#include "SPIDevice.h"
#include <math.h>   
#include <ctime>
#include <pthread.h>
#include <termios.h>










using namespace exploringBB;
using namespace std;

queue<int> commandBuffer;
queue<unsigned char> inputBuffer;
int serialComm, encoderComm;
int wlen;
bool debug = true;
fstream filestr;
// Helper function for textual date and time.
// DTTMSZ must allow extra character for the null terminator.

#define DTTMFMT "%Y-%m-%d %H:%M:%S "
#define DTTMSZ 21
int SYSTEM_VOLTAGES = 0;
char buff[DTTMSZ];


static char *getDtTm(char *buff) {
	time_t t = time(0);
	strftime(buff, DTTMSZ, DTTMFMT, localtime(&t));
	return buff;
}



#define POWER_INPUT     0
#define SYSTEM_VOLTAGES 1
#define FEEDBACK        2
#define SINE            3
#define THERMISTORS     7

#define LID 		    0
#define ARRESTMENT	    1
#define GEARBOX		    2
#define CONNING_TOWER   3  
#define METER_1		    4
#define METER_2		    5
#define BOARD_1		    6
#define BOARD_2		    7
#define CROSS           8
#define LONG            9
#define ZH	            10
#define BEAM            11
#define ZP	            12
#define PLUS_5R         13

#define SYS_ZH	        0
#define SYS_3P3         1
#define SYS_BBB_5       2
#define SYS_5R          3
#define SYS_M5          4
#define SYS_12          5
#define SYS_BATT_1      6
#define SYS_BATT_2      7

#define ADC_REF         5.0 //4.096

GPIO *Heartbeat;            //global pointers


// Globals



GPIO systemPowerEnable(37);             //P8_22
GPIO arrestmentHeaterFET(38);           //P8_3
GPIO conningTowerFET(39);               //P8_4
GPIO meterHeaterFET(34);                //P8_5
GPIO gearboxHeaterFET(35);              //P8_6
GPIO boardHeaterFET1(66);               //P8_7
GPIO boardHeaterFET2(67);               //P8_8
GPIO SHAFT_DIR(45);                     //P8_11
GPIO SHAFT_PWM(44);                     //P8_12
GPIO ARRESTMENT_DIR(27);                //P8_17
GPIO ARRESTMENT_PWM(65);                //P8_18
GPIO CROSS_DIR(8);                      //P8_35
GPIO CROSS_PWM(80);                     //P8_36
GPIO LONG_DIR(78);                      //P8_37
GPIO LONG_PWM(79);                      //P8_38   

GPIO FB_MUX_A0(32);                     //P8_25
GPIO FB_MUX_A1(61);                     //P8_26
GPIO thermistor_MUX_A0(10);             //P8_31
GPIO thermistor_MUX_A1(11);             //P8_32
GPIO thermistor_MUX_A2(9);              //P8_33
GPIO thermistor_MUX_A3(81);             //P8_34
GPIO SPI0_CS0_MUX_A0(76);               //P8_39
GPIO SPI0_CS0_MUX_A1(77);               //P8_40
GPIO SPI0_CS0_MUX_A2(74);               //P8_41
GPIO POWER_MUX_A0(75);                  //P8_42 
GPIO POWER_MUX_A1(72);                  //P8_43 
GPIO POWER_MUX_A2(73);                  //P8_44 
   
// INPUTS
GPIO LONG_FAULT(86);                    //P8_27
// GPIO Heartbeat(87);                    //P8_29

GPIO ARRESTMENT_FAULT(89);              //P8_30
GPIO SHAFT_FAULT(69);                   //P8_9
GPIO CROSS_FAULT(68);                   //P8_210

SPIDevice ADC(0, 0);                     // chip select 0 on bus 1
SPIDevice Encoder(1, 0);                 // chip select 0 on bus 1

SPIDevice spi(0, 0);                     // chip select 0 on bus 1
PWM PWM1A("3", "0");


float p3p3v    = 0;
float bbb5     = 0;
float p5r      = 0;
float m5v      = 0;
float p12v     = 0;
float batt1v   = 0;
float batt2v   = 0;
float zhv      = 0;

float zh_fb    = 0;
float p5r_fb   = 0;
float zpv      = 0;
float beam     = 0;
int beamInt    = 0;
unsigned short beamShort = 0;
unsigned short beamFreq  = 0;
unsigned short crossShort = 0;
unsigned short crossFreq  = 0;
unsigned short longShort = 0;
unsigned short longFreq  = 0;

const float thermistorVals[] = { 884600, 830900, 780800, 733900, 690200, 649300, 611000, 575200, 541700, 510400, 481000, 453500, 427700, 403500, 380900, 359600, 339600, 320900, 303300, 286700, 271200, 256500, 242800, 229800, 217600, 206200, 195400, 185200, 175600, 166600, 158000, 150000, 142400, 135200, 128500, 122100, 116000, 110300, 104900, 99800, 94980, 90410, 86090, 81990, 78110, 74440, 70960, 67660, 64530, 61560, 58750, 56070, 53540, 51130, 48840, 46670, 44600, 42640, 40770, 38990, 37300, 35700, 34170, 32710, 31320, 30000, 28740, 27540, 26400, 25310, 24270, 23280, 22330, 21430, 20570, 19740, 18960, 18210, 17490, 16800, 16150, 15520, 14920, 14350, 13800, 13280, 12770, 12290, 11830, 11390, 10970, 10570, 10180, 9807, 9450, 9109, 8781, 8467, 8166, 7876, 7599, 7332, 7076, 6830, 6594, 6367, 6149, 5940, 5738, 5545, 5359, 5180, 5007, 4842, 4682, 4529, 4381, 4239, 4102, 3970, 3843, 3720, 3602, 3489, 3379, 3273, 3172, 3073, 2979, 2887, 2799, 2714, 2632, 2552, 2476, 2402, 2331, 2262, 2195, 2131, 2069, 2009, 1950, 1894, 1840, 1788, 1737, 1688, 1640, 1594, 1550, 1507, 1465, 1425, 1386, 1348, 1311, 1276, 1241, 1208, 1176, 1145, 1114, 1085, 1057, 1029, 1002, 976.3, 951.1, 926.7, 903, 880, 857.7, 836.1, 815, 794.6, 774.8, 755.6, 736.9, 718.8, 701.2, 684.1, 667.5, 651.3, 635.6, 620.3, 605.5, 591.1, 577.1, 563.5, 550.2 };
const float THERMISTOR_C[] = { -40, -39, -38, -37, -36, -35, -34, -33, -32, -31, -30, -29, -28, -27, -26, -25, -24, -23, -22, -21, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150 };

string Meter                              = "Land";
string Hardware_revision                  = "1.0";         
string Software_revision                  = "1.5";       
string Calibration_version                = "calibrated";  
string Customer                           = "Orangelamp";  
   
int pwm_freq                              = 125;       
int sense_freq                            = 10000;     
float adc_offset                          = 0.0044678;   
float zh_offset                           = 0.0;         
       
float p12v_offset                         = 0.0;         
float p5v_offset                          = 0.0;         
float p3p3v_offset                        = 0.0;         
      
float batt_v_offset                       = 0.0;         
float beam_offset                         = 0.0;         
float m5v_offset                          = 0.0;         
float zp_offset                           = 0.0;   

float lid_thermistor_offset               = 0.0;  
float battery_thermistor_offset           = 0.0;   
float gearbox_thermistor_offset           = 0.0;         
float conning_tower_thermistor_offset     = 0.0;         
float arrestment_thermistor_offset        = 0.0;         
float meter_thermistor_1_offset           = 0.0;         
float meter_thermistor_2_offset           = 0.0;            

float adc_divider                         = 0.0044678;   
float zh_divider                          = 0.0;         
float lid_thermistor_divider              = 0.0;         
float p12v_divider                        = 0.0;         
float p5v_divider                         = 0.0;         
float p3p3v_divider                       = 0.0;         
float battery_thermistor_divider          = 0.0;         
float batt_v_divider                      = 0.0;         
float beam_divider                        = 0.0;         
float m5v_divider                         = 0.0;         
float zp_divider                          = 0.0; 

float gearbox_thermistor_divider          = 0.0;    
float conning_tower_thermistor_divider    = 0.0;         
float arrestment_thermistor_divider       = 0.0;         
float meter_thermistor_1_divider          = 0.0;         
float meter_thermistor_2_divider          = 0.0;    
float beam10                              = 0.0;         
float beam50                              = 2.5;         
float beam90                              = 5.0;         
float p5vTarget                           = 5.0;        
float p5vError                            = 0.2;        
float p3p3vTarget                         = 3.3;        
float p3p3vError                          = 0.2;        
float p12vTarget                          = 12.0;       
float p12vError                           = 0.2;        
float batteryTarget                       = 12.0;       
float batteryError                        = 2.0;        
float zh_Target                           = 36.0;       
float zh_Error                            = 0.5;        
float zpTarget                            = 24.0;       
float zpError                             = 0.01;    
float meterThermistor1                    = 0;
float meterThermistor2                    = 0;
float meterHeaterThermistorAverage        = 0;
float arrestmentThermistor                = 0;
float gearboxThermistor                   = 0;
float conningTowerThermistor              = 0;
float lidThermistor                       = 0;
short lidThermistorInt                    = 0;
float lidAlarmHigh                      = 70;
float lidAlarmLow                       = 50;
float boardThermistor1                    = 0;
float boardThermistor2                    = 0;






float meterHeaterSetpoint                 = 58;
float arrestmentHeaterSetpoint            = 58;
float gearboxHeaterSetpoint               = 58;
float conningTowerHeaterSetpoint          = 58;
float boardHeater1Setpoint                = 58;
float boardHeater2Setpoint                = 58;

float longLevelZero                       = 3.077;  //3.79544;  //Value at level
float longLevelDivider                    =.002;  //Arc seconds per Volt     
float crossLevelZero                      = 1.6;  //2.942;  //Value at level
float crossLevelDivider                   =.002;  //Arc seconds per Volt     
float crossZero                         = 1.6;
float crossMin                          = 0.8;
float crossMax                          = 2.4;
float crossNormFactor                   = 0;
float longZero                          = 2.00;
float longMin                           =.75;
float longMax                           = 4.00;
float longNormFactor                    = 0;
float levelFreqPerVolt                  = 100;
float beamFreqPerVolt                   = 100;
 
 
float crossValue                          = 0;
float longValue                           = 0;
int crossValueInt                         = 0;
int longValueInt                          = 0;
int pwmInt                                = 0x7FFF;
float pmwFloat                            = 50;

unsigned char inBuf[80];

GPIO_VALUE meterHeaterStatus              = LOW;
GPIO_VALUE gearboxHeaterStatus            = LOW;
GPIO_VALUE conningTowerHeaterStatus       = LOW;
GPIO_VALUE arrestmentHeaterStatus         = LOW;
GPIO_VALUE boardHeater1Status             = LOW;
GPIO_VALUE boardHeater2Status             = LOW;





bool meterThermistorUsed                  = true;
bool arrestmentThermistorUsed             = true;
bool gearboxThermistorUsed                = true;
bool conningTowerThermistorUsed           = true;
bool lidThermistorUsed                    = true;
bool boardThermistorUsed                  = false;
bool oneHzData                            = false;

int beatCount                             = 0;
int heaterCheckTime, voltageCheckTime, heartBeatChecktime;
int landRun                               = 1;

float round2(float var) 
{ 
	// 37.66666 * 100 =3766.66 
	// 3766.66 + .5 =37.6716    for rounding off value 
	// then type cast to int so value is 3766 
	// then divided by 100 so the value converted into 37.66 
	float value = (int)(var * 100 +.5); 
	return (float)value / 100; 
} 

int logError(string error_1) {
	filestr.open("error.log", fstream::out | fstream::app);
	filestr << getDtTm(buff) << error_1 << std::endl;
	filestr.close();
}

int callbackLongFault(int var) {
	logError("Long motor motor falut detected");
	if (debug == true) {
		cout << "Long motor motor falut detected" << endl;
	}
	//  Send error to Android
	return var;
}

int callbackCrossFault(int var) {
	logError("Cross level motor falut detected");
	if (debug == true) {
		cout << "Cross level motor falut detected" << endl;
	}
	//  Send error to Android
	return var;
}

int callbacShaftFault(int var) {
	logError("Shaft motor falut detected");
	if (debug == true) {
		cout << "Shaft motor falut detected" << endl;
	}
	//  Send error to Android
	return var;
}

int callbackArrestmentFault(int var) {
	logError("Arrestment motor falut detected");
	if (debug == true) {
		cout << "Arrestment motor falut detected" << endl;
	}
	return var;
}



void trim(string& s) {
	size_t p = s.find_first_not_of(" \t");
	s.erase(0, p);
    
	p = s.find_last_not_of(" \t");
	if (string::npos != p)
		s.erase(p + 1);
}
  

/*********************************************************
                     UART setup
*********************************************************/
int set_interface_attribs(int fd, int speed)
{
	struct termios tty;
	if (tcgetattr(fd, &tty) < 0) {
		printf("Error from tcgetattr: %s\n", strerror(errno));
		return -1;
	}
	cfsetospeed(&tty, (speed_t)speed);
	cfsetispeed(&tty, (speed_t)speed);
	tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8; /* 8-bit characters */
	tty.c_cflag &= ~PARENB; /* no parity bit */
	tty.c_cflag &= ~CSTOPB; /* only need 1 stop bit */
	tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

	/* setup for non-canonical mode */
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		printf("Error from tcsetattr: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

void set_mincount(int fd, int mcount)
{
	struct termios tty;
	if (tcgetattr(fd, &tty) < 0) {
		printf("Error tcgetattr: %s\n", strerror(errno));
		return;
	}
	tty.c_cc[VMIN] = mcount ? 1 : 0;
	tty.c_cc[VTIME] = 5; /* half second timer */

	if (tcsetattr(fd, TCSANOW, &tty) < 0)
		printf("Error tcsetattr: %s\n", strerror(errno));
}
/*********************************************************
                     End UART setup
*********************************************************/

int transmitData(unsigned char cmdArray[80]) {
	int txLength = (int)cmdArray[0] + 1;
	cout << hex << cmdArray[4] << endl;
	int transmit[txLength]; // = (int) cmdArray;
	size_t txSize = txLength;
	for (int x = 0; x < txLength; x++) {
		transmit[x] = (int)cmdArray[x];
		if ((write(serialComm, &transmit[x], 1)) < 0) {
			        //send the string
		perror("Failed to write to the output\n");
			return -1;
		}
	}
	tcdrain(serialComm); /* delay for output */

	return 0;
}


int Alternate_Break() {
	cout << "Sending alternate break" << endl;
	int transmit = 0x050702;
	size_t txSize = sizeof(transmit);
	cout << "alt break " << txSize << endl;

	if ((write(serialComm, &transmit, txSize)) < 0) {
		        //send the string
	    perror("Failed to write to the output\n");
		return -1;
	}

	tcdrain(serialComm); /* delay for output */

	return 0;
}


int calculateChecksum(unsigned char cmdArray[80], int byteLength) {
	short checksum = 0;

	for (int index = 0; index < byteLength; index++) {
		checksum = checksum ^ cmdArray[index];
		// cout << "Checksum = " << checksum << endl;
	}
    
	return checksum;
}










// Received data is currently limited to one 16 bit value.
// Add additional functions when more input data is needed
int intToByte(unsigned char data_bytes[80], int byteLength) {
	int byte1, byte2, byte3, byte4;
	int dataOut = 0;
	// byteLength = byteLength -2;
	// cout << "Byte length " << byteLength << endl;
	 byte1 =  (int)data_bytes[2] << 8;
	byte2 = (int)(data_bytes[3]);
	byte3 = (int)(data_bytes && 0x00ff0000) >> 16;
	byte4 = (int)(data_bytes && 0xff000000) >> 24;

	// cout << hex << byte1 << endl;
	// cout << hex << byte2 << endl;

    
	if(byteLength == 2) {
		dataOut = byte1 + byte2;
		// cout << hex << dataOut << endl;
		// cout << hex << dataOut << endl;
		return dataOut;
	}   
	else if(byteLength == 1) {
		dataOut = (int)data_bytes[2];
		// cout << hex << dataOut << endl;
		return dataOut;
	}   
	else if(byteLength == 3) {
		dataOut = byte1 + byte2 + byte3;
		// cout << hex << dataOut << endl;
		return dataOut;
	}  
	else if(byteLength == 4) {
		dataOut = byte1 + byte2 + byte3 + byte4;
		// cout << hex << dataOut << endl;
		return dataOut;
	}   
	else {
		perror("Invalid data length");
		return -1;
	}
    
}


int printDataSet(unsigned char tx_data_bytes[80], int rdlen) {

	unsigned char   *p;
	cout <<  "Current Data Set ";
	for (p = tx_data_bytes; rdlen-- > 0; p++) {
		printf(" 0x%x", *p);
	}
	printf("\n");
}




// [4, 2, 243,   49,   196]  = 5%    F331 ‭62257‬
// [4, 2, 230,   100,  132]  = 10%   E664
// [4, 2, 127,   255,  134]  = 50%   7FFF ‭32767‬
// [4, 2, 25,    153,  134]  = 90%   1999
// [4, 2, 5,     30,   29]   = 98%   051E ‭1310‬

int setPWMDutyCycle(PWM pwm, int newDutyCycle) {
	float dutyCycle;
	dutyCycle = 100.0 - ((float) newDutyCycle / 65535) * 100;
	// I should be able to change to simple oercentage if PWM is set to active low.
   
	if((dutyCycle > 2.0) & (dutyCycle < 98.1)) {
   
		// if(debug){cout << "Setting PWM duty cycle to " << dutyCycle << "%" << endl;}
		pwm.setDutyCycle(dutyCycle);
		pmwFloat = dutyCycle;
		if (debug){cout << "Duty cycle set to " << dutyCycle << endl; }
		return 0;
	}
	else {
		if (debug) {
			cout << "Bad duty cycle " << dutyCycle << endl;
		}
		// Set error flag
		return - 1;
	}
   
}

/* Purpose: This command tells the meter to send the current data set. 
The beam and level values are from a counter that is clocked by the respective signal. 
The counter counts to 0xFFFF, then rolls over (which of course must be accounted for). 
The difference between two consecutive values divied by the time interval is the signal frequency. 
The temperature value is an analog value converted to binary, which can vary from 0 to 255. 
It is not calibrated, however it is set to about 128 when the meter is at its operating temperature. 
Command ID: 03
Parameters: None
--Byte----Function----------------------

  0x02    Number of bytes to follow
  0x03    Command ID
  0xXX    Chechsum

Returns: Beam Freq, Long Level Freq, Cross Level Freq, Thermometer value

--Byte----Function----------------------

  0x09    Number of bytes to follow
  0x03    Response ID
  0xXX    Beam MSB
  0xXX    Beam LSB
  0xXX    Long Level MSB
  0xXX    Long Level LSB
  0xXX    Cross Level MSB
  0xXX    Cross Level LSB
  0xXX    Temperature
  0xXX    Chechsum
*/


int createFreqData() {
	beamFreq = (unsigned short)(beam * 1000 + 6000);
	crossFreq = (unsigned short)(crossValue * 2000 + 6000);
	longFreq = (unsigned short)(longValue * 2000 + 6000);
	cout << "Beam " << beamFreq << endl;
	cout << "Cross" << crossFreq << endl;
	cout << "Long" << longFreq << endl;
}
int Send_Current_Data_Set() {
	unsigned char tx_data_bytes[10] = { 0x09, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	createFreqData();
	tx_data_bytes[2] = beamFreq >> 8;
	tx_data_bytes[3] = beamFreq;
	tx_data_bytes[4] = longFreq >> 8;
	tx_data_bytes[5] = longFreq;
	tx_data_bytes[6] = crossFreq >> 8;
	tx_data_bytes[7] = crossFreq;
	tx_data_bytes[8] = lidThermistorInt;
	tx_data_bytes[9] = calculateChecksum(tx_data_bytes, 9);
	printDataSet(tx_data_bytes, 10);
    
	// for(int x = 0; x < 10; x++){
	//     transmitData(tx_data_bytes);
	//     usleep(1000000);
	// }
    
    
    
  return 0;
}

/*
Purpose: Tells the meter to transmit the current PWM duty cycle value.
The 16 bit value represents the duty cycle stored in the meter,
with 0x0000 = 0% DC and 0xFA00 (64000 decimal) = 100% DC.
This is the full range of the force feedback system.
Command ID: FE
Parameters: None
--Byte----Function----------------------
  0x02    Number of bytes to follow
  0xFE    Command ID
  0xXX    Checksum

Returns:
--Byte----Function----------------------
  0x04    Number of bytes to follow
  0xFE    Respose ID
  0xXX    MSB
  0xXX    LSB
  0xXX    Checksum
*/


int Get_PWM_Duty_Cycle() {
	unsigned char tx_data_bytes[5] = { 0x04, 0xFE, 0x00, 0x00, 0x00 };
	tx_data_bytes[2] =  pwmInt >> 8;
	tx_data_bytes[3] = pwmInt;
	tx_data_bytes[4] = calculateChecksum(tx_data_bytes, 4);
	// printDataSet( tx_data_bytes, 5);
	transmitData(tx_data_bytes);
}

int Set_PWM_Duty_Cycle(unsigned char tx_data_bytes[5]) {
	// cout << "Duty cycle data " << hex << (int)tx_data_bytes[2] << (int)tx_data_bytes[3] << endl;
	int newDutyCycle = ((int)tx_data_bytes[2] << 8) + (int)tx_data_bytes[3];
	cout << hex << "Hex duty cycle: " << hex << newDutyCycle << endl;
	// setPWMDutyCycle(PWM1A, newDutyCycle);
	if(debug) {
		cout << hex << "New duty cycle: : " << newDutyCycle << endl;
	}
}

int parse_commands(unsigned char command_bytes[80]) {
	int command_byte = command_bytes[1];
	cout << "Command: " << command_byte << endl;
	int numBytes = command_bytes[0];
	int oldChecksum = command_bytes[numBytes];

	if (oldChecksum == calculateChecksum(command_bytes, numBytes)) {
		if (command_byte == 0x00) {
			cout << "Send_System_Status()" << endl;
		}
		else if (command_byte == 0x01) {
			cout << "Send data at 1 sec interval - ENABLE" << endl;
			oneHzData = true;
            
		}    
		else if (command_byte == 0x02) {
			int newDutyCycle = intToByte(command_bytes, 2);
			Set_PWM_Duty_Cycle(command_bytes);
            
		}
		else if (command_byte == 0x03) {
			cout << "Send_Current_Data_Set" << endl;
			Send_Current_Data_Set();
		}    
		else if (command_byte == 0xFD) {
			cout << "CLR TIME;" << endl;
			// Synch 1Hz with clock
		}
		else if (command_byte == 0xFE) {
			cout << "Get_PWM_Duty_Cycle" << endl;
			Get_PWM_Duty_Cycle();
		}
        

		else if (command_byte == 0xFF) {
			cout << "Stop_Sending_Data_Sets" << endl;
			oneHzData = false;
		}
	}
	else if (command_byte == 0xF9) {
		cout << "Handshake request" << endl;
		Alternate_Break();
	}
	else {
		//set comm error
		cout << "Bad checksum" << endl;
	}

}


int getNewData() {
	/* simple noncanonical input */
	do {
		unsigned char buf[80];
		int rdlen;

		rdlen = read(serialComm, buf, sizeof(buf) - 1);
		if (rdlen > 0) {
			/*display hex */
			unsigned char   *p;
			// printf(" 0x%x", (int)buf[1]);
			// cout << "Buffer size: " << rdlen << endl;
			printf("\n");
			printf("Read %d:", rdlen);
			for (p = buf; rdlen-- > 0; p++)
				printf(" 0x%x", *p);
			printf("\n");

			if (buf[0] + 1 == rdlen)// Check for valid command message length
			{
				if (calculateChecksum(buf, buf[0]) == 1)// Check for valid checksum
				{
					parse_commands(buf);
				}
				else
				{
					perror("Receiver / Queue / Command Error >> Bad Checksum");
					// set error flag bit
				}
			}
        
			else
			{
				perror("Receiver / Queue / Command Error");
				// set error flag bit
			}


		} 
		else if (rdlen < 0) 
		{
			printf("Error from read: %d: %s\n", rdlen, strerror(errno));
		}
		/* repeat read to get full message */
	} while (1);
}



// int checkForData(){
void *checkForData(void *value) {
	if (debug) {
		cout << "checkForData thread started" << endl;
	}
	while (landRun == 1) {
		/* simple noncanonical input */
		do {
			unsigned char buf[80];
			int rdlen;
    
			rdlen = read(serialComm, buf, sizeof(buf) - 1);
			if (rdlen > 0) {
#ifdef DISPLAY_STRING
				buf[rdlen] = 0;
				printf("Read %d: \"%s\"\n", rdlen, buf);
#else /* display hex */
				unsigned char   *p;
				int tempX;
				// printf(" 0x%x", (int)buf[1]);
				cout << "Buffer size: " << rdlen << endl;
                
				printf("\n");
				printf("Read %d:", rdlen);
				for (p = buf; rdlen-- > 0; p++) {
					inputBuffer.push(*p);
					printf(" 0x%x", *p);
				}
				printf("\n");
				// 	parse_commands(buf);
	
#endif
			} 
			else if (rdlen < 0) {
				printf("Error from read: %d: %s\n", rdlen, strerror(errno));
			}
			/* repeat read to get full message */
		} while (1);
	}
    
}





















int createSysDataBuffer(short bytesToFollow, short responseID, unsigned char currentDataSet[]) {

	currentDataSet[0] = bytesToFollow;
	currentDataSet[1] = responseID;
	currentDataSet[2] = (int)beam;
	currentDataSet[3] = (int)(beam) >> 8;

	currentDataSet[4] = (int)crossValue;
	currentDataSet[5] = (int)(crossValue) >> 8;

	currentDataSet[6] = (int)longValue;
	currentDataSet[7] = (int)(longValue) >> 8;

	currentDataSet[8] = (int)lidThermistor;         // Need to determine what to put here
	currentDataSet[10] = calculateChecksum(currentDataSet, bytesToFollow);
    
  
    
    
	// cout << "Bytes to follow " <<  (short)currentDataSet[0] << endl;   
	// cout << "ID " <<  (int)currentDataSet[1] << endl;
	// cout << "Beam LSB " <<  hex << (int)currentDataSet[2] << endl;
	// cout << "Beam MSB " <<  hex << (int)currentDataSet[3] << endl;
	// cout << "crossLevel LSB " <<  hex << (int)currentDataSet[4] << endl;
	// cout << "crossLevel MSB " <<  hex << (int)currentDataSet[5] << endl;
	// cout << "longLevel LSB " <<  hex << (int)currentDataSet[6] << endl;
	// cout << "longLevel MSB " <<  hex << (int)currentDataSet[7] << endl;
	// cout << "lidTemp LSB " <<  hex << (int)currentDataSet[8] << endl;
	// cout << "lidTemp MSB " <<  hex << (int)currentDataSet[9] << endl;
	// cout << "Checksum " <<  hex << (int)currentDataSet[10] << endl;  
	// return dataOut;
    
}







int getDividers() {
	float dValue;
	ifstream ip("adc_divider.csv");
	if (!ip.is_open()) std::cout << "ERROR:  File Open" << "\n";
   
	string dividerName, dividerValue;
	std::string::size_type sz;      // alias of size_t
 
	while(ip.good()) {
       
		getline(ip, dividerName, ',');
		getline(ip, dividerValue, '\n');
		dValue = stof(dividerValue, &sz);
		float val = std::stof(dividerValue); 
		//  cout << dividerName << "\t" << dValue << endl;
		 trim(dividerName);
		// cout << dividerName << endl;
       
       
		if(dividerName == "zh_divider") {
			zh_divider = stof(dividerValue, &sz);   
		}
		else if(dividerName == "lid_thermistor_divider") {    
			lid_thermistor_divider = stof(dividerValue, &sz);   
		}
		else if(dividerName == "p12v_divider") {    
			p12v_divider = stof(dividerValue, &sz);
		}
		else if(dividerName == "p5v_divider") {    
			p5v_divider = stof(dividerValue, &sz);
		}
		else if(dividerName == "p3p3v_divider") {                              
			p3p3v_divider = stof(dividerValue, &sz);
		}    
		else if(dividerName == "battery_thermistor_divider") {    
			battery_thermistor_divider = stof(dividerValue, &sz);
		}
		else if(dividerName == "batt_v_divider") {    
			batt_v_divider = stof(dividerValue, &sz);
		}
		else if(dividerName == "beam_divider") {                        
			beam_divider = stof(dividerValue, &sz);
		}
		else if(dividerName == "m5v_divider") {    
			m5v_divider = stof(dividerValue, &sz);
		}
		else if(dividerName == "zp_divider") {                           
			zp_divider = stof(dividerValue, &sz);
		}
		else if(dividerName == "gearbox_thermistor_divider") {    
			gearbox_thermistor_divider = stof(dividerValue, &sz);
		}
		else if(dividerName == "conning_tower_thermistor_divider") {            
			conning_tower_thermistor_divider = stof(dividerValue, &sz);
		}
		else if(dividerName == "arrestment_thermistor_divider") {         
			arrestment_thermistor_divider = stof(dividerValue, &sz);
		}
		else if(dividerName == "meter_thermistor_1_divider") {            
			meter_thermistor_1_divider = stof(dividerValue, &sz);
		}
		else if(dividerName == "meter_thermistor_2_divider") {
			meter_thermistor_2_divider = stof(dividerValue, &sz);
		}   
		else {
			cout << "Invalid entry" << endl;
		}
       
		cout << dividerName << "\t" << dividerValue << endl;
   
	}
 
	ip.close();
}




int selectFB_MUX(int source) {
	switch (source) {
	case ZH :
		FB_MUX_A0.setValue(LOW);
		FB_MUX_A1.setValue(LOW);
		return 0;
		break;
	case BEAM :
		FB_MUX_A0.setValue(HIGH);
		FB_MUX_A1.setValue(LOW);
		return 0;
		break;    
	case ZP :
		FB_MUX_A0.setValue(LOW);
		FB_MUX_A1.setValue(HIGH);
		return 0;
		break;
	case PLUS_5R :
		FB_MUX_A0.setValue(HIGH);
		FB_MUX_A1.setValue(HIGH);
		return 0;
		break; 
	default :
		perror("FB MUX: Invalid selection.");
		return -1;
	}
} 



int selectSourceMUX(int source) {
	switch (source) {
	case POWER_INPUT :
		SPI0_CS0_MUX_A0.setValue(LOW);
		SPI0_CS0_MUX_A1.setValue(LOW);
		SPI0_CS0_MUX_A2.setValue(LOW);
		return 0;
		break;
	case SYSTEM_VOLTAGES :
		SPI0_CS0_MUX_A0.setValue(HIGH);
		SPI0_CS0_MUX_A1.setValue(LOW);
		SPI0_CS0_MUX_A2.setValue(LOW);
		return 0;
		break;    
	case FEEDBACK :
		SPI0_CS0_MUX_A0.setValue(LOW);
		SPI0_CS0_MUX_A1.setValue(HIGH);
		SPI0_CS0_MUX_A2.setValue(LOW);
		return 0;
		break;
	case THERMISTORS :
		SPI0_CS0_MUX_A0.setValue(HIGH);
		SPI0_CS0_MUX_A1.setValue(HIGH);
		SPI0_CS0_MUX_A2.setValue(HIGH);
		return 0;
		break; 
	default :
		perror("CS0 MUX: Invalid selection.");
		return -1;
	}
} 


int selectSysVoltageMUX(int source) {
	switch (source) {
	case SYS_3P3 :
		POWER_MUX_A0.setValue(LOW);
		POWER_MUX_A1.setValue(LOW);
		POWER_MUX_A2.setValue(LOW);
		return 0;
		break;
	case SYS_BBB_5 :
		POWER_MUX_A0.setValue(HIGH);
		POWER_MUX_A1.setValue(LOW);
		POWER_MUX_A2.setValue(HIGH);
		return 0;
		break;    
	case SYS_5R :
		POWER_MUX_A0.setValue(LOW);
		POWER_MUX_A1.setValue(HIGH);
		POWER_MUX_A2.setValue(HIGH);
		return 0;
		break;
	case SYS_M5 :
		POWER_MUX_A0.setValue(LOW);
		POWER_MUX_A1.setValue(LOW);
		POWER_MUX_A2.setValue(HIGH);
		return 0;
		break; 
	case SYS_12 :
		POWER_MUX_A0.setValue(HIGH);
		POWER_MUX_A1.setValue(LOW);
		POWER_MUX_A2.setValue(LOW);
		return 0;
		break; 
	case SYS_BATT_1 :
		POWER_MUX_A0.setValue(HIGH);
		POWER_MUX_A1.setValue(HIGH);
		POWER_MUX_A2.setValue(HIGH);
		return 0;
		break;             
	case SYS_BATT_2 :
		POWER_MUX_A0.setValue(HIGH);
		POWER_MUX_A1.setValue(HIGH);
		POWER_MUX_A2.setValue(LOW);
		return 0;
		break;              
	case SYS_ZH :
		POWER_MUX_A0.setValue(HIGH);
		POWER_MUX_A1.setValue(HIGH);
		POWER_MUX_A2.setValue(HIGH);
		return 0;
		break;              
	default :
		perror("CS0 MUX: Invalid selection.");
		return -1;
	}
} 

int   selectThermistorLevelMUX(int source) {
	switch (source) {
	case LID :
		thermistor_MUX_A0.setValue(LOW);
		thermistor_MUX_A1.setValue(LOW);
		thermistor_MUX_A2.setValue(LOW);
		thermistor_MUX_A3.setValue(LOW);
		return 0;
		break;
	case  ARRESTMENT :
		thermistor_MUX_A0.setValue(HIGH);
		thermistor_MUX_A1.setValue(LOW);
		thermistor_MUX_A2.setValue(LOW);
		thermistor_MUX_A3.setValue(LOW);
		return 0;
		break;    
	case  GEARBOX :
		thermistor_MUX_A0.setValue(LOW);
		thermistor_MUX_A1.setValue(HIGH);
		thermistor_MUX_A2.setValue(LOW);
		thermistor_MUX_A3.setValue(LOW);
		return 0;
		break;
	case  CONNING_TOWER :
		thermistor_MUX_A0.setValue(HIGH);
		thermistor_MUX_A1.setValue(HIGH);
		thermistor_MUX_A2.setValue(LOW);
		thermistor_MUX_A3.setValue(LOW);
		return 0;
		break; 
            
	case METER_1 :
		thermistor_MUX_A0.setValue(LOW);
		thermistor_MUX_A1.setValue(LOW);
		thermistor_MUX_A2.setValue(HIGH);
		thermistor_MUX_A3.setValue(LOW);
		return 0;
		break;
	case  METER_2 :
		thermistor_MUX_A0.setValue(HIGH);
		thermistor_MUX_A1.setValue(LOW);
		thermistor_MUX_A2.setValue(HIGH);
		thermistor_MUX_A3.setValue(LOW);
		return 0;
		break;    
	case  BOARD_1 :
		thermistor_MUX_A0.setValue(LOW);
		thermistor_MUX_A1.setValue(HIGH);
		thermistor_MUX_A2.setValue(HIGH);
		thermistor_MUX_A3.setValue(LOW);
		return 0;
		break;
	case  BOARD_2 :
		thermistor_MUX_A0.setValue(HIGH);
		thermistor_MUX_A1.setValue(HIGH);
		thermistor_MUX_A2.setValue(HIGH);
		thermistor_MUX_A3.setValue(LOW);
		return 0;
		break; 
	case  CROSS :
		thermistor_MUX_A0.setValue(LOW);
		thermistor_MUX_A1.setValue(LOW);
		thermistor_MUX_A2.setValue(LOW);
		thermistor_MUX_A3.setValue(HIGH);
		return 0;
		break;
	case  LONG :
		thermistor_MUX_A0.setValue(HIGH);
		thermistor_MUX_A1.setValue(LOW);
		thermistor_MUX_A2.setValue(LOW);
		thermistor_MUX_A3.setValue(HIGH);
		return 0;
		break;             
	default :
		perror("Thermistor MUX: Invalid selection.");
		return -1;
	}
} 





int heaterWork() {
   
}


bool checkLevels() {
	unsigned char receive[3] = { 0 };
	unsigned char transmit[3] = { 0 };
   
	// Measure Cross
	selectSourceMUX(THERMISTORS);
	selectThermistorLevelMUX(CROSS);
	ADC.transfer(transmit, receive, 3);
	// cross.level = cross.measureLevel();
   
	receive[3] = { 0 };
	selectSourceMUX(THERMISTORS);
	selectThermistorLevelMUX(LONG);
	ADC.transfer(transmit, receive, 3);
	// long.level = long.measureLevel();
	//
   
}

int setUpSPI_0() {
	ADC.setSpeed(100000);           // set the speed to 1 MHz
	 ADC.setMode(SPIDevice::MODE3);   // set the mode to Mode3
	 ADC.setBitsPerWord(8);
}

int setUpSPI_1() {
	Encoder.setSpeed(100000);           // set the speed to 1 MHz
	 Encoder.setMode(SPIDevice::MODE3);   // set the mode to Mode3
	 Encoder.setBitsPerWord(8);
}

float getVoltage(float rawVoltage) {
	// cout << "GetVoltage" << endl;
	float volts;
	volts = rawVoltage / 65536;
	volts = volts * ADC_REF;
	return volts;
}

float getAvgVolts() {
	float volts, rawVoltage;
	unsigned char receive[3] = { 0 };
	unsigned char transmit[3] = { 0 };
	float avgV = 0;
	int a;
	for (a = 0; a < 200; a++) {
		ADC.transfer(transmit, receive, 3);
		rawVoltage = (receive[1] << 8) + receive[2];
		rawVoltage = getVoltage(rawVoltage);
      
		avgV += rawVoltage;
		usleep(100);
	}
	avgV = avgV / a;
	// cout << "Raw voltage: " << avgV << endl;
	return avgV;
}

float checkDivider(int source, float rawVoltage) {
	float divider = 1;
	// cout << "Source " << source << endl;
   if(source == SYS_3P3) {
		divider = 1;
		//   cout << "System 3.3V" << "\t" << rawVoltage <<"\t" << p3p3v_divider << endl; 
		  return rawVoltage * p3p3v_divider;
	}
	else if(source == SYS_BBB_5) {
		divider = 2.0;
		//   cout << "System BBB 5V" << "\t" << rawVoltage <<"\t" << divider << endl; 
		  return rawVoltage * divider;
	} 
	else if(source == SYS_5R) {
		divider = 2.0;
		//   cout << "System 5.0V" << "\t" << rawVoltage <<"\t" << p5v_divider << endl; 
		  return rawVoltage * p5v_divider;
	}
	else if(source ==  SYS_M5) {
		divider = -2.457;
		//   cout << "System -5.0V" << "\t" << rawVoltage <<"\t" << divider << endl; 
		  return rawVoltage * m5v_divider;
	}
	else if(source ==  SYS_12) {
		divider = 6.1;
		//   cout << "System 12V" << "\t" << rawVoltage <<"\t" << p12v_divider << endl; 
		  return rawVoltage * p12v_divider;
	}
	else if(source ==  SYS_BATT_1) {
		divider = 6.1;
		cout << "Battery 1" << "\t" << rawVoltage << "\t" << batt_v_divider << endl; 
		return rawVoltage * batt_v_divider;
	}            
	else if(source ==  SYS_BATT_2) {
		divider = 6.1;
		//   cout << "Battery 2" << "\t" << rawVoltage <<"\t" << batt_v_divider << endl; 
		  return rawVoltage * batt_v_divider;
	}         
	else if(source ==  SYS_ZH) {
		divider = 10;
		cout << "System ZH" << "\t" << rawVoltage << "\t" << zh_divider << endl; 
		return rawVoltage * zh_divider;
	}     
	else if(source ==  ZP) {
		divider = 10;
		cout << "ZP" << "\t" << rawVoltage << "\t" << zp_divider << endl; 
		return rawVoltage * zp_divider;
	}
	else if(source ==  ZH) {
		divider = 10;
		cout << "Feedback ZH" << "\t" << rawVoltage << "\t" << zh_divider << endl; 
		return rawVoltage * zh_divider;
	}            
	else if(source ==  PLUS_5R) {
		divider = 2.0;
		cout << "Feedback 5V" << "\t" << rawVoltage << "\t" << p5v_divider << endl; 
		return rawVoltage * p5v_divider;
	}         
	else if(source ==  BEAM) {
		divider = 1;
		//   cout << "Beam" << "\t" << rawVoltage <<"\t" << divider << endl; 
		  return rawVoltage * beam_divider;
	}       

	else {
		perror("CS0 MUX: Invalid selection.");
		return -1;
	}
}






float measureVoltage(int source) {
	float volts, rawVoltage;
	rawVoltage = getAvgVolts();
	volts = checkDivider(source, rawVoltage);

	return volts;
}










float getThermistorResistance(float thermistorVoltage) {
	float refVolts = p5r;
	float thermistorCurrent = ((refVolts - thermistorVoltage) / 6980);
	float thermistorResistance = thermistorVoltage / thermistorCurrent;
	// cout << "Thermistor resistance = " << thermistorResistance << endl;
	return thermistorResistance;
}



double getTempC(double tempK) {
	// cout << tempK;
	return tempK - 273.15;
}

float getTemperature(float thermistorResistance) {
	// cout << "Thermistor resistance = " << thermistorResistance << endl;
	double c = 1.27470e-7;
	double b = 0.000221061;
	double a = 0.000935397; 
	double tempK;
	double x = log(thermistorResistance);
	double y = pow(x, 3);


	tempK = 1.0 / (a + b * x + c * y);
	// tempK = 1 / ( a + b * log(thermistorResistance) + c * (log(thermistorResistance))^3);
	return (float)getTempC(tempK);
}


float measureLid() {
	float volts, rawVoltage;
	unsigned char receive[3] = { 0 };
	unsigned char transmit[3] = { 0 };
   
	selectSourceMUX(THERMISTORS);
	selectThermistorLevelMUX(LID);
	usleep(1000);
	rawVoltage = getAvgVolts();
	float res1 = getThermistorResistance(rawVoltage);
	lidThermistor = getTemperature(res1);
	printf("\tLid:  %4.2f\n", lidThermistor);
}



float measureThermistors() {
	float volts, rawVoltage;
	unsigned char receive[3] = { 0 };
	unsigned char transmit[3] = { 0 };
   
	selectSourceMUX(THERMISTORS);

   
	if (meterThermistorUsed) {
		selectThermistorLevelMUX(METER_1);
		usleep(1000);
      

		rawVoltage = getAvgVolts();
		float res1 = getThermistorResistance(rawVoltage);
		meterThermistor1 = getTemperature(res1);
        
		// cout << "Meter 1: " << meterThermistor1 << endl;
        
        
		selectThermistorLevelMUX(METER_2);
		usleep(1000);

		rawVoltage = getAvgVolts();
		float res2 = getThermistorResistance(rawVoltage);
		meterThermistor2 = getTemperature(res2);
		// cout << "Meter 2: " << meterThermistor2 << endl;
      
		meterHeaterThermistorAverage = (meterThermistor1 + meterThermistor2) / 2;
		//   cout << "Meter Avg: " << meterHeaterThermistorAverage << endl;
      
	}
   
	if (conningTowerThermistorUsed) {
		selectThermistorLevelMUX(CONNING_TOWER);
		usleep(1000);
		rawVoltage = getAvgVolts();
		float res1 = getThermistorResistance(rawVoltage);      
		conningTowerThermistor = getTemperature(res1);
      
		// cout << "Conning Tower: " << conningTowerThermistor << endl;
	}

	if (arrestmentThermistorUsed) {
		selectThermistorLevelMUX(ARRESTMENT);
		usleep(1000);
		rawVoltage = getAvgVolts();
		float res1 = getThermistorResistance(rawVoltage);    
		arrestmentThermistor = getTemperature(res1);
      
		//   cout << "Arrestment: " << arrestmentThermistor << endl;
	}

	if (gearboxThermistorUsed) {
		selectThermistorLevelMUX(GEARBOX);
		//need loop for average temp
		usleep(1000);
		rawVoltage = getAvgVolts();
		float res1 = getThermistorResistance(rawVoltage);
		gearboxThermistor = getTemperature(res1);

		//   cout << "Gearbox: " << gearboxThermistor << endl;
	}
	if (lidThermistorUsed) {
		selectThermistorLevelMUX(LID);
		usleep(1000);
		rawVoltage = getAvgVolts();
		float res1 = getThermistorResistance(rawVoltage);
		lidThermistor = getTemperature(res1);
		if (lidThermistor < lidAlarmLow) {
			lidThermistorInt = 0x00;
		}
		else if (lidThermistor < lidAlarmHigh) {
			lidThermistorInt = 0xFF;
		}
		else {
			lidThermistorInt = 0x69;
		}
      
		//   cout << "Lid: " << lidThermistor  << endl;
	} 
    
    
	// cout << "Meter Avg: " << meterHeaterThermistorAverage << "\t";
	// cout << "Conning Tower: " << conningTowerThermistor << "\t";
	// cout << "Arrestment: " << arrestmentThermistor << "\t";
	// cout << "Gearbox: " << gearboxThermistor << "\t";
	// cout << "Lid: " << lidThermistor  << "\n" << endl;
    
	printf("Meter Avg:  %4.2f", meterHeaterThermistorAverage);
	printf("\tConning Tower:  %4.2f", conningTowerThermistor);
	printf("\tArrestment:  %4.2f", arrestmentThermistor);
	printf("\tGearbox:  %4.2f", gearboxThermistor);
	printf("\tLid:  %4.2f\n", lidThermistor);
    
    
    
    
    
	return 0;
   
}



float measAdcAvg(int avgCount) {
	float rawVoltage, averageVoltage;
	float summVoltage = 0;
	unsigned char receive[3] = { 0 };
	unsigned char transmit[3] = { 0 };   
	for (int x = 0; x < avgCount; x++) {
		ADC.transfer(transmit, receive, 3);
		rawVoltage = (receive[1] << 8) + receive[2];
		rawVoltage = getVoltage(rawVoltage);
		summVoltage += rawVoltage;
		usleep(1000);
	}
	averageVoltage = summVoltage / (float) avgCount;
	return averageVoltage;
}


int levelOk(float crossValue, float longValue) {
	// cout << crossValue << "\t" << longValue << endl;
	if(crossValue < -1.0 || crossValue > 1.0) {
		cout << "Bad cross" << endl;
		return 0;
	}
	else if(longValue < -1.0 || longValue > 1.0) {
		cout << "Bad Long" << endl;
		return 0;
	}
	else { 
		return 1;
	}
}



int measureLevels() {
	float volts, rawVoltage;
	unsigned char receive[3] = { 0 };
	unsigned char transmit[3] = { 0 };
   
	selectSourceMUX(THERMISTORS);
	selectThermistorLevelMUX(CROSS);

	usleep(10000);
	float x = measAdcAvg(100);
	// Range = crossMax - crossMin
	// cout << " Cross Max " << crossMax << endl;
	// cout << "Meas " << x << endl;
	cout << "Cross: " << x << "Zero " << crossLevelZero << endl;
	crossShort = (unsigned short)((crossMax - x) * crossNormFactor);
	cout << "Normalized Cross level = 0x" <<  hex << crossShort << endl; 
	crossValue =  x; //(x - crossLevelZero) / .002 / 60;

   
	selectThermistorLevelMUX(LONG);
	usleep(10000);   
	x = measAdcAvg(100);
	cout << "Long: " << x << "Zero " << longLevelZero << endl;
	longShort = (unsigned short)((longMax - x) * longNormFactor);
	cout << "Normalized Long level = 0x" <<  hex << longShort << endl; 
	longValue =  x; //(x - longLevelZero) / .002 / 60;

	// printf ("Cross: %4.4f Arc Minutes\n", crossValue);
	// printf ("Long:  %4.4f Arc Minutes\n", longValue);
	//   if (levelOk(crossValue, longValue) == 0){
	//       cout << "Cross:" << "\t" <<  crossValue << " Arc Minutes" <<endl;
	//       cout << "Long:"  << "\t" <<  longValue  << " Arc Minutes" <<endl;
	//   }
	   return levelOk(crossValue, longValue);
   
}



int measSystemVoltages() {
	// cout << "Sys voltages" << endl;
   
	selectSourceMUX(SYSTEM_VOLTAGES);
   
	selectSysVoltageMUX(SYS_3P3);
	p3p3v = measureVoltage(SYS_3P3);
   
	selectSysVoltageMUX(SYS_BBB_5);
	bbb5  = measureVoltage(SYS_BBB_5);
   
	selectSysVoltageMUX(SYS_5R);
	p5r   = measureVoltage(SYS_5R);
   
	selectSysVoltageMUX(SYS_M5);
	m5v   = measureVoltage(SYS_M5);
   
	selectSysVoltageMUX(SYS_12);
	p12v  = measureVoltage(SYS_12);
   
	selectSysVoltageMUX(SYS_BATT_1);
	batt1v = measureVoltage(SYS_BATT_1);
   
	selectSysVoltageMUX(SYS_BATT_2);
	batt2v = measureVoltage(SYS_BATT_2);
   
	selectSysVoltageMUX(SYS_ZH);
	zhv   = measureVoltage(SYS_ZH);
   
   
   
   
	selectSourceMUX(FEEDBACK);
	selectFB_MUX(PLUS_5R);
	p5r_fb = measureVoltage(PLUS_5R);
   
	selectFB_MUX(ZP);
	zpv = measureVoltage(ZP);
   
	selectFB_MUX(ZH);
	zh_fb = measureVoltage(ZH);
   
   
	debug = true;
	if (debug == true) {
		//   cout << "Verifying system voltages" << endl;
		  cout << "System 3.3V:"        <<  "\t\t" << p3p3v << endl;
		cout << "System BBB 5.0V:"    <<  "\t" << bbb5 << endl;
		cout << "System 5.0V:"        <<  "\t\t" << p5r << endl;
		cout << "System 12.0V:"       <<  "\t\t" << p12v << endl;
		cout << "System Battery 1:"   <<  "\t" << batt1v << endl;
		cout << "System Battery 2:"   <<  "\t" << batt2v << endl;
		cout << "System -5V:"         <<  "\t\t" << m5v << endl;
		cout << "System ZH: " <<  "\t\t" << zhv << endl;   
		cout << "System FB ZH: "  << "\t\t" << zh_fb << endl;   
		cout << "System FB ZP: "  << "\t\t" << zpv << endl;   
		cout << "System +5R: " << "\t\t" << p5r_fb << "\t" << endl;   
	}
	return 0;
}

int setHeaters() {
   
	if (meterThermistorUsed) {
		if (meterHeaterFET.setValue(meterHeaterStatus) != 0) {
			cout << "Meter Heater failed"  << endl;
			// log to file
		}   
		// cout << "Meter Heater status " << meterHeaterStatus << endl;
	}
   
   
	if (gearboxThermistorUsed) {
		if (gearboxHeaterFET.setValue(gearboxHeaterStatus) != 0) {
			cout << "Gearbox Heater failed"  << endl;
			// log to file
		}
		//  cout << "Gearbox Heater status " << gearboxHeaterStatus << endl;
	}
    
	if (arrestmentThermistorUsed) {
		if (arrestmentHeaterFET.setValue(arrestmentHeaterStatus) != 0) {
			//  if (arrestmentHeaterFET.setValue(arrestmentHeaterStatus) != 0){
			   cout << "Arrestment Heater failed"  << endl;
			// log to file
		}   
		// cout << "Arrestment Heater status " << arrestmentHeaterStatus << endl;
	}
   
	if (conningTowerThermistorUsed) {
		if (conningTowerFET.setValue(conningTowerHeaterStatus) != 0) {
			//  if (arrestmentHeaterFET.setValue(arrestmentHeaterStatus) != 0){
			   cout << "Conning Tower Heater failed"  << endl;
			// log to file
		}   
		// cout << "Conning Tower Heater status " << conningTowerHeaterStatus << endl;
	}
	if (boardThermistorUsed) {
      
	}
   
	return 0;
   
}

int updateHeaterStatus() {
	if (meterThermistorUsed) {
		if (meterHeaterThermistorAverage < meterHeaterSetpoint - 0.2) {
			meterHeaterStatus = HIGH;
		}
		else if (meterHeaterThermistorAverage > meterHeaterSetpoint + 0.2) {
			meterHeaterStatus = LOW;
		}
      
	}
	if (gearboxThermistorUsed) {
		if (gearboxThermistor < gearboxHeaterSetpoint - 0.2) {
			gearboxHeaterStatus = HIGH;
		}
		else if (gearboxThermistor > gearboxHeaterSetpoint + 0.2) {
			gearboxHeaterStatus = LOW;
		}
	}
	if (arrestmentThermistorUsed) {
		if (arrestmentThermistor < arrestmentHeaterSetpoint - 0.2) {
			arrestmentHeaterStatus = HIGH;
		}
		else if (arrestmentThermistor < arrestmentHeaterSetpoint + 0.2) {
			arrestmentHeaterStatus = LOW;
		}
	}    
	if (conningTowerThermistorUsed) {
		if (conningTowerThermistor < conningTowerHeaterSetpoint - 0.2) {
			conningTowerHeaterStatus = HIGH;
		}
		else if (conningTowerThermistor > conningTowerHeaterSetpoint + 0.2) {
			conningTowerHeaterStatus = LOW;
		}
	}  

	setHeaters();
	return 0;
}



void *checkHeaters(void *value) { 
	int heaterTimeIncrement = 10;
	time_t now = time(0);
	if (debug) {
		cout << "checkHeaters thread started" << endl;
	}
	while (landRun == 1) {
        
		tm *ltm = localtime(&now);
		usleep(10e6);
		// if ((int)ltm->tm_sec == heaterCheckTime){
		     heaterCheckTime += heaterTimeIncrement;
		measureThermistors();
		updateHeaterStatus();
		// }
	}
}  




float measureBeam() {
	selectSourceMUX(FEEDBACK);
	selectFB_MUX(BEAM);
	beam = measureVoltage(BEAM);
	float beamTemp = (beam / ADC_REF * 65535);
	beamShort = (unsigned short)round(beamTemp);
	cout << "Beam temp " << beamTemp << endl;
	cout << "Beam " << beamShort << "\t" << hex << "0x" << beamShort << endl;
	return beam;
}


void *heartBeat(void *value) {
	if (debug) {
		cout << "heart Beat thread started"  << endl;
	}
	while (landRun == 1) {
		time_t now = time(0);
		tm *ltm = localtime(&now);
		unsigned char command_bytes[80];
   
		if (inputBuffer.size() > 1) {
			unsigned char command_bytes[80];
			// cout << "Buffer size " << inputBuffer.size() << endl;
			int commandSize;
			commandSize = (int)inputBuffer.front();
			cout << "Bytes to follow " << commandSize << endl;
			// inputBuffer.pop();
			// unsigned char inputBuffer[commandSize - 1];
			// command_bytes[0] = commandSize;
			for(int x = 0 ; x <= commandSize ; x++) {
				command_bytes[x] = inputBuffer.front();
				inputBuffer.pop();
			}
            
			printDataSet(command_bytes, commandSize + 1);
			// cout << "Buffer size " << inputBuffer.size() << "\n" << endl;
			parse_commands(command_bytes);
		}

		if (ltm->tm_sec == heartBeatChecktime) {
			cout << ltm->tm_sec << "\t" << heartBeatChecktime << endl;
			heartBeatChecktime += 10;
			if (heartBeatChecktime > 59)
			{
				heartBeatChecktime = 0;
			}
			oneHzData = true;
			if (oneHzData) {
				beam = measureBeam();
				Send_Current_Data_Set();                

			}
		}
		usleep(1e5);  //5e6      
	}
}






void *getCommands(void *value) {
	string command;
	PWM PWM1A("3", "0");
	usleep(10000000);     // allow time for file system to export files
	while(landRun) {
		// CHECK FOR POWER DOWN BUTTON

		cout << "Enter Command ";   
		cin >> command;
		if (command == "quit") {
			cout << "Shutting down heaters" << endl;
			arrestmentHeaterFET.setValue(LOW);
			conningTowerFET.setValue(LOW); 
			meterHeaterFET.setValue(LOW);
			gearboxHeaterFET.setValue(LOW);
			boardHeaterFET1.setValue(LOW);
			boardHeaterFET2.setValue(LOW);
			landRun = 0;
		}
		else if (command == "volts") {
			selectSourceMUX(SYSTEM_VOLTAGES);
			cout << "Reading Volts" << endl;
			measSystemVoltages();
		}
        
		else if (command == "load") {
			getDividers();
		}
		else if (command == "temp") {
			measureThermistors();
		}
		else if (command == "lid") {
			measureLid();
		}
		else if (command == "beam") {
			cout << "Measuring beam...";
			cout << measureBeam() << endl;
		}
		else if (command == "levels") {
			measureLevels();
		}
		else if (command == "pwm") {
			int dutyCycle;
			// 5%    F331 ‭62257
			// 10%   E664 58980
			// 50%   7FFF ‭32767
			// 90%   1999 6553
			// 98%   051E ‭1310‬
		  cout << "Enter duty cycle: ";
			cin >> dutyCycle;
			setPWMDutyCycle(PWM1A, dutyCycle);
		} 
        
		else if (command == "read") {
			spi.writeRegister(0x2D, 0x08);   // POWER_CTL for the ADXL345
		// spi.readRegisters(0xF);
		}
	}
}








//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
	// SET DEBUG TO TRUE FOR INITIAL STARTUP EVERY TIME
	// SET TO FALSE AT END OF STARTUP
   
	//   Heartbeat = new GPIO(87);  
   
   
	   bool levelsOk                          =     false;
	char buff[DTTMSZ];
	// these should be in a .h file or in a heater class
     
	getDividers(); // Get voltage divider from file
   
	filestr.open("start.log", fstream::out | fstream::app);
	filestr << getDtTm(buff) << "Started program" << std::endl;
	filestr.close();
	// Setup GPIO etc.  
	// OUTPUTS   
	// GPIO outGPIO(37);//, inGPIO(115), 
	if(debug == true) {
		cout << "Initializing GPIO" << endl;
	}
   
	// Make global??
	// system("./config_pins.sh"); // myfile.sh should be chmod +x
   
 
	// Set GPIO Direction 
	   if(debug == true) {
		cout << "Setting GPIO direction" << endl;
	}
	systemPowerEnable.setDirection(OUTPUT);
	arrestmentHeaterFET.setDirection(OUTPUT);
	conningTowerFET.setDirection(OUTPUT);     
	meterHeaterFET.setDirection(OUTPUT);      
	gearboxHeaterFET.setDirection(OUTPUT);  
	boardHeaterFET1.setDirection(OUTPUT);    
	boardHeaterFET2.setDirection(OUTPUT);  
	SHAFT_DIR.setDirection(OUTPUT);        
	SHAFT_PWM.setDirection(OUTPUT);        
	ARRESTMENT_DIR.setDirection(OUTPUT);      
	ARRESTMENT_PWM.setDirection(OUTPUT);      
	CROSS_DIR.setDirection(OUTPUT);          
	CROSS_PWM.setDirection(OUTPUT);          
	LONG_DIR.setDirection(OUTPUT);          
	LONG_PWM.setDirection(OUTPUT);            
	FB_MUX_A0.setDirection(OUTPUT);           
	FB_MUX_A1.setDirection(OUTPUT);    
   
	thermistor_MUX_A0.setDirection(OUTPUT);  
	thermistor_MUX_A1.setDirection(OUTPUT);  
	thermistor_MUX_A2.setDirection(OUTPUT);    
	thermistor_MUX_A3.setDirection(OUTPUT);   
   
	SPI0_CS0_MUX_A0.setDirection(OUTPUT);    
	SPI0_CS0_MUX_A1.setDirection(OUTPUT);    
	SPI0_CS0_MUX_A2.setDirection(OUTPUT);    
	// Read cal file.  Set offsets etc.
  



	/***********************************************
	 *             Heater startup
	 ***********************************************/


	   if(debug == true) {
		cout << "Starting Arrestment Heater" << endl;
	}
	if (arrestmentHeaterFET.setValue(LOW) != 0) {
		cout << "Arrestment Heater failed to start"  << endl;
	}                          // start the PWM output
	else if(debug == true) {
		cout << "Arrestment Heater running" << endl;
	}


   
	if (debug == true) {
		cout << "Starting Conning Tower Heater" << endl;
	}
	if (conningTowerFET.setValue(LOW) != 0) {
		cout << "Conning Tower Heater failed to start"  << endl;
	}                          // start the PWM output
	else if(debug == true) {
		cout << "Conning Tower Heater running" << endl;
	}  
   
   
   
	if (debug == true) {
		cout << "Starting Meter Heater" << endl;
	}
	if (meterHeaterFET.setValue(LOW) != 0) {
		cout << "Meter Heater failed to start"  << endl;
	}                          // start the PWM output
	else if(debug == true) {
		cout << "Meter Heater running" << endl;
	} 
   
   
   
	if (debug == true) {
		cout << "Starting Gearbox Heater" << endl;
	}
	if (gearboxHeaterFET.setValue(LOW) != 0) {
		cout << "Gearbox Heater failed to start"  << endl;
	}                          // start the PWM output
	else if(debug == true) {
		cout << "Gearbox Heater running" << endl;
	} 
   
   
	if (debug == true) {
		cout << "Starting Board Heater #1" << endl;
	}
	if (boardHeaterFET1.setValue(LOW) != 0) {
		cout << "Board Heater #1 failed to start"  << endl;
	}                          // start the PWM output
	else if(debug == true) {
		cout << "Board Heater #1 running" << endl;
	} 
   
   
	if (debug == true) {
		cout << "Starting Board Heater #2" << endl;
	}
	if (boardHeaterFET2.setValue(LOW) != 0) {
		cout << "Board Heater #2 failed to start"  << endl;
	}                          // start the PWM output
	else if(debug == true) {
		cout << "Board Heater #2 running" << endl;
	}   
   
   
   
   
   
   
   
   
   
	/***********************************************
	 *             Initialize SPI
	 ***********************************************/   
	if (debug == true) {
		cout << "Initializing SPI" << endl;
	}

	setUpSPI_0();
	setUpSPI_1();

	// SPIDevice spi(0,0);             // chip select 0 on bus 1
	// spi.debug = true;
	spi.setSpeed(100000);           // set the speed to 1 MHz
	// cout << "The device ID is: " << (int) spi.readRegister(0x00) << endl;
	spi.setMode(SPIDevice::MODE3);   // set the mode to Mode3
	spi.setBitsPerWord(8);
	// cout << "Bits is: "  << spi.bits << endl;
	// //spi.writeRegister(0x2D, 0x08);  // POWER_CTL for the ADXL345
	// spi.readRegisters(0xF);


  
   
	// Setup serial port for Bluetooth   
	   if(debug == true) {
		cout << "Initializing Bluetooth Communications" << endl;
	}   
	serialComm = open("/dev/ttyO4", O_RDWR | O_NOCTTY | O_SYNC);
	if (serialComm < 0) {
		printf("Error opening %s: %s\n", "/dev/ttyO4", strerror(errno));
		return -1;
	}
	else {
		cout << "/dev/ttyO4 open" << endl; // Serial 4
	}
   
	set_interface_attribs(serialComm, B9600); /*baudrate 9600, 8 bits, no parity, 1 stop bit */





	if (debug == true) {
		cout << "Initializing PWM" << endl;
	}

	if (debug == true) {
		cout << "Exporting Forcing plates PWM" << endl;
	}
	PWM PWM1A("3", "0");
	usleep(10000000);     // allow time for file system to export files
   
	   if(debug == true) {
		cout << "Exporting ALT Sense plate PWM" << endl;
	}
	PWM PWM2A("6", "0");
	usleep(10000000);     // allow time for file system to export files
   
	   if(debug == true) {
		cout << "Exporting Sense plate PWM" << endl;
	}
	PWM PWM2B("6", "1"); 
	usleep(10000000);     // allow time for file system to export files

  
   
	/**********************************************************
	*            Initialize Force plates
	**********************************************************/
   
	if(debug == true) {
		cout << "Initializing Forcing plates PWM" << endl;
	}
	PWM1A.setFrequency(125);
	PWM1A.setDutyCycle(50.0f);       
	PWM1A.setPolarity(PWM::ACTIVE_LOW);  
	if (PWM1A.run() != 0) {
		cout << "Forcing plates PWM failed to start"  << endl;
	}                          // start the PWM output
	else if(debug == true) {
		cout << "Forcing plates PWM running" << endl;
	}                    
   
	/**********************************************************  
	*            Initialize ALT Sense plates
	**********************************************************/
   
	if (debug == true) {
		cout << "Initializing Sense plates ALT PWM" << endl;
	}
	PWM2A.setFrequency(65000);       
	PWM2A.setDutyCycle(50.0f);       
	PWM2A.setPolarity(PWM::ACTIVE_LOW);  
	if (PWM2A.run() != 0) {
		cout << "Sense plates ALT PWM failed to start"  << endl;
	}                          // start the PWM output
	else if(debug == true) {
		cout << "Sense plates ALT PWM running" << endl;
	}                    
                       
	/**********************************************************  
	*             Initialize Sense plates
	**********************************************************/
   
   
	if (debug == true) {
		cout << "Initializing Sense plates PWM" << endl;
	}
	PWM2B.setFrequency(65000);        
	PWM2B.setDutyCycle(50.0f);       
	PWM2B.setPolarity(PWM::ACTIVE_LOW);  
	if (PWM2B.run() != 0) {
		cout << "Sense plates PWM failed to start"  << endl;
	}                          // start the PWM output
	else if(debug == true) {
		cout << "Sense plates PWM running" << endl;
	} 




	// [4, 2, 243,   49,   196]  = 5%    F331 ‭62257‬
	// [4, 2, 230,   100,  132]  = 10%   E664
	// [4, 2, 127,   255,  134]  = 50%   7FFF ‭32767‬
	// [4, 2, 25,    153,  134]  = 90%   1999 6553
	// [4, 2, 5,     30,   29]   = 98%   051E ‭1310‬


	//   if(debug == true){
	//       cout << "Initializing 1Hz timer" << endl;
	//   }

	    crossNormFactor = 65535 / (crossMax - crossMin);
	longNormFactor  = 65536 / (longMax - longMin);
    
	cout << "Cross Normalization Factor: "  <<  crossNormFactor << endl;
	cout << "Long Normalization Factor: "  <<  longNormFactor << endl;
   
	int x = 0;

	// checkForData();
   if(debug == true) {
		cout << "Program startup complete. Entering main loop" << endl;
		debug = false;
	}
   
	measSystemVoltages();
	time_t now = time(0);
	tm *ltm = localtime(&now);
    
	heaterCheckTime = (int)ltm->tm_sec + 1;
	voltageCheckTime = (int)ltm->tm_sec + 1; 
	heartBeatChecktime = (int)ltm->tm_sec + 1;
   
	/**********************************************************  
	*        Initialize Communications thread for Bluetooth
	**********************************************************/  
   
	pthread_t commThread;
	if (pthread_create(&commThread, NULL, &checkForData, &x) != 0) {
		cout << "Failed to create comm thread" << endl;
		return 1;
	}    
   
   
  
	/**********************************************************  
	*        Initialize Heater Thread
	**********************************************************/    

	pthread_t heaterThread;  
	// if(pthread_create(&heaterThread, NULL, &checkHeaters, &x)!=0){
	//     cout << "Failed to create the heater thread" << endl;
	// 	return 1;
	// } 

	/**********************************************************  
	*        Initialize Heatbeat Thread
	**********************************************************/    

	pthread_t heartbeatThread; 
	// time_t now = time(0);
	// tm *ltm = localtime(&now);
	heartBeatChecktime = 1 + ltm->tm_sec;
	// if(pthread_create(&heartbeatThread, NULL, &heartBeat, &x)!=0){
	//     cout << "Failed to create the Heart beat thread" << endl;
	// 	return 1;
	// }
  
	/**********************************************************  
	*        Initialize Command thread for debug
	**********************************************************/  
   
   pthread_t commandThread;
	if (pthread_create(&commandThread, NULL, &getCommands, &x) != 0) {
		cout << "Failed to create command thread" << endl;
		return 1;
	}    
  
	while (landRun) {
		usleep(10000000);     // allow time for file system to export files
	}
	/**********************************************************  
	*        Allow threads to die
	**********************************************************/  
    
	void* result;              // OPTIONAL: receive data back from pthread
	// pthread_join(checkHeaters, &result);   // allow the pthread to complete 
	// pthread_join(heartBeat, &result);
	// pthread_join(getCommands, &result);
	// pthread_join(checkForData, &result);
   // selectSourceMUX(1);
   // usleep(10000000);    // allow time for file system to export files

   cout << "Exiting" << endl;

	return 0;
}
/*      Issues
    1)  Battery 1 not measuring correctly.  Always 19.7V
    2)  ZP not measuring correctly.  Measures 32.7V s/b 36V.  Measures correctly on main board but not feedback board.
    3)  Op-amp for meter heater not working - replace




        Normalized frequencies
    Beam    10% - 7kHz  50% - 8KHz  90% - 9KHz
    Levels  -2 Arc Min - 8KHz   Level - 10KHz   +2 Arc Min - 12KHz
    Lid     Low - 0     Good - (100 - 110)      High - 256
    
            Voltages
    Beam    10% -   50% -   90% - 
    Levels  -2 Arc Min -    Level -    +2 Arc Min - 

*/
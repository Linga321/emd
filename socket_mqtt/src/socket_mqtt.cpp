/*
===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
 */

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

// TODO: insert other include files here
#include <cstdio>
#include <cstring>
#include "systick.h"
#include "LpcUart.h"
#include "esp8266_socket.h"
#include "retarget_uart.h"
#include "ModbusMaster.h"
#include "ModbusRegister.h"
#include "MQTTClient.h"

#include "DigitalIoPin.h"
#include "LiquidCrystal.h"

// TODO: insert other include files here
#include <atomic>
#include <mutex>
#include <cctype>
#include <stdio.h>
#include <math.h>
#if defined(BOARD_NXP_LPCXPRESSO_1549)
/** 8-bit I2C addresses of presser Sensor */
#define I2C_PRES_ADDR_8BIT  (0x40)
#endif

#define TICKRATE_HZ1 (1000)	/* 15 ticks per second */
/* I2C clock is set to 50kHz */
#define I2C_CLK_DIVIDER         (1440)
/* 100KHz I2C bit-rate */
#define I2C_BITRATE         (50000)
/* Standard I2C mode */
#define I2C_MODE    (0)

#define I2C_STMPE811_ADDR_7BIT 0x40

static volatile int state;
/* I2CM transfer record */
static I2CM_XFER_T  i2cmXferRec;

#define SSID "LingaMobile"
#define PWORD "123456lk"
#define MQTTADD "18.198.188.151" // connection adress 192.168.1.254
#define MQTT_PORT 21883 // connection port 1883
// TODO: insert other definitions and declarations here


// TODO: insert other definitions and declarations here
static volatile int counter;
static volatile uint32_t systicks;

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
void SysTick_Handler(void)
{
	systicks++;
	if(counter > 0) counter--;
}

uint32_t get_ticks(void)
{
	return systicks;
}

#ifdef __cplusplus
}
#endif

void Sleep(int ms)
{
	counter = ms;
	while(counter > 0) {
		__WFI();
	}
}

/* this function is required by the modbus library */
uint32_t millis() {
	return systicks;
}

// Pressure sensor related function
void Init_I2C_PinMux(void);
void setupI2CMaster();
void SetupXferRecAndExecute(
		uint8_t devAddr,
		uint8_t *txBuffPtr,
		uint16_t txSize,
		uint8_t *rxBuffPtr,
		uint16_t rxSize);
int ReadSensorStatus(void);
void SetSensorStatus(uint8_t mode);
int16_t ReadPresserI2CM(void);
// Modbus related function
bool setFrequency(ModbusMaster& node, uint16_t freq);
void printRegister(ModbusMaster& node, uint16_t reg);
void modBusInit(ModbusMaster& node);
void setFanSpeed(ModbusMaster& node);
// LCD display related function
void displayLcd(LiquidCrystal& lcd, char *display_mesg);

// Main function
void mqttTest();


#if 1

struct MQTTDATA
{
	int setpoint =10;
	int speed;
	int pressure;
	int err =0;
	int auto_ =1;
};

static MQTTDATA mqttdata;

int main(void) {

#if defined (__USE_LPCOPEN)
	// Read clock settings and update SystemCoreClock variable
	SystemCoreClockUpdate();
#if !defined(NO_BOARD_LIB)
	// Set up and initialize all required blocks and
	// functions related to the board hardware
	Board_Init();
	// Set the LED to the state of "On"
	Board_LED_Set(0, true);
#endif
#endif
	Chip_RIT_Init(LPC_RITIMER);
	Chip_Clock_SetSysTickClockDiv(1);
	// this call initializes debug uart for stdout redirection
	retarget_init();
	/* Set up SWO to PIO1_2 */
	Chip_SWM_MovablePortPinAssign(SWM_SWO_O, 1, 2); // Needed for SWO printf
	/* set I2C Chip_IOCON_PinMuxSet */
	Init_I2C_PinMux();
	/* Allocate I2C handle, setup I2C rate, and initialize I2C clocking */
	setupI2CMaster();
	/* Disable the interrupt for the I2C */
	NVIC_DisableIRQ(I2C0_IRQn);
	/* Enable and setup SysTick Timer at a periodic rate */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ1);
	/* System starts print statement */
	printf("\nBoot\n");
	/* starts main programs that uses mqtt connection  */

	// Enter an infinite loo
	mqttTest();



	// Enter an infinite loop, just incrementing a counter
	while(1) {
		// "Dummy" NOP to allow source level single
		// stepping of tight while() loop
		__asm volatile ("nop");
	}
	return 0 ;
}
#endif


#if 1

char * jsonDecode(char *str1){
	int i, j = 0;
	for (i = 0; str1[i] != '}'; i++) {
		if (str1[i] != '"' && str1[i] != ',' && str1[i] != '{' && str1[i] != '}' && str1[i] != ':') {
			str1[j] = str1[i];
			j++;
		}
	}
	str1[j] = '\0';
	return str1;
}

void messageArrived(MessageData* data)
{
	printf("Message arrived on topic %.*s: %.*s\n", data->topicName->lenstring.len, data->topicName->lenstring.data,
			data->message->payloadlen, (char *)data->message->payload);
	char * result = jsonDecode((char *)data->message->payload);


	int setP = 0;
	int state = 0;
	int i = 0, j = 0;
	char target1[] = "speed";
	char target2[] = "pressure";
	char target3[] = "false";
	char *p1 = strstr(result, target1); // finding the text
	char *p2 = strstr(result, target2);
	char *p3 = strstr(result, target3);

	if(p3) {
		state = 0;
	}
	else {
		state = 1;
	}

	char *token = strtok(result," ");

	// loop until strtok() returns NULL

	while (token)  {
		// print token
		if(i == 3) {
			setP = std::atoi(token);
		}
		// take subsequent tokens
		token = strtok(NULL," ");
		i++;
	}

	if(p1) {
		printf("Speed");
		mqttdata.setpoint = setP;
		mqttdata.err = false;
		mqttdata.auto_ = state;
	}
	else if(p2) {
		mqttdata.setpoint = setP;
		mqttdata.err = false;
		mqttdata.auto_ = state;
	}
	else {
		mqttdata.err = true;
	}
}

void mqttTest()
{
	/* connect to mqtt broker, subscribe to a topic, send and receive messages regularly every 1 sec */
	int ctr =0;
	char str_stauts[2][6] ={ "false", "true"}; // for display text instead of 0,1

	char buffer[17];
	bool atSetpoint = true;
	bool btn_press = false;
	const int delay = 100;
	unsigned char sendbuf[256], readbuf[2556];
	int rc = 0, i=0,
			count = 0;


	DigitalIoPin sw1(0,10,true, true, false);
	DigitalIoPin sw2(0,9,true, true, false);
	DigitalIoPin sw3(0,24,true, true, false);
	Sleep(1000);

	DigitalIoPin rs(0,0,false,false,false);
	DigitalIoPin en(1,3,false,false,false);
	DigitalIoPin d4(1,8,false,false,false);
	DigitalIoPin d5(0,5,false,false,false);
	DigitalIoPin d6(0,6,false,false,false);
	DigitalIoPin d7(0,7,false,false,false);
	Sleep(10);
	LiquidCrystal lcd(&rs,&en,&d4,&d5,&d6,&d7);

	lcd.begin(16, 2);
	lcd.setCursor(0, 0);
	lcd.print("Booting");
	lcd.setCursor(0, 1);
	lcd.print("Wait");


	ModbusMaster node(2); // Create modbus object that connects to slave id 2

	modBusInit(node);  // init other modbus and connection

	MQTTClient client;
	Network network;
	MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;
	NetworkInit(&network,SSID,PWORD); // conn wifi
	MQTTClientInit(&client, &network, 30000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));

	do {
		// if mqtt lost connection
		char* address = (char *) MQTTADD;
		if ((rc = NetworkConnect(&network, address, MQTT_PORT)) != 0)
			printf("Return code from network connect is %d\n", rc);

		connectData.MQTTVersion = 3;
		connectData.clientID.cstring = (char *)"esp_test";

		if ((rc = MQTTConnect(&client, &connectData)) != 0)
			printf("Return code from MQTT connect is %d\n", rc);
		else
			printf("MQTT Connected\n");

		if ((rc = MQTTSubscribe(&client, "controller/settings", QOS2, messageArrived)) != 0)
			printf("Return code from MQTT subscribe is %d\n", rc);

		uint32_t sec = 0;
		while (atSetpoint)
		{
			ctr=0;
			// send one message per second

			if(get_ticks() / TICKRATE_HZ1 != sec) {
				MQTTMessage message;
				char payload[100];

				sec = get_ticks() / TICKRATE_HZ1;
				++count;

				message.qos = QOS1;
				message.retained = 0;
				message.payload = payload;
				sprintf(payload, "{ \"nr\": %d,\"speed\": %d,\"setpoint\": %d,\"pressure\": %d,\"auto\": %s,\"error\": %s}", count,mqttdata.speed,mqttdata.setpoint,mqttdata.pressure,str_stauts[mqttdata.auto_], str_stauts[mqttdata.err]);
				message.payloadlen = strlen(payload);
				if ((rc = MQTTPublish(&client, "controller/status", &message)) != 0)
					printf("Return code from MQTT publish is %d\n", rc);
			}

			// button events
			int setpnt=mqttdata.setpoint;
			int max = mqttdata.auto_ ? 120:100;
			while(sw1.read() && setpnt<max){
				Sleep(100);
				setpnt++;
			}

			while(sw3.read() && setpnt>0){
				Sleep(100);
				setpnt--;
			}

			if(mqttdata.setpoint != setpnt){

				snprintf(buffer, 16, "SETPOINT>%3d", setpnt);
				displayLcd(lcd, buffer);
				mqttdata.setpoint =setpnt;
				Sleep(2000);
			}
			setpnt =0;
			while(sw2.read()){
				btn_press=true;
				setpnt++;
				Sleep(100);
			}
			if(btn_press== true && setpnt >10){
				btn_press=false;
				mqttdata.auto_ = mqttdata.auto_ ? 0:1;
			}

			snprintf(buffer, 16, "SPE>%3d|PRS>%3d", mqttdata.speed,mqttdata.pressure);
			displayLcd(lcd, buffer);


			if(mqttdata.auto_ == 0){

				// after project demo update
				if(mqttdata.pressure > 120 && mqttdata.speed >0) { // when fan pres higher than 120,then lower the speed
					mqttdata.speed = mqttdata.speed-1;
					mqttdata.err = true;
				}else{ //
					if(mqttdata.setpoint != mqttdata.speed) {
						mqttdata.speed=mqttdata.setpoint;
						mqttdata.err = false;
					}
				}


				// frequency is scaled:
				// 20000 = 50 Hz, 0 = 0 Hz, linear scale 400 units/Hz
				setFanSpeed(node);
				Sleep(delay);
				mqttdata.pressure =  (int)ReadPresserI2CM();
			}


			if(mqttdata.auto_ == 1) {

				while(mqttdata.setpoint != mqttdata.pressure && mqttdata.speed<100 && mqttdata.speed>0) {
					if(mqttdata.setpoint > mqttdata.pressure) {
						mqttdata.speed = mqttdata.speed+1;
					}else if(mqttdata.setpoint < mqttdata.pressure) {
						mqttdata.speed = mqttdata.speed+1;
					}
					else {
						mqttdata.speed = mqttdata.speed-1;
					}
					mqttdata.pressure =  (int)ReadPresserI2CM();
					// frequency is scaled:
					// 20000 = 50 Hz, 0 = 0 Hz, linear scale 400 units/Hz
					setFanSpeed(node);
				}

				if(mqttdata.setpoint != mqttdata.pressure) {
					mqttdata.err=1;
				}
				else {
					mqttdata.err=0;
				}

				Sleep(1000);
				// frequency is scaled:
				// 20000 = 50 Hz, 0 = 0 Hz, linear scale 400 units/Hz
				setFanSpeed(node);
			}

			if(rc != 0) {// if network lost
				NetworkDisconnect(&network);
				Sleep(delay);
				atSetpoint = false;
			}
			// run MQTT for 100 ms
			if ((rc = MQTTYield(&client, 100)) != 0)
				printf("Return code from yield is %d\n", rc);
		}
		printf("MQTT connection closed!\n");

		ctr++;
	} while(ctr < 3 && !atSetpoint);// if network is lost try 3 times
}
#endif

# if 1 // function that uses modbus library directly

void printRegister(ModbusMaster& node, uint16_t reg)
{
	uint8_t result;
	// slave: read 16-bit registers starting at reg to RX buffer
	result = node.readHoldingRegisters(reg, 1);

	// do something with data if read is successful
	if (result == node.ku8MBSuccess)
	{
		printf("R%d=%04X\n", reg, node.getResponseBuffer(0));
	}
	else {
		printf("R%d=???\n", reg);
	}
}

bool setFrequency(ModbusMaster& node, uint16_t freq)
{
	uint8_t result;
	int ctr;
	bool atSetpoint;
	const int delay = 50;

	node.writeSingleRegister(1, freq); // set motor frequency

	printf("Set freq = %d\n", freq/40); // for debugging

	// wait until we reach set point or timeout occurs
	ctr = 0;
	atSetpoint = false;
	do {
		Sleep(delay);
		// read status word
		result = node.readHoldingRegisters(3, 1);
		// check if we are at setpoint
		if (result == node.ku8MBSuccess) {
			if(node.getResponseBuffer(0) & 0x0100) atSetpoint = true;
		}
		ctr++;
	} while(ctr < 20 && !atSetpoint);

	printf("Elapsed: %d\n", ctr * delay); // for debugging

	return atSetpoint;
}


void modBusInit(ModbusMaster& node) {
	node.begin(9600); // set transmission rate - other parameters are set inside the object and can't be changed here

	printRegister(node, 3); // for debugging

	node.writeSingleRegister(0, 0x0406); // prepare for starting

	printRegister(node, 3); // for debugging

	Sleep(1000); // give converter some time to set up
	// note: we should have a startup state machine that check converter status and acts per current status
	//       but we take the easy way out and just wait a while and hope that everything goes well

	printRegister(node, 3); // for debugging

	node.writeSingleRegister(0, 0x047F); // set drive to start mode

	printRegister(node, 3); // for debugging

	Sleep(1000); // give converter some time to set up
	// note: we should have a startup state machine that check converter status and acts per current status
	//       but we take the easy way out and just wait a while and hope that everything goes well

	printRegister(node, 3); // for debugging
}

void setFanSpeed(ModbusMaster& node)
{

	if(mqttdata.speed<7 && mqttdata.speed !=0 ) { // slow
		mqttdata.speed=7;
	}
	int j = 0;
	int Freq = round((float)mqttdata.speed *0.01*20000);
	const uint16_t fa = (uint16_t)Freq ;
	uint8_t result;
	// slave: read (2) 16-bit registers starting at register 102 to RX buffer
	do {
		result = node.readHoldingRegisters(102, 2);
		j++;
	} while(j < 20 && result != node.ku8MBSuccess);
	// note: sometimes we don't succeed on first read so we try up to threee times
	// if read is successful print frequency and current (scaled values)
	if (result == node.ku8MBSuccess) {
		printf("F=%4d, I=%4d  (ctr=%d)\n", node.getResponseBuffer(0), node.getResponseBuffer(1),j);
		state=0;
	}
	else {
		printf("ctr=%d\n",j);
	}
	setFrequency(node, fa);
}
#endif

# if 1

void displayLcd(LiquidCrystal& lcd, char *display_mesg){
	char str_mode[2][8] ={ "MANUAL", "AUTO"}; // for display text instead of
	lcd.begin(16, 2);
	lcd.clear();
	lcd.print(str_mode[mqttdata.auto_]);
	lcd.setCursor(0, 1);
	lcd.print(display_mesg);
}
#endif


/* Master I2CM receive in polling mode */
#if defined(BOARD_NXP_LPCXPRESSO_1549)
/* Function to read Sensor I2C presser sensor and output result */
void Init_I2C_PinMux(void)
{
#if defined(BOARD_KEIL_MCB1500)||defined(BOARD_NXP_LPCXPRESSO_1549)
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 22, IOCON_DIGMODE_EN | I2C_MODE);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 23, IOCON_DIGMODE_EN | I2C_MODE);
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SCL);
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SDA);
#else
#error "No I2C Pin Muxing defined for this example"
#endif
}

/* Setup I2C handle and parameters */

void setupI2CMaster()
{
	/* Enable I2C clock and reset I2C peripheral - the boot ROM does not do this */
	Chip_I2C_Init(LPC_I2C0);
	/* Setup clock rate for I2C */
	Chip_I2C_SetClockDiv(LPC_I2C0, I2C_CLK_DIVIDER);
	/* Setup I2CM transfer rate */
	Chip_I2CM_SetBusSpeed(LPC_I2C0, I2C_BITRATE);
	/* Enable Master Mode */
	Chip_I2CM_Enable(LPC_I2C0);
}

void SetupXferRecAndExecute(
		uint8_t devAddr,
		uint8_t *txBuffPtr,
		uint16_t txSize,
		uint8_t *rxBuffPtr,
		uint16_t rxSize){
	/* Setup I2C transfer record */
	i2cmXferRec.slaveAddr = devAddr;
	i2cmXferRec.status = 0;
	i2cmXferRec.txSz = txSize;
	i2cmXferRec.rxSz = rxSize;
	i2cmXferRec.txBuff = txBuffPtr;
	i2cmXferRec.rxBuff = rxBuffPtr;
	Chip_I2CM_XferBlocking(LPC_I2C0, &i2cmXferRec);
}


int ReadSensorStatus(void)
{
	uint8_t idAndRevision[3];
	uint8_t registerAddress = (0xF1);
	char arr[80];
	/* Read STMPE811 CHIP_ID and ID_VER registers */
	SetupXferRecAndExecute(
			/* The Sensor I2C bus address */
			I2C_PRES_ADDR_8BIT,
			/* Transmit one byte, the register address */
			&registerAddress, 1,
			/* Receive back three bytes, the contents of the CHIP_ID and ID_VER registers */
			idAndRevision, 3);

	/* Test for valid operation */
	if (i2cmXferRec.status == I2CM_STATUS_OK) {
		/* Output CHIP_ID and ID_VER values. */

		snprintf(arr,80,"\n\rSTMPE811 CHIP_ID:0x%04x ID_VER:0x%02x.\r\n", (int) (idAndRevision[0] << 8) | idAndRevision[1], idAndRevision[2]);
		Board_UARTPutSTR(arr); // print

	}
	else {
		snprintf(arr,80,"\n\rError %d reading CHIP_ID and ID_VER registers of STMPE811.\r\n",i2cmXferRec.status);
	}
}

void SetSensorStatus(uint8_t mode){
	uint8_t ConfigRegisterAddress = (0xF1);
	// to write to the register, two bytes back to back have to be sent
	uint8_t writedata[2] = {ConfigRegisterAddress,mode};
	SetupXferRecAndExecute(
			I2C_PRES_ADDR_8BIT,
			writedata, 2,
			NULL,0);
}

int16_t ReadPresserI2CM(void)
{
	uint8_t presser[3]; // reading data
	uint8_t PresserRegisterAddress = (0xF1); // the command for reading value
	uint8_t txSz = 1; // Transmit data size


	/* Read Sensor presser sensor */
	SetupXferRecAndExecute(
			/* The Sensor I2C bus address */
			I2C_PRES_ADDR_8BIT,
			/* Transmit one byte, the presser Sensor register address */
			&PresserRegisterAddress, txSz,
			/* Receive back three bytes, the contents of the presser register */
			presser, 3);

	/* Test for valid operation */
	if (i2cmXferRec.status == I2CM_STATUS_OK) {
		// calculate presser different between H,L
		int16_t pres = ((int16_t)presser[0] << 8) | presser[1];

		return pres/240*0.95f; // return the presser
	}
	return -404; // if status of is not normal, then report the error msg

	//return -400; // if the sensor on standby mode, then report the error msg


}
#endif

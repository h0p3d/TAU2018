/***************************************************************************//**
 * @file main.c
 * Hope Dargan -- hoped@mit.edu
 * July 2018
 * Most recent version of RF sense code.
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "retargetswo.h"
#include "rail.h"
#include "rail_types.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_rtc.h"
#include "em_rtcc.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "rail_config.h"
#include "hal_common.h"
#include "rtcdrv_config.h"
#include "rtcdriver.h"
#include "bsp.h"
#include "gpiointerrupt.h"
#include "application_properties.h"

/* Defines */
#define TESTING_MODE 			1      //toggle testing mode
#define BUTTONS_ENABLED			0	   // toggle button enabling, useful for use with testing mode
#define MAX_BUFFER_SIZE  		128    // Memory manager configuration
#define RAIL_TX_FIFO_SIZE  		128    // Length of largest packet to send
#define ID_PACKET_SIZE			16	   // Length of all ID packets
#define ARRAY_SIZE				25	   // Length of packetArray that keeps track of all current interactions. Adjust to sample size.
#define APP_MAX_PACKET_LENGTH   (MAX_BUFFER_SIZE - 12) /* sizeof(RAIL_RxPacketInfo_t) == 12) */
#define HEADER_LENGTH 			8      //number of bytes
#define SLEEP_TIME_MS 			1000   // 1 second in ms
#define WAKE_TIME_MS 			10     // ms
#define SNOOZE_ENABLED			0      // toggle ability to enter snooze mode
#define SNOOZE_TIME_MS 			300000 // 5 minutes
#define RFSENSE_TIME			2500   //microseconds Note:calculate rfsense time- txData size in kilobits/ bitrate (kbps), then convert to microseconds
#define INTERACTION_TIME		100000000  // 5 minutes in microseconds

/* Method declarations */
void RAILCb_Generic(RAIL_Handle_t railHandle, RAIL_Events_t events);
void radioInit();
void timer_set();
void sleep(bool snooze);
void main_initialization();
void log_packets();

#if SNOOZE_ENABLED
int rfWakeCount = 0; // used for snooze mode- count number of times woken by rfSense without receiving id packets
#endif

//Declare variables
RTCDRV_TimerID_t RTCid;
RAIL_Handle_t railHandle = NULL;
const uint8_t channel = 0; // default channel always used
volatile bool packetTx  = false; //send a packet
volatile bool packetRx = true;  //go to receive mode
volatile bool wakeToggle = false; //go to wake mode
volatile bool timerFinished = false; // sleep mode timer expired?
volatile bool rfSensed = false; // wake up due to rfsense?
uint16_t txCount = 0; //used to track number of packets sent each wake mode
uint8_t rxCount = 0; // number of id packets received in a given wake cycle

volatile bool sleepToggle = !BUTTONS_ENABLED; //go to sleep mode
//without button_enabled this will trigger sleep wake cycle, enabling buttons let's you manually send it to sleep mode

static uint8_t txFifo[RAIL_TX_FIFO_SIZE];
//length 16 bytes. First 8 bytes = header. Keep first byte 0xFF for frame type indication, Second 8 bytes = packet id
static const uint8_t txData[] =
{0xFF, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,};
// length 128 bytes. Do not change first byte from 0x00 - it indicates frame type.
static const uint8_t txWakeup[] =
{0x00, 0x96, 0xab, 0x5c, 0x2d, 0x1d, 0x2b, 0x7b, 0xd0, 0xfd, 0x67, 0x5f, 0x66, 0x7f, 0xe9, 0xb4, 0xd7, 0x6e, 0xfe, 0x5b,
 0x21, 0xcc, 0x04, 0xc7, 0x16, 0x82, 0x4c, 0xcd, 0xa2, 0x5c, 0x1f, 0xca, 0xb0, 0xc1, 0xe1, 0xdb, 0x11, 0x94, 0x61, 0xf6,
 0x2f, 0x27, 0x22, 0x54, 0x9b, 0xa2, 0x18, 0xe7, 0xfd, 0xa0, 0x29, 0x71, 0xd8, 0xe8, 0x31, 0x20, 0x2a, 0x92, 0xb7, 0x16,
 0xe2, 0xef, 0x58, 0x83, 0x00, 0x96, 0xab, 0x5c, 0x2d, 0x1d, 0x2b, 0x7b, 0xd0, 0xfd, 0x67, 0x5f, 0x66, 0x7f, 0xe9, 0xb4,
 0xd7, 0x6e, 0xfe, 0x5b, 0x21, 0xcc, 0x04, 0xc7, 0x16, 0x82, 0x4c, 0xcd, 0xa2, 0x5c, 0x1f, 0xca, 0xb0, 0xc1, 0xe1, 0xdb,
 0x11, 0x94, 0x61, 0xf6, 0x2f, 0x27, 0x22, 0x54, 0x9b, 0xa2, 0x18, 0xe7, 0xfd, 0xa0, 0x29, 0x71, 0xd8, 0xe8, 0x31, 0x20,
 0x2a, 0x92, 0xb7, 0x16, 0xe2, 0xef, 0x58, 0x83,};

/* Structure for logging id packets */
typedef struct RxIdPacket{
	RAIL_Time_t initialTimeRx;
	RAIL_Time_t newestTimeRx;
	int16_t     totalRssi;
	uint16_t	idRxCount;
	int8_t		maxRssi;
	int8_t		slotAvailable;
	uint8_t     interactionsCurrentCycle;
	uint8_t 	id[ID_PACKET_SIZE];
} RxIdPacket_t;
RxIdPacket_t packetArray[ARRAY_SIZE]; // used to track packets received in a current wake cycle.

typedef struct RxPacketLog{
	uint16_t    id;
	RAIL_Time_t initialTimeRx;   //TODO Add date and time, not just rail time
	uint16_t    interactionDuration; // in seconds
	int8_t		meanRssi;
	int8_t		maxRssi;
} RxPacketLog_t;
RxPacketLog_t packetLog[100]; // log packet ids and time they were received, arbitrarily large
uint32_t logCount = 0;
uint8_t receiveBuffer[MAX_BUFFER_SIZE];

static RAIL_Config_t railCfg = {
		.eventsCallback = &RAILCb_Generic,
};

#if BUTTONS_ENABLED
void gpioCallback(uint8_t pin);
//Buttons
typedef struct ButtonArray{
	GPIO_Port_TypeDef   port;
	unsigned int        pin;
} ButtonArray_t;
static const ButtonArray_t buttonArray[BSP_NO_OF_BUTTONS] = BSP_GPIO_BUTTONARRAY_INIT;

#endif

#if TESTING_MODE
void printLog(int startIndex, int endIndex);
void printData();
uint32_t totalWake = 0;
uint32_t rssiCount = 0;
int rssiTotal = 0;
#endif

int main(void)
{
	main_initialization();
	while (1) //infinite loop
	{
		/*
		 * If a wakeup was triggered due to RF energy, on wakeup send a wakeup packet followed by an id packet
		 * (contained in txData array.) The board that caused the wakeup will then be able to respond to the id
		 *  packet. If woken up by the sleep timer, send a wake packet (contained in txWakeup array)
		 */
		if (packetTx) { // transmit mode
			packetTx = false;
			static const RAIL_TxPowerLevel_t id_power = 0;
			RAIL_Idle(railHandle, RAIL_IDLE, true);
			if (txCount == 0){ // send wakeup packet on wakeup
				RAIL_SetTxPower(railHandle, RAIL_TX_POWER_LEVEL_SUBGIG_MAX);
				RAIL_WriteTxFifo(railHandle, txWakeup, sizeof(txWakeup)/sizeof(txWakeup[0]), true);
				packetTx = rfSensed; //if woken by rfSense, also send id Packet
			}
			else{ // otherwise send normal id packet
				RAIL_SetTxPower(railHandle, id_power);
				RAIL_WriteTxFifo(railHandle, txData, sizeof(txData), true);
			}
			txCount ++;
			RAIL_StartTx(railHandle, channel, RAIL_TX_OPTIONS_DEFAULT, NULL);
		}
		/*
		 * Prepares board to wait and receive a packet.
		 * In order to see the meat of what receive mode does, go to the RAIL_CbGeneric method.
		 */
		if (packetRx) { //receive mode
			packetRx = false;
			RAIL_Idle(railHandle, RAIL_IDLE, true);
			RAIL_StartRx(railHandle, channel, NULL);
		}
		/*
		 * Sleep mode is the low energy power mode that conserves energy. Sleep mode lasts on average for a period of
		 *SLEEP_TIME_MS by setting a RTC timer.  Sleep mode ends in one of two ways. Either the timer expires and
		 *sends the board into wake mode or the board detects RF energy for a period of 2.5 milliseconds  and sends
		 *the board into wake mode. Before going to sleep, the board stores all of the id packets received
		 *during the current wake cycle using the logPackets method.
		 */
		if (sleepToggle) { //sleep mode
			sleepToggle = false;
			bool snooze = false;
#if SNOOZE_ENABLED
			rfWakeCount = rxCount > 0 ? 0 : rfWakeCount;
			static const uint wakeLimit = 60;
			snooze = rfWakeCount >= wakeLimit; //snooze if repeatedly woken up by RF energy without receiving id packets
#endif
			log_packets();
			sleep(snooze);
		}
		/*
		 * When the board wakes, it resets various counters for receive and transmit mode, enables packetTx so the
		 *  wakeup packet can be sent. When the wake timer expires, it triggers sleep mode.
		 */
		if (wakeToggle) { //wake mode
			wakeToggle = false;
			packetTx = true; // go to transmit mode
			rxCount = 0; // every time you wake reset packetArray
			txCount = 0; //reset number of packets sent each wake cycle
			timer_set();
		}
	}
}

/**************************************************************************//**
 * @brief RTC Callback. Called when sleep timer expires. Triggers wake up.
 *****************************************************************************/
void rtcCallback( RTCDRV_TimerID_t id, void *user )
{
	timerFinished = true;
}

/******************************************************************************
 * Sleep Mode Implementation
 *****************************************************************************/
void sleep(bool snooze)
{
	packetRx = false;
	packetTx = false;
	timerFinished = false;
	rfSensed = false;
	static const uint32_t rfUs = RFSENSE_TIME; // how long in microseconds rf energy must be sensed
	// Shut down radio packet reception for EM2+ sleep
	RAIL_Idle(railHandle, RAIL_IDLE_ABORT, false);
	// Disable interrupts heading into RAIL_StartRfSense() so we don't miss
	// the event occurring before we try to sleep.
	CORE_DECLARE_IRQ_STATE;
	CORE_ENTER_CRITICAL();
	uint32_t sleep_time = snooze ? SNOOZE_TIME_MS : SLEEP_TIME_MS;
	if (snooze){ //disable RF sense detection and sleep for SNOOZE_TIME_MS
		RTCDRV_StartTimer( RTCid, rtcdrvTimerTypePeriodic, sleep_time, rtcCallback, NULL );
	}
	else{
		// if did not communicate on wakeup, randomize sleep every wake up to prevent synchronization
		if (rxCount == 0){
			uint8_t num_sleep_mods = 5; // use only odd numbers to get an even distribution
			int8_t rand_num = (rand()%num_sleep_mods)-(num_sleep_mods-1)/2; //get a random number centered at 0
			sleep_time += rand_num*WAKE_TIME_MS;
		}
		RAIL_StartRfSense(railHandle, RAIL_RFSENSE_SUBGHZ, rfUs, NULL);
		RTCDRV_StartTimer( RTCid, rtcdrvTimerTypePeriodic, sleep_time, rtcCallback, NULL );
	}
	do { // Loop modally here until either RfSense or RTC expires
		EMU_EnterEM2(true);
		CORE_EXIT_CRITICAL(); // Briefly enable IRQs to let them run
		CORE_ENTER_CRITICAL(); // but shut back off in case we loop
		rfSensed = RAIL_IsRfSensed(railHandle); // check for RF energy trigger
	} while (!rfSensed && !timerFinished);
#if SNOOZE_ENABLED
	rfWakeCount = rfSensed ? rfWakeCount + 1 : 0; // count number of times in a row woken up by rfSense
#endif
	CORE_EXIT_CRITICAL(); // Back on permanently
	wakeToggle = true;  // once you exit loop, wake system up
}

/******************************************************************************
 * Timer Implementation for Wake Mode
 *****************************************************************************/
void timer_callback(RAIL_Handle_t railHandle){
	// if timer expires, go to sleep
	sleepToggle = true;
}
void timer_set(){
	RAIL_Time_t wake_time = WAKE_TIME_MS * 1000; //convert to micro sec
	RAIL_TimerCallback_t timerCb = timer_callback;
	RAIL_CancelTimer(railHandle); //cancel previous timer if it is running
	RAIL_SetTimer(railHandle, wake_time, RAIL_TIME_DELAY, timerCb);
}

/**************************************************************************//**
 * Before going to sleep at end of wake period, store the packets in
 * packetArray to packetLog. Note that packetLog has a limited size when it
 * is initialized. If size is exceeded, it will wrap around in a circular
 * buffer and write over previously received data.
 *****************************************************************************/
void log_packets()
{
	uint initialLog = logCount;
	RAIL_Time_t current_time = RAIL_GetTime();
	for (int i = 0; i < ARRAY_SIZE; i++){
		if (!packetArray[i].slotAvailable) { // if slot is currently in use for an interaction,
			packetArray[i].interactionsCurrentCycle = 0; // reset number of interactions for next wake cycle
			if (current_time - packetArray[i].initialTimeRx >= INTERACTION_TIME) { //check that interaction is finished
				packetLog[logCount].initialTimeRx = packetArray[i].initialTimeRx;
				packetLog[logCount].interactionDuration = (packetArray[i].newestTimeRx - packetArray[i].initialTimeRx)/1000000; // convert to s
				packetLog[logCount].maxRssi = packetArray[i].maxRssi;
				packetLog[logCount].meanRssi = packetArray[i].totalRssi / packetArray[i].idRxCount;
				packetLog[logCount].id = 0;
				for (int j=0; j < ID_PACKET_SIZE; j++)
					packetLog[logCount].id += packetArray[i].id[j];
				logCount += 1;
				packetArray[i].slotAvailable = 1; // make slot available for use
			}
		}
		if (logCount == sizeof(packetLog)/sizeof(packetLog[0]))
		{
		#if TESTING_MODE
			printLog(initialLog, logCount);
		#endif
			logCount = 0; // wrap around and start over
			initialLog = 0;
		}
	}
#if TESTING_MODE
	if (logCount > initialLog) printLog(initialLog, logCount);
	else{
		printf("  Time: %.2f", current_time/1000000.0);
		printData();
	}
#endif
}
/**************************************************************************//**
 * Compares two arrays of equal length up to 255 bytes,
 * returns true if each index matches
 *****************************************************************************/
bool compareArray(uint8_t *array1, uint8_t *array2, uint8_t length)
{
	for (int i = 0; i < length; i++) {
		if (array1[i] != array2[i]) {
			return false;
		}
	}
	return true;
}
/**************************************************************************//**
 * Compares id in packet received to all the ids received in this
 * wake session, returns index if match is found
 *****************************************************************************/
int arrayIdIndex(uint8_t *id, uint8_t length){
	for (int k=0; k < ARRAY_SIZE; k++){
		if (!packetArray[k].slotAvailable){  // check if slot is in use
			if (compareArray(id, packetArray[k].id, length)){ // check if ids match
						return k;
				}
		}
	}
	return -1; // no match found
}
/******************************************************************************
 * RAIL Callback Implementation
 *****************************************************************************/
void RAILCb_Generic(RAIL_Handle_t railHandle, RAIL_Events_t events)
{
	if (events & RAIL_EVENT_TX_PACKET_SENT) {
		packetRx = !packetTx; //if another pack is not going to be sent, enable receive mode
	}
	if (events & (RAIL_EVENT_TX_UNDERFLOW
			| RAIL_EVENT_TX_BLOCKED
			| RAIL_EVENT_TX_ABORTED)) {
		packetRx = true; // if a transmit failed go to receive mode
	}
	/*
	 * When an RAIL_EVENT_RX_PACKET_RECEIVED event is triggered the following occurs. If
	 * the packet is successfully received, it checks that the packet is an ID packet by
	 *  first comparing the length of the received packet (all ID packets have the same
	 *  length) and then by comparing the header (the 8 bytes/values of txData array is
	 *  the header. Must be the same for all id packets.) If both match, the program
	 *  checks to make sure the id packet has not already been added to the packetArray
	 *  (stores id packets received for current wake session) before adding it.
	 *  This prevents unnecessary log entries.
	 */
	if (events & RAIL_EVENT_RX_PACKET_RECEIVED) {
		RAIL_RxPacketInfo_t packetInfo;
		RAIL_GetRxPacketInfo(railHandle, RAIL_RX_PACKET_HANDLE_NEWEST, &packetInfo);
		if ((packetInfo.packetStatus != RAIL_RX_PACKET_READY_SUCCESS)
				&& (packetInfo.packetStatus != RAIL_RX_PACKET_READY_CRC_ERROR)) {
			// RAIL_EVENT_RX_PACKET_RECEIVED must be handled last in order to return
			// early on aborted packets here.
			return;
		}
		// Read id packet into our packet structure
		uint16_t length = packetInfo.packetBytes;
		if (length == ID_PACKET_SIZE){
		    memcpy(receiveBuffer,
		           packetInfo.firstPortionData,
		           packetInfo.firstPortionBytes);
		    memcpy(receiveBuffer + packetInfo.firstPortionBytes,
		           packetInfo.lastPortionData,
		           length - packetInfo.firstPortionBytes);
		}
		//check if packet has right length and matching header
		if (length == ID_PACKET_SIZE && compareArray(receiveBuffer, (uint8_t *)txData, HEADER_LENGTH)) {
			RAIL_RxPacketDetails_t packetDetails;
			RAIL_GetRxPacketDetailsAlt(railHandle,RAIL_RX_PACKET_HANDLE_NEWEST, &packetDetails);

			//get id packet
			uint8_t id[length];
			for (int k=0; k<length; k++){
				id[k] = receiveBuffer[k];
			}
			//check if id is in packetArray
			int i = arrayIdIndex(id, length);
			if (i >= 0){ //id packet has an active interaction
				if (packetDetails.rssi > packetArray[i].maxRssi)
					packetArray[i].maxRssi = packetDetails.rssi;
				packetArray[i].newestTimeRx = packetDetails.timeReceived.packetTime;
				packetArray[i].interactionsCurrentCycle += 1;
				packetArray[i].totalRssi += packetDetails.rssi;
				packetArray[i].idRxCount += 1;
				packetTx = packetTx ? packetTx : packetArray[i].interactionsCurrentCycle < 2; // respond up 2 times to same id packet in a given wake cycle to increase communication chance
			}
			else { //start a new interaction log
				//find first unused slot
				for (int k = 0; k < ARRAY_SIZE; k++){
					if (packetArray[k].slotAvailable){
						i = k;
						break;
					}
				}
				if (i != -1){
					for (int j=0; j < ID_PACKET_SIZE; j++)
								packetArray[i].id[j] = id[j];
					packetArray[i].initialTimeRx = packetDetails.timeReceived.packetTime;
					packetArray[i].idRxCount = 1;
					packetArray[i].interactionsCurrentCycle = 1;
					packetArray[i].maxRssi = packetDetails.rssi;
					packetArray[i].totalRssi = packetDetails.rssi;
					packetArray[i].newestTimeRx = packetDetails.timeReceived.packetTime;
					packetArray[i].slotAvailable = 0;
					packetTx = true;
				}
				else{ // Houston, we have a problem.
					printf("Packet array isn't big enough to hold all current interactions. Adjust ARRAY_SIZE.");
				}
			}
			packetRx = !packetTx; // if we are done transmitting go to receive mode
			rxCount = rxCount+1;
#if TESTING_MODE
			rssiCount += 1;
			rssiTotal += packetDetails.rssi;
#endif
		}
		else { //if packet received is not an id packet, wait for another packet
			packetRx = true;
		}
	}
}
/******************************************************************************
 * Radio Initialization
 ******************************************************************************/
void main_initialization()
{
	// Necessary initialization steps do not change
	CHIP_Init();   // Initialize the chip
	halInit(); // Initialize the system clocks and other HAL components
	CMU_ClockEnable(cmuClock_GPIO, true);
	BSP_Init(BSP_INIT_BCC); // Initialize the BSP
#if BUTTONS_ENABLED
	// Enable the buttons on the board
	for (int i = 0; i < BSP_NO_OF_BUTTONS; i++) {
		GPIO_PinModeSet(buttonArray[i].port, buttonArray[i].pin, gpioModeInputPull, 1);
	}
	// Button Interrupt Config
	GPIOINT_Init();
	GPIOINT_CallbackRegister(buttonArray[0].pin, gpioCallback);
	GPIOINT_CallbackRegister(buttonArray[1].pin, gpioCallback);
	GPIO_IntConfig(buttonArray[0].port, buttonArray[0].pin, false, true, true);
	GPIO_IntConfig(buttonArray[1].port, buttonArray[1].pin, false, true, true);
#endif
	// Initialize Radio
	radioInit();
	// Configure RAIL callbacks
	RAIL_ConfigEvents(railHandle,
			RAIL_EVENTS_ALL,
			(RAIL_EVENT_RX_PACKET_RECEIVED
					| RAIL_EVENT_TX_PACKET_SENT
					| RAIL_EVENT_TX_BLOCKED
					| RAIL_EVENT_TX_ABORTED
					| RAIL_EVENT_TX_UNDERFLOW));
	// Initialize the PA now that the HFXO is up and the timing is correct
	RAIL_TxPowerConfig_t txPowerConfig = {
#if HAL_PA_2P4_LOWPOWER
			.mode = RAIL_TX_POWER_MODE_2P4_LP,
#else
			.mode = RAIL_TX_POWER_MODE_2P4_HP,
#endif
			.voltage = BSP_PA_VOLTAGE,
			.rampTime = HAL_PA_RAMP,
	};
	if (channelConfigs[0]->configs[0].baseFrequency < 1000000000UL) {
		// Use the Sub-GHz PA if required
		txPowerConfig.mode = RAIL_TX_POWER_MODE_SUBGIG;
	}
	if (RAIL_ConfigTxPower(railHandle, &txPowerConfig) != RAIL_STATUS_NO_ERROR) {
		// Error: The PA could not be initialized due to an improper configuration.
		// Please ensure your configuration is valid for the selected part.
		while (1) ;
	}
	RAIL_SetTxPower(railHandle, HAL_PA_POWER);
	RAIL_SetTxFifo(railHandle, txFifo, 0, sizeof(txFifo));

	//Real Time Counter for low energy mode Initialization
#if defined( _EMU_DCDCCTRL_MASK )
	/* Init DCDC regulator */
	EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_DEFAULT;
	EMU_DCDCInit(&dcdcInit);
#endif
	/* Initialize RTC driver. */
	RTCDRV_Init();
	RTCDRV_AllocateTimer( &RTCid );
	/* Initialize PacketArray slots to be available */
	for (int x = 0; x < ARRAY_SIZE; x++)
		packetArray[x].slotAvailable = 1;

}

/******************************************************************************
 * Configuration Utility Functions
 *****************************************************************************/
void radioInit()
{
	railHandle = RAIL_Init(&railCfg, NULL);
	if (railHandle == NULL) {
		while (1) ;
	}
	RAIL_ConfigCal(railHandle, RAIL_CAL_ALL);

	// Set us to a valid channel for this config and force an update in the main
	// loop to restart whatever action was going on
	RAIL_ConfigChannels(railHandle, channelConfigs[0], NULL);
}

#if BUTTONS_ENABLED
/******************************************************************************
 * Button Press Callback Implementation -- for testing
 *****************************************************************************/
void gpioCallback(uint8_t pin)
{
	// press button 0- transmit packet
	if (pin == buttonArray[0].pin) {
		packetTx = true;
	}
	// press button 1- trigger sleep mode
	else if (pin == buttonArray[1].pin) {
		sleepToggle = true;
	}
}
#endif

# if TESTING_MODE
void printLog(int startIndex, int endIndex)
{
	for(int i=startIndex; i<endIndex; i++){
		printf("ID: %d",packetLog[i].id);
		float time_stamp = packetLog[i].initialTimeRx / 1000000.0; //convert from microseconds to seconds
		printf("   Start Time: %f s", time_stamp);
		printf("   Interaction Duration: %d s", packetLog[i].interactionDuration);
		printf("   Max RSSI: %d", packetLog[i].maxRssi);
		printf("   Mean RSSI: %d", packetLog[i].meanRssi);
		printf("\n\n");
	}

}

void printData()
{
	totalWake += 1;
	/* print success rate of packets received */
	float communicationRate = (100.0 * rssiCount) / totalWake;
	float rssiAvg = (rssiTotal * 1.0) / rssiCount;
	printf("  Communication %%: %.2f", communicationRate);
	printf("  RSSI Average %%: %.2f", rssiAvg);
	printf("\n\n");
}
#endif

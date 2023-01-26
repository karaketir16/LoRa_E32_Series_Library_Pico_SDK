/*
 * Euint8_t LoRa E32 Series
 * https://www.mischianti.org/category/my-libraries/lora-e32-devices/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Renzo Mischianti www.mischianti.org All right reserved.
 *
 * You may copy, alter and reuse this code in any way you like, but please leave
 * reference to www.mischianti.org in your comments if you redistribute this code.
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "LoRa_E32.h"

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/time.h"

#include <string.h>


#define millis()\
to_ms_since_boot (get_absolute_time())
 

#define INPUT	GPIO_IN
#define OUTPUT	GPIO_OUT

#define pinMode(pin, mode)\
gpio_init(pin);\
gpio_set_dir(pin, mode)

#define digitalWrite(pin, val)\
gpio_put(pin, val)

#define digitalRead(pin)\
gpio_get(pin)

#define m64(val) (((uint64_t)(val)))
#define ONE (m64(1))

#define bitRead(val, bitNo)\
(m64(val) & (ONE << bitNo))

#define HIGH 1
#define LOW 0



LoRa_E32::LoRa_E32(HardwareSerial* serial, uint8_t auxPin, uint8_t m0Pin, uint8_t m1Pin, UART_BPS_RATE bpsRate){ //, uint32_t serialConfig
    this->auxPin = auxPin;

    this->m0Pin = m0Pin;
    this->m1Pin = m1Pin;

    this->hs = serial;

    this->bpsRate = bpsRate;
}


bool LoRa_E32::begin(){
	DEBUG_PRINT("RX MIC ---> ");
	DEBUG_PRINTLN(this->txE32pin);
	DEBUG_PRINT("TX MIC ---> ");
	DEBUG_PRINTLN(this->rxE32pin);
	DEBUG_PRINT("AUX ---> ");
	DEBUG_PRINTLN(this->auxPin);
	DEBUG_PRINT("M0 ---> ");
	DEBUG_PRINTLN(this->m0Pin);
	DEBUG_PRINT("M1 ---> ");
	DEBUG_PRINTLN(this->m1Pin);

	if (this->auxPin != -1) {
		pinMode(this->auxPin, INPUT);
		DEBUG_PRINTLN("Init AUX pin!");
	}
	if (this->m0Pin != -1) {
		pinMode(this->m0Pin, OUTPUT);
		DEBUG_PRINTLN("Init M0 pin!");
		digitalWrite(this->m0Pin, HIGH);

	}
	if (this->m1Pin != -1) {
		pinMode(this->m1Pin, OUTPUT);
		DEBUG_PRINTLN("Init M1 pin!");
		digitalWrite(this->m1Pin, HIGH);

	}

    DEBUG_PRINTLN("Begin ex");

	uart_set_baudrate(this->hs, this->bpsRate);

    Status status = setMode(MODE_0_NORMAL);
    return status==E32_SUCCESS;
}

/*

Utility method to wait until module is doen tranmitting
a timeout is provided to avoid an infinite loop

*/

Status LoRa_E32::waitCompleteResponse(unsigned long timeout, unsigned int waitNoAux) {

	Status result = E32_SUCCESS;

	unsigned long t = millis();

	// make darn sure millis() is not about to reach max data type limit and start over
	if (((unsigned long) (t + timeout)) == 0){
		t = 0;
	}

	// if AUX pin was supplied and look for HIGH state
	// note you can omit using AUX if no pins are available, but you will have to use delay() to let module finish
	if (this->auxPin != -1) {
		while (digitalRead(this->auxPin) == LOW) {
			if ((millis() - t) > timeout){
				result = ERR_E32_TIMEOUT;
				DEBUG_PRINTLN("Timeout error!");
				return result;
			}
		}
		DEBUG_PRINTLN("AUX HIGH!");
	}
	else {
		// if you can't use aux pin, use 4K7 pullup with Arduino
		// you may need to adjust this value if transmissions fail
		this->managedDelay(waitNoAux);
		DEBUG_PRINTLN(F("Wait no AUX pin!"));
	}


	// per data sheet control after aux goes high is 2ms so delay for at least that long)
	this->managedDelay(20);
	DEBUG_PRINTLN(F("Complete!"));
	return result;
}

/*

delay() in a library is not a good idea as it can stop interrupts
just poll internal time until timeout is reached

*/


void LoRa_E32::managedDelay(unsigned long timeout) {

	unsigned long t = millis();

	// make darn sure millis() is not about to reach max data type limit and start over
	if (((unsigned long) (t + timeout)) == 0){
		t = 0;
	}

	while ((millis() - t) < timeout) 	{ 	}

}

/*

Method to send a chunk of data provided data is in a struct--my personal favorite as you
need not parse or worry about sprintf() inability to handle floats

TTP: put your structure definition into a .h file and include in both the sender and reciever
sketches

NOTE: of your sender and receiver MCU's are different (Teensy and Arduino) caution on the data
types each handle ints floats differently

*/

Status LoRa_E32::sendStruct(void *structureManaged, uint16_t size_) {
		if (size_ > MAX_SIZE_TX_PACKET + 2){
			return ERR_E32_PACKET_TOO_BIG;
		}

		Status result = E32_SUCCESS;

		uint8_t* ch = (uint8_t *) structureManaged;

		for(int i = 0 ; i < size_ ; i++){
			uart_putc_raw(this->hs, ch[i]);
		}

		result = this->waitCompleteResponse(1000);
		if (result != E32_SUCCESS) return result;

		DEBUG_PRINTLN(F("ok!"))

		return result;
}


/*

Method to get a chunk of data provided data is in a struct--my personal favorite as you
need not parse or worry about sprintf() inability to handle floats

TTP: put your structure definition into a .h file and include in both the sender and reciever
sketches

NOTE: of your sender and receiver MCU's are different (Teensy and Arduino) caution on the data
types each handle ints floats differently

*/


Status LoRa_E32::receiveStruct(void *structureManaged, uint16_t size_) {
	Status result = E32_SUCCESS;

	uint8_t* ch = (uint8_t *) structureManaged;

	for(int i = 0; i < size_; i++){
		ch[i] = uart_getc(this->hs);
	}

	result = this->waitCompleteResponse(1000);
	if (result != E32_SUCCESS) return result;

	return result;
}

/*

method to set the mode (program, normal, etc.)

*/

Status LoRa_E32::setMode(MODE_TYPE mode) {

	// data sheet claims module needs some extra time after mode setting (2ms)
	// most of my projects uses 10 ms, but 40ms is safer

	this->managedDelay(40);

	if (this->m0Pin == -1 && this->m1Pin == -1) {
		DEBUG_PRINTLN(F("The M0 and M1 pins is not set, this mean that you are connect directly the pins as you need!"))
	}else{
		switch (mode)
		{
		  case MODE_0_NORMAL:
			// Mode 0 | normal operation
			digitalWrite(this->m0Pin, LOW);
			digitalWrite(this->m1Pin, LOW);
			DEBUG_PRINTLN("MODE NORMAL!");
			break;
		  case MODE_1_WAKE_UP:
			digitalWrite(this->m0Pin, HIGH);
			digitalWrite(this->m1Pin, LOW);
			DEBUG_PRINTLN("MODE WAKE UP!");
			break;
		  case MODE_2_POWER_SAVING:
			digitalWrite(this->m0Pin, LOW);
			digitalWrite(this->m1Pin, HIGH);
			DEBUG_PRINTLN("MODE POWER SAVING!");
			break;
		  case MODE_3_SLEEP:
			// Mode 3 | Setting operation
			digitalWrite(this->m0Pin, HIGH);
			digitalWrite(this->m1Pin, HIGH);
			DEBUG_PRINTLN("MODE PROGRAM/SLEEP!");
			break;
		  default:
			return ERR_E32_INVALID_PARAM;
		}
	}
	// data sheet says 2ms later control is returned, let's give just a bit more time
	// these modules can take time to activate pins
	this->managedDelay(40);

	// wait until aux pin goes back low
	Status res = this->waitCompleteResponse(1000);

	if (res == E32_SUCCESS){
		this->mode = mode;
	}

	return res;
}

MODE_TYPE LoRa_E32::getMode(){
	return this->mode;
}

void LoRa_E32::writeProgramCommand(PROGRAM_COMMAND cmd){
	  uint8_t CMD[3] = {cmd, cmd, cmd};
	  // uint8_t size =
		for(int i = 0 ; i < 3 ; i++){
			uart_putc_raw(this->hs, CMD[i]);
		}

	  this->managedDelay(50);  //need ti check
}

ResponseStructContainer LoRa_E32::getConfiguration(){
	ResponseStructContainer rc;

	rc.status.code = checkUARTConfiguration(MODE_3_PROGRAM);
	if (rc.status.code!=E32_SUCCESS) return rc;

	MODE_TYPE prevMode = this->mode;

	rc.status.code = this->setMode(MODE_3_PROGRAM);
	if (rc.status.code!=E32_SUCCESS) return rc;

	this->writeProgramCommand(READ_CONFIGURATION);

	rc.data = malloc(sizeof(Configuration));
	rc.status.code = this->receiveStruct((uint8_t *)rc.data, sizeof(Configuration));

#ifdef LoRa_E32_DEBUG
	 this->printParameters((Configuration *)rc.data);
#endif

	if (rc.status.code!=E32_SUCCESS) {
		this->setMode(prevMode);
		return rc;
	}

	DEBUG_PRINTLN("----------------------------------------");
	DEBUG_PRINT(F("HEAD BIN INSIDE: "));  DEBUG_PRINT(((Configuration *)rc.data)->HEAD, BIN);DEBUG_PRINT(" ");DEBUG_PRINT(((Configuration *)rc.data)->HEAD, DEC);DEBUG_PRINT(" ");DEBUG_PRINTLN(((Configuration *)rc.data)->HEAD, HEX);
	DEBUG_PRINTLN("----------------------------------------");

	rc.status.code = this->setMode(prevMode);
	if (rc.status.code!=E32_SUCCESS) return rc;

//	this->printParameters(*configuration);

	if (0xC0 != ((Configuration *)rc.data)->HEAD && 0xC2 != ((Configuration *)rc.data)->HEAD){
		rc.status.code = ERR_E32_HEAD_NOT_RECOGNIZED;
	}

//	rc.data = configuration;
	return rc;
}

RESPONSE_STATUS LoRa_E32::checkUARTConfiguration(MODE_TYPE mode){
	if (mode==MODE_3_PROGRAM && this->bpsRate!=UART_BPS_RATE_9600){
		return ERR_E32_WRONG_UART_CONFIG;
	}
	return E32_SUCCESS;
}

ResponseStatus LoRa_E32::setConfiguration(Configuration configuration, PROGRAM_COMMAND saveType){
	ResponseStatus rc;

	rc.code = checkUARTConfiguration(MODE_3_PROGRAM);
	if (rc.code!=E32_SUCCESS) return rc;

	MODE_TYPE prevMode = this->mode;

	rc.code = this->setMode(MODE_3_PROGRAM);
	if (rc.code!=E32_SUCCESS) return rc;

	this->writeProgramCommand(READ_CONFIGURATION);

	configuration.HEAD = saveType;

	rc.code = this->sendStruct((uint8_t *)&configuration, sizeof(Configuration));
	if (rc.code!=E32_SUCCESS) {
		this->setMode(prevMode);
		return rc;
	}

	DEBUG_PRINTLN("----------------------------------------");
	DEBUG_PRINT(F("HEAD BIN INSIDE: "));  DEBUG_PRINT(configuration.HEAD, BIN);DEBUG_PRINT(" ");DEBUG_PRINT(configuration.HEAD, DEC);DEBUG_PRINT(" ");DEBUG_PRINTLN(configuration.HEAD, HEX);
	DEBUG_PRINTLN("----------------------------------------");

	rc.code = this->setMode(prevMode);
	if (rc.code!=E32_SUCCESS) return rc;

//	this->printParameters(*configuration);

	if (0xC0 != configuration.HEAD && 0xC2 != configuration.HEAD){
		rc.code = ERR_E32_HEAD_NOT_RECOGNIZED;
	}

	return rc;
}

ResponseStructContainer LoRa_E32::getModuleInformation(){
	ResponseStructContainer rc;

	rc.status.code = checkUARTConfiguration(MODE_3_PROGRAM);
	if (rc.status.code!=E32_SUCCESS) return rc;

	MODE_TYPE prevMode = this->mode;

	rc.status.code = this->setMode(MODE_3_PROGRAM);
	if (rc.status.code!=E32_SUCCESS) return rc;

	this->writeProgramCommand(READ_MODULE_VERSION);

	struct ModuleInformation *moduleInformation = (ModuleInformation *)malloc(sizeof(ModuleInformation));
	rc.status.code = this->receiveStruct((uint8_t *)moduleInformation, sizeof(ModuleInformation));
	if (rc.status.code!=E32_SUCCESS) {
		this->setMode(prevMode);
		return rc;
	}

	rc.status.code = this->setMode(prevMode);
	if (rc.status.code!=E32_SUCCESS) return rc;

//	this->printParameters(*configuration);

	if (0xC3 != moduleInformation->HEAD){
		rc.status.code = ERR_E32_HEAD_NOT_RECOGNIZED;
	}

	DEBUG_PRINTLN("----------------------------------------");
	DEBUG_PRINT(F("HEAD BIN INSIDE: "));  DEBUG_PRINT(moduleInformation->HEAD, BIN);DEBUG_PRINT(" ");DEBUG_PRINT(moduleInformation->HEAD, DEC);DEBUG_PRINT(" ");DEBUG_PRINTLN(moduleInformation->HEAD, HEX);

	DEBUG_PRINT(F("Freq.: "));  DEBUG_PRINTLN(moduleInformation->frequency, HEX);
	DEBUG_PRINT(F("Version  : "));  DEBUG_PRINTLN(moduleInformation->version, HEX);
	DEBUG_PRINT(F("Features : "));  DEBUG_PRINTLN(moduleInformation->features, HEX);
	DEBUG_PRINTLN("----------------------------------------");

	rc.data = moduleInformation; // malloc(sizeof (moduleInformation));

	return rc;
}


ResponseStatus LoRa_E32::resetModule(){
	ResponseStatus status;

	status.code = checkUARTConfiguration(MODE_3_PROGRAM);
	if (status.code!=E32_SUCCESS) return status;

	MODE_TYPE prevMode = this->mode;

	status.code = this->setMode(MODE_3_PROGRAM);
	if (status.code!=E32_SUCCESS) return status;

	this->writeProgramCommand(WRITE_RESET_MODULE);

	status.code = this->waitCompleteResponse(1000);
	if (status.code!=E32_SUCCESS)  {
		this->setMode(prevMode);
		return status;
	}


	status.code = this->setMode(prevMode);
	if (status.code!=E32_SUCCESS) return status;

	return status;
}


#pragma pack(push, 1)
ResponseStatus LoRa_E32::sendMessage(const void *message, const uint8_t size){
	ResponseStatus status;
	status.code = this->sendStruct((uint8_t *)message, size);
	if (status.code!=E32_SUCCESS) return status;

	return status;
}

typedef struct fixedStransmission
{
	uint8_t ADDH = 0;
	uint8_t ADDL = 0;
	uint8_t CHAN = 0;
	unsigned char message[];
}FixedStransmission;
#pragma pack(pop)


FixedStransmission *init_stack(int m){
	FixedStransmission *st = (FixedStransmission *)malloc(sizeof(FixedStransmission)+m*sizeof(int));
    return st;
}

ResponseStatus LoRa_E32::sendFixedMessage( uint8_t ADDH,uint8_t ADDL, uint8_t CHAN, const void *message, const uint8_t size){
	FixedStransmission *fixedStransmission = init_stack(size);

	fixedStransmission->ADDH = ADDH;
	fixedStransmission->ADDL = ADDL;
	fixedStransmission->CHAN = CHAN;

	memcpy(fixedStransmission->message,(unsigned char*)message,size);

	ResponseStatus status;
	status.code = this->sendStruct((uint8_t *)fixedStransmission, size+3);

	free(fixedStransmission);

	if (status.code!=E32_SUCCESS) return status;

	return status;
}
ResponseStatus LoRa_E32::sendBroadcastFixedMessage(uint8_t CHAN, const void *message, const uint8_t size){
	return this->sendFixedMessage(0xFF, 0xFF, CHAN, message, size);
}


#define KeeLoq_NLF		0x3A5C742E

unsigned long LoRa_E32::encrypt(unsigned long data)
{
  unsigned long x = data;
  unsigned long r;
  int keyBitNo, index;
  unsigned long keyBitVal,bitVal;

  for (r = 0; r < 528; r++)
  {
    keyBitNo = r & 63;
    if(keyBitNo < 32)
      keyBitVal = bitRead(this->halfKeyloqKey,keyBitNo); // key low
    else
      keyBitVal = bitRead(this->halfKeyloqKey, keyBitNo - 32);// key hight
    index = 1 * bitRead(x,1) + 2 * bitRead(x,9) + 4 * bitRead(x,20) + 8 * bitRead(x,26) + 16 * bitRead(x,31);
    bitVal = bitRead(x,0) ^ bitRead(x, 16) ^ bitRead(KeeLoq_NLF,index) ^ keyBitVal;
    x = (x>>1) ^ bitVal<<31;
  }
  return x;
}

unsigned long LoRa_E32::decrypt(unsigned long data)
{
  unsigned long x = data;
  unsigned long r;
  int keyBitNo, index;
  unsigned long keyBitVal,bitVal;

  for (r = 0; r < 528; r++)
  {
    keyBitNo = (15-r) & 63;
    if(keyBitNo < 32)
      keyBitVal = bitRead(this->halfKeyloqKey,keyBitNo); // key low
    else
      keyBitVal = bitRead(this->halfKeyloqKey, keyBitNo - 32); // key hight
    index = 1 * bitRead(x,0) + 2 * bitRead(x,8) + 4 * bitRead(x,19) + 8 * bitRead(x,25) + 16 * bitRead(x,30);
    bitVal = bitRead(x,31) ^ bitRead(x, 15) ^ bitRead(KeeLoq_NLF,index) ^ keyBitVal;
    x = (x<<1) ^ bitVal;
  }
  return x;
 }
#ifdef LoRa_E32_DEBUG
void LoRa_E32::printParameters(struct Configuration *configuration) {
	DEBUG_PRINTLN("----------------------------------------");

	DEBUG_PRINT(F("HEAD : "));  DEBUG_PRINT(configuration->HEAD, BIN);DEBUG_PRINT(" ");DEBUG_PRINT(configuration->HEAD, DEC);DEBUG_PRINT(" ");DEBUG_PRINTLN(configuration->HEAD, HEX);
	DEBUG_PRINTLN(F(" "));
	DEBUG_PRINT(F("AddH : "));  DEBUG_PRINTLN(configuration->ADDH, DEC);
	DEBUG_PRINT(F("AddL : "));  DEBUG_PRINTLN(configuration->ADDL, DEC);
	DEBUG_PRINT(F("Chan : "));  DEBUG_PRINT(configuration->CHAN, DEC); DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration->getChannelDescription());
	DEBUG_PRINTLN(F(" "));
	DEBUG_PRINT(F("SpeedParityBit     : "));  DEBUG_PRINT(configuration->SPED.uartParity, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration->SPED.getUARTParityDescription());
	DEBUG_PRINT(F("SpeedUARTDatte  : "));  DEBUG_PRINT(configuration->SPED.uartBaudRate, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration->SPED.getUARTBaudRate());
	DEBUG_PRINT(F("SpeedAirDataRate   : "));  DEBUG_PRINT(configuration->SPED.airDataRate, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration->SPED.getAirDataRate());

	DEBUG_PRINT(F("OptionTrans        : "));  DEBUG_PRINT(configuration->OPTION.fixedTransmission, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration->OPTION.getFixedTransmissionDescription());
	DEBUG_PRINT(F("OptionPullup       : "));  DEBUG_PRINT(configuration->OPTION.ioDriveMode, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration->OPTION.getIODroveModeDescription());
	DEBUG_PRINT(F("OptionWakeup       : "));  DEBUG_PRINT(configuration->OPTION.wirelessWakeupTime, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration->OPTION.getWirelessWakeUPTimeDescription());
	DEBUG_PRINT(F("OptionFEC          : "));  DEBUG_PRINT(configuration->OPTION.fec, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration->OPTION.getFECDescription());
	DEBUG_PRINT(F("OptionPower        : "));  DEBUG_PRINT(configuration->OPTION.transmissionPower, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration->OPTION.getTransmissionPowerDescription());

	DEBUG_PRINTLN("----------------------------------------");
}
#endif

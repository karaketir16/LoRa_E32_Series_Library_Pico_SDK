/*
 * Euint8_t LoRa E32 Series
 *
 * AUTHOR:  Renzo Mischianti
 * VERSION: 1.5.9
 *
 * https://www.mischianti.org/category/my-libraries/lora-e32-devices/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Renzo Mischianti www.mischianti.org All right reserved.
 *
 * You may copy, alter and reuse this code in any way you like, but please leave
 * reference to www.mischianti.org in your comments if you redistribute this code.
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
#ifndef LoRa_E32_h
#define LoRa_E32_h

#define MAX_SIZE_TX_PACKET 58

#include <stdint.h>
#include <string>
#include "includes/statesNaming.h"

#include <stdio.h>
#include "hardware/uart.h"

#include <iostream>

typedef uart_inst_t HardwareSerial;

// Uncomment to enable printing out nice debug messages.
// #define LoRa_E32_DEBUG

// Define where debug output will be printed.
#define DEBUG_PRINTER Serial

// Setup debug printing macros.
#ifdef LoRa_E32_DEBUG
	#define DEBUG_PRINT(...) { printf(__VA_ARGS__); }
	#define DEBUG_PRINTLN(...) { DEBUG_PRINT(__VA_ARGS__); printf("\r\n"); }
#else
	#define DEBUG_PRINT(...) {}
	#define DEBUG_PRINTLN(...) {}
#endif

enum MODE_TYPE
{
  MODE_0_NORMAL 		= 0,
  MODE_1_WAKE_UP 		= 1,
  MODE_2_POWER_SAVING 	= 2,
  MODE_3_SLEEP 			= 3,
  MODE_3_PROGRAM 		= 3,
  MODE_INIT 			= 0xFF
};

enum PROGRAM_COMMAND
{
  WRITE_CFG_PWR_DWN_SAVE  	= 0xC0,
  READ_CONFIGURATION 		= 0xC1,
  WRITE_CFG_PWR_DWN_LOSE 	= 0xC2,
  READ_MODULE_VERSION   	= 0xC3,
  WRITE_RESET_MODULE     	= 0xC4
};

#pragma pack(push, 1)
struct Speed {
  uint8_t airDataRate : 3; //bit 0-2
	std::string getAirDataRate() {
		return getAirDataRateDescriptionByParams(this->airDataRate);
	}

  uint8_t uartBaudRate: 3; //bit 3-5
	std::string getUARTBaudRate() {
		return getUARTBaudRateDescriptionByParams(this->uartBaudRate);
	}

  uint8_t uartParity:   2; //bit 6-7
	std::string getUARTParityDescription() {
		return getUARTParityDescriptionByParams(this->uartParity);
	}
};

struct Option {
	uint8_t transmissionPower	: 2; //bit 0-1
	std::string getTransmissionPowerDescription() {
		return getTransmissionPowerDescriptionByParams(this->transmissionPower);
	}

	uint8_t fec       		: 1; //bit 2
	std::string getFECDescription() {
		return getFECDescriptionByParams(this->fec);
	}

	uint8_t wirelessWakeupTime : 3; //bit 3-5
	std::string getWirelessWakeUPTimeDescription() {
		return getWirelessWakeUPTimeDescriptionByParams(this->wirelessWakeupTime);
	}

	uint8_t ioDriveMode  		: 1; //bit 6
	std::string getIODroveModeDescription() {
		return getIODriveModeDescriptionDescriptionByParams(this->ioDriveMode);
	}

	uint8_t fixedTransmission	: 1; //bit 7
	std::string getFixedTransmissionDescription() {
		return getFixedTransmissionDescriptionByParams(this->fixedTransmission);
	}

};

struct Configuration {
	uint8_t HEAD = 0;
	uint8_t ADDH = 0;
	uint8_t ADDL = 0;
	struct Speed SPED;
	uint8_t CHAN = 0;
	std::string getChannelDescription() {
		return std::to_string(this->CHAN + OPERATING_FREQUENCY) + "MHz";
	}
	struct Option OPTION;
};

struct ModuleInformation {
	uint8_t HEAD = 0;
	uint8_t frequency = 0;
	uint8_t version = 0;
	uint8_t features = 0;
};

struct ResponseStatus {
	Status code;
	std::string getResponseDescription() {
		return getResponseDescriptionByParams(this->code);
	}
};

struct ResponseStructContainer {
	void *data;
	ResponseStatus status;
	void close() {
		free(this->data);
	}
};
struct ResponseContainer {
	std::string data;
	ResponseStatus status;
};
//struct FixedStransmission {
//		uint8_t ADDL = 0;
//		uint8_t ADDH = 0;
//		uint8_t CHAN = 0;
//		void *message;
//};
#pragma pack(pop)

class LoRa_E32 {
	public:

		LoRa_E32(HardwareSerial* serial, uint8_t auxPin, uint8_t m0Pin, uint8_t m1Pin, UART_BPS_RATE bpsRate);

		bool begin();
        Status setMode(MODE_TYPE mode);
        MODE_TYPE getMode();

		ResponseStructContainer getConfiguration();
		ResponseStatus setConfiguration(Configuration configuration, PROGRAM_COMMAND saveType = WRITE_CFG_PWR_DWN_LOSE);

		ResponseStructContainer getModuleInformation();
		ResponseStatus resetModule();

		ResponseStatus sendMessage(const void *message, const uint8_t size);

	    ResponseContainer receiveMessageUntil(char delimiter = '\0');
		ResponseStructContainer receiveMessage(const uint8_t size);

		ResponseStatus sendMessage(const std::string message);
		ResponseContainer receiveMessage();

		ResponseStatus sendFixedMessage(uint8_t ADDH, uint8_t ADDL, uint8_t CHAN, const std::string message);

        ResponseStatus sendFixedMessage(uint8_t ADDH,uint8_t ADDL, uint8_t CHAN, const void *message, const uint8_t size);
        ResponseStatus sendBroadcastFixedMessage(uint8_t CHAN, const void *message, const uint8_t size);
        ResponseStatus sendBroadcastFixedMessage(uint8_t CHAN, const std::string message);

		ResponseContainer receiveInitialMessage(const uint8_t size);

        int available();
	private:
		HardwareSerial* hs;

		bool isSoftwareSerial = true;

        int8_t txE32pin = -1;
        int8_t rxE32pin = -1;
        int8_t auxPin = -1;

		int8_t m0Pin = -1;
		int8_t m1Pin = -1;

		unsigned long halfKeyloqKey = 0x06660708;
		unsigned long encrypt(unsigned long data);
		unsigned long decrypt(unsigned long data);

		UART_BPS_RATE bpsRate = UART_BPS_RATE_9600;

		MODE_TYPE mode = MODE_0_NORMAL;

		void managedDelay(unsigned long timeout);
		Status waitCompleteResponse(unsigned long timeout = 1000, unsigned int waitNoAux = 100);
		void flush();
		void cleanUARTBuffer();

		Status sendStruct(void *structureManaged, uint16_t size_);
		Status receiveStruct(void *structureManaged, uint16_t size_);
        void writeProgramCommand(PROGRAM_COMMAND cmd);

		RESPONSE_STATUS checkUARTConfiguration(MODE_TYPE mode);

#ifdef LoRa_E32_DEBUG
		void printParameters(struct Configuration *configuration);
#endif
};

#endif

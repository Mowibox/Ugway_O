/*
 * This file is part of libphidget22
 *
 * Copyright (c) 2015-2022 Phidgets Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __PHIDGETVINTPACKETS_H
#define __PHIDGETVINTPACKETS_H

typedef uint8_t VINTPacketType;

#define VINT_PACKET_ERRORBYTE_OUTOFRANGEHIGH	((uint8_t) 1)
#define VINT_PACKET_ERRORBYTE_OUTOFRANGELOW		((uint8_t)-1)
#define VINT_PACKET_ERRORBYTE_OUTOFRANGE		((uint8_t) 0)

enum GenericPacket {
	VINT_NET_SETSTATUS													= 0x01,

	VINT_PACKET_TYPE_RTD_WIRESETUP										= 0xC0,
	VINT_PACKET_TYPE_ISA_SETPOWERSUPPLY									= 0xC1,
	VINT_PACKET_TYPE_ISA_SETINPUTMODE									= 0xC2,
	VINT_PACKET_TYPE_AXES_SETAXISCHANGETRIGGER							= 0xC3,
	VINT_PACKET_TYPE_AXES_AXISVALUECHANGE								= 0xC4,
	VINT_PACKET_TYPE_AXES_AXESUPDATE									= 0xC5,
	VINT_PACKET_TYPE_AXES_SATURATION									= 0xC6,
	VINT_PACKET_TYPE_AXES_INVALID										= 0xC7,

	VINT_PACKET_TYPE_CALIBRATION_MODE									= 0xF0,
	VINT_PACKET_TYPE_CALIBRATION_WRITE									= 0xF1,
	VINT_PACKET_TYPE_CALIBRATION_DATA									= 0xF2,
	VINT_PACKET_TYPE_CALIBRATION_EXIT									= 0xF3,

	VINT_PACKET_TYPE_DATAINTERVAL_RESP									= 0xF5,

	VINT_PACKET_TYPE_FAILSAFE_TIME										= 0xF6,
	VINT_PACKET_TYPE_FAILSAFE_RESET										= 0xF7,

	VINT_PACKET_TYPE_PHIDGET_RESET										= 0xFA,
	VINT_PACKET_TYPE_PHIDGET_ENABLE										= 0xFB,
	VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL							= 0xFC,
	VINT_PACKET_TYPE_CONFIGURATION										= 0xFD,
};

enum BLDCMotorPacket {
	VINT_PACKET_TYPE_BLDCMOTOR_MOTORSTATUSUPDATE						= 0x20,
	VINT_PACKET_TYPE_BLDCMOTOR_SETVELOCITY								= 0x22,
	VINT_PACKET_TYPE_BLDCMOTOR_SETACCELERATION							= 0x23,
	VINT_PACKET_TYPE_BLDCMOTOR_SETBRAKINGSTRENGTH						= 0x24,
	VINT_PACKET_TYPE_BLDCMOTOR_SETSTALLVELOCITY							= 0x25,
	VINT_PACKET_TYPE_BLDCMOTOR_STALLDETECTED							= 0x26,
};

enum MotorPositionControllerPacket {
	VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKP						= 0x30,
	VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKD						= 0x31,
	VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKI						= 0x32,
	VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETDEADBAND				= 0x33,
	VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETTARGETPOSITION			= 0x34,
	VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_MOTORSTATUSUPDATE			= 0x35,
	VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETACCELERATION			= 0x36,
	VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETVELOCITYLIMIT			= 0x37,
	VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETENCODERIOMODE			= 0x38,
	VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETENGAGED					= 0x39,
	VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETSTALLVELOCITY			= 0x3A,
	VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_STALLDETECTED				= 0x3B
};

enum CurrentInputPacket {
	VINT_PACKET_TYPE_CURRENTINPUT_SETCURRENTCHANGETRIGGER				= 0x53,
	VINT_PACKET_TYPE_CURRENTINPUT_CURRENTCHANGE							= 0x54,
	VINT_PACKET_TYPE_CURRENTINPUT_SATURATION							= 0x55,
};

enum CurrentOutputPacket {
	VINT_PACKET_TYPE_CURRENTOUTPUT_SETENABLED							= 0x67,
	VINT_PACKET_TYPE_CURRENTOUTPUT_SETCURRENT							= 0x68,
	VINT_PACKET_TYPE_CURRENTOUTPUT_CURRENTERROR							= 0x69
};


enum DataAdadpterPacket {
	VINT_PACKET_TYPE_DATAADAPTER_PACKET_DATA							= 0x40,
	VINT_PACKET_TYPE_DATAADAPTER_PACKET_DATA_ERROR						= 0x41,
	VINT_PACKET_TYPE_DATAADAPTER_PACKET_DATA_END						= 0x42,
	VINT_PACKET_TYPE_DATAADAPTER_PACKET_TIMEOUT							= 0x43,
	VINT_PACKET_TYPE_DATAADAPTER_PACKET_DROPPED							= 0x44,
	VINT_PACKET_TYPE_DATAADAPTER_VOLTAGE_ERROR							= 0x45,
	VINT_PACKET_TYPE_DATAADAPTER_PACKET_ACK								= 0x46,
	VINT_PACKET_TYPE_DATAADAPTER_PACKET_SYNC							= 0x47,

	VINT_PACKET_TYPE_DATAADAPTER_TX_DATA								= 0x50,
	VINT_PACKET_TYPE_DATAADAPTER_BAUD_RATE								= 0x51,
	VINT_PACKET_TYPE_DATAADAPTER_HANDSHAKE_MODE							= 0x52,
	VINT_PACKET_TYPE_DATAADAPTER_DTR_DSR_MODE							= 0x53,
	VINT_PACKET_TYPE_DATAADAPTER_PARITY_MODE							= 0x54,
	VINT_PACKET_TYPE_DATAADAPTER_STOP_BITS								= 0x55,
	VINT_PACKET_TYPE_DATAADAPTER_DATA_BITS								= 0x56,
	VINT_PACKET_TYPE_DATAADAPTER_MSB_ORDER								= 0x57,
	VINT_PACKET_TYPE_DATAADAPTER_PROTOCOL								= 0x58,
	VINT_PACKET_TYPE_DATAADAPTER_SPIMODE								= 0x59,
	VINT_PACKET_TYPE_DATAADAPTER_ADDRESS								= 0x5A,
	VINT_PACKET_TYPE_DATAADAPTER_ENDIANNESS								= 0x5B,
	VINT_PACKET_TYPE_DATAADAPTER_TIMEOUT								= 0x5C,
	VINT_PACKET_TYPE_DATAADAPTER_IOVOLTAGE								= 0x5D,
	VINT_PACKET_TYPE_DATAADAPTER_TX_TIMEOUT								= 0x5E,
};

enum DCMotorPacket {
	VINT_PACKET_TYPE_MOTORCONTROLLER_SETTARGETPOSITION					= 0x58,
	VINT_PACKET_TYPE_MOTORCONTROLLER_SETVELOCITYLIMIT					= 0x59,
	VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION					= 0x5A,
	VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE							= 0x5D,
	VINT_PACKET_TYPE_MOTORCONTROLLER_FAN								= 0x5E,
	VINT_PACKET_TYPE_DCMOTOR_MOTORSTATUSUPDATE							= 0x60,
	VINT_PACKET_TYPE_DCMOTOR_DEADBAND									= 0x61,
	VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT							= 0x62,
	VINT_PACKET_TYPE_DCMOTOR_OVERTEMPERATURE							= 0x63,
	VINT_PACKET_TYPE_DCMOTOR_SETVELOCITY								= 0x64,
	VINT_PACKET_TYPE_DCMOTOR_SETBRAKINGSTRENGTH							= 0x65,
	VINT_PACKET_TYPE_DCMOTOR_SETCURRENTREGULATORGAIN					= 0x66,
};

enum DigitalInputPacket {
	VINT_PACKET_TYPE_DIGITALINPUT_INVALID								= 0x07,
	VINT_PACKET_TYPE_DIGITALINPUT_STATECHANGE							= 0x08,
	VINT_PACKET_TYPE_DIGITALINPUT_STATECHANGE2							= 0x09,
};

enum DigitalOutputPacket {
	VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE							= 0x0A,
	VINT_PACKET_TYPE_DIGITALOUTPUT_LEDCURRENTLIMIT						= 0x0B,
	VINT_PACKET_TYPE_DIGITALOUTPUT_LEDFORWARDVOLTAGE					= 0x0C,
	VINT_PACKET_TYPE_DIGITALOUTPUT_FAILSAFE								= 0x0D,
	VINT_PACKET_TYPE_DIGITALOUTPUT_SETFREQUENCY							= 0x0F,
	VINT_PACKET_TYPE_DIGITALOUTPUT_INVALID								= 0x10
};

enum DistanceSensorPacket {
	VINT_PACKET_TYPE_DISTANCESENSOR_SETDISTANCECHANGETRIGGER			= 0x95,
	VINT_PACKET_TYPE_DISTANCESENSOR_DISTANCECHANGE						= 0x96,
	VINT_PACKET_TYPE_DISTANCESENSOR_SATURATION							= 0x97,
	VINT_PACKET_TYPE_DISTANCESENSOR_SONARREFLECTIONSUPDATE				= 0x98,
	VINT_PACKET_TYPE_SONAR_QUIETMODE									= 0x99,
};

enum EncoderPacket {
	VINT_PACKET_TYPE_ENCODER_SETENCODERCHANGETRIGGER					= 0x30,
	VINT_PACKET_TYPE_ENCODER_ENCODERCHANGE								= 0x31,
	VINT_PACKET_TYPE_ENCODER_IOMODE										= 0x32,
	VINT_PACKET_TYPE_ENCODER_ENABLE										= 0x33,
};

enum FrequencyCounterPacket {
	VINT_PACKET_TYPE_FREQUENCYCOUNTER_COUNTCHANGE						= 0x84,
};

enum GyroscopePacket {
	VINT_PACKET_TYPE_GYROSCOPE_ZERO										= 0x33,
	VINT_PACKET_TYPE_GYROSCOPE_UNZERO									= 0x34,
};

enum CapacitiveTouchPacket {
	VINT_PACKET_TYPE_CAPACITIVETOUCH_TOUCHVALUECHANGE					= 0x35,
	VINT_PACKET_TYPE_CAPACITIVETOUCH_SENSITIVITY						= 0x36,
	VINT_PACKET_TYPE_CAPACITIVETOUCH_TOUCHVALUECHANGETRIGGER			= 0x37,
};

enum HumiditySensorPacket {
	VINT_PACKET_TYPE_HUMIDITYSENSOR_SETHUMIDITYCHANGETRIGGER			= 0x44,
	VINT_PACKET_TYPE_HUMIDITYSENSOR_HUMIDITYCHANGE						= 0x45,
	VINT_PACKET_TYPE_HUMIDITYSENSOR_SATURATION							= 0x46,
};

enum IRPacket {
	VINT_PACKET_TYPE_IR_DATA_RECEIVED									= 0x30,
	VINT_PACKET_TYPE_IR_DATA_SEND										= 0x31,
	VINT_PACKET_TYPE_IR_READY											= 0x32,
	VINT_PACKET_TYPE_IR_DATA_DEFINE										= 0x33,
};

enum LCDPacket {
	VINT_PACKET_TYPE_GRAPHICLCD_SETBACKLIGHT							= 0x20,
	VINT_PACKET_TYPE_GRAPHICLCD_SETCONTRAST								= 0x21,
	VINT_PACKET_TYPE_GRAPHICLCD_DRAWPIXEL								= 0x22,
	VINT_PACKET_TYPE_GRAPHICLCD_DRAWLINE								= 0x23,
	VINT_PACKET_TYPE_GRAPHICLCD_DRAWRECT								= 0x24,
	VINT_PACKET_TYPE_GRAPHICLCD_WRITETEXT								= 0x25,
	VINT_PACKET_TYPE_GRAPHICLCD_WRITEBITMAP								= 0x26,
	VINT_PACKET_TYPE_GRAPHICLCD_BITMAPDATA								= 0x27,
	VINT_PACKET_TYPE_GRAPHICLCD_FLUSH									= 0x28,
	VINT_PACKET_TYPE_GRAPHICLCD_CLEAR									= 0x29,
	VINT_PACKET_TYPE_GRAPHICLCD_SETSLEEP								= 0x2A,
	VINT_PACKET_TYPE_GRAPHICLCD_SETFRAMEBUFFER							= 0x2B,
	VINT_PACKET_TYPE_GRAPHICLCD_SAVEFRAMEBUFFER							= 0x2C,
	VINT_PACKET_TYPE_GRAPHICLCD_COPY									= 0x2D,
	VINT_PACKET_TYPE_GRAPHICLCD_BITMAPDATA2								= 0x2E,
	VINT_PACKET_TYPE_GRAPHICLCD_INIT									= 0x2F,
};

enum LightSensorPacket {
	VINT_PACKET_TYPE_LIGHTSENSOR_SETILLUMINANCECHANGETRIGGER			= 0x50,
	VINT_PACKET_TYPE_LIGHTSENSOR_ILLUMINANCECHANGE						= 0x51,
	VINT_PACKET_TYPE_LIGHTSENSOR_SATURATION								= 0x52,
};

enum MagnetometerPacket {
	VINT_PACKET_TYPE_MAGNETOMETER_RESET_CORRECTION_PARAMETERS			= 0x4D,
	VINT_PACKET_TYPE_MAGNETOMETER_SET_CORRECTION_PARAMETERS				= 0x4E,
	VINT_PACKET_TYPE_MAGNETOMETER_SAVECORRECTIONPARAMETERS				= 0x4F,
};

enum PCLPacket {
	VINT_PACKET_TYPE_PCL_ENABLE											= 0xB0,
	VINT_PACKET_TYPE_PCL_DUTYCYCLELIMIT									= 0xB1,
	VINT_PACKET_TYPE_PCL_TARGETPOSITION									= 0xB2,
	VINT_PACKET_TYPE_PCL_POSITIONECHO									= 0xB3,
	VINT_PACKET_TYPE_PCL_ACCELERATION									= 0xB4,
};

enum PHPacket {
	VINT_PACKET_TYPE_PHSENSOR_PHCHANGE									= 0xB9,
	VINT_PACKET_TYPE_PHSENSOR_SATURATION								= 0xBA,
	VINT_PACKET_TYPE_PHSENSOR_SETPHCHANGETRIGGER						= 0xBB,
	VINT_PACKET_TYPE_PHSENSOR_SETCORRECTIONTEMPERATURE					= 0xBC,
};

enum PowerGuardPacket {
	VINT_PACKET_TYPE_POWERGUARD_SETFANMODE								= 0x9F,
	VINT_PACKET_TYPE_POWERGUARD_POWERON									= 0xA0,
	VINT_PACKET_TYPE_POWERGUARD_POWEROFF								= 0xA1,
	VINT_PACKET_TYPE_POWERGUARD_SETOVERVOLTAGE							= 0xA2,
	VINT_PACKET_TYPE_POWERGUARD_OVERVOLTAGE								= 0xA3,
	VINT_PACKET_TYPE_POWERGUARD_ENERGYDUMP								= 0xA4,
	VINT_PACKET_TYPE_POWERGUARD_OVERTEMPERATURE							= 0xA5,
	VINT_PACKET_TYPE_POWERGUARD_SETPOWERENABLED							= 0xA6,
	VINT_PACKET_TYPE_POWERGUARD_FAILSAFE								= 0xA7,
};

enum PressureSensorPacket {
	VINT_PACKET_TYPE_PRESSURESENSOR_SETPRESSURECHANGETRIGGER			= 0x40,
	VINT_PACKET_TYPE_PRESSURESENSOR_PRESSURECHANGE						= 0x41,
	VINT_PACKET_TYPE_PRESSURESENSOR_SATURATION							= 0x42,
	VINT_PACKET_TYPE_PRESSURESENSOR_BARALTITUDE							= 0x43,
};

enum RCServoPacket {
	VINT_PACKET_TYPE_RCSERVO_SETTARGETPOSITION							= 0x58,
	VINT_PACKET_TYPE_RCSERVO_SETVELOCITYLIMIT							= 0x59,
	VINT_PACKET_TYPE_RCSERVO_SETACCELERATION							= 0x5A,
	VINT_PACKET_TYPE_RCSERVO_MOTORSTATUSUPDATE							= 0x60,
	VINT_PACKET_TYPE_RCSERVO_ARRIVED_AT_TARGET_POSITION					= 0x70,
	VINT_PACKET_TYPE_RCSERVO_SETENABLED									= 0x71,
	VINT_PACKET_TYPE_RCSERVO_SETMINPULSEWIDTH							= 0x72,
	VINT_PACKET_TYPE_RCSERVO_SETMAXPULSEWIDTH							= 0x73,
	VINT_PACKET_TYPE_RCSERVO_BAD_POWER_SUPPLY							= 0x74,
	VINT_PACKET_TYPE_RCSERVO_FAILSAFEPOS								= 0x75,
	VINT_PACKET_TYPE_RCSERVO_FAILSAFE									= 0x79,
	VINT_PACKET_TYPE_RCSERVO_VOLTAGE									= 0x76,
	VINT_PACKET_TYPE_RCSERVO_SETSPEEDRAMPINGSTATE						= 0x77,
	VINT_PACKET_TYPE_RCSERVO_DUTYCYCLELIMIT								= 0x78,
};

enum ResistanceInputPacket {
	VINT_PACKET_TYPE_RESISTANCEINPUT_RESISTANCECHANGE					= 0x9A,
	VINT_PACKET_TYPE_RESISTANCEINPUT_SETRESISTANCECHANGETRIGGER			= 0x9B,
	VINT_PACKET_TYPE_RESISTANCEINPUT_SATURATION							= 0x9C,
};

enum RFIDPacket {
	VINT_PACKET_TYPE_RFID_TAG											= 0x60,
	VINT_PACKET_TYPE_RFID_TAG_LOST										= 0x61,
	VINT_PACKET_TYPE_RFID_TAG_WRITE										= 0x62,
	VINT_PACKET_TYPE_RFID_ANTENNA_ON									= 0x63,
};

enum SoundSensorPacket {
	VINT_PACKET_TYPE_SOUNDSENSOR_SATURATION								= 0x90,
	VINT_PACKET_TYPE_SOUNDSENSOR_SETSPLCHANGETRIGGER					= 0x91,
	VINT_PACKET_TYPE_SOUNDSENSOR_DBCHANGE								= 0x92,
	VINT_PACKET_TYPE_SOUNDSENSOR_SPLRANGE								= 0x93,
};

enum SpatialPacket {
	VINT_PACKET_TYPE_SPATIAL_ZERO_AHRS									= 0x50,
	VINT_PACKET_TYPE_SPATIAL_AHRS_ALGORITHM								= 0x51,
	VINT_PACKET_TYPE_SPATIAL_AHRS_MAG_GAIN								= 0x52,
	VINT_PACKET_TYPE_SPATIAL_RESET_AHRS									= 0x53,
	VINT_PACKET_TYPE_SPATIAL_SET_AHRS_PARAMS							= 0x54,
	VINT_PACKET_TYPE_SPATIAL_SET_HEATING								= 0x55,
};

enum StepperPacket {
	VINT_PACKET_TYPE_STEPPER_SETCONTROLMODE								= 0x77,
	VINT_PACKET_TYPE_STEPPER_SETENGAGED									= 0x78,
	VINT_PACKET_TYPE_STEPPER_STATUSUPDATE								= 0x7B,
	VINT_PACKET_TYPE_STEPPER_SETFAILSAFEMODE							= 0x7C,
	VINT_PACKET_TYPE_STEPPER_SETDIRECTION								= 0x7D,
	VINT_PACKET_TYPE_STEPPER_SETFAILURETIMEOUT							= 0x7E,
	VINT_PACKET_TYPE_STEPPER_SETCURRENTLIMIT							= 0x7F,
	VINT_PACKET_TYPE_STEPPER_SETHOLDINGCURRENTLIMIT						= 0x7A,
	VINT_PACKET_TYPE_STEPPER_POWER_ERROR								= 0x76,
	VINT_PACKET_TYPE_STEPPER_FAULT_ERROR								= 0x77,
	VINT_PACKET_TYPE_STEPPER_OVERTEMPERATURE							= 0x78,
};

enum TemperatureSensorPacket {
	VINT_PACKET_TYPE_TEMPERATURESENSOR_SETTEMPERATURECHANGETRIGGER		= 0x3A,
	VINT_PACKET_TYPE_TEMPERATURESENSOR_TEMPERATURECHANGE				= 0x3B,
	VINT_PACKET_TYPE_TEMPERATURESENSOR_SATURATION						= 0x3C,
	VINT_PACKET_TYPE_TEMPERATURESENSOR_IREMISSIVITY						= 0x3D,
	VINT_PACKET_TYPE_TEMPERATURESENSOR_THERMOCOUPLETYPE					= 0x3E,
	VINT_PACKET_TYPE_TEMPERATURESENSOR_RTDTYPE							= 0x3F,
	VINT_PACKET_TYPE_TEMPERATURESENSOR_BADCONNECTION					= 0x40,
};

enum VoltageInputPacket {
	VINT_PACKET_TYPE_VOLTAGEINPUT_SETVOLTAGECHANGETRIGGER				= 0x10,
	VINT_PACKET_TYPE_VOLTAGEINPUT_VOLTAGECHANGE							= 0x11,
	VINT_PACKET_TYPE_VOLTAGEINPUT_SATURATION							= 0x12,
	VINT_PACKET_TYPE_VOLTAGEINPUT_VCP1001RANGE							= 0x13,
	VINT_PACKET_TYPE_VOLTAGEINPUT_VCP1002RANGE							= 0x14,
	VINT_PACKET_TYPE_VOLTAGEINPUT_VCP1000RANGE							= 0x15,
	VINT_PACKET_TYPE_VOLTAGEINPUT_PHRANGE								= 0x16,
	VINT_PACKET_TYPE_VOLTAGEINPUT_BADCONNECTION							= 0x17,
};

enum VoltageOutputPacket {
	VINT_PACKET_TYPE_VOLTAGEOUTPUT_SETENABLED							= 0x04,
	VINT_PACKET_TYPE_VOLTAGEOUTPUT_SETVOLTAGE							= 0x05,
	VINT_PACKET_TYPE_VOLTAGEOUTPUT_VOLTAGEERROR							= 0x06,
	VINT_PACKET_TYPE_VOLTAGEOUTPUT_SETMODE								= 0x07,
	VINT_PACKET_TYPE_VOLTAGEOUTPUT_FAILSAFE								= 0x0E,
};

enum VoltageRatioInputPacket {
	VINT_PACKET_TYPE_VOLTAGERATIOINPUT_SETVOLTAGERATIOCHANGETRIGGER		= 0x1A,
	VINT_PACKET_TYPE_VOLTAGERATIOINPUT_VOLTAGERATIOCHANGE				= 0x1B,
	VINT_PACKET_TYPE_VOLTAGERATIOINPUT_SATURATION						= 0x1C,
	VINT_PACKET_TYPE_VOLTAGERATIOINPUT_BRIDGEGAIN						= 0x1D,
	VINT_PACKET_TYPE_VOLTAGERATIOINPUT_BRIDGEENABLED					= 0x1E
};

/**
 * Packets used within the bootloader area
 */
typedef enum {
	VINT_BOOTLOADER_PACKET_TYPE_DEVICEINFO  							= 0x01,
	VINT_BOOTLOADER_PACKET_TYPE_FIRMWAREUPGRADEDATA						= 0x02
} VINTBootloaderPacketType;

/* Return codes for data sent PC -> hub -> device */
typedef enum {
	/* Define status codes in the range 0xE0 - 0xFE */
	VINTPacketStatusCode_ACK 											= 0xE0,		/* Success */
	VINTPacketStatusCode_NAK 											= 0xE1,		/* Not ready. Library should try to re-send. */
	VINTPacketStatusCode_INVALIDARG 									= 0xE2,		/* A provided argument contained an invalid value */
	VINTPacketStatusCode_INVALIDPACKETTYPE 								= 0xE3,		/* The specified packet type is not used on this device */
	VINTPacketStatusCode_INVALIDSEQUENCE 								= 0xE4,		/* The received packet was not sent in the required sequence */
	VINTPacketStatusCode_INVALIDCOMMAND 								= 0xE5,		/* The requested action cannot be performed due to configuration/state of device */
	VINTPacketStatusCode_MALFORMED 										= 0xE6,		/* The packet length/contents did not match those specified in the documentation */
	VINTPacketStatusCode_NOSPACE 										= 0xE7,		/* There was not enough space in the hub queue for this packet */
	VINTPacketStatusCode_UNEXPECTED 									= 0xE8,		/* Generic unexpected status. Should not happen if firmware is properly written */
	VINTPacketStatusCode_NOTATTACHED									= 0xE9,		/* VINT Device no longer attached. */
	VINTPacketStatusCode_TOOBIG											= 0xEA,		/* Packet too long */
	VINTPacketStatusCode_FAILSAFE										= 0xEB,		/* The device's failsafe condition has tripped, and all packets but RESET will get this response. */
} VINTPacketStatusCode;

#endif

/* Dynamixel Communication 2.0 */
/* Implements the dynamixel communication protocol for XL-320. */
#ifndef MOTORS_H
#define MOTORS_H

#include "mbed.h"

class DM2 {
 
public:
 	// Create Dynamixel Communication protocol 2.0
    DM2(PinName tx, PinName rx, int baud=1000000);

    int Level;

    int Test(int ID=1);
    int Ping(int ID=1);
    int SetID(int ID, int newID);
    int SetBaud(int ID, int rate);
    int SetReturnLevel(int ID, int lvl);
    int SetLED(int ID, int colour); 
    int Rainbow(int ID);
    int SetP(int ID, int value);
	int SetI(int ID, int value);
	int SetD(int ID, int value);
	int SetGoalPosition(int ID, int angle);
	int SetGoalVelocity(int ID, int velocity);
	int SetGoalTorque(int ID, int torque);
	int SetPunch(int ID, int punch);
	int GetValue(int ID, int address, int *val);

private :
	// REPLY BUFFER
    static unsigned char recycle[255];

	int statusError(unsigned char* buf, int n);
	int length(unsigned char* buf);
	void flush();
	void packetPrint(int bytes, unsigned char* buf);

	void write(unsigned char* buf, int n);
	int read(int ID, unsigned char* buf, int nMax=255);
	int send(int ID, int bytes, unsigned char* data, unsigned char ins=0x03, unsigned char* reply=recycle);

	//int ping(int ID);
	int dataPack(int start, unsigned char* data, int address, int value);
	int dataPush(int ID, int address, int value);
	int dataPull(int ID, int address, int* val);

    Serial _out;
    Serial _in;
    int _baud;
    int _bitPeriod;
};

// EEPROM 
#define XL_MODEL_NUMBER_L 0
#define XL_MODEL_NUMBER_H 1
#define XL_VERSION 2
#define XL_ID 3
#define XL_BAUD_RATE 4
#define XL_RETURN_DELAY_TIME 5
#define XL_CW_ANGLE_LIMIT_L 6
#define XL_CW_ANGLE_LIMIT_H 7
#define XL_CCW_ANGLE_LIMIT_L 8
#define XL_CCW_ANGLE_LIMIT_H 9
#define XL_CONTROL_MODE 11
#define XL_LIMIT_TEMPERATURE 12
#define XL_DOWN_LIMIT_VOLTAGE 13
#define XL_UP_LIMIT_VOLTAGE 14
#define XL_MAX_TORQUE_L 15
#define XL_MAX_TORQUE_H 16
#define XL_RETURN_LEVEL 17
#define XL_ALARM_SHUTDOWN 18
// RAM 
#define XL_TORQUE_ENABLE 24
#define XL_LED 25
#define XL_D_GAIN 27
#define XL_I_GAIN 28
#define XL_P_GAIN 29
#define XL_GOAL_POSITION_L 30
#define XL_GOAL_SPEED_L 32
#define XL_GOAL_TORQUE 35
#define XL_PRESENT_POSITION 37
#define XL_PRESENT_SPEED 39
#define XL_PRESENT_LOAD 41
#define XL_PRESENT_VOLTAGE 45
#define XL_PRESENT_TEMPERATURE 46
#define XL_REGISTERED_INSTRUCTION 47
#define XL_MOVING 49
#define XL_HARDWARE_ERROR 50
#define XL_PUNCH 51
// INS
const unsigned char INS_Ping = 0x01;         // Corresponding device ID command to check if packet reaches
const unsigned char INS_Read = 0x02;         // Read command
const unsigned char INS_Write = 0x03;        // Write command
const unsigned char INS_RegWrite = 0x04;     // When receiving a write command packet data is not immediately written instead it goes into standby momentarily until action command arrives
const unsigned char INS_Action = 0x05;       // Go command for Reg Write
const unsigned char INS_Factory = 0x06;      // Reset All data to factory default settings
const unsigned char INS_Reboot = 0x08;       // Reboot device
const unsigned char INS_StatusReturn = 0x55; // Instruction Packet response
const unsigned char INS_SyncRead = 0x82;     // Read data from the same location and same size for multiple devices simultaneously
const unsigned char INS_SyncWrite = 0x83;    // Write data from the same location and same size for multiple devices simultaneously
const unsigned char INS_BulkRead = 0x92;     // Read data from the different locations and different sizes for multiple devices simultaneously
const unsigned char INS_BulkWrite = 0x93;    // Write data from the different locations and different sizes for multiple devices simultaneously
// ID
const unsigned char ID_Broadcast = 0xFE; // 254(0xFE) is used as the Broadcast ID
// Util
#define DM_MAKEWORD(a, b) ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) | ((unsigned short)((unsigned char)(((unsigned long)(b)) & 0xff))) << 8))
#define DM_LOBYTE(w) ((unsigned char)(((unsigned long)(w)) & 0xff))
#define DM_HIBYTE(w) ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);

#endif
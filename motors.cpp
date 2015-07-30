#include "motors.h"
#include "mbed.h"
#include <math.h>

DM2::DM2(PinName tx, PinName rx, int baud)
	: _out(tx, NC)
	, _in(NC, rx)
{
	_baud = baud;
	_bitPeriod = 1.0/_baud;
	
	_out.baud(_baud);
	_in.baud(_baud);
}

unsigned char DM2::recycle[255] = {0};

// Dynamixel Communication 2.0 Checksum
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size) {
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++) {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    return crc_accum;
}

int DM2::length(unsigned char* buf) {
	//printf("LENGTH LEN_L 0x%x LEN_H 0x%x = %d ;", buf[5], buf[6], DM_MAKEWORD(buf[5], buf[6]) + 4);
	
	// Header(3) + Reserved(1) + ID(1) +  Packet Length(?) + CRC(2)
	int a = DM_MAKEWORD(buf[5], buf[6]) + 7;
	return a;
}

int DM2::statusError(unsigned char* buf, int n) {
	if (n < 9) {
		flush();
		return 0; // Nil
	}

	if ((buf[0]!=0xFF)||(buf[1]!=0xFF)||(buf[2]!=0xFD)||(buf[3]!=0x00)) {
		flush();
		printf("WRONG HEADER\n");
		return 1; 
	}

	int l = length(buf);
	if ((l < 11) || (l != n)) {
		flush();
		printf("WRONG LENGTH\n");
		return 1;
	}

	printf("STATUS ERROR %i", buf[8]);
	return buf[8]; // Status error
}


// packetPrint
void DM2::packetPrint(int bytes, unsigned char* buf) {
	printf("PACKET {");
	for (int i=0; i < bytes; i++) {
		printf("0x%x ", buf[i]);
	}
	printf("} ");
}

// Flush
void DM2::flush() {
	while (_in.readable()) {
		_in.getc();
	}
}

// Write 
void DM2::write(unsigned char* buf, int n) {
	for (int i=0; i < n; i++) {
		_out.putc(buf[i]);
	}
	for (int i=0; i < n; i++) {
		_in.getc();
	}
}

// Read
// Read reply returns payload length, 0 if error.
int DM2::read(int ID, unsigned char* buf, int nMax) {
	int n = 0; 		 // Bytes read
	int timeout = 0; // Timeout
	//bool flag = true;
	while ((timeout < 500) && (n < nMax)) {
		if (_in.readable()) {
			buf[n] = _in.getc();
			n++;
			timeout = 0;
		}
		
		wait(_bitPeriod);
		timeout++;
		
		/*
		if ((n > 6) && (flag)) {
			int l = length(buf);
			if (l < nMax) {
				nMax = l;
			}
			flag = false;
		}
		*/
	}

	return n;
}

// Send
// Dynamixel Communication 2.0 Protocol
// Header, Reserved, ID, Packet Length, Instruction, Parameter, 16bit CRC
int DM2::send(int ID, int bytes, unsigned char* data, unsigned char ins, unsigned char* reply) {
	unsigned char buf[255]; // Packet

	// Header
	buf[0] = 0xFF;
	buf[1] = 0xFF;
	buf[2] = 0xFD;

	// Reserved
	buf[3] = 0x00;

	// ID
	buf[4] = ID;

	// Packet Length
	buf[5] = DM_LOBYTE(bytes+3);
	buf[6] = DM_HIBYTE(bytes+3);

	// Instruction
	buf[7] = ins;

	// Parameter
	for (int i=0; i < bytes; i++) {
		buf[8+i] = data[i];
	}

	// Checksum
	unsigned short CRC = update_crc(0, buf, bytes+8);
	buf[bytes+8] = DM_LOBYTE(CRC);
	buf[bytes+9] = DM_HIBYTE(CRC);

	// Transmit
	write(buf, bytes+10);

	// Broadcast doesn't reply.
	if ((ID == ID_Broadcast) || (Level == 0) || ((Level == 1) && (ins != INS_Read))) {
		return 0;	
	}
	
	// Read reply
	printf("- Reading\n");
	int n = read(ID, reply);
	if (n == 0) {
		printf("NULL");
		return 0;
	}
	printf("- Read %i bytes\n", n);

	return statusError(reply, n); // Error code
};

int DM2::Test(int ID) {
	unsigned char TxPacket[14] = {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00};
	unsigned short CRC = update_crc ( 0, TxPacket , 12 ) ; // 12 = 5 + Packet Length(0x00 0x07) = 5+7
	printf("CRC %i\n", CRC);
	unsigned char CRC_L = DM_LOBYTE(CRC);
	unsigned char CRC_H = DM_HIBYTE(CRC);
	printf("CRC_L %i\n", CRC_L);
	printf("CRC_H %i\n", CRC_H);

	TxPacket[12] = CRC_L;
	TxPacket[13] = CRC_H;

	printf("TRANMITTING \n");
	for (int i = 0; i < (14); ++i) {
		_out.putc(TxPacket[i]);
		_in.getc(); // Echo
	}

	wait_ms(0.5);

	printf("DATA { ");
	int timeout = 0;
	int plen = 0;
	while ((timeout < 256) && (plen<15)) {
		if (_in.readable()) {
			printf("0x%x ", _in.getc());
			plen++;
			timeout = 0;
		}

		// wait for the bit period
		wait(_bitPeriod);
		timeout++;
	}
	printf("} \n");

	return (0);
}

// dataPack sets data in char array and returns length.
int DM2::dataPack(int start, unsigned char* data, int address, int value){
	data[start+0] = (unsigned char)DM_LOBYTE(address);
	data[start+1] = (unsigned char)DM_HIBYTE(address);
	data[start+2] = DM_LOBYTE(value);
	data[start+3] = DM_HIBYTE(value);

	return start + 4;
}

// dataPush is a generic wrapper for single value set commands.
int DM2::dataPush(int ID, int address, int value){
	flush(); // Flush reply
	unsigned char data[4];
    int bytes = 0;

    bytes = dataPack(bytes, data, address, value);

    return send(ID, bytes, data, INS_Write);
}
int DM2::dataPull(int ID, int address, int *val){
	flush(); // Flush reply
	unsigned char data[4];
    int bytes = 0;

    bytes = dataPack(bytes, data, address, 2);

    unsigned char buf[15] = {0};
   	int ec = send(ID, bytes, data, INS_Read, buf);
   	if (ec != 0) {
   		return ec;
   	}
   	*val = DM_MAKEWORD(buf[9], buf[10]);
	packetPrint(15, buf);
   	return ec;
}


// Ping
int DM2::Ping(int ID){
	unsigned char pong[15] = {0};

	int ec = send(ID, 0, NULL, INS_Ping, pong);
	printf("- ec %i ", ec);
	if (ec != 0) {
		printf("PING { ");
		for (int i = 0; i < 15; ++i){
			printf("0x%x ", pong[i]);
		}
		printf("} \n");
	}

	return ec;
}

// SetID
int DM2::SetID(int ID, int newID){
    return dataPush(ID, XL_ID, newID);
};

// SetBaud
// 0: 9600, 1:57600, 2:115200, 3:1Mbps
int DM2::SetBaud(int ID, int rate) {
	if ((rate > 3) || rate < 0) {
		printf("Incorrect baud rate. \n");
		return 1;
	}
	return dataPush(ID, XL_BAUD_RATE, rate);
}

// SetReturnLevel
// 0: None, 1: Read, 2: All.
int DM2::SetReturnLevel(int ID, int lvl) {
	return dataPush(ID, XL_RETURN_LEVEL, lvl);
}

// SetLED sets motor led colours.
// r = 1, g = 2, y = 3, b = 4, p = 5, c = 6, w = 7, o = 0
int DM2::SetLED(int ID, int colour){
	return dataPush(ID, XL_LED, colour);
}

// Rainbow
int DM2::Rainbow(int ID){
	for (int i = 1; i < 8; ++i)
	{
		int status = SetLED(ID, i);
		if (status != 0) {
			return status;
		}
		wait(1);
	}
	return SetLED(ID, 0);
}

int DM2::SetP(int ID, int value){
	return dataPush(ID, XL_P_GAIN, value);
}
int DM2::SetI(int ID, int value){
	return dataPush(ID, XL_I_GAIN, value);
}
int DM2::SetD(int ID, int value){
	return dataPush(ID, XL_D_GAIN, value);
}

// SetGoalPosition
// 1024 = -150, 512 = 0 (ORIGIN), 0 = +150
int DM2::SetGoalPosition(int ID, int angle){
	return dataPush(ID, XL_GOAL_POSITION_L, angle);
}
int DM2::SetGoalVelocity(int ID, int velocity){
	return dataPush(ID, XL_GOAL_SPEED_L, velocity);
}
int DM2::SetGoalTorque(int ID, int torque){
	return dataPush(ID, XL_GOAL_TORQUE, torque);
}

int DM2::SetPunch(int ID, int punch){
	return dataPush(ID, XL_PUNCH, punch);
}

int DM2::GetValue(int ID, int address, int *val){
	return dataPull(ID, address, val);
}
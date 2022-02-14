/*********************************************************************************
 * Name         : Chimera-2017V Emulator                                         *
 * Author       : Ayas Nasih (S1600655)		                                     *
 * Date         : 12/08/2020 													 *
 * Revision Date: 10/09/2020                                                  	 *
 * Description  : Emulator for the Chimera-2017V Microprocessor                  *
 *********************************************************************************
 */

#include "stdafx.h"
#include <winsock2.h>

#pragma comment(lib, "wsock32.lib")


#define STUDENT_NUMBER    "S1600655"

#define IP_ADDRESS_SERVER "127.0.0.1"

#define PORT_SERVER 0x1984 // We define a port that we are going to use.
#define PORT_CLIENT 0x1985 // We define a port that we are going to use.

#define WORD  unsigned short
#define DWORD unsigned long
#define BYTE  unsigned char

#define MAX_FILENAME_SIZE 500
#define MAX_BUFFER_SIZE   500

SOCKADDR_IN server_addr;
SOCKADDR_IN client_addr;

SOCKET sock;  // This is our socket, it is the handle to the IO address to read/write packets

WSADATA data;

char InputBuffer [MAX_BUFFER_SIZE];

char hex_file [MAX_BUFFER_SIZE];
char trc_file [MAX_BUFFER_SIZE];

//////////////////////////
//   Registers          //
//////////////////////////

#define FLAG_V  0x80
#define FLAG_Z  0x40
#define FLAG_N  0x10
#define FLAG_I  0x04
#define FLAG_C  0x01
#define REGISTER_A	0
#define REGISTER_X 0
#define REGISTER_Y 1
#define REGISTER_P 2
BYTE Index_Registers[3];

BYTE Registers[1];
BYTE Flags;
WORD ProgramCounter;
WORD StackPointer;


////////////
// Memory //
////////////

#define MEMORY_SIZE	65536

BYTE Memory[MEMORY_SIZE];

#define TEST_ADDRESS_1  0x01FA
#define TEST_ADDRESS_2  0x01FB
#define TEST_ADDRESS_3  0x01FC
#define TEST_ADDRESS_4  0x01FD
#define TEST_ADDRESS_5  0x01FE
#define TEST_ADDRESS_6  0x01FF
#define TEST_ADDRESS_7  0x0200
#define TEST_ADDRESS_8  0x0201
#define TEST_ADDRESS_9  0x0202
#define TEST_ADDRESS_10  0x0203
#define TEST_ADDRESS_11  0x0204
#define TEST_ADDRESS_12  0x0205


///////////////////////
// Control variables //
///////////////////////

bool memory_in_range = true;
bool halt = false;


///////////////////////
// Disassembly table //
///////////////////////

char opcode_mneumonics[][14] =
{
"ILLEGAL     ", 
"CLC impl     ", 
"ILLEGAL     ", 
"LDA  #       ", 
"LDA abs      ", 
"LDA abs,X    ", 
"LDA abs,Y    ", 
"LDA abs,XY   ", 
"LDA pag      ", 
"ILLEGAL     ", 
"STO abs      ", 
"STO abs,X    ", 
"STO abs,Y    ", 
"STO abs,XY   ", 
"STO pag      ", 
"ILLEGAL     ", 

"TAY impl     ", 
"STC impl     ", 
"ILLEGAL     ", 
"ADC  #       ", 
"ADC abs      ", 
"ADC abs,X    ", 
"ADC abs,Y    ", 
"ADC abs,XY   ", 
"ADC pag      ", 
"TST abs      ", 
"TST abs,X    ", 
"TST abs,Y    ", 
"TST abs,XY   ", 
"TSTA A,A     ", 
"PSHA impl    ", 
"PSHs impl    ", 

"TYA impl     ", 
"CLI impl     ", 
"ILLEGAL     ", 
"SBC  #       ", 
"SBC abs      ", 
"SBC abs,X    ", 
"SBC abs,Y    ", 
"SBC abs,XY   ", 
"SBC pag      ", 
"INC abs      ", 
"INC abs,X    ", 
"INC abs,Y    ", 
"INC abs,XY   ", 
"INCA A,A     ", 
"POPA impl    ", 
"POPs impl    ", 

"TSA impl     ", 
"STI impl     ", 
"ILLEGAL     ", 
"ADD  #       ", 
"ADD abs      ", 
"ADD abs,X    ", 
"ADD abs,Y    ", 
"ADD abs,XY   ", 
"ADD pag      ", 
"DEC abs      ", 
"DEC abs,X    ", 
"DEC abs,Y    ", 
"DEC abs,XY   ", 
"DECA A,A     ", 
"ILLEGAL     ", 
"JMPR abs     ", 

"TAP impl     ", 
"SEV impl     ", 
"DECX impl    ", 
"SUB  #       ", 
"SUB abs      ", 
"SUB abs,X    ", 
"SUB abs,Y    ", 
"SUB abs,XY   ", 
"SUB pag      ", 
"RCR abs      ", 
"RCR abs,X    ", 
"RCR abs,Y    ", 
"RCR abs,XY   ", 
"RCRA A,A     ", 
"SRA impl     ", 
"CCC abs      ", 

"TPA impl     ", 
"CLV impl     ", 
"INCX impl    ", 
"CMP  #       ", 
"CMP abs      ", 
"CMP abs,X    ", 
"CMP abs,Y    ", 
"CMP abs,XY   ", 
"CMP pag      ", 
"RCL abs      ", 
"RCL abs,X    ", 
"RCL abs,Y    ", 
"RCL abs,XY   ", 
"RCLA A,A     ", 
"SCC impl     ", 
"CCS abs      ", 

"ILLEGAL     ", 
"CMC impl     ", 
"DEY impl     ", 
"OR  #        ", 
"OR abs       ", 
"OR abs,X     ", 
"OR abs,Y     ", 
"OR abs,XY    ", 
"OR pag       ", 
"SHL abs      ", 
"SHL abs,X    ", 
"SHL abs,Y    ", 
"SHL abs,XY   ", 
"SHLA A,A     ", 
"SCS impl     ", 
"CNE abs      ", 

"JP abs       ", 
"CMV impl     ", 
"INY impl     ", 
"AND  #       ", 
"AND abs      ", 
"AND abs,X    ", 
"AND abs,Y    ", 
"AND abs,XY   ", 
"AND pag      ", 
"SAR abs      ", 
"SAR abs,X    ", 
"SAR abs,Y    ", 
"SAR abs,XY   ", 
"SARA A,A     ", 
"SNE impl     ", 
"CEQ abs      ", 

"JCC abs      ", 
"SWI impl     ", 
"DEP impl     ", 
"EOR  #       ", 
"EOR abs      ", 
"EOR abs,X    ", 
"EOR abs,Y    ", 
"EOR abs,XY   ", 
"EOR pag      ", 
"LSR abs      ", 
"LSR abs,X    ", 
"LSR abs,Y    ", 
"LSR abs,XY   ", 
"LSRA A,A     ", 
"SEQ impl     ", 
"CVC abs      ", 

"JCS abs      ", 
"RTI impl     ", 
"INP impl     ", 
"BT  #        ", 
"BT abs       ", 
"BT abs,X     ", 
"BT abs,Y     ", 
"BT abs,XY    ", 
"BT pag       ", 
"NOT abs      ", 
"NOT abs,X    ", 
"NOT abs,Y    ", 
"NOT abs,XY   ", 
"NOTA A,A     ", 
"SVC impl     ", 
"CVS abs      ", 

"JNE abs      ", 
"LDX  #       ", 
"LDX abs      ", 
"LDX abs,X    ", 
"LDX abs,Y    ", 
"LDX abs,XY   ", 
"LDX pag      ", 
"RTS impl     ", 
"ILLEGAL     ", 
"NEG abs      ", 
"NEG abs,X    ", 
"NEG abs,Y    ", 
"NEG abs,XY   ", 
"NEGA A,0     ", 
"SVS impl     ", 
"CMI abs      ", 

"JEQ abs      ", 
"LDY  #       ", 
"LDY abs      ", 
"LDY abs,X    ", 
"LDY abs,Y    ", 
"LDY abs,XY   ", 
"LDY pag      ", 
"ILLEGAL     ", 
"ILLEGAL     ", 
"RAL abs      ", 
"RAL abs,X    ", 
"RAL abs,Y    ", 
"RAL abs,XY   ", 
"RALA A,A     ", 
"SMI impl     ", 
"CPL abs      ", 

"JVC abs      ", 
"LDP  #       ", 
"LDP abs      ", 
"LDP abs,X    ", 
"LDP abs,Y    ", 
"LDP abs,XY   ", 
"LDP pag      ", 
"ILLEGAL     ", 
"ILLEGAL     ", 
"ROR abs      ", 
"ROR abs,X    ", 
"ROR abs,Y    ", 
"ROR abs,XY   ", 
"RORA A,A     ", 
"SPL impl     ", 
"CHI abs      ", 

"JVS abs      ", 
"ILLEGAL     ", 
"ILLEGAL     ", 
"STX abs      ", 
"STX abs,X    ", 
"STX abs,Y    ", 
"STX abs,XY   ", 
"STX pag      ", 
"ILLEGAL     ", 
"CLR abs      ", 
"CLR abs,X    ", 
"CLR abs,Y    ", 
"CLR abs,XY   ", 
"CLRA A,0     ", 
"SGE impl     ", 
"CLE abs      ", 

"JMI abs      ", 
"NOP impl     ", 
"ILLEGAL     ", 
"STY abs      ", 
"STY abs,X    ", 
"STY abs,Y    ", 
"STY abs,XY   ", 
"STY pag      ", 
"LODS  #      ", 
"LODS abs     ", 
"LODS abs,X   ", 
"LODS abs,Y   ", 
"LODS abs,XY  ", 
"LODS pag     ", 
"SGT impl     ", 
"ILLEGAL     ", 

"JPL abs      ", 
"HALT impl    ", 
"ILLEGAL     ", 
"STP abs      ", 
"STP abs,X    ", 
"STP abs,Y    ", 
"STP abs,XY   ", 
"STP pag      ", 
"ILLEGAL     ", 
"ILLEGAL     ", 
"ILLEGAL     ", 
"STOS abs     ", 
"STOS abs,X   ", 
"STOS abs,Y   ", 
"STOS abs,XY  ", 
"STOS pag     ", 

}; 

////////////////////////////////////////////////////////////////////////////////
//                           Simulator/Emulator (Start)                       //
////////////////////////////////////////////////////////////////////////////////
BYTE fetch()
{
	BYTE byte = 0;

	if ((ProgramCounter >= 0) && (ProgramCounter <= MEMORY_SIZE))
	{
		memory_in_range = true;
		byte = Memory[ProgramCounter];
		ProgramCounter++;
	}
	else
	{
		memory_in_range = false;
	}
	return byte;
}

/**
 *  Gets Absolute Address (abs)
 *
 * params:
 *      None
 *
 * returns:
 *     WORD: Absolute Address
 * 
 */
WORD address_ABS()
{ //ABS
	BYTE LB = 0;
	BYTE HB = 0;
	WORD address = 0;
	LB = fetch();
	HB = fetch();
	address += (WORD)((WORD)HB << 8) + LB;

	return address;
}

/**
 *  Gets Absolute Address with Index Registers X (abs, X)
 *
 * params:
 *      None
 *
 * returns:
 *     WORD: Absolute Address (X)
 * 
 */
WORD address_ABSX()
{ //ABSX
	BYTE LB = 0;
	BYTE HB = 0;
	WORD address = 0;
	address += Index_Registers[REGISTER_X];
	LB = fetch();
	HB = fetch();
	address += (WORD)((WORD)HB << 8) + LB;
	return address;
}

/**
 *  Gets Absolute Address with Index Registers Y (abs, Y)
 *
 * params:
 *      None
 *
 * returns:
 *     WORD: Absolute Address (Y)
 * 
 */
WORD address_ABSY()
{ //ASBY
	BYTE LB = 0;
	BYTE HB = 0;
	WORD address = 0;
	address += Index_Registers[REGISTER_Y];
	LB = fetch();
	HB = fetch();
	address += (WORD)((WORD)HB << 8) + LB;

	return address;
}

/**
 *  Gets Absolute Address with Index Registers X and Y (abs, XY)
 *
 * params:
 *      None
 *
 * returns:
 *     WORD: Absolute Address (XY)
 * 
 */
WORD address_ABSXY()
{
	BYTE LB = 0;
	BYTE HB = 0;
	WORD address = 0;
	address += (WORD)((WORD)Index_Registers[REGISTER_Y] << 8) + Index_Registers[REGISTER_X];
	LB = fetch();
	HB = fetch();
	address += (WORD)((WORD)HB << 8) + LB;

	return address;
}


/**
 *  Gets Paged Address (pag)
 *
 * params:
 *      None
 *
 * returns:
 *     WORD: Paged Address
 * 
 */
WORD address_PAG()
{
	WORD high = Index_Registers[REGISTER_P];
	WORD low = fetch();
	WORD address = 0;
	address += (WORD)((WORD)high << 8) + low;
	return address;
}

/**
 * Checks whether the address is within addressable range.
 *
 * params:
 *      WORD addr: The address
 *
 * returns:
 *     A boolean indicating validity.
 * 
 */
bool addressable(WORD addr){
	return (addr >= 0 && addr < MEMORY_SIZE);
}

/**
 * Loads a byte from memory into a location
 *
 * params:
 *      WORD *dst: Pointer to location to which memory must be assigned.
 *      WORD addr: An address
 *
 * returns:
 *     None
 * 
 */
void load(BYTE *dst, WORD addr) {
	if(addressable(addr)) {
		*dst = Memory[addr];
	}
}

/**
 * Loads a word from memory into a location
 *
 * params:
 *      WORD *dst: Pointer to location to which memory must be assigned.
 *      WORD addr: An address
 *
 * returns:
 *     None
 * 
 */
void loadW(WORD *dst, WORD addr) {
	if(addressable(addr)) {
		*dst = Memory[addr];
	}
}

/**
 * Stores byte data in an address
 *
 * params:
 *      WORD address: the address to store data in
 *      BYTE loc: The location of data that needs to be stored
 *
 * returns:
 *     None
 * 
 */
void storeMemory(WORD address, BYTE loc)
{
	if (addressable(address))
	{
		Memory[address] = loc;
	}
}

/**
 * Clears the given flag
 *
 * params:
 *      int flag: A flag
 *
 * returns:
 *     None
 * 
 */
void clearFlag(int flag) { 
	Flags &= 0xFF - flag; 
}

/**
 * Sets the given flag
 *
 * params:
 *      int flag: A flag
 *
 * returns:
 *     None
 * 
 */
void setFlag(int flag) { 
	Flags |= flag; 
}

/**
 * Checks if a flag is set
 *
 * params:
 *      int flag: A flag
 *
 * returns:
 *     Boolean true if flag is set, False if flag is not set.
 * 
 */
bool checkFlag(int flag) { 
	return (Flags & flag) == flag; 
}

/**
 * Conditionally sets Negative flag based on data passed.
 *
 * params:
 *      BYTE inReg: A byte of data
 *
 * returns:
 *     None
 * 
 */
void set_flag_n(BYTE inReg)
{
	BYTE reg;
	reg = inReg;

	if ((reg & 0x80) != 0)
	{
		setFlag(FLAG_N);
	}
	else
	{
		clearFlag(FLAG_N);
	}
}

/**
 * Conditionally sets Zero flag based on data passed.
 *
 * params:
 *      BYTE inReg: A byte of data
 *
 * returns:
 *     None
 * 
 */
void set_flag_z(BYTE inReg)
{
	BYTE reg;
	reg = inReg;
	if (reg == 0)
	{
		setFlag(FLAG_Z);
	}
	else
	{
		clearFlag(FLAG_Z);
	}
}

/**
 * Evaluates whether or not to set Carry flag based on whether the WORD passed >= 256bits.
 *
 * params:
 *      WORD inReg: A WORD of data
 *
 * returns:
 *     None
 * 
 */
void set_flag_c(WORD inReg)
{
	WORD temp_word = inReg;
	if (temp_word >= 0x100)
	{
		setFlag(FLAG_C);
	}
	else
	{
		clearFlag(FLAG_C);
	}
}

/**
 *  Used to set both negative and zero flag at the same time.
 *
 * params:
 *      BYTE inReg: A byte of data
 *
 * returns:
 *     None
 * 
 */
void set_flag_zn(BYTE inReg)
{
	set_flag_n(inReg);
	set_flag_z(inReg);
}

/**
 *  Sets overflow flag if there is an overflow or borrow on the most significant bit.
 *
 * params:
 *      BYTE in1: A BYTE of data
 *      BYTE in2: A BYTE of data
 *      BYTE out1: A BYTE of data
 *
 * returns:
 *     None
 * 
 */
void set_flag_v(BYTE in1, BYTE in2, BYTE out1)
{
	BYTE reg1in;
	BYTE reg2in;
	BYTE regOut;
	reg1in = in1;
	reg2in = in2;
	regOut = out1;
	if ((((reg1in & 0x80) == 0x80) && ((reg2in & 0x80) == 0x80) &&
		 (((BYTE)regOut & 0x80) != 0x80)) ||
		(((reg1in & 0x80) != 0x80) &&
		 ((reg2in & 0x80) != 0x80) && (((BYTE)regOut & 0x80) == 0x80)))
	{
		setFlag(FLAG_V);
	}
	else
	{
		Flags = Flags & (~FLAG_V);
	}
}


/**
 *  Pushes the ProgramCounter onto the StackPointer
 *
 * params:
 *      None
 *
 * returns:
 *     None
 * 
 */
void ppushs() {
	if ((StackPointer >= 2) && (StackPointer < MEMORY_SIZE)) // ProgramCounter is 16bit
	{
		Memory[StackPointer] = (BYTE)(ProgramCounter & 0xFF);
		StackPointer--;
		Memory[StackPointer] = (BYTE)((ProgramCounter >> 8) & 0xFF);
		StackPointer--;
	}
}

/**
 *  Bit test Memory or Accumulator
 *
 * params:
 *      WORD: An address
 *
 * returns:
 *     None
 * 
 */
void TST(WORD address)
{
	if (address >= 0 && address < MEMORY_SIZE)
	{
		Memory[address] = Memory[address] - 0x00;
	}
	set_flag_zn(Memory[address]);
}

void Group_1(BYTE opcode){
	BYTE LB = 0;
	BYTE HB = 0;
	WORD address = 0;
	WORD data = 0;
	WORD temp_word = 0;
	BYTE param1 = 0;
	WORD param2 = 0;
	BYTE saved_flags = 0;

	switch(opcode) {


		/**
		 * 
		 * LDA - Loads Memory intoAccumulator
		 * 
		 **/

		case 0x03: // #
			data = fetch();
			Registers[REGISTER_A] = data;
			set_flag_zn(Registers[REGISTER_A]);
			clearFlag(FLAG_C);
			break;

		case 0x04: // abs
			address += address_ABS();
			load(&Registers[REGISTER_A], address);
			set_flag_zn(Registers[REGISTER_A]);
			clearFlag(FLAG_C);
			break;

		case 0x05: // abs,X
			address += address_ABSX();
			load(&Registers[REGISTER_A], address);
			set_flag_zn(Registers[REGISTER_A]);
			clearFlag(FLAG_C);
			break;

		case 0x06: // abs,Y
			address += address_ABSY();
			load(&Registers[REGISTER_A], address);
			set_flag_zn(Registers[REGISTER_A]);
			clearFlag(FLAG_C);
			break;

		case 0x07: // abs,XY
			address += address_ABSXY();
			load(&Registers[REGISTER_A], address);
			set_flag_zn(Registers[REGISTER_A]);
			clearFlag(FLAG_C);
			break;

		case 0x08: // pag
			address += address_PAG();
			load(&Registers[REGISTER_A], address);
			set_flag_zn(Registers[REGISTER_A]);
			clearFlag(FLAG_C);
			break;


		/**
		 * 
		 * STO - Stores Accumulator into Memory
		 * 
		 **/

		case 0x0A: // abs
			address += address_ABS();
			storeMemory(address, Registers[REGISTER_A]);
			set_flag_zn(Memory[address]);
			clearFlag(FLAG_C);
			break;

		case 0x0B: // abs,X
			address += address_ABSX();
			storeMemory(address, Registers[REGISTER_A]);
			set_flag_zn(Memory[address]);
			clearFlag(FLAG_C);
			break;

		case 0x0C: // abs,Y
			address += address_ABSY();
			storeMemory(address, Registers[REGISTER_A]);
			set_flag_zn(Memory[address]);
			clearFlag(FLAG_C);
			break;

		case 0x0D: // abs,XY
			address += address_ABSXY();
			storeMemory(address, Registers[REGISTER_A]);
			set_flag_zn(Memory[address]);
			clearFlag(FLAG_C);
			break;

		case 0x0E: // pag
			address += address_PAG();
			storeMemory(address, Registers[REGISTER_A]);
			set_flag_zn(Memory[address]);
			clearFlag(FLAG_C);
			break;


		/**
		 * 
		 * LDY - Loads Memory into register Y
		 * 
		 **/

		case 0xB1: // #
			data = fetch();
			Index_Registers[REGISTER_Y] = data;
			set_flag_zn(Index_Registers[REGISTER_Y]);
			clearFlag(FLAG_C);
			break;

		case 0xB2: // abs
			address += address_ABS();
			load(&Index_Registers[REGISTER_Y], address);
			set_flag_zn(Index_Registers[REGISTER_Y]);
			clearFlag(FLAG_C);
			break;

		case 0xB3: // abs,X
			address += address_ABSX();
			load(&Index_Registers[REGISTER_Y], address);
			set_flag_zn(Index_Registers[REGISTER_Y]);
			clearFlag(FLAG_C);
			break;

		case 0xB4: // abs,Y
			address += address_ABSY();
			load(&Index_Registers[REGISTER_Y], address);
			set_flag_zn(Index_Registers[REGISTER_Y]);
			clearFlag(FLAG_C);
			break;

		case 0xB5: // abs,XY
			address += address_ABSXY();
			load(&Index_Registers[REGISTER_Y], address);
			set_flag_zn(Index_Registers[REGISTER_Y]);
			clearFlag(FLAG_C);
			break;

		case 0xB6: // pag
			address += address_PAG();
			load(&Index_Registers[REGISTER_Y], address);
			set_flag_zn(Index_Registers[REGISTER_Y]);
			clearFlag(FLAG_C);
			break;

		/**
		 * 
		 * STY - Stores register Y into Memory
		 * 
		 **/

		case 0xE3: // abs
			address += address_ABS();
			storeMemory(address, Index_Registers[REGISTER_Y]);
			set_flag_zn(Memory[address]);
			clearFlag(FLAG_C);
			break;

		case 0xE4: // abs,X
			address += address_ABSX();
			storeMemory(address, Index_Registers[REGISTER_Y]);
			set_flag_zn(Memory[address]);
			clearFlag(FLAG_C);
			break;

		case 0xE5: // abs,Y
			address += address_ABSY();
			storeMemory(address, Index_Registers[REGISTER_Y]);
			set_flag_zn(Memory[address]);
			clearFlag(FLAG_C);
			break;

		case 0xE6: // abs,XY
			address += address_ABSXY();
			storeMemory(address, Index_Registers[REGISTER_Y]);
			set_flag_zn(Memory[address]);
			clearFlag(FLAG_C);
			break;

		case 0xE7: // pag
			address += address_PAG();
			storeMemory(address, Index_Registers[REGISTER_Y]);
			set_flag_zn(Memory[address]);
			clearFlag(FLAG_C);
			break;

		/**
		 * 
		 * TAY - Transters Accumulator to register Y
		 * 
		 **/

		case 0x10: // impl
		Index_Registers[REGISTER_Y] = Registers[REGISTER_A];
		set_flag_n(Index_Registers[REGISTER_Y]);
		break;

		/**
		 * 
		 * TYA - Transters Register Y to Accumulator
		 * 
		 **/

		case 0x20: // impl
			Registers[REGISTER_A] = Index_Registers[REGISTER_Y];
			set_flag_zn(Registers[REGISTER_A]);
			break;

		/**
		 * 
		 * DEY - 
		 * 
		 **/

		case 0x62: //impl
			--Index_Registers[REGISTER_Y];
			set_flag_z((BYTE)Index_Registers[REGISTER_Y]);
			break;

		/**
		 * 
		 * INY - 
		 * 
		 **/

		case 0x72: //impl
			++Index_Registers[REGISTER_Y];
			set_flag_z((BYTE)Index_Registers[REGISTER_Y]);
			break;

		/**
		 * 
		 * STOS - 
		 * 
		 **/

		case 0xFB: // abs
			address += address_ABS();
			storeMemory(address, StackPointer);
			set_flag_zn(Memory[address]);
			clearFlag(FLAG_C);
			break;

		case 0xFC: // abs,X
			address += address_ABSX();
			storeMemory(address, StackPointer);
			set_flag_zn(Memory[address]);
			clearFlag(FLAG_C);
			break;

		case 0xFD: // abs,Y
			address += address_ABSY();
			storeMemory(address, StackPointer);
			set_flag_zn(Memory[address]);
			clearFlag(FLAG_C);
			break;

		case 0xFE: // abs,XY
			address += address_ABSXY();
			storeMemory(address, StackPointer);
			set_flag_zn(Memory[address]);
			clearFlag(FLAG_C);
			break;

		case 0xFF: // pag
			address += address_PAG();
			storeMemory(address, StackPointer);
			set_flag_zn(Memory[address]);
			clearFlag(FLAG_C);
			break;

		/**
		 * 
		 * TSA - 
		 * 
		 **/

		case 0x30: // impl
			Registers[REGISTER_A] = Flags;
			break;

		/**
		 * 
		 * ADD - 
		 * 
		 **/

		case 0x33: // #
			param1 = Registers[REGISTER_A];
			param2 = fetch();
			temp_word = (WORD)param1 + (WORD)param2;
			set_flag_c(temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			set_flag_zn(Registers[REGISTER_A]);
			set_flag_v(param1, param2, Registers[REGISTER_A]);
			break;

		case 0x34: // abs
			param1 = Registers[REGISTER_A];
			address = address_ABS();
			loadW(&param2, address);
			temp_word = (WORD)param1 + (WORD)param2;
			set_flag_c(temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			set_flag_zn(Registers[REGISTER_A]);
			set_flag_v(param1, param2, Registers[REGISTER_A]);
			break;

		case 0x35: // abs,X
			param1 = Registers[REGISTER_A];
			address = address_ABSX();
			loadW(&param2, address);
			temp_word = (WORD)param1 + (WORD)param2;
			set_flag_c(temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			set_flag_zn(Registers[REGISTER_A]);
			set_flag_v(param1, param2, Registers[REGISTER_A]);
			break;

		case 0x36: // abs,Y
			param1 = Registers[REGISTER_A];
			address = address_ABSY();
			loadW(&param2, address);
			temp_word = (WORD)param1 + (WORD)param2;
			set_flag_c(temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			set_flag_zn(Registers[REGISTER_A]);
			set_flag_v(param1, param2, Registers[REGISTER_A]);
			break;

		case 0x37: // abs,XY
			param1 = Registers[REGISTER_A];
			address = address_ABSXY();
			loadW(&param2, address);
			temp_word = (WORD)param1 + (WORD)param2;
			set_flag_c(temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			set_flag_zn(Registers[REGISTER_A]);
			set_flag_v(param1, param2, Registers[REGISTER_A]);
			break;

		case 0x38: // pag
			param1 = Registers[REGISTER_A];
			address = address_PAG();
			loadW(&param2, address);
			temp_word = (WORD)param1 + (WORD)param2;
			set_flag_c(temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			set_flag_zn(Registers[REGISTER_A]);
			set_flag_v(param1, param2, Registers[REGISTER_A]);
			break;

		/**
		 * 
		 * SUB - 
		 * 
		 **/

		case 0x43: // #
			param1 = Registers[REGISTER_A];
			param2 = fetch();
			temp_word = (WORD)param1 - (WORD)param2;
			set_flag_c(temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			set_flag_zn(Registers[REGISTER_A]);
			set_flag_v(param1, param2, Registers[REGISTER_A]);
			break;

		case 0x44: // abs
			param1 = Registers[REGISTER_A];
			address = address_ABS();
			loadW(&param2, address);
			temp_word = (WORD)param1 - (WORD)param2;
			set_flag_c(temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			set_flag_zn(Registers[REGISTER_A]);
			set_flag_v(param1, param2, Registers[REGISTER_A]);
			break;

		case 0x45: // abs,X
			param1 = Registers[REGISTER_A];
			address = address_ABSX();
			loadW(&param2, address);
			temp_word = (WORD)param1 - (WORD)param2;
			set_flag_c(temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			set_flag_zn(Registers[REGISTER_A]);
			set_flag_v(param1, param2, Registers[REGISTER_A]);
			break;

		case 0x46: // abs,Y
			param1 = Registers[REGISTER_A];
			address = address_ABSY();
			loadW(&param2, address);
			temp_word = (WORD)param1 - (WORD)param2;
			set_flag_c(temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			set_flag_zn(Registers[REGISTER_A]);
			set_flag_v(param1, param2, Registers[REGISTER_A]);
			break;

		case 0x47: // abs,XY
			param1 = Registers[REGISTER_A];
			address = address_ABSXY();
			loadW(&param2, address);
			temp_word = (WORD)param1 - (WORD)param2;
			set_flag_c(temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			set_flag_zn(Registers[REGISTER_A]);
			set_flag_v(param1, param2, Registers[REGISTER_A]);
			break;

		case 0x48: // pag
			param1 = Registers[REGISTER_A];
			address = address_PAG();
			loadW(&param2, address);
			temp_word = (WORD)param1 - (WORD)param2;
			set_flag_c(temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			set_flag_zn(Registers[REGISTER_A]);
			set_flag_v(param1, param2, Registers[REGISTER_A]);
			break;

		/**
		 * 
		 * LDP - 
		 * 
		 **/

		case 0xC1: // #
			data = fetch();
			Index_Registers[REGISTER_P] = data;
			break;

		case 0xC2: // abs
			address += address_ABS();
			load(&Index_Registers[REGISTER_P], address);
			break;

		case 0xC3: // abs,X
			address += address_ABSX();
			load(&Index_Registers[REGISTER_P], address);
			break;

		case 0xC4: // abs,Y
			address += address_ABSY();
			load(&Index_Registers[REGISTER_P], address);
			break;

		case 0xC5: // abs,XY
			address += address_ABSXY();
			load(&Index_Registers[REGISTER_P], address);
			break;

		case 0xC6: // pag
			address += address_PAG();
			load(&Index_Registers[REGISTER_P], address);
			break;

		/**
		 * 
		 * STP - 
		 * 
		 **/

		case 0xF3: // abs
			address += address_ABS();
			storeMemory(address, Index_Registers[REGISTER_P]);
			break;

		case 0xF4: // abs,X
			address += address_ABSX();
			storeMemory(address, Index_Registers[REGISTER_P]);
			break;

		case 0xF5: // abs,Y
			address += address_ABSY();
			storeMemory(address, Index_Registers[REGISTER_P]);
			break;

		case 0xF6: // abs,XY
			address += address_ABSXY();
			storeMemory(address, Index_Registers[REGISTER_P]);
			break;

		case 0xF7: // pag
			address += address_PAG();
			storeMemory(address, Index_Registers[REGISTER_P]);
			break;

		/**
		 * 
		 * TAP - 
		 * 
		 **/

		case 0x40: // impl
			Registers[REGISTER_P] = Registers[REGISTER_A];
			break;

		/**
		 * 
		 * TPA - 
		 * 
		 **/

		case 0x50: // impl
			Registers[REGISTER_A] = Registers[REGISTER_P];
			break;

		/**
		 * 
		 * DEP - 
		 * 
		 **/

		case 0x82: // impl
			--Index_Registers[REGISTER_P];
			set_flag_z((BYTE)Index_Registers[REGISTER_P]);
			break;

		/**
		 * 
		 * INP - 
		 * 
		 **/

		case 0x92: // impl
			++Index_Registers[REGISTER_P];
			set_flag_z((BYTE)Index_Registers[REGISTER_P]);
			break;

		/**
		 * 
		 * LDX - 
		 * 
		 **/

		case 0xA1: // #
			data = fetch();
			Index_Registers[REGISTER_X] = data;
			set_flag_zn(Index_Registers[REGISTER_X]);
			clearFlag(FLAG_C);
			break;

		case 0xA2: // abs
			address += address_ABS();
			load(&Index_Registers[REGISTER_X], address);
			set_flag_zn(Index_Registers[REGISTER_X]);
			clearFlag(FLAG_C);
			break;

		case 0xA3: // abs,X
			address += address_ABSX();
			load(&Index_Registers[REGISTER_X], address);
			set_flag_zn(Index_Registers[REGISTER_X]);
			clearFlag(FLAG_C);
			break;

		case 0xA4: // abs,Y
			address += address_ABSY();
			load(&Index_Registers[REGISTER_X], address);
			set_flag_zn(Index_Registers[REGISTER_X]);
			clearFlag(FLAG_C);
			break;

		case 0xA5: // abs,XY
			address += address_ABSXY();
			load(&Index_Registers[REGISTER_X], address);
			set_flag_zn(Index_Registers[REGISTER_X]);
			clearFlag(FLAG_C);
			break;

		case 0xA6: // pag
			address += address_PAG();
			load(&Index_Registers[REGISTER_X], address);
			set_flag_zn(Index_Registers[REGISTER_X]);
			clearFlag(FLAG_C);
			break;

		/**
		 * 
		 * STX - 
		 * 
		 **/

		case 0xD3: // abs
			address += address_ABS();
			storeMemory(address, Index_Registers[REGISTER_X]);
			set_flag_zn(Memory[address]);
			clearFlag(FLAG_C);
			break;

		case 0xD4: // abs,X
			address += address_ABSX();
			storeMemory(address, Index_Registers[REGISTER_X]);
			set_flag_zn(Memory[address]);
			clearFlag(FLAG_C);
			break;

		case 0xD5: // abs,Y
			address += address_ABSY();
			storeMemory(address, Index_Registers[REGISTER_X]);
			set_flag_zn(Memory[address]);
			clearFlag(FLAG_C);
			break;

		case 0xD6: // abs,XY
			address += address_ABSXY();
			storeMemory(address, Index_Registers[REGISTER_X]);
			set_flag_zn(Memory[address]);
			clearFlag(FLAG_C);
			break;

		case 0xD7: // pag
			address += address_PAG();
			storeMemory(address, Index_Registers[REGISTER_X]);
			set_flag_zn(Memory[address]);
			clearFlag(FLAG_C);
			break;

		/**
		 * 
		 * DECX - 
		 * 
		 **/

		case 0x42: // impl
			--Index_Registers[REGISTER_X];
			set_flag_z(Index_Registers[REGISTER_X]);
			break;

		/**
		 * 
		 * INCX - 
		 * 
		 **/

		case 0x52: // impl
			++Index_Registers[REGISTER_X];
			set_flag_z(Index_Registers[REGISTER_X]);
			break;

		/**
		 * 
		 * LODS - 
		 * 
		 **/

		case 0xE8: // #
			data = fetch();
			StackPointer = data;
			StackPointer += (WORD)fetch() << 8;
			set_flag_zn(StackPointer);
			clearFlag(FLAG_C);
			break;

		case 0xE9: // abs
			address += address_ABS();
			if (addressable(address))
			{
				StackPointer = Memory[address];
				StackPointer += (WORD)Memory[address + 1] << 8;				
			}
			set_flag_zn(StackPointer);
			clearFlag(FLAG_C);
			break;

		case 0xEA: // abs,X
			address += address_ABSX();
			if (addressable(address))
			{
				StackPointer = Memory[address];
				StackPointer += (WORD)Memory[address + 1] << 8;
			}
			set_flag_zn(StackPointer);
			clearFlag(FLAG_C);
			break;

		case 0xEB: // abs,Y
			address += address_ABSY();
			if (addressable(address))
			{
				StackPointer = Memory[address];
				StackPointer += (WORD)Memory[address + 1] << 8;				
			}
			set_flag_zn(StackPointer);
			clearFlag(FLAG_C);
			break;

		case 0xEC: // abs,XY
			address += address_ABSXY();
			if (addressable(address))
			{
				StackPointer = Memory[address];
				StackPointer += (WORD)Memory[address + 1] << 8;
			}
			set_flag_zn(StackPointer);
			clearFlag(FLAG_C);
			break;

		case 0xED: // pag
			address += address_PAG();
			if (addressable(address))
			{
				StackPointer = Memory[address];
				StackPointer += (WORD)Memory[address + 1] << 8;
			}
			set_flag_zn(StackPointer);
			clearFlag(FLAG_C);
			break;

		/**
		 * 
		 * ADC - 
		 * 
		 **/

		case 0x13: // #
			param1 = Registers[REGISTER_A];
			param2 = fetch();
			temp_word = (WORD)param1 + (WORD)param2;
			if (checkFlag(FLAG_C))
			{
				temp_word++;
			}
			set_flag_c(temp_word);
			set_flag_zn((BYTE)temp_word);
			set_flag_v(param1, param2, (BYTE)temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			break;

		case 0x14: // abs
			param1 = Registers[REGISTER_A];
			address += address_ABS();
			loadW(&param2, address);
			temp_word = (WORD)param1 + (WORD)param2;
			if (checkFlag(FLAG_C))
			{
				temp_word++;
			}
			set_flag_c(temp_word);
			set_flag_zn((BYTE)temp_word);
			set_flag_v(param1, param2, (BYTE)temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			break;

		case 0x15: // abs,X
			param1 = Registers[REGISTER_A];
			address += address_ABSX();
			loadW(&param2, address);
			temp_word = (WORD)param1 + (WORD)param2;
			if (checkFlag(FLAG_C))
			{
				temp_word++;
			}
			set_flag_c(temp_word);
			set_flag_zn((BYTE)temp_word);
			set_flag_v(param1, param2, (BYTE)temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			break;

		case 0x16: // abs,Y
			param1 = Registers[REGISTER_A];
			address += address_ABSY();
			loadW(&param2, address);
			temp_word = (WORD)param1 + (WORD)param2;
			if (checkFlag(FLAG_C))
			{
				temp_word++;
			}
			set_flag_c(temp_word);
			set_flag_zn((BYTE)temp_word);
			set_flag_v(param1, param2, (BYTE)temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			break;

		case 0x17: // abs,XY
			param1 = Registers[REGISTER_A];
			address += address_ABSXY();
			loadW(&param2, address);
			temp_word = (WORD)param1 + (WORD)param2;
			if (checkFlag(FLAG_C))
			{
				temp_word++;
			}
			set_flag_c(temp_word);
			set_flag_zn((BYTE)temp_word);
			set_flag_v(param1, param2, (BYTE)temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			break;

		case 0x18: // pag
			param1 = Registers[REGISTER_A];
			address += address_PAG();
			loadW(&param2, address);
			temp_word = (WORD)param1 + (WORD)param2;
			if (checkFlag(FLAG_C))
			{
				temp_word++;
			}
			set_flag_c(temp_word);
			set_flag_zn((BYTE)temp_word);
			set_flag_v(param1, param2, (BYTE)temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			break;

		/**
		 * 
		 * SBC - 
		 * 
		 **/

		case 0x23: // #
			param1 = Registers[REGISTER_A];
			param2 = fetch();
			temp_word = (WORD)param1 - (WORD)param2;
			if (checkFlag(FLAG_C)) {
				temp_word--;
			}
			set_flag_c(temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			set_flag_zn((BYTE)temp_word);
			set_flag_v(param1, -param2, (BYTE)temp_word);
			break;

		case 0x24: // abs
			param1 = Registers[REGISTER_A];
			address = address_ABS();
			param2 = Memory[address];
			temp_word = (WORD)param1 - (WORD)param2;
			if (checkFlag(FLAG_C))
			{
				temp_word--;
			}
			set_flag_c(temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			set_flag_zn((BYTE)temp_word);
			set_flag_v(param1, -param2, (BYTE)temp_word);
			break;

		case 0x25: // abs,X
			param1 = Registers[REGISTER_A];
			address = address_ABSX();
			param2 = Memory[address];
			temp_word = (WORD)param1 - (WORD)param2;
			if (checkFlag(FLAG_C))
			{
				temp_word--;
			}
			set_flag_c(temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			set_flag_zn((BYTE)temp_word);
			set_flag_v(param1, -param2, (BYTE)temp_word);
			break;

		case 0x26: // abs,Y
			param1 = Registers[REGISTER_A];
			address = address_ABSY();
			param2 = Memory[address];
			temp_word = (WORD)param1 - (WORD)param2;
			if (checkFlag(FLAG_C))
			{
				temp_word--;
			}
			set_flag_c(temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			set_flag_zn((BYTE)temp_word);
			set_flag_v(param1, -param2, (BYTE)temp_word);
			break;

		case 0x27: // abs,XY
			param1 = Registers[REGISTER_A];
			address = address_ABSXY();
			param2 = Memory[address];
			temp_word = (WORD)param1 - (WORD)param2;
			if (checkFlag(FLAG_C))
			{
				temp_word--;
			}
			set_flag_c(temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			set_flag_zn((BYTE)temp_word);
			set_flag_v(param1, -param2, (BYTE)temp_word);
			break;

		case 0x28: // pag
			param1 = Registers[REGISTER_A];
			address = address_PAG();
			param2 = Memory[address];
			temp_word = (WORD)param1 - (WORD)param2;
			if (checkFlag(FLAG_C))
			{
				temp_word--;
			}
			set_flag_c(temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			set_flag_zn((BYTE)temp_word);
			set_flag_v(param1, -param2, (BYTE)temp_word);
			break;

		/**
		 * 
		 * CMP - 
		 * 
		 **/

		case 0x53: // #
			param1 = Registers[REGISTER_A];
			param2 = fetch();
			temp_word = (WORD)param1 - (WORD)param2;
			if (checkFlag(FLAG_C))
			{
				temp_word--;
			}
			set_flag_c(temp_word);
			set_flag_zn((BYTE)temp_word);
			set_flag_v(param1, -param2, (BYTE)temp_word);
			break;

		case 0x54: // abs
			param1 = Registers[REGISTER_A];
			address += address_ABS();
			loadW(&param2, address);
			temp_word = (WORD)param1 - (WORD)param2;
			if (checkFlag(FLAG_C))
			{
				temp_word--;
			}
			set_flag_c(temp_word);
			set_flag_zn((BYTE)temp_word);
			set_flag_v(param1, -param2, (BYTE)temp_word);
			break;

		case 0x55: // abs,X
			param1 = Registers[REGISTER_A];
			address += address_ABSX();
			loadW(&param2, address);
			temp_word = (WORD)param1 - (WORD)param2;
			if (checkFlag(FLAG_C))
			{
				temp_word--;
			}
			set_flag_c(temp_word);
			set_flag_zn((BYTE)temp_word);
			set_flag_v(param1, -param2, (BYTE)temp_word);
			break;

		case 0x56: // abs,Y
			param1 = Registers[REGISTER_A];
			address += address_ABSY();
			loadW(&param2, address);
			temp_word = (WORD)param1 - (WORD)param2;
			if (checkFlag(FLAG_C))
			{
				temp_word--;
			}
			set_flag_c(temp_word);
			set_flag_zn((BYTE)temp_word);
			set_flag_v(param1, -param2, (BYTE)temp_word);
			break;

		case 0x57: // abs,XY
			param1 = Registers[REGISTER_A];
			address += address_ABSXY();
			loadW(&param2, address);
			temp_word = (WORD)param1 - (WORD)param2;
			if (checkFlag(FLAG_C))
			{
				temp_word--;
			}
			set_flag_c(temp_word);
			set_flag_zn((BYTE)temp_word);
			set_flag_v(param1, -param2, (BYTE)temp_word);
			break;

		case 0x58: // pag
			param1 = Registers[REGISTER_A];
			address += address_PAG();
			loadW(&param2, address);
			temp_word = (WORD)param1 - (WORD)param2;
			if (checkFlag(FLAG_C))
			{
				temp_word--;
			}
			set_flag_c(temp_word);
			set_flag_zn((BYTE)temp_word);
			set_flag_v(param1, -param2, (BYTE)temp_word);
			break;

		/**
		 * 
		 * OR - 
		 * 
		 **/

		case 0x63: // #
			data = fetch();
			temp_word = (WORD)Registers[REGISTER_A] | data;
			set_flag_zn((BYTE)temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			break;

		case 0x64: // abs
			address = address_ABS();
			loadW(&param2, address);
			temp_word = (WORD)Registers[REGISTER_A] | param2;
			set_flag_zn((BYTE)temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			break;

		case 0x65: // abs,X
			address = address_ABSX();
			loadW(&param2, address);
			temp_word = (WORD)Registers[REGISTER_A] | param2;
			set_flag_zn((BYTE)temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			break;

		case 0x66: // abs,Y
			address = address_ABSY();
			loadW(&param2, address);
			temp_word = (WORD)Registers[REGISTER_A] | param2;
			set_flag_zn((BYTE)temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			break;

		case 0x67: // abs,XY
			address = address_ABSXY();
			loadW(&param2, address);
			temp_word = (WORD)Registers[REGISTER_A] | param2;
			set_flag_zn((BYTE)temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			break;

		case 0x68: // pag
			address += address_PAG();
			loadW(&param2, address);
			temp_word = (WORD)Registers[REGISTER_A] | param2;
			set_flag_zn((BYTE)temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			break;

		/**
		 * 
		 * EOR - 
		 * 
		 **/

		case 0x83: // #
			data = fetch();
			temp_word = (WORD)Registers[REGISTER_A] ^ data;
			set_flag_zn((BYTE)temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			break;

		case 0x84: // abs
			address = address_ABS();
			loadW(&param2, address);
			temp_word = (WORD)Registers[REGISTER_A] ^ param2;
			set_flag_zn((BYTE)temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			break;

		case 0x85: // abs,X
			address = address_ABSX();
			loadW(&param2, address);
			temp_word = (WORD)Registers[REGISTER_A] ^ param2;
			set_flag_zn((BYTE)temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			break;

		case 0x86: // abs,Y
			address = address_ABSY();
			loadW(&param2, address);
			temp_word = (WORD)Registers[REGISTER_A] ^ param2;
			set_flag_zn((BYTE)temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			break;

		case 0x87: // abs,XY
			address = address_ABSXY();
			loadW(&param2, address);
			temp_word = (WORD)Registers[REGISTER_A] ^ param2;
			set_flag_zn((BYTE)temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			break;

		case 0x88: // pag
			address += address_PAG();
			loadW(&param2, address);
			temp_word = (WORD)Registers[REGISTER_A] ^ param2;
			set_flag_zn((BYTE)temp_word);
			Registers[REGISTER_A] = (BYTE)temp_word;
			break;

		/**
		 * 
		 * PSHA - 
		 * 
		 **/

		case 0x1E: // impl
			if ((StackPointer >= 1) && (StackPointer < MEMORY_SIZE))
			{
				Memory[StackPointer] = Registers[REGISTER_A];
				StackPointer--;
			}
			break;


		/**
		 * 
		 * PSHs - 
		 * 
		 **/

		case 0x1F: // impl
			if ((StackPointer >= 1) && (StackPointer < MEMORY_SIZE))
			{
				Memory[StackPointer] = Flags;
				StackPointer--;
			}
			break;

		/**
		 * 
		 * POPA - 
		 * 
		 **/

		case 0x2E: // impl
			if ((StackPointer >= 0) && (StackPointer < MEMORY_SIZE - 1))
			{
				StackPointer++;
				Registers[REGISTER_A] = Memory[StackPointer];
			}
			break;

		/**
		 * 
		 * POPs - 
		 * 
		 **/

		case 0x2F: // impl
			if ((StackPointer >= 0) && (StackPointer < MEMORY_SIZE - 1))
			{
				StackPointer++;
				Flags = Memory[StackPointer];
			}
			break;

		/**
		 * 
		 * JP - 
		 * 
		 **/

		case 0x70: // abs
			address = address_ABS();
			ProgramCounter = address;
			break;

		/**
		 * 
		 * JMPR - 
		 * 
		 **/

		case 0x3F: // abs
			address = address_ABS();
			if ((StackPointer >= 2) && (StackPointer < MEMORY_SIZE))
			{
				Memory[StackPointer] = (BYTE)((ProgramCounter >> 8) & 0xFF);
				StackPointer--;
				Memory[StackPointer] = (BYTE)(ProgramCounter & 0xFF);
				StackPointer--;
			}
			ProgramCounter = address;
			break;

		/**
		 * 
		 * JCC - 
		 * 
		 **/

		case 0x80: // abs
			address = address_ABS();
			if (!checkFlag(FLAG_C))
			{
				if ((StackPointer >= 2) && (StackPointer < MEMORY_SIZE))
				{
					Memory[StackPointer] = (BYTE)((ProgramCounter >> 8) & 0xFF);
					StackPointer--;
					Memory[StackPointer] = (BYTE)(ProgramCounter & 0xFF);
					StackPointer--;
				}
				ProgramCounter = address;
			}
			break;

		/**
		 * 
		 * JCS - 
		 * 
		 **/

		case 0x90: // abs
			address = address_ABS();
			if (checkFlag(FLAG_C))
			{
				ppushs();
				ProgramCounter = address;
			}
			break;

		/**
		 * 
		 * JNE - 
		 * 
		 **/
		case 0xA0: // abs
			address = address_ABS();
			if (!checkFlag(FLAG_Z))
			{
				ppushs();
				ProgramCounter = address;
			}
			break;

		/**
		 * 
		 * JEQ - 
		 * 
		 **/		
		
		case 0xB0: // abs
			address = address_ABS();
			if (checkFlag(FLAG_Z))
			{
				ppushs();
				ProgramCounter = address;
			}
			break;

		/**
		 * 
		 * JVC - 
		 * 
		 **/	

		case 0xC0: // abs
			address = address_ABS();
			if (!checkFlag(FLAG_V))
			{
				ppushs();
				ProgramCounter = address;
			}
			break;

		/**
		 * 
		 * JVS - 
		 * 
		 **/

		case 0xD0: // abs
			address = address_ABS();
			if (checkFlag(FLAG_V))
			{
				ppushs();
				ProgramCounter = address;
			}
			break;

		/**
		 * 
		 * JMI - 
		 * 
		 **/

		case 0xE0: // abs
			address = address_ABS();
			if (checkFlag(FLAG_N))
			{
				ppushs();
				ProgramCounter = address;
			}
			break;

		/**
		 * 
		 * JPL - 
		 * 
		 **/

		case 0xF0: // abs
			address = address_ABS();
			if (!checkFlag(FLAG_N))
			{
				ppushs();
				ProgramCounter = address;
			}
			break;

		/**
		 * 
		 * CCC - 
		 * 
		 **/
		case 0x4F: // abs
			address = address_ABS();
			if (!checkFlag(FLAG_C))
			{
				ppushs();
				ProgramCounter = address;
			}
			break;

		/**
		 * 
		 * CCS - 
		 * 
		 **/
		case 0x5F: //abs
			address = address_ABS();
			if (checkFlag(FLAG_C))
			{
				ppushs();
				ProgramCounter = address;
			}
			break;

		/**
		 * 
		 * CNE - 
		 * 
		 **/

		case 0x6F: // abs
			address = address_ABS();
			if (!checkFlag(FLAG_Z))
			{
				ppushs();
				ProgramCounter = address;
			}
			break;

		/**
		 * 
		 * CEQ - 
		 * 
		 **/

		case 0x7F: // abs
			address = address_ABS();
			if (checkFlag(FLAG_Z))
			{
				ppushs();
				ProgramCounter = address;
			}
			break;

		/**
		 * 
		 * CVC - 
		 * 
		 **/
		case 0x8F: // abs
			address = address_ABS();
			if (!checkFlag(FLAG_V))
			{
				ppushs();
				ProgramCounter = address;
			}
			break;

		/**
		 * 
		 * CVS - 
		 * 
		 **/
		case 0x9F: //abs
			address = address_ABS();
			if (checkFlag(FLAG_V))
			{
				ppushs();
				ProgramCounter = address;
			}
			break;

		/**
		 * 
		 * CMI - 
		 * 
		 **/
		case 0xAF: // abs
			address = address_ABS();
			if (checkFlag(FLAG_N))
			{
				ppushs();
				ProgramCounter = address;
			}
			break;

		/**
		 * 
		 * CPL - 
		 * 
		 **/
		case 0xBF: // abs
			address = address_ABS();
			if (!checkFlag(FLAG_N))
			{
				ppushs();
				ProgramCounter = address;
			}
			break;

		/**
		 * 
		 * CHI - 
		 * 
		 **/
		case 0xCF: // abs
			if ((checkFlag(FLAG_Z)) || (checkFlag(FLAG_C)))
			{
				address = address_ABS();
				if (addressable(address))
				{
					ppushs();
					ProgramCounter = address;
				}
			}
			break;

		/**
		 * 
		 * CLE - 
		 * 
		 **/
		case 0xDF: // abs
			if ((!checkFlag(FLAG_Z)) && (!checkFlag(FLAG_C)))
			{
				address = address_ABS();
				if (addressable(address))
				{
					ppushs();
					ProgramCounter = address;
				}
			}
			break;

		/**
		 * 
		 * CLC - 
		 * 
		 **/
		case 0x01:
			clearFlag(FLAG_C);
			break;

		/**
		 * 
		 * STC - 
		 * 
		 **/
		case 0x11:
			setFlag(FLAG_C);
			break;

		/**
		 * 
		 * CLI - 
		 * 
		 **/
		case 0x21:
			clearFlag(FLAG_I);
			break;

		/**
		 * 
		 * STI - 
		 * 
		 **/
		case 0x31:
			setFlag(FLAG_I);
			break;

		/**
		 * 
		 * SEV - 
		 * 
		 **/
		case 0x41:
			setFlag(FLAG_V);
			break;

		/**
		 * 
		 * CLV - 
		 * 
		 **/
		case 0x51:
			clearFlag(FLAG_V);
			break;

		/**
		 * 
		 * CMC - 
		 * 
		 **/
		case 0x61:
			Flags ^= FLAG_C;
			break;

		/**
		 * 
		 * CMV - 
		 * 
		 **/
		case 0x71:
			Flags ^= FLAG_V;
			break;

		/**
		 * 
		 * NOP - 
		 * 
		 **/
		case 0xE1:
			break;

		/**
		 * 
		 * HALT - 
		 * 
		 **/
		case 0xF1:
			halt = true;
			break;

		/**
		 * 
		 * SWI - 
		 * 
		 **/
		case 0x81:
	
				if (StackPointer >= 1 && StackPointer < MEMORY_SIZE)
				{
					Memory[StackPointer] = Registers[REGISTER_A];
					StackPointer--;

					Memory[StackPointer] = Flags;
					StackPointer--;
				}
				setFlag(FLAG_I);
				break;

		/**
		 * 
		 * RTI - 
		 * 
		 **/
		case 0x91:
			if (StackPointer >= 0 && StackPointer < MEMORY_SIZE - 1)
			{
				StackPointer++;
				Flags = (BYTE)Memory[StackPointer] << 8;

				StackPointer++;
				Registers[REGISTER_A] = Memory[StackPointer];
			}

			break;

			/**
			 * 
			 * RTS - 
			 * 
			 **/

			case 0xA7: // impl
				if ((StackPointer >= 0) && (StackPointer < MEMORY_SIZE - 2))
				{
					StackPointer++;
					LB = Memory[StackPointer];
					StackPointer++;
					HB = Memory[StackPointer];
					ProgramCounter = ((WORD)HB << 8) + (WORD)LB;
				}
				break;

			/**
			 * 
			 * SRA - 
			 * 
			 **/
			case 0x4E:
				if ((ProgramCounter >= 0) && (ProgramCounter <= MEMORY_SIZE))
				{
					memory_in_range = true;
					ProgramCounter = ProgramCounter + 2;
				}

				break;

			/**
			 * 
			 * SCC - 
			 * 
			 **/
			case 0x5E:
				if (!checkFlag(FLAG_C))
				{
					if ((ProgramCounter >= 0) && (ProgramCounter <= MEMORY_SIZE))
					{
						memory_in_range = true;
						ProgramCounter = ProgramCounter + 2;
					}
				}
				break;

			/**
			 * 
			 * SCS - 
			 * 
			 **/
			case 0x6E:
				if (checkFlag(FLAG_C))
				{
					if ((ProgramCounter >= 0) && (ProgramCounter <= MEMORY_SIZE))
					{
						memory_in_range = true;
						ProgramCounter = ProgramCounter + 2;
					}
				}
				break;

			/**
			 * 
			 * SNE - 
			 * 
			 **/
			case 0x7E:
				if (!checkFlag(FLAG_Z))
				{
					if ((ProgramCounter >= 0) && (ProgramCounter <= MEMORY_SIZE))
					{
						memory_in_range = true;
						ProgramCounter = ProgramCounter + 2;
					}
				}
				break;

			/**
			 * 
			 * SEQ - 
			 * 
			 **/
			case 0x8E:
				if (checkFlag(FLAG_Z))
				{
					if ((ProgramCounter >= 0) && (ProgramCounter <= MEMORY_SIZE))
					{
						memory_in_range = true;
						ProgramCounter = ProgramCounter + 2;
					}
				}
				break;

			/**
			 * 
			 * SVC - 
			 * 
			 **/			
			case 0x9E:
				if (!checkFlag(FLAG_V))
				{
					if ((ProgramCounter >= 0) && (ProgramCounter <= MEMORY_SIZE))
					{
						memory_in_range = true;
						ProgramCounter = ProgramCounter + 2;
					}
				}
				break;

			/**
			 * 
			 * SVS - 
			 * 
			 **/			
			case 0xAE:
				if (checkFlag(FLAG_V))
				{
					if ((ProgramCounter >= 0) && (ProgramCounter <= MEMORY_SIZE))
					{
						memory_in_range = true;
						ProgramCounter = ProgramCounter + 2;
					}
				}
				break;

			/**
			 * 
			 * SMI - 
			 * 
			 **/
			case 0xBE:

				if (checkFlag(FLAG_N))
				{
					if ((ProgramCounter >= 0) && (ProgramCounter <= MEMORY_SIZE))
					{
						memory_in_range = true;
						ProgramCounter = ProgramCounter + 2;
					}
				}
				break;

			/**
			 * 
			 * SPL - 
			 * 
			 **/
			case 0xCE:
				if (!checkFlag(FLAG_N))
				{
					if ((ProgramCounter >= 0) && (ProgramCounter <= MEMORY_SIZE))
					{
						memory_in_range = true;
						ProgramCounter = ProgramCounter + 2;
					}
				}

				break;

			/**
			 * 
			 * SGE - 
			 * 
			 **/
			case 0xDE:
				if (!((!checkFlag(FLAG_N)) ^ (!checkFlag(FLAG_V))))
				{
					if ((ProgramCounter >= 0) && (ProgramCounter <= MEMORY_SIZE))
					{
						memory_in_range = true;
						ProgramCounter = ProgramCounter + 2;
					}
				}
				break;

			/**
			 * 
			 * SGT - 
			 * 
			 **/
			case 0xEE:
				if ((checkFlag(FLAG_Z)) || (checkFlag(FLAG_N)) ^ (checkFlag(FLAG_V)))
				{
					if ((ProgramCounter >= 0) && (ProgramCounter <= MEMORY_SIZE))
					{
						memory_in_range = true;
						ProgramCounter = ProgramCounter + 2;
					}
				}
				break;

			/**
			 * 
			 * INC - 
			 * 
			 **/
			case 0x29: // abs
				address = address_ABS();
				if (addressable(address))
				{
					Memory[address]++;
				}
				set_flag_zn(Memory[address]);
				break;

			case 0x2A: // abs,X
				address = address_ABSX();
				if (addressable(address))
				{
					Memory[address]++;
				}
				set_flag_zn(Memory[address]);
				break;

			case 0x2B: // abs,Y
				address = address_ABSY();
				if (addressable(address))
				{
					Memory[address]++;
				}
				set_flag_zn(Memory[address]);
				break;

			case 0x2C: // abs,XY
				address = address_ABSXY();
				if (addressable(address))
				{
					Memory[address]++;
				}
				set_flag_zn(Memory[address]);
				break;

			/**
			 * 
			 * DEC - 
			 * 
			 **/	
			case 0x39: // abs
				address = address_ABS();
				if (addressable(address))
				{
					Memory[address]--;
				}
				set_flag_zn(Memory[address]);
				break;

			case 0x3A: // abs,X
				address = address_ABSX();
				if (addressable(address))
				{
					Memory[address]--;
				}
				set_flag_zn(Memory[address]);
				break;

			case 0x3B: // abs,Y
				address = address_ABSY();
				if (addressable(address))
				{
					Memory[address]--;
				}
				set_flag_zn(Memory[address]);
				break;

			case 0x3C: // abs,XY
				address = address_ABSXY();
				if (addressable(address))
				{
					Memory[address]--;
				}
				set_flag_zn(Memory[address]);
				break;

			/**
			 * 
			 * INCA - 
			 * 
			 **/
			case 0x2D: // A
				++Registers[REGISTER_A];
				set_flag_zn(Registers[REGISTER_A]);
				break;
				
			/**
			 * 
			 * DECA - 
			 * 
			 **/
			case 0x3D: // A
				--Registers[REGISTER_A];
				set_flag_zn(Registers[REGISTER_A]);
				break;

			/**
			 * 
			 * RCR - 
			 * 
			 **/			
			case 0x49: // abs
				address = address_ABS();
				saved_flags = Flags;
				if (addressable(address))
				{
					if ((Memory[address] & 0x01) == 0x01)
					{
						setFlag(FLAG_C);
					}
					else
					{
						clearFlag(FLAG_C);
					}
					Memory[address] = (Memory[address] >> 1) & 0x7F;
					if ((saved_flags & FLAG_C) == FLAG_C)
					{
						Memory[address] = Memory[address] | 0x80;
					}
					set_flag_zn(Memory[address]);
				}
				break;

			case 0x4A: // abs,X
				address = address_ABSX();
				saved_flags = Flags;
				if (addressable(address))
				{
					if ((Memory[address] & 0x01) == 0x01)
					{
						setFlag(FLAG_C);
					}
					else
					{
						clearFlag(FLAG_C);
					}
					Memory[address] = (Memory[address] >> 1) & 0x7F;
					if ((saved_flags & FLAG_C) == FLAG_C)
					{
						Memory[address] = Memory[address] | 0x80;
					}
					set_flag_zn(Memory[address]);
				}
				break;

			case 0x4B: // abs,Y
				address = address_ABSY();
				saved_flags = Flags;
				if (addressable(address))
				{
					if ((Memory[address] & 0x01) == 0x01)
					{
						setFlag(FLAG_C);
					}
					else
					{
						clearFlag(FLAG_C);
					}
					Memory[address] = (Memory[address] >> 1) & 0x7F;
					if ((saved_flags & FLAG_C) == FLAG_C)
					{
						Memory[address] = Memory[address] | 0x80;
					}
					set_flag_zn(Memory[address]);
				}
				break;

			case 0x4C: // abs,XY
				address = address_ABSXY();
				saved_flags = Flags;
				if (addressable(address))
				{
					if ((Memory[address] & 0x01) == 0x01)
					{
						setFlag(FLAG_C);
					}
					else
					{
						clearFlag(FLAG_C);
					}
					Memory[address] = (Memory[address] >> 1) & 0x7F;
					if ((saved_flags & FLAG_C) == FLAG_C)
					{
						Memory[address] = Memory[address] | 0x80;
					}
					set_flag_zn(Memory[address]);
				}
				break;

			/**
			 * 
			 * RCRA - 
			 * 
			 **/
			case 0x4D: // A
				saved_flags = Flags;
				if ((Registers[REGISTER_A] & 0x01) == 0x01)
				{
					setFlag(FLAG_C);
				}
				else
				{
					clearFlag(FLAG_C);
				}
				Registers[REGISTER_A] = (Registers[REGISTER_A] >> 1) & 0x7F;
				if (saved_flags & FLAG_C == FLAG_C)
				{
					Registers[REGISTER_A] = Registers[REGISTER_A] | 0x80;
				}
				set_flag_zn(Registers[REGISTER_A]);
				break;

			/**
			 * 
			 * RCL - 
			 * 
			 **/
			case 0x59: // abs
				address = address_ABS();
				saved_flags = Flags;
				if (addressable(address))
				{
					if ((Memory[address] & 0x80) == 0x80)
					{
						setFlag(FLAG_C);
					}
					else
					{
						clearFlag(FLAG_C);
					}
					Memory[address] = (Memory[address] << 1) & 0xFE;
					if ((saved_flags & FLAG_C) == FLAG_C)
					{
						Memory[address] = Memory[address] | 0x01;
					}
					set_flag_zn(Memory[address]);
				}
				break;

			case 0x5A: // abs,X
				address = address_ABSX();
				saved_flags = Flags;
				if (addressable(address))
				{
					if ((Memory[address] & 0x80) == 0x80)
					{
						setFlag(FLAG_C);
					}
					else
					{
						clearFlag(FLAG_C);
					}
					Memory[address] = (Memory[address] << 1) & 0xFE;
					if ((saved_flags & FLAG_C) == FLAG_C)
					{
						Memory[address] = Memory[address] | 0x01;
					}
					set_flag_zn(Memory[address]);
				}
				break;

			case 0x5B: // abs,Y
				address = address_ABSY();
				saved_flags = Flags;
				if (addressable(address))
				{
					if ((Memory[address] & 0x80) == 0x80)
					{
						setFlag(FLAG_C);
					}
					else
					{
						clearFlag(FLAG_C);
					}
					Memory[address] = (Memory[address] << 1) & 0xFE;
					if ((saved_flags & FLAG_C) == FLAG_C)
					{
						Memory[address] = Memory[address] | 0x01;
					}
					set_flag_zn(Memory[address]);
				}
				break;

			case 0x5C: // abs,XY
				address = address_ABSXY();
				saved_flags = Flags;
				if (addressable(address))
				{
					if ((Memory[address] & 0x80) == 0x80)
					{
						setFlag(FLAG_C);
					}
					else
					{
						clearFlag(FLAG_C);
					}
					Memory[address] = (Memory[address] << 1) & 0xFE;
					if ((saved_flags & FLAG_C) == FLAG_C)
					{
						Memory[address] = Memory[address] | 0x01;
					}
					set_flag_zn(Memory[address]);
				}
				break;

			/**
			 * 
			 * RCLA - 
			 * 
			 **/
			case 0x5D: // A
				saved_flags = Flags;
				if ((Registers[REGISTER_A] & 0x80) == 0x80)
				{
					setFlag(FLAG_C);
				}
				else
				{
					clearFlag(FLAG_C);
				}
				Registers[REGISTER_A] = (Registers[REGISTER_A] << 1) & 0xFE;
				if (saved_flags & FLAG_C == FLAG_C)
				{
					Registers[REGISTER_A] = Registers[REGISTER_A] | 0x01;
				}
				set_flag_zn(Registers[REGISTER_A]);
				break;

			/**
			 * 
			 * SHL - 
			 * 
			 **/
			case 0x69: // abs
				address = address_ABS();
				if (addressable(address))
				{
					temp_word = Memory[address];
					if ((temp_word & 0x80) != 0)
					{
						setFlag(FLAG_C);
					}
					else
					{
						clearFlag(FLAG_C);
					}
					temp_word = temp_word << 1;
					temp_word = temp_word & 0xFE;
					set_flag_zn((BYTE)temp_word);
					Memory[address] = (BYTE)temp_word;
				}
				break;

			case 0x6A: // abs,X
				address = address_ABSX();
				if (addressable(address))
				{
					temp_word = Memory[address];
					if ((temp_word & 0x80) != 0)
					{
						setFlag(FLAG_C);
					}
					else
					{
						clearFlag(FLAG_C);
					}
					temp_word = temp_word << 1;
					temp_word = temp_word & 0xFE;
					set_flag_zn((BYTE)temp_word);
					Memory[address] = (BYTE)temp_word;
				}
				break;

			case 0x6B: // abs,Y
				address = address_ABSY();
				if (addressable(address))
				{
					temp_word = Memory[address];
					if ((temp_word & 0x80) != 0)
					{
						setFlag(FLAG_C);
					}
					else
					{
						clearFlag(FLAG_C);
					}
					temp_word = temp_word << 1;
					temp_word = temp_word & 0xFE;
					set_flag_zn((BYTE)temp_word);
					Memory[address] = (BYTE)temp_word;
				}
				break;

			case 0x6C: // abs,XY
				address = address_ABSXY();
				if (addressable(address))
				{
					temp_word = Memory[address];
					if ((temp_word & 0x80) != 0)
					{
						setFlag(FLAG_C);
					}
					else
					{
						clearFlag(FLAG_C);
					}
					temp_word = temp_word << 1;
					temp_word = temp_word & 0xFE;
					set_flag_zn((BYTE)temp_word);
					Memory[address] = (BYTE)temp_word;
				}
				break;

			/**
			 * 
			 * SHLA - 
			 * 
			 **/
			case 0x6D: // A
				temp_word = Registers[REGISTER_A];
				if ((temp_word & 0x80) != 0)
				{
					setFlag(FLAG_C);
				}
				else
				{
					clearFlag(FLAG_C);
				}
				temp_word <<= 1;
				temp_word &= 0xFE;
				Registers[REGISTER_A] = (BYTE)temp_word;
				set_flag_zn(Registers[REGISTER_A]);
				break;

			// SAR
			case 0x79: // abs
				address = address_ABS();
				temp_word = Memory[address];
				if ((Memory[address] & 0x01) == 0x01)
				{
					setFlag(FLAG_C);
				}
				else
				{
					clearFlag(FLAG_C);
				}
				Memory[address] = Memory[address] >> 1;
				if ((temp_word & 0x80) == 0x80)
				{
					Memory[address] |= 1 << 7;
				}
				set_flag_zn(Memory[address]);
				break;

			case 0x7A: // abs,X
				address = address_ABSX();
				temp_word = Memory[address];
				if ((Memory[address] & 0x01) == 0x01)
				{
					setFlag(FLAG_C);
				}
				else
				{
					clearFlag(FLAG_C);
				}
				Memory[address] = Memory[address] >> 1;
				if ((temp_word & 0x80) == 0x80)
				{
					Memory[address] |= 1 << 7;
				}
				set_flag_zn(Memory[address]);
				break;

			case 0x7B: // abs,Y
				address = address_ABSY();
				temp_word = Memory[address];
				if ((Memory[address] & 0x01) == 0x01)
				{
					setFlag(FLAG_C);
				}
				else
				{
					clearFlag(FLAG_C);
				}
				Memory[address] = Memory[address] >> 1;
				if ((temp_word & 0x80) == 0x80)
				{
					Memory[address] |= 1 << 7;
				}
				set_flag_zn(Memory[address]);
				break;

			case 0x7C: // abs,XY
				address = address_ABSXY();
				temp_word = Memory[address];
				if ((Memory[address] & 0x01) == 0x01)
				{
					setFlag(FLAG_C);
				}
				else
				{
					clearFlag(FLAG_C);
				}
				Memory[address] = Memory[address] >> 1;
				if ((temp_word & 0x80) == 0x80)
				{
					Memory[address] |= 1 << 7;
				}
				set_flag_zn(Memory[address]);
				break;

			/**
			 * 
			 * SARA - 
			 * 
			 **/
			case 0x7D: // SARA
				if ((Registers[REGISTER_A] & 0x01) == 0x01)
				{
					setFlag(FLAG_C);
				}
				else
				{
					clearFlag(FLAG_C);
				}
				Registers[REGISTER_A] = (Registers[REGISTER_A] >> 1) & 0x7F;
				if (checkFlag(FLAG_N))
				{
					Registers[REGISTER_A] = (Registers[REGISTER_A] | 0x80);
				}
				set_flag_zn(Registers[REGISTER_A]);
				break;

			/**
			 * 
			 * LSR - 
			 * 
			 **/
			case 0x89: // abs
				address = address_ABS();
				if (addressable(address))
				{
					temp_word = Memory[address];
					if ((temp_word & 0x01) == 0x01)
					{
						setFlag(FLAG_C);
					}
					else
					{
						clearFlag(FLAG_C);
					}
					temp_word = (temp_word >> 1) & 0x7F;
					set_flag_zn((BYTE)temp_word);
					Memory[address] = temp_word;
				}
				break;

			case 0x8A: // abs,X
				address = address_ABSX();
				if (addressable(address))
				{
					temp_word = Memory[address];
					if ((temp_word & 0x01) == 0x01)
					{
						setFlag(FLAG_C);
					}
					else
					{
						clearFlag(FLAG_C);
					}
					temp_word = (temp_word >> 1) & 0x7F;
					set_flag_zn((BYTE)temp_word);
					Memory[address] = temp_word;
				}
				break;

			case 0x8B: // abs,Y
				address = address_ABSY();
				if (addressable(address))
				{
					temp_word = Memory[address];
					if ((temp_word & 0x01) == 0x01)
					{
						setFlag(FLAG_C);
					}
					else
					{
						clearFlag(FLAG_C);
					}
					temp_word = (temp_word >> 1) & 0x7F;
					set_flag_zn((BYTE)temp_word);
					Memory[address] = temp_word;
				}
				break;

			case 0x8C: // abs,XY
				address = address_ABSXY();
				if (addressable(address))
				{
					temp_word = Memory[address];
					if ((temp_word & 0x01) == 0x01)
					{
						setFlag(FLAG_C);
					}
					else
					{
						clearFlag(FLAG_C);
					}
					temp_word = (temp_word >> 1) & 0x7F;
					set_flag_zn((BYTE)temp_word);
					Memory[address] = temp_word;
				}
				break;

			/**
			 * 
			 * LSRA - 
			 * 
			 **/
			case 0x8D: // A
				temp_word = Registers[REGISTER_A];
				if ((temp_word & 0x01) == 0x01)
				{
					setFlag(FLAG_C);
				}
				else
				{
					clearFlag(FLAG_C);
				}
				temp_word = (temp_word >> 1) & 0x7F;
				set_flag_zn((BYTE)temp_word);
				Registers[REGISTER_A] = (BYTE)temp_word;
				break;

			/**
			 * 
			 * AND - 
			 * 
			 **/
			case 0x73: // #
				param1 = Registers[REGISTER_A];
				param2 = fetch();
				temp_word = (WORD)param1 & (WORD)param2;
				set_flag_n((BYTE)temp_word);
				set_flag_z((BYTE)temp_word);
				// set_flag_v(param1, param2, (BYTE)temp_word);
				clearFlag(FLAG_V);
				Registers[REGISTER_A] = (BYTE)temp_word;
				break;

			case 0x74: // abs
				param1 = Registers[REGISTER_A];
				address = address_ABS();
				if(addressable(address)) {
					param2 = Memory[address];
				}
				temp_word = (WORD)param1 & (WORD)param2;
				set_flag_n((BYTE)temp_word);
				set_flag_z((BYTE)temp_word);
				set_flag_v(param1, param2, (BYTE)temp_word);
				Registers[REGISTER_A] = (BYTE)temp_word;
				break;

			case 0x75: // abs,X
				param1 = Registers[REGISTER_A];
				address = address_ABSX();
				if (addressable(address))
				{
					param2 = Memory[address];
				}
				temp_word = (WORD)param1 & (WORD)param2;
				set_flag_n((BYTE)temp_word);
				set_flag_z((BYTE)temp_word);
				set_flag_v(param1, param2, (BYTE)temp_word);
				Registers[REGISTER_A] = (BYTE)temp_word;
				break;

			case 0x76: // abs,Y
				param1 = Registers[REGISTER_A];
				address = address_ABSY();
				if (addressable(address))
				{
					param2 = Memory[address];
				}
				temp_word = (WORD)param1 & (WORD)param2;
				set_flag_n((BYTE)temp_word);
				set_flag_z((BYTE)temp_word);
				set_flag_v(param1, param2, (BYTE)temp_word);
				Registers[REGISTER_A] = (BYTE)temp_word;
				break;

			case 0x77: // abs,XY
				param1 = Registers[REGISTER_A];
				address = address_ABSXY();
				if(addressable(address)) {
					param2 = Memory[address];
				}
				temp_word = (WORD)param1 & (WORD)param2;
				set_flag_n((BYTE)temp_word);
				set_flag_z((BYTE)temp_word);
				set_flag_v(param1, param2, (BYTE)temp_word);
				Registers[REGISTER_A] = (BYTE)temp_word;
				break;

			case 0x78: // pag
				param1 = Registers[REGISTER_A];
				address = address_PAG();
				if (addressable(address))
				{
					param2 = Memory[address];
				}
				temp_word = (WORD)param1 & (WORD)param2;
				set_flag_n((BYTE)temp_word);
				set_flag_z((BYTE)temp_word);
				set_flag_v(param1, param2, (BYTE)temp_word);
				Registers[REGISTER_A] = (BYTE)temp_word;
				break;

			/**
			 * 
			 * BT - 
			 * 
			 **/
			case 0x93: // #
				param1 = Registers[REGISTER_A];
				param2 = fetch();
				temp_word = (WORD)param1 & (WORD)param2;
				set_flag_zn((BYTE)temp_word);
				clearFlag(FLAG_V);
				break;

			case 0x94: // abs
				param1 = Registers[REGISTER_A];
				address = address_ABS();
				if (addressable(address))
				{
					param2 = Memory[address];
				}
				temp_word = (WORD)param1 & (WORD)param2;
				set_flag_zn((BYTE)temp_word);
				clearFlag(FLAG_V);
				break;

			case 0x95: // abs,X
				param1 = Registers[REGISTER_A];
				address = address_ABSX();
				if (addressable(address))
				{
					param2 = Memory[address];
				}
				temp_word = (WORD)param1 & (WORD)param2;
				set_flag_zn((BYTE)temp_word);
				clearFlag(FLAG_V);
				break;

			case 0x96: // abs,Y
				param1 = Registers[REGISTER_A];
				address = address_ABSY();
				if (addressable(address))
				{
					param2 = Memory[address];
				}
				temp_word = (WORD)param1 & (WORD)param2;
				set_flag_zn((BYTE)temp_word);
				clearFlag(FLAG_V);
				break;

			case 0x97: // abs,XY
				param1 = Registers[REGISTER_A];
				address = address_ABSXY();
				if (addressable(address))
				{
					param2 = Memory[address];
				}
				temp_word = (WORD)param1 & (WORD)param2;
				set_flag_zn((BYTE)temp_word);
				clearFlag(FLAG_V);
				break;

			case 0x98: // pag
				param1 = Registers[REGISTER_A];
				address = address_PAG();
				if (addressable(address))
				{
					param2 = Memory[address];
				}
				temp_word = (WORD)param1 & (WORD)param2;
				set_flag_zn((BYTE)temp_word);
				clearFlag(FLAG_V);
				break;

			// NOT
			case 0x99: // abs
				address = address_ABS();
				storeMemory(address, ~Memory[address]);
				set_flag_zn(Memory[address]);
				set_flag_c((WORD)Memory[address]);
				break;

			case 0x9A: // abs,X
				address = address_ABSX();
				storeMemory(address, ~Memory[address]);
				set_flag_zn(Memory[address]);
				set_flag_c((WORD)Memory[address]);
				break;

			case 0x9B: // abs,Y
				address = address_ABSY();
				storeMemory(address, ~Memory[address]);
				set_flag_zn(Memory[address]);
				set_flag_c((WORD)Memory[address]);
				break;

			case 0x9C: // abs,XY
				address = address_ABSXY();
				storeMemory(address, ~Memory[address]);
				set_flag_zn(Memory[address]);
				set_flag_c((WORD)Memory[address]);
				break;

			/**
			 * 
			 * NOTA - 
			 * 
			 **/
			case 0x9D:
				Registers[REGISTER_A] = ~Registers[REGISTER_A];
				set_flag_zn(Registers[REGISTER_A]);
				set_flag_c((WORD)Memory[address]);
				break;

			/**
			 * 
			 * NEG - 
			 * 
			 **/
			case 0xA9: // abs
				address = address_ABS();
				temp_word = 0 - Memory[address];
				set_flag_zn((BYTE)temp_word);
				storeMemory(address, (BYTE)temp_word);
				break;

			case 0xAA: // abs,X
				address = address_ABSX();
				temp_word = 0 - Memory[address];
				set_flag_zn((BYTE)temp_word);
				storeMemory(address, (BYTE)temp_word);
				break;

			case 0xAB: // abs,Y
				address = address_ABSY();
				temp_word = 0 - Memory[address];
				set_flag_zn((BYTE)temp_word);
				storeMemory(address, (BYTE)temp_word);
				break;

			case 0xAC: // abs,XY
				address = address_ABSXY();
				temp_word = 0 - Memory[address];
				set_flag_zn((BYTE)temp_word);
				storeMemory(address, (BYTE)temp_word);
				break;

			/**
			 * 
			 * NEGA - 
			 * 
			 **/
			case 0xAD: // A
				temp_word = 0 - Registers[REGISTER_A];
				Registers[REGISTER_A] = (BYTE)temp_word;
				set_flag_zn(Registers[REGISTER_A]);
				break;

			/**
			 * 
			 * RAL - 
			 * 
			 **/
			case 0xB9: // abs
				address = address_ABS();
				temp_word = Memory[address];
				if (addressable(address))
				{
					if ((temp_word & 0x80) == 0x80)
					{
						temp_word = (temp_word << 1) + 0x01;
					}
					else
					{
						temp_word = (temp_word << 1);
					}
					set_flag_zn((BYTE)temp_word);
					storeMemory(address, (BYTE)temp_word);
				}
				break;

			case 0xBA: // abs,X
				address = address_ABSX();
				temp_word = Memory[address];
				if (addressable(address))
				{
					if ((temp_word & 0x80) == 0x80)
					{
						temp_word = (temp_word << 1) + 0x01;
					}
					else
					{
						temp_word = (temp_word << 1);
					}
					set_flag_zn((BYTE)temp_word);
					storeMemory(address, (BYTE)temp_word);
				}
				break;

			case 0xBB: // abs,Y
				address = address_ABSY();
				temp_word = Memory[address];
				if (addressable(address))
				{
					if ((temp_word & 0x80) == 0x80)
					{
						temp_word = (temp_word << 1) + 0x01;
					}
					else
					{
						temp_word = (temp_word << 1);
					}
					set_flag_zn((BYTE)temp_word);
					storeMemory(address, (BYTE)temp_word);
				}
				break;

			case 0xBC: // abs,XY
				address = address_ABSXY();
				temp_word = Memory[address];
				if (addressable(address))
				{
					if ((temp_word & 0x80) == 0x80)
					{
						temp_word = (temp_word << 1) + 0x01;
					}
					else
					{
						temp_word = (temp_word << 1);
					}
					set_flag_zn((BYTE)temp_word);
					storeMemory(address, (BYTE)temp_word);
				}
				break;

			/**
			 * 
			 * RALA - 
			 * 
			 **/
			case 0xBD: // A
				temp_word = Registers[REGISTER_A];
				if ((temp_word & 0x80) == 0x80)
				{
					temp_word = (temp_word << 1) + 0x01;
				}
				else
				{
					temp_word = (temp_word << 1);
				}
				set_flag_zn((BYTE)temp_word);
				Registers[REGISTER_A] = (BYTE)temp_word;
				break;

			/**
			 * 
			 * ROR - 
			 * 
			 **/
			case 0xC9: // abs
				address = address_ABS();
				temp_word = Memory[address];
				if (addressable(address))
				{
					if ((temp_word & 0x01) == 0x01)
					{
						temp_word = (temp_word >> 1) + 0x80;
					}
					else
					{
						temp_word = (temp_word >> 1);
					}
					set_flag_zn((BYTE)temp_word);
					storeMemory(address, (BYTE)temp_word);
				}
				break;

			case 0xCA: // abs,X
				address = address_ABSX();
				temp_word = Memory[address];
				if (addressable(address))
				{
					if ((temp_word & 0x01) == 0x01)
					{
						temp_word = (temp_word >> 1) + 0x80;
					}
					else
					{
						temp_word = (temp_word >> 1);
					}
					set_flag_zn((BYTE)temp_word);
					storeMemory(address, (BYTE)temp_word);
				}
				break;

			case 0xCB: // abs,Y
				address = address_ABSY();
				temp_word = Memory[address];
				if (addressable(address))
				{
					if ((temp_word & 0x01) == 0x01)
					{
						temp_word = (temp_word >> 1) + 0x80;
					}
					else
					{
						temp_word = (temp_word >> 1);
					}
					set_flag_zn((BYTE)temp_word);
					storeMemory(address, (BYTE)temp_word);
				}
				break;

			case 0xCC: // abs,XY
				address = address_ABSXY();
				temp_word = Memory[address];
				if (addressable(address))
				{
					if ((temp_word & 0x01) == 0x01)
					{
						temp_word = (temp_word >> 1) + 0x80;
					}
					else
					{
						temp_word = (temp_word >> 1);
					}
					set_flag_zn((BYTE)temp_word);
					storeMemory(address, (BYTE)temp_word);
				}
				break;

			/**
			 * 
			 * RORA - 
			 * 
			 **/
			case 0xCD: // abs,XY
				temp_word = Registers[REGISTER_A];
				if ((temp_word & 0x01) == 0x01)
				{
					temp_word = (temp_word >> 1) + 0x80;
				}
				else
				{
					temp_word = (temp_word >> 1);
				}
				set_flag_zn((BYTE)temp_word);
				Registers[REGISTER_A] = (BYTE)temp_word;
				break;

			/**
			 * 
			 * CLR - 
			 * 
			 **/
			case 0xD9: // abs
				address = address_ABS();
				Memory[address] = 0;
				clearFlag(FLAG_N);
				setFlag(FLAG_Z);
				break;

			case 0xDA: // abs,X
				address = address_ABSX();
				Memory[address] = 0;
				clearFlag(FLAG_N);
				setFlag(FLAG_Z);
				break;

			case 0xDB: // abs,Y
				address = address_ABSY();
				Memory[address] = 0;
				clearFlag(FLAG_N);
				setFlag(FLAG_Z);
				break;

			case 0xDC: // abs,XY
				address = address_ABSXY();
				Memory[address] = 0;
				clearFlag(FLAG_N);
				setFlag(FLAG_Z);
				break;

			/**
			 * 
			 * CLRA - 
			 * 
			 **/
			case 0xDD: // abs,XY
				Registers[REGISTER_A] = 0;
				clearFlag(FLAG_N);
				setFlag(FLAG_Z);
				break;

			/**
			 * 
			 * TST - 
			 * 
			 **/
			case 0x19: // abs
				address = address_ABS();
				TST(address);
				break;

			case 0x1A: // abs,X
				address = address_ABSX();
				TST(address);
				break;

			case 0x1B: // abs,Y
				address = address_ABSY();
				TST(address);
				break;

			case 0x1C: // abs,XY
				address = address_ABSXY();
				TST(address);
				break;

			/**
			 * 
			 * TSTA - 
			 * 
			 **/
			case 0x1D: // A
				Registers[REGISTER_A] -= 0x00;
				set_flag_zn(Registers[REGISTER_A]);
				break;
			}
}

void execute(BYTE opcode)
{	

	if(opcode <= 0xFF){
		Group_1(opcode);
	}
	else
	{
		printf("ERROR> Unrecognised Op-code %X\n", opcode);
	}
}

void emulate()
{
	BYTE opcode;

	ProgramCounter = 0;
	halt = false;
	memory_in_range = true;
	int sanity = 0;
	printf("                    A  X  Y  P  SP\n");

	while ((!halt) && (memory_in_range) && sanity <= 100)
	{
		printf("%04X ", ProgramCounter);           // Print current address
		opcode = fetch();
		execute(opcode);

		printf("%s  ", opcode_mneumonics[opcode]);  // Print current opcode

		printf("%02X ", Registers[REGISTER_A]);
		printf("%02X ", Index_Registers[REGISTER_X]);
		printf("%02X ", Index_Registers[REGISTER_Y]);
		printf("%02X ", Index_Registers[REGISTER_P]);
		printf("%04X ", StackPointer);              // Print Stack Pointer

		if ((Flags & FLAG_V) == FLAG_V)	
		{
			printf("V=1 ");
		}
		else
		{
			printf("V=0 ");
		}
		if ((Flags & FLAG_Z) == FLAG_Z)	
		{
			printf("Z=1 ");
		}
		else
		{
			printf("Z=0 ");
		}
		if ((Flags & FLAG_N) == FLAG_N)	
		{
			printf("N=1 ");
		}
		else
		{
			printf("N=0 ");
		}
		if ((Flags & FLAG_I) == FLAG_I)	
		{
			printf("I=1 ");
		}
		else
		{
			printf("I=0 ");
		}
		if ((Flags & FLAG_C) == FLAG_C)	
		{
			printf("C=1 ");
		}
		else
		{
			printf("C=0 ");
		}
		printf("\n"); // New line
		sanity++;
	}

	printf("\n");  // New line
}


////////////////////////////////////////////////////////////////////////////////
//                            Simulator/Emulator (End)                        //
////////////////////////////////////////////////////////////////////////////////


void initialise_filenames() {
	int i;

	for (i=0; i<MAX_FILENAME_SIZE; i++) {
		hex_file [i] = '\0';
		trc_file [i] = '\0';
	}
}




int find_dot_position(char *filename) {
	int  dot_position;
	int  i;
	char chr;

	dot_position = 0;
	i = 0;
	chr = filename[i];

	while (chr != '\0') {
		if (chr == '.') {
			dot_position = i;
		}
		i++;
		chr = filename[i];
	}

	return (dot_position);
}


int find_end_position(char *filename) {
	int  end_position;
	int  i;
	char chr;

	end_position = 0;
	i = 0;
	chr = filename[i];

	while (chr != '\0') {
		end_position = i;
		i++;
		chr = filename[i];
	}

	return (end_position);
}


bool file_exists(char *filename) {
	bool exists;
	FILE *ifp;

	exists = false;

	if ( ( ifp = fopen( filename, "r" ) ) != NULL ) {
		exists = true;

		fclose(ifp);
	}

	return (exists);
}



void create_file(char *filename) {
	FILE *ofp;

	if ( ( ofp = fopen( filename, "w" ) ) != NULL ) {
		fclose(ofp);
	}
}



bool getline(FILE *fp, char *buffer) {
	bool rc;
	bool collect;
	char c;
	int  i;

	rc = false;
	collect = true;

	i = 0;
	while (collect) {
		c = getc(fp);

		switch (c) {
		case EOF:
			if (i > 0) {
				rc = true;
			}
			collect = false;
			break;

		case '\n':
			if (i > 0) {
				rc = true;
				collect = false;
				buffer[i] = '\0';
			}
			break;

		default:
			buffer[i] = c;
			i++;
			break;
		}
	}

	return (rc);
}






void load_and_run(int args,_TCHAR** argv) {
	char chr;
	int  ln;
	int  dot_position;
	int  end_position;
	long i;
	FILE *ifp;
	long address;
	long load_at;
	int  code;

	// Prompt for the .hex file

	printf("\n");
	printf("Enter the hex filename (.hex): ");

	if(args == 2){
		ln = 0;
		chr = argv[1][ln];
		while (chr != '\0')
		{
			if (ln < MAX_FILENAME_SIZE)
			{
				hex_file [ln] = chr;
				trc_file [ln] = chr;
				ln++;
			}
			chr = argv[1][ln];
		}
	} else {
		ln = 0;
		chr = '\0';
		while (chr != '\n') {
			chr = getchar();

			switch(chr) {
			case '\n':
				break;
			default:
				if (ln < MAX_FILENAME_SIZE)	{
					hex_file [ln] = chr;
					trc_file [ln] = chr;
					ln++;
				}
				break;
			}
		}

	}
	// Tidy up the file names

	dot_position = find_dot_position(hex_file);
	if (dot_position == 0) {
		end_position = find_end_position(hex_file);

		hex_file[end_position + 1] = '.';
		hex_file[end_position + 2] = 'h';
		hex_file[end_position + 3] = 'e';
		hex_file[end_position + 4] = 'x';
		hex_file[end_position + 5] = '\0';
	} else {
		hex_file[dot_position + 0] = '.';
		hex_file[dot_position + 1] = 'h';
		hex_file[dot_position + 2] = 'e';
		hex_file[dot_position + 3] = 'x';
		hex_file[dot_position + 4] = '\0';
	}

	dot_position = find_dot_position(trc_file);
	if (dot_position == 0) {
		end_position = find_end_position(trc_file);

		trc_file[end_position + 1] = '.';
		trc_file[end_position + 2] = 't';
		trc_file[end_position + 3] = 'r';
		trc_file[end_position + 4] = 'c';
		trc_file[end_position + 5] = '\0';
	} else {
		trc_file[dot_position + 0] = '.';
		trc_file[dot_position + 1] = 't';
		trc_file[dot_position + 2] = 'r';
		trc_file[dot_position + 3] = 'c';
		trc_file[dot_position + 4] = '\0';
	}

	if (file_exists(hex_file)) {
		// Clear Registers and Memory

		Registers[REGISTER_A] = 0;
		Index_Registers[REGISTER_X] = 0;
		Index_Registers[REGISTER_Y] = 0;
		Index_Registers[REGISTER_P] = 0;
		Flags = 0;
		ProgramCounter = 0;
		StackPointer = 0;

		for (i=0; i<MEMORY_SIZE; i++) {
			Memory[i] = 0x00;
		}

		// Load hex file

		if ( ( ifp = fopen( hex_file, "r" ) ) != NULL ) {
			printf("Loading file...\n\n");

			load_at = 0;

			while (getline(ifp, InputBuffer)) {
				if (sscanf(InputBuffer, "L=%x", &address) == 1) {
					load_at = address;
				} else if (sscanf(InputBuffer, "%x", &code) == 1) {
					if ((load_at >= 0) && (load_at <= MEMORY_SIZE)) {
						Memory[load_at] = (BYTE)code;
					}
					load_at++;
				} else {
					printf("ERROR> Failed to load instruction: %s \n", InputBuffer);
				}
			}

			fclose(ifp);
		}

		// Emulate

		emulate();
	} else {
		printf("\n");
		printf("ERROR> Input file %s does not exist!\n", hex_file);
		printf("\n");
	}
}

void building(int args,_TCHAR** argv){
	char buffer[1024];
	load_and_run(args,argv);
	sprintf(buffer, "0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X", 
		Memory[TEST_ADDRESS_1],
		Memory[TEST_ADDRESS_2],
		Memory[TEST_ADDRESS_3],
		Memory[TEST_ADDRESS_4], 
		Memory[TEST_ADDRESS_5],
		Memory[TEST_ADDRESS_6], 
		Memory[TEST_ADDRESS_7],
		Memory[TEST_ADDRESS_8], 
		Memory[TEST_ADDRESS_9], 
		Memory[TEST_ADDRESS_10],
		Memory[TEST_ADDRESS_11],
		Memory[TEST_ADDRESS_12]
		);
	sendto(sock, buffer, strlen(buffer), 0, (SOCKADDR *)&server_addr, sizeof(SOCKADDR));
}



void test_and_mark() {
	char buffer[1024];
	bool testing_complete;
	int  len = sizeof(SOCKADDR);
	char chr;
	int  i;
	int  j;
	bool end_of_program;
	long address;
	long load_at;
	int  code;
	int  mark;
	int  passed;

	printf("\n");
	printf("Automatic Testing and Marking\n");
	printf("\n");

	testing_complete = false;

	sprintf(buffer, "Test Student %s", STUDENT_NUMBER);
	sendto(sock, buffer, strlen(buffer), 0, (SOCKADDR *)&server_addr, sizeof(SOCKADDR));

	while (!testing_complete) {
		memset(buffer, '\0', sizeof(buffer));

		if (recvfrom(sock, buffer, sizeof(buffer)-1, 0, (SOCKADDR *)&client_addr, &len) != SOCKET_ERROR) {
			printf("Incoming Data: %s \n", buffer);

			//if (strcmp(buffer, "Testing complete") == 1)
			if (sscanf(buffer, "Testing complete %d", &mark) == 1) {
				testing_complete = true;
				printf("Current mark = %d\n", mark);

			}else if (sscanf(buffer, "Tests passed %d", &passed) == 1) {
				//testing_complete = true;
				printf("Passed = %d\n", passed);

			} else if (strcmp(buffer, "Error") == 0) {
				printf("ERROR> Testing abnormally terminated\n");
				testing_complete = true;
			} else {
				// Clear Registers and Memory

		Registers[REGISTER_A] = 0;
		Index_Registers[REGISTER_X] = 0;
		Index_Registers[REGISTER_Y] = 0;
		Index_Registers[REGISTER_P] = 0;
				Flags = 0;
				ProgramCounter = 0;
				StackPointer = 0;
				for (i=0; i<MEMORY_SIZE; i++) {
					Memory[i] = 0;
				}

				// Load hex file

				i = 0;
				j = 0;
				load_at = 0;
				end_of_program = false;
				FILE *ofp;
				fopen_s(&ofp ,"branch.txt", "a");

				while (!end_of_program) {
					chr = buffer[i];
					switch (chr) {
					case '\0':
						end_of_program = true;

					case ',':
						if (sscanf(InputBuffer, "L=%x", &address) == 1) {
							load_at = address;
						} else if (sscanf(InputBuffer, "%x", &code) == 1) {
							if ((load_at >= 0) && (load_at <= MEMORY_SIZE)) {
								Memory[load_at] = (BYTE)code;
								fprintf(ofp, "%02X\n", (BYTE)code);
							}
							load_at++;
						} else {
							printf("ERROR> Failed to load instruction: %s \n", InputBuffer);
						}
						j = 0;
						break;

					default:
						InputBuffer[j] = chr;
						j++;
						break;
					}
					i++;
				}
				fclose(ofp);
				// Emulate

				if (load_at > 1) {
					emulate();
					// Send and store results
					sprintf(buffer, "%02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X", 
						Memory[TEST_ADDRESS_1],
						Memory[TEST_ADDRESS_2],
						Memory[TEST_ADDRESS_3],
						Memory[TEST_ADDRESS_4], 
						Memory[TEST_ADDRESS_5],
						Memory[TEST_ADDRESS_6], 
						Memory[TEST_ADDRESS_7],
						Memory[TEST_ADDRESS_8], 
						Memory[TEST_ADDRESS_9], 
						Memory[TEST_ADDRESS_10],
						Memory[TEST_ADDRESS_11],
						Memory[TEST_ADDRESS_12]
						);
					sendto(sock, buffer, strlen(buffer), 0, (SOCKADDR *)&server_addr, sizeof(SOCKADDR));
				}
			}
		}
	}
}



int _tmain(int argc, _TCHAR* argv[])
{
	char chr;
	char dummy;

	printf("\n");
	printf("Microprocessor Emulator\n");
	printf("UWE Computer and Network Systems Assignment 1\n");
	printf("\n");

	initialise_filenames();

	if (WSAStartup(MAKEWORD(2, 2), &data) != 0) return(0);

	sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);  // Here we create our socket, which will be a UDP socket (SOCK_DGRAM).
	if (!sock) {	
		// Creation failed! 
	}

	memset(&server_addr, 0, sizeof(SOCKADDR_IN));
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = inet_addr(IP_ADDRESS_SERVER);
	server_addr.sin_port = htons(PORT_SERVER);

	memset(&client_addr, 0, sizeof(SOCKADDR_IN));
	client_addr.sin_family = AF_INET;
	client_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
	client_addr.sin_port = htons(PORT_CLIENT);

	chr = '\0';
	while ((chr != 'e') && (chr != 'E'))
	{
		printf("\n");
		printf("Please select option\n");
		printf("L - Load and run a hex file\n");
		printf("T - Have the server test and mark your emulator\n");
		printf("E - Exit\n");
		if(argc == 2){ building(argc,argv); exit(0);}
		printf("Enter option: ");
		chr = getchar();
		if (chr != 0x0A)
		{
			dummy = getchar();  // read in the <CR>
		}
		printf("\n");

		switch (chr)
		{
		case 'L':
		case 'l':
			load_and_run(argc,argv);
			break;

		case 'T':
		case 't':
			test_and_mark();
			break;

		default:
			break;
		}
	}

	closesocket(sock);
	WSACleanup();


	return 0;
}



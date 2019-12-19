/*=============================================================================
 * Author: Ignacio Majul <imajul89@gmail.com>
 * Date: 2019/12/10
 *===========================================================================*/

/*==================[inclusions]=============================================*/
#include "24C32_DS3231.h"   // <= own header
#include "sapi.h"

#define eeprom24C32DelayMs   delayInaccurateMs

/*==================[internal data definition]===============================*/
DEBUG_PRINT_ENABLE

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

//-----------------------------------------------------------------------------
// MANAGEMENT
//-----------------------------------------------------------------------------

uint8_t eeprom24C32I2cAddress( Eeprom24C32_t* eeprom )
{
	bool_t a0 = eeprom->A0;
	bool_t a1 = eeprom->A1;
	bool_t a2 = eeprom->A2;

	//                0b1010               A2        A1       A0
	return (EEPROM24C32_CTRL_CODE<<3) | (a2<<2) | (a1<<1) | (a0<<0);
}

uint8_t RTC_Address( void )
{
	//           0b1101000
	return (DS3231_SLAVE_ADDRESS);
}

//-----------------------------------------------------------------------------
// INITIALIZATION
//-----------------------------------------------------------------------------

bool_t eeprom24C32Init( Eeprom24C32_t* eeprom,
		int32_t i2c, bool_t A0, bool_t A1, bool_t A2,
		int32_t pageSize, int32_t memorySize )
{

	bool_t retVal = FALSE;
	uint8_t testByte = 0;

	// I2C port connected to EEPROM, example I2C0
	eeprom->i2c = i2c;
	// Use this if fixed address
	eeprom->A0 = A0;
	eeprom->A1 = A1;
	eeprom->A2 = A2;
	// EEPROM capacity
	eeprom->pageSize = pageSize;   // EEPROM page size [bytes]
	eeprom->memorySize = memorySize; // EEPROM total memory size [bytes]

	return TRUE; //retVal;
}

void RTC_Init( rtcDS3231_t* now)
{
	now->year = (((ANIO / 10) << 4)|(ANIO % 10));
	now->month = (((MES / 10) << 4)|(MES % 10));
	now->mday = (((DIA_MES / 10) << 4)|(DIA_MES % 10));
	now->wday = (((DIA_SEMANA / 10) << 4)|(DIA_SEMANA % 10));
	now->hour = (((HORA / 10) << 4)|(HORA % 10));
	now->min = (((MINUTOS / 10) << 4)|(MINUTOS % 10));
	now->sec= (((SEGUNDOS / 10) << 4)|(SEGUNDOS % 10));
	now->alarm1_seconds = (((ALARMA_SEGUNDOS / 10) << 4)|(ALARMA_SEGUNDOS % 10));	 /* 0 to 59   */
	now->alarm1_minutes = 0b10000000 | (((ALARMA_MINUTOS / 10) << 4)|(ALARMA_MINUTOS % 10));	 /* 0 to 59   */
	now->alarm1_hours = 0b10000000 | (((ALARMA_HORA / 10) << 4)|(ALARMA_HORA % 10));	 /* 1–12 + AM/PM / 00–23 */
	now->alarm1_DY_DT = 0b10000000;	 /* bit 7 = Alarm when hours, minutes, and seconds match. LSB=01 to 7  or 1 to 31 */
	now->alarm2_minutes = 0;	 /* 0 to 59   */
	now->alarm2_hours = 0;	 /* 1–12 + AM/PM / 00–23 */
	now->alarm2_DY_DT = 0;	 /* 01 to 7  or 1 to 31 */
	now->control = 0b00000101;  /* Alarm 1 enable & Interrupt Control enable */
	now->control_status = 0;
	now->aging_offset;
	now->MSB_temp;     	/* temperatura byte superior */
	now->LSB_temp;		/* temperatura byte inferior */
}

//-----------------------------------------------------------------------------
// WRITE OPERATIONS
//-----------------------------------------------------------------------------

// Write time registers
void RTC_write_time( rtcDS3231_t* time, int32_t i2c)
{
	uint8_t dato[17];

	dato[0]=0x00;
	dato[1]=time->sec;
	dato[2]=time->min;
	dato[3]=time->hour;
	dato[4]=time->wday;
	dato[5]=time->mday;
	dato[6]=time->month;
	dato[7]=time->year;
	dato[8]=time->alarm1_seconds;
	dato[9]=time->alarm1_minutes;
	dato[10]=time->alarm1_hours;
	dato[11]=time->alarm1_DY_DT;
	dato[12]=time->alarm2_minutes;
	dato[13]=time->alarm2_hours;
	dato[14]=time->alarm2_DY_DT;
	dato[15]=time->control;
	dato[16]=time->control_status;

	i2cWrite(i2c,RTC_Address(),dato,17,TRUE );
}

// Reset alarm flags
void RTC_reset_alarm( rtcDS3231_t* time, int32_t i2c)
{
	uint8_t dato[2];

	dato[0]=0x0F;
	dato[1]=time->control_status;

	i2cWrite(i2c,RTC_Address(),dato,2,TRUE );
}

// Byte Write
bool_t eeprom24C32WriteByte( Eeprom24C32_t* eeprom,
		uint32_t memoryAddress, uint8_t byteToWrite )
{

	bool_t retVal = TRUE; // True if OK

	// Check memory address
	if( memoryAddress > eeprom->memorySize ) {
		return FALSE;
	}

	uint8_t dataToWrite[3];

	// Memory address High
	dataToWrite[0] = EEPROM_ADDRESS_HIGH( memoryAddress );
	// Memory address Low
	dataToWrite[1] = EEPROM_ADDRESS_LOW( memoryAddress );

	// Byte to write
	dataToWrite[2] = (uint8_t)byteToWrite;

	/* uint8_t i2cNumber, uint8_t  i2cSlaveAddress,
      uint8_t* transmitDataBuffer, uint16_t transmitDataBufferSize,
      bool_t sendWriteStop */
	retVal = i2cWrite( eeprom->i2c,
			eeprom24C32I2cAddress( eeprom ),
			dataToWrite, 3, TRUE );

	eeprom24C32DelayMs(5); // Twc - Write cycle time (byte or page)

	return retVal; // Byte writed
}

bool_t eeprom24C32WriteDate( Eeprom24C32_t* eeprom24C32, uint16_t* eeprom_address, rtcDS3231_t time)
{
	eeprom24C32WriteByte( eeprom24C32, *eeprom_address, time.mday);
	(*eeprom_address)++;
	eeprom24C32WriteByte( eeprom24C32, *eeprom_address, time.month);
	(*eeprom_address)++;
	eeprom24C32WriteByte( eeprom24C32, *eeprom_address, time.year);
	(*eeprom_address)++;
	return TRUE;
}

// Page Write
bool_t eeprom24C32WritePage( Eeprom24C32_t* eeprom, uint32_t page,
		uint8_t* byteBuffer, uint32_t byteBufferSize )
{

	bool_t retVal = TRUE; // True if OK

	// Check valid buffer size
	if( byteBufferSize != eeprom->pageSize ) {
		return FALSE;
	}

	uint16_t i=0;

	uint16_t memoryAddress = page * eeprom->pageSize;

	uint8_t dataToWrite[ byteBufferSize+2 ]; // 2 bytes more for memory address

	// Memory address High
	dataToWrite[0] = EEPROM_ADDRESS_HIGH( memoryAddress );
	// Memory address Low
	dataToWrite[1] = EEPROM_ADDRESS_LOW( memoryAddress );

	// Bytes to write
	for( i=0; i<byteBufferSize; i++ ) {
		dataToWrite[i+2] = byteBuffer[i];
	}

	// uint8_t i2cNumber, uint8_t  i2cSlaveAddress,
	// uint8_t* transmitDataBuffer, uint16_t transmitDataBufferSize,
	// bool_t sendWriteStop
	retVal = i2cWrite( eeprom->i2c,
			eeprom24C32I2cAddress( eeprom),
			dataToWrite, (byteBufferSize+2), TRUE );

	eeprom24C32DelayMs(5); // Twc - Write cycle time (byte or page)

	return retVal; // Byte writed
}

//-----------------------------------------------------------------------------
// READ OPERATIONS
//-----------------------------------------------------------------------------

// Read time registers
rtcDS3231_t RTC_read_time( rtcDS3231_t* now, int32_t i2c)
{
	uint8_t lectura[19];

	i2cRead( i2c, RTC_Address(),(uint8_t*)0, 0, FALSE, lectura, 19, TRUE);

	now->year = lectura[10];
	now->month = lectura[9];
	now->mday = lectura[8];
	now->wday = lectura[7];
	now->hour = lectura[6];
	now->min = lectura[5];
	now->sec = lectura[4];

	//now->alarm1_seconds = (((ALARMA_SEGUNDOS / 10) << 4)|(ALARMA_SEGUNDOS % 10));	 /* 0 to 59   */
	//now->alarm1_minutes = 0b10000000 | (((ALARMA_MINUTOS / 10) << 4)|(ALARMA_MINUTOS % 10));	 /* 0 to 59   */
	//now->alarm1_hours = 0b10000000 | (((ALARMA_HORA / 10) << 4)|(ALARMA_HORA % 10));	 /* 1–12 + AM/PM / 00–23 */
	//now->alarm1_DY_DT = 0b10000000;	 /* bit 7 = Alarm when hours, minutes, and seconds match. LSB=01 to 7  or 1 to 31 */
	//now->alarm2_minutes = 0;	 /* 0 to 59   */
	//now->alarm2_hours = 0;	 /* 1–12 + AM/PM / 00–23 */
	//now->alarm2_DY_DT = 0;	 /* 01 to 7  or 1 to 31 */

	//now->control = 0b00000101;  /* Alarm 1 enable & Interrupt Control enable */
	//now->control_status = 0;
	//now->aging_offset;

	now->MSB_temp = (int8_t)lectura[2];     	  /* temperatura byte superior */
	now->LSB_temp = ((uint8_t)lectura[3]>>6)*25;		/* temperatura byte inferior */

	return *now;
}

// Current Address Read
bool_t eeprom24C32ReadCurrentAddress( Eeprom24C32_t* eeprom,
		uint32_t memoryAddress,
		uint8_t* readedByte )
{
	bool_t retVal = TRUE; // True if OK

	// uint8_t i2cNumber, uint8_t i2cSlaveAddress,
	// uint8_t* dataToReadBuffer, uint16_t dataToReadBufferSize,
	// bool_t sendWriteStop,
	// uint8_t* reciveDataBuffer, uint16_t reciveDataBufferSize,
	// bool_t sendReadStop
	retVal = i2cRead( eeprom->i2c,
			eeprom24C32I2cAddress( eeprom ),
			(uint8_t*)0, 0,
			FALSE,
			readedByte, 1, TRUE );

	return retVal; // read correct
}

// Random Read
bool_t eeprom24C32ReadRandom( Eeprom24C32_t* eeprom,
		uint32_t memoryAddress, uint8_t* readedByte )
{
	bool_t retVal = TRUE; // True if OK

	// Check memory address
	if( memoryAddress > eeprom->memorySize ) {
		return FALSE;
	}

	uint8_t addressToRead[ 2 ]; // 2 bytes for memory address

	// Memory address High
	addressToRead[0] = EEPROM_ADDRESS_HIGH( memoryAddress );
	// Memory address Low
	addressToRead[1] = EEPROM_ADDRESS_LOW( memoryAddress );

	// uint8_t i2cNumber, uint8_t i2cSlaveAddress,
	// uint8_t* dataToReadBuffer, uint16_t dataToReadBufferSize,
	// bool_t sendWriteStop,
	// uint8_t* reciveDataBuffer, uint16_t reciveDataBufferSize,
	// bool_t sendReadStop
	retVal = i2cRead( eeprom->i2c,
			eeprom24C32I2cAddress( eeprom ),
			addressToRead, 2, FALSE,
			readedByte, 1, TRUE );

	//eeprom24C32DelayMs(1); // ??? - Read cycle time (byte or page)

	return retVal; // read correct
}

// Sequential Read
bool_t eeprom24C32ReadSequential( Eeprom24C32_t* eeprom, uint32_t address,
		uint8_t* byteBuffer, uint32_t byteBufferSize )
{
	bool_t retVal = TRUE; // True if OK

	// Check valid buffer size
	if( byteBufferSize > eeprom->memorySize  ) {
		return FALSE;
	}

	// TODO: Check valid address
	//if( address >= eeprom24C32AmountOfPagesGet(eeprom) ) {
	//   return FALSE;
	//}

	uint8_t addressToRead[ 2 ]; // 2 bytes for memory address

	// Memory address High
	addressToRead[0] = EEPROM_ADDRESS_HIGH( address );
	// Memory address Low
	addressToRead[1] = EEPROM_ADDRESS_LOW( address );

	// uint8_t i2cNumber, uint8_t i2cSlaveAddress,
	// uint8_t* dataToReadBuffer, uint16_t dataToReadBufferSize,
	// bool_t sendWriteStop,
	// uint8_t* reciveDataBuffer, uint16_t reciveDataBufferSize,
	// bool_t sendReadStop
	retVal = i2cRead( eeprom->i2c,
			eeprom24C32I2cAddress( eeprom),
			addressToRead, 2, FALSE,
			byteBuffer, byteBufferSize, TRUE );

	return retVal; // Byte writed
}

/*==================[ISR external functions definition]======================*/

/*==================[end of file]============================================*/

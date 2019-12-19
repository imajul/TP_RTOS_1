/*=============================================================================
 * Author: Ignacio Majul <imajul89@gmail.com>
 * Date: 2019/12/10
 *===========================================================================*/

/*=====[Avoid multiple inclusion - begin]====================================*/

#ifndef _24C32_DS3132_H_
#define _24C32_DS3132_H_

/*==================[inclusions]=============================================*/

#include "sapi.h"

/*==================[c++]====================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

// EEPROM24C32 total memory size in bytes
#define EEPROM_32_K_BIT              4096 // Memory size [bytes]

// EEPROM24C32 single page size (in bytes)
#define EEPROM24C32_PAGE_SIZE       32 // [bytes per page]

// EEPROM24C32 & DS3231 I2C address
#define EEPROM24C32_CTRL_CODE       0xA // 0b1010
#define DS3231_SLAVE_ADDRESS   		0x68 // 0b1101000

// Function aliases
#define eeprom24C32ReadByte         eeprom24C32ReadRandom

// Function utilities
#define EEPROM_ADDRESS_HIGH(address)   ( (uint8_t)((address&(0xFFFFFF00))>>8) )
#define EEPROM_ADDRESS_LOW(address)    ( (uint8_t)( address&(0x000000FF)) )

// ----------------------------------------------------------------------------

// EEPROM24C32 total memory size in bytes
// #define EEPROM24C32_MEMORY_SIZE   4 KBytes
#define EEPROM24C32_FIRST_MEMORY_ADDRESS  0x0000
#define EEPROM24C32_LAST_MEMORY_ADDRESS   0x0FFF
#define EEPROM24C32_MEMORY_SIZE           (EEPROM24C32_LAST_MEMORY_ADDRESS  + 1) //

// EEPROM24C32_MEMORY_SIZE / EEPROM24C32_PAGE_SIZE
#define EEPROM24C32_PAGE_AMOUNT           EEPROAM24C32_MEMORY_SIZE / EEPROM24C32_PAGE_SIZE

// SETEO DE LA HORA
#define ANIO 19
#define MES 12
#define DIA_MES 25
#define DIA_SEMANA 03
#define HORA 20
#define MINUTOS 19
#define SEGUNDOS 00

// SETEO DE LA ALARMA
#define ALARMA_HORA 20       // desestimado segun configuracion actual
#define ALARMA_MINUTOS 30	 // desestimado segun configuracion actual
#define ALARMA_SEGUNDOS 05

/*==================[typedef]================================================*/

typedef struct{

   int32_t i2c; // I2C port connected to EEPROM, example I2C0
   // Use this if fixed address
   bool_t A0;          // EEPROM I2C address
   bool_t A1;          // EEPROM I2C address
   bool_t A2;		   // EEPROM I2C address
   // Use this if controlling address from MCU
   int32_t gpioA0;     // GPIO conected to A0 of EEPROM
   int32_t gpioA1;     // GPIO conected to A1 of EEPROM  
   // Use this if controlling EEPROM power from MCU
   int32_t gpioPower;  // GPIO to manage power of EEPROM
   // Use this if controlling WP pin from MCU
   int32_t gpioWP;     // GPIO conected to Write Proyection Pin of EEPROM
   // EEPROM capacity
   int32_t pageSize;   // EEPROM page size [bytes]
   int32_t memorySize; // EEPROM total memory size [bytes]
} Eeprom24C32_t;

typedef struct{

	uint8_t  year;	 /* 0 to 99 */
    uint8_t  month;  /* 1 to 12   */
	uint8_t  mday;	 /* 1 to 31   */
	uint8_t  wday;	 /* 1 to 7    */
	uint8_t  hour;	 /* 0 to 23   */
	uint8_t  min;	 /* 0 to 59   */
	uint8_t  sec;	 /* 0 to 59   */
	uint8_t  alarm1_seconds;	 /* 0 to 59   */
	uint8_t  alarm1_minutes;	 /* 0 to 59   */
	uint8_t  alarm1_hours;	 /* 1–12 + AM/PM / 00–23 */
	uint8_t  alarm1_DY_DT;	 /* 01 to 7  or 1 to 31 */
	uint8_t  alarm2_minutes;	 /* 0 to 59   */
	uint8_t  alarm2_hours;	 /* 1–12 + AM/PM / 00–23 */
	uint8_t  alarm2_DY_DT;	 /* 01 to 7  or 1 to 31 */
	uint8_t control;
	uint8_t control_status;
	uint8_t aging_offset;
	uint8_t MSB_temp;     	/* temperatura byte superior */
	uint8_t LSB_temp;		/* temperatura byte inferior */
} rtcDS3231_t;

/*==================[external functions declaration]=========================*/

//-----------------------------------------------------------------------------
//  RTC and EEPROM MANAGEMENT
//-----------------------------------------------------------------------------

uint8_t eeprom24C32I2cAddress( Eeprom24C32_t* eeprom);

uint8_t RTC_setAddress( void );

//-----------------------------------------------------------------------------
// INITIALIZATION
//-----------------------------------------------------------------------------

bool_t eeprom24C32Init( Eeprom24C32_t* eeprom,
		int32_t i2c, bool_t A0, bool_t A1, bool_t A2,
		int32_t pageSize, int32_t memorySize );

void RTC_Init( rtcDS3231_t* now);

//-----------------------------------------------------------------------------
// WRITE OPERATIONS
//-----------------------------------------------------------------------------

// Write time registers
void RTC_write_time( rtcDS3231_t* time, int32_t i2c);

// Reset alarm flags
void RTC_reset_alarm( rtcDS3231_t* time, int32_t i2c);

// Byte Write
bool_t eeprom24C32WriteByte( Eeprom24C32_t* eeprom,
                                uint32_t memoryAddress, uint8_t byteToWrite );

bool_t eeprom24C32WriteDate( Eeprom24C32_t* eeprom24C32, uint16_t* eeprom_address, rtcDS3231_t time);

// Page Write
bool_t eeprom24C32WritePage( Eeprom24C32_t* eeprom, uint32_t page,
                                uint8_t* byteBuffer, uint32_t byteBufferSize );

//-----------------------------------------------------------------------------
// READ OPERATIONS
//-----------------------------------------------------------------------------

// Read time registers
rtcDS3231_t RTC_read_time( rtcDS3231_t* now, int32_t i2c);

// Current Address Read
bool_t eeprom24C32ReadCurrentAddress( Eeprom24C32_t* eeprom,
                                         uint32_t memoryAddress,
                                         uint8_t* readedByte );
                                         
// Random Read
bool_t eeprom24C32ReadRandom( Eeprom24C32_t* eeprom,
                                 uint32_t memoryAddress, uint8_t* readedByte );

// Sequential Read
bool_t eeprom24C32ReadSequential( Eeprom24C32_t* eeprom, uint32_t address,
                                     uint8_t* byteBuffer, uint32_t byteBufferSize );


/*==================[c++]====================================================*/
#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* _24C32_DS3132_H_ */

/*=============================================================================
 * Author: Ignacio Majul <imajul89@gmail.com>
 * Date: 2019/12/10
 *===========================================================================*/

/*=====[Inclusions of function dependencies]=================================*/

#include "TP_RTOS.h"
#include "24C32_DS3231.h"
#include "sapi.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/*=====[Definition macros of private constants]==============================*/

DEBUG_PRINT_ENABLE

#define UP      1
#define FALLING 2
#define DOWN	3
#define RISING  4

#define ANTI_BOUNCE_TIME_MS 50

/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/

typedef struct
{
	rtcDS3231_t date;
	Eeprom24C32_t eeprom24C32;
	uint16_t eeprom_address;
	xQueueHandle Cola_ISR;           // cola de mensajes para manejo de IRQ

}rtc_eeprom_t;

rtc_eeprom_t rtc_eeprom;

SemaphoreHandle_t Mutex_uart;    //Mutex que protege a la UART de concurrencia
SemaphoreHandle_t Mutex_rtc;     //Mutex que protege al modulo RTC de concurrencia
SemaphoreHandle_t Mutex_eeprom;  //Mutex que protege al modulo EEPROM de concurrencia

xQueueHandle Cola_EEPROM;        // cola de mensajes para manejo de EEPROM

/*=====[Definitions of private global variables]=============================*/

/*=====[Main function, program entry point after power on or reset]==========*/

void My_IRQ_Init (void);

void Read_task( void* taskParmPtr );

void Write_task( void* taskParmPtr );

int main( void )
{
	// ----- Setup -----------------------------------
	boardInit();
	debugPrintConfigUart( UART_USB, 115200 );

	Mutex_uart = xSemaphoreCreateMutex();
	Mutex_rtc= xSemaphoreCreateMutex();
	Mutex_eeprom= xSemaphoreCreateMutex();

	rtc_eeprom.Cola_ISR = xQueueCreate(10,sizeof(uint8_t));
	Cola_EEPROM = xQueueCreate(10,sizeof(rtc_eeprom_t));

	My_IRQ_Init();

	i2cInit( I2C0, 100000 );
	debugPrintlnString( "I2C initialization complete." );

	RTC_Init(&rtc_eeprom.date);        // inicializo la estructura time con los registros horarios
	debugPrintlnString( "RTC initialization complete." );

	eeprom24C32Init( &rtc_eeprom.eeprom24C32, I2C0, 1, 1, 1, EEPROM24C32_PAGE_SIZE, EEPROM_32_K_BIT );  // inicializo la EEPROM
	rtc_eeprom.eeprom_address = EEPROM24C32_FIRST_MEMORY_ADDRESS;
	debugPrintlnString( "EEPROM initialization complete." );

	RTC_write_time(&rtc_eeprom.date, I2C0);  // cargo la hora en el RTC DS3231

	eeprom24C32WriteDate(&rtc_eeprom.eeprom24C32, &rtc_eeprom.eeprom_address, rtc_eeprom.date); // escribo la fecha inicial en la EEPROM

	xTaskCreate(Read_task, (const char *)"Read_RTC", configMINIMAL_STACK_SIZE*2, &rtc_eeprom, tskIDLE_PRIORITY+1, 0);

	xTaskCreate(Write_task,(const char *)"Write_EEPROM",configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, 0);

	vTaskStartScheduler();

	while( true )
	{

	}

	// YOU NEVER REACH HERE, because this program runs directly or on a
	// microcontroller and is not called by any Operating System, as in the
	// case of a PC program.
	return 0;
}

void Read_task( void* taskParmPtr )
{
	uint8_t pin;
	rtc_eeprom_t* rtc_eeprom_p;
	rtc_eeprom_p = (rtc_eeprom_t*) taskParmPtr;

	while (TRUE)
	{
		if (xQueueReceive(rtc_eeprom_p->Cola_ISR, &pin, portMAX_DELAY))               // Espero evento de Lectura completada
		{
			if (pdFALSE == (xQueueReceive(rtc_eeprom_p->Cola_ISR, &pin, (ANTI_BOUNCE_TIME_MS / portTICK_RATE_MS))))
			{
				switch (pin)
				{
				case GPIO0:
					xSemaphoreTake( Mutex_uart, portMAX_DELAY);                 // Proteccion de la UART
					debugPrintlnString("Alarma de tiempo generada");
					xSemaphoreGive( Mutex_uart );
					break;

				case TEC1:
					xSemaphoreTake( Mutex_uart, portMAX_DELAY);                 // Proteccion de la UART
					debugPrintlnString("Tecla presionada");
					xSemaphoreGive( Mutex_uart );
					break;
				}
			}

			xSemaphoreTake( Mutex_rtc, portMAX_DELAY);                  // Proteccion del RTC
			rtc_eeprom_p->date = RTC_read_time( &(rtc_eeprom_p->date), I2C0);   // Leo los registros del RTC
			RTC_reset_alarm(&(rtc_eeprom_p->date), I2C0);			        // Reseteo alarma en el RTC
			xSemaphoreGive( Mutex_rtc );

			xQueueSend(Cola_EEPROM, rtc_eeprom_p, portMAX_DELAY);
		}
	}
}

void Write_task( void* taskParmPtr )
{
	uint8_t Pin;
	rtc_eeprom_t rtc_eeprom;
	uint8_t readedByte = 0;

	while (TRUE)
	{
		if (xQueueReceive(Cola_EEPROM, &rtc_eeprom, portMAX_DELAY)) //Espero evento de Lectura completada.
		{
			xSemaphoreTake( Mutex_uart, portMAX_DELAY);   // proteccion de la UART.
			debugPrintHex( rtc_eeprom.date.hour,8 );   // imprimo la hora por UART
			debugPrintString(":");
			debugPrintHex( rtc_eeprom.date.min,8 );
			debugPrintString(":");
			debugPrintHex( rtc_eeprom.date.sec,8 );
			debugPrintString("  ");
			debugPrintHex( rtc_eeprom.date.mday,8 );
			debugPrintString("/");
			debugPrintHex( rtc_eeprom.date.month,8 );
			debugPrintString("/");
			debugPrintHex( rtc_eeprom.date.year,8 );
			debugPrintString(" Temperatura = ");
			debugPrintInt(rtc_eeprom.date.MSB_temp);
			debugPrintString(".");
			debugPrintInt(rtc_eeprom.date.LSB_temp);
			debugPrintString("\r\n");
			xSemaphoreGive( Mutex_uart );

			xSemaphoreTake( Mutex_eeprom, portMAX_DELAY);  // proteccion del RTC.
			eeprom24C32WriteByte( &rtc_eeprom.eeprom24C32, rtc_eeprom.eeprom_address, rtc_eeprom.date.MSB_temp); // escribo la temperatura leida por I2C en la EEPROM
			eeprom24C32ReadRandom( &rtc_eeprom.eeprom24C32, rtc_eeprom.eeprom_address, &readedByte );  // leo la hora escrita e imprimo
			debugPrintString( "Temperatura leida de EEPROM: " );
			debugPrintInt( readedByte );
			debugPrintString("\r\n");
			debugPrintString("\r\n");

			if(rtc_eeprom.eeprom_address == EEPROM24C32_LAST_MEMORY_ADDRESS) {    // si llego al ultimo registro de la EEPROM genero overflow de direccon
				rtc_eeprom.eeprom_address = EEPROM24C32_FIRST_MEMORY_ADDRESS;
			} else {
				rtc_eeprom.eeprom_address++;
			}
			xSemaphoreGive( Mutex_eeprom );
		}
	}
}

void GPIO0_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (Chip_PININT_GetFallStates(LPC_GPIO_PIN_INT) & PININTCH0) // Verificamos que la interrupción es la esperada
	{
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH0); // Borramos el flag de interrupción
		uint8_t pin = GPIO0;
		xQueueSendFromISR(rtc_eeprom.Cola_ISR, &pin, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void GPIO1_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (Chip_PININT_GetFallStates(LPC_GPIO_PIN_INT) & PININTCH1)  //Verificamos que la interrupción es la esperada
	{
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH1); //Borramos el flag de interrupción
		uint8_t pin = TEC1;
		xQueueSendFromISR(rtc_eeprom.Cola_ISR, &pin, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void My_IRQ_Init (void)
{
	Chip_PININT_Init(LPC_GPIO_PIN_INT);

	Chip_SCU_GPIOIntPinSel(0, 3, 0);  // Canal 0, Puerto 3, Pin 0 correspondiente al pin GPIO0 de la EDU-CIAA
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH0);
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH0);

	Chip_SCU_GPIOIntPinSel(1, 0, 4);  // Canal 1, Puerto 0, Pin 4 correspondiente a la TEC1 de la EDU-CIAA
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH1);
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH1);

	NVIC_SetPriority(PIN_INT0_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY-1);
	NVIC_EnableIRQ(PIN_INT0_IRQn);
	NVIC_SetPriority(PIN_INT1_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY-2);
	NVIC_EnableIRQ(PIN_INT1_IRQn);

}

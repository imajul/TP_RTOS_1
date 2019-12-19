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

/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/

typedef struct
{
	rtcDS3231_t date;
	Eeprom24C32_t eeprom24C32;
	uint16_t eeprom_address;

}rtc_eeprom_t;

rtc_eeprom_t rtc_eeprom;

SemaphoreHandle_t Mutex_uart;    //Mutex que protege a la UART de concurrencia
SemaphoreHandle_t Mutex_rtc;     //Mutex que protege al modulo RTC de concurrencia
SemaphoreHandle_t Mutex_eeprom;  //Mutex que protege al modulo EEPROM de concurrencia

xQueueHandle Cola_ISR;           // cola de mensajes para manejo de IRQ
xQueueHandle Cola_EEPROM;        // cola de mensajes para manejo de EEPROM

/*=====[Definitions of private global variables]=============================*/

/*=====[Main function, program entry point after power on or reset]==========*/

void My_IRQ_Init (void);

void Read( void* taskParmPtr );

void Write( void* taskParmPtr );

int main( void )
{
	// ----- Setup -----------------------------------
	boardInit();
	debugPrintConfigUart( UART_USB, 115200 );

	My_IRQ_Init();

	Mutex_uart = xSemaphoreCreateMutex();
	Mutex_rtc= xSemaphoreCreateMutex();
	Mutex_eeprom= xSemaphoreCreateMutex();

	Cola_ISR = xQueueCreate(10,sizeof(gpioMap_t));
	Cola_EEPROM = xQueueCreate(10,sizeof(rtc_eeprom_t));

	i2cInit( I2C0, 100000 );
	debugPrintlnString( "I2C initialization complete." );

	RTC_Init(&rtc_eeprom.date);  // inicializo la estructura time con los registros horarios
	debugPrintlnString( "RTC initialization complete." );

	eeprom24C32Init( &rtc_eeprom.eeprom24C32, I2C0, 1, 1, 1, EEPROM24C32_PAGE_SIZE, EEPROM_32_K_BIT );  // inicializo la EEPROM
	rtc_eeprom.eeprom_address = EEPROM24C32_FIRST_MEMORY_ADDRESS;
	debugPrintlnString( "EEPROM initialization complete." );

	RTC_write_time(&rtc_eeprom.date, I2C0);  // cargo la hora en el RTC DS3231

	eeprom24C32WriteDate(&rtc_eeprom.eeprom24C32, &rtc_eeprom.eeprom_address, rtc_eeprom.date); // escribo la fecha inicial en la EEPROM

	xTaskCreate(
			Read,                     // Funcion de la tarea a ejecutar
			(const char *)"Read_RTC",     // Nombre de la tarea como String amigable para el usuario
			configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
			0,               			  // Parametros de tarea
			tskIDLE_PRIORITY+1,         // Prioridad de la tarea
			0                           // Puntero a la tarea creada en el sistema
	);

	xTaskCreate(
			Write,                     // Funcion de la tarea a ejecutar
			(const char *)"Write_EEPROM",     // Nombre de la tarea como String amigable para el usuario
			configMINIMAL_STACK_SIZE*2,		 // Cantidad de stack de la tarea
			0,                 					// Parametros de tarea
			tskIDLE_PRIORITY+1,         // Prioridad de la tarea
			0                           // Puntero a la tarea creada en el sistema
	);

	vTaskStartScheduler();


	while( true )
	{

	}

	// YOU NEVER REACH HERE, because this program runs directly or on a
	// microcontroller and is not called by any Operating System, as in the
	// case of a PC program.
	return 0;
}

void Read( void* taskParmPtr )
{
	gpioMap_t Pin;
	rtc_eeprom_t rtc_eeprom;

	while (TRUE)
	{
		if (xQueueReceive(Cola_ISR, &Pin, portMAX_DELAY))         // Espero evento de Lectura completada
		{
			xSemaphoreTake( Mutex_uart, portMAX_DELAY);           // Proteccion de la UART
			printf("Alarma recibida");
			xSemaphoreGive( Mutex_uart );

			xSemaphoreTake( Mutex_rtc, portMAX_DELAY);            // Proteccion del RTC
			rtc_eeprom.date = RTC_read_time( &rtc_eeprom.date, I2C0);   // Leo los registros del RTC
			RTC_reset_alarm(&(rtc_eeprom.date), I2C0);			  // Reseteo alarma en el RTC
			xSemaphoreGive( Mutex_rtc );

			xQueueSend(Cola_EEPROM, &rtc_eeprom, portMAX_DELAY);
		}
	}
}

void Write( void* taskParmPtr )
{
	gpioMap_t Pin;
	rtc_eeprom_t rtc_eeprom;

	while (TRUE)
	{
		if (xQueueReceive(Cola_EEPROM, &rtc_eeprom, portMAX_DELAY)) //Espero evento de Lectura completada
		{
			xSemaphoreTake( Mutex_uart, portMAX_DELAY);   // proteccion de la UART
			printf("Lectura recibida. Temperatura = %c", rtc_eeprom.date.MSB_temp);
			xSemaphoreGive( Mutex_uart );

			xSemaphoreTake( Mutex_eeprom, portMAX_DELAY);  // proteccion del RTC
			eeprom24C32WriteByte( &rtc_eeprom.eeprom24C32, rtc_eeprom.eeprom_address, rtc_eeprom.date.MSB_temp); // escribo la temperatura leida por I2C en la EEPROM
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
		gpioMap_t Pin = GPIO1;
		xQueueSendFromISR(Cola_ISR, &Pin, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void GPIO2_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (Chip_PININT_GetFallStates(LPC_GPIO_PIN_INT) & PININTCH2)  //Verificamos que la interrupción es la esperada
	{
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH2); //Borramos el flag de interrupción
		gpioMap_t Pin = TEC1;
		xQueueSendFromISR(Cola_ISR, &Pin, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

//Función de inicialización de IRQs
void My_IRQ_Init (void)
{
	//Inicializamos las interrupciones (LPCopen)
	Chip_PININT_Init(LPC_GPIO_PIN_INT);

	//Inicializamos de cada evento de interrupción (LPCopen)
	Chip_SCU_GPIOIntPinSel(0, 0, 4); //Mapeo del pin donde ocurrirá el evento y el canal al que lo va a enviar. (Canal 0 a 7, Puerto GPIO, Pin GPIO)
	// pin correspondiente a la tecla TEC1
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH0);//Se configura el canal para que se active por flanco
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH0);//Se configura para que el flanco sea el de bajada

	Chip_SCU_GPIOIntPinSel(1, 3, 3);//En este caso el canal de interrupción es 1
	// pin correspondiente al GPIO1 de la EDU-CIAA
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH1);
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH1);//Se configura para que el flanco sea el de bajada

	//Una vez que se han configurado los eventos para cada canal de interrupcion
	//Se activan las interrupciones para que comiencen a llamar al handler
	NVIC_SetPriority(PIN_INT0_IRQn, 8);
	NVIC_EnableIRQ(PIN_INT0_IRQn);
	NVIC_SetPriority(PIN_INT1_IRQn, 8);
	NVIC_EnableIRQ(PIN_INT1_IRQn);

}

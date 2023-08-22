/* Standard includes. */
#include <stdio.h>
#include <conio.h>
#include <string.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "extint.h"

/* Hardware simulator utility functions */
#include "HW_access.h"

/* SERIAL SIMULATOR CHANNEL TO USE */
#define COM_CH (0)
#define COM_CH1 (1)
#define COM_CH2 (2)

	/* TASK PRIORITIES */
#define	OBRADA				( tskIDLE_PRIORITY + 1 )
#define	SERVICE_TASK_PRI	( tskIDLE_PRIORITY + 2 )
#define	TASK_SERIAL_SEND	( tskIDLE_PRIORITY + 3 )
#define	TASK_SERIAL_REC		( tskIDLE_PRIORITY + 4 )
#define	SENZORI				( tskIDLE_PRIORITY + 5 )



/* TASKS: FORWARD DECLARATIONS */
static void SerialSend_Task0(void* pvParameters);
static void SerialSend_Task1(void* pvParameters);
static void led_bar_tsk(void* pvParameters);
static void Seg7_ispis_task(void* pvParameters);
static void slanjeNaPC_tsk(void* pvParameters);

/* TIMER FUNCTIONS*/
static void Ispis_TimerCallback(TimerHandle_t Tmh);
static void RX_senzori_callback(TimerHandle_t Tmh);

/* Funkcije deklaracija pre upotrebe */



/* Globalne promenljive za generalnu upotrebu */
#define R_BUF_SIZE (32)
static uint8_t ukljuceno = 0;
uint8_t automatski;
double trenutna_temp = 0;



/* 7-SEG NUMBER DATABASE - ALL HEX DIGITS */
static const uint8_t hexnum[] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71 };

/* GLOBAL OS-HANDLES */
static SemaphoreHandle_t LED_INT_BinarySemaphore;
static SemaphoreHandle_t TBE_BS_0, TBE_BS_1, TBE_BS_2;
static SemaphoreHandle_t RXC_BS_0, RXC_BS_1, RXC_BS_2;
SemaphoreHandle_t RX_senzori_semafor;
SemaphoreHandle_t seg7_ispis;
SemaphoreHandle_t mutex_serijska;
SemaphoreHandle_t semafor1;
SemaphoreHandle_t stanje_PC;
QueueHandle_t LED_Queue;

TimerHandle_t per_TimerHandle;
TimerHandle_t RX_senzori_timer;
TimerHandle_t ispis_timer;


static QueueHandle_t seg7_queue;
static QueueHandle_t seg7automatski_queue;
static QueueHandle_t seg7d_queue;
static QueueHandle_t serijska_ispis_queue;
static QueueHandle_t serijska_ispis_duzina;
static QueueHandle_t serijska_prijem_niz = NULL;
static QueueHandle_t serijska_prijem_duzina = NULL;


/* Strukture za redove */
typedef struct serijska_ispis_podataka {//svi potrebni podaci za ispis na serijsku
	uint8_t duzina_stringa;
	uint8_t poruka[80];
}serijska_ispis_podataka;

typedef struct seg7_podaci { //svi potrebni podaci za ispis na 7-segmentni displej 
	uint8_t min_max_vr;
	uint8_t automatski;
	double trenutna_temp;
	double minimalno;
	double maksimalno;
}seg7_podaci;

typedef struct podaci_za_stanje { //svi potrebni podaci za formiranje poruke za stanje
	double temperatura;
	uint8_t ukljuceno;
	uint8_t automatski;
}podaci_za_stanje;



// INTERRUPTS //
/* OPC - ON INPUT CHANGE - INTERRUPT HANDLER */
static uint32_t OnLED_ChangeInterrupt(void)
{
	BaseType_t xHigherPTW = pdFALSE;

	xSemaphoreGiveFromISR(LED_INT_BinarySemaphore, &xHigherPTW);

	portYIELD_FROM_ISR((uint32_t)xHigherPTW);
}


/* TBE - TRANSMISSION BUFFER EMPTY - INTERRUPT HANDLER */
static uint32_t prvProcessTBEInterrupt(void)
{
	BaseType_t xHigherPTW = pdFALSE;

	if (get_TBE_status(0) != 0)
	{
		if (xSemaphoreGiveFromISR(TBE_BS_0, &xHigherPTW) != pdTRUE)
		{
			printf("Greska TBE_BS_0\n");
		}
	}
	if (get_TBE_status(1) != 0)
	{
		if(xSemaphoreGiveFromISR(TBE_BS_1, &xHigherPTW) != pdTRUE)
		{
			printf("Greska TBE_BS_1\n");
		}
	}
	if (get_TBE_status(2) != 0)
	{
		if(xSemaphoreGiveFromISR(TBE_BS_2, &xHigherPTW) != pdTRUE)
		{
		printf("Greska TBE_BS_2\n");
		}
	}
	portYIELD_FROM_ISR((uint32_t)xHigherPTW);
}


/* RXC - RECEPTION COMPLETE - INTERRUPT HANDLER */
static uint32_t prvProcessRXCInterrupt(void)
{
	BaseType_t xHigherPTW = pdFALSE;

	if (get_RXC_status(0) != 0)
	{
		if (xSemaphoreGiveFromISR(RXC_BS_0, &xHigherPTW) != pdTRUE)
		{
			printf("Greska RXC_BS_0\n");
		}
	}

	if (get_RXC_status(1) != 0)
	{
		if (xSemaphoreGiveFromISR(RXC_BS_1, &xHigherPTW) != pdTRUE)
		{
			printf("Greska RXC_BS_1\n");
		}
	}

	if (get_RXC_status(2) != 0) {
		if (xSemaphoreGiveFromISR(RXC_BS_2, &xHigherPTW) != pdTRUE)
		{
			printf("Greska RXC_BS_2\n");
		}
	}

	portYIELD_FROM_ISR((uint32_t)xHigherPTW);
}


/* PERIODIC TIMER CALLBACK */
static void TimerCallback(TimerHandle_t xTimer)
{
	xSemaphoreGive(seg7_ispis);
} //svakih 250ms osvjezavanje displeja  

static void Ispis_TimerCallback(TimerHandle_t ispis_timer) {
	xSemaphoreGive(stanje_PC);

}

static void RX_senzori_Timercallback(TimerHandle_t RX_senzori_timer) {
	xSemaphoreGive(RX_senzori_semafor);
}


/* MAIN - SYSTEM STARTUP POINT */
//void main(void);
void main_demo(void)
{
	// Inicijalizacija periferija //
	init_7seg_comm();
	init_LED_comm();
	
	// Inicijalizacija serijske TX na kanalu 0 //
		if (init_serial_uplink(COM_CH) != 0)
		{
			printf("Neuspjesna inicijalizacija TX na kanalu 0\n");
		}
	// Inicijalizacija serijske RX na kanalu 0 //
		if (init_serial_downlink(COM_CH) != 0)
		{
			printf("Neuspjesna inicijalizacija RX na kanalu 0\n");
		}
	// Inicijalizacija serijske TX na kanalu 1 //
		if (init_serial_uplink(COM_CH1) != 0)
		{
			printf("Neuspjesna inicijalizacija TX na kanalu 1\n");
		}
	// Inicijalizacija serijske RX na kanalu 1 //
		if (init_serial_downlink(COM_CH1) != 0)
		{
			printf("Neuspjesna inicijalizacija RX na kanalu 1\n");
		}
	// Inicijalizacija serijske TX na kanalu 2 //
		if (init_serial_uplink(COM_CH2) != 0)
		{
			printf("Neuspjesna inicijalizacija TX na kanalu 2\n");
		}
	// Inicijalizacija serijske RX na kanalu 2 //
		if (init_serial_downlink(COM_CH2) != 0)
		{
			printf("Neuspjesna inicijalizacija RX na kanalu 2\n");
		}

	// INTERRUPT HANDLERS
	/* ON INPUT CHANGE INTERRUPT HANDLER */
		vPortSetInterruptHandler(portINTERRUPT_SRL_OIC, OnLED_ChangeInterrupt);

	/* SERIAL TRANSMISSION INTERRUPT HANDLER */
		vPortSetInterruptHandler(portINTERRUPT_SRL_TBE, prvProcessTBEInterrupt);

	/* SERIAL RECEPTION INTERRUPT HANDLER */
		vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXCInterrupt);

		
	/* Create binary semaphores */
		LED_INT_BinarySemaphore = xSemaphoreCreateBinary();

	/*TIMER*/
		//per_TimerHandle = xTimerCreate("Timer1", pdMS_TO_TICKS(80), pdTRUE, NULL, TimerCallback);
		//xTimerStart(per_TimerHandle, 0);
		
		//RX_senzori_timer = xTimerCreate("Timer2", pdMS_TO_TICKS(200), pdTRUE, NULL, RX_senzori_Timercallback);
		//xTimerStart(RX_senzori_timer, 0);
		
		ispis_timer = xTimerCreate("Timer3", pdMS_TO_TICKS(10000), pdTRUE, NULL, Ispis_TimerCallback);
		xTimerStart(ispis_timer, 0);

		/* Create TBE semaphore - serial transmit comm */
		TBE_BS_0 = xSemaphoreCreateBinary();
		if (TBE_BS_0 == NULL)
		{
			printf("Greska prilikom kreiranja TBE_BS_0\n");
		}
		TBE_BS_1 = xSemaphoreCreateBinary();
		if (TBE_BS_1 == NULL)
		{
			printf("Greska prilikom kreiranja TBE_BS_1\n");
		}
		TBE_BS_2 = xSemaphoreCreateBinary();
		if (TBE_BS_2 == NULL)
		{
			printf("Greska prilikom kreiranja TBE_BS_2\n");
		}

		/* Create RXC semaphore - serial transmit comm */
		RXC_BS_0 = xSemaphoreCreateBinary();
		if (RXC_BS_0 == NULL)
		{
			printf("Greska prilikom kreiranja RXC_BS_0\n");
		}
		RXC_BS_1 = xSemaphoreCreateBinary();
		if (RXC_BS_1 == NULL)
		{
			printf("Greska prilikom kreiranja RXC_BS_1\n");
		}
		RXC_BS_2 = xSemaphoreCreateBinary();
		if (RXC_BS_2 == NULL)
		{
			printf("Greska prilikom kreiranja RXC_BS_2\n");
		}
	// Semafori
		mutex_serijska = xSemaphoreCreateBinary();
		semafor1 = xSemaphoreCreateBinary();
		seg7_ispis = xSemaphoreCreateBinary();
		stanje_PC = xSemaphoreCreateBinary();
	// Kreiranje taskova //
		BaseType_t status;
		
	// SERIAL RECEIVER AND SEND TASK //
		status = xTaskCreate(SerialSend_Task0, "SRx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_SEND, NULL);
		if (status != pdPASS)
		{
			printf("Greska prilikom kreiranja taska\n");
		}
		status = xTaskCreate(SerialSend_Task1, "STx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_SEND, NULL);
		if (status != pdPASS)
		{
			printf("Greska prilikom kreiranja taska\n");
		}


		seg7_queue = xQueueCreate(1, sizeof(uint8_t));// red za seg7 ispis
		seg7automatski_queue = xQueueCreate(2, sizeof(uint8_t));
		seg7d_queue = xQueueCreate(2, sizeof(double[3]));
		
		serijska_ispis_queue = xQueueCreate(3, sizeof(uint8_t[80])); //red za skladistenje poruke za ispis
		serijska_ispis_duzina = xQueueCreate(3, sizeof(uint8_t)); //red za skladistenje duzine rijeci

		serijska_prijem_niz = xQueueCreate(3, sizeof(uint8_t[12])); //red za skladistenje primljene rijeci (komande)
		serijska_prijem_duzina = xQueueCreate(3, sizeof(uint8_t)); //red za skladistenje duzine primljene rijeci


	
	/* Kreiranje redova za komunikaciju izmedju taskova */
		LED_Queue = xQueueCreate(2, sizeof(uint8_t));

	/* create a led bar TASK */
		xTaskCreate(led_bar_tsk, "ST", configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);
		xTaskCreate(Seg7_ispis_task, "Seg_7", configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);
		//xTaskCreate(stanje_PC, "Stanje", configMINIMAL_STACK_SIZE, NULL, OBRADA_TASK_PRI, NULL);
	

	// START SCHEDULER
	vTaskStartScheduler();
	while (1);
}



/* Sa ovim taskom simuliramo vrednost trenutne temperature koja stize sa senzora svakih 200ms, tako sto
   svakih 200ms saljemo karakter 'A' i u AdvUniCom simulatoru omogucimo tu opciju (AUTO ukljucen) */
static void SerialSend_Task0(void* pvParameters) { 
	uint8_t prim = (uint8_t)'A';

	for (;;) //umesto while(1)
	{ 
		vTaskDelay(pdMS_TO_TICKS(200));
		if (send_serial_character(COM_CH, prim) != 0) 
		{
			printf("Greska prilikom slanja - kanal 0");
		}
	}

}

/* Sa ovim taskom simuliramo vrednost trenutne temperature koja stize sa senzora svakih 200ms, tako sto
   svakih 200ms saljemo karakter 'A' i u AdvUniCom simulatoru omogucimo tu opciju (AUTO ukljucen) */
static void SerialSend_Task1(void* pvParameters) {
	uint8_t prim = (uint8_t)'a';

	for (;;) 
	{ 
		vTaskDelay(pdMS_TO_TICKS(200));
		if (send_serial_character(COM_CH1, prim) != 0)
		{
			printf("Greska prilikom slanja - kanal 1");
		}
	}

}

static void led_bar_tsk(void* pvParameters)
{
	uint8_t senzor_ocitavenje; // koristimo ga za masku kao promenljivu kako bi ocitali vrednost bita
	uint8_t min_max_vr;
	uint8_t vent = 0;
	//serijska_ispis_podataka* ispis_podataka;
	uint8_t duzina_niza = 0;
	uint8_t temp1[80] = { 0 };

	while (1)
	{
		xSemaphoreTake(LED_INT_BinarySemaphore, portMAX_DELAY);
		get_LED_BAR(0, &senzor_ocitavenje);



		if ((senzor_ocitavenje & 0x01) != 0) {
			ukljuceno = 1;
			set_LED_BAR(1, 0x01);
			printf("led_klima_aktivna\n");
		}
		else {
			ukljuceno = 0;
			set_LED_BAR(1, 0x00);
		}

		if (ukljuceno && !automatski) {
			if ((senzor_ocitavenje & 0x10) != 0) {
				vent = 1;

				xSemaphoreTake(mutex_serijska, portMAX_DELAY);
				//char poruka[] = "VENT:1";
				strcpy(temp1, "VENT:1");
				duzina_niza = sizeof(temp1) - 1;
				if (xQueueSend(serijska_ispis_queue, &temp1, 0U) != pdTRUE) {
					printf("Neuspjesno slanje podataka u red\n");
				}
				if (xQueueSend(serijska_ispis_duzina, &duzina_niza, 0U) != pdTRUE) {
					printf("Neuspjesno slanje podataka u red\n");
				}

				send_serial_character(COM_CH2, 13);
				xSemaphoreTake(semafor1, portMAX_DELAY);

				xSemaphoreGive(mutex_serijska);

			}
			
			else if ((senzor_ocitavenje & 0x20) != 0) {
				vent = 2;
				xSemaphoreTake(mutex_serijska, portMAX_DELAY);
				//char poruka[] = "VENT:2";
				strcpy(temp1, "VENT:2");
				duzina_niza = sizeof(temp1) - 1;

				if (xQueueSend(serijska_ispis_queue, &temp1, 0U) != pdTRUE) {
					printf("Neuspjesno slanje podataka u red\n");
				}
				if (xQueueSend(serijska_ispis_duzina, &duzina_niza, 0U) != pdTRUE) {
					printf("Neuspjesno slanje podataka u red\n");
				}

				send_serial_character(COM_CH2, 13);
				xSemaphoreTake(semafor1, portMAX_DELAY);
				xSemaphoreGive(mutex_serijska);
			}

			else if ((senzor_ocitavenje & 0x40) != 0) {
				vent = 3;
				xSemaphoreTake(mutex_serijska, portMAX_DELAY);
				//char poruka[] = "VENT:3";
				strcpy(temp1, "VENT:3");
				duzina_niza = sizeof(temp1) - 1;

				if (xQueueSend(serijska_ispis_queue, &temp1, 0U) != pdTRUE) {
					printf("Neuspjesno slanje podataka u red\n");
				}
				if (xQueueSend(serijska_ispis_duzina, &duzina_niza, 0U) != pdTRUE) {
					printf("Neuspjesno slanje podataka u red\n");
				}

				send_serial_character(COM_CH2, 13);
				xSemaphoreTake(semafor1, portMAX_DELAY);

				xSemaphoreGive(mutex_serijska);
			}

			else if ((senzor_ocitavenje & 0x80) != 0) {
				vent = 4;
				xSemaphoreTake(mutex_serijska, portMAX_DELAY);
				//char poruka[] = "VENT:4";
				strcpy(temp1, "VENT:4");
				duzina_niza = sizeof(temp1) - 1;

				if (xQueueSend(serijska_ispis_queue, &temp1, 0U) != pdTRUE) {
					printf("Neuspjesno slanje podataka u red\n");
				}
				if (xQueueSend(serijska_ispis_duzina, &duzina_niza, 0U) != pdTRUE) {
					printf("Neuspjesno slanje podataka u red\n");
				}

				send_serial_character(COM_CH2, 13);
				xSemaphoreTake(semafor1, portMAX_DELAY);
			}
			
			else {
				vent = 0;
			}

			printf("vent: %u\n", (unsigned)vent);
		}

		if (senzor_ocitavenje & 0x02) { // min, max vrednost drugom ledovkom od dole
			min_max_vr = 1;
		}
		else {
			min_max_vr = 0;
		}

		if (xQueueSend(seg7_queue, &min_max_vr, 0U) != pdTRUE) {
			printf("Neuspjesno slanje podataka u red\n");
		}
	}
		
}

void Seg7_ispis_task(void* pvParameters) {

	double minimalna=0;
	double maksimalna=0;
	double trenutna_temp=0;

	double d[3] = {minimalna, maksimalna, trenutna_temp };

	uint8_t min_max_vr = 0;
	uint8_t automatski = 0;

	while (1) {

		xSemaphoreTake(seg7_ispis, portMAX_DELAY);

		if (xQueueReceive(seg7d_queue, &d, pdMS_TO_TICKS(250)) != pdTRUE) {
			printf("Greska pri preuzimanju vrijednosti iz reda\n");
		}
		if (xQueueReceive(seg7_queue, &min_max_vr, pdMS_TO_TICKS(250)) != pdTRUE) {
			printf("Greska pri preuzimanju vrijednosti iz reda\n");
		}
		if (xQueueReceive(seg7automatski_queue, &automatski, pdMS_TO_TICKS(250)) != pdTRUE) {
			printf("Greska pri preuzimanju vrijednosti iz reda\n");
		}

		// Trenutna temperatura
		select_7seg_digit(1);
		set_7seg_digit(hexnum[(uint8_t)trenutna_temp / 10]);
		select_7seg_digit(2);
		set_7seg_digit(hexnum[(uint8_t)trenutna_temp % 10]);

		if (automatski == 1) {
			select_7seg_digit(0);
			set_7seg_digit(hexnum[0]);
		}
		else {
			select_7seg_digit(0);
			set_7seg_digit(hexnum[1]);
		}

		//Trebalo bi da moze i samo set_7seg_digit(hexnum[(uint8_t)trenutna_temp / 10], 1); bez select_7seg ali iskače warning.

		// Maksimum i miminum
		if (min_max_vr == 1) { 
			select_7seg_digit(3);
			set_7seg_digit(hexnum[(uint8_t)maksimalna / 10]);
			select_7seg_digit(4);
			set_7seg_digit(hexnum[(uint8_t)maksimalna % 10]);
		}
		else {
			select_7seg_digit(3);
			set_7seg_digit(hexnum[(uint8_t)minimalna / 10]);
			select_7seg_digit(4);
			set_7seg_digit(hexnum[(uint8_t)minimalna % 10]);
		}
	}
}

// Prvo sam probao preko Queue, pa nesto nije radilo kako treba, pa sam uradio preko promenljive temp2 i strcpy i strcat.
void slanjeNaPC_tsk(void* pvParameters) {
	uint8_t duzina_niza_ispis = 0;
	uint8_t temp2[80] = { 0 };

	while (1) {
		xSemaphoreTake(stanje_PC, portMAX_DELAY);
		xSemaphoreTake(mutex_serijska, portMAX_DELAY);


		// stanje
		/*if (xQueueSend(serijska_ispis_queue, "Stanje: ", 0U) != pdTRUE) {
			printf("Neuspesno slanje podataka u red\n");
		
		} */

		strcpy(temp2, "Stanje: ");
		duzina_niza_ispis = sizeof("Stanje: ") - 1;


	}
	if (automatski == 1) {

		/*if (xQueueSend(serijska_ispis_queue, "AUTOMATSKI", 0U) != pdTRUE) {
			printf("Neuspesno slanje podataka u red\n");
		} */
		strcat(temp2, "AUTOMATSKI");
		duzina_niza_ispis += sizeof("AUTOMATSKI") - 1;
	}
	else {
		/*if (xQueueSend(serijska_ispis_queue, "MANUELNO", 0U) != pdTRUE) {
			printf("Neuspesno slanje podataka u red\n");
		} */
		strcat(temp2, "MANUELNO");
		duzina_niza_ispis += sizeof("MANUELNO") - 1;
	}

	/*if (xQueueSend(serijska_ispis_queue, ", radi: ", 0U) != pdTRUE) {
		printf("Neuspesno slanje podataka u red\n");
	} */
	strcat(temp2, "radi: ");
	duzina_niza_ispis += sizeof(", radi: ") - 1;

	// ukljuceno
	if (ukljuceno == 1) {

		/*if (xQueueSend(serijska_ispis_queue, "UKLJUCENO", 0U) != pdTRUE) {
			printf("Neuspesno slanje podataka u red\n");
		} */
		strcat(temp2, "UKLJUCENO");
		duzina_niza_ispis += sizeof("UKLJUCENO") - 1;
	}
	else {

		/*if (xQueueSend(serijska_ispis_queue, "ISKLJUCENO", 0U) != pdTRUE) {
			printf("Neuspesno slanje podataka u red\n");
		}*/
		strcat(temp2, "ISKLJUCENO");
		duzina_niza_ispis += sizeof("ISKLJUCENO") - 1;
	}
	/*temperatura
	if (xQueueSend(serijska_ispis_queue, ", temp:", 0U) != pdTRUE) {
		printf("Neuspesno slanje podataka u red\n");
	}
	duzina_niza_ispis += sizeof(", temp:") - 1;


	char desetica = (unsigned)trenutna_temp / 10 + '0';
	//send_serial_character(COM_CH2, desetica + '0');
	char jedinica = (unsigned)trenutna_temp % 10 + '0';
	//send_serial_character(COM_CH2, jedinica + '0');
	/*if ((xQueueSend(serijska_ispis_queue, &temp_karakter, 0U)) != pdTRUE) {
		printf("Neuspesno slanje podataka u red\n");
	}
	duzina_niza_ispis++;
	xQueueSend(serijska_ispis_queue, &desetica, 0U);

	xQueueSend(serijska_ispis_queue, &jedinica, 0U);

	send_serial_character(COM_CH2, 13);

	*/

	//temperatura
	strcat(temp2, ", temp:");
	duzina_niza_ispis += sizeof(", temp:") - 1;

	temp2[duzina_niza_ispis++] = (unsigned)trenutna_temp / 10 + '0';
	temp2[duzina_niza_ispis++] = (unsigned)trenutna_temp % 10 + '0';

	if (xQueueSend(serijska_ispis_queue, &temp2, 0U) != pdTRUE) {
		printf("Neuspesno slanje podataka u red\n");

	}
	if (xQueueSend(serijska_ispis_duzina, &duzina_niza_ispis, 0U) != pdTRUE) {
		printf("Neuspesno slanje podataka u red\n");

		// Zapocinjemo ispis
		send_serial_character(COM_CH2, 13);

		xSemaphoreTake(semafor1, portMAX_DELAY);
		xSemaphoreGive(mutex_serijska);
	}
}
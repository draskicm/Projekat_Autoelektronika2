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
static void SerialWrite(void* pvParameters);
static void SerialSend_Task0(void* pvParameters);
static void SerialSend_Task1(void* pvParameters);
static void primljeno_sa_senzora(void* pvParameters);
static void primljeno_sa_kanal0(void* pvParameters);
static void primljeno_sa_kanal1(void* pvParameters);
static void SerialReceive_Task(void* pvParameters);
static void obrada_podataka(void* pvParameters);

/* TIMER FUNCTIONS*/
static void RX_senzori_callback(TimerHandle_t Tmh); //svakih 200ms primi vrijednosti sa senzora i obradi ih
static void ispis_tajmer_callback(TimerHandle_t Tmh); //svakih 10s ispisuje stanje sistema

/* Funkcije deklaracija pre upotrebe */

/* Globalne promenljive za generalnu upotrebu */
#define R_BUF_SIZE (32)
static char buffer0[R_BUF_SIZE];
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

//TimerHandle_t per_TimerHandle;
//TimerHandle_t RX_senzori_timer;
//TimerHandle_t ispis_podaci_tajmer;

static QueueHandle_t queue_senzor1;
static QueueHandle_t queue_senzor2;
static QueueHandle_t seg7_queue;
static QueueHandle_t seg7automatski_queue;
static QueueHandle_t seg7d_queue;
static QueueHandle_t serijska_ispis_queue;
static QueueHandle_t serijska_ispis_duzina;
static QueueHandle_t serijska_prijem_niz;
static QueueHandle_t serijska_prijem_duzina;
static QueueHandle_t formi;

/* Strukture za redove */
typedef struct serijska_ispis_podataka {//svi potrebni podaci za ispis na serijsku
	uint8_t duzina_stringa;
	uint8_t poruka[70];
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
		xSemaphoreGiveFromISR(TBE_BS_0, &xHigherPTW);

	if (get_TBE_status(1) != 0)
		xSemaphoreGiveFromISR(TBE_BS_1, &xHigherPTW);

	if (get_TBE_status(2) != 0)
		xSemaphoreGiveFromISR(TBE_BS_2, &xHigherPTW);

	portYIELD_FROM_ISR(xHigherPTW);
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
} //svakih 200ms osvezavanje displeja  


static void ispis_tajmer_callback(TimerHandle_t ispis_podaci_tajmer) {
	xSemaphoreGive(stanje_PC);
}


static void RX_senzori_callback(TimerHandle_t RX_senzori_timer) {
	xSemaphoreGive(RX_senzori_semafor);
}



/* MAIN - SYSTEM STARTUP POINT */
void main_demo(void);
void main_demo(void)
{

	// Inicijalizacija periferija //
	if (init_LED_comm() != 0)
	{
		printf("Neuspesna inicijalizacija\n");
	}
	if (init_7seg_comm() != 0)
	{
		printf("Neuspesna inicijalizacija\n");
	}

	// Inicijalizacija serijske TX na kanalu 0 //
	if (init_serial_uplink(COM_CH) != 0)
	{
		printf("Neuspesna inicijalizacija TX na kanalu 0\n");
	}
	// Inicijalizacija serijske RX na kanalu 0 //
	if (init_serial_downlink(COM_CH) != 0)
	{
		printf("Neuspesna inicijalizacija RX na kanalu 0\n");
	}
	// Inicijalizacija serijske TX na kanalu 1 //
	if (init_serial_uplink(COM_CH1) != 0)
	{
		printf("Neuspesna inicijalizacija TX na kanalu 1\n");
	}
	// Inicijalizacija serijske RX na kanalu 1 //
	if (init_serial_downlink(COM_CH1) != 0)
	{
		printf("Neuspesna inicijalizacija RX na kanalu 1\n");
	}
	// Inicijalizacija serijske TX na kanalu 2 //
	if (init_serial_uplink(COM_CH2) != 0)
	{
		printf("Neuspesna inicijalizacija TX na kanalu 2\n");
	}
	// Inicijalizacija serijske RX na kanalu 2 //
	if (init_serial_downlink(COM_CH2) != 0)
	{
		printf("Neuspesna inicijalizacija RX na kanalu 2\n");
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
	if (LED_INT_BinarySemaphore == NULL)
	{
		printf("Greska prilikom kreiranja semafora LED_INT_BinarySemaphore\n");
	}
	mutex_serijska = xSemaphoreCreateBinary();
	if (mutex_serijska == NULL)
	{
		printf("Greska prilikom kreiranja semafora mutex_serijska\n");
	}
	semafor1 = xSemaphoreCreateBinary();
	if (semafor1 == NULL)
	{
		printf("Greska prilikom kreiranja semafora semafor1\n");
	}
	seg7_ispis = xSemaphoreCreateBinary();
	if (seg7_ispis == NULL)
	{
		printf("Greska prilikom kreiranja semafora seg7_ispis\n");
	}
	stanje_PC = xSemaphoreCreateBinary();
	if (stanje_PC == NULL)
	{
		printf("Greska prilikom kreiranja semafora stanje_PC\n");
	}


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


	// Kreiranje taskova //
	BaseType_t status;

	// SERIAL RECEIVER AND SEND TASK //
	status = xTaskCreate(SerialWrite, "Write", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_SEND, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska SerialWrite\n");
	}
	status = xTaskCreate(SerialSend_Task0, "Task0", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_SEND, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska SerialSend_Task0\n");
	}
	status = xTaskCreate(SerialSend_Task1, "Task1", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_SEND, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska SerialSend_Task1\n");
	}
	status = xTaskCreate(SerialReceive_Task, "SRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_REC, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska SerialReceive_Task\n");
	}

	/* Kreiranje redova */
	seg7_queue = xQueueCreate(1, sizeof(uint8_t));// red za seg7 ispis
	if (seg7_queue == NULL)
	{
		printf("Greska prilikom kreiranja seg7_queue\n");
	}
	seg7automatski_queue = xQueueCreate(2, sizeof(uint8_t));
	if (seg7automatski_queue == NULL)
	{
		printf("Greska prilikom kreiranja seg7automatski_queue\n");
	}
	seg7d_queue = xQueueCreate(2, sizeof(double[3]));
	if (seg7d_queue == NULL)
	{
		printf("Greska prilikom kreiranja seg7d_queue\n");
	}
	serijska_ispis_queue = xQueueCreate(3, sizeof(uint8_t[70])); //red za skladistenje poruke za ispis
	if (serijska_ispis_queue == NULL)
	{
		printf("Greska prilikom kreiranja serijska_ispis_queue\n");
	}
	serijska_ispis_duzina = xQueueCreate(3, sizeof(uint8_t)); //red za skladistenje duzine reci
	if (serijska_ispis_duzina == NULL)
	{
		printf("Greska prilikom kreiranja serijska_ispis_duzina\n");
	}
	serijska_prijem_niz = xQueueCreate(3, sizeof(uint8_t[12])); //red za skladistenje primljene reci (komande)
	if (serijska_prijem_niz == NULL)
	{
		printf("Greska prilikom kreiranja serijska_prijem_niz\n");
	}
	serijska_prijem_duzina = xQueueCreate(3, sizeof(uint8_t)); //red za skladistenje duzine primljene reci
	if (serijska_prijem_duzina == NULL)
	{
		printf("Greska prilikom kreiranja serijska_prijem_duzina\n");
	}
	formi = xQueueCreate(1, sizeof(uint8_t[3]));
	if (formi == NULL)
	{
		printf("Greska prilikom kreiranja formi\n");
	}
	queue_senzor1 = xQueueCreate(2, sizeof(double)); //kreiramo Queue duzine dva double
	if (queue_senzor1 == NULL)
	{
		printf("Greska prilikom kreiranja queue_senzor1\n");
	}
	queue_senzor2 = xQueueCreate(2, sizeof(double)); //kreiramo Queue duzine dva double
	if (queue_senzor2 == NULL)
	{
		printf("Greska prilikom kreiranja queue_senzor2\n");
	}
	LED_Queue = xQueueCreate(2, sizeof(uint8_t));
	if (LED_Queue == NULL)
	{
		printf("Greska prilikom kreiranja LED_Queue\n");
	}

	/* create a led bar TASK */
	status = xTaskCreate(obrada_podataka, "obrada", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)OBRADA, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska obrada_podataka\n");
	}
	status = xTaskCreate(primljeno_sa_kanal0, "kanal0", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_REC, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska primljeno_sa_kanal0\n");
	}
	status = xTaskCreate(primljeno_sa_kanal1, "kanal1", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_REC, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska primljeno_sa_kanal1\n");
	}

	// START SCHEDULER
	vTaskStartScheduler();
	while (1);
}

//ispis poruke na serijsku
static void SerialWrite(void* pvParameters) {
	uint8_t priv = 0;
	uint8_t p[70];
	uint8_t duzina = 0;

	for (;;) {
		xSemaphoreTake(TBE_BS_2, portMAX_DELAY); //ceka se da bude prazan
		xQueueReceive(serijska_ispis_queue, &p, pdMS_TO_TICKS(20)); //pogledaj da li ima novih vrednosti, ako nema za 20ms radi dalje
		xQueueReceive(serijska_ispis_duzina, &duzina, pdMS_TO_TICKS(20)); //pogledaj ima li nesto novo, ako nema za 20ms radi dalje

		if (priv < duzina) { //slanje slovo po slovo dok se ne ispise cela poruka
			send_serial_character(COM_CH2, p[priv++]);
		}
		else {	//svaki parametar se resetuje i daje se semafor da se ispis zavrsen
			priv = 0;
			xSemaphoreGive(semafor1);
			duzina = 0;
		}
	}
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

static void primljeno_sa_senzora(void* pvParameters) {
	double senzor1 = 0;
	double senzor2 = 0;
	double o[3] = { 0 };
	double min = 0;
	double max = 99;

	for (;;) {
		if (xQueueReceive(queue_senzor1, &senzor1, portMAX_DELAY) != pdTRUE)
		{
			printf("ERROR\n");
		}
		if (xQueueReceive(queue_senzor2, &senzor2, portMAX_DELAY) != pdTRUE)
		{
			printf("ERROR\n");
		}

		trenutna_temp = (senzor1 + senzor2) / 2;	//srednja vrednost temperature sa senzora
		if (trenutna_temp > max)
		{
			max = trenutna_temp;
		}
		if (trenutna_temp < min && senzor1 != 0 && senzor2 != 0) { //ne treba nikad da senzor bude 0 pa mora postojati ovaj uslov
			min = trenutna_temp;
		}

		o[0] = min;
		o[1] = max;
		o[2] = trenutna_temp;

		xQueueSend(seg7d_queue, &o, 0U); //0U=unsigned integer zero
	}
}

static void primljeno_sa_kanal0(void* pvParameters) {
	double senzor1 = 0;
	uint8_t cc = 0;
	uint8_t broj_karak = (uint8_t)0;
	uint8_t temp_kanal0[7] = { 0 };

	for (;;) {
		if (xSemaphoreTake(RXC_BS_0, portMAX_DELAY) != pdTRUE)
		{
			printf("Greska prilikom preuzimanja semafora RXC_BS_0\n");
		}

		if (get_serial_character(COM_CH, &cc) != 0)
		{
			printf("Greska pri prijemu karaktera na kanalu 0\n");
		}

		if (cc == 0x0d) {
			senzor1 = atof(temp_kanal0);
			broj_karak = 0;
			xQueueSend(queue_senzor1, &senzor1, 0U);
		}
		else {
			temp_kanal0[broj_karak++] = cc;
		}
	}
}

static void primljeno_sa_kanal1(void* pvParameters) {
	double senzor2 = 0;
	uint8_t cc = 0;
	uint8_t broj_karak = (uint8_t)0;
	uint8_t temp_kanal1[7] = { 0 };

	for (;;) {
		if (xSemaphoreTake(RXC_BS_1, portMAX_DELAY) != pdTRUE)
		{
			printf("Greska prilikom preuzimanja semafora RXC_BS_1\n");
		}

		if (get_serial_character(COM_CH1, &cc) != 0)
		{
			printf("Greska pri prijemu karaktera na kanalu 1\n");
		}

		if (cc == 0x0d) {
			senzor2 = atof(temp_kanal1);
			broj_karak = 0;
			xQueueSend(queue_senzor2, &senzor2, 0U);
		}
		else {
			temp_kanal1[broj_karak++] = cc;
		}
	}
}

//ucitavanje komande poruke sa kanala 2
static void SerialReceive_Task(void* pvParameters) {
	uint8_t cc = 0;
	uint8_t duzina = 0;
	uint8_t prom = (uint8_t)0;
	uint8_t prom_buff[12];

	for (;;) {
		if (xSemaphoreTake(RXC_BS_2, portMAX_DELAY) != pdTRUE)/*suspend task until a character is received*/
		{
			printf("Greska prilikom preuzimanja semafora na kanalu 2\n");
		}

		if (get_serial_character(COM_CH2, &cc) != 0)//ucitavanje primljenog karakteraka u promenljivu cc
		{
			printf("Greska pri prijemu karaktera na kanalu 2\n");
		}

		if (cc == 0x0d) {
			duzina = prom;
			xQueueSend(serijska_prijem_niz, &prom_buff, 0U);
			xQueueSend(serijska_prijem_duzina, &prom, 0U);
			prom = 0;
		}

		else if (prom < R_BUF_SIZE) {
			prom_buff[prom++] = cc;
		}
	}
}

static void obrada_podataka(void* pvParameters)
{
	
}

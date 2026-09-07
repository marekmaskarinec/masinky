#include "ch32fun.h"
#include "ch32v20xhw.h"
#include "funconfig.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef APPCONF_UART
#define UART_BUF_SIZE 32

static volatile bool g_uart_rx_err = false;
static volatile char g_uart_data[2][UART_BUF_SIZE];
static volatile size_t g_uart_data_len;
static volatile size_t g_uart_data_idx = 0;
static volatile bool g_uart_data_rdy = false;

__attribute__((interrupt)) __attribute__((section(".srodata"))) void
USART2_IRQHandler(void)
{
	static size_t len = 0;

	USART2->STATR |= USART_STATR_RXNE;
	g_uart_rx_err |= (USART2->STATR & USART_STATR_ORE) != 0;
	char byte = USART2->DATAR & 0xff;
	const size_t idx = !g_uart_data_idx;
	g_uart_data[idx][len++] = byte;
	if (byte == '\n') {
		g_uart_data_len = len;
		len = 0;
		g_uart_data_idx = idx;
		g_uart_data_rdy = true;
		return;
	}

	if (len >= sizeof(g_uart_data[0])) {
		g_uart_rx_err = 1;
		len = 0;
	}
}

static void
uart_init(void)
{
	RCC->APB1PCENR |= RCC_APB1Periph_USART2;
	RCC->AHBPCENR = RCC_AHBPeriph_SRAM | RCC_AHBPeriph_DMA1;

	funPinMode(PA2, GPIO_CFGLR_OUT_10Mhz_AF_PP);
	funPinMode(PA3, GPIO_CFGLR_IN_FLOAT);

	USART2->CTLR1 = USART_WordLength_8b | USART_Parity_No | USART_Mode_Tx;
	USART2->CTLR2 = USART_StopBits_1;
	USART2->CTLR3 = USART_HardwareFlowControl_None;

	// const int baud_rate = 9600;
	// baud = FCLK / (16 * USARTDIV)
	// 115200 == 144MHz / (16 * USARTDIV)
	// USARTDIV == 78.125 == 78 + (2/16)
	// NOTE: This value works only when the debugger isn't attached. In case UART
	// is needed with a debugger, 65.125 should be used.
	USART2->BRR = (78 << 3) | 2;
	// USART2->BRR = USART2->BRR = (((FUNCONF_SYSTEM_CORE_CLOCK) + (baud_rate) / 2) /
	// (baud_rate));
	USART2->CTLR1 |= CTLR1_UE_Set | USART_CTLR1_RE;

	USART2->CTLR1 |= USART_CTLR1_RXNEIE;
	NVIC_EnableIRQ(USART2_IRQn);
}

int
putchar(int c)
{
	int timeout = 1000000;
	while (!(USART2->STATR & USART_FLAG_TC) && --timeout)
		;
	if (timeout == 0)
		return 0;
	USART2->DATAR = c;
	return 1;
}

int
_write(int fd, const char *buf, int size)
{
	for (size_t i = 0; i < size; i++) {
		putchar(buf[i]);
	}
	return size;
}
#endif

// These values are calculated based on the system clock clock being 144MHz
// The print function bellow will print the actual baud rate
// if the clock or divider is different.
//                     TS1           TS2            BRP
#define AHB1_DIV 1
#define CAN_BAUD_25kbps ((5 << 16) | (4 << 20) | (479 / AHB1_DIV))
#define CAN_BAUD_50kbps ((5 << 16) | (4 << 20) | (239 / AHB1_DIV))
#define CAN_BAUD_100kbps ((5 << 16) | (4 << 20) | (119 / AHB1_DIV))
#define CAN_BAUD_125kbps ((5 << 16) | (4 << 20) | (59 / AHB1_DIV))
#define CAN_BAUD_250kbps ((5 << 16) | (4 << 20) | (47 / AHB1_DIV))
#define CAN_BAUD_500kbps ((5 << 16) | (4 << 20) | (23 / AHB1_DIV))
#define CAN_BAUD_750kbps ((5 << 16) | (4 << 20) | (11 / AHB1_DIV))
#define CAN_BAUD_1Mbps ((5 << 16) | (4 << 20) | (7 / AHB1_DIV))
#define CAN_BAUD CAN_BAUD_1Mbps
#define CAN_FIFO 0
#define CAN_STATUS_OK (CAN_TSTATR_RQCP0 | CAN_TSTATR_TXOK0)

static inline uint32_t
can_get_apb1_div(void)
{
	// Get APB1 clock divider
	if (RCC->CFGR0 & RCC_PPRE1_DIV2)
		return 2;
	else if (RCC->CFGR0 & RCC_PPRE1_DIV4)
		return 4;
	else if (RCC->CFGR0 & RCC_PPRE1_DIV8)
		return 8;
	else if (RCC->CFGR0 & RCC_PPRE1_DIV16)
		return 16;
	else
		return 1; // No division
}

static void
can_init(void)
{
	printf("Init CAN\n");

	RCC->APB1PCENR |= RCC_APB1Periph_CAN1;
	RCC->APB2PCENR |= RCC_APB2Periph_AFIO;

	// Configure AF remapping for CAN
	AFIO->PCFR1 &= ~AFIO_PCFR1_CAN_REMAP;	    // Clear remap bits
	AFIO->PCFR1 |= AFIO_PCFR1_CAN_REMAP_REMAP1; // Set PA11 and PA12 for CAN RX and TX

	// Configure GPIO
	funPinMode(PA11, GPIO_CFGLR_IN_FLOAT);
	funPinMode(PA12, GPIO_CFGLR_OUT_50Mhz_AF_PP);

	// Pull down CAN_STB
	funPinMode(PA1, GPIO_CFGLR_OUT_10Mhz_PP);
	funDigitalWrite(PA1, FUN_LOW);
	// Pull up CAN_VIO
	funPinMode(PA5, GPIO_CFGLR_OUT_10Mhz_PP);
	funDigitalWrite(PA5, FUN_HIGH);

	// Wake up
	CAN1->CTLR &= (~(uint32_t)CAN_CTLR_SLEEP);
	// Initialise
	CAN1->CTLR |= CAN_CTLR_INRQ | CAN_CTLR_NART;

	// Wait for intialisation to complete
	while (!(CAN1->STATR & CAN_STATR_INAK))
		;

	printf("System Core Clock: %uMHz\n", FUNCONF_SYSTEM_CORE_CLOCK / 1000000);

	CAN1->BTIMR = CAN_BAUD;
	CAN1->BTIMR |= 0b11 << 24;

	const uint32_t ts1 = (CAN1->BTIMR & CAN_BTIMR_TS1) >> 16;
	const uint32_t ts2 = (CAN1->BTIMR & CAN_BTIMR_TS2) >> 20;
	const uint32_t brp = CAN1->BTIMR & CAN_BTIMR_BRP;
	const uint32_t sjw = (CAN1->BTIMR & CAN_BTIMR_SJW) >> 24;
	const uint32_t baud =
	    (FUNCONF_SYSTEM_CORE_CLOCK / can_get_apb1_div()) / ((ts1 + ts2 + 3) * (brp + 1));
	printf("ts1=%lu,ts2=%lu,brp=%lu,sjw=%lu,baud=%lubps\n", ts1, ts2, brp, sjw, baud);

	// Set up rx filter
	CAN1->FCTLR |= FCTLR_FINIT; // Enter initialisation mode
	{
		static const size_t filter_id = 0; // Filter 0

		// Set ID to match
		CAN1->sFilterRegister[filter_id].FR1 = 0x0;
		// Set which bits of the ID to match (mask)
		CAN1->sFilterRegister[filter_id].FR2 = 0; // Accept all messages

		CAN1->FAFIFOR = (CAN_FIFO << filter_id); // assign filter to FIFO
		CAN1->FMCFGR = (0 << filter_id);	 // 1: id mode, 0: mask mode
		CAN1->FSCFGR = (1 << filter_id);	 // 1: 32 bit filter, 0: 16 bit filter
		CAN1->FWR = (1 << filter_id);		 // enable filter
	}
	CAN1->FCTLR &= ~FCTLR_FINIT; // Exit initialisation mode

	CAN1->CTLR &= ~(1 << 16UL);
	CAN1->CTLR &= ~(uint32_t)CAN_CTLR_INRQ;
	// Wait for intialisation to complete
	while (CAN1->STATR & CAN_STATR_INAK)
		;
}

static int
can_tx(uint32_t id, const uint8_t *src, size_t size)
{
	int mailbox = -1;

	if (CAN1->TSTATR & CAN_TSTATR_TME0)
		mailbox = 0;
	else if (CAN1->TSTATR & CAN_TSTATR_TME1)
		mailbox = 1;
	else if (CAN1->TSTATR & CAN_TSTATR_TME2)
		mailbox = 2;

	if (-1 != mailbox) {
		// Set ID
		CAN1->sTxMailBox[mailbox].TXMIR = (id << 21) & CAN_TXMI0R_STID;

		// Set data length
		CAN1->sTxMailBox[mailbox].TXMDTR = size & 0x0F;

		// Clear Data
		CAN1->sTxMailBox[mailbox].TXMDLR = 0;
		CAN1->sTxMailBox[mailbox].TXMDHR = 0;

		for (size_t i = 0; i < size; i++)
			if (i < 4)
				CAN1->sTxMailBox[mailbox].TXMDLR |= ((uint32_t)src[i] << (i * 8));
			else
				CAN1->sTxMailBox[mailbox].TXMDHR |=
				    ((uint32_t)src[i] << ((i - 4) * 8));

		CAN1->sTxMailBox[mailbox].TXMIR |= CAN_TXMI0R_TXRQ;
	}
	return mailbox;
}

static int
can_message_sent(int mailbox)
{
	if (mailbox < 0 || mailbox > 2)
		return 0;

	const uint32_t status = (CAN1->TSTATR >> (mailbox * 8)) & 0xFF;

	const int sent = (status & CAN_STATUS_OK) == CAN_STATUS_OK;
	const int mailbox_empty = (CAN1->TSTATR >> (26 + mailbox)) & 1;

	return sent && mailbox_empty;
}

static size_t
can_rx(uint8_t *dst, uint32_t *id, uint8_t fifo)
{
	// Get ID
	if (CAN_RXMI0R_IDE & CAN1->sFIFOMailBox[fifo].RXMIR)
		*id = (CAN_RXMI0R_EXID & (uint32_t)CAN1->sFIFOMailBox[fifo].RXMIR) >> 3;
	else
		*id = (CAN_RXMI0R_STID & (uint32_t)CAN1->sFIFOMailBox[fifo].RXMIR) >> 21;

	size_t size = CAN_RXMDT0R_DLC & CAN1->sFIFOMailBox[fifo].RXMDTR;

	for (size_t i = 0; i < size; i++)
		if (i < 4)
			dst[i] = CAN1->sFIFOMailBox[fifo].RXMDLR >> (i * 8);
		else
			dst[i] = CAN1->sFIFOMailBox[fifo].RXMDHR >> ((i - 4) * 8);

	// Release the FIFO
	if (fifo == 0)
		CAN1->RFIFO0 |= CAN_RFIFO0_RFOM0;
	else
		CAN1->RFIFO1 |= CAN_RFIFO1_RFOM1;

	return size;
}

static uint32_t g_esig;
static void
esig_init(void)
{
	printf("Init ESIG\n");
	g_esig = ESIG->UID0 ^ ESIG->UID1 ^ ESIG->UID2;
}

static void
process_uart(void)
{
	if (!g_uart_data_rdy)
		return;
	g_uart_data_rdy = 0;

	static char buf[UART_BUF_SIZE + 1];
	const size_t len = g_uart_data_len;
	for (size_t i = 0; i < len; i++) {
		buf[i] = g_uart_data[g_uart_data_idx][i];
	}
	buf[len] = 0;

	if (strcmp(buf, "ping\n") == 0) {
		printf("pong\n");
	} else if (strcmp(buf, "esig\n") == 0) {
		printf("%08lX\n", g_esig);
	} else if (strstr(buf, "cantx") == buf) {
		can_tx(0x123, buf, len);
		printf("ok\n");
	} else {
		printf("err\n");
	}
}

static void
process_can(void)
{
	static uint8_t data[32];

	const uint32_t messages = CAN1->RFIFO0 & CAN_RFIFO0_FMP0;
	if (!messages)
		return;

	uint32_t id = 0;
	const size_t numbytes = can_rx(data, &id, CAN_FIFO);
	printf("msg rx: id=%lu, data=", id);
	for (int i = 0; i < numbytes; i++)
		printf("%02x", data[i]);
	putchar('\n');
}

int
main()
{
	SystemInit();

	funGpioInitAll();
	// Enable GPIOs
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC;

	Delay_Ms(200);

#ifdef APPCONF_UART
	uart_init();
#endif
	printf("Hello :)\nBuild  " __DATE__ " " __TIME__ "\n");
	esig_init();
	can_init();

	printf("Init completed\n");

	while (1) {
		process_uart();
		process_can();
		Delay_Ms(1);
	}
}

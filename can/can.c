/* Small example showing how to use the SWIO programming pin to
   do printf through the debug interface */

#include "ch32fun.h"
#include "ch32v20xhw.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define UART_BUF_SIZE 32

static volatile bool g_uart_tx_dma_busy = false;
static volatile bool g_uart_rx_err = false;
static volatile char g_uart_data[2][UART_BUF_SIZE];
static volatile size_t g_uart_data_len;
static volatile size_t g_uart_data_idx = 0;
static volatile bool g_uart_data_rdy = false;
// DMA transfer completion interrupt. It will fire when the DMA transfer is
// complete.
__attribute__((interrupt)) __attribute__((section(".srodata"))) void
DMA1_Channel7_IRQHandler(void)
{
	// Clear flag
	DMA1->INTFCR |= DMA_CTCIF7;
	g_uart_tx_dma_busy = false;
}

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
	USART2->BRR = (65 << 3) | 2;
	// USART2->BRR = USART2->BRR = (((FUNCONF_SYSTEM_CORE_CLOCK) + (baud_rate) / 2) /
	// (baud_rate));
	USART2->CTLR1 |= CTLR1_UE_Set | USART_CTLR1_RE;

	// Init DMA TX on channel 7
	USART2->CTLR3 = USART_DMAReq_Tx;
	// Disable channel just in case there is a transfer in progress
	DMA1_Channel7->CFGR &= ~DMA_CFGR1_EN;
	DMA1_Channel7->PADDR = (intptr_t)&USART2->DATAR;
	// MEM2MEM: 0 (memory to peripheral)
	// PL: 0 (low priority since UART is a relatively slow peripheral)
	// MSIZE/PSIZE: 0 (8-bit)
	// MINC: 1 (increase memory address)
	// CIRC: 0 (one shot)
	// DIR: 1 (read from memory)
	// TEIE: 0 (no tx error interrupt)
	// HTIE: 0 (no half tx interrupt)
	// TCIE: 1 (transmission complete interrupt enable)
	// EN: 0 (do not enable DMA yet)
	DMA1_Channel7->CFGR = DMA_CFGR1_MINC | DMA_CFGR1_DIR | DMA_CFGR1_TCIE;
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);

	USART2->CTLR1 |= USART_CTLR1_RXNEIE;
	NVIC_EnableIRQ(USART2_IRQn);
}

static int
uart_tx(const char *buf, size_t size)
{
	for (size_t i = 0; i < size; i++) {
		while (!(USART2->STATR & USART_FLAG_TC))
			;
		USART2->DATAR = *buf++;
	}
	return size;
}

static int
uart_tx_dma(const char *buf, size_t size)
{
	g_uart_tx_dma_busy = true;
	// Disable DMA channel (just in case a transfer is pending)
	DMA1_Channel7->CFGR &= ~DMA_CFGR1_EN;
	// Set transfer length and source address
	DMA1_Channel7->CNTR = size;
	DMA1_Channel7->MADDR = (intptr_t)buf;
	// Enable DMA channel to start the transfer
	DMA1_Channel7->CFGR |= DMA_CFGR1_EN;
	return size;
}

static uint32_t g_esig;
static void
esig_init(void)
{
	g_esig = ESIG->UID0 ^ ESIG->UID1 ^ ESIG->UID2;
}

static void
process_uart_rx(void)
{
	static char buf[UART_BUF_SIZE + 1];
	const size_t len = g_uart_data_len;
	for (size_t i = 0; i < len; i++) {
		buf[i] = g_uart_data[g_uart_data_idx][i];
	}
	buf[len] = 0;
	printf("line=%s, err=%x\n", buf, g_uart_rx_err);

	if (strcmp(buf, "ping\n") == 0) {
		strncpy(buf, "pong\n", sizeof(buf) - 1);
	} else if (strcmp(buf, "esig\n") == 0) {
		sprintf(buf, "%08lX\n", g_esig);
	} else {
		strncpy(buf, "err\n", sizeof(buf) - 1);
	}

	uart_tx(buf, strlen(buf));
}

int
main()
{
	SystemInit();

	funGpioInitAll();
	// Enable GPIOs
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC;

	esig_init();
	uart_init();

	while (1) {
		if (g_uart_data_rdy) {
			process_uart_rx();
			g_uart_data_rdy = 0;
		}
		Delay_Ms(1);
	}
}

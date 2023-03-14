#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <math.h>

// Constantes
#define SPI_NUMERO SPI1
#define SPI_CS_PORT GPIOB
#define SPI_CS_PIN GPIO6

// Structs
struct comando_spi{ // Estrutura para o comando spi

	bool r_w; // Leitura ou escrita
	uint16_t endereco; // Endereco do registrador
	uint8_t tamanho; // Tamanho dos dados
	uint8_t comando_buf[2]; // Buffer de dados

}comando_spi;

// Funcoes
static void spi_enviar(struct comando_spi comando){ // Envio do comando serial

		// Criacao do buffer do comando
		uint16_t spi_buffer[2] = {0x0000, 0x0000}; // Buffer de dados spi

		spi_buffer[0] |= (comando.r_w << 15) | ((comando.tamanho & 0x03) << 13) | (comando.endereco & 0x1fff); 
		spi_buffer[1] |= (comando.comando_buf[1] << 8) | comando.comando_buf[0];

		gpio_clear(SPI_CS_PORT, SPI_CS_PIN);
		for(int i=0; i<2; i++){
			spi_send(SPI_NUMERO, spi_buffer[i]);
			(void) spi_read(SPI_NUMERO); // Aguarda o envio
		}
		gpio_set(SPI_CS_PORT, SPI_CS_PIN);
}

static void spi_config(void) // Rotina de configuracao spi
{
	rcc_periph_clock_enable(RCC_SPI1); // hab clock SPI1
	rcc_periph_clock_enable(RCC_GPIOA); // hab clock GPIOA
	rcc_periph_clock_enable(RCC_GPIOB); // hab clock GPIOB

	// CS 
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);
	gpio_set_af(GPIOA, GPIO_AF5, GPIO7); // GPIO7 == PA7  AF_5 == SPI1 (MUX pino)

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
	gpio_set_af(GPIOA, GPIO_AF5, GPIO6); // GPIO6 == PA6 AF_5 == SPI1 (MUX pino)
	
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5);
	gpio_set_af(GPIOA, GPIO_AF5, GPIO5); // GPIO5 == PA5 AF_5 == SPI1 (MUX pino)

	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_8, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_16BIT, SPI_CR1_MSBFIRST);

	spi_enable(SPI1);

}

static void usart_setup(void) // Rotina de configuracao usart
{
	// Controle de clock e reset (RCC)
	rcc_periph_clock_enable(RCC_USART2); // hab clock periferico de USART
	rcc_periph_clock_enable(RCC_GPIOA); // hab clock GPIOA
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2); // GPIO2 == PA2 AF_7 == USART (MUX pino)

	// Configuracao gpio
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2); // GPIO2 == PA2 AF_7 == USART (MUX pino)

	// Configuracao USART
	usart_set_baudrate(USART2, 115200); // baud
	usart_set_databits(USART2, 8); // tamanho da palavra
	usart_set_stopbits(USART2, USART_STOPBITS_1); // bit de parada (USART_CR2_STOPBITS_1 - define 1 stop bit)
	usart_set_mode(USART2, USART_MODE_TX); // modo apenas tx
	//usart_set_mode(USART1, USART_MODE_TX_RX); // modo tx rx
	usart_set_parity(USART2, USART_PARITY_NONE); // bit de paridade (impar, par, nenhum)
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE); // bit de controle de fluxo - definido como none

	usart_enable(USART2); // hab USART2
}

static void usart_print(uint32_t usart, int16_t value) // Print usart
{
	int8_t i;
	int8_t nr_digits = 0;
	char buffer[25];

	if (value < 0) {
		usart_send_blocking(usart, '-');
		value = value * -1;
	}

	if (value == 0) {
		usart_send_blocking(usart, '0');
	}

	// preenche o buffer
	while (value > 0) {
		buffer[nr_digits++] = "0123456789"[value % 10]; // incrementa o indice do buffer e seleciona um dos elementos da string "0123456789" a partir de value%10
		value /= 10;
	}

	// Envia o buffer
	for (i = nr_digits-1; i >= 0; i--) {
		usart_send_blocking(usart, buffer[i]);
	}

	// Retorno e quebra de linha
	usart_send_blocking(usart, '\r');
	usart_send_blocking(usart, '\n');
}

static uint16_t spi_receber(){ // Leitura usart
	gpio_clear(SPI_CS_PORT, SPI_CS_PIN); // CS ativado
	uint16_t dados_r = spi_read(SPI_NUMERO); // Recebimento dos dados
	gpio_set(SPI_CS_PORT, SPI_CS_PIN); // CS desativado
	return dados_r;
}

static void gerar_ftw(double freq, double f0, uint8_t* ftw_v){ // Geracao da ftw

	unsigned long long ftw = 0x000000000000;

	ftw = (unsigned long long) pow(2, 48) * (freq/f0);

	ftw_v[0] = (ftw >> 0) & 0xFF;
	ftw_v[1] = (ftw >> 8) & 0xFF;
	ftw_v[2] = (ftw >> 16) & 0xFF;
	ftw_v[3] = (ftw >> 24) & 0xFF;
	ftw_v[4] = (ftw >> 32) & 0xFF;
	ftw_v[5] = (ftw >> 40) & 0xFF;
}

static void enviar_ftw(uint8_t* ftw){
}

// Main
int main(void)
{
	// Organizar e reestruturar
	uint16_t temp;

	double freq = 82.731e6;
	double f0 = 1e9;
	unsigned long long ftw = 0x000000000000;

	usart_setup();

	temp = 666;

	uint16_t buffer_spi, buffer_spi_rcv;

	ftw = (unsigned long long) pow(2, 48) * (freq/f0);

	uint8_t ftw_v[6];

	ftw_v[0] = (ftw >> 0) & 0xFF;
	ftw_v[1] = (ftw >> 8) & 0xFF;
	ftw_v[2] = (ftw >> 16) & 0xFF;
	ftw_v[3] = (ftw >> 24) & 0xFF;
	ftw_v[4] = (ftw >> 32) & 0xFF;
	ftw_v[5] = (ftw >> 40) & 0xFF;

	spi_config();

	freq = 200e6;
	f0 = 1e9;

	gerar_ftw(freq, f0, ftw_v);

	struct comando_spi comando_teste;

	comando_teste.r_w = 0; // Leitura ou escrita
	comando_teste.endereco = 0x0010; // Endereco do registrador
	comando_teste.tamanho = 0x01; // Tamanho dos dados
	comando_teste.comando_buf[1] = 0xC0; // Buffer de dados
	comando_teste.comando_buf[0] = 0x00; // Buffer de dados
	spi_enviar(comando_teste);

	comando_teste.r_w = 0; // Leitura ou escrita
	comando_teste.endereco = 0x0020; // Endereco do registrador
	comando_teste.tamanho = 0x01; // Tamanho dos dados
	comando_teste.comando_buf[1] = 0x17; // Buffer de dados
	comando_teste.comando_buf[0] = 0x00; // Buffer de dados
	spi_enviar(comando_teste);

	comando_teste.r_w = 0; // Leitura ou escrita
	comando_teste.endereco = 0x0022; // Endereco do registrador
	comando_teste.tamanho = 0x01; // Tamanho dos dados
	comando_teste.comando_buf[1] = 0x04 ; // Buffer de dados
	comando_teste.comando_buf[0] = 0x00; // Buffer de dados
	spi_enviar(comando_teste);

	while (1) {

		for(int i=5; i>=0; i--){
			usart_print(USART2, ftw_v[i]);
		}

		comando_teste.r_w = 0; // Leitura ou escrita
		comando_teste.endereco = 0x01AB; // Endereco do registrador
		comando_teste.tamanho = 0x01; // Tamanho dos dados
		comando_teste.comando_buf[1] = ftw_v[5]; // Buffer de dados
		comando_teste.comando_buf[0] = ftw_v[4]; // Buffer de dados
		spi_enviar(comando_teste);

		//comando_teste.endereco = 0x005; // Endereco do registrador
		//comando_teste.comando_buf[1] = 0x01; // Buffer de dados
		//comando_teste.comando_buf[0] = 0x00; // Buffer de dados
		//spi_enviar(comando_teste);

		comando_teste.endereco = 0x01A9; // Endereco do registrador
		comando_teste.comando_buf[1] = ftw_v[3]; // Buffer de dados
		comando_teste.comando_buf[0] = ftw_v[2]; // Buffer de dados
		spi_enviar(comando_teste);

		//comando_teste.endereco = 0x005; // Endereco do registrador
		//comando_teste.comando_buf[1] = 0x01; // Buffer de dados
		//comando_teste.comando_buf[0] = 0x00; // Buffer de dados
		//spi_enviar(comando_teste);

		comando_teste.endereco = 0x01A7; // Endereco do registrador
		comando_teste.comando_buf[1] = ftw_v[1]; // Buffer de dados
		comando_teste.comando_buf[0] = ftw_v[0]; // Buffer de dados
		spi_enviar(comando_teste);

		comando_teste.endereco = 0x005; // Endereco do registrador
		comando_teste.comando_buf[1] = 0x01; // Buffer de dados
		comando_teste.comando_buf[0] = 0x00; // Buffer de dados
		spi_enviar(comando_teste);

		for (int i = 0; i < 3e7; i++) {
			__asm__("nop");
		}

		freq = freq-10e3;
		gerar_ftw(freq, f0, ftw_v);

	}

	return 0;
}


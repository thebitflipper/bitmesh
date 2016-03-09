#ifndef __nrf24_h_
#define __nrf24_h_

#include "nrf24_definitions.h"
#include "includes.h"

#define NRF_1MBIT   0
#define NRF_2MBIT   1
#define NRF_250KBIT 2

enum NRF24_packetstatus {
	NRF24_NOT_SENT,
	NRF24_IN_TRANSIT,
	NRF24_MAX_RT,
	NRF24_SENT
};

void NRF24_init(
		uint8_t use_interrupts, /* 0(yes) or 1(no) */
		uint8_t use_crc /* 0, 1 or 2 bytes */
);
enum NRF24_packetstatus NRF24_get_packetstatus();
void NRF24_reset_packetstatus();
void NRF24_set_channel(uint8_t channel);
void NRF24_flush_tx();
void NRF24_flush_rx();
uint8_t NRF24_get_rx_size();
uint8_t NRF24_read_payload(uint8_t *pld, uint8_t *pld_len, uint8_t *pipe_nr);
void NRF24_get_addr(uint8_t *buffer, uint8_t pipe_nr);
void NRF24_set_speed(uint8_t speed);

void NRF24_set_pl_len(uint8_t pipe_nr, uint8_t len);
/* ACK will be enabled for the pipe also. */
void NRF24_enable_dyn_pl(uint8_t pipe_nr);
void NRF24_disable_dyn_pl(uint8_t pipe_nr);

void NRF24_enable_ack(uint8_t pipe_nr);
void NRF24_disable_ack(uint8_t pipe_nr);

void NRF24_enable_dyn_ack();
void NRF24_disable_dyn_ack();


void NRF24_enable_pipe(uint8_t pipe_nr);
void NRF24_disable_pipe(uint8_t pipe_nr);
/* 3 to 5 bytes */
void NRF24_set_addr_w(uint8_t addr_w);
void NRF24_set_rx_addr(uint8_t pipe_nr, uint8_t *addr);

uint8_t NRF24_send_packet(uint8_t *addr, uint8_t *payload,
			  uint8_t pl_length, uint8_t use_ack,
			  uint8_t sync, uint8_t *retransmits);

void NRF24_irq_handle();
void NRF24_poll_handle();

void NRF24_power_up();
void NRF24_power_down();

uint8_t NRF24_get_register(uint8_t reg);
void NRF24_set_register(uint8_t reg, uint8_t val);

void NRF24_print_registers();

uint8_t NRF24_is_module_connected();
#endif

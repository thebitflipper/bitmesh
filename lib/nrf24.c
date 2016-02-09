#include "nrf24.h"
#include "spi.h"
#include "uart.h"
#define CSN_hi	   NRF24_PORT |=  (1<<CSN);
#define CSN_lo	   NRF24_PORT &= ~(1<<CSN);
#define CE_hi	   NRF24_PORT |=  (1<<CE);
#define CE_lo	   NRF24_PORT &= ~(1<<CE);

uint8_t nrf24_status = 0;
uint8_t nrf24_waiting_for_ack = 0;
uint8_t nrf24_p0_addr_save[5] = {};
uint8_t nrf24_p0_use_ack = 1;

void NRF24_set_tx_addr(uint8_t *addr);
enum NRF24_packetstatus nrf24_packetstatus = NRF24_NOT_SENT;
void NRF24_set_register(uint8_t reg, uint8_t val){
	/* W_REGISTER is "Executable in power down or standby modes
	   only." */
	CE_lo;
	CSN_lo;
	nrf24_status = SPI_Transmit(W_REGISTER | (REGISTER_MASK & reg));
	SPI_Transmit(val);
	CSN_hi;
	CE_hi;

}

uint8_t NRF24_get_register(uint8_t reg){
	CSN_lo;
	nrf24_status = SPI_Transmit(R_REGISTER | (REGISTER_MASK & reg));
	uint8_t val = SPI_Transmit(NOP);
	CSN_hi;
	return val;
}

enum NRF24_packetstatus NRF24_get_packetstatus(){
	return nrf24_packetstatus;
}

void NRF24_reset_packetstatus(){
        nrf24_packetstatus = NRF24_NOT_SENT;
}

void NRF24_set_bit(uint8_t reg, uint8_t bit){
	uint8_t r = NRF24_get_register(reg);
	r = r | (1 << bit);
	NRF24_set_register(reg, r);
}

void NRF24_clear_bit(uint8_t reg, uint8_t bit){
	uint8_t r = NRF24_get_register(reg);
	r = r & ~(1 << bit);
	NRF24_set_register(reg, r);
}

void NRF24_init(
		uint8_t use_interrupts, /* 0 or 1 */
		uint8_t use_crc /* 0, 1 or 2 bytes */
		){

	/* Set the CE and CSN to output */
	DDRB |= (1 << CE);
	DDRB |= (1 << CSN);
	SPI_init();
	uint8_t r =
		(use_interrupts   << MASK_RX_DR  )|
		(use_interrupts   << MASK_TX_DS  )|
		(use_interrupts   << MASK_MAX_RT )|
		((use_crc ? 1 : 0) << EN_CRC      )|
		((use_crc - 1)     << CRCO        )|
		(1 << PWR_UP      )| /* Power up */
		(1 << PRIM_RX     ); /* PRX mode */
	/* printf("CONFIG -> 0x%02hhX\n",r); */
	NRF24_set_register(CONFIG, r);

	/* Enable dynamic acks on all pipes */
	NRF24_set_bit(EN_AA, ENAA_P0);
	NRF24_set_bit(EN_AA, ENAA_P1);
	NRF24_set_bit(EN_AA, ENAA_P2);
	NRF24_set_bit(EN_AA, ENAA_P3);
	NRF24_set_bit(EN_AA, ENAA_P4);
	NRF24_set_bit(EN_AA, ENAA_P5);

	/* Disable all data pipes */
	NRF24_set_register(EN_RXADDR, 0x00);

	/* 5 byte addresses */
	NRF24_set_addr_w(5);

	/* Default to 1ms wait and 5 retransmits */
	NRF24_set_register(SETUP_RETR, (0x0F << ARD) | (0x0F << ARC));

	/* Deafult channel 2 */
	NRF24_set_channel(2);

	/* 250Kbit and 0dBm */
	NRF24_set_register(RF_SETUP ,0x27);

	/* Reset status register */
	NRF24_set_register(STATUS, 0x70);

	/* Default rx addr */
	uint8_t addr[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
	NRF24_set_rx_addr(0, addr);
	NRF24_set_register(RX_ADDR_P2, 0xC3);
	NRF24_set_register(RX_ADDR_P3, 0xC4);
	NRF24_set_register(RX_ADDR_P4, 0xC5);
	NRF24_set_register(RX_ADDR_P5, 0xC6);

	/* Default tx addr */
	NRF24_set_tx_addr(addr);

	/* Default pipe size */
	NRF24_set_pl_len(0,  0);
	NRF24_set_pl_len(1,  0);
	NRF24_set_pl_len(2,  0);
	NRF24_set_pl_len(3,  0);
	NRF24_set_pl_len(4,  0);
	NRF24_set_pl_len(5,  0);

	/* Disable dynamic payload size */
	NRF24_set_register(DYNPD, 0x00);

	/* Enable no ack packets (broadcast) */
	NRF24_set_bit(FEATURE, EN_DYN_ACK);

	/* Make sure everithing is empty */
	NRF24_flush_tx();
	NRF24_flush_rx();
	CE_hi;
}


void NRF24_set_channel(uint8_t channel){
	if(channel > 125){
		channel = 125;
	}
	NRF24_set_register(RF_CH, channel);
}
void NRF24_flush_tx(){
	CSN_lo;
	nrf24_status = SPI_Transmit(FLUSH_TX);
	CSN_hi;
}
void NRF24_get_status(){
	CSN_lo;
	nrf24_status = SPI_Transmit(NOP);
	CSN_hi;
}
void NRF24_flush_rx(){
	CSN_lo;
	nrf24_status = SPI_Transmit(FLUSH_RX);
	CSN_hi;
}
uint8_t NRF24_get_rx_size(){
	CSN_lo;
	nrf24_status = SPI_Transmit(R_RX_PLD_WID);
	uint8_t s = SPI_Transmit(NOP);
	CSN_hi;
	return s;
}
uint8_t NRF24_read_payload(uint8_t *pld, uint8_t *pld_len, uint8_t *pipe_nr){
	uint8_t len = NRF24_get_rx_size();
	if(pld_len != NULL){
		*pld_len = len;
	}
	/* Fetch the pipe number */
	uint8_t p_nr = (nrf24_status & (0x07 << RX_P_NO)) >> RX_P_NO;
	if(pipe_nr != NULL){
		*pipe_nr = p_nr;
	}
	/* printf("PIPE NR 0x%02hhX\n", p_nr); */
	if(p_nr == 0x07){
		/* Nothing to read */
		return 0;
	} else {
		CSN_lo;
		nrf24_status = SPI_Transmit(R_RX_PAYLOAD);
		for(uint8_t i = 0; i < len; i++){
			pld[i] = SPI_Transmit(NOP);
		}
		CSN_hi;
		return 1;
	}

}
void NRF24_get_addr(uint8_t *buffer, uint8_t pipe_nr){
	if(pipe_nr > 5) return;

	uint8_t aw = NRF24_get_register(SETUP_AW) + 2;

	CSN_lo;
	nrf24_status = SPI_Transmit(R_REGISTER | (REGISTER_MASK & (RX_ADDR_P0 + pipe_nr)));
	if(pipe_nr < 2){
		for(uint8_t i = 0; i < aw; i++){
			buffer[i] = SPI_Transmit(NOP);
		}
	} else {
		buffer[0] = SPI_Transmit(NOP);
	}
	CSN_hi;
}
void NRF24_set_speed(uint8_t speed){
	uint8_t s=0;
	switch (speed) {
	case NRF_1MBIT:
		{s = ((0 << RF_DR_LOW)|(0 << RF_DR_HIGH)); break; }
	case NRF_2MBIT:
		{s = ((0 << RF_DR_LOW)|(1 << RF_DR_HIGH)); break; }
	case NRF_250KBIT:
		{s = ((1 << RF_DR_LOW)|(0 << RF_DR_HIGH)); break; }
	}
	s = (NRF24_get_register(RF_SETUP) & ~((1 << RF_DR_LOW)|(1 << RF_DR_HIGH))) | s;
	NRF24_set_register(RF_SETUP, s);
}

void NRF24_set_pl_len(uint8_t pipe_nr, uint8_t len){
	if(pipe_nr > 5) return;
	NRF24_set_register(RX_PW_P0 + pipe_nr, len);
}

/* /\* ACK will be enabled for the pipe also. *\/ */
/* void NRF24_enable_dyn_pl(uint8_t pipe_nr){} */
/* void NRF24_disable_dyn_pl(uint8_t pipe_nr); */

/* void NRF24_enable_ack(uint8_t pipe_nr){ */
/* 	if(pipe_nr == 0){ */
/* 		nrf24_p0_use_ack = 1; */
/* 	} */
/* 	uint8_t r = NRF24_get_register(EN_AA); */
/* 	r = r | (1 << pipe_nr); */
/* 	NRF24_set_register(EN_AA, r); */
/* } */
/* void NRF24_disable_ack(uint8_t pipe_nr){ */
/* 	if(pipe_nr == 0){ */
/* 		nrf24_p0_use_ack = 0; */
/* 	} */
/* 	uint8_t r = NRF24_get_register(EN_AA); */
/* 	r = r & (~(1 << pipe_nr)); */
/* 	NRF24_set_register(EN_AA, r); */
/* } */

void NRF24_enable_pipe(uint8_t pipe_nr){
	uint8_t r = NRF24_get_register(EN_RXADDR);
	r = r | (1 << pipe_nr);
	NRF24_set_register(EN_RXADDR, r);
}
void NRF24_disable_pipe(uint8_t pipe_nr){
	uint8_t r = NRF24_get_register(EN_RXADDR);
	r = r & ~(1 << pipe_nr);
	NRF24_set_register(EN_RXADDR, r);
}

/* void NRF24_enable_dyn_ack(); */
/* void NRF24_disable_dyn_ack(); */

/* 3 to 5 bytes */
void NRF24_set_addr_w(uint8_t addr_w){
	addr_w -= 2;
	if(addr_w < 1){ addr_w = 1;}
	if(addr_w > 3){ addr_w = 3;}
	NRF24_set_register(SETUP_AW, addr_w);
}
void NRF24_set_rx_addr(uint8_t pipe_nr, uint8_t *addr){
	uint8_t aw = NRF24_get_register(SETUP_AW) + 2;
	CE_lo;
	CSN_lo;
	nrf24_status = SPI_Transmit(W_REGISTER | (REGISTER_MASK & (RX_ADDR_P0 + pipe_nr)));
	if(pipe_nr < 2){
		for(uint8_t i = 0; i < aw; i++){
			SPI_Transmit(addr[aw-i-1]);
		}
	} else {
		SPI_Transmit(*addr);
	}
	CSN_hi;
	CE_hi;
}
void NRF24_set_tx_addr(uint8_t *addr){
	uint8_t aw = NRF24_get_register(SETUP_AW) + 2;
	CSN_lo;
	nrf24_status = SPI_Transmit(W_REGISTER | (REGISTER_MASK & TX_ADDR));
	for(uint8_t i = 0; i < aw; i++){
		SPI_Transmit(addr[aw-i-1]);
	}
	CSN_hi;
}

uint8_t NRF24_send_packet(uint8_t *addr, uint8_t *payload,
			  uint8_t pl_length, uint8_t use_ack,
			  uint8_t sync){
	/* uint8_t aw = NRF24_get_register(SETUP_AW); */
	if(nrf24_status & (1 << TX_FULL) || nrf24_waiting_for_ack){
		/* No space for payload */
		printf("NO SPACE!\n");
		return 0;
	}
	/* Go to TX mode */
	CE_lo;

	/* Go to TX mode */
	NRF24_set_register(CONFIG, NRF24_get_register(CONFIG) & ~(1 << PRIM_RX));

	NRF24_enable_pipe(0);

	if(use_ack){
		NRF24_set_rx_addr(0, addr);
	}

	/* printf("tx -> %02hhX%02hhX%02hhX%02hhX%02hhX\n", addr[0],addr[1],addr[2],addr[3],addr[4]); */
	NRF24_set_tx_addr(addr);

	CSN_lo;
	if(use_ack){
		/* printf("NORMAL!\n"); */
		SPI_Transmit(W_TX_PAYLOAD);
	} else {
		SPI_Transmit(W_TX_PAYLOAD_NO_ACK);
	}
	for(uint8_t i = 0; i < pl_length; i++){
		SPI_Transmit(payload[i]);
	}
	CSN_hi;

	/* Transmit packet */
	CE_hi;
	_delay_us(15);
	CE_lo;

	if(use_ack){
		if(sync){
			/* Spin until ack or max rt */
			do {
				_delay_ms(1);
				NRF24_get_status();
			} while (nrf24_status & ((1<<MAX_RT)|(1<<TX_DS)));

			nrf24_packetstatus = NRF24_NOT_SENT;

			if(nrf24_status & (1<<MAX_RT)){
				/* No ack */
				return 0;
			} else {
				/* Ack */
				return 1;
			}

		} else {
			nrf24_waiting_for_ack = 1;
		}
	}

	nrf24_packetstatus = NRF24_IN_TRANSIT;

	return 1;
}

void NRF24_irq_handle(){

}
void NRF24_poll_handle(){
	NRF24_get_status();
	/* printf("STATUS -> %02hhX\n", nrf24_status); */
	if(nrf24_status & (1 << RX_DR)){
		/* A packet has been received */
	}
	if(nrf24_status & (1 << TX_DS)){
		/* A packet has been sent or acked */
		nrf24_waiting_for_ack = 0;
		nrf24_packetstatus = NRF24_SENT;

		/* Start listening again */
		NRF24_set_register(CONFIG, NRF24_get_register(CONFIG) | (1 << PRIM_RX));
		CE_hi;

	}
	if(nrf24_status & (1 << MAX_RT)){
		/* We are using acks and max retransmissions has been
		   reached */
		NRF24_flush_tx();


		nrf24_waiting_for_ack = 0;
		nrf24_packetstatus = NRF24_MAX_RT;

		/* Start listening again */
		NRF24_set_register(CONFIG, NRF24_get_register(CONFIG) | (1 << PRIM_RX));
		CE_hi;

	}
	/* Reset interrupts */
	NRF24_set_register(STATUS, 0x70);

}
void NRF24_power_up(){
	NRF24_set_register(CONFIG, NRF24_get_register(CONFIG) | (1 << PWR_UP));
}
void NRF24_power_down(){
	NRF24_set_register(CONFIG, NRF24_get_register(CONFIG) & ~(1 << PWR_UP));
}

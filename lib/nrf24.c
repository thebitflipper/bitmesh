#include "nrf24.h"
#include "spi.h"
#define CSN_hi	   NRF24_PORT |=  (1<<CSN);
#define CSN_lo	   NRF24_PORT &= ~(1<<CSN);
#define CE_hi	   NRF24_PORT |=  (1<<CE);
#define CE_lo	   NRF24_PORT &= ~(1<<CE);

uint8_t nrf24_status = 0;
uint8_t nrf24_waiting_for_ack = 0;
uint8_t nrf24_p0_addr_save[5] = {};
void NRF24_set_register(uint8_t reg, uint8_t val){
	CSN_lo;
	nrf24_status = SPI_Transmit(W_REGISTER | (REGISTER_MASK & reg));
	SPI_Transmit(val);
	CSN_hi;
}
uint8_t NRF24_get_register(uint8_t reg){
	CSN_lo;
	nrf24_status = SPI_Transmit(R_REGISTER | (REGISTER_MASK & reg));
	uint8_t val = SPI_Transmit(NOP);
	CSN_hi;
	return val;
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
	NRF24_set_register(CONFIG, r);
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
	uint8_t p_nr = (nrf24_status & (0x07 << RX_P_NO)) >> RX_P_NO;
	if(pipe_nr != NULL){
		*pipe_nr = p_nr;
	}

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

/* void NRF24_enable_ack(uint8_t pipe_nr); */
/* void NRF24_disable_ack(uint8_t pipe_nr); */

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
	uint8_t aw = NRF24_get_register(SETUP_AW);
	CSN_lo;
	nrf24_status = SPI_Transmit(W_REGISTER | (REGISTER_MASK & (RX_ADDR_P0 + pipe_nr)));
	if(pipe_nr < 2){
		for(uint8_t i = 0; i < aw; i++){
			SPI_Transmit(addr[i]);
		}
	} else {
		SPI_Transmit(addr[0]);
	}
	CSN_hi;
}
void NRF24_set_tx_addr(uint8_t *addr){
	uint8_t aw = NRF24_get_register(SETUP_AW);
	CSN_lo;
	nrf24_status = SPI_Transmit(W_REGISTER | (REGISTER_MASK & TX_ADDR));
	for(uint8_t i = 0; i < aw; i++){
		SPI_Transmit(addr[i]);
	}
	CSN_hi;
}

uint8_t NRF24_send_packet(uint8_t *addr, uint8_t *payload, uint8_t pl_length, uint8_t use_ack){
	/* uint8_t aw = NRF24_get_register(SETUP_AW); */
	if(nrf24_status & (1 << TX_FULL) || nrf24_waiting_for_ack){
		/* No space for payload */
		return 0;
	}
	/* Go to TX mode */
	CE_lo;
	NRF24_set_register(CONFIG, NRF24_get_register(CONFIG) & ~(1 << PRIM_RX));

	NRF24_get_addr(nrf24_p0_addr_save, 0);
	NRF24_set_rx_addr(0, addr);
	NRF24_set_tx_addr(addr);
	CSN_lo;
	if(use_ack){
		SPI_Transmit(W_TX_PAYLOAD);
	} else {
		SPI_Transmit(W_TX_PAYLOAD_NO_ACK);
	}
	for(uint8_t i = 0; i < pl_length; i++){
		SPI_Transmit(payload[i]);
	}
	CSN_hi;

	CE_hi;
	_delay_us(15);
	CE_lo;

	nrf24_waiting_for_ack = 1;


	return 1;
}

void NRF24_irq_handle(){

}
void NRF24_poll_handle(){
	NRF24_get_status();
	if(nrf24_status & (1 << RX_DR)){
		/* A packet has been received */
	}
	if(nrf24_status & (1 << TX_DS)){
		/* A packet has been sent or acked */
		nrf24_waiting_for_ack = 0;
		NRF24_set_rx_addr(0, nrf24_p0_addr_save);

		/* Start listening again */
		NRF24_set_register(CONFIG, NRF24_get_register(CONFIG) | (1 << PRIM_RX));
		CE_hi;

	}
	if(nrf24_status & (1 << MAX_RT)){
		/* We are using acks and max retransmissions has been
		   reached */
		nrf24_waiting_for_ack = 0;
		NRF24_set_rx_addr(0, nrf24_p0_addr_save);

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

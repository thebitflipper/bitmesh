#ifndef __nrf24_definition_h_
#define __nrf24_definition_h_

/* SPI Commands */
#define R_REGISTER	    0x00
#define W_REGISTER	    0x20
#define R_RX_PAYLOAD	    0x61
#define W_TX_PAYLOAD	    0xA0
#define FLUSH_TX	    0xE1
#define FLUSH_RX	    0xE2
#define REUSE_TX_PL	    0xE3
#define R_RX_PLD_WID	    0x60
#define W_ACK_PAYLOAD	    0xA8
#define W_TX_PAYLOAD_NO_ACK 0xB0
#define NOP		    0xff

/* Misc */
#define REGISTER_MASK 0x1F

/* Registers */
#define CONFIG	    0x00
#define EN_AA	    0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH	    0x05
#define RF_SETUP    0x06
#define STATUS	    0x07
#define OBSERVE_TX  0x08
#define RPD	    0x09
#define RX_ADDR_P0  0x0A	/* Max 5 bytes */
#define RX_ADDR_P1  0x0B	/* Max 5 bytes */
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR	    0x10	/* Max 5 bytes */
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1C
#define FEATURE	    0x1D

/* CONFIG mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC	    3
#define CRCO	    2
#define PWR_UP	    1
#define PRIM_RX	    0

/* EN_AA mnemonics */
#define ENAA_P5 5
#define ENAA_P4 4
#define ENAA_P3 3
#define ENAA_P2 2
#define ENAA_P1 1
#define ENAA_P0 0

/* EN_RXADDR mnemonics */
#define ENX_P5 5
#define ENX_P4 4
#define ENX_P3 3
#define ENX_P2 2
#define ENX_P1 1
#define ENX_P0 0

/* SETUP_AW mnemonics */
#define AW 0 			/* 2 bits */

/* SETUP_RETR mnemonics */
#define ARD 4			/* 4 bits */
#define ARC 0			/* 4 bits */

/* RF_CH mnemonics */
/* Not needed */

/* RF_SETUP mnemonics */
#define CONT_WAVE  7
#define RF_DR_LOW  5
#define PLL_LOCK   4
#define RF_DR_HIGH 3
#define RF_PWR     1		/* 2 bits */

/* STATUS mnemonics */
#define RX_DR   6
#define TX_DS   5
#define MAX_RT  4
#define RX_P_NO 1		/* 3 bits */
#define TX_FULL 0

/* OBSERVE_TX mnemonics */
#define PLOS_CNT 4		/* 4 bits */
#define ARC_CNT  0		/* 4 bits */

/* RPD mnemonics */
/* Not needed */

/* FIFO_STATUS mnemonics */
#define TX_REUSE 6
/* #define TX_FULL  5 (exists in STATUS) */
#define TX_EMPTY 4
#define RX_FULL  1
#define RX_EMPTY 0

/* DYDPD mnemonics */
#define DPL_P5 5
#define DPL_P4 4
#define DPL_P3 3
#define DPL_P2 2
#define DPL_P1 1
#define DPL_P0 0

/* FEATURE mnemonics */
#define EN_DPL	   2
#define EN_ACK_PAY 1
#define EN_DYN_ACK 0

#endif

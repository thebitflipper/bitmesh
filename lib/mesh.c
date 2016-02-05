#include "mesh.h"
#include "nrf24.h"
#include "uart.h"

#define MESH_ADDR_BROADCAST 0xFF
#define MESH_ADDR_NEW_DEVICE 0x00

enum MESH_STATE{
	MESH_STATE_UNCONNECTED,
	MESH_STATE_BRD_SENT,
	MESH_STATE_CONNECTED,
};

enum MESH_PACKET {
	MESH_PACKET_BRD_CONNECT = 0x00,
	MESH_PACKET_NEW_ROUTE   = 0x01, /* {_, a, b} -> b is reachable by a*/
	MESH_PACKET_DEL_ROUTE   = 0x02,
	MESH_PACKET_OFFER       = 0x03, /* Offer a route to a node */
};
enum MESH_STATE mesh_state = MESH_STATE_UNCONNECTED;
uint8_t mesh_buffer[16] = {0};
uint8_t mesh_recv_buffer[16] = {0};
uint8_t mesh_our_addr = 0;
uint8_t mesh_our_hopcount = 0;
unsigned long last_ms = 0;
unsigned long last_state_ms = 0;
void mesh_send(uint8_t addr){
	uint8_t addr_buffer[5] = ADDR;
	addr_buffer[4] = addr;
	printf("SEND(%d) -> 0x", addr_buffer[4]);
	for(int i = 0; i < 16; i++){
		printf("%02hhX", mesh_buffer[i]);
	}
	printf("\n");

	NRF24_send_packet(addr_buffer, mesh_buffer, 16, 1);
}

void mesh_make_route_packet(uint8_t addr){
	mesh_buffer[0] = (unsigned char)MESH_PACKET_NEW_ROUTE;
	mesh_buffer[1] = mesh_our_addr;
	mesh_buffer[2] = addr;
}


void mesh_send_broadcast(){
	uint8_t addr[5] = ADDR;
	addr[4] = MESH_ADDR_BROADCAST;
	NRF24_send_packet(addr, mesh_buffer, 16, 0);
}

void mesh_send_brd_connect(){
	mesh_buffer[0] = (unsigned char)MESH_PACKET_BRD_CONNECT;
	mesh_buffer[1] = mesh_our_addr;
	printf("Connecting (%d)\n", mesh_our_addr);
	mesh_state = MESH_STATE_BRD_SENT;
	mesh_send_broadcast();
}


void mesh_init(uint8_t is_sink){

	NRF24_init(1, 1);

	NRF24_set_pl_len(0, 16);
	NRF24_set_pl_len(1, 16);
	NRF24_set_pl_len(2,  0);
	NRF24_set_pl_len(3,  0);
	NRF24_set_pl_len(4,  0);
	NRF24_set_pl_len(5,  0);

	NRF24_set_addr_w(5);

	NRF24_disable_ack(0);
	NRF24_enable_ack(1);

	NRF24_set_channel(120);
	NRF24_set_speed(NRF_250KBIT);

	uint8_t addr[5] = ADDR;
	addr[4] = 0xff;
	NRF24_set_rx_addr(0, addr);

	if(is_sink){
		/* Set our address to the sink address. */
		addr[4] = 0x01;
		mesh_our_addr = 1;
	} else {
		/* Use the temporary "new device" address */
		addr[4] = 0x00;
	}

	NRF24_set_rx_addr(1, addr);

	/* Connect to the network */
	if(is_sink){
		mesh_state = MESH_STATE_CONNECTED;
	} else {
		mesh_send_brd_connect();
	}
}


uint8_t mesh_is_connected(){
	if(mesh_state == MESH_STATE_CONNECTED){
		return 1;
	} else {
		return 0;
	}
}
void mesh_poll(unsigned long ms){
	uint8_t a[5];
	printf("REG -> %02hhX\n", NRF24_get_register(0x03));
	NRF24_get_addr(a, 0);
	printf("0 -> %02hhX%02hhX%02hhX%02hhX%02hhX\n", a[0],a[1],a[2],a[3],a[4]);
	NRF24_get_addr(a, 1);
	printf("1 -> %02hhX%02hhX%02hhX%02hhX%02hhX\n", a[0],a[1],a[2],a[3],a[4]);
        last_ms = ms;
	NRF24_poll_handle();
	uint8_t pld_len;
	uint8_t pipe_nr;
	while(NRF24_read_payload(mesh_recv_buffer, &pld_len, &pipe_nr)){
		printf("D -> 0x");
		for(int i = 0; i < 16; i++){
			printf("%02hhX", mesh_recv_buffer[i]);
		}
		printf("\n");
		if(pipe_nr == 0){
			/* Broadcast packet */
			switch (mesh_recv_buffer[0]) {
			case MESH_PACKET_BRD_CONNECT: {
				printf("Node %d got BRD_CONNECT from %d\n",
				       mesh_our_addr,
				       mesh_recv_buffer[1]);
				mesh_buffer[0] = (unsigned char)MESH_PACKET_OFFER;
				mesh_buffer[1] = mesh_our_addr;
				mesh_buffer[2] = mesh_our_hopcount;
				mesh_send(mesh_recv_buffer[1]);
				break;
			}}
		} else {
			switch (mesh_recv_buffer[0]) {
			case MESH_PACKET_OFFER: {
				printf("Node %d got PACKET_OFFER from %d with hopc %d\n",
				      mesh_our_addr,
				      mesh_recv_buffer[1],
				      mesh_recv_buffer[2]);
				break;
			}}
			/* Unicast packet */
		}
	}
	unsigned long state_ms_diff = (ms - last_state_ms);
	switch (mesh_state) {
	case MESH_STATE_BRD_SENT: {
		if(state_ms_diff > 5000){
			/* Send request again */
			mesh_send_brd_connect();
			last_state_ms = ms;
			break;
		}
	}
	case MESH_STATE_CONNECTED: {

		break;
	}
	case MESH_STATE_UNCONNECTED: {
		/* Send stuff */
		break;
	}
	}
}

/* uint8_t mesh_send_data(uint8_t address, uint8_t *data){ */

/* } */

#include "mesh.h"
#include "nrf24.h"
#include "uart.h"

/* #define MESH_ADDR_BROADCAST  0xFF */
#define MESH_ADDR_BROADCAST  0xFF
#define MESH_ADDR_NEW_DEVICE 0x00
#define MESH_ADDR_SINK	     0x01

#define MESH_PIPE_BRD  5
#define MESH_PIPE_ADDR 1

#define MESH_MAX_NODES 32

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
	MESH_PACKET_PING        = 0x04
};

struct mesh_state {
	/* Last time mesh poll ran */
	unsigned long last_ms;
	/* Last time we changed connection state */
	unsigned long last_state_ms;
	/* Last ping time */
	unsigned long last_ping;
	/* If connected to mesh or not */
	enum MESH_STATE state;
	uint8_t tx_buffer[16];
	uint8_t rx_buffer[16];
	/* Our address */
	uint8_t addr;
	/* Our hopcount, 255 as default */
	uint8_t hopcount;
	/* Our parent, 0 if no parent */
	uint8_t parent;
	/* Routing table */
	uint8_t route[MESH_MAX_NODES];
};

struct mesh_state mesh;

void mesh_send(uint8_t addr){
	uint8_t addr_buffer[5] = ADDR;
	addr_buffer[4] = addr;
	printf("SEND(%d) -> 0x", addr_buffer[4]);
	for(int i = 0; i < 16; i++){
		printf("%02hhX", mesh.tx_buffer[i]);
	}
	printf("\n");

	NRF24_send_packet(addr_buffer, mesh.tx_buffer, 16, 1);
}

void mesh_send_ping(uint8_t addr){
	mesh.tx_buffer[0] = (unsigned char)MESH_PACKET_PING;
	/* From */
	mesh.tx_buffer[1] = mesh.addr;
	/* To */
	mesh.tx_buffer[2] = addr;

	uint8_t addr_c[5] = ADDR;
        addr_c[4] = mesh.parent;
        /* addr_c[4] = mesh.parent; */
	NRF24_send_packet(addr_c, mesh.tx_buffer, 16, 1);
}

void mesh_publish_route(uint8_t addr){
	mesh.tx_buffer[0] = (unsigned char)MESH_PACKET_NEW_ROUTE;
	/* Through node (this node): */
	mesh.tx_buffer[1] = mesh.addr;
	/* you can reach: */
	mesh.tx_buffer[2] = addr;

	uint8_t parent[5] = ADDR;
	parent[4] = mesh.parent;
	NRF24_send_packet(parent, mesh.tx_buffer, 16, 0);
}


void mesh_send_brd_connect(){
	mesh.tx_buffer[0] = (unsigned char)MESH_PACKET_BRD_CONNECT;
	mesh.tx_buffer[1] = mesh.addr;
	uint8_t addr[5] = ADDR;
	addr[4] = MESH_ADDR_BROADCAST;
	NRF24_send_packet(addr, mesh.tx_buffer, 16, 0);
	mesh.state = MESH_STATE_BRD_SENT;

}

void mesh_init(uint8_t is_sink){

	mesh.last_ms = 0;
	mesh.last_state_ms = 0;
	mesh.state = MESH_STATE_UNCONNECTED;
	mesh.hopcount = 255;
	mesh.parent = 0;

	for (int i = 0; i < MESH_MAX_NODES; i++){
		mesh.route[0] = 0;
	}

	NRF24_init(0, 1);

	NRF24_set_pl_len(MESH_PIPE_ADDR, 16);
	NRF24_set_pl_len(MESH_PIPE_BRD, 16);
	/* NRF24_set_pl_len(0, 16); */

	/* We use pipe 1 for our address */
	NRF24_enable_pipe(MESH_PIPE_ADDR);
	/* And pipe 5 for the broadcast address */
	NRF24_enable_pipe(MESH_PIPE_BRD);
	/* NRF24_enable_pipe(0); */

	NRF24_set_channel(120);
	NRF24_set_speed(NRF_250KBIT);


	if(is_sink){
		/* Set our address to the sink address. */
		mesh.addr = MESH_ADDR_SINK;
	} else {
		/* Use the temporary "new device" address */
		mesh.addr = MESH_ADDR_NEW_DEVICE;
	}

	uint8_t addr[5] = ADDR;
	addr[4] = mesh.addr;
	NRF24_set_rx_addr(MESH_PIPE_ADDR, addr);

	uint8_t brd_addr = MESH_ADDR_BROADCAST;
	NRF24_set_rx_addr(MESH_PIPE_BRD, &brd_addr);


	/* Connect to the network */
	if(is_sink){
		mesh.state = MESH_STATE_CONNECTED;
		mesh.hopcount = 0;
	} else {
		mesh_send_brd_connect();
	}
}

uint8_t mesh_is_connected(){
	if(mesh.state == MESH_STATE_CONNECTED){
		return 1;
	} else {
		return 0;
	}
}

void mesh_poll(unsigned long ms){
	/* uint8_t a[5]; */
	/* uint8_t r = NRF24_get_register(0x07); */
	/* if ( r != 0x0E ){ */
	/* printf("REG -> %02hhX\n", r);; */
		/* NRF24_get_addr(a, 0); */
		/* printf("0 -> %02hhX%02hhX%02hhX%02hhX%02hhX\n", a[0],a[1],a[2],a[3],a[4]); */
		/* NRF24_get_addr(a, 1); */
		/* printf("1 -> %02hhX%02hhX%02hhX%02hhX%02hhX\n", a[0],a[1],a[2],a[3],a[4]); */
		/* NRF24_get_addr(a, 5); */
		/* printf("5 -> %02hhX\n", a[0]); */
	/* } */
	/* printf("POWER -> %02hhX RF -> %02hhX FIFO -> %02hhX FEA -> %02hhX ENAA -> %02hhX \n", */
	/*        NRF24_get_register(RPD), */
	/*        NRF24_get_register(RF_SETUP), */
	/*        NRF24_get_register(FIFO_STATUS), */
	/*        NRF24_get_register(FEATURE), */
	/*        NRF24_get_register(EN_AA) */
	/*        ); */
	/* printf("REG -> %02hhX\n", NRF24_get_register(0x06)); */
	/* printf("REG -> %02hhX\n", NRF24_get_register(RPD)); */
	/* printf("REG -> %02hhX\n", NRF24_get_register(0x08)); */
	/* printf("REG -> %02hhX\n", NRF24_get_register(0x09)); */
        mesh.last_ms = ms;
	NRF24_poll_handle();

	switch (NRF24_get_packetstatus()) {
	case NRF24_MAX_RT : {
		printf("NO ack received\n");
		NRF24_reset_packetstatus();
		break;
	}
	case NRF24_SENT : {
		printf("Packet sent ---------------\n");
		NRF24_reset_packetstatus();
		break;
	}
	default:{}
	}
	uint8_t pld_len;
	uint8_t pipe_nr;
	while(NRF24_read_payload(mesh.rx_buffer, &pld_len, &pipe_nr)){
		printf("D(%d) -> 0x",pipe_nr);
		for(int i = 0; i < 16; i++){
			printf("%02hhX", mesh.rx_buffer[i]);
		}
		printf("\n");
		if(pipe_nr == MESH_PIPE_BRD){
			/* Broadcast packet */
			switch (mesh.rx_buffer[0]) {
			case MESH_PACKET_BRD_CONNECT: {
				printf("Node %d got BRD_CONNECT from %d\n",
				       mesh.addr,
				       mesh.rx_buffer[1]);
				/* Respond */
				mesh.tx_buffer[0] = (unsigned char)MESH_PACKET_OFFER;
				mesh.tx_buffer[1] = mesh.addr;
				mesh.tx_buffer[2] = mesh.hopcount;
				mesh_send(mesh.rx_buffer[1]);
				break;
			}}
		} else {
			/* Unicast packet */
			switch (mesh.rx_buffer[0]) {
			case MESH_PACKET_OFFER: {
				printf("Node %d got PACKET_OFFER from %d with hopc %d\n",
				      mesh.addr,
				      mesh.rx_buffer[1],
				      mesh.rx_buffer[2]);
				if(mesh.state == MESH_STATE_BRD_SENT){
					printf("Accepting route!\n");
					/* Connect to the node */
					mesh.parent = mesh.rx_buffer[1];
					mesh.hopcount = mesh.rx_buffer[2] + 1;
					mesh_publish_route(mesh.addr);
					mesh.state = MESH_STATE_CONNECTED;
					mesh.last_state_ms = ms;
				}
				break;
			}
			case MESH_PACKET_PING: {
				printf("Node %d got PING from %d to %d\n",
				       mesh.addr,
				       mesh.rx_buffer[1],
				       mesh.rx_buffer[2]);
				if(mesh.rx_buffer[2] != mesh.addr){
					/* Ping not for us! */
					printf("Ping not for us!\n");
				}
				break;
			}
			case MESH_PACKET_NEW_ROUTE: {
				printf("We can reach %d through %d\n",
				       mesh.rx_buffer[1],
				       mesh.rx_buffer[2]);
				mesh.route[mesh.rx_buffer[1]] = mesh.rx_buffer[2];
				break;
			}
			}
		}
	}
	unsigned long state_ms_diff = (ms - mesh.last_state_ms);
	switch (mesh.state) {
	case MESH_STATE_BRD_SENT: {
		if(state_ms_diff > 5000){
			/* Send request again */
			mesh_send_brd_connect();
			/* mesh_send_ping(0x01); */
			printf("Requesting connection!\n");
			/* uint8_t addr[5] = ADDR; */
			/* addr[4] = 0x01; */
			/* NRF24_send_packet(addr, mesh_buffer, 16, 1); */
			mesh.last_state_ms = ms;
		}
		break;
	}
	case MESH_STATE_CONNECTED: {
		if(mesh.addr != MESH_ADDR_SINK && (ms - mesh.last_ping) > 5000){
			printf("Send Ping\n");
			mesh_send_ping(MESH_ADDR_SINK);
			mesh.last_ping = ms;
		}
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

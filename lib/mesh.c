#include "mesh.h"
#include "nrf24.h"
#include "uart.h"
#include "includes.h"
#include <string.h>
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
	MESH_STATE_W_FOR_ADDRESS,
	MESH_STATE_COLLECTING_PARENTS,
};

enum MESH_PACKET {
	/* BROADCAST */
	MESH_PACKET_BRD_CONNECT = 0x00,
	/* Broadcas only, 1 = source addr */

	/* UNICAST */
	MESH_PACKET_NEW_ROUTE   = 0x01,
	/* 1 = source addr, 2 = dest addr, 3 = current sender,
	   4 = node, 5 = nodes parent */
	MESH_PACKET_DEL_ROUTE   = 0x02,
	MESH_PACKET_OFFER       = 0x03,
	/* 1 = source addr, 2 = dest addr, 3 = current sender,
	   4 = hopcount(source), */
	MESH_PACKET_PING        = 0x04,
	/* 1 = source addr, 2 = dest addr, 3 = current sender */
	MESH_PACKET_PONG        = 0x05,
	/* 1 = source addr, 2 = dest addr, 3 = current sender */
	MESH_PACKET_CHANGE_ADDR = 0x06,
	/* 1 = source addr, 2 = dest addr, 3 = current sender, 3 new addr */
	MESH_PACKET_REQ_ADDR    = 0x07
	/* 1 = source addr, 2 = dest addr, 3 = current sender */
};

/* These defines are used for debugging */
#define fmt16 "0x%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX"
#define tx16 mesh.tx_buffer[0],mesh.tx_buffer[1],mesh.tx_buffer[2],mesh.tx_buffer[3],mesh.tx_buffer[4],mesh.tx_buffer[5],mesh.tx_buffer[6],mesh.tx_buffer[7],mesh.tx_buffer[8],mesh.tx_buffer[9],mesh.tx_buffer[10],mesh.tx_buffer[11],mesh.tx_buffer[12],mesh.tx_buffer[13],mesh.tx_buffer[14],mesh.tx_buffer[15]
#define rx16 mesh.rx_buffer[0],mesh.rx_buffer[1],mesh.rx_buffer[2],mesh.rx_buffer[3],mesh.rx_buffer[4],mesh.rx_buffer[5],mesh.rx_buffer[6],mesh.rx_buffer[7],mesh.rx_buffer[8],mesh.rx_buffer[9],mesh.rx_buffer[10],mesh.rx_buffer[11],mesh.rx_buffer[12],mesh.rx_buffer[13],mesh.rx_buffer[14],mesh.rx_buffer[15]

struct potential_parent {
	unsigned long last_changed;
	unsigned long parent;
	unsigned long hopcount;
};

struct mesh_state {
	/* Last time mesh poll ran */
	unsigned long last_ms;
	/* Last time we changed connection state */
	unsigned long last_state_ms;
	/* Last ping time */
	unsigned long last_ping;
	/* Last time we asked for a new address */
	unsigned long last_addr;
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
	/* We store the potential parents in this struct */
	struct potential_parent pot;
};

struct mesh_state mesh;
uint8_t new_address = 2;

uint8_t mesh_get_parent(uint8_t addr){
	return mesh.route[addr];
}

uint8_t mesh_next_hop(uint8_t target_addr){
	/* TODO: Make sure this does not hang */
	D("Calculating next hop for %d\n", target_addr);
	if(target_addr == MESH_ADDR_SINK){
		/* The target is the sink, send up */
		return mesh.parent;
	}



	/* Check if the target is below us */
	uint8_t prev = target_addr;
	uint8_t up   = mesh_get_parent(target_addr);
	if(up == 0){
		/* This is a temporary address. No one should have 0
		   as a parent */
		return 255;
	}

	while (up != 255){
		D("PREV %d UP %d\n", prev, up);
		if(up == mesh.addr){
			return prev;
		}
		prev = up;
		up = mesh_get_parent(up);
	}

	/* The target is not below us and we are the sink */
	if (mesh.addr == MESH_ADDR_SINK){
		D("NOT ROUTABLE!!!!!\n");
		return 255;
	} else {
		return mesh.parent;
	}
}

uint8_t mesh_send(uint8_t addr, uint8_t sync){
	/* Calculate next hop */
	uint8_t next_hop = mesh_next_hop(addr);

	if(next_hop != 255){
		uint8_t addr_buffer[5] = ADDR;
		addr_buffer[4] = next_hop;
		D("SEND(%d, n%2d, f%2d) "fmt16"\n", mesh.addr, next_hop, addr, tx16);
		return NRF24_send_packet(addr_buffer, mesh.tx_buffer, 16, 1, sync);
	}
	return 0;
}

uint8_t mesh_send_direct(uint8_t addr, uint8_t sync){
	/* Calculate next hop */

	uint8_t addr_buffer[5] = ADDR;
	addr_buffer[4] = addr;
	D("SEND(n%2d, f%2d) "fmt16"\n", addr, addr, tx16);
	return NRF24_send_packet(addr_buffer, mesh.tx_buffer, 16, 1, sync);
}

void mesh_send_ping(uint8_t addr){
	mesh.tx_buffer[0] = (unsigned char)MESH_PACKET_PING;
	/* From */
	mesh.tx_buffer[1] = mesh.addr;
	/* To */
	mesh.tx_buffer[2] = addr;
	/* This node */
	mesh.tx_buffer[3] = mesh.addr;

	mesh_send(addr, 1);
}

void mesh_send_pong(uint8_t addr){
	mesh.tx_buffer[0] = (unsigned char)MESH_PACKET_PONG;
	/* From */
	mesh.tx_buffer[1] = mesh.addr;
	/* To */
	mesh.tx_buffer[2] = addr;
	/* This node */
	mesh.tx_buffer[3] = mesh.addr;

	mesh_send(addr, 1);
}

uint8_t mesh_publish_route(uint8_t addr, uint8_t node, uint8_t new_parent){
	mesh.tx_buffer[0] = (unsigned char)MESH_PACKET_NEW_ROUTE;

	mesh.tx_buffer[1] = mesh.addr;
	mesh.tx_buffer[2] = addr;

	mesh.tx_buffer[3] = node;
	mesh.tx_buffer[4] = new_parent;

	return mesh_send(addr,1);
}


void mesh_send_brd_connect(uint8_t change_state){
	mesh.tx_buffer[0] = (unsigned char)MESH_PACKET_BRD_CONNECT;
	mesh.tx_buffer[1] = mesh.addr;
	uint8_t addr[5] = ADDR;
	addr[4] = MESH_ADDR_BROADCAST;
	NRF24_send_packet(addr, mesh.tx_buffer, 16, 0, 0);
	if(change_state){
		mesh.state = MESH_STATE_BRD_SENT;
	}
}

void mesh_init(uint8_t is_sink){

	mesh.last_ms = 0;
	mesh.last_addr = 0;
	mesh.last_state_ms = 0;
	mesh.state = MESH_STATE_UNCONNECTED;
	mesh.hopcount = 255;
	mesh.parent = 0;

	for (int i = 0; i < MESH_MAX_NODES; i++){
		mesh.route[i] = 255;
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
		mesh_send_brd_connect(1);
	}
}

uint8_t mesh_is_connected(){
	if(mesh.state == MESH_STATE_CONNECTED){
		return 1;
	} else {
		return 0;
	}
}

void mesh_route_dump(){
	for (int i = 0; i < MESH_MAX_NODES; i++){
		printf("-R %d %d\n", i, mesh.route[i]);
	}
}

void mesh_poll(unsigned long ms){
	/* D("POLL\n"); */
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
		D("NO ack received +++++++++\n");
		NRF24_reset_packetstatus();
		break;
	}
	case NRF24_SENT : {
	        D("Packet sent--------------\n");
		NRF24_reset_packetstatus();
		break;
	}
	default:{}
	}
	uint8_t pld_len;
	uint8_t pipe_nr;
	while(NRF24_read_payload(mesh.rx_buffer, &pld_len, &pipe_nr)){
		D("D(%d) -> 0x" fmt16 "\n",pipe_nr, rx16);
		if(pipe_nr == MESH_PIPE_BRD){
			/* Broadcast packet */
			switch (mesh.rx_buffer[0]) {
			case MESH_PACKET_BRD_CONNECT: {
				D("Node %d got BRD_CONNECT from %d\n",
				  mesh.addr,
				  mesh.rx_buffer[1]);
				/* Respond */
				mesh.tx_buffer[0] = (unsigned char)MESH_PACKET_OFFER;
				mesh.tx_buffer[1] = mesh.addr;
				mesh.tx_buffer[2] = mesh.rx_buffer[1];
				mesh.tx_buffer[3] = mesh.addr;
				mesh.tx_buffer[4] = mesh.hopcount;
				mesh_send_direct(mesh.rx_buffer[1], 1);
				D("Sent offer to %d\n", mesh.rx_buffer[1]);
				break;
			}}
		} else {

			if (mesh.rx_buffer[3] == mesh.parent){
				/* Packet is sent from parent */
			} else if (mesh_get_parent(mesh.rx_buffer[3]) == mesh.addr){
				/* Message is sent from a direct child */
			} else {
				/* Sender is not parent or direct
				   child. Perhaps we lost connection
				   to this node earlier. Add us as the
				   nodes parent. */
				mesh.route[mesh.rx_buffer[3]] = mesh.addr;
				/* TODO: Perhaps we should ask the
				   node to inform us of its children?
				   But did we remove its children also
				   from the routing table? */
			}

			uint8_t for_us = 1;
			/* Unicast packet */
			if (mesh.rx_buffer[2] != mesh.addr){
				/* Packets final destination is not
				   for us. */
				for_us = 0;
				D("Forwarding packet\n");
				memcpy(mesh.tx_buffer, mesh.rx_buffer, 16);

				/* Set our address */
				mesh.tx_buffer[3] = mesh.addr;

				/* Try sending the packet */
				uint8_t forwarded = 0;
				for (int i = 0; i< 10; ++i){
					if(mesh_send(mesh.rx_buffer[2], 1)){
						forwarded=1;
						break;
					}
					_delay_ms(3);
				}

				if(!forwarded){
					D("Lost connection to %d!\n", mesh_next_hop(mesh.rx_buffer[2]));
					if(mesh.rx_buffer[2] == mesh.parent){
						/* We lost connection to the mesh */
						mesh.state = MESH_STATE_UNCONNECTED;
					}
				}
			}

			switch (mesh.rx_buffer[0]) {
			case MESH_PACKET_OFFER: {
				/* Only accept new offers when we sent
				   brodcast or collecting parents,
				   basically the same state.  */
				if(!(mesh.state == MESH_STATE_BRD_SENT ||
				     mesh.state == MESH_STATE_COLLECTING_PARENTS)) break;

				D("Node %d got PACKET_OFFER from %d with hopc %d\n",
				  mesh.addr,
				  mesh.rx_buffer[1],
				  mesh.rx_buffer[4]);
				if(mesh.state == MESH_STATE_BRD_SENT){
					mesh.pot.parent = mesh.rx_buffer[1];
					mesh.pot.hopcount = mesh.rx_buffer[4];
					mesh.pot.last_changed = ms;
					mesh.state = MESH_STATE_COLLECTING_PARENTS;
				} else if (mesh.state == MESH_STATE_COLLECTING_PARENTS){
					if(mesh.rx_buffer[4] < mesh.pot.hopcount){
						D("This parent is better!\n");
						/* We found a better parent */
						mesh.pot.parent = mesh.rx_buffer[1];
						mesh.pot.hopcount = mesh.rx_buffer[4];
						mesh.pot.last_changed = ms;
					}
				}
				break;
			}
			case MESH_PACKET_PING: {
				/* Only accept ping when connected */
				if(mesh.state != MESH_STATE_CONNECTED) break;
				D("Node %d got PING from %d to %d\n",
				  mesh.addr,
				  mesh.rx_buffer[1],
				  mesh.rx_buffer[2]);
				if(for_us){
					D("Sending pong\n");
					mesh_send_pong(mesh.rx_buffer[1]);
				} else {
					/* Ping not for us! */
					D("Ping not for us!\n");
				}
				break;
			}
			case MESH_PACKET_PONG: {
				/* Only accept ping when connected */
				if(mesh.state != MESH_STATE_CONNECTED) break;
				D("Node %d got PONG from %d to %d\n",
				  mesh.addr,
				  mesh.rx_buffer[1],
				  mesh.rx_buffer[2]);
				if(!for_us){
					/* Pong not for us! */
					D("Pong not for us!\n");
				}
				break;
			}
			case MESH_PACKET_NEW_ROUTE: {
				/* Only accept new routes when connected */
				if(mesh.state != MESH_STATE_CONNECTED) break;
				/* Check if packet is for us */
				if(mesh.addr  !=    mesh.rx_buffer[2]) break;

				D("New route from %hhd to %hhd. Node %hhd has parent %hhd\n",
				  mesh.rx_buffer[1],
				  mesh.rx_buffer[2],
				  mesh.rx_buffer[3],
				  mesh.rx_buffer[4]);
				mesh.route[mesh.rx_buffer[3]] = mesh.rx_buffer[4];
				if(mesh.addr != MESH_ADDR_SINK){
					/* We are not the sink. Forward the route upwards */
					mesh_publish_route(mesh.parent, mesh.rx_buffer[3], mesh.rx_buffer[4]);
				}

				mesh_route_dump();
				break;
			}
			case MESH_PACKET_CHANGE_ADDR: {
				if (mesh.rx_buffer[2] != mesh.addr) break;
				/* This packet is for us */
				D("Our address is now %d\n", mesh.rx_buffer[3]);
				mesh.addr = mesh.rx_buffer[3];
				mesh_publish_route(mesh.parent, mesh.addr, mesh.parent);
				mesh.state = MESH_STATE_CONNECTED;
				break;
			}
			case MESH_PACKET_REQ_ADDR: {
				if (mesh.rx_buffer[2] != mesh.addr &&
				    mesh.addr != MESH_ADDR_SINK) break;
				/* This packet is for us and we are the sink */
				D("We give node %d the new address %d\n", mesh.rx_buffer[1], new_address);

				mesh.tx_buffer[0] = MESH_PACKET_CHANGE_ADDR;
				mesh.tx_buffer[1] = mesh.addr;
				mesh.tx_buffer[2] = mesh.rx_buffer[1];
				mesh.tx_buffer[3] = new_address++;

				mesh_send(mesh.tx_buffer[2], 1);
				break;
			}
			}
		}
	}
	unsigned long state_ms_diff = (ms - mesh.last_state_ms);
	switch (mesh.state) {
	case MESH_STATE_BRD_SENT: {
		if(state_ms_diff > 2000){
			/* Send request again */
			mesh_send_brd_connect(1);
			mesh.last_state_ms = ms;
		}
		break;
	}
	case MESH_STATE_CONNECTED: {
		if(mesh.addr != MESH_ADDR_SINK &&
		   (ms - mesh.last_ping) > 5000){
			D("Send Ping\n");
			mesh_send_ping(MESH_ADDR_SINK);
			mesh.last_ping = ms;
		}
		break;
	}
	case MESH_STATE_UNCONNECTED: {
		/* Send stuff */
		mesh_send_brd_connect(1);
		mesh.last_state_ms = ms;
		break;
	}
	case MESH_STATE_W_FOR_ADDRESS: {
		if((ms - mesh.last_addr) > 10000){
			/* New address timeout! */
			mesh.tx_buffer[0] = MESH_PACKET_REQ_ADDR;
			mesh.tx_buffer[1] = mesh.addr;
			mesh.tx_buffer[2] = MESH_ADDR_SINK;
			mesh.tx_buffer[3] = mesh.addr;
			if(mesh_send(MESH_ADDR_SINK, 1)){
				/* Packet is away! */
				D("Send request for new address!\n");
				mesh.last_addr = ms;
			} else {
				D("Failed to send request for new address!\n");
				/* TODO: We failed to send to parent node, maybe do something */
			}
		}

		/* Send stuff */
		break;
	}
	case MESH_STATE_COLLECTING_PARENTS: {
		if((ms - mesh.pot.last_changed) > 5000){
			D("Trying to accept route!\n");
			/* We have waited long enough for potential
			   parents, connect to the best one. */
			if (mesh_publish_route(mesh.pot.parent,
					       mesh.addr,
					       mesh.pot.parent)){

				mesh.parent = mesh.pot.parent;
				mesh.hopcount = mesh.pot.hopcount + 1;
				mesh.last_state_ms = ms;
				D("Connected to %d!\n",mesh.parent);

				if (mesh.addr == MESH_ADDR_NEW_DEVICE){
					/* We are using the temporary new device address */
					mesh.state = MESH_STATE_W_FOR_ADDRESS;
				} else {
					mesh.state = MESH_STATE_CONNECTED;
				}
			} else {
				D("Failed to connect!\n");
			}
		} else if ((ms - mesh.last_state_ms) > 500){
			D("Broadcast\n");
			mesh_send_brd_connect(0);
			mesh.last_state_ms = ms;
		}
	}
	}
}

/* uint8_t mesh_send_data(uint8_t address, uint8_t *data){ */

/* } */

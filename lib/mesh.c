#include "mesh.h"
#include "nrf24.h"
#include "uart.h"
#include "includes.h"
#include <string.h>
#include <avr/eeprom.h>

#ifndef cb
    #define cb(reg, bit)    reg &= ~(1<<bit) //clear bit
#endif

#ifndef sb
    #define sb(reg, bit)    reg |= (1<<bit) //set bit
#endif

#ifndef rb
    #define rb(reg, bit)    (reg & (1<<bit)) //read bit
#endif
/* #define MESH_ADDR_BROADCAST  0xFF */
#define MESH_ADDR_BROADCAST  0xFF
#define MESH_ADDR_NEW_DEVICE 0x00
#define MESH_ADDR_SINK	     0x01

#define MESH_PIPE_BRD  5
#define MESH_PIPE_ADDR 1

#define MESH_MAX_NODES 32
#define MESH_MAX_CHILDREN 2

#define MESH_LOSS_THRESHOLD 3

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
	/* 1 = source addr, 2 = dest addr, 3 = current sender,
	   4 = node, 5 = nodes old parent */
	MESH_PACKET_OFFER       = 0x03,
	/* 1 = source addr, 2 = dest addr, 3 = current sender,
	   4 = hopcount(source), */
	MESH_PACKET_PING        = 0x04,
	/* 1 = source addr, 2 = dest addr, 3 = current sender */
	MESH_PACKET_PONG        = 0x05,
	/* 1 = source addr, 2 = dest addr, 3 = current sender */
	MESH_PACKET_CHANGE_ADDR = 0x06,
	/* 1 = source addr, 2 = dest addr, 3 = current sender, 4 new addr */
	MESH_PACKET_REQ_ADDR    = 0x07,
	/* 1 = source addr, 2 = dest addr, 3 = current sender */
	MESH_PACKET_ALIVE_REQ   = 0x08,
	/* 1 = source addr, 2 = dest addr, 3 = current sender,the rest
	   of the bytes will be copied into the reply */
	MESH_PACKET_ALIVE_RSP   = 0x09,
	/* 1 = source addr, 2 = dest addr, 3 = current sender, the
	   rest of the bytes are the ones sent in the req */
	MESH_PACKET_NEW_HOPCOUNT= 0x0A,
	/* 1 = source addr, 2 = dest addr, 3 = current sender, 4 the
	   hopcount of the source node */
	MESH_PACKET_NOP         = 0x0B
	/* 1 = source addr, 2 = dest addr, 3 = current sender, this
	   packet does nothing on the dest node */
};

/* These defines are used for debugging */
#define fmt16 "0x%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX"
#define tx16 mesh.tx_buffer[0],mesh.tx_buffer[1],mesh.tx_buffer[2],mesh.tx_buffer[3],mesh.tx_buffer[4],mesh.tx_buffer[5],mesh.tx_buffer[6],mesh.tx_buffer[7],mesh.tx_buffer[8],mesh.tx_buffer[9],mesh.tx_buffer[10],mesh.tx_buffer[11],mesh.tx_buffer[12],mesh.tx_buffer[13],mesh.tx_buffer[14],mesh.tx_buffer[15]
#define rx16 mesh.rx_buffer[0],mesh.rx_buffer[1],mesh.rx_buffer[2],mesh.rx_buffer[3],mesh.rx_buffer[4],mesh.rx_buffer[5],mesh.rx_buffer[6],mesh.rx_buffer[7],mesh.rx_buffer[8],mesh.rx_buffer[9],mesh.rx_buffer[10],mesh.rx_buffer[11],mesh.rx_buffer[12],mesh.rx_buffer[13],mesh.rx_buffer[14],mesh.rx_buffer[15]

void mesh_route_dump();
uint8_t mesh_is_child(uint8_t node);
uint8_t mesh_publish_del_route(uint8_t addr, uint8_t node, uint8_t old_parent);

struct potential_parent {
	unsigned long last_changed;
	uint8_t parent;
	uint8_t hopcount;
};

struct mesh_state {
	/* Last time mesh poll ran */
	unsigned long last_ms;
	/* Last time we changed connection state */
	unsigned long last_state_ms;
	/* Last ping time */
	unsigned long last_ping;
	/* Last pong time (from sink) */
	unsigned long last_pong;
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
	uint8_t packetloss[MESH_MAX_NODES];
	/* We store the potential parents in this struct */
	struct potential_parent pot;
};

uint8_t *mesh_eeprom_addr      = (uint8_t*)0x00;
/* uint8_t *mesh_eeprom_node_info = (uint8_t*)0x01; */

/* TODO: STore free addresses in eeprom */
#define MESH_NODE_FREE_ADDR (uint8_t*)0x01

struct mesh_state mesh;
/* uint8_t new_address = 2; */

uint8_t mesh_get_parent(uint8_t addr){
	return mesh.route[addr];
}

void mesh_mark_node_taken(uint8_t node){
	eeprom_busy_wait();
	uint8_t a = eeprom_read_byte(MESH_NODE_FREE_ADDR+node);
	if(rb(a,0)){
		/* This address is not marked taken */
		eeprom_busy_wait();
		eeprom_update_byte(MESH_NODE_FREE_ADDR+node, cb(a, 0));
	}
}

/* Returns 0 if failure */
uint8_t mesh_generate_address(){
	for(int i = 2; i < MESH_MAX_NODES; i++){
		eeprom_busy_wait();
		uint8_t a = eeprom_read_byte(MESH_NODE_FREE_ADDR+i);
		if(rb(a,0)){
			/* This address is not used */
			eeprom_busy_wait();
			eeprom_update_byte(MESH_NODE_FREE_ADDR+i, cb(a, 0));
			return i;
		} else {
			/* This address is used */
		}

	}
	return 0;
}

uint8_t mesh_route_goes_to_sink(uint8_t node){
	while(!(node == 255 || node == MESH_ADDR_SINK)){
	        node = mesh_get_parent(node);
	}

	if(node == MESH_ADDR_SINK){
		return 1;
	} else {
		return 0;
	}
}

uint8_t mesh_is_above(uint8_t a, uint8_t b){
	uint8_t node = b;
	while(!(node == 255 || node == MESH_ADDR_SINK || node == a)){
	        node = mesh_get_parent(node);
	}
	if(node == a){
		return 1;
	} else {
		return 0;
	}
}

void mesh_route_update(uint8_t node, uint8_t parent){
	if(node    < MESH_MAX_NODES &&
	   (parent  < MESH_MAX_NODES || parent == 255) &&
	   parent != MESH_ADDR_NEW_DEVICE &&
	   !mesh_is_above(node, parent)){
		mesh.route[node] = parent;
	} else {
		/* TODO This should not happen, handle it */
	}

	if(mesh.addr == MESH_ADDR_SINK){
		mesh_route_dump();
	}

	mesh_mark_node_taken(node);
	mesh_mark_node_taken(parent);

	if(parent == 255){
		for(int i = 0; i < MESH_MAX_NODES; i++){
			if(!mesh_route_goes_to_sink(i)){
				mesh.route[i] = 255;
			}
		}
	}
}

uint8_t mesh_next_hop(uint8_t target_addr){
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
		/* D("PREV %d UP %d\n", prev, up); */
		if(up == mesh.addr || up == MESH_ADDR_SINK){
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
	uint8_t status = 0;
	if(next_hop != 255){
		uint8_t addr_buffer[5] = ADDR;
		addr_buffer[4] = next_hop;
		D("SEND(%d, n%2d, f%2d) "fmt16"\n", mesh.addr, next_hop, addr, tx16);
		status = NRF24_send_packet(addr_buffer, mesh.tx_buffer, 16, 1, sync);
		if(status){
			/* We sent a message successfully to next_hop */
			mesh.packetloss[next_hop] = 0;
		} else {
			D("PACKETLOSS!\n");
			if(mesh.packetloss[next_hop] < 255){
				mesh.packetloss[next_hop]++;
			}
			if(mesh.packetloss[next_hop] >= MESH_LOSS_THRESHOLD){
				/* We lost connection to the node next_hop */
				if(next_hop == mesh.parent && mesh.addr != MESH_ADDR_SINK){
					/* We are now disconnected from the mesh */
					mesh.state = MESH_STATE_UNCONNECTED;
					D("UNCONNECTED!\n");
					/* TODO: Inform children that
					   thew are unconnected. */
				} else if (mesh_is_child(next_hop)){
					mesh_publish_del_route(mesh.parent, next_hop, mesh.addr);
				}
			}
		}
	} else if(next_hop == 255 && mesh.addr != MESH_ADDR_SINK){
		mesh.state = MESH_STATE_UNCONNECTED;
	}
	return status;
}

uint8_t mesh_send_direct(uint8_t addr, uint8_t sync){
	/* Calculate next hop */

	uint8_t addr_buffer[5] = ADDR;
	addr_buffer[4] = addr;
	D("DEND(n%2d, f%2d) "fmt16"\n", addr, addr, tx16);
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
	mesh.tx_buffer[3] = mesh.addr;

	mesh.tx_buffer[4] = node;
	mesh.tx_buffer[5] = new_parent;

	return mesh_send(addr,1);
}

uint8_t mesh_publish_del_route(uint8_t addr, uint8_t node, uint8_t old_parent){
	mesh.tx_buffer[0] = (unsigned char)MESH_PACKET_DEL_ROUTE;

	mesh.tx_buffer[1] = mesh.addr;
	mesh.tx_buffer[2] = addr;
	mesh.tx_buffer[3] = mesh.addr;

	mesh.tx_buffer[4] = node;
	mesh.tx_buffer[5] = old_parent;

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
		mesh.packetloss[i] = 0;
	}

	NRF24_init(0, 1);

	NRF24_set_pl_len(MESH_PIPE_ADDR, 16);
	NRF24_set_pl_len(MESH_PIPE_BRD, 16);
	/* NRF24_set_pl_len(0, 16); */

	/* We use pipe 1 for our address */
	NRF24_enable_pipe(MESH_PIPE_ADDR);
	/* And pipe 5 for the broadcast address */
	NRF24_enable_pipe(MESH_PIPE_BRD);
	/* We did not get acks without this: */
	NRF24_enable_pipe(0);

	NRF24_set_channel(120);
	NRF24_set_speed(NRF_250KBIT);


	if(is_sink){
		/* Set our address to the sink address. */
		mesh.addr = MESH_ADDR_SINK;
	} else {
		eeprom_busy_wait();
		/* Load node address from eeprom */
		mesh.addr = eeprom_read_byte(mesh_eeprom_addr);
		if(mesh.addr >= MESH_MAX_NODES){
			/* Use the temporary "new device" address */
			mesh.addr = MESH_ADDR_NEW_DEVICE;
		}
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
		eeprom_busy_wait();
		printf("-R %d %d %02hhx\n",
		       i,	/* Node */
		       mesh_get_parent(i), /* Parent */
		       eeprom_read_byte(MESH_NODE_FREE_ADDR+i)); /* Static node info */
	}
	/* Easier to parse the routeing table with this -- */
	printf("--\n");
}

void mesh_inform_child_hopcount(uint8_t child){
	mesh.tx_buffer[0] = MESH_PACKET_NEW_HOPCOUNT;
	mesh.tx_buffer[1] = mesh.addr;
	mesh.tx_buffer[2] = child;
	mesh.tx_buffer[3] = mesh.addr;
	mesh.tx_buffer[4] = mesh.hopcount;
	uint8_t sent = 0;
	uint8_t c = 0;
	do {
		sent = mesh_send(mesh.tx_buffer[2], 1);
		c++;
	} while(!sent && c < 3);
}

void mesh_inform_children_hopcount(){
	/* Iterate over all the nodes */
	for(int i = 0; i<MESH_MAX_NODES; i++){
		if(mesh_get_parent(i) == mesh.addr){
			/* We are the node i's parent */
			mesh_inform_child_hopcount(mesh_get_parent(i));
		}
	}
}

uint8_t mesh_accept_more_children(){
	/* Count direct children */
	uint8_t c = 0;
	for(int i = 0; i<MESH_MAX_NODES; i++){
		if(mesh_get_parent(i) == mesh.addr){
			c++;
		}
	}
	if(c >= MESH_MAX_CHILDREN){
		/* TODO: Cleanup dead children */
		return 0;
	} else {
		return 1;
	}
}

uint8_t mesh_is_child(uint8_t node){
	if(mesh_get_parent(node) == mesh.addr){
		return 1;
	} else {
		return 0;
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
				if(mesh.state != MESH_STATE_CONNECTED) break;
				D("Node %d got BRD_CONNECT from %d\n",
				  mesh.addr,
				  mesh.rx_buffer[1]);
				if(mesh.rx_buffer[1] == mesh.parent &&
				   mesh.addr != MESH_ADDR_SINK){
					/* OMG! This broadcast connect
					   is from our parent! We
					   cannot reach the sink */
					mesh.state = MESH_STATE_UNCONNECTED;
				} else if(mesh_accept_more_children() || mesh_is_child(mesh.rx_buffer[1])){
					/* The
					   mesh_accept_more_children
					   is for testing purposes so
					   that more interesing meshes
					   are formed */

					if(mesh.addr == MESH_ADDR_SINK){
						mesh.tx_buffer[0] = (unsigned char)MESH_PACKET_OFFER;
						mesh.tx_buffer[1] = mesh.addr;
						mesh.tx_buffer[2] = mesh.rx_buffer[1];
						mesh.tx_buffer[3] = mesh.addr;
						mesh.tx_buffer[4] = mesh.hopcount;
						if(mesh_send_direct(mesh.rx_buffer[1], 1)){
							D("Sent offer to %d\n", mesh.rx_buffer[1]);
						} else {
							D("Failed offer to %d\n", mesh.rx_buffer[1]);
						}
					} else {
						/* Check if we are connected,
						   The response happens in the
						   respons handle for the
						   response of this packet. */
						mesh.tx_buffer[0] = (unsigned char)MESH_PACKET_ALIVE_REQ;
						mesh.tx_buffer[1] = mesh.addr;
						mesh.tx_buffer[2] = MESH_ADDR_SINK;
						mesh.tx_buffer[3] = mesh.addr;
						mesh.tx_buffer[4] = mesh.rx_buffer[1];
						mesh_send(MESH_ADDR_SINK, 1);
					}

				}}
				break;
			}
		} else {
			uint8_t for_us = 1;
			if(mesh.state == MESH_STATE_CONNECTED){
				if (mesh.rx_buffer[3] == mesh.parent){
					/* Packet is sent from parent */
				} else if (mesh_get_parent(mesh.rx_buffer[3]) == mesh.addr){
					/* Message is sent from a direct child */
				} else {
					/* Sender is not parent or direct
					   child. Perhaps we lost connection
					   to this node earlier. Add us as the
					   nodes parent. */
					mesh_route_update(mesh.rx_buffer[3], mesh.addr);
					/* Inform the child of our hopcount */
					mesh_inform_child_hopcount(mesh.rx_buffer[3]);
					/* TODO: Perhaps we should ask the
					   node to inform us of its children?
					   But did we remove its children also
					   from the routing table? */
					mesh_publish_route(mesh.parent, mesh.rx_buffer[3], mesh.addr);
				}


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
					for (int i = 0; i < MESH_LOSS_THRESHOLD; ++i){
						if(mesh_send(mesh.rx_buffer[2], 1)){
							break;
						}
						_delay_ms(3);
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
					/* TODO: Change this in production */
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
				if(mesh.addr == MESH_ADDR_SINK){
					/* Easy to parse */
					printf("-P %d\n", mesh.rx_buffer[1]);
				}

				if(for_us){
					D("Sending pong to %d\n", mesh.rx_buffer[1]);
					mesh_send_pong(mesh.rx_buffer[1]);
				} else {
					/* Ping not for us! */
					D("Ping not for us!\n");
				}
				break;
			}
			case MESH_PACKET_PONG: {
				/* Only accept pong when connected */
				if(mesh.state != MESH_STATE_CONNECTED) break;
				D("Node %d got PONG from %d to %d\n",
				  mesh.addr,
				  mesh.rx_buffer[1],
				  mesh.rx_buffer[2]);
				if(!for_us){
					/* Pong not for us! */
					D("Pong not for us!\n");
				} else {
					if(mesh.rx_buffer[1] == MESH_ADDR_SINK){
						/* This pong is from the sink */
						mesh.last_pong = ms;
					}
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
				  mesh.rx_buffer[4],
				  mesh.rx_buffer[5]);
				mesh_route_update(mesh.rx_buffer[4], mesh.rx_buffer[5]);
				if(mesh.addr != MESH_ADDR_SINK){
					/* We are not the sink. Forward the route upwards */
					mesh_publish_route(mesh.parent, mesh.rx_buffer[4], mesh.rx_buffer[5]);
				}

				mesh_route_dump();
				break;
			}
			case MESH_PACKET_DEL_ROUTE: {
				/* Only accept route info when connected */
				if(mesh.state != MESH_STATE_CONNECTED) break;
				/* Check if packet is for us. TODO:
				   Maybe snoop on all route
				   packets? */
				if(mesh.addr != mesh.rx_buffer[2]) break;

				if(mesh_get_parent(mesh.rx_buffer[4]) == mesh.rx_buffer[5]){
					/* Remove node from routing table */
					mesh_route_update(mesh.rx_buffer[4], 255);
					D("Node %d has disappeared\n", mesh.rx_buffer[4]);
				}

				if(mesh.addr != MESH_ADDR_SINK){
					/* We are not the sink. Forward the route upwards */
					mesh_publish_del_route(mesh.parent, mesh.rx_buffer[3], mesh.rx_buffer[4]);
				}

				mesh_route_dump();
				break;
			}
			case MESH_PACKET_CHANGE_ADDR: {
				if (mesh.rx_buffer[2] != mesh.addr) break;
				/* This packet is for us */

				D("Our address is now %d\n", mesh.rx_buffer[4]);
				mesh.addr = mesh.rx_buffer[4];

				eeprom_busy_wait();
				/* Save the address to EEPROM */
				eeprom_update_byte(mesh_eeprom_addr, mesh.addr);

				uint8_t addr[5] = ADDR;
				addr[4] = mesh.addr;
				NRF24_set_rx_addr(MESH_PIPE_ADDR, addr);
				mesh_publish_route(mesh.parent, mesh.addr, mesh.parent);
				mesh.state = MESH_STATE_CONNECTED;
				break;
			}
			case MESH_PACKET_REQ_ADDR: {
				if (mesh.rx_buffer[2] != mesh.addr &&
				    mesh.addr != MESH_ADDR_SINK) break;

				uint8_t new_address = mesh_generate_address();
				if(new_address){
					D("We give node %d the new address %d\n", mesh.rx_buffer[1], new_address);

					mesh.tx_buffer[0] = MESH_PACKET_CHANGE_ADDR;
					mesh.tx_buffer[1] = mesh.addr;
					mesh.tx_buffer[2] = mesh.rx_buffer[1];
					mesh.tx_buffer[3] = mesh.addr;
					mesh.tx_buffer[4] = new_address;

					mesh_send(mesh.tx_buffer[2], 1);
				}
				break;
			}
			case MESH_PACKET_ALIVE_REQ: {
				if (!for_us) break;
				D("Alive req from %d\n", mesh.rx_buffer[1]);
				memcpy(mesh.tx_buffer, mesh.rx_buffer, 16);
				mesh.tx_buffer[0] = MESH_PACKET_ALIVE_RSP;
				mesh.tx_buffer[1] = mesh.addr;
				mesh.tx_buffer[2] = mesh.rx_buffer[1];
				mesh.tx_buffer[3] = mesh.addr;
				mesh_send(mesh.tx_buffer[2], 1);
				break;
			}
			case MESH_PACKET_ALIVE_RSP: {
				if (!for_us) break;
				D("Alive RSP from %d\n", mesh.rx_buffer[1]);

				mesh.tx_buffer[0] = (unsigned char)MESH_PACKET_OFFER;
				mesh.tx_buffer[1] = mesh.addr;
				mesh.tx_buffer[2] = mesh.rx_buffer[4];
				mesh.tx_buffer[3] = mesh.addr;
				mesh.tx_buffer[4] = mesh.hopcount;
				if(mesh_send_direct(mesh.rx_buffer[4], 1)){
					D("Sent offer to %d\n", mesh.rx_buffer[4]);
				} else {
					D("Failed offer to %d\n", mesh.rx_buffer[4]);
				}

				break;
			}
			case MESH_PACKET_NEW_HOPCOUNT: {
				if (!for_us) break;
				D("New hopcount from %d\n", mesh.rx_buffer[1]);
				if(mesh.rx_buffer[1] == mesh.parent){
					if((mesh.rx_buffer[4] + 1) != mesh.hopcount){
						/* We have now a new hopcount */
						mesh.hopcount = mesh.rx_buffer[4];
						mesh_inform_children_hopcount();

					}
				}

				break;
			}

			}
		}
	}
	unsigned long state_ms_diff = (ms - mesh.last_state_ms);
	switch (mesh.state) {
	case MESH_STATE_BRD_SENT: {
		if(state_ms_diff > 500){
			/* Send request again */
			D("Broadcast\n");
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
			}
		}

		/* Send stuff */
		break;
	}
	case MESH_STATE_COLLECTING_PARENTS: {
		if((ms - mesh.pot.last_changed) > 2000){
			D("Trying to accept route!\n");
			mesh_route_dump();
			/* We have waited long enough for potential
			   parents, connect to the best one. */
			mesh.parent = mesh.pot.parent;
			mesh.hopcount = mesh.pot.hopcount + 1;
			mesh.last_state_ms = ms;
			mesh_route_update(mesh.addr, mesh.parent);

			if (mesh_publish_route(mesh.pot.parent,
					       mesh.addr,
					       mesh.pot.parent)){

				D("Connected to %d!\n",mesh.parent);
				if (mesh.addr == MESH_ADDR_NEW_DEVICE){
					/* We are using the temporary new device address */
					mesh.state = MESH_STATE_W_FOR_ADDRESS;
					/* Ugly hack so that we will
					   send request for new
					   address */
					mesh.last_addr = ms - 30000;
				} else {
					mesh.state = MESH_STATE_CONNECTED;
				}
			} else {
				D("Failed to connect!\n");
			}
		}/*  else if ((ms - mesh.last_state_ms) > 100){ */
		/* 	D("Broadcast\n"); */
		/* 	mesh_send_brd_connect(0); */
		/* 	mesh.last_state_ms = ms; */
		/* } */
	}
	}
}

/* uint8_t mesh_send_data(uint8_t address, uint8_t *data){ */

/* } */

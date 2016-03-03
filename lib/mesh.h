#ifndef __MESH_H_
#define __MESH_H_

#include "includes.h"

/* Connect to the network */
void mesh_init(uint8_t is_sink);

/* Check if connected to the network. Run this after init to check if
   successful. This also will return the nodes address. */
uint8_t mesh_is_connected();

/* Call this functionality regularly. This function handles all the
   routing and such things. */
void mesh_poll();

/* Send data (14 bytes) to address. */
/* uint8_t mesh_send_data(uint8_t address, uint8_t *data); */

#endif

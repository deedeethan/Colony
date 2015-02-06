#ifndef _BOM_H
#define _BOM_H

extern "C" {
#include <stdint.h>
}

#include <messages/bom.h>

// constants for direction
#define BOM_FRONT (messages::bom::FRONT)
#define BOM_BACK  (messages::bom::BACK)
#define BOM_LEFT  (messages::bom::LEFT)
#define BOM_RIGHT (messages::bom::RIGHT)

// timing, in us, for read of valid bits
#define MIN_LOW_PW 600
#define MAX_LOW_PW 1400
#define MIN_HIGH_PW 1600
#define MAX_HIGH_PW 2400

// returned by bom_get if there is no new data since last call
#define BOM_NO_DATA -1

// i/o pins
// if these are changed, remember to change bom_init and/or ISRs
#define PIN_BOM_SIG PINB
#define P_BOM_SIG0 PB0
#define P_BOM_SIG1 PB1
#define P_BOM_SIG2 PB2
#define P_BOM_SIG3 PB3

#define PORT_BOM_EMIT PORTF
#define P_BOM_EMIT0 PF4
#define P_BOM_EMIT1 PF5
#define P_BOM_EMIT2 PF6
#define P_BOM_EMIT3 PF7

typedef uint8_t bom_msg_t;

inline char bom_msg_get_robot_id(bom_msg_t msg) {
    return msg >> 2;
}
inline char bom_msg_get_dir(bom_msg_t msg) {
    return msg & 3;
}
inline bom_msg_t bom_msg_make(char id, char dir) {
    return (id << 2) | (dir & 3);
}

void set_robot_id(char id);
char get_robot_id(void);

// NOTE: call range_init before bom_init to ensure timer 5 is set up!
void bom_init(void);
void bom_send(char dir);
int bom_get(char dir);

// toggles output - should be called at around 76 kHz
void bom_isr();

#endif

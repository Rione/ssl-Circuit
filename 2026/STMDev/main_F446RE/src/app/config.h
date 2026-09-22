#ifndef __Parametors__
#define __Parametors__

#define PHOTOSENSOR_THRESHOLD 750
#define BATTERY_CUT_OFF 150

// CAN Standard ID
// kickboard
#define PHOTOTHRESHOLD 0x09        // 15
#define CHARGE_START 0x10          // 16
#define DISCHARGE_START 0x11       // 17
#define KICK_STRAIGHT 0x12         // 18
#define KICK_CHIP 0x13             // 19
#define KICKER_BOARD_PACKET 0x123  // 291

// -- old dribble, no use
// #define DRIBBLE 0x14         // 20
// #define DRIBLE_STOP 0x15     // 21

// motor driver board
#define MD_SEND 0x1AA  // 426
#define SEND_EMG 0x00  // 0
#define MD0_RECV 0x200
#define MD1_RECV 0x201
#define MD2_RECV 0x202
#define MD3_RECV 0x203

// dribbler board
#define DRIBBLE_SEND 0xF4
#define DRIBBLE_RECV 0xF5

#endif
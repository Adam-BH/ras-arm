// LAYER 6 — APP · state machine
// Decides WHAT the robot does now. One switch(state), checked every loop, never blocks.
#pragma once
#include "../config.h"
#include "../4_motion/stations.h"

typedef enum { ST_BOOT, ST_HOMING, ST_IDLE, ST_READ_COLOUR, ST_PICK, ST_VERIFY,
               ST_PLACE, ST_RETURN, ST_TEACH, ST_ERROR, ST_ESTOP, ST_COUNT } state_t;
#define STATE_NAMES "BOOT\0HOMING\0IDLE\0READ_COLOUR\0PICK\0VERIFY\0PLACE\0RETURN\0TEACH\0ERROR\0ESTOP"

typedef enum { E_NONE, E_HOME_FAILED, E_COLOUR_UNKNOWN, E_PICK_FAILED,
               E_UNREACHABLE, E_ESTOP, E_COUNT } fsm_error_t;      // number = LED blink count
#define COLOUR_NAMES "EMPTY\0RED\0BLUE\0UNKNOWN"   // index = colour_t
#define ERROR_NAMES "NONE\0HOME_FAILED\0COLOUR_UNKNOWN\0PICK_FAILED\0UNREACHABLE\0ESTOP"

typedef enum { CMD_HOME, CMD_RUN, CMD_STOP, CMD_TEACH, CMD_DONE, CMD_RESET } cmd_t;

// answer to every request: R_OK or the reason it was refused (→ "ERR <CMD> <reason>")
// R_ST + st_err_t = a station refused by the validation (reason from ST_ERR_NAMES)
typedef enum { R_OK, R_STATE, R_BUSY, R_RUNNING, R_UNREACHABLE, R_LIMIT, R_ESTOP, R_ST } fsm_reply_t;
#define REPLY_REASONS "ok\0state\0busy\0running\0unreachable\0limit\0estop"

void        fsm_init(void);
void        fsm_tick(uint32_t now);
fsm_reply_t fsm_command(cmd_t c, uint32_t now);

// manual moves: accepted in IDLE (not running) and TEACH; jog/save/go/cal only in TEACH
fsm_reply_t fsm_move(const pose_t *p, uint32_t ms);
fsm_reply_t fsm_line(const pose_t *p, float mm_s);
fsm_reply_t fsm_movej(const angles_t *a, uint32_t ms);
fsm_reply_t fsm_jog(float dx, float dy, float dz, float droll);
fsm_reply_t fsm_jogj(uint8_t j, float delta);
fsm_reply_t fsm_grip(float mm);
fsm_reply_t fsm_save(uint8_t station);    // current tip → station (validated, committed)
fsm_reply_t fsm_go(uint8_t station);
fsm_reply_t fsm_preview(uint8_t station); // ST GO: above, then slow line down
bool        fsm_teaching(void);           // ST edits are accepted only in TEACH
fsm_reply_t fsm_calibrate(colour_t c);
fsm_reply_t fsm_can_tune(void);           // SET allowed now?

bool        fsm_busy(void);               // a manual move, homing or calibration is running
state_t     fsm_state(void);
fsm_error_t fsm_error(void);
bool        fsm_homed(void);
bool        fsm_running(void);
uint16_t    fsm_cycles(void);
colour_t    fsm_last_colour(void);        // last colour decision (any reading)

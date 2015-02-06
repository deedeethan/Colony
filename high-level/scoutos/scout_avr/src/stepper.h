#ifndef _STEPPER_H_
#define _STEPPER_H_

#define S_STEP    PD6
#define S_DIR     PD7

#define S_MS      PB7

#define S_EN      PF0

#define STEP_WHOLE 2
#define STEP_HALF 1

/* Initializes the stepper with whole steps and no direction. step_sweep_bounds
 * and step_dir should be called before using step_sweep. */
void step_init();

/* Don't call step_do_step or step_sweep if you haven't called step_enable() */

void step_set_size(char size);
void step_dir(int dir);
void step_do_step();
void step_flush();
void step_sweep_bounds(int ccw, int cw);
void step_sweep();
void step_enable();
void step_disable();
int step_get_pos();


#endif

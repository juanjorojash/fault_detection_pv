/*
 * my_timing.h
 *
 *  Created on: 14.01.2020
 *      Author: Adrian Wenzel
 */

#ifndef MY_TIMING_H_
#define MY_TIMING_H_

#include <stdint.h>

#define t_start(cycles) cycles = Timer32_getValue(TIMER32_0_BASE)
/* After calling t_stop(cycles), "cycles" will hold the number of cycles passed between t_start and t_stop */
#define t_stop(cycles) cycles -= Timer32_getValue(TIMER32_0_BASE) + 2   // Timer32 is counting downwards, that's why subtracting new value from old
#define t_clock() Timer32_getValue(TIMER32_0_BASE)                      // use: cycles = t_clock()

void t_init(void);

#endif /* MY_TIMING_H_ */

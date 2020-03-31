/*
 * pv_sim.h
 *
 *  Created on: 17.03.2020
 *      Author: Adrian Wenzel
 */

#ifndef PV_SIM_H_
#define PV_SIM_H_

#include <stdint.h>

/* Variable declarations (externally defined variables) */
extern float physicalValues[];
extern uint16_t V_bat_sim;

/* Defines and typedefs */
#ifndef DUTYCYCLE_DEFAULT
#define DUTYCYCLE_DEFAULT   461
#endif
#ifndef PV_MAX_VOLTAGE      // must not be higher than 6.5 V, because ADC input channel uses voltage divider gain 0.5 (max voltage: 3.3V)
#define PV_MAX_VOLTAGE 4.8  // max voltage of PV array
#endif
#define V_BAT_SIM_DEFAULT 3700  // (V)
#define V_BAT_SIM_MAXIMUM 4400  // (V)
#define V_BAT_SIM_MINIMUM 2300  // (V)


/* Functions */
void pv_sim_measurement();
void pv_sim_resetDutycycle(void);
void pv_sim_stepDutycycle(uint16_t cmdUart_dutyCycleStep, uint16_t cmdCtrl_dutyCycleStep);
void pv_sim_trigger_sweep(void);
void pv_sim_sweep(void);
bool pv_sim_sweeping_active(void);



#endif /* PV_SIM_H_ */

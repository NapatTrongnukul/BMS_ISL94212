/*
 * ADC_sense.h
 *
 *  Created on: Jul 31, 2023
 *      Author: joe_n
 */

#ifndef ADC_SENSE_H_
#define ADC_SENSE_H_

#include "VI_ACQ_init.h"

double V_CH1_CONV = 0.00013731;
double I_CH1_OFFSET = 0.0048964;
double I_CH1_CONV = 3.8013e-5;

double ADC_sense();
int * volt_can_conv(int);

#endif /* ADC_SENSE_H_ */

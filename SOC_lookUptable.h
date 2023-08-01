/*
 * SOC_lookUptable.h
 *
 *  Created on: Jul 31, 2023
 *      Author: joe_n
 */

#ifndef SOC_LOOKUPTABLE_H_
#define SOC_LOOKUPTABLE_H_

double soc_t,slope_predict,y_intercept,soc_obv_n;
double * LookUpTable_SoC(double);

double slope[13] = {60.419,121.325,347.586,418.75,192.5,113.8202,111.0989,102.0202,94.88095,95.6,99.64286,94.4444,132.5};
double y_c[13] = {-199.322,-410.424,-1212.42,-1468.05,-653.095,-365.6,-355.413,-320.605,-292.527,-295.415,-311.858,-290.57,-447.093};

#endif /* SOC_LOOKUPTABLE_H_ */

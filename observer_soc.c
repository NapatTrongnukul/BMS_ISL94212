/*
 * observer_soc.c
 *
 *  Created on: Jul 31, 2023
 *      Author: joe_n
 */

#include <math.h>
#include "observer_soc.h"

double observer_soc(double soc_n,double soc_n1, double Rp_s, double slope_s,double Cp_s,double L1_s,double L2_s,double I, double I_n,double I_n1,double V, double V_n,double V_n1)
{
    double k1,k2,k3,k4,b1,b2,b3,b4,b5,b6,b7,b8,b9,soc_n2;

    k1 = (1/(Rp_s*Cp_s))+L1_s+L2_s*slope_s;
    k2 = (L2_s*slope_s)/(Rp_s*Cp_s);
    k3 = (1/(Rp_s*Cp_s))+L1_s-((L2_s*QR)/Cp_s);
    k4 = (1/(Rp_s*Cp_s));

    b1 = (4+(2*k1*Ts)+(pow(Ts,2)*k2));
    b2 = (8-(2*pow(Ts,2)*k2));
    b3 = (2*k1*Ts-(pow(Ts,2)*k2)-4);
    b4 = (2*Ts+k3*pow(Ts,2))/QR;
    b5 = (2*k3*pow(Ts,2))/QR;
    b6 = (k3*pow(Ts,2)-2*Ts)/QR;
    b7 = L2_s*k4*pow(Ts,2)+2*L2_s*Ts;
    b8 = 2*L2_s*k4*pow(Ts,2);
    b9 = (L2_s*k4*pow(Ts,2))-(2*L2_s*Ts);

    soc_n2 = soc_n1*(b2/b1)+soc_n*(b3/b1)+I_n1*(b4/b1)+I_n*(b5/b1)+I*(b6/b1)+V_n1*(b7/b1)+V_n*(b8/b1)+V*(b9/b1);
    return soc_n2;
}

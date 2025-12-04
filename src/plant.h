#pragma once
#include "q15.h"

// y[k+1] = y[k] + alpha*( -y[k] + u[k] )
static inline q15 plant_step(q15 y, q15 u, q15 alpha){
    q15 dif = sub_q15(u, y);
    q15 delta = mul_q15(alpha, dif);
    q15 y_next = add_q15(y, delta);
    return y_next;
}

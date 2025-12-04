#include "pid.h"

q15 pid_step(pid_t* pid, q15 set, q15 meas){
    // błąd
    q15 e = sub_q15(set, meas);

    // --- P ---
    q15 p_term = mul_q15(pid->kp, e);

    // --- D z filtrem (wyjście filtru to Q15) ---
    q15 de = sub_q15(e, pid->e_prev); // różnica błędów
    q15 one_minus_alpha = (q15)(32767 - pid->d_alpha);
    q15 a_part = mul_q15(pid->d_alpha, pid->d_prev);
    q15 b_part = mul_q15(one_minus_alpha, de);
    q15 d_filt = add_q15(a_part, b_part); // Q15
    q15 d_term = mul_q15(pid->kd, d_filt);

    // --- I: wstępny przyrost całki (bez korekty anti-windup) ---
    q15 i_inc = mul_q15(pid->ki, e); // Q15
    int32_t i_tent32 = (int32_t)pid->i_acc + (int32_t)i_inc; // akumulator int32

    // --- u_raw = P + I_tent + D (int32, może wyjść poza int16) ---
    int32_t u_raw32 = (int32_t)p_term + i_tent32 + (int32_t)d_term;

    // --- saturacja do ograniczeń aktuatora (u_sat) ---
    int32_t u_sat32 = u_raw32;
    if (u_sat32 > (int32_t)pid->u_max) u_sat32 = (int32_t)pid->u_max;
    if (u_sat32 < (int32_t)pid->u_min) u_sat32 = (int32_t)pid->u_min;

    // --- korekcja anti-windup przez back-calculation (po saturacji) ---
    int32_t diff32 = u_raw32 - u_sat32;         // >0 kiedy u_raw powyżej saturacji
    q15 diff_q15 = sat16(diff32);               // ogranicz do zakresu q15
    q15 corr = mul_q15(pid->aw_beta, diff_q15); // Q15

    // zastosuj korekcję: odejmij corr gdy u_raw > u_sat (diff>0)
    i_tent32 -= (int32_t)corr;

    // ogranicz akumulator całkowy do i_limit
    if (i_tent32 >  (int32_t)pid->i_limit) i_tent32 =  (int32_t)pid->i_limit;
    if (i_tent32 < -(int32_t)pid->i_limit) i_tent32 = -(int32_t)pid->i_limit;
    pid->i_acc = (q15)i_tent32;

    // zaktualizuj stany członu D
    pid->d_prev = d_filt;
    pid->e_prev = e;

    // końcowe wyjście (przelicz ponownie z zapisaną i_acc dla spójności)
    int32_t u_out32 = (int32_t)p_term + (int32_t)pid->i_acc + (int32_t)d_term;
    if (u_out32 > (int32_t)pid->u_max) u_out32 = (int32_t)pid->u_max;
    if (u_out32 < (int32_t)pid->u_min) u_out32 = (int32_t)pid->u_min;

    return (q15)u_out32;
}

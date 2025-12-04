#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "q15.h"
#include "pid.h"
#include "plant.h"

#define SIM_STEPS 1000
#define STEADY_WINDOW 50

typedef struct {
    int rise_time;        // pierwszy k, dla którego y >= 0.9*set, -1 jeśli nigdy
    int settling_time;    // pierwszy k po którym |y-set| <= 0.02*|set| dla reszty przebiegu, -1 jeśli nigdy
    float overshoot_pct;  // max((y_max - set)/|set| * 100, 0)
    float steady_error;   // średni błąd bezwzględny w ostatnich STEADY_WINDOW krokach
} metrics_t;

static metrics_t simulate_pid(pid_t pid_cfg, float plant_alpha_f, q15 set_q15, q15 y0_q15){
    q15 alpha = f2q15(plant_alpha_f);
    q15 y = y0_q15;
    float set_f = q15tof(set_q15);

    float ys[SIM_STEPS];
    int rise_time = -1;
    float y_max = -1e30f;

    for(int k=0;k<SIM_STEPS;k++){
        q15 u = pid_step(&pid_cfg, set_q15, y);
        y = plant_step(y, u, alpha);
        float yf = q15tof(y);
        ys[k] = yf;
        if (yf > y_max) y_max = yf;
        if (rise_time == -1 && yf >= 0.9f * set_f) rise_time = k;
    }

    // przeregulowanie w %
    float overshoot_pct = 0.0f;
    if (y_max > set_f && fabsf(set_f) > 1e-6f){
        overshoot_pct = (y_max - set_f) / fabsf(set_f) * 100.0f;
    }

    // błąd ustalony: średni błąd bezwzględny w ostatnich STEADY_WINDOW krokach
    int start = SIM_STEPS - STEADY_WINDOW;
    if (start < 0) start = 0;
    float sum_err = 0.0f;
    for(int k=start;k<SIM_STEPS;k++){
        sum_err += fabsf(ys[k] - set_f);
    }
    float steady_error = sum_err / (float)(SIM_STEPS - start);

    // czas ustalania: pierwszy k taki, że dla wszystkich j>=k |y[j]-set| <= 0.02*|set|
    int settling_time = -1;
    float tol = 0.02f * fabsf(set_f);
    for(int k=0;k<SIM_STEPS;k++){
        int ok = 1;
        for(int j=k;j<SIM_STEPS;j++){
            if (fabsf(ys[j] - set_f) > tol){
                ok = 0;
                break;
            }
        }
        if (ok){
            settling_time = k;
            break;
        }
    }

    metrics_t m = { rise_time, settling_time, overshoot_pct, steady_error };
    return m;
}

static void print_table_sep(int cols, int w){
    for(int c=0;c<cols;c++){
        putchar('|');
        for(int i=0;i<w;i++) putchar('-');
    }
    puts("|");
}

static void print_table_row(const char *cells[], int cols, int w){
    for(int c=0;c<cols;c++){
        printf("| %-*s", w-1, cells[c]); // wypełnij z lewej, zostaw jedno miejsce na separator
    }
    puts("|");
}

int main(void){
    // parametry scenariusza (wspólne)
    q15 set = f2q15(0.5f);
    q15 y0  = f2q15(0.0f);
    q15 u_min = f2q15(-1.0f);
    q15 u_max = f2q15( 1.0f);
    q15 i_limit = f2q15(1.0f);
    q15 d_alpha_default = f2q15(0.9f); // parametr filtru dla D w PID (bliżej 1 -> większe tłumienie)

    // konfiguracje PID (wybrane 3 reprezentatywne zestawy)
    #define NPID 3
    const float pid_vals[NPID][3] = {
        {0.3f,   0.0f,  0.0f},   // #1 konserwatywny P
        {0.6f,  0.05f,  0.0f},   // #2 umiarkowane PI
        {1.2f,  0.10f,  0.01f}   // #3 agresywny PID
    };

    // dwie wartości alpha rośliny (wolna i szybsza)
    #define NALPHA 2
    const float plant_alphas[] = { 0.05f, 0.20f };

    printf("Warunki początkowe: set = %.3f, y0 = %.3f, u_min = %.3f, u_max = %.3f, i_limit = %.3f, d_alpha = %.3f\n",
        q15tof(set), q15tof(y0), q15tof(u_min), q15tof(u_max), q15tof(i_limit), q15tof(d_alpha_default));
    printf("\n");
    // ustawienia tabeli: liczba kolumn i stała szerokość kolumny
    const int COLS = 10;
    const int COL_W = 15; // jednakowa szerokość dla wszystkich kolumn

    // komórki nagłówka (po jednym łańcuchu na kolumnę)
    const char *hdr[] = {
        "Konf", "Kp", "Ki", "Kd", "alpha(plant)",
        "Czas narast.", "Przeregul. [%]", "Blad ustal.", "Czas ustal.", "Uwagi"
    };

    // wydrukuj nagłówek z równą szerokością kolumn
    print_table_sep(COLS, COL_W);
    print_table_row(hdr, COLS, COL_W);
    print_table_sep(COLS, COL_W);

    int cfg_idx = 1;
    char buf[COLS][64];

    for(int pi=0; pi<NPID; ++pi){
        for(int ai=0; ai<NALPHA; ++ai){
            float kp_f = pid_vals[pi][0];
            float ki_f = pid_vals[pi][1];
            float kd_f = pid_vals[pi][2];
            float plant_a = plant_alphas[ai];

            pid_t pid = {
                .kp = f2q15(kp_f),
                .ki = f2q15(ki_f),
                .kd = f2q15(kd_f),
                .i_acc = 0,
                .d_prev = 0,
                .e_prev = 0,
                .d_alpha = d_alpha_default,
                .i_limit = i_limit,
                .u_min = u_min,
                .u_max = u_max,
                .aw_beta = f2q15(0.3f) // umiarkowana korekta antywiatr (back-calculation AW)
            };

            metrics_t m = simulate_pid(pid, plant_a, set, y0);

            // przygotuj komórki wiersza jako łańcuchy
            snprintf(buf[0], sizeof buf[0], "#%d", cfg_idx);
            snprintf(buf[1], sizeof buf[1], "%.003f", kp_f);
            snprintf(buf[2], sizeof buf[2], "%.003f", ki_f);
            snprintf(buf[3], sizeof buf[3], "%.003f", kd_f);
            snprintf(buf[4], sizeof buf[4], "%.2f", plant_a);
            if (m.rise_time >= 0) snprintf(buf[5], sizeof buf[5], "%d", m.rise_time);
            else snprintf(buf[5], sizeof buf[5], "-");
            snprintf(buf[6], sizeof buf[6], "%6.2f", m.overshoot_pct);
            snprintf(buf[7], sizeof buf[7], "%6.4f", m.steady_error);
            if (m.settling_time >= 0) snprintf(buf[8], sizeof buf[8], "%d", m.settling_time);
            else snprintf(buf[8], sizeof buf[8], "-");
            // krótka heurystyczna uwaga
            const char *note = "stab.";
            if (m.overshoot_pct > 20.0f) note = "przeregul?";
            if (m.settling_time < 0) note = "brak stab.";
            if (kp_f >= 1.0f && m.overshoot_pct > 50.0f) note = "oscyluje";
            snprintf(buf[9], sizeof buf[9], "%s", note);

            // zbuduj tablicę wskaźników dla funkcji drukującej
            const char *row[COLS];
            for(int c=0;c<COLS;c++) row[c] = buf[c];

            print_table_row(row, COLS, COL_W);
            print_table_sep(COLS, COL_W);

            cfg_idx++;
        }
    }

    return 0;
}

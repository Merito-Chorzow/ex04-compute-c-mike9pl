#include <stdio.h>
#include "q15.h"
#include "pid.h"
#include "plant.h"

int main(void){
    const float kp_vals[] = {0.05f, 0.6f, 1.2f}; // małe, umiarkowane, duże (może oscylować)
    const int nk = sizeof(kp_vals)/sizeof(kp_vals[0]);

    for(int vi=0; vi<nk; ++vi){
        float kp_f = kp_vals[vi];
        printf("=== Test Kp = %.3f ===\n", kp_f);

        pid_t pid = {
            .kp = f2q15(kp_f),
            .ki = f2q15(0.0f),   // wyłączone I
            .kd = f2q15(0.0f),   // wyłączone D
            .i_acc = 0,
            .d_prev = 0,         // inicjalizacja stanu filtru D
            .e_prev = 0,         // inicjalizacja poprzedniego błędu
            .d_alpha = f2q15(0.9f), // filtr D: 0.9 (zwiększone tłumienie)
            .i_limit = f2q15(1.0f),
            .u_min = f2q15(-1.0f),
            .u_max = f2q15( 1.0f),
        };
    
        q15 y = f2q15(0.0f);
        q15 set = f2q15(0.5f);
        q15 alpha = f2q15(0.05f);

    for(int k=0;k<1000;k++){
        q15 u = pid_step(&pid, set, y);
        y = plant_step(y, u, alpha);

        if(k % 50 == 0){
            printf("k=%4d set=%+.3f y=%+.3f u=%+.3f\n",
                k, q15tof(set), q15tof(y), q15tof(u));
        }
    }
        printf("\n");
    }

    // przykład testu windup: porównanie bez AW i z AW
    const float kp_f = 0.6f;

    // 1) bez AW (aw_beta = 0), wąska saturacja -> obserwuj windup
    {
        printf("=== No AW (aw_beta=0), narrow u limits ===\n");
        pid_t pid = {
            .kp = f2q15(kp_f),
            .ki = f2q15(0.05f),
            .kd = f2q15(0.0f),
            .i_acc = 0,
            .d_prev = 0,
            .e_prev = 0,
            .d_alpha = f2q15(0.0f),
            .i_limit = f2q15(1.0f),
            .u_min = f2q15(-0.05f),   // wąska saturacja -> łatwy windup
            .u_max = f2q15( 0.05f),
            .aw_beta = f2q15(0.0f),
        };

        q15 y = f2q15(0.0f);
        q15 set = f2q15(0.5f);
        q15 alpha = f2q15(0.05f);

        for(int k=0;k<1000;k++){
            q15 u = pid_step(&pid, set, y);
            y = plant_step(y, u, alpha);
            if(k % 50 == 0){
                printf("k=%3d set=%+.3f y=%+.3f u=%+.3f i_acc=%+.3f\n",
                    k, q15tof(set), q15tof(y), q15tof(u), q15tof(pid.i_acc));
            }
        }
        printf("\n");
    }

    // 2) z AW (aw_beta > 0), te same warunki
    {
        printf("=== With AW (aw_beta=0.3), narrow u limits ===\n");
        pid_t pid = {
            .kp = f2q15(kp_f),
            .ki = f2q15(0.05f),
            .kd = f2q15(0.0f),
            .i_acc = 0,
            .d_prev = 0,
            .e_prev = 0,
            .d_alpha = f2q15(0.0f),
            .i_limit = f2q15(1.0f),
            .u_min = f2q15(-0.05f),
            .u_max = f2q15( 0.05f),
            .aw_beta = f2q15(0.3f),
        };

        q15 y = f2q15(0.0f);
        q15 set = f2q15(0.5f);
        q15 alpha = f2q15(0.05f);

        for(int k=0;k<1000;k++){
            q15 u = pid_step(&pid, set, y);
            y = plant_step(y, u, alpha);
            if(k % 50 == 0){
                printf("k=%3d set=%+.3f y=%+.3f u=%+.3f i_acc=%+.3f\n",
                    k, q15tof(set), q15tof(y), q15tof(u), q15tof(pid.i_acc));
            }
        }
        printf("\n");
    }

    return 0;
}

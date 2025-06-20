#include <stdio.h>
#include "../p_controller.h"

#define Kp         1.5f
#define SETPOINT   10.0f
#define MIN_OUTPUT 0.0f
#define MAX_OUTPUT 5.0f
#define DT         0.1f   // Zaman adımı (100 ms)
#define MAX_STEPS  100     // Maksimum adım sayısı

int main(void)
{
    sPController controller;
    PController_vInit(&controller, Kp, SETPOINT, MIN_OUTPUT, MAX_OUTPUT);

    float measurement = 0.0f;  // Başlangıç değeri
    float output = 0.0f;

    printf("Simulated Closed-Loop P Controller\n");
    printf("Kp = %.2f, Setpoint = %.2f, Output Limits = [%.2f, %.2f]\n\n",
           controller.fKp, controller.fSetpoint, controller.fMinOutput, controller.fMaxOutput);

    for (int step = 0; step < MAX_STEPS; ++step)
    {
        output = PController_fCompute(&controller, measurement);
        measurement += output * DT;  // Sistemi entegre ediyoruz

        printf("Step %2d: Measurement = %.3f, Output = %.3f\n", step, measurement, output);

        if (measurement >= SETPOINT - 0.01f) {
            printf("\nSetpoint reached at step %d.\n", step);
            break;
        }
    }

    return 0;
}

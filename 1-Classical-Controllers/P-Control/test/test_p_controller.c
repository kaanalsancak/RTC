/**
 * @file test_p_controller.c
 * @brief Simulates a closed-loop proportional (P) controller.
 *
 * This program demonstrates the behavior of a simple P controller in a closed-loop system.
 * It initializes a P controller with specified parameters (gain, setpoint, output limits),
 * then simulates the system response over discrete time steps. At each step, the controller
 * computes the output based on the current measurement, and the system state is updated.
 * The simulation continues until the setpoint is reached or the maximum number of steps is exceeded.
 *
 * @author [Your Name]
 * @date [Date]
 */

#include <stdio.h>
#include "../p_controller.h"

#define Kp         1.5f      /**< Proportional gain */
#define SETPOINT   10.0f     /**< Desired setpoint */
#define MIN_OUTPUT 0.0f      /**< Minimum controller output */
#define MAX_OUTPUT 5.0f      /**< Maximum controller output */
#define DT         0.1f      /**< Time step (100 ms) */
#define MAX_STEPS  100       /**< Maximum number of simulation steps */

/**
 * @brief Main function to simulate the closed-loop P controller.
 *
 * Initializes the P controller and runs a simulation loop, updating the measurement
 * based on the controller output until the setpoint is reached or the maximum number
 * of steps is completed.
 *
 * @return int Returns 0 upon successful completion.
 */
int main(void)
{
    sPController controller;
    PController_vInit(&controller, Kp, SETPOINT, MIN_OUTPUT, MAX_OUTPUT);

    float measurement = 0.0f;  /**< Initial system measurement */
    float output = 0.0f;       /**< Controller output */

    printf("Simulated Closed-Loop P Controller\n");
    printf("Kp = %.2f, Setpoint = %.2f, Output Limits = [%.2f, %.2f]\n\n",
           controller.fKp, controller.fSetpoint, controller.fMinOutput, controller.fMaxOutput);

    for (int step = 0; step < MAX_STEPS; ++step)
    {
        output = PController_fCompute(&controller, measurement);
        measurement += output * DT; /**< Integrate system response */

        printf("Step %2d: Measurement = %.3f, Output = %.3f\n", step, measurement, output);

        if (measurement >= SETPOINT - 0.01f) {
            printf("\nSetpoint reached at step %d.\n", step);
            break;
        }
    }

    return 0;
}
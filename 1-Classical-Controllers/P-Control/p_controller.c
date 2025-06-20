/*------------------------------------------------------------------------------------------------*/
/**
 * @file p_controller.c
 * @brief Implementation of a Proportional (P) Controller with saturation support
 *
 * This source file provides the functional implementation of a basic P controller.
 * The controller calculates the error between the desired setpoint and the current
 * measurement and multiplies it by a gain (Kp) to generate a control output.
 * To ensure safe actuator operation, the output is clamped between specified
 * minimum and maximum limits (saturation).
 */
/*------------------------------------------------------------------------------------------------*/
#include "p_controller.h"
#include <stddef.h>
/*------------------------------------------------------------------------------------------------*/
/**
 * @brief Initializes the P controller parameters including gain, setpoint, and saturation limits.
 *
 * This function sets the controller's proportional gain (Kp), desired setpoint,
 * and output boundaries (min/max). It must be called before using the controller.
 *
 * @param psController Pointer to the controller instance to initialize.
 * @param fKp Proportional gain coefficient.
 * @param fSetpoint Desired setpoint value (reference).
 * @param fMinOutput Minimum allowable output (saturation lower limit).
 * @param fMaxOutput Maximum allowable output (saturation upper limit).
 */
/*------------------------------------------------------------------------------------------------*/
void PController_vInit(sPController* psController, 
                       const float fKp, 
                       const float fSetpoint, 
                       const float fMinOutput, 
                       const float fMaxOutput)
{
    psController->fKp = fKp;
    psController->fSetpoint = fSetpoint;
    psController->fMinOutput = fMinOutput;
    psController->fMaxOutput = fMaxOutput;
}

/*------------------------------------------------------------------------------------------------*/
/**
 * @brief Calculates the control output with proportional logic and applies saturation limits.
 *
 * The control signal is computed as:
 *     output = Kp * (setpoint - measurement)
 *
 * This output is then clamped to the configured minimum and maximum values to ensure
 * safe and bounded actuator behavior.
 *
 * @param psController Pointer to the controller instance containing parameters.
 * @param fMeasurement Current measured value from the system (feedback).
 * @return float Saturated control output signal.
 */
/*------------------------------------------------------------------------------------------------*/
float PController_fCompute(sPController* psController, float fMeasurement)
{
    /* Validate input parameters */
    if (psController == NULL) {
        return 0.0f; /* Handle null  gracefully */
    }
    float fError = psController->fSetpoint - fMeasurement;
    float fOutput = psController->fKp * fError;

    /* Apply saturation limits */
    if (fOutput > psController->fMaxOutput) {
        fOutput = psController->fMaxOutput;
    } else if (fOutput < psController->fMinOutput) {
        fOutput = psController->fMinOutput;
    }

    return fOutput;
}

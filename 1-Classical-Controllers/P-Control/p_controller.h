/*------------------------------------------------------------------------------------------------*/
/**
 * @file p_controller.h
 * @brief Header file for a Proportional (P) Controller with saturation support
 *
 * This module defines the data structure and function prototypes for a 
 * simple Proportional Controller (P-controller). The controller calculates
 * the output based on the error between the setpoint and measured value, scaled
 * by a proportional gain. Output saturation limits are included to prevent
 * actuator overdriving.
 */
/*------------------------------------------------------------------------------------------------*/

#ifndef P_CONTROLLER_H
#define P_CONTROLLER_H

/**
 * @brief Structure representing a Proportional (P) Controller with saturation.
 */
typedef struct {
    float fKp;            /**< @brief Proportional gain constant (Kp). Multiplies the error term. */
    float fSetpoint;      /**< @brief Desired reference value (setpoint) for the system. */
    float fMinOutput;     /**< @brief Minimum output limit to prevent underdriving actuators. */
    float fMaxOutput;     /**< @brief Maximum output limit to prevent overdriving actuators. */
} sPController;

/**
 * @brief Initializes the P controller with gain, setpoint, and output saturation limits.
 *
 * @param psController Pointer to the P controller instance.
 * @param fKp Proportional gain coefficient.
 * @param fSetpoint Target setpoint value.
 * @param fMinOutput Minimum allowed control output.
 * @param fMaxOutput Maximum allowed control output.
 */
void PController_vInit(sPController* psController, float fKp, float fSetpoint, float fMinOutput, float fMaxOutput);

/**
 * @brief Computes the saturated control output based on current measurement.
 *
 * This function calculates the control signal as:
 * `output = Kp * (setpoint - measurement)`
 * and clamps it between the configured min and max output values.
 *
 * @param psController Pointer to the P controller instance.
 * @param fMeasurement Current measured process value (feedback).
 * @return float Saturated control signal to apply.
 */
float PController_fCompute(sPController* psController, float fMeasurement);

#endif // P_CONTROLLER_H

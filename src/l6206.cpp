/**
  ******************************************************************************
  * @file    l6206.cpp
  * @author  STMicroelectronics
  * @version V1.0.0
  * @date    November 14th, 2025
  * @brief   L6206 driver
  * @note     (C) COPYRIGHT 2025 STMicroelectronics
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2025 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "l6206.h"

/* Private constants ---------------------------------------------------------*/

/// The Number of L6206 devices required for initialisation is not supported
#define L6206_ERROR_0   (0xC000)
/// Error: Access a motor index greater than the one of the current bridge configuration
#define L6206_ERROR_1   (0xC001)
/// Error: Use of a bridgeId greater than BRIDGE_B
#define L6206_ERROR_2   (0xC002)

/// Maximum frequency of the PWMs in Hz
#define L6206_MAX_PWM_FREQ   (100000)

/// Minimum frequency of the PWMs in Hz
#define L6206_MIN_PWM_FREQ   (2)

/* Private variables ---------------------------------------------------------*/
uint8_t L6206::numberOfDevices = 0;

/* Private function prototypes -----------------------------------------------*/

/******************************************************//**
 * @brief  Attaches a user callback to the error Handler.
 * The call back will be then called each time the library
 * detects an error
 * @param[in] callback Name of the callback to attach
 * to the error Handler
 * @retval None
 **********************************************************/
void L6206::L6206_AttachErrorHandler(void (*callback)(uint16_t))
{
  errorHandlerCallback = (void (*)(uint16_t))callback;
}

/******************************************************//**
 * @brief Disable the specified bridge
 * @param[in] bridgeId (from 0 for bridge A to 1 for bridge B)
 * @retval None
 * @note  When input of different bridges are parallelized
 * together, the disabling of one bridge leads to the disabling
 * of the second one
 **********************************************************/
void L6206::L6206_DisableBridge(uint8_t bridgeId)
{
  L6206_Board_DisableBridge(bridgeId);
  devicePrm.bridgeEnabled[bridgeId] = FALSE;
  if (devicePrm.config >= PARALLELING_IN1A_IN2A__IN1B_IN2B__1_BIDIR_MOTOR) {
    // If the two bridges have to work together, disable both
    if (bridgeId == BRIDGE_A) {
      L6206_Board_DisableBridge(BRIDGE_B);
      devicePrm.bridgeEnabled[BRIDGE_B] = FALSE;
    } else {
      L6206_Board_DisableBridge(BRIDGE_A);
      devicePrm.bridgeEnabled[BRIDGE_A] = FALSE;
    }
  }
}

/******************************************************//**
 * @brief Enable the specified bridge
 * @param[in] bridgeId (from 0 for bridge A to 1 for bridge B)
 * @retval None
 * @note  When input of different bridges are parallelized
 * together, the enabling of one bridge leads to the enabling
 * of the second one
 **********************************************************/
void L6206::L6206_EnableBridge(uint8_t bridgeId)
{
  devicePrm.bridgeEnabled[bridgeId] = TRUE;
  if (devicePrm.config >= PARALLELING_IN1A_IN2A__IN1B_IN2B__1_BIDIR_MOTOR) {
    // If the two bridges have to work together, enable both
    L6206_Board_EnableBridge(bridgeId, 0);
    if (bridgeId == BRIDGE_A) {
      L6206_Board_EnableBridge(BRIDGE_B, 1);
      devicePrm.bridgeEnabled[BRIDGE_B] = TRUE;
    } else {
      L6206_Board_EnableBridge(BRIDGE_A, 1);
      devicePrm.bridgeEnabled[BRIDGE_A] = TRUE;
    }
  } else {
    L6206_Board_EnableBridge(bridgeId, 1);
  }
}

/******************************************************//**
 * @brief Start the L6206 library
 * @param[in] init pointer to the initialization data
 * @retval None
 **********************************************************/
void L6206::L6206_Init(void *init)
{
  l6206DriverInstance++;

  /* Initialise the GPIOs */
  // Done in the constructor
  //L6206_Board_GpioInit();

  if (init == NULL) {
    /* Set context variables to the predefined values from l6206_target_config.h */
    L6206_SetDeviceParamsToPredefinedValues();
  } else {
    L6206_SetDeviceParamsToGivenValues((L6206_Init_t *) init);
  }

  /* Initialise input bridges PWMs */
  L6206_SetDualFullBridgeConfig(devicePrm.config);
}

/******************************************************//**
 * @brief  Get the PWM frequency of the specified bridge
 * @param[in] bridgeId 0 for bridge A, 1 for bridge B
 * @retval Freq in Hz
 **********************************************************/
uint32_t L6206::L6206_GetBridgeInputPwmFreq(uint8_t bridgeId)
{
  return (devicePrm.pwmFreq[(bridgeId << 1)]);
}

/******************************************************//**
 * @brief  Returns the current speed of the specified motor
 * @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS
 * @retval current speed in % from 0 to 100
 **********************************************************/
uint16_t L6206::L6206_GetCurrentSpeed(uint8_t motorId)
{
  uint16_t speed = 0;

  if (motorId > nbMaxMotorsByConfig[devicePrm.config]) {
    L6206_ErrorHandler(L6206_ERROR_1);
  } else if (devicePrm.motionState[motorId] != INACTIVE) {
    speed = devicePrm.speed[motorId];
  }

  return (speed);
}

/******************************************************//**
 * @brief Returns the device state
 * @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS
 * @retval State (STEADY or INACTIVE)
 **********************************************************/
motorState_t L6206::L6206_GetDeviceState(uint8_t motorId)
{
  motorState_t state = INACTIVE;

  if (motorId > nbMaxMotorsByConfig[devicePrm.config]) {
    L6206_ErrorHandler(L6206_ERROR_1);
  } else {
    state = devicePrm.motionState[motorId];
  }
  return (state);
}

/******************************************************//**
 * @brief Returns the FW version of the library
 * @retval L6206_FW_VERSION
 **********************************************************/
uint32_t L6206::L6206_GetFwVersion(void)
{
  return (L6206_FW_VERSION);
}

/******************************************************//**
 * @brief  Returns the max  speed of the specified motor
 * @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS
 * @retval maxSpeed in % from 0 to 100
 **********************************************************/
uint16_t L6206::L6206_GetMaxSpeed(uint8_t motorId)
{
  uint16_t speed = 0;
  if (motorId > nbMaxMotorsByConfig[devicePrm.config]) {
    L6206_ErrorHandler(L6206_ERROR_1);
  } else {
    speed = devicePrm.speed[motorId];
  }
  return (speed);
}

/******************************************************//**
 * @brief  Get the status of the bridge enabling of the corresponding bridge
 * @param[in] bridgeId from 0 for bridge A to 1 for bridge B
 * @retval State of the Enable&Fault pin of the corresponding bridge (1 set, 0 for reset)
  **********************************************************/
uint16_t L6206::L6206_GetBridgeStatus(uint8_t bridgeId)
{
  uint16_t status = L6206_Board_GetFaultPinState(bridgeId);

  return (status);
}

/******************************************************//**
 * @brief  Immediately stops the motor and disable the power bridge
 * @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS
 * @retval None
 * @note  if two motors uses the same power bridge, the
 * power bridge will be disable only if the two motors are
 * stopped
 **********************************************************/
void L6206::L6206_HardHiz(uint8_t motorId)
{
  if (motorId > nbMaxMotorsByConfig[devicePrm.config]) {
    L6206_ErrorHandler(L6206_ERROR_1);
  } else {
    /* Get bridge Id of the corresponding motor */
    uint8_t bridgeId = L6206_GetBridgeIdUsedByMotorId(motorId);

    if (devicePrm.bridgeEnabled[bridgeId] != FALSE) {
      bool skip = FALSE;

      /* Check if another motor is currently running by using the same bridge */
      switch (devicePrm.config) {
        case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
          if ((motorId > 0) && (devicePrm.motionState[1] == STEADY) && (devicePrm.motionState[2] == STEADY)) {
            skip = TRUE;
          }
          break;
        case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
          if ((motorId < 2) && (devicePrm.motionState[0] == STEADY) && (devicePrm.motionState[1] == STEADY)) {
            skip = TRUE;
          }
          break;
        case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
          if (((motorId < 2) && (devicePrm.motionState[0] == STEADY) && (devicePrm.motionState[1] == STEADY)) ||
              ((motorId > 1) && (devicePrm.motionState[2] == STEADY) && (devicePrm.motionState[3] == STEADY))) {
            skip = TRUE;
          }
          break;
        case PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
          if ((motorId > 0) && (devicePrm.motionState[1] == STEADY) && (devicePrm.motionState[2] == STEADY)) {
            skip = TRUE;
          }
          break;
        case PARALLELING_IN1B_IN2B__2_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
          if ((motorId < 2) && (devicePrm.motionState[0] == STEADY) && (devicePrm.motionState[1] == STEADY)) {
            skip = TRUE;
          }
          break;
        case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_UNDIR_MOTOR_BRIDGE_1A__1_UNDIR_MOTOR_BRIDGE_2A:
          if ((devicePrm.motionState[0] == STEADY) && (devicePrm.motionState[1] == STEADY)) {
            skip = TRUE;
          }
          break;
        default:
          break;
      }

      if (skip == FALSE) {
        /* Disable the bridge */
        L6206_DisableBridge(bridgeId);
      }
    }
    /* Disable the PWM */
    L6206_HardStop(motorId);
  }
}

/******************************************************//**
 * @brief  Stops the motor without disabling the bridge
 * @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS
 * @retval none
 **********************************************************/
void L6206::L6206_HardStop(uint8_t motorId)
{
  if (motorId > nbMaxMotorsByConfig[devicePrm.config]) {
    L6206_ErrorHandler(L6206_ERROR_1);
  } else if (devicePrm.motionState[motorId] != INACTIVE) {
    uint8_t bridgeInputFirstPWM, bridgeInputSecondPWM;

    /* Get bridge input of the corresponding motor */
    bridgeInputFirstPWM = L6206_GetBridgeInputUsedByMotorId(motorId);

    /* for bidirectional motor, disable second PWM*/
    if (L6206_IsBidirectionalMotor(motorId)) {
      bridgeInputSecondPWM = L6206_GetSecondBridgeInputUsedByMotorId(motorId);
      L6206_Board_PwmStop(bridgeInputSecondPWM);
    }

    /* Disable corresponding PWM */
    L6206_Board_PwmStop(bridgeInputFirstPWM);

    /* Set inactive state */
    devicePrm.motionState[motorId] = INACTIVE;
  }
}

/******************************************************//**
 * @brief Read id
 * @retval Id of the l6206 Driver Instance
 **********************************************************/
uint16_t L6206::L6206_ReadId(void)
{
  return (l6206DriverInstance);
}

/******************************************************//**
 * @brief  Runs the motor
 * @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS
 * @param[in] direction FORWARD or BACKWARD
 * @retval None
 * @note  For unidirectional motor, direction parameter has
 * no effect
 **********************************************************/
void L6206::L6206_Run(uint8_t motorId, motorDir_t direction)
{
  if (motorId > nbMaxMotorsByConfig[devicePrm.config]) {
    L6206_ErrorHandler(L6206_ERROR_1);
  } else if ((devicePrm.motionState[motorId] == INACTIVE) ||
             (devicePrm.direction[motorId] != direction)) {
    uint8_t bridgeId;
    uint8_t bridgeInput;

    /* Eventually deactivate motor */
    if (devicePrm.motionState[motorId] != INACTIVE) {
      L6206_HardStop(motorId);
    }

    /* Store new direction */
    if (L6206_IsBidirectionalMotor(motorId)) {
      devicePrm.direction[motorId] = direction;
    } else {
      devicePrm.direction[motorId] = FORWARD;
    }

    /* Switch to steady state */
    devicePrm.motionState[motorId] = STEADY;

    /* Get bridge Id of the corresponding motor */
    bridgeId = L6206_GetBridgeIdUsedByMotorId(motorId);

    /* Get bridge input of the corresponding motor */
    bridgeInput = L6206_GetBridgeInputUsedByMotorId(motorId);

    /* Enable bridge */
    L6206_EnableBridge(bridgeId);

    /* Set PWM */
    if (L6206_IsBidirectionalMotor(motorId)) {
      /* for bidirectional motor */
      L6206_Board_PwmSetFreq(bridgeInput, devicePrm.pwmFreq[bridgeInput], (100 - devicePrm.speed[motorId]));
      bridgeInput = L6206_GetSecondBridgeInputUsedByMotorId(motorId);
      L6206_Board_PwmSetFreq(bridgeInput, devicePrm.pwmFreq[bridgeInput], 100);
    } else {
      /* for unidirectional motor */
      L6206_Board_PwmSetFreq(bridgeInput, devicePrm.pwmFreq[bridgeInput], devicePrm.speed[motorId]);
    }
  }
}

/******************************************************//**
 * @brief Get the motor current direction
 * @param[in] motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1)
 * @retval direction
 **********************************************************/
motorDir_t L6206::L6206_GetDirection(uint8_t motorId)
{
  if (motorId >= nbMaxMotorsByConfig[devicePrm.config]) {
    L6206_ErrorHandler(L6206_ERROR_1);
  }

  if (L6206_IsBidirectionalMotor(motorId)) {
    return devicePrm.direction[motorId];
  } else {
    return FORWARD;
  }
}

/******************************************************//**
 * @brief Set dual full bridge parallelling configuration
 * @param[in] newConfig bridge configuration to apply from
 * dualFullBridgeConfig_t enum
 * @retval None
 **********************************************************/
void L6206::L6206_SetDualFullBridgeConfig(unsigned int newConfig)
{
  devicePrm.config = (dualFullBridgeConfig_t)newConfig;

  /* Start to reset all else if several inits are used consecutively */
  /* they will fail */
  L6206_Board_PwmDeInit(INPUT_1A);
  L6206_Board_PwmDeInit(INPUT_2A);
  L6206_Board_PwmDeInit(INPUT_1B);
  L6206_Board_PwmDeInit(INPUT_2B);

  /* Initialise the bridges inputs PWMs --------------------------------------*/
  switch (devicePrm.config) {
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
    case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
    case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
      L6206_Board_PwmInit(INPUT_1A);
      L6206_Board_PwmInit(INPUT_2A);
      L6206_Board_PwmInit(INPUT_1B);
      L6206_Board_PwmInit(INPUT_2B);
      break;
    case PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
      L6206_Board_PwmDeInit(INPUT_2A);
      L6206_Board_PwmInit(INPUT_1A);
      L6206_Board_PwmInit(INPUT_1B);
      L6206_Board_PwmInit(INPUT_2B);
      break;
    case PARALLELING_IN1B_IN2B__1_BIDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1B_IN2B__2_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
      L6206_Board_PwmDeInit(INPUT_2B);
      L6206_Board_PwmInit(INPUT_1A);
      L6206_Board_PwmInit(INPUT_2A);
      L6206_Board_PwmInit(INPUT_1B);
      break;
    case PARALLELING_IN1A_IN2A__IN1B_IN2B__1_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1A_IN2A__IN1B_IN2B__1_BIDIR_MOTOR:
      L6206_Board_PwmDeInit(INPUT_2A);
      L6206_Board_PwmDeInit(INPUT_2B);
      L6206_Board_PwmInit(INPUT_1A);
      L6206_Board_PwmInit(INPUT_1B);
      break;
    case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_UNDIR_MOTOR_BRIDGE_1A__1_UNDIR_MOTOR_BRIDGE_2A:
    case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_BIDIR_MOTOR:
      L6206_Board_PwmDeInit(INPUT_1B);
      L6206_Board_PwmDeInit(INPUT_2B);
      L6206_Board_PwmInit(INPUT_1A);
      L6206_Board_PwmInit(INPUT_2A);
      break;
    case PARALLELING_ALL_WITH_IN1A___1_UNDIR_MOTOR:
      L6206_Board_PwmDeInit(INPUT_2A);
      L6206_Board_PwmDeInit(INPUT_1B);
      L6206_Board_PwmDeInit(INPUT_2B);
      L6206_Board_PwmInit(INPUT_1A);
      break;
    default:
      break;
  }
}
/******************************************************//**
 * @brief  Changes the max speed of the specified device
 * @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS
 * @param[in] newMaxSpeed in % from 0 to 100
 * @retval true if the command is successfully executed, else false
 **********************************************************/
bool L6206::L6206_SetMaxSpeed(uint8_t motorId, uint16_t newMaxSpeed)
{
  bool cmdExecuted = FALSE;

  if (motorId > nbMaxMotorsByConfig[devicePrm.config]) {
    L6206_ErrorHandler(L6206_ERROR_1);
  } else {
    devicePrm.speed[motorId] = newMaxSpeed;
    if (devicePrm.motionState[motorId] != INACTIVE) {
      uint8_t bridgeInput;

      /* Get Bridge input of the corresponding motor */
      bridgeInput = L6206_GetBridgeInputUsedByMotorId(motorId);

      /* Set PWM frequency*/
      if (L6206_IsBidirectionalMotor(motorId)) {
        /* for bidirectional motor */
        L6206_Board_PwmSetFreq(bridgeInput, devicePrm.pwmFreq[bridgeInput], (100 - devicePrm.speed[motorId]));
      } else {
        /* for unidirectional motor */
        L6206_Board_PwmSetFreq(bridgeInput, devicePrm.pwmFreq[bridgeInput], devicePrm.speed[motorId]);
      }
    }
    cmdExecuted = TRUE;
  }
  return cmdExecuted;
}

/******************************************************//**
 * @brief  Changes the PWM frequency of the bridge input
 * @param[in] bridgeId 0 for bridge A, 1 for bridge B
 * @param[in] newFreq in Hz
 * @retval None
 **********************************************************/
void L6206::L6206_SetBridgeInputPwmFreq(uint8_t bridgeId, uint32_t newFreq)
{
  uint8_t loop;

  if (newFreq > L6206_MAX_PWM_FREQ) {
    newFreq = L6206_MAX_PWM_FREQ;
  }
  for (loop = 0; loop < 2; loop++) {
    uint8_t motorId;
    uint8_t bridgeInput = (bridgeId << 1) + loop;
    devicePrm.pwmFreq[bridgeInput] = newFreq;

    /* Get motor Id using this bridge */
    motorId = L6206_GetMotorIdUsingbridgeInput(bridgeInput);

    /* Immediately update frequency if motor is running */
    if (devicePrm.motionState[motorId] != INACTIVE) {
      /* Test if motor is bidir */
      if (L6206_IsBidirectionalMotor(motorId)) {
        if (bridgeInput != L6206_GetSecondBridgeInputUsedByMotorId(motorId)) {
          /* Set PWM frequency for bidirectional motor of the first bridge */
          L6206_Board_PwmSetFreq(bridgeInput, devicePrm.pwmFreq[bridgeInput], (100 - devicePrm.speed[motorId]));
        } else {
          /* Set PWM frequency for bidirectional motor of the second bridge */
          L6206_Board_PwmSetFreq(bridgeInput, devicePrm.pwmFreq[bridgeInput], 100);
        }
      } else {
        /* Set PWM frequency for unidirectional motor */
        L6206_Board_PwmSetFreq(bridgeInput, devicePrm.pwmFreq[bridgeInput], devicePrm.speed[motorId]);
      }
    }
  }
}
/**
  * @}
  */

/******************************************************//**
 * @brief  Sets the number of devices to be used
 * @param[in] nbDevices (from 1 to MAX_NUMBER_OF_DEVICES)
 * @retval TRUE if successful, FALSE if failure, attempt to set a number of
 * devices greater than MAX_NUMBER_OF_DEVICES
 *********************************************************/
bool L6206::L6206_SetNbDevices(uint8_t nbDevices)
{
  if (nbDevices <= MAX_NUMBER_OF_DEVICES) {
    return TRUE;
  } else {
    return FALSE;
  }
}

/******************************************************//**
* @brief  Returns the number of devices
* @retval number of devices
**********************************************************/
uint8_t L6206::L6206_GetNbDevices()
{
  return numberOfDevices;
}

/******************************************************//**
 * @brief Error handler which calls the user callback (if defined)
 * @param[in] error Number of the error
 * @retval None
 **********************************************************/
void L6206::L6206_ErrorHandler(uint16_t error)
{
  if (errorHandlerCallback != NULL) {
    (void) errorHandlerCallback(error);
  } else {
    while (1) {
      /* Infinite loop */
    }
  }
}

/******************************************************//**
 * @brief  Handlers of the flag interrupt which calls the user callback (if defined)
 * @retval None
 **********************************************************/
void L6206::L6206_FaultInterruptHandler(void)
{
  bool status;

  status = L6206_GetBridgeStatus(BRIDGE_A);
  if (status != devicePrm.bridgeEnabled[BRIDGE_A]) {
    devicePrm.bridgeEnabled[BRIDGE_A] = status;
  }

  status = L6206_GetBridgeStatus(BRIDGE_B);
  if (status != devicePrm.bridgeEnabled[BRIDGE_B]) {
    devicePrm.bridgeEnabled[BRIDGE_B] = status;
  }

  if (int_cb != NULL) {
    int_cb();
  }
}

/******************************************************//**
 * @brief  Get the bridges Id used by a given motor
 * @param motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS
 * @retval bridgeId 0 for bridge A , 1 for bridge B
 **********************************************************/
uint8_t L6206::L6206_GetBridgeIdUsedByMotorId(uint8_t motorId)
{
  uint8_t bridgeId;
  switch (devicePrm.config) {
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1B_IN2B__1_BIDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1A_IN2A__IN1B_IN2B__1_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
      if (motorId == 0) {
        bridgeId = BRIDGE_A;
      } else {
        bridgeId = BRIDGE_B;
      }
      break;
    case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
    case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1B_IN2B__2_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
      if (motorId < 2) {
        bridgeId = BRIDGE_A;
      } else {
        bridgeId = BRIDGE_B;
      }
      break;
    case PARALLELING_IN1A_IN2A__IN1B_IN2B__1_BIDIR_MOTOR:
    case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_UNDIR_MOTOR_BRIDGE_1A__1_UNDIR_MOTOR_BRIDGE_2A:
    case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_BIDIR_MOTOR:
    case PARALLELING_ALL_WITH_IN1A___1_UNDIR_MOTOR:
    default:
      bridgeId = BRIDGE_A;
      break;
  }
  return (bridgeId);
}

/******************************************************//**
 * @brief  Get the motor Id which is using the specified bridge input
 * @param bridgeInput 0 for bridgeInput 1A, 1 for 2A, 2 for 1B, 3 for 3B
 * @retval id of the motor (0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS)
 **********************************************************/
uint8_t L6206::L6206_GetMotorIdUsingbridgeInput(uint8_t bridgeInput)
{
  uint8_t motorId;

  switch (devicePrm.config) {
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1B_IN2B__1_BIDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1A_IN2A__IN1B_IN2B__1_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
      if (bridgeInput >= INPUT_1B) {
        motorId = 1;
      } else {
        motorId = 0;
      }
      break;
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
    case   PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
      if (bridgeInput == INPUT_2B) {
        motorId = 2;
      } else if (bridgeInput == INPUT_1B) {
        motorId = 1;
      } else {
        motorId = 0;
      }
      break;
    case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1B_IN2B__2_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
      if (bridgeInput >= INPUT_1B) {
        motorId = 2;
      } else if (bridgeInput == INPUT_2A) {
        motorId = 1;
      } else {
        motorId = 0;
      }
      break;
    case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
      if (bridgeInput == INPUT_2B) {
        motorId = 3;
      } else if (bridgeInput == INPUT_1B) {
        motorId = 2;
      } else if (bridgeInput == INPUT_2A) {
        motorId = 1;
      } else {
        motorId = 0;
      }
      break;
    case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_UNDIR_MOTOR_BRIDGE_1A__1_UNDIR_MOTOR_BRIDGE_2A:
      if ((bridgeInput == INPUT_2A) || (bridgeInput == INPUT_2B)) {
        motorId = 1;
      } else {
        motorId = 0;
      }
      break;
    case PARALLELING_IN1A_IN2A__IN1B_IN2B__1_BIDIR_MOTOR:
    case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_BIDIR_MOTOR:
    case PARALLELING_ALL_WITH_IN1A___1_UNDIR_MOTOR:
    default:
      motorId = 0;
      break;
  }

  return (motorId);
}
/******************************************************//**
 * @brief  Get the PWM input used by a given motor
 * @param motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS
 * @retval PWM input 0 for 1A, 1 for 2A, 2 for 1B, 3 for 3B
 **********************************************************/
uint8_t L6206::L6206_GetBridgeInputUsedByMotorId(uint8_t motorId)
{
  uint8_t bridgeInput;
  switch (devicePrm.config) {
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
      if (motorId == 0) {
        if (devicePrm.direction[0] == FORWARD) {
          bridgeInput = INPUT_1A;
        } else {
          bridgeInput = INPUT_2A;
        }
      } else {
        if (devicePrm.direction[1] == FORWARD) {
          bridgeInput = INPUT_1B;
        } else {
          bridgeInput = INPUT_2B;
        }
      }
      break;
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
      if (motorId == 0) {
        if (devicePrm.direction[0] == FORWARD) {
          bridgeInput = INPUT_1A;
        } else {
          bridgeInput = INPUT_2A;
        }
      } else if (motorId == 1) {
        bridgeInput = INPUT_1B;
      } else {
        bridgeInput = INPUT_2B;
      }
      break;
    case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
      if (motorId == 0) {
        bridgeInput = INPUT_1A;
      } else if (motorId == 1) {
        bridgeInput = INPUT_2A;
      } else {
        if (devicePrm.direction[2] == FORWARD) {
          bridgeInput = INPUT_1B;
        } else {
          bridgeInput = INPUT_2B;
        }
      }
      break;
    case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
      if (motorId == 0) {
        bridgeInput = INPUT_1A;
      } else if (motorId == 1) {
        bridgeInput = INPUT_2A;
      } else if (motorId == 2) {
        bridgeInput = INPUT_1B;
      } else {
        bridgeInput = INPUT_2B;
      }
      break;
    case PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
      if (motorId == 0) {
        bridgeInput = INPUT_1A;
      } else {
        if (devicePrm.direction[1] == FORWARD) {
          bridgeInput = INPUT_1B;
        } else {
          bridgeInput = INPUT_2B;
        }
      }
      break;
    case PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
      if (motorId == 0) {
        bridgeInput = INPUT_1A;
      } else if (motorId == 1) {
        bridgeInput = INPUT_1B;
      } else {
        bridgeInput = INPUT_2B;
      }
      break;
    case PARALLELING_IN1B_IN2B__1_BIDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
      if (motorId == 0) {
        if (devicePrm.direction[0] == FORWARD) {
          bridgeInput = INPUT_1A;
        } else {
          bridgeInput = INPUT_2A;
        }
      } else {
        bridgeInput = INPUT_1B;
      }
      break;
    case PARALLELING_IN1B_IN2B__2_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
      if (motorId == 0) {
        bridgeInput = INPUT_1A;
      } else if (motorId == 1) {
        bridgeInput = INPUT_2A;
      } else {
        bridgeInput = INPUT_1B;
      }
      break;
    case PARALLELING_IN1A_IN2A__IN1B_IN2B__1_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
      if (motorId == 0) {
        bridgeInput = INPUT_1A;
      } else {
        bridgeInput = INPUT_1B;
      }
      break;
    case PARALLELING_IN1A_IN2A__IN1B_IN2B__1_BIDIR_MOTOR:
      if (devicePrm.direction[0] == FORWARD) {
        bridgeInput = INPUT_1A;
      } else {
        bridgeInput = INPUT_1B;
      }
      break;
    case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_UNDIR_MOTOR_BRIDGE_1A__1_UNDIR_MOTOR_BRIDGE_2A:
      if (motorId == 0) {
        bridgeInput = INPUT_1A;
      } else {
        bridgeInput = INPUT_2A;
      }
      break;
    case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_BIDIR_MOTOR:
      if (devicePrm.direction[0] == FORWARD) {
        bridgeInput = INPUT_1A;
      } else {
        bridgeInput = INPUT_2A;
      }
      break;
    case PARALLELING_ALL_WITH_IN1A___1_UNDIR_MOTOR:
    default:
      bridgeInput = INPUT_1A;
      break;
  }
  return (bridgeInput);
}

/******************************************************//**
 * @brief  Get the second PWM input used by a given bidirectional motor
 * @param motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS
 * @retval PWM input 0 for 1A, 1 for 2A, 2 for 1B, 3 for 3B
 **********************************************************/
uint8_t L6206::L6206_GetSecondBridgeInputUsedByMotorId(uint8_t motorId)
{
  uint8_t bridgeInput = 0xFF;

  switch (devicePrm.config) {
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
      if (motorId == 0) {
        if (devicePrm.direction[0] == FORWARD) {
          bridgeInput = INPUT_2A;
        } else {
          bridgeInput = INPUT_1A;
        }
      } else {
        if (devicePrm.direction[1] == FORWARD) {
          bridgeInput = INPUT_2B;
        } else {
          bridgeInput = INPUT_1B;
        }
      }
      break;
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
      if (motorId == 0) {
        if (devicePrm.direction[0] == FORWARD) {
          bridgeInput = INPUT_2A;
        } else {
          bridgeInput = INPUT_1A;
        }
      }
      break;
    case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
      if (motorId == 2) {
        if (devicePrm.direction[2] == FORWARD) {
          bridgeInput = INPUT_2B;
        } else {
          bridgeInput = INPUT_1B;
        }
      }
      break;
    case PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
      if (motorId == 1) {
        if (devicePrm.direction[1] == FORWARD) {
          bridgeInput = INPUT_2B;
        } else {
          bridgeInput = INPUT_1B;
        }
      }
      break;
    case PARALLELING_IN1B_IN2B__1_BIDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
      if (motorId == 0) {
        if (devicePrm.direction[0] == FORWARD) {
          bridgeInput = INPUT_2A;
        } else {
          bridgeInput = INPUT_1A;
        }
      }
      break;
    case PARALLELING_IN1A_IN2A__IN1B_IN2B__1_BIDIR_MOTOR:
      if (devicePrm.direction[0] == FORWARD) {
        bridgeInput = INPUT_1B;
      } else {
        bridgeInput = INPUT_1A;
      }
      break;
    case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_BIDIR_MOTOR:
      if (devicePrm.direction[0] == FORWARD) {
        bridgeInput = INPUT_2A;
      } else {
        bridgeInput = INPUT_1A;
      }
      break;
    default:
      bridgeInput = 0XFF;
      break;
  }

  if (bridgeInput == 0XFF) {
    L6206_ErrorHandler(L6206_ERROR_2);
  }

  return (bridgeInput);
}

/******************************************************//**
 * @brief  Test if motor is bidirectional
 * @param motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS
 * @retval True if motor is bidirectional, else false
 **********************************************************/
bool L6206::L6206_IsBidirectionalMotor(uint8_t motorId)
{
  bool isBiDir = FALSE;
  switch (devicePrm.config) {
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1A_IN2A__IN1B_IN2B__1_BIDIR_MOTOR:
    case PARALLELING_IN1A_IN1B__IN2A_IN2B__1_BIDIR_MOTOR:
      isBiDir = TRUE;
      break;
    case PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B:
    case PARALLELING_IN1B_IN2B__1_BIDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B:
      if (motorId == 0) {
        isBiDir = TRUE;
      }
      break;
    case PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
      if (motorId == 2) {
        isBiDir = TRUE;
      }
      break;
    case PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B:
      if (motorId == 1) {
        isBiDir = TRUE;
      }
      break;
    default:
      break;
  }

  return (isBiDir);
}


/******************************************************//**
 * @brief  Sets the parameters of the device to predefined values
 * from l6206_target_config.h
 * @retval None
 **********************************************************/
void L6206::L6206_SetDeviceParamsToPredefinedValues(void)
{
  uint32_t i;

  memset(&devicePrm, 0, sizeof(devicePrm));

  devicePrm.config = L6206_CONF_PARAM_PARALLE_BRIDGES;

  devicePrm.pwmFreq[INPUT_1A] = L6206_CONF_PARAM_FREQ_PWM1A;
  devicePrm.pwmFreq[INPUT_2A] = L6206_CONF_PARAM_FREQ_PWM2A;
  devicePrm.pwmFreq[INPUT_1B] = L6206_CONF_PARAM_FREQ_PWM1B;
  devicePrm.pwmFreq[INPUT_2B] = L6206_CONF_PARAM_FREQ_PWM2B;

  for (i = 0; i < MAX_NUMBER_OF_BRUSH_DC_MOTORS; i++) {
    devicePrm.speed[i] = 100;
    devicePrm.direction[i] = FORWARD;
    devicePrm.motionState[i] = INACTIVE;
  }

  for (i = 0; i < L6206_NB_MAX_BRIDGES; i++) {
    devicePrm.bridgeEnabled[i] = FALSE;
  }
}


/******************************************************//**
 * @brief  Set the parameters of the device to values of initDevicePrm structure
 * @param initDevicePrm structure containing values to initialize the device
 * parameters
 * @retval None
 **********************************************************/
void L6206::L6206_SetDeviceParamsToGivenValues(L6206_Init_t *initDevicePrm)
{
  uint32_t i;

  memset(&devicePrm, 0, sizeof(devicePrm));

  devicePrm.config = initDevicePrm->config;

  devicePrm.pwmFreq[INPUT_1A] = initDevicePrm->pwmFreq[INPUT_1A];
  devicePrm.pwmFreq[INPUT_2A] = initDevicePrm->pwmFreq[INPUT_2A];
  devicePrm.pwmFreq[INPUT_1B] = initDevicePrm->pwmFreq[INPUT_1B];
  devicePrm.pwmFreq[INPUT_2B] = initDevicePrm->pwmFreq[INPUT_2B];

  for (i = 0; i < MAX_NUMBER_OF_BRUSH_DC_MOTORS; i++) {
    devicePrm.speed[i] = 100;
    devicePrm.direction[i] = FORWARD;
    devicePrm.motionState[i] = INACTIVE;
  }

  for (i = 0; i < L6206_NB_MAX_BRIDGES; i++) {
    devicePrm.bridgeEnabled[i] = FALSE;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

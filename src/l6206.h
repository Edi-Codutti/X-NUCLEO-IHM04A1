/**
  ******************************************************************************
  * @file    l6206.h
  * @author  STMicroelectronics
  * @version V1.0.0
  * @date    November 14th, 2025
  * @brief   Header for L6206 driver
  * @note    (C) COPYRIGHT 2025 STMicroelectronics
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __L6206_H
#define __L6206_H

/* Includes ------------------------------------------------------------------*/
#include "Arduino.h"
#include "l6206_target_config.h"
#include "motor_def.h"
#include "BDCMotor.h"

/* Exported Constants --------------------------------------------------------*/

/// Current FW major version
#define L6206_FW_MAJOR_VERSION (uint8_t)(1)
/// Current FW minor version
#define L6206_FW_MINOR_VERSION (uint8_t)(0)
/// Current FW patch version
#define L6206_FW_PATCH_VERSION (uint8_t)(0)
/// Current FW version
#define L6206_FW_VERSION (uint32_t)((L6206_FW_MAJOR_VERSION<<16)|\
                                            (L6206_FW_MINOR_VERSION<<8)|\
                                            (L6206_FW_PATCH_VERSION))

///Max number of Brush DC motors
#define L6206_NB_MAX_MOTORS (4)
///Number of Bridges
#define L6206_NB_MAX_BRIDGES (2)
///Max number of input of Bridges
#define L6206_NB_MAX_BRIDGE_INPUT (2 * L6206_NB_MAX_BRIDGES)

/// Bridge A
#define BRIDGE_A         (0)
/// Bridge B
#define BRIDGE_B         (1)

/// Bridge Input 1A
#define INPUT_1A         (0)
/// Bridge Input 2A
#define INPUT_2A         (1)
/// Bridge Input 1B
#define INPUT_1B         (2)
/// Bridge Input 2B
#define INPUT_2B         (3)

/* Exported Types  -------------------------------------------------------*/

/* Typedefs ------------------------------------------------------------------*/
template <typename T>
struct Callback;

template <typename Ret, typename... Params>
struct Callback<Ret(Params...)> {
  template <typename... Args>
  static Ret callback(Args... args)
  {
    return func(args...);
  }
  static std::function<Ret(Params...)> func;
};

template <typename Ret, typename... Params>
std::function<Ret(Params...)> Callback<Ret(Params...)>::func;

typedef void (*FaultInterruptHandler_Callback)(void);

/// Device Parameters Structure Type
typedef struct {
  /// Dual full bridge configuration
  dualFullBridgeConfig_t config;

  /// Pwm frequency of the bridge input
  uint32_t pwmFreq[L6206_NB_MAX_BRIDGE_INPUT];

  /// Speed% (from 0 to 100) of the corresponding motor
  uint16_t speed[L6206_NB_MAX_MOTORS];

  /// FORWARD or BACKWARD direction of the motors
  motorDir_t direction[L6206_NB_MAX_MOTORS];

  /// Current State of the motors
  motorState_t motionState[L6206_NB_MAX_MOTORS];

  /// Current State of the bridges
  bool bridgeEnabled[L6206_NB_MAX_BRIDGES];

} deviceParams_t;

/// Motor driver initialization structure definition
typedef struct {
  dualFullBridgeConfig_t config;
  uint32_t pwmFreq[L6206_NB_MAX_BRIDGE_INPUT];
} L6206_Init_t;

class L6206 : public BDCMotor {
  public:

    /*** Constructor and Destructor Methods ***/

    /**
     * @brief Constructor.
     * @param flag_and_enableA_pin  pin name of the ENA pin of the component.
     * @param flag_and_enableB_pin  pin name of the ENB pin of the component.
     * @param pwm1A_pin             pin name for the PWM input for input 1A.
     * @param pwm2A_pin             pin name for the PWM input for input 2A.
     * @param pwm1B_pin             pin name for the PWM input for input 1B.
     * @param pwm2B_pin             pin name for the PWM input for input 2B.
     */
    L6206(uint8_t flag_and_enableA_pin, uint8_t flag_and_enableB_pin, uint8_t pwm1A_pin, uint8_t pwm2A_pin, uint8_t pwm1B_pin, uint8_t pwm2B_pin) :
      BDCMotor(),
      flag_and_enableA(flag_and_enableA_pin),
      flag_and_enableB(flag_and_enableB_pin),
      pwm1A(pwm1A_pin),
      pwm2A(pwm2A_pin),
      pwm1B(pwm1B_pin),
      pwm2B(pwm2B_pin)
    {
      TIM_TypeDef *pwm1A_instance;
      TIM_TypeDef *pwm2A_instance;
      TIM_TypeDef *pwm1B_instance;
      TIM_TypeDef *pwm2B_instance;

      pinMode(flag_and_enableA, OUTPUT);
      pinMode(flag_and_enableB, OUTPUT);

      pwm1A_instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pwm1A), PinMap_PWM);
      pwm1A_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pwm1A), PinMap_PWM));
      pwm1A_timer = new HardwareTimer(pwm1A_instance);
      first_time_pwm1A = true;
      pwm2A_instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pwm2A), PinMap_PWM);
      pwm2A_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pwm2A), PinMap_PWM));
      pwm2A_timer = new HardwareTimer(pwm2A_instance);
      first_time_pwm2A = true;
      pwm1B_instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pwm1B), PinMap_PWM);
      pwm1B_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pwm1B), PinMap_PWM));
      pwm1B_timer = new HardwareTimer(pwm1B_instance);
      first_time_pwm1B = true;
      pwm2B_instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pwm2B), PinMap_PWM);
      pwm2B_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pwm2B), PinMap_PWM));
      pwm2B_timer = new HardwareTimer(pwm2B_instance);
      first_time_pwm2B = true;

      Callback<void()>::func = std::bind(&L6206::L6206_FaultInterruptHandler, this);
      callback_handler = static_cast<FaultInterruptHandler_Callback>(Callback<void()>::callback);

      errorHandlerCallback = NULL;
      memset(&devicePrm, 0, sizeof(deviceParams_t));
      l6206DriverInstance = numberOfDevices++;
    }

    /**
     * @brief Destructor.
     */
    virtual ~L6206()
    {
      free(pwm1A_timer);
      free(pwm2A_timer);
      free(pwm1B_timer);
      free(pwm2B_timer);
    }


    /*** Public Component Related Methods ***/

    /**
     * @brief Public functions inherited from the Component Class
     */

    /**
     * @brief  Initialize the component.
     * @param  init Pointer to device specific initialization structure.
     * @retval "0" in case of success, an error code otherwise.
     */
    virtual int init(void *init = NULL)
    {
      L6206_Init((void *) init);
      return 0;
    }

    /**
     * @brief  Getting the ID of the component.
     * @param  id Pointer to an allocated variable to store the ID into.
     * @retval "0" in case of success, an error code otherwise.
     */
    virtual int read_id(uint8_t *id = NULL)
    {
      *id = L6206_ReadId();
      return 0;
    }

    /**
     * @brief Public functions inherited from the BCDMotor Class
     */

    /**
     * @brief  Disabling the specified bridge.
     * @param  bridgeId from 0 for bridge A to 1 for bridge B.
     * @retval None.
     */
    virtual void disable_bridge(unsigned int bridgeId)
    {
      L6206_DisableBridge(bridgeId);
    }

    /**
     * @brief  Enabling the specified bridge.
     * @param  bridgeId from 0 for bridge A to 1 for bridge B
     * @retval None.
     */
    virtual void enable_bridge(unsigned int bridgeId)
    {
      L6206_EnableBridge(bridgeId);
    }

    /**
     * @brief  Getting the PWM frequency of the specified bridge;
     * @param  bridgeId from 0 for bridge A to 1 for bridge B.
     * @retval The frequency in Hz of the specified bridge input PWM.
     */
    virtual unsigned int get_bridge_input_pwm_freq(unsigned int bridgeId)
    {
      return (unsigned int) L6206_GetBridgeInputPwmFreq(bridgeId);
    }

    /**
     * @brief  Getting the bridge status.
     * @param  bridgeId from 0 for bridge A to 1 for bridge B.
     * @retval The status.
     */
    virtual unsigned int get_bridge_status(unsigned int bridgeId)
    {
      return (unsigned int) L6206_GetBridgeStatus(bridgeId);
    }

    /**
     * @brief  Getting the device State.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1).
     * @retval The device state (STEADY or INACTIVE)
     */
    virtual unsigned int get_device_state(unsigned int motorId)
    {
      return (motorState_t) L6206_GetDeviceState(motorId);
    }

    /**
     * @brief  Getting the current speed in % of the specified motor.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1).
     * @retval The current speed in %.
     */
    virtual unsigned int get_speed(unsigned int motorId)
    {
      return (unsigned int) L6206_GetCurrentSpeed(motorId);
    }

    /**
     * @brief  Stopping the motor and disabling the power bridge immediately.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1).
     * @retval None.
     */
    virtual void hard_hiz(unsigned int motorId)
    {
      L6206_HardHiz(motorId);
    }

    /**
     * @brief  Stopping the motor immediately.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1).
     * @retval None.
     */
    virtual void hard_stop(unsigned int motorId)
    {
      L6206_HardStop(motorId);
    }

    /**
     * @brief  Running the motor.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1).
     * @param  direction The direction of rotation.
     * @retval None.
     */
    virtual void run(unsigned int motorId, direction_t direction)
    {
      L6206_Run(motorId, (motorDir_t) direction);
    }

    /**
     * @brief  Setting the PWM frequency of the specified bridge.
     * @param  bridgeId from 0 for bridge A to 1 for bridge B.
     * @param  frequency of the PWM in Hz
     * @retval None.
     */
    virtual void set_bridge_input_pwm_freq(unsigned int bridgeId, unsigned int frequency)
    {
      L6206_SetBridgeInputPwmFreq(bridgeId, frequency);
    }

    /**
     * @brief  Setting the dual bridge configuration mode.
     * @param  configuration. The bridge configuration.
     * @retval None.
     */
    virtual void set_dual_full_bridge_config(unsigned int configuration)
    {
      L6206_SetDualFullBridgeConfig(configuration);
    }

    /**
     * @brief  Setting the speed in %.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1).
     * @param  speed The new speed in %.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool set_speed(unsigned int motorId, unsigned int speed)
    {
      return (bool) L6206_SetMaxSpeed(motorId, speed);
    }

    /**
     * @brief Public functions NOT inherited
     */

    /**
     * @brief  Attaching an error handler.
     * @param  fptr An error handler.
     * @retval None.
     */
    virtual void attach_error_handler(void (*fptr)(uint16_t error))
    {
      L6206_AttachErrorHandler((void (*)(uint16_t error)) fptr);
    }

    /**
     * @brief  Getting the motor current direction.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1).
     * @retval direction The direction of rotation.
     */
    virtual direction_t get_direction(unsigned int motorId)
    {
      return (direction_t) L6206_GetDirection(motorId);
    }

    /**
     * @brief  Getting the version of the firmware.
     * @param  None.
     * @retval The version of the firmware.
     */
    virtual unsigned int get_fw_version(void)
    {
      return (unsigned int) L6206_GetFwVersion();
    }

    uint8_t get_nb_devices(void)
    {
      return L6206_GetNbDevices();
    }

    bool set_nb_devices(uint8_t nbDevices)
    {
      return L6206_SetNbDevices(nbDevices);
    }

    /*** Public Interrupt Related Methods ***/

    /**
     * @brief  Attaching an interrupt handler to the FLAG interrupt.
     * @param  fptr An interrupt handler.
     * @retval None.
     */
    void attach_flag_irq(void (*fptr)(void))
    {
      int_cb = fptr;
    }

  protected:

    /*** Protected Component Related Methods ***/

    /**
     * @brief parameter setting and init
     */
    void L6206_Init(void *pInit);
    void L6206_SetDeviceParamsToGivenValues(L6206_Init_t *pInitPrm);
    void L6206_SetDeviceParamsToPredefinedValues(void);
    void L6206_SetDualFullBridgeConfig(unsigned int newConfig);

    uint16_t L6206_ReadId(void);
    void L6206_AttachErrorHandler(void (*callback)(uint16_t));
    void L6206_FaultInterruptHandler(void);
    void L6206_ErrorHandler(uint16_t error);
    void L6206_DisableBridge(uint8_t bridgeId);
    void L6206_EnableBridge(uint8_t bridgeId);
    uint32_t L6206_GetBridgeInputPwmFreq(uint8_t bridgeId);
    uint16_t L6206_GetBridgeStatus(uint8_t bridgeId);
    uint16_t L6206_GetCurrentSpeed(uint8_t motorId);
    motorState_t L6206_GetDeviceState(uint8_t motorId);
    motorDir_t L6206_GetDirection(uint8_t motorId);
    uint16_t L6206_GetMaxSpeed(uint8_t motorId);
    uint32_t L6206_GetFwVersion(void);
    void L6206_HardHiz(uint8_t motorId);
    void L6206_HardStop(uint8_t motorId);
    void L6206_Run(uint8_t motorId, motorDir_t direction);
    void L6206_SetBridgeInputPwmFreq(uint8_t bridgeId, uint32_t newFreq);
    bool L6206_SetMaxSpeed(uint8_t motorId, uint16_t newMaxSpeed);
    uint8_t L6206_GetNbDevices(void);
    uint8_t L6206_GetBridgeIdUsedByMotorId(uint8_t motorId);
    uint8_t L6206_GetMotorIdUsingbridgeInput(uint8_t bridgeInput);
    uint8_t L6206_GetBridgeInputUsedByMotorId(uint8_t motorId);
    uint8_t L6206_GetSecondBridgeInputUsedByMotorId(uint8_t motorId);
    bool L6206_IsBidirectionalMotor(uint8_t motorId);
    bool L6206_SetNbDevices(uint8_t nbDevices);

    /*** Component's I/O Methods ***/

    /**
     * @brief  Making the CPU wait.
     * @param  None.
     * @retval None.
     */
    void L6206_Board_Delay(uint32_t ms_delay)
    {
      delay(ms_delay);
    }

    /**
     * @brief  Disable bridge.
     * @param  bridgeId the bridge id.
     * @retval None.
     */
    void L6206_Board_DisableBridge(uint8_t bridgeId)
    {
      if (bridgeId == 0) {
        detachInterrupt(flag_and_enableA);
        pinMode(flag_and_enableA, OUTPUT);
        digitalWrite(flag_and_enableA, 0);
      } else {
        detachInterrupt(flag_and_enableB);
        pinMode(flag_and_enableB, OUTPUT);
        digitalWrite(flag_and_enableB, 0);
      }
    }

    /**
     * @brief  Enable bridge.
     * @param  bridgeId the bridge id.
     * @param  addDelay if different from 0, a delay is added after bridge activation.
     * @retval None.
     */
    void L6206_Board_EnableBridge(uint8_t bridgeId, uint8_t addDelay)
    {
      if (bridgeId == BRIDGE_A) {
        pinMode(flag_and_enableA, OUTPUT);
        digitalWrite(flag_and_enableA, 1);
        if (addDelay) {
          L6206_Board_Delay(1);
        }
        pinMode(flag_and_enableA, INPUT_PULLUP);
        attachInterrupt(flag_and_enableA, callback_handler, FALLING);
      } else {  // BRIDGE_B
        pinMode(flag_and_enableB, OUTPUT);
        digitalWrite(flag_and_enableB, 1);
        if (addDelay) {
          L6206_Board_Delay(1);
        }
        pinMode(flag_and_enableB, INPUT_PULLUP);
        attachInterrupt(flag_and_enableB, callback_handler, FALLING);
      }
    }

    /**
     * @brief  Get the status of the flag and enable Pin.
     * @param  bridgeId the bridge id.
     * @retval the digital state of the pin.
     */
    uint8_t L6206_Board_GetFaultPinState(uint8_t bridgeId)
    {
      if (bridgeId == BRIDGE_A) {
        return ((uint16_t)digitalRead(flag_and_enableA));
      } else { // BRIDGE B
        return ((uint16_t)digitalRead(flag_and_enableB));
      }
    }

    /**
    * @brief  Deinitialising the PWM.
    * @param  pwmId 0 for bridge A PWM, 1 for bridge B PWM, 2 for REFA PWM, 3 for REFB PWM.
    * @retval None.
    */
    void L6206_Board_PwmDeInit(uint8_t pwmId) {}

    /**
    * @brief  Initialising the PWM.
    * @param  pwmId 0 for bridge A PWM, 1 for bridge B PWM, 2 for REFA PWM, 3 for REFB PWM.
    * @param  onlyChannel.
    * @retval None.
    */
    void L6206_Board_PwmInit(uint8_t pwmId) {}

    /********************************************************
    * @brief  Sets the frequency of PWM used for bridges inputs
    * @param[in] bridgeInput 0 for input 1A, 1 for input 2A,
    * 2 for input 1B,  3 for input 2B
    * @param[in] newFreq in Hz
    * @param[in] duty Duty cycle
    * @retval None
    * @note The frequency is directly the current speed of the device
    **********************************************************/
    void L6206_Board_PwmSetFreq(uint8_t pwmId, uint32_t newFreq, uint8_t duty)
    {
      switch (pwmId) {
        case 0:
        default:
          /* Setting the period and the duty-cycle of PWM 1A. */
          if (!first_time_pwm1A) {
            pwm1A_timer->pauseChannel(pwm1A_channel);
          } else {
            first_time_pwm1A = false;
          }
          pwm1A_timer->setPWM(pwm1A_channel, pwm1A, newFreq, duty);
          break;
        case 1:
          /* Setting the period and the duty-cycle of PWM 2A. */
          if (!first_time_pwm2A) {
            pwm2A_timer->pauseChannel(pwm2A_channel);
          } else {
            first_time_pwm2A = false;
          }
          pwm2A_timer->setPWM(pwm2A_channel, pwm2A, newFreq, duty);
          break;
        case 2:
          /* Setting the period and the duty-cycle of PWM 1B. */
          if (!first_time_pwm1B) {
            pwm1B_timer->pauseChannel(pwm1B_channel);
          } else {
            first_time_pwm1B = false;
          }
          pwm1B_timer->setPWM(pwm1B_channel, pwm1B, newFreq, duty);
          break;
        case 3:
          /* Setting the period and the duty-cycle of PWM 2B. */
          if (!first_time_pwm2B) {
            pwm2B_timer->pauseChannel(pwm2B_channel);
          } else {
            first_time_pwm2B = false;
          }
          pwm2B_timer->setPWM(pwm2B_channel, pwm2B, newFreq, duty);
          break;
      }
    }

    /**
     * @brief  Stopping the PWM.
     * @param  pwmId 0 for input 1A, 1 for 2A, 2 for 1B PWM, 3 for 2B.
     * @retval None.
     */
    void L6206_Board_PwmStop(uint8_t pwmId)
    {
      switch (pwmId) {
        case INPUT_1A:
        default:
          if (!first_time_pwm1A) {
            pwm1A_timer->pauseChannel(pwm1A_channel);
          } else {
            first_time_pwm1A = false;
          }
          pwm1A_timer->setPWM(pwm1A_channel, pwm1A, devicePrm.pwmFreq[INPUT_1A], 0);
          break;
        case INPUT_2A:
          if (!first_time_pwm2A) {
            pwm2A_timer->pauseChannel(pwm2A_channel);
          } else {
            first_time_pwm2A = false;
          }
          pwm2A_timer->setPWM(pwm2A_channel, pwm2A, devicePrm.pwmFreq[INPUT_2A], 0);
          break;
        case INPUT_1B:
          if (!first_time_pwm1B) {
            pwm1B_timer->pauseChannel(pwm1B_channel);
          } else {
            first_time_pwm1B = false;
          }
          pwm1B_timer->setPWM(pwm1B_channel, pwm1B, devicePrm.pwmFreq[INPUT_1B], 0);
          break;
        case INPUT_2B:
          if (!first_time_pwm2B) {
            pwm2B_timer->pauseChannel(pwm2B_channel);
          } else {
            first_time_pwm2B = false;
          }
          pwm2B_timer->setPWM(pwm2B_channel, pwm2B, devicePrm.pwmFreq[INPUT_2B], 0);
          break;
      }
    }

    /*** Component's Instance Variables ***/

    /* Flag Interrupt. */
    uint8_t flag_and_enableA;
    uint8_t flag_and_enableB;
    void (*int_cb)(void);
    FaultInterruptHandler_Callback callback_handler;

    /* Standby/reset pin. */
    uint8_t standby_reset;

    /* Direction pin of bridge A. */
    uint8_t dirA;

    /* Direction pin of bridge B. */
    uint8_t dirB;

    /* Pulse Width Modulation pin for bridge A input. */
    HardwareTimer *pwm1A_timer;
    uint32_t pwm1A_channel;
    uint8_t pwm1A;
    bool first_time_pwm1A;

    /* Pulse Width Modulation pin for bridge A input. */
    HardwareTimer *pwm2A_timer;
    uint32_t pwm2A_channel;
    uint8_t pwm2A;
    bool first_time_pwm2A;

    /* Pulse Width Modulation pin for RefA signal. */
    HardwareTimer *pwm1B_timer;
    uint32_t pwm1B_channel;
    uint8_t pwm1B;
    bool first_time_pwm1B;

    /* Pulse Width Modulation pin for RefB signal. */
    HardwareTimer *pwm2B_timer;
    uint32_t pwm2B_channel;
    uint8_t pwm2B;
    bool first_time_pwm2B;

    /* Data. */
    void (*errorHandlerCallback)(uint16_t error);
    deviceParams_t devicePrm;
    uint8_t l6206DriverInstance;

    /* Static data. */
    static uint8_t numberOfDevices;

    /* Const data. */
    const uint8_t nbMaxMotorsByConfig[PARALLELING_END_ENUM] = {2, 3, 3, 4, 2, 3, 2, 3, 2, 1, 2, 1, 1};
};

#endif /* #ifndef __L6206_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

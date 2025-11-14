/**
 ******************************************************************************
 * @file    X_NUCLEO_IHM15A1_HelloWorld.ino
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    12 January 2022
 * @brief   Arduino test application for the STMicroelectronics X-NUCLEO-IHM15A1
 *          Motor Control Expansion Board: control of 2 DC Brush motors.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2022 STMicroelectronics</center></h2>
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

/* Arduino specific header files. */
#include "Arduino.h"

/* Component specific header files. */
#include "l6206.h"

/* Definitions ---------------------------------------------------------------*/

#define SerialPort Serial

/* Variables -----------------------------------------------------------------*/
L6206_Init_t gMotorInitStruct = {
  .config = L6206_CONF_PARAM_PARALLE_BRIDGES,
  .pwmFreq = {L6206_CONF_PARAM_FREQ_PWM1A, L6206_CONF_PARAM_FREQ_PWM2A, L6206_CONF_PARAM_FREQ_PWM1B, L6206_CONF_PARAM_FREQ_PWM2B}
};

/* Motor Control Component. */
L6206 *motor;

static volatile uint16_t gLastError;

/* Functions -----------------------------------------------------------------*/

/**
 * @brief  This is an example of error handler.
 * @param[in] error Number of the error
 * @retval None
 * @note   If needed, implement it, and then attach it:
 *           + motor->attach_error_handler(&my_error_handler);
 */
void my_error_handler(uint16_t error)
{
  /* Printing to the console. */
  char report[256];
  sprintf(report, "Error 0x%x detected\r\n\n", error);
  SerialPort.print(report);

  uint32_t blinkDelay = 0;

  /* Backup error number */
  gLastError = error;
  switch (gLastError) {
    case 0XBAD0: blinkDelay = 100;
      // fail on Bridge A: fast LED blinking -> 100ms
      motor->disable_bridge(0);
      motor->hard_stop(1);
      break;
    case 0XBAD1: blinkDelay = 500;
      // fail on Bridge B: LED blinking -> 500ms
      motor->disable_bridge(1);
      motor->hard_stop(0);
      break;
    case 0XBADB: blinkDelay = 2000;
      // fail on Bridge A: slow LED blinking -> 2s
      motor->disable_bridge(0);
      motor->disable_bridge(1);
      break;
    default:
      blinkDelay = 10;
  }

  /* Infinite loop */
  while (1) {
    delay(blinkDelay);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(blinkDelay);
    digitalWrite(LED_BUILTIN, LOW);
  }
}

/**
 * @brief  This is an example of user handler for the flag interrupt.
 * @param  None
 * @retval None
 */
void my_flag_irq_handler(void)
{
  /* Code to be customised */
  /************************/
  /****** Code to be customised... *******/

  /**
    *********************************************************************
    * In this example the code checks on which bridge the FAULT occurred.
    * Then an error code is generated according to the FAULT detected and
    * the error handler is called.
    *********************************************************************
  */

  SerialPort.print("    WARNING: \"FLAG\" interrupt triggered.\r\n");

  /* Get the state of bridge A */
  uint16_t bridgeStateA = 1;
  uint16_t bridgeStateB = 1;

  if (motor->get_device_state(0) != INACTIVE) {
    bridgeStateA = motor->get_bridge_status(BRIDGE_A);
  }

  if (motor->get_device_state(1) != INACTIVE) {
    bridgeStateB = motor->get_bridge_status(BRIDGE_B);
  }

  if (bridgeStateA == 0 && bridgeStateB != 0) {
    /* Bridge A was disabled due to overcurrent */
    /* When  motor was running */
    my_error_handler(0XBAD0);
  } else if (bridgeStateA != 0 && bridgeStateB == 0) {
    /* Bridges B was disabled due to overcurrent */
    /* When  motor was running */
    my_error_handler(0XBAD1);
  } else if (bridgeStateA == 0 && bridgeStateB == 0) {
    /* Both Bridges B was disabled due to overcurrent */
    /* over temperature or UVLO when  motor was running */
    my_error_handler(0XBADB);
  }
}

/* setup ----------------------------------------------------------------------*/

void setup()
{
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  /* Initialize Serial Port */
  SerialPort.begin(115200);

  /* Printing to the console. */
  SerialPort.println("STARTING DEMO PROGRAM");

  /* Initialization of the motor driver*/
  /* Please, be careful that pins can change if you change the Nucleo board */
  /* Give a look at the DataSheet of the Nucleo to find a correct configuration */
  motor = new L6206(D2, A4, D5, D4, A0, A1);

  if (motor->init(&gMotorInitStruct) != COMPONENT_OK) {
    exit(EXIT_FAILURE);
  }

  /* Attaching and enabling an interrupt handler. */
  motor->attach_flag_irq(&my_flag_irq_handler);

  /* Attaching an error handler */
  motor->attach_error_handler(&my_error_handler);

  /* Printing to the console. */
  SerialPort.print("Motor Control Application Example for 4 unidirectional brush DC motors\r\n");

  motor->set_dual_full_bridge_config(PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B);

  /* Set PWM Frequency of bridge A inputs to 10000 Hz */
  motor->set_bridge_input_pwm_freq(BRIDGE_A, 10000);

  /* Set PWM Frequency of bridge B inputs to 10000 Hz */
  motor->set_bridge_input_pwm_freq(BRIDGE_B, 10000);
}

/* loop ----------------------------------------------------------------------*/

void loop()
{
  static uint8_t demoStep = 0;

  switch (demoStep) {
    case 0: {
        SerialPort.print("STEP 0: Motor(0) Speed=20%%\r\n");
        /* Set speed of motor 0 to 100 % */
        motor->set_speed(0, 20);
        /* start motor 0 to run */
        motor->run(0, BDCMotor::FWD);
        break;
      }
    case 1: {
        SerialPort.print("STEP 1: Motor(0) Speed=40%%, Motor(1) Speed=20%%\r\n");
        /* Set speed of motor 0 to 40 % */
        motor->set_speed(0, 40);
        /* Set speed of motor 1 to 20 % */
        motor->set_speed(1, 20);
        /* start motor 1 to run */
        motor->run(1, BDCMotor::FWD);
        break;
      }
    case 2: {
        SerialPort.print("STEP 2: Motor(0) Speed=60%%, Motor(1) Speed=40%%, Motor(2) Speed=20%%\r\n");
        /* Set speed of motor 0 to 60 % */
        motor->set_speed(0, 60);
        /* Set speed of motor 1 to 40 % */
        motor->set_speed(1, 40);
        /* Set speed of motor 2 to 20 % */
        motor->set_speed(2, 20);
        /* start motor 2 to run */
        motor->run(2, BDCMotor::FWD);
        break;
      }
    case 3: {
        SerialPort.print("STEP 3: Motor(0) Speed=80%%, Motor(1) Speed=60%%, Motor(2) Speed=40%%, Motor(3) Speed=20%%\r\n");
        /* Set speed of motor 0 to 80 % */
        motor->set_speed(0, 80);
        /* Set speed of motor 1 to 60 % */
        motor->set_speed(1, 60);
        /* Set speed of motor 2 to 40 % */
        motor->set_speed(2, 40);
        /* Set speed of motor 3 to 20 % */
        motor->set_speed(3, 20);
        /* start motor 3 to run */
        motor->run(3, BDCMotor::FWD);
        break;
      }
    case 4:
    default: {
        SerialPort.print("STEP 4: Motor(0:3) Stopped and bridge disabled\r\n");
        motor->hard_hiz(0);
        motor->hard_hiz(1);
        motor->hard_hiz(2);
        motor->hard_hiz(3);
        break;
      }
  }

  /* Wait for 2 seconds */
  delay(2000);

  /* Increment demostep*/
  demoStep = (demoStep + 1) % 5;
}

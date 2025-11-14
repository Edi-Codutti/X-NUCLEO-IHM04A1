# X-NUCLEO-IHM04A1

The X-NUCLEO-IHM04A1 is a dual brush DC motor drive expansion board based on the L6206
(DMOS dual full bridge driver) to drive dual bipolar DC or quad unipolar DC motors.
It provides an affordable and easy-to-use solution for driving DC motors in your STM32 Nucleo project.
The X-NUCLEO-IHM04A1 is compatible with the Arduino UNO R3 connector,
and supports the addition of other expansion boards with a single STM32 Nucleo board.
The user can also mount the ST Morpho connector.

## Examples

There are 3 examples with the X-NUCLEO-IHM04A1 library.
Each one provides a simple example of usage of the X-NUCLEO-IHM04A1 board.
* X_NUCLEO_IHM04A1_2BiDir: This application shows how to use two brush DC motors connected to the board,
moving the rotors with several speed values, direction of rotations, etc.
* X_NUCLEO_IHM04A1_1BiDir: This application is similar to the previous one, but, at every bridge,
the two half-bridges are connected in parallel so that the dissipation in the device 
will be reduced, but the peak current rating is not increased.
This example uses 1 brush DC motor in bidirectional mode.
* X_NUCLEO_IHM04A1_4UniDir: This application shows how use the board to control 4 brush DC motors in unidirectional mode.

## Documentation

You can find the source files at  
https://github.com/stm32duino/X-NUCLEO-IHM04A1

The L6206 datasheet is available at  
https://www.st.com/en/motor-drivers/l6206.html

Information about the IHM04A1 can be found at  
https://www.st.com/en/evaluation-tools/x-nucleo-ihm04a1.html
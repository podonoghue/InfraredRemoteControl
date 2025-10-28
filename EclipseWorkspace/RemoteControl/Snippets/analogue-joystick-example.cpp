/**
 ============================================================================
 * @file analogue-joystick-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief Example showing use of a use of 2 ADC channels with a 2-pot joystick
 *
 *  Created on: 10/6/2016
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"
#include "adc.h"

using namespace USBDM;

/*
 * External Joy-stick
 * 2 x Analogue input
 * 1 x Digital input
 *
 */

// Connection mapping - change as required
using MyAdc = USBDM::Adc0;

using Joystick_X = Adc0::Channel<8>;
using Joystick_Y = Adc0::Channel<9>;
using Joystick_K = GpioD<5,  ActiveLow>;

int main(void) {

   // Enable and configure ADC
   MyAdc::configure(AdcResolution_8bit_se);

   // Connect ADC channels to pins
   Joystick_X::setInput();
   Joystick_Y::setInput();

   // Connect and configure digital input pin
   Joystick_K::setInput(PinPull_Up);

   for(;;) {
      int  x      = Joystick_X::readAnalogue();
      int  y      = Joystick_Y::readAnalogue();
      bool button = Joystick_K::isPressed();
      console.writeln("Joystick (X,Y,K) = ", x, ", ", y, ", ", button?"Pressed":"Released");
   }
}

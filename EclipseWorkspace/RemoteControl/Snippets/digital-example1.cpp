/**
 ============================================================================
 * @file digital-example1.cpp (180.ARM_Peripherals/Snippets/)
 * @brief Basic C++ GPIO output example
 *
 *  Created on: 10/1/2016
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"

using namespace USBDM;

/*
 * Simple example flashing LEDs on digital outputs
 */

// Connection mapping - change as required
using RedLED   = GpioA<1,  ActiveLow>;
using GreenLED = GpioA<2,  ActiveLow>;
using BlueLED  = GpioD<5,  ActiveLow>;

int main() {

   static constexpr PcrInit ledInit {
      PinDriveStrength_High,
      PinDriveMode_PushPull,
      PinSlewRate_Slow
   };

   RedLED::setOutput(ledInit);
   GreenLED::setOutput(ledInit);
   BlueLED::setOutput(ledInit);

   console.setEcho(EchoMode_Off);

   console.write("Choose colour to toggle (R)ed or (G)reen or (B)lue :");
   for(;;) {
      switch(console.readChar()) {
         case 'r': case 'R' : RedLED::toggle();   break;
         case 'g': case 'G' : GreenLED::toggle(); break;
         case 'b': case 'B' : BlueLED::toggle();   break;
         default: break;
      }
   }
}

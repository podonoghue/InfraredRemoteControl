/**
 * @file    cmt-remote.h  (180.ARM_Peripherals/Snippets)
 * @brief   Example showing use of CMT to implement various IR protocols
 */
#ifndef SOURCES_IRREMOTE_H_
#define SOURCES_IRREMOTE_H_

#include "hardware.h"

namespace USBDM {

/**
 * Class to wrap CMT hardware for Interval based IR code protocols e.g. NEC, Laser, Samsung etc
 */
class IrRemote {

   static constexpr uint32_t SONY_LENGTH_MASK = 0xC000'0000;
   static constexpr uint32_t SONY_LENGTH_12   = 0x0000'0000;
   static constexpr uint32_t SONY_LENGTH_15   = 0x8000'0000;
   static constexpr uint32_t SONY_LENGTH_20   = 0x4000'0000;

public:

   static constexpr uint32_t makeSonyCode(uint32_t length, uint32_t code, uint32_t address) {
      if (length == 12) {
         return code|(address<<7)|SONY_LENGTH_12;
      }
      if (length == 15) {
         return code|(address<<7)|SONY_LENGTH_15;
      }
      if (length == 20) {
         return code|(address<<7)|SONY_LENGTH_20;
      }
      return -1U;
   }

   enum Protocol {
      p_LASER,
      p_SONY_TV,
      p_SAMSUNG_TV,
      p_SAMSUNG_DVD,
      p_NEC,
      p_TEAC,
   };
};
} // End namespace USBDM

#endif /* SOURCES_IRREMOTE_H_ */

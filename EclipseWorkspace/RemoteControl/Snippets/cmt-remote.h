/**
 * @file    cmt-remote.h  (180.ARM_Peripherals/Snippets)
 * @brief   Example showing use of CMT to implement various IR protocols
 */
#ifndef SOURCES_IRREMOTE_H_
#define SOURCES_IRREMOTE_H_

#include "hardware.h"
#include "cmt.h"
#include "pit.h"

namespace USBDM {

/**
 * Class to wrap CMT hardware for Interval based IR code protocols e.g. NEC, Laser, Samsung etc
 *
 * Interval IR protocol
 *
 * 38kHz carrier, Pulse distance code, bits transmitted LSB first
 *
 * Pulse interval encoding of bits:
 *
 *    <---- startHigh ----> <--- startLow --->  (Ticks)
 *    +--------------------+                   +--
 *    |---  carrier Hz  ---|                   |       Start
 *  --+--------------------+-------------------+--
 *
 *    <----  zeroHigh ----> <---- zeroLow ---->  (Ticks)
 *    +--------------------+                   +--
 *    |---  carrier Hz  ---|                   |        Logic 0
 *  --+--------------------+-------------------+--
 *
 *    <----   oneHigh  ----> <---  oneLow --->  (Ticks)
 *    +--------------------+                   +--
 *    |---  carrier Hz  ---|                   |        Logic 1
 *  --+--------------------+-------------------+--
 *
 *    <----   oneHigh  ----> <--- startLow --->  (Ticks)
 *    +--------------------+                   +--
 *    |---  carrier Hz  ---|                   |        Middle stop (optional)
 *  --+--------------------+-------------------+--
 *
 *    <----  zeroHigh ----> <---- zeroLow ---->  (Ticks)
 *    +--------------------+                   +--
 *    |---  carrier Hz  ---|                   |        Stop
 *  --+--------------------+-------------------+--
 *
 *  Packet format:
 *                   <-- packetLength bits (exl. stop) -->
 *    +------------ +-----//-------+-------+-----//-------+----------+
 *    |    Start    |    DATA      | Stop  |    DATA      |   Stop   |
 *  --+-------------+-----//-------+-------+-----//-------+----------+
 *                                   ^
 *                                   | middleStopBit
 */
class IrRemote {

public:

   // Example IRP {38k,500}<1,-1|1,-3>(9,-9,D:8,S:8,1,-9,E:4,F:8,-68u,~F:8,1,-118)+
   struct Parameters {
      Hertz    carrier;

      // Following are in CMT ticks = 1us
      Ticks    zeroHigh;
      Ticks    zeroLow;
      Ticks    oneHigh;
      Ticks    oneLow;
      Ticks    startHigh;
      Ticks    startLow;
      Ticks    repeatTime;
      Ticks    repeatHigh;
      Ticks    repeatLow;

      unsigned packetLength;  //< Number of bits in entire packet
      unsigned middleStopBit; //< Position of middle stop bit (0=none)
      unsigned repeats;       //< Number of repeats (including original)
      bool     fastRepeats;   //< Indicates use of fast repeat where only leader + stop are used on repeats

   };

   enum Protocol {
      p_LASER,
      p_SONY_TV,
      p_SAMSUNG_TV,
      p_SAMSUNG_DVD,
      p_NEC,
      p_TEAC,
   };

protected:

   enum State {
      s_Initial,
      s_Start,
      s_FirstWord,
      s_MiddleStop,
      s_SecondWord,
      s_Stop,
      s_SpaceTrailer,
      s_MarkTrailer,  // Minimum time between packets
      s_Complete,
   };

   // Parameters for transmission
   inline static Parameters parameters;

   // Debug LED connection - change as required
   using DebugLed = Digital_D2;

   /** State of transmission */
   inline static volatile State state;

   /** 1st Command/address/extended value to transmit */
   inline static volatile uint32_t data1;

   /** 2nd Command/address/extended value to transmit */
   inline static volatile uint32_t data2;

   /** Indicates use of fast repeat where only leader + stop are used on repeats */
   inline static Protocol protocol;

   IrRemote() = delete;
   IrRemote(const IrRemote &) = delete;

   /**
    * Starts transmission
    *
    * @param delay Post command delay in milliseconds
    */
   static void start(unsigned delay=0) {

      busyFlag = true;

      /// Carrier half period in CMT clock cycles (Based on 8MHz CMT clock)
      const Ticks  carrierHalfPeriodInTicks = Ticks(8_MHz/parameters.carrier/2);

      const Cmt::Init cmtInitValue {
         NvicPriority_Normal,
         cmtCallback,

         CmtMode_Time ,                                       // (cmt_msc_mode)   Mode of operation - Time
         CmtClockPrescaler_Auto ,                             // (cmt_pps_ppsdiv) Primary Prescaler Divider - Auto ~8MHz
         CmtIntermediatePrescaler_DivBy1 ,                    // (cmt_msc_cmtdiv) Intermediate frequency Prescaler - Intermediate frequency /4
         CmtOutput_ActiveHigh ,                               // (cmt_oc_output)  Output Control - Disabled
         CmtEndOfCycleAction_Interrupt ,                      // (cmt_dma_irq)    End of Cycle Event handling - Interrupt Request
         CmtPrimaryCarrierHighTime(carrierHalfPeriodInTicks), // (cmt_cgh1_ph)    Primary Carrier High Time Data Value
         CmtPrimaryCarrierLowTime(carrierHalfPeriodInTicks),  // (cmt_cgl1_pl)    Primary Carrier Low Time Data Value
         CmtMarkPeriod(parameters.startHigh) ,                // (cmt_mark)       Mark period
         CmtSpacePeriod(parameters.startLow),                 // (cmt_space)      Space period
      };

      DebugLed::setOutput();

      Cmt::OutputPin::setOutput(PinDriveStrength_High);

      // Restart state machine
      state        = s_Initial;

      // Set up post-command delay
      delayInMilliseconds = delay;

      // Starts CMT operation (1st interrupt)
      Cmt::configure(cmtInitValue);
   }

   /**
    * Callback from CMT.
    *
    * This processes each timing interval of the transmission.
    */
   static void cmtCallback();
   static void cmtCallback_Sony();

   /**
    * Callback for past command delay
    */
   static void pitCallback();

   // Indicates the interface is busy sending the last code
   static inline bool busyFlag = false;

   // Delay after sending command before considered not busy
   static inline unsigned delayInMilliseconds = 0;

   static volatile inline unsigned intCount = 0;
   /**
    * Indicate if CMT is still busy sending a transmission
    *
    * @return True if CMT is still transmitting
    */
   static bool isBusy() {
      return busyFlag;
   }

   /**
    * Busy-wait for CMT to complete transmitting sequence
    */
   static void waitUntilComplete() {
      while(isBusy()) {
      }
   }

};

/**
 * Class to wrap CMT hardware for Laser-DVD protocol
 *
 * Based on NEC IR protocol IRP code {38.0k,564}<1,-1|1,-3>(16,-8,D:8,S:8,F:8,~F:8,1,^108m,(16,-4,1,^108m)*)
 *
 * 38kHz carrier, Pulse distance code, bits transmitted LSB first
 *
 * Pulse interval encoding of bits:
 *
 *    <--- 9000 us ---> <- 4500 us->
 *    +----------------+            +
 *    |xxxx 38kHz xxxxx|            |                    Start
 *  --+----------------+------------+
 *
 *    <--- 562.5 us ---> <--- 562.5 us --->
 *    +-----------------+                  +
 *    |xxxx  38kHz  xxxx|                  |             Logic 0 = 1:1
 *  --+-----------------+------------------+
 *
 *    <--- 562.5 us ---> <------- 1687.5 us ------->
 *    +-----------------+                           +
 *    |xxxx  38kHz  xxxx|                           |    Logic 1 = 1:3
 *  --+-----------------+---------------------------+
 *
 *    <--- 562.5 us ---> <--- 562.5 us --->
 *    +-----------------+                  +
 *    |xxxx  38kHz  xxxx|                  |             Stop = Logic 0 = 1:1
 *  --+-----------------+------------------+
 *
 *  Packet format: Data = Device:8=FF,Subtype:8=00,Function:8,~Function:8
 *                   <---        32 bits         -->
 *    +------------ +---------------//--------------+----------+
 *    |    Start    |             DATA              |   Stop   |
 *  --+-------------+---------------//--------------+----------+
 */
class  IrLaserDVD : public IrRemote {
private:

   IrLaserDVD() = delete;
   IrLaserDVD(const IrLaserDVD &) = delete;

   /// Constant describing transmission
   static constexpr Parameters myParameters = {
         /* carrier       */    38_kHz,

         // Following are in CMT ticks = 1us, multiples of 564u
         /* zeroHigh       1   */    564_ticks,
         /* zeroLow       -1   */    564_ticks,
         /* oneHigh        1   */    564_ticks,
         /* oneLow        -3   */  3*564_ticks,
         /* startHigh     16   */ 16*564_ticks,
         /* startLow      -8   */  8*564_ticks,
         /* repeatTime   108m  */ 108000_ticks,
         /* repeatHigh         */ 16*564_ticks,
         /* repeatLow          */  4*564_ticks,

         /* packetLength       */    32,
         /* middleStopBit      */    0,
         /* repeats            */    3,
         /* fastRepeats        */    true,
   };

public:
   /**
    * Common codes
    */
   enum Code : uint32_t {
      A_B                 =    0xAA55FF00,
      ANGLE               =    0xF00FFF00,
      AUDIO               =    0xBC43FF00,
      CHANNEL             =    0xE718FF00,
      CLEAR               =    0xAB54FF00,
      COPY_DELETE         =    0xEA15FF00,
      DOWN                =    0xB748FF00,
      DVD_USB             =    0xF807FF00,
      EJECT               =    0xFF00FF00,
      FORWARD             =    0xEF10FF00,
      FORWARD_SCENE       =    0xE31CFF00,
      LEFT                =    0xB34CFF00,
      MARK                =    0xEC13FF00,
      MENU                =    0xF40BFF00,
      MUTE                =    0xA35CFF00,
      NUM0                =    0xB24DFF00,
      NUM1                =    0xF20DFF00,
      NUM2                =    0xF609FF00,
      NUM3                =    0xFA05FF00,
      NUM4                =    0xB04FFF00,
      NUM5                =    0xB44BFF00,
      NUM6                =    0xB847FF00,
      NUM7                =    0xB14EFF00,
      NUM8                =    0xB54AFF00,
      NUM9                =    0xB946FF00,
      OK                  =    0xF906FF00,
      ON_OFF              =    0xF30CFF00,
      OSD                 =    0xA25DFF00,
      PAUSE               =    0xEB14FF00,
      PAUSE_PLAY          =    0xE817FF00,
      PBC                 =    0xE619FF00,
      PLAY                =    0xA05FFF00,
      PROG                =    0xBD42FF00,
      Q_PLAY              =    0xE916FF00,
      REPEAT              =    0xAE51FF00,
      RETURN              =    0xEE11FF00,
      REVERSE             =    0xA857FF00,
      REVERSE_SCENE       =    0xA45BFF00,
      RIGHT               =    0xBF40FF00,
      SEARCH              =    0xBA45FF00,
      SETUP               =    0xFC03FF00,
      SLOW                =    0xA758FF00,
      STEP                =    0xED12FF00,
      STOP                =    0xF50AFF00,
      SUBTITLE            =    0xFE01FF00,
      TITLE               =    0xAF50FF00,
      UP                  =    0xBB44FF00,
      VIDEO               =    0xA659FF00,
      VOLUME_DOWN         =    0xF708FF00,
      VOLUME_UP           =    0xFB04FF00,
      ZOOM                =    0xBE41FF00,
   };

   /**
    * Start transmission of sequence.
    *
    * @param code       Command to send
    * @param repeats    Number of times to repeat (including original). 0 => use default for protocol
    */
   static void send(Code code, unsigned delay, unsigned repeats=0) {

      waitUntilComplete();

      console.writeln("Laser-DVD: 0x", code, Radix_16);

      data1        = code;
      data2        = 0;
      protocol     = p_NEC;

      parameters   = myParameters;
      if (repeats != 0) {
         parameters.repeats = repeats;
      }

      IrRemote::start(delay);
   }

};

static constexpr uint32_t makeTeacCode(uint8_t device, uint8_t subDevice, uint8_t code) {

   return (device<<24)|(subDevice<<16)|(code<<8)|(0x00<<0);
}

/**
 * Class to wrap CMT hardware for Teac-DVD protocol
 *
 * Based on NEC IR protocol
 *
 * 38kHz carrier, Pulse distance code, bits transmitted LSB first
 *
 * Pulse interval encoding of bits:
 *
 *    <--- 9000 us ---> <- 4500 us->
 *    +----------------+            +
 *    |xxxx 38kHz xxxxx|            |                    Start
 *  --+----------------+------------+
 *
 *    <--- 562.5 us ---> <--- 562.5 us --->
 *    +-----------------+                  +
 *    |xxxx  38kHz  xxxx|                  |             Logic 0
 *  --+-----------------+------------------+
 *
 *    <--- 562.5 us ---> <------- 1687.5 us ------->
 *    +-----------------+                           +
 *    |xxxx  38kHz  xxxx|                           |    Logic 1
 *  --+-----------------+---------------------------+
 *
 *    <--- 562.5 us ---> <--- 562.5 us --->
 *    +-----------------+                  +
 *    |xxxx  38kHz  xxxx|                  |             Stop = Logic 0
 *  --+-----------------+------------------+
 *
 *  Packet format: Data = Device:8=BF,Subtype:8=00,Function:8,~G:8=00
 *                   <---        32 bits         -->
 *    +------------ +---------------//--------------+----------+
 *    |    Start    |             DATA              |   Stop   |
 *  --+-------------+---------------//--------------+----------+
 */
class  IrTeacPVR : public IrRemote {
private:

   IrTeacPVR() = delete;
   IrTeacPVR(const IrTeacPVR &) = delete;

   /// Constant describing transmission
   static constexpr Parameters myParameters = {
         /* carrier       */    38_kHz,

         // Following are in CMT ticks = 1us, multiples of 564u
         /* zeroHigh       1   */    564_ticks,
         /* zeroLow       -1   */    564_ticks,
         /* oneHigh        1   */    564_ticks,
         /* oneLow        -3   */  3*564_ticks,
         /* startHigh     16   */ 16*564_ticks,
         /* startLow      -8   */  8*564_ticks,
         /* repeatTime   108m  */ 108000_ticks,
         /* repeatHigh         */ 16*564_ticks,
         /* repeatLow          */  4*564_ticks,

         /* packetLength  */    32,
         /* middleStopBit */    0,
         /* repeats       */    3,
         /* fastRepeats   */    true,
   };

public:
   /**
    * Common codes
    */
   enum Code : uint32_t {   //              Dev  Sub  F
      AUDIO               =    makeTeacCode(0xAE,0x51,0xBF),
      BLUE                =    makeTeacCode(0xFC,0x03,0xBF),
      DOWN                =    makeTeacCode(0xE9,0x16,0xBF),
      EPG                 =    makeTeacCode(0xB2,0x4D,0xBF),
      EXIT                =    makeTeacCode(0xFA,0x05,0xBF),
      FAV                 =    makeTeacCode(0xAA,0x55,0xBF),
      FORWARD             =    makeTeacCode(0xB7,0x48,0xBF),
      FORWARD_SCENE       =    makeTeacCode(0xF4,0x0B,0xBF),
      GOTO                =    makeTeacCode(0xE8,0x17,0xBF),
      GREEN               =    makeTeacCode(0xBF,0x40,0xBF),
      INFO                =    makeTeacCode(0xF1,0x0E,0xBF),
      LEFT                =    makeTeacCode(0xA5,0x5A,0xBF),
      LIST                =    makeTeacCode(0xE7,0x18,0xBF),
      MENU                =    makeTeacCode(0xBA,0x45,0xBF),
      MUTE                =    makeTeacCode(0xE6,0x19,0xBF),
      NUM0                =    makeTeacCode(0xF0,0x0F,0xBF),
      NUM1                =    makeTeacCode(0xAD,0x52,0xBF),
      NUM2                =    makeTeacCode(0xAF,0x50,0xBF),
      NUM3                =    makeTeacCode(0xEF,0x10,0xBF),
      NUM4                =    makeTeacCode(0xA9,0x56,0xBF),
      NUM5                =    makeTeacCode(0xAB,0x54,0xBF),
      NUM6                =    makeTeacCode(0xEB,0x14,0xBF),
      NUM7                =    makeTeacCode(0xB1,0x4E,0xBF),
      NUM8                =    makeTeacCode(0xB3,0x4C,0xBF),
      NUM9                =    makeTeacCode(0xF3,0x0C,0xBF),
      OK                  =    makeTeacCode(0xE5,0x1A,0xBF),
      ON_OFF              =    makeTeacCode(0xA6,0x59,0xBF),
      PAUSE               =    makeTeacCode(0xBB,0x44,0xBF),
      PLAY                =    makeTeacCode(0xB9,0x46,0xBF),
      REC                 =    makeTeacCode(0xA7,0x58,0xBF),
      RECALL              =    makeTeacCode(0xEC,0x13,0xBF),
      RED                 =    makeTeacCode(0xBD,0x42,0xBF),
      REPEAT              =    makeTeacCode(0xF8,0x07,0xBF),
      REVERSE             =    makeTeacCode(0xB5,0x4A,0xBF),
      REVERSE_SCENE       =    makeTeacCode(0xF7,0x08,0xBF),
      RIGHT               =    makeTeacCode(0xE4,0x1B,0xBF),
      STOP                =    makeTeacCode(0xFB,0x04,0xBF),
      SUBTITLE            =    makeTeacCode(0xEE,0x11,0xBF),
      TTX                 =    makeTeacCode(0xF2,0x0D,0xBF),
      TV_RADIO            =    makeTeacCode(0xEA,0x15,0xBF),
      UP                  =    makeTeacCode(0xF9,0x06,0xBF),
      YELLOW              =    makeTeacCode(0xFF,0x00,0xBF),
};

   /**
    * Start transmission of sequence.
    *
    * @param code       Command to send
    * @param repeats    Number of times to repeat (including original). 0 => use default for protocol
    */
   static void send(Code code, unsigned delay=0, unsigned repeats=0) {

      waitUntilComplete();

      console.writeln("Teac-PVR: 0x", code, Radix_16);

      data1        = code;
      data2        = 0;
      protocol     = p_TEAC;

      parameters   = myParameters;
      if (repeats != 0) {
         parameters.repeats = repeats;
      }

      IrRemote::start(delay);
   }

};

/**
 * Class to wrap CMT hardware for Laser-DVD protocol
 *
 * Based on NEC IR protocol
 *
 * 38kHz carrier, Pulse distance code, bits transmitted LSB first
 *
 * Pulse interval encoding of bits:
 *
 *    <--- 9000 us ---> <- 4500 us->
 *    +----------------+            +
 *    |xxxx 38kHz xxxxx|            |                    Start
 *  --+----------------+------------+
 *
 *    <--- 562.5 us ---> <--- 562.5 us --->
 *    +-----------------+                  +
 *    |xxxx  38kHz  xxxx|                  |             Logic 0
 *  --+-----------------+------------------+
 *
 *    <--- 562.5 us ---> <------- 1687.5 us ------->
 *    +-----------------+                           +
 *    |xxxx  38kHz  xxxx|                           |    Logic 1
 *  --+-----------------+---------------------------+
 *
 *    <--- 562.5 us ---> <--- 562.5 us --->
 *    +-----------------+                  +
 *    |xxxx  38kHz  xxxx|                  |             Stop = Logic 0
 *  --+-----------------+------------------+
 *
 *  Packet format: Data = Device:8=FF,Subtype:8=00,Function:8,~Function:8
 *                   <---        32 bits         -->
 *    +------------ +---------------//--------------+----------+
 *    |    Start    |             DATA              |   Stop   |
 *  --+-------------+---------------//--------------+----------+
 */
class  IrTeacDVD : public IrRemote {
private:

   IrTeacDVD() = delete;
   IrTeacDVD(const IrTeacDVD &) = delete;

   /// Constant describing transmission
   static constexpr Parameters myParameters = {
         /* carrier       */    38_kHz,

         // Following are in CMT ticks = 1us, multiples of 564u
         /* zeroHigh       1   */    564_ticks,
         /* zeroLow       -1   */    564_ticks,
         /* oneHigh        1   */    564_ticks,
         /* oneLow        -3   */  3*564_ticks,
         /* startHigh     16   */ 16*564_ticks,
         /* startLow      -8   */  8*564_ticks,
         /* repeatTime   108m  */ 108000_ticks,
         /* repeatHigh         */ 16*564_ticks,
         /* repeatLow          */  4*564_ticks,

         /* packetLength  */    32,
         /* middleStopBit */    0,
         /* repeats       */    3,
         /* fastRepeats   */    true,
   };

public:
   /**
    * Common codes
    */
   enum Code : uint32_t {
      A_B                 =    0xA15EFF00,
      ANGLE               =    0xA758FF00,
      CLEAR               =    0xA35CFF00,
      DOWN                =    0xAA55FF00,
      DVD_USB             =    0xA45BFF00,
      EJECT               =    0xF708FF00,
      ENTER               =    0xAD52FF00,
      FORWARD             =    0xB748FF00,
      FORWARD_SCENE       =    0xB54AFF00,
      L_R                 =    0xA25DFF00,
      LANGUAGE            =    0xA659FF00,
      LEFT                =    0xAE51FF00,
      MENU                =    0xAB54FF00,
      MUTE                =    0xFA05FF00,
      N_P                 =    0xA25DFF00,
      NUM_10_PLUS         =    0xBB44FF00,
      NUM0                =    0xB946FF00,
      NUM1                =    0xF906FF00,
      NUM2                =    0xF807FF00,
      NUM3                =    0xF609FF00,
      NUM4                =    0xF50AFF00,
      NUM5                =    0xF40BFF00,
      NUM6                =    0xBF40FF00,
      NUM7                =    0xBE41FF00,
      NUM8                =    0xBD42FF00,
      NUM9                =    0xBC43FF00,
      ON_OFF              =    0xFB04FF00,
      OSD                 =    0xFE01FF00,
      PAUSE               =    0xB34CFF00,
      PBC                 =    0xA956FF00,
      PLAY                =    0xB44BFF00,
      PROG                =    0xA45BFF00,
      RANDOM              =    0xEC13FF00,
      REPEAT              =    0xA15EFF00,
      RESET               =    0xEE11FF00,
      RETURN              =    0xA55AFF00,
      REVERSE             =    0xB847FF00,
      REVERSE_SCENE       =    0xB649FF00,
      RIGHT               =    0xAC53FF00,
      RIPPING             =    0xEF10FF00,
      SETUP               =    0xB14EFF00,
      SLOW                =    0xA35CFF00,
      STOP                =    0xB24DFF00,
      SUBTITLE            =    0xA857FF00,
      TIME                =    0xFF00FF00,
      TITLE               =    0xAF50FF00,
      UP                  =    0xB04FFF00,
      VIDEO               =    0xBA45FF00,
      VOLUME_DOWN         =    0xFC03FF00,
      VOLUME_UP           =    0xFD02FF00,
      ZOOM                =    0xED12FF00,
   };

   /**
    * Start transmission of sequence.
    *
    * @param code       Command to send
    * @param repeats    Number of times to repeat (including original). 0 => use default for protocol
    */
   static void send(Code code, unsigned delay=0, unsigned repeats=0) {

      waitUntilComplete();

      console.writeln("Teac-DVD: 0x", code, Radix_16);

      data1        = code;
      data2        = 0;
      protocol     = p_TEAC;

      parameters   = myParameters;
      if (repeats != 0) {
         parameters.repeats = repeats;
      }

      IrRemote::start(delay);
   }

};

/**
 * Class to wrap CMT hardware for Samsung DVD protocol
 *
 * Samsung DVD protocol   IRP notation: {38k,500}<1,-1|1,-3>(9,-9,D:8,S:8,1,-9,E:4,F:8,-68u,~F:8,1,-118)+
 *
 * 38kHz carrier, Pulse distance code, bits transmitted LSB first
 *
 * Pulse interval encoding of bits:
 *
 *    <----- 4500 us -----> <----- 4500 us ----->
 *    +-------------------+                      +
 *    |xxxxxx 38kHz xxxxxx|                      |          Start = 9:-9
 *  --+-------------------+----------------------+
 *
 *    <---- 500 us ----> <---- 500 us ---->
 *    +-----------------+                  +
 *    |xxxx  38kHz  xxxx|                  |                Logic 0 = 1:-1
 *  --+-----------------+------------------+
 *
 *    <---- 500 us ----> <-------- 1500 us ---------->
 *    +-----------------+                            +
 *    |xxxx  38kHz  xxxx|                            |      Logic 1 = 1:-3
 *  --+-----------------+-------------//-------------+
 *
 *    <----- 500 us ---> <------ 4500 us ------>
 *    +-----------------+                      +
 *    |xxxx  38kHz  xxxx|                      |            Middle Stop = 1:-9
 *  --+-----------------+----------//----------+
 *
 *    <---- 500 us ----> <---- 500 us ---->
 *    +-----------------+                  +
 *    |xxxx  38kHz  xxxx|                  |                Stop = 1:-1
 *  --+-----------------+------------------+
 *
 *  Packet formats:
 *
 *    20 bit   <--   8 bits  --> <--  8 bits  -->     <--   4 +  8 bits     -->
 *    +-------+-----------------+---------------+----+----------------------+------+
 *    | Start |    Command      |    Address    | MS |       Extended       | Stop |
 *  --+-------+-----------------+---------------+----+----------------------+------+--
 */
class  IrSamsungDVD : public IrRemote {
private:

   /// Constant describing transmission
   /// IRP notation: {38k,500u}<1,-1|1,-3>(9,-9,D:8,S:8,1,-9,E:4,F:8,-68u,~F:8,1,-118)+
   static constexpr Parameters myParameters = {
         /* carrier       */    38_kHz,

         // Following are in CMT ticks = 1us, multiples of 500u
         /* zeroHigh      1  */    500_ticks,
         /* zeroLow      -1  */    500_ticks,
         /* oneHigh       1  */    500_ticks,
         /* oneLow       -3  */  3*500_ticks,
         /* startHigh     9  */  9*500_ticks,
         /* startLow     -9  */  9*500_ticks,
         /* repeatTime       */ 120000_ticks,
         /* repeatHigh       */  9*500_ticks, // Full repeat
         /* repeatLow        */  9*500_ticks,

         /* packetLength     */   16+20, // D:8,S:8,1,-9,E:4,F:8,G:8
         /* middleStopBit    */   16,    // Stop bit after 16th bit
         /* repeats          */    1,
         /* fastRepeats      */    false,
   };

public:
   enum Device : uint32_t {
      DVD = 0x0020,
   };

   /**
    * Common codes
    */
   enum Code : uint32_t {
      A_B                 =    0xD7287,
      ANGLE               =    0xCC337,
      AUDIO               =    0xDA257,
      BLUE                =    0xDB247,
      DOWN                =    0xE6197,
      EJECT               =    0xFE017,
      EXIT                =    0xD42B7,
      FORWARD             =    0xEA157,
      FORWARD_SCENE       =    0xEE117,
      GREEN               =    0xDD227,
      HOME                =    0xE9167,
      INFO                =    0xE11E7,
      LEFT                =    0xE41B7,
      MENU                =    0xE21D7,
      NUM0                =    0xF40B7,
      NUM1                =    0xFD027,
      NUM2                =    0xFC037,
      NUM3                =    0xFB047,
      NUM4                =    0xFA057,
      NUM5                =    0xF9067,
      NUM6                =    0xF8077,
      NUM7                =    0xF7087,
      NUM8                =    0xF6097,
      NUM9                =    0xF50A7,
      OK                  =    0xE31C7,
      ON_OFF              =    0xFF007,
      PAUSE               =    0xCD327,
      PLAY                =    0xEB147,
      RED                 =    0xDE217,
      REPEAT              =    0xD8277,
      RETURN              =    0xE8177,
      REVERSE             =    0xED127,
      REVERSE_SCENE       =    0xF20D7,
      RIGHT               =    0xE51A7,
      SCREEN              =    0xC6397,
      STOP                =    0xEC137,
      SUBTITLE            =    0xD9267,
      TITLE_MENU          =    0xDF207,
      TOOLS               =    0xC53A7,
      UP                  =    0xE7187,
      YELLOW              =    0xDC237,
   };

   IrSamsungDVD() = delete;
   IrSamsungDVD(const IrSamsungDVD &) = delete;

   //   IrLaserDVD() : IrRemote(cmtInitValue) {
   //   }

   /**
    * Start transmission of sequence.
    *
    * @param code          Command to send
    * @param device        Device
    * @param repeats       Number of times to repeat (including original). 0 => use default for protocol
    */
   static void send(Code code, unsigned delay=0, Device device=DVD, unsigned repeats=0) {

      waitUntilComplete();

      console.writeln("Samsung-DVD: 0x", code, Radix_16);

      data1        = device;
      data2        = code;
      protocol     = p_SAMSUNG_DVD;

      parameters   = myParameters;
      if (repeats != 0) {
         parameters.repeats = repeats;
      }
      IrRemote::start(delay);
   }

};

constexpr uint32_t SONY_LENGTH_MASK = 0xC000'0000;
constexpr uint32_t SONY_LENGTH_12   = 0x0000'0000;
constexpr uint32_t SONY_LENGTH_15   = 0x8000'0000;
constexpr uint32_t SONY_LENGTH_20   = 0x4000'0000;

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

class  IrSonyTV : public IrRemote {
private:

   IrSonyTV() = delete;
   IrSonyTV(const IrSonyTV &) = delete;

   /// Constant describing transmission
   static constexpr Parameters myParameters = {
         /* carrier       */    40_kHz,

         // Following are in CMT ticks = 1us, base unit for code is 600_ticks
         /* zeroHigh      */    600_ticks,
         /* zeroLow       */    600_ticks,
         /* oneHigh       */  2*600_ticks,
         /* oneLow        */    600_ticks,
         /* startHigh     */  4*600_ticks,
         /* startLow      */    600_ticks,
         /* repeatTime    */ 50'000_ticks,
         /* repeatHigh    */  4*600_ticks,  // Full repeats
         /* repeatLow     */    600_ticks,

         /* packetLength  */    0,     // Variable
         /* middleStopBit */    0,     // Not used
         /* repeats       */    3,
         /* fastRepeats   */    false, // Not used
   };

   /// Carrier half period in CMT clock cycles (Based on 8MHz CMT clock)
   static constexpr Ticks  PRIMARY_CARRIER_HALF_TIME = Ticks(8_MHz/myParameters.carrier/2);

   static constexpr Cmt::Init cmtInitValue {
      NvicPriority_Normal,
      cmtCallback,

      CmtMode_Time ,                                        // (cmt_msc_mode)   Mode of operation - Time
      CmtClockPrescaler_Auto ,                              // (cmt_pps_ppsdiv) Primary Prescaler Divider - Auto ~8MHz
      CmtIntermediatePrescaler_DivBy1 ,                     // (cmt_msc_cmtdiv) Intermediate frequency Prescaler - Intermediate frequency /4
      CmtOutput_ActiveHigh ,                                // (cmt_oc_output)  Output Control - Disabled
      CmtEndOfCycleAction_Interrupt ,                       // (cmt_dma_irq)    End of Cycle Event handling - Interrupt Request
      CmtPrimaryCarrierHighTime(PRIMARY_CARRIER_HALF_TIME), // (cmt_cgh1_ph)    Primary Carrier High Time Data Value
      CmtPrimaryCarrierLowTime(PRIMARY_CARRIER_HALF_TIME),  // (cmt_cgl1_pl)    Primary Carrier Low Time Data Value
      CmtMarkPeriod(myParameters.startHigh) ,               // (cmt_mark)       Mark period
      CmtSpacePeriod(myParameters.startLow),                // (cmt_space)      Space period
   };

public:
   /**
    * Common codes
    */
   enum Code : uint32_t {
      //                                  Length Code  Address
      APPS                 = makeSonyCode(15,    0x7D, 0x1A),
      AUDIO                = makeSonyCode(12,    0x17, 0x01),
      BLUE                 = makeSonyCode(15,    0x24, 0x97),
      CHANNEL_DOWN         = makeSonyCode(12,    0x11, 0x01),
      CHANNEL_UP           = makeSonyCode(12,    0x10, 0x01),
      DIGITAL_ANALOG       = makeSonyCode(15,    0x0D, 0x77),
      DISCOVER             = makeSonyCode(15,    0x73, 0x1A),
      DOWN                 = makeSonyCode(12,    0x75, 0x01),
      FOOTBALL             = makeSonyCode(15,    0x76, 0x1A),
      FORWARD              = makeSonyCode(15,    0x1C, 0x97),
      GREEN                = makeSonyCode(15,    0x26, 0x97),
      GUIDE                = makeSonyCode(15,    0x5B, 0xA4),
      HELP                 = makeSonyCode(15,    0x7B, 0x1A),
      HOME                 = makeSonyCode(12,    0x60, 0x01),
      I_PLUS               = makeSonyCode(12,    0x3A, 0x01),
      LEFT                 = makeSonyCode(12,    0x34, 0x01),
      MUTE                 = makeSonyCode(12,    0x14, 0x01),
      NUM0                 = makeSonyCode(12,    0x09, 0x01),
      NUM1                 = makeSonyCode(12,    0x00, 0x01),
      NUM2                 = makeSonyCode(12,    0x01, 0x01),
      NUM3                 = makeSonyCode(12,    0x02, 0x01),
      NUM4                 = makeSonyCode(12,    0x03, 0x01),
      NUM5                 = makeSonyCode(12,    0x04, 0x01),
      NUM6                 = makeSonyCode(12,    0x05, 0x01),
      NUM7                 = makeSonyCode(12,    0x06, 0x01),
      NUM8                 = makeSonyCode(12,    0x07, 0x01),
      NUM9                 = makeSonyCode(12,    0x08, 0x01),
      OK                   = makeSonyCode(12,    0x65, 0x01),
      ON_OFF               = makeSonyCode(12,    0x15, 0x01),
      ON                   = makeSonyCode(12,    0x2E, 0x01),
      OFF                  = makeSonyCode(12,    0x2F, 0x01),
      OPTIONS              = makeSonyCode(15,    0x36, 0x97),
      PAUSE                = makeSonyCode(15,    0x19, 0x97),
      PLAY                 = makeSonyCode(15,    0x1A, 0x97),
      RECORD               = makeSonyCode(15,    0x20, 0x97),
      RED                  = makeSonyCode(15,    0x25, 0x97),
      RELATED_SEARCH       = makeSonyCode(15,    0x7E, 0x1A),
      RETURN               = makeSonyCode(15,    0x23, 0x97),
      REVERSE              = makeSonyCode(15,    0x1B, 0x97),
      RIGHT                = makeSonyCode(12,    0x33, 0x01),
      SOCIAL_VIEW          = makeSonyCode(15,    0x74, 0x1A),
      SOURCE               = makeSonyCode(12,    0x25, 0x01),
/*
INPUT HDMI 1,Sony15,26,-1,90
INPUT HDMI 2,Sony15,26,-1,91
INPUT HDMI 3,Sony15,26,-1,92
INPUT HDMI 4,Sony15,26,-1,93
INPUT HDMI 5,Sony15,26,-1,94

INPUT HDMI 1 (2011),Sony12,26,-1,90
INPUT HDMI 2 (2011),Sony12,26,-1,91
INPUT HDMI 3 (2011),Sony12,26,-1,92
INPUT HDMI 4 (2011),Sony12,26,-1,93
INPUT HDMI 5 (2011),Sony12,26,-1,94
 */

      SOURCE_TV            = makeSonyCode(12,    36, 0x01), // ??
//      SOURCE_HDMI_1        = makeSonyCode(12,    90, 26), // ??
//      SOURCE_HDMI_2        = makeSonyCode(12,    91, 26), // ??
//      SOURCE_HDMI_3        = makeSonyCode(12,    92, 26), // ??
//      SOURCE_HDMI_4        = makeSonyCode(12,    93, 26), // ??
//      SOURCE_HDMI_5        = makeSonyCode(12,    94, 26), // ??
      SOURCE_HDMI_1        = makeSonyCode(15,    90, 26), // ??
      SOURCE_HDMI_2        = makeSonyCode(15,    91, 26), // ??
      SOURCE_HDMI_3        = makeSonyCode(15,    92, 26), // ??
      SOURCE_HDMI_4        = makeSonyCode(15,    93, 26), // ??
      SOURCE_HDMI_5        = makeSonyCode(15,    94, 26), // ??

      SOURCE_1             = makeSonyCode(12,    0x40, 0x01), // maybe?
      SOURCE_2             = makeSonyCode(12,    0x41, 0x01), // maybe?
      SOURCE_3             = makeSonyCode(12,    0x42, 0x01), // maybe?
      SOURCE_RGB1          = makeSonyCode(12,    0x43, 0x01), // maybe?
      SOURCE_RGB2          = makeSonyCode(12,    0x44, 0x01), // maybe?
      SOURCE_4             = makeSonyCode(12,    0x47, 0x01), // maybe?
      SOURCE_5             = makeSonyCode(12,    0x48, 0x01), // maybe?
      SOURCE_6             = makeSonyCode(12,    0x49, 0x01), // maybe?
      STANDBY              = makeSonyCode(12,    0x2F, 0x01), // maybe?
      STOP                 = makeSonyCode(15,    0x18, 0x97),
      SWAP                 = makeSonyCode(12,    0x3B, 0x01),
      SYNC_MENU            = makeSonyCode(15,    0x58, 0x1A),
      TITLE                = makeSonyCode(15,    0x65, 0x1A),
      TV_PAUSE             = makeSonyCode(15,    0x67, 0x1A),
      UNKNOWN              = makeSonyCode(15,    0x28, 0x97),
      UP                   = makeSonyCode(12,    0x74, 0x01),
      VOLUME_DOWN          = makeSonyCode(12,    0x13, 0x01),
      VOLUME_UP            = makeSonyCode(12,    0x12, 0x01),
      YELLOW               = makeSonyCode(15,    0x27, 0x97),
   };

   enum Address {
      tv             = 1,
      vcr1           = 2,
      teletext       = 3,
      widescreen     = 4,
      laserDisk      = 6,
      vcr2           = 7,
      vcr3           = 11,
      surroundSound  = 12,
      cassette       = 16,
      cdPlayer       = 17,
      equalizer      = 18,
      dvd            = 26,
      //   TV_Digital_Effects_15_bit            = 0xA4   ,
      //   PlayStation_2_DVD_20_bit             = 0x093A ,
      //   PlayStation_2_PS2_20_bit             = 0x1B5A ,
   };

   /**
    *
    * @param cmtCommand Command (command+address+length)
    * @param repeats    Number of times to repeat (including original). 0 => use default for protocol
    */
   static void send(Code cmtCommand, unsigned delay, unsigned repeats=3) {
      unsigned length = 0;

      IrRemote::waitUntilComplete();

      console.writeln("Sony-TV: 0x", (uint32_t)cmtCommand, Radix_16);

      data1        = cmtCommand&~SONY_LENGTH_MASK;
      data2        = 0;
      protocol     = p_SONY_TV;

      parameters = myParameters;
      if (repeats != 0) {
         parameters.repeats = repeats;
      }
      switch (cmtCommand&SONY_LENGTH_MASK) {
         case SONY_LENGTH_12: length = 12; break; // START + 7-bit COMMAND, 5-bit ADDRESS
         case SONY_LENGTH_15: length = 15; break; // START + 7-bit COMMAND, 8-bit ADDRESS
         case SONY_LENGTH_20: length = 20; break; // START + 7-bit COMMAND, 5-bit ADDRESS, 8-bit EXTENDED
      }
      parameters.packetLength = length;

      IrRemote::start(delay);
   }

};
} // End namespace USBDM

#endif /* SOURCES_IRREMOTE_H_ */

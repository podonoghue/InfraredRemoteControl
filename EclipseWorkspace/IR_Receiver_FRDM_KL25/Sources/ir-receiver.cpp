/**
 ============================================================================
 * @file    ir-receiver (180.ARM_Peripherals/Sources)
 * @brief   Basic C++ demo
 *
 *  Created on: 10/1/2016
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"
#include "cmt-remote.h"
#include "pit.h"
#include "Queue.h"

using namespace USBDM;

// DebugPin connection - change as required
using DebugPin   = Digital_D13;
using ResultPin  = Digital_D3;
using CapturePin = Digital_A0; // Inverted
using EventPin   = Digital_D4; // Indicates event

enum Protocol {
   Protocol_Unknown,
   Protocol_Sony,
   Protocol_NEC,
   Protocol_Samsung,
   Protocol_Panasonic,
};

class Event {

public:
   uint32_t prefix    = 0;
   uint32_t data      = 0;
   unsigned numBits   = 0;
   Protocol protocol  = Protocol_Unknown;
   Ticks    timeStamp = 0_ticks;

   Event() {
   }
};

Queue<Event, 20> eventQueue;

/**
 * Calculate interval in ticks/10
 *
 * @param count      Multiple of interval to use
 * @param interval   Base interval for sequence
 */
constexpr unsigned convert(unsigned count, unsigned interval) {
   return ((count*interval)+5)/10;
}

/**
 * Check is count equals target value within bounds ~+/- 12%
 *
 * @param count      Count to check
 * @param target     Target count value
 *
 * @return  True if count matches target interval
 */
constexpr bool checkWithBounds(unsigned count, unsigned target) {
   const unsigned min = (target*13)/16;
   const unsigned max = (target*19)/16;
   return (min<count)&&(count<max);
}

/**
 * Pit channel allocated to interval measurement
 */
static PitChannelNum pitChannelNum = PitChannelNum_None;

class Watcher {
public:
   Watcher() {
      DebugPin::set();
   }

   ~Watcher() {
      DebugPin::clear();
   }
};

/**
 * Call-back executed @10us interval
 */
void intervalHandler() {

//   Watcher w{};

//   return;

   enum State {
      s_hunt,              /// Hunting for start of sequence
      s_startHigh,        /// Confirming leader high time for NEC of Samsung codes

      s_startLowNec,       /// Confirming start bit low time
      s_sampleDataHighNec, /// Sampling data bit high
      s_sampleDataLowNec,  /// Sampling data bit low
      s_sampleStopHighNec, /// Sampling stop bit high
      s_sampleStopLowNec,  /// Sampling stop bit low

      s_startLowPanasonic,       /// Confirming start bit low time
      s_sampleDataHighPanasonic, /// Sampling data bit high
      s_sampleDataLowPanasonic,  /// Sampling data bit low
      s_sampleStopHighPanasonic, /// Sampling stop bit high
      s_sampleStopLowPanasonic,  /// Sampling stop bit low

      s_startLowSony,       /// Confirming start bit low time
      s_sampleDataHighSony, /// Sampling data bit high
      s_sampleDataLowSony,  /// Sampling data bit low
                            // No stop bit - use timeout

      s_startLowSamsung,               /// Confirming start bit low time
      s_sampleFirstWordHighSamsung,    /// Sampling data bit high 1st word
      s_sampleFirstWordLowSamsung,     /// Sampling data bit low 1st word
      s_sampleSecondWordHighSamsung,   /// Sampling data bit high 2nd word
      s_sampleSecondWordLowSamsung,    /// Sampling data bit low 2nd word
      s_sampleMiddleStopHighSamsung,   /// Sampling middle stop bit high
      s_sampleMiddleStopLowSamsung,    /// Sampling middle stop bit low
      s_sampleStopHighSamsung,         /// Sampling stop bit high
      s_sampleStopLowSamsung,          /// Sampling stop bit low
   };


   // Values are in Timer ticks = 10us
   constexpr unsigned START_HIGH_NEC             = convert(16, 564); /// ~  9000 us
   constexpr unsigned START_LOW_NEC              = convert( 8, 564); /// ~  4500 us
   constexpr unsigned DATA_HIGH_NEC              = convert( 1, 564); /// ~   564 us
   constexpr unsigned DATA0_LOW_NEC              = convert( 1, 564); /// ~   564 us
   constexpr unsigned DATA1_LOW_NEC              = convert( 3, 564); /// ~  1690 us
   constexpr unsigned STOP_HIGH_NEC              = convert( 1, 564); /// ~   564 us
   constexpr unsigned STOP_LOW_NEC               = convert( 1, 564); /// ~   564 us
   constexpr unsigned NEC_WORD_LENGTH  = 32;

   constexpr unsigned START_HIGH_PANASONIC             = convert( 8, 432); /// ~  3456 us
   constexpr unsigned START_LOW_PANASONIC              = convert( 4, 432); /// ~  1728 us
   constexpr unsigned DATA_HIGH_PANASONIC              = convert( 1, 432); /// ~   432 us
   constexpr unsigned DATA0_LOW_PANASONIC              = convert( 1, 432); /// ~   432 us
   constexpr unsigned DATA1_LOW_PANASONIC              = convert( 3, 432); /// ~  1296 us
   constexpr unsigned STOP_HIGH_PANASONIC              = convert( 1, 432); /// ~   432 us
   constexpr unsigned STOP_LOW_PANASONIC               = convert( 1, 432); /// ~   432 us
   constexpr unsigned PANASONIC_CHECK_LENGTH = 16;
   constexpr unsigned PANASONIC_WORD_LENGTH  = 48;  // 1st 16 discarded after check?

   constexpr unsigned START_HIGH_SAMSUNG         = convert( 9, 500); /// ~  4500 us
   constexpr unsigned START_LOW_SAMSUNG          = convert( 9, 500); /// ~  4500 us
   constexpr unsigned DATA_HIGH_SAMSUNG          = convert( 1, 500); /// ~   500 us
   constexpr unsigned DATA0_LOW_SAMSUNG          = convert( 1, 500); /// ~   500 us
   constexpr unsigned DATA1_LOW_SAMSUNG          = convert( 3, 500); /// ~  1500 us
   constexpr unsigned MIDDLE_STOP_HIGH_SAMSUNG   = convert( 1, 500); /// ~   500 us
   constexpr unsigned MIDDLE_STOP_LOW_SAMSUNG    = convert( 9, 500); /// ~  4500 us
   constexpr unsigned STOP_HIGH_SAMSUNG          = convert( 1, 500); /// ~   500 us
   constexpr unsigned STOP_LOW_SAMSUNG           = convert( 1, 500); /// ~   500 us
   constexpr unsigned SAMSUNG_WORD1_LENGTH  = 16;
   constexpr unsigned SAMSUNG_WORD2_LENGTH  = 20;

   constexpr unsigned START_HIGH_SONY            = convert( 4, 600); /// ~ 2400 us
   constexpr unsigned START_LOW_SONY             = convert( 1, 600); /// ~  600 us
   constexpr unsigned DATA0_HIGH_SONY            = convert( 1, 600); /// ~  600 us
   constexpr unsigned DATA1_HIGH_SONY            = convert( 2, 600); /// ~  600 us
   constexpr unsigned DATA_LOW_SONY              = convert( 1, 600); /// ~  600 us
   constexpr unsigned DATA_LOW_TIMEOUT_SONY      = convert( 2, 600); /// ~ 1200 us
   constexpr unsigned SONY_WORD_LENGTH1  = 12;
   constexpr unsigned SONY_WORD_LENGTH2  = 15;
   constexpr unsigned SONY_WORD_LENGTH3  = 20;

   constexpr unsigned DATA_TIMEOUT =       convert(1,20000);  /// ~ 20000 us

   constexpr unsigned FILTER       =       convert(1,100);    /// ~   200 us

   static unsigned counter      = 0;
   static State    state        = s_hunt;
   static unsigned filter       = 0;
   static uint32_t sampleData   = 0;
   static uint32_t bitNum       = 0;
   static uint32_t bitMask      = 0b1;

   bool lastSample;
   static bool sample;

   // Current event to assemble
   static Event event;

//   Timer::clearEventFlag();

   lastSample = sample;
   sample     = CapturePin::isLow(); // Inverted

   if (lastSample == sample) {
      filter++;
   }
   else {
      filter = 0;
   }
   counter++;

//   DebugPin::toggle();

   // Only do check after N consistent samples (100us)
   if (filter<FILTER) {
      return;
   }
   DebugPin::write(sample);

   static Ticks eventStart = 0_ticks;

   switch(state) {
      /*
       * Common
       */
      case s_hunt       : // Hunting for start of sequence
         ResultPin::clear();
         if (sample) {
            state = s_startHigh;
            counter      = 0;
            sampleData   = 0;
            event.prefix = 0;
            event.data   = 0;
            event.protocol = Protocol_Unknown;
            bitNum       = 0;
            bitMask      = 0b1;
            eventStart   = Lptmr0::getCounterValue();
         }
         break;

      case s_startHigh : // Checking leader high time
         if (!sample) {
            if (checkWithBounds(counter, START_HIGH_NEC)) {
               state = s_startLowNec;
            }
            else if (checkWithBounds(counter, START_HIGH_SAMSUNG)) {
               state = s_startLowSamsung;
            }
            else if (checkWithBounds(counter, START_HIGH_SONY)) {
               state = s_startLowSony;
            }
            else if (checkWithBounds(counter, START_HIGH_PANASONIC)) {
               state = s_startLowPanasonic;
            }
            else {
               // Failed
               state = s_hunt;
               break;
            }
            counter = 0;
         }
         else {
            // No timeout on high
         }
         break;

         /*
          * Sony code
          */
      case s_startLowSony  : // Checking leader low time
         if (sample) {
            if (checkWithBounds(counter, START_LOW_SONY)) {
               state = s_sampleDataHighSony;
//               ResultPin::set();
            }
            else {
               // Failed
               state = s_hunt;
            }
            counter = 0;
         }
         else if (counter>DATA_TIMEOUT) {
            // Timeout - Failed
            state = s_hunt;
            break;
         }
         break;

      case s_sampleDataHighSony : // Sampling data high
         if (!sample) {
            state = s_sampleDataLowSony;
            if (checkWithBounds(counter, DATA0_HIGH_SONY)) {
               // No action
            }
            else if (checkWithBounds(counter, DATA1_HIGH_SONY)) {
               sampleData = sampleData|bitMask;
            }
            else {
               // Failed
               state = s_hunt;
               break;
            }
            counter = 0;
            bitNum++;
            bitMask <<=1;
         }
         else {
            // No timeout on high
         }
         break;

      case s_sampleDataLowSony  : // Sampling data low
         if (sample) {
            if (checkWithBounds(counter, DATA_LOW_SONY)) {
               state = s_sampleDataHighSony;
            }
            else {
               // Failed
               state = s_hunt;
               break;
            }
            counter = 0;
         }
         else if (counter>DATA_LOW_TIMEOUT_SONY) {
            // No stop bit on Sony - use timeout
            state = s_hunt;

            if ((bitNum == SONY_WORD_LENGTH1)||(bitNum == SONY_WORD_LENGTH2)||(bitNum == SONY_WORD_LENGTH3)) {
               // Save data
               event.data      = sampleData;
               event.numBits   = bitNum;
               event.prefix    = 0;
               event.protocol  = Protocol_Sony;
               event.timeStamp = eventStart;
               eventQueue.enQueue(event);
            }
            else {
               // Failed
               state = s_hunt;
               break;
            }
//            ResultPin::clear();

            break;
         }
         break;

         /*
          * NEC code
          */
      case s_startLowNec  : // Checking leader low time
         if (sample) {
            if (checkWithBounds(counter, START_LOW_NEC)) {
               state = s_sampleDataHighNec;
               ResultPin::set();
            }
            else {
               // Failed
               state = s_hunt;
               break;
            }
            counter = 0;
         }
         else if (counter>DATA_TIMEOUT) {
            // Timeout - Failed
            state = s_hunt;
            break;
         }
         break;

      case s_sampleDataHighNec : // Sampling data high
         if (!sample) {
            if (checkWithBounds(counter, DATA_HIGH_NEC)) {
               state = s_sampleDataLowNec;
            }
            else {
               // Failed
               state = s_hunt;
               break;
            }
            counter = 0;
         }
         else {
            // No timeout on high
         }
         break;

      case s_sampleDataLowNec  : // Sampling data low
         if (sample) {
            if (checkWithBounds(counter, DATA0_LOW_NEC)) {
               // No action
            }
            else if (checkWithBounds(counter, DATA1_LOW_NEC)) {
               sampleData = sampleData|bitMask;
            }
            else {
               // Failed
               state = s_hunt;
               break;
            }
            counter = 0;
            bitNum++;
            bitMask <<=1;
            if (bitNum == NEC_WORD_LENGTH) {
               state = s_sampleStopHighNec;
            }
            else {
               state = s_sampleDataHighNec;
            }
         }
         else if (counter>DATA_TIMEOUT) {
            // Timeout - Failed
            state = s_hunt;
            break;
         }
         break;

      case s_sampleStopHighNec: // Checking stop bit high
         if (!sample) {
            if (checkWithBounds(counter, STOP_HIGH_NEC)) {
               state = s_sampleStopLowNec;
            }
            else {
               // Failed
               state = s_hunt;
               break;
            }
            counter = 0;
         }
         else {
            // No timeout on high
         }
         break;

      case s_sampleStopLowNec: // Checking stop bit low

         if (sample) {
            // Failed STOP low time
            counter = 0;
            state = s_hunt;
            break;
         }
         else if (counter>STOP_LOW_NEC) {
            // Complete
            state = s_hunt;

            // Save data (discards stop bit)
            event.data      = sampleData;
            event.prefix    = 0;
            event.protocol = Protocol_NEC;
            event.timeStamp = eventStart;
            eventQueue.enQueue(event);
//            ResultPin::clear();
         }
         break;

         /*
          * Samsung
          */
      case s_startLowSamsung  : /// Confirming leader low time
         if (sample) {
            if (checkWithBounds(counter, START_LOW_SAMSUNG)) {
               state = s_sampleFirstWordHighSamsung;
//               ResultPin::set();
            }
            else {
               // Failed
               state = s_hunt;
            }
            counter = 0;
         }
         else if (counter>DATA_TIMEOUT) {
            // Timeout - Failed
            state = s_hunt;
         }
         break;

      case s_sampleFirstWordHighSamsung : /// Sampling data high
         if (!sample) {
            if (checkWithBounds(counter, DATA_HIGH_SAMSUNG)) {
               state = s_sampleFirstWordLowSamsung;
            }
            else {
               // Failed
               state = s_hunt;
            }
            counter = 0;
         }
         else {
            // No timeout on high
         }
         break;

      case s_sampleFirstWordLowSamsung  : /// Sampling data low
         if (sample) {
            if (checkWithBounds(counter, DATA0_LOW_SAMSUNG)) {
               // No action
            }
            else if (checkWithBounds(counter, DATA1_LOW_SAMSUNG)) {
               sampleData = sampleData|bitMask;
            }
            else {
               // Failed
               state = s_hunt;
               break;
            }
            counter = 0;
            bitNum++;
            bitMask <<=1;
            if (bitNum == SAMSUNG_WORD1_LENGTH) {
               state = s_sampleMiddleStopHighSamsung;
            }
            else {
               state = s_sampleFirstWordHighSamsung;
            }
         }
         else if (counter>DATA_TIMEOUT) {
            // Timeout - Failed
            state = s_hunt;
         }
         break;

      case s_sampleMiddleStopHighSamsung: // Checking stop bit high
         if (!sample) {
            if (checkWithBounds(counter, MIDDLE_STOP_HIGH_SAMSUNG)) {
               state = s_sampleMiddleStopLowSamsung;
            }
            else {
               // Failed
               state = s_hunt;
               break;
            }
            counter = 0;
         }
         else {
            // No timeout on high
         }
         break;

      case s_sampleMiddleStopLowSamsung: // Checking stop bit low
         if (sample) {
            if (checkWithBounds(counter, MIDDLE_STOP_LOW_SAMSUNG)) {
               state      = s_sampleSecondWordHighSamsung;
               event.prefix     = sampleData;
               sampleData = 0;
               bitNum     = 0;
               bitMask    = 0b1;
            }
            else {
               // Failed
               state = s_hunt;
               break;
            }
            counter = 0;
         }
         else if (counter>DATA_TIMEOUT) {
            // Timeout - Failed
            state = s_hunt;
         }
         break;

      case s_sampleSecondWordHighSamsung : /// Sampling data high
         if (!sample) {
            if (checkWithBounds(counter, DATA_HIGH_SAMSUNG)) {
               state = s_sampleSecondWordLowSamsung;
            }
            else {
               // Failed
               state = s_hunt;
            }
            counter = 0;
         }
         else {
            // No timeout on high
         }
         break;

      case s_sampleSecondWordLowSamsung  : /// Sampling data low
         if (sample) {
            if (checkWithBounds(counter, DATA0_LOW_SAMSUNG)) {
               // No action
            }
            else if (checkWithBounds(counter, DATA1_LOW_SAMSUNG)) {
               sampleData = sampleData|bitMask;
            }
            else {
               // Failed
               state = s_hunt;
               break;
            }
            counter = 0;
            bitNum++;
            bitMask <<=1;
            if (bitNum == SAMSUNG_WORD2_LENGTH) {
               state = s_sampleStopHighSamsung;
            }
            else {
               state = s_sampleSecondWordHighSamsung;
            }
         }
         else if (counter>DATA_TIMEOUT) {
            // Timeout - Failed
            state = s_hunt;
         }
         break;


      case s_sampleStopHighSamsung: // Checking stop bit high
         if (!sample) {
            if (checkWithBounds(counter, STOP_HIGH_SAMSUNG)) {
               state = s_sampleStopLowSamsung;
            }
            else {
               // Failed
               state = s_hunt;
               break;
            }
            counter = 0;
         }
         else {
            // No timeout on high
         }
         break;

      case s_sampleStopLowSamsung: // Checking stop bit low

         if (sample) {
            // Failed STOP low time
            counter = 0;
            state = s_hunt;
         }
         else if (counter>STOP_LOW_SAMSUNG) {
            // Complete
            state = s_hunt;

            // Save data (discards stop bit)
            event.data      = sampleData;
            event.protocol  = Protocol_Samsung;
            event.timeStamp = eventStart;
            eventQueue.enQueue(event);
//            ResultPin::clear();
         }
         break;

         /*
          * Panasonic code
          */
      case s_startLowPanasonic  : // Checking leader low time
         if (sample) {
            if (checkWithBounds(counter, START_LOW_PANASONIC)) {
//               ResultPin::set();
               state = s_sampleDataHighPanasonic;
            }
            else {
               // Failed
               state = s_hunt;
               break;
            }
            counter = 0;
         }
         else if (counter>DATA_TIMEOUT) {
            // Timeout - Failed
            state = s_hunt;
            break;
         }
         break;

      case s_sampleDataHighPanasonic : // Sampling data high
         if (!sample) {
            state = s_sampleDataLowPanasonic;
         }
         else if (counter>DATA_TIMEOUT) {
            // Timeout - Failed
            state = s_hunt;
            break;
         }
         break;

      case s_sampleDataLowPanasonic  : // Sampling data low
         if (sample) {
            if (checkWithBounds(counter, DATA_HIGH_PANASONIC+DATA0_LOW_PANASONIC)) {
               // No action
            }
            else if (checkWithBounds(counter, DATA1_LOW_PANASONIC+DATA0_LOW_PANASONIC)) {
               sampleData = sampleData|bitMask;
            }
            else {
               // Failed
               state = s_hunt;
               break;
            }
            counter = 0;
            bitNum++;
            bitMask <<=1;
            if (bitNum == PANASONIC_CHECK_LENGTH) {
               bitMask    = 0b1;
               event.prefix     = sampleData;
               sampleData = 0;
            }
            if (bitNum == PANASONIC_WORD_LENGTH) {
               state = s_sampleStopHighPanasonic;
            }
            else {
               state = s_sampleDataHighPanasonic;
            }
         }
         else if (counter>DATA_TIMEOUT) {
            // Timeout - Failed
            state = s_hunt;
            break;
         }
         break;

      case s_sampleStopHighPanasonic: // Checking stop bit high
         if (!sample) {
            state = s_sampleStopLowPanasonic;
         }
         else if (counter>DATA_TIMEOUT) {
            // Timeout - Failed
            state = s_hunt;
            break;
         }
         break;

      case s_sampleStopLowPanasonic: // Checking stop bit low

         if (sample) {
            // Failed STOP low time
            counter = 0;
            state = s_hunt;
            break;
         }
         else if (counter>(STOP_LOW_PANASONIC+STOP_HIGH_PANASONIC)) {
            // Complete
            state = s_hunt;

            // Save data (discards stop bit)
            event.data      = sampleData;
            event.protocol  = Protocol_Panasonic;
            event.timeStamp = eventStart;
            eventQueue.enQueue(event);
//            ResultPin::clear();
         }
         break;
   }
}

void configureIntervalHardware() {

   CapturePin::setInput();

   static constexpr Pit::Init PitInit {
      PitDebugMode_StopInDebug ,   // (pit_mcr_frz)              Freeze in Debug - Timers stop in Debug
      NvicPriority_High,           // (irqLevel)                 IRQ priority level - High
   };

   static constexpr Pit::ChannelInit TimerInit {

      PitChannelEnable_Enabled ,   // (pit_tctrl_ten[0])         Timer Channel Enable - Channel enabled
      PitChannelAction_Interrupt , // (pit_tctrl_tie[0])         Action on timer event - None
      239_ticks,                   // (pit_ldval_tsv[0])         Reload value channel 0

      intervalHandler,             // (handlerName_Ch0)          User declared event handler
      };

   pitChannelNum = Pit::allocateChannel();
   checkError();
   Pit::configure(PitInit);
   Pit::configure(pitChannelNum, TimerInit);

   /**
    * Timer initialisation value
    */
   static constexpr Timer::TimeIntervalModeInit TimerInitValue {
//      NvicPriority_Normal ,                  // (irqLevel)                 IRQ priority level - Normal
//      intervalHandler,                       // (handlerName)              User declared event handler

      LptmrCounterActionOnEvent_None ,       // (lptmr_csr_tfc)            Counter Action on Compare Event - counter reset
      LptmrEventAction_None,                 // (lptmr_csr_tie)            Timer action on event - Interrupt
      LptmrClockSel_Lpoclk ,                 // (lptmr_psr_pcs)            Clock source for LPTMR - Oscillator External Reference Clock (OSCERCLK)
      LptmrPrescale_Direct ,                 // (lptmr_psr_prescaler)      Prescaler Value - Prescaler = 1
      65535_ticks,                           // (lptmr_cmr_compare)        Timer Compare Value = ~65 s
   };

   Timer::configure(TimerInitValue);
}

struct CodeNameEntry {
   uint32_t code;
   const char *name;
};

const char*findLaserDVDName(uint32_t code) {
   static CodeNameEntry codeNameEntry[] = {
         0xFF00FF00, "EJECT",
         0xF30CFF00, "ON_OFF",
         0xBC43FF00, "AUDIO",
         0xA35CFF00, "MUTE",
         0xFE01FF00, "SUBTITLE",
         0xF40BFF00, "MENU",
         0xF20DFF00, "NUM1",
         0xF609FF00, "NUM2",
         0xFA05FF00, "NUM3",
         0xA25DFF00, "OSD",
         0xB04FFF00, "NUM4",
         0xB44BFF00, "NUM5",
         0xB847FF00, "NUM6",
         0xEA15FF00, "COPY_DELETE",
         0xB14EFF00, "NUM7",
         0xB54AFF00, "NUM8",
         0xB946FF00, "NUM9",
         0xB24DFF00, "NUM0",
         0xFC03FF00, "SETUP",
         0xEE11FF00, "RETURN",
         0xF708FF00, "VOL_DOWN",
         0xFB04FF00, "VOL_UP",
         0xE817FF00, "PAUSE_PLAY",
         0xF50AFF00, "STOP",
         0xA758FF00, "SLOW",
         0xBA45FF00, "SEARCH",
         0xED12FF00, "STEP",
         0xAB54FF00, "CLEAR",
         0xEC13FF00, "MARK",
         0xE916FF00, "Q_PLAY",
         0xAA55FF00, "A_B",
         0xBE41FF00, "ZOOM",
         0xA857FF00, "REVERSE",
         0xEF10FF00, "FORWARD",
         0xA45BFF00, "REVERSE_SCENE",
         0xE31CFF00, "FORWARD_SCENE",
         0xAE51FF00, "REPEAT",
         0xE619FF00, "PBC",
         0xE718FF00, "CHANNEL",
         0xF00FFF00, "ANGLE",
         0xA659FF00, "VIDEO",
         0xF807FF00, "DVD_USB",
         0xBD42FF00, "PROG",
         0xAF50FF00, "TITLE",
         0xBB44FF00, "UP",
         0xB748FF00, "DOWN",
         0xB34CFF00, "LEFT",
         0xBF40FF00, "RIGHT",
         0xF906FF00, "OK",
         0xEB14FF00, "PAUSE",
         0xA05FFF00, "PLAY",
   };


   const char *name="Unknown";

   for (unsigned index=0; index<(sizeof(codeNameEntry)/sizeof(codeNameEntry[0])); index++) {
      if (code == codeNameEntry[index].code) {
         name = codeNameEntry[index].name;
         break;
      }
   }
   return name;
}

const char*findTeacDVDName(uint32_t code) {
   static CodeNameEntry codeNameEntry[] = {
         0xA15EFF00, "A_B",
         0xA758FF00, "ANGLE",
         0xA35CFF00, "CLEAR",
         0xAA55FF00, "DOWN",
         0xA45BFF00, "DVD_USB",
         0xF708FF00, "EJECT",
         0xAD52FF00, "ENTER",
         0xB748FF00, "FORWARD",
         0xB54AFF00, "FORWARD_SCENE",
         0xA25DFF00, "L_R",
         0xA659FF00, "LANGUAGE",
         0xAE51FF00, "LEFT",
         0xAB54FF00, "MENU",
         0xFA05FF00, "MUTE",
         0xA25DFF00, "N_P",
         0xBB44FF00, "NUM_10_PLUS",
         0xB946FF00, "NUM0",
         0xF906FF00, "NUM1",
         0xF807FF00, "NUM2",
         0xF609FF00, "NUM3",
         0xF50AFF00, "NUM4",
         0xF40BFF00, "NUM5",
         0xBF40FF00, "NUM6",
         0xBE41FF00, "NUM7",
         0xBD42FF00, "NUM8",
         0xBC43FF00, "NUM9",
         0xFB04FF00, "ON_OFF",
         0xFE01FF00, "OSD",
         0xB34CFF00, "PAUSE",
         0xA956FF00, "PBC",
         0xB44BFF00, "PLAY",
         0xA45BFF00, "PROG",
         0xEC13FF00, "RANDOM",
         0xA15EFF00, "REPEAT",
         0xEE11FF00, "RESET",
         0xA55AFF00, "RETURN",
         0xB847FF00, "REVERSE",
         0xB649FF00, "REVERSE_SCENE",
         0xAC53FF00, "RIGHT",
         0xEF10FF00, "RIPPING",
         0xB14EFF00, "SETUP",
         0xA35CFF00, "SLOW",
         0xB24DFF00, "STOP",
         0xA857FF00, "SUBTITLE",
         0xFF00FF00, "TIME",
         0xAF50FF00, "TITLE",
         0xB04FFF00, "UP",
         0xBA45FF00, "VIDEO",
         0xFC03FF00, "VOL_DOWN",
         0xFD02FF00, "VOL_UP",
         0xED12FF00, "ZOOM",
   };

   const char *name="Unknown";

   for (unsigned index=0; index<(sizeof(codeNameEntry)/sizeof(codeNameEntry[0])); index++) {
      if (code == codeNameEntry[index].code) {
         name = codeNameEntry[index].name;
         break;
      }
   }
   return name;
}

const char*findBlaupunktDVDName(uint32_t code) {
   static CodeNameEntry codeNameEntry[] = {
         0xB946FF00, "A_B",
         0xF00FFF00, "ANGLE",
         0xBD42FF00, "AUDIO",
         0xFE01FF00, "DOWN",
         0xB44BFF00, "EJECT",
         0xB34CFF00, "FORWARD",
         0xBF40FF00, "FORWARD_SCENE",
         0xA25DFF00, "GOTO",
         0xE916FF00, "L_R",
         0xF30CFF00, "LEFT",
         0xE41BFF00, "MENU",
         0xE21DFF00, "MUTE",
         0xAA55FF00, "NUM0",
         0xEC13FF00, "NUM1",
         0xEE11FF00, "NUM10_PLUS",
         0xB748FF00, "NUM2",
         0xA15EFF00, "NUM3",
         0xA05FFF00, "NUM4",
         0xB649FF00, "NUM5",
         0xAC53FF00, "NUM6",
         0xB04FFF00, "NUM7",
         0xAF50FF00, "NUM8",
         0xBB44FF00, "NUM9",
         0xF20DFF00, "OK",
         0xFF00FF00, "ON_OFF",
         0xE11EFF00, "OSD",
         0xEB14FF00, "P_N",
         0xF906FF00, "PLAY_PAUSE",
         0xE01FFF00, "PROG",
         0xF807FF00, "REPEAT",
         0xAB54FF00, "RESET",
         0xB14EFF00, "RETURN",
         0xA956FF00, "REVERSE",
         0xE51AFF00, "REVERSE_SCENE",
         0xFA05FF00, "RIGHT",
         0xF10EFF00, "SETUP",
         0xED12FF00, "SLOW",
         0xFB04FF00, "STEP",
         0xA857FF00, "STOP",
         0xBC43FF00, "SUBTITLE",
         0xF609FF00, "TITLE",
         0xFD02FF00, "UP",
         0xEA15FF00, "USB",
         0xF40BFF00, "VOLUME_DOWN",
         0xF50AFF00, "VOLUME_UP",
         0xE718FF00, "ZOOM",
};

   const char *name="Unknown";

   for (unsigned index=0; index<(sizeof(codeNameEntry)/sizeof(codeNameEntry[0])); index++) {
      if (code == codeNameEntry[index].code) {
         name = codeNameEntry[index].name;
         break;
      }
   }
   return name;
}

const char*findTeacPVRName(uint32_t code) {
   static CodeNameEntry codeNameEntry[] = {
         0xA659BF00, "ON_OFF",
         0xA758BF00, "REC",
         0xE718BF00, "LIST",
         0xE619BF00, "MUTE",
         0xB24DBF00, "EPG",
         0xF10EBF00, "INFO",
         0xF20DBF00, "TTX",
         0xAE51BF00, "AUDIO",
         0xEE11BF00, "SUBTITLE",
         0xBA45BF00, "MENU",
         0xFA05BF00, "EXIT",
         0xAA55BF00, "FAV",
         0xEA15BF00, "TV_RADIO",
         0xF906BF00, "UP",
         0xE916BF00, "DOWN",
         0xA55ABF00, "LEFT",
         0xE41BBF00, "RIGHT",
         0xE51ABF00, "OK",
         0xAD52BF00, "NUM1",
         0xAF50BF00, "NUM2",
         0xEF10BF00, "NUM3",
         0xA956BF00, "NUM4",
         0xAB54BF00, "NUM5",
         0xEB14BF00, "NUM6",
         0xB14EBF00, "NUM7",
         0xB34CBF00, "NUM8",
         0xF30CBF00, "NUM9",
         0xF00FBF00, "NUM0",
         0xEC13BF00, "RECALL",
         0xE817BF00, "GOTO",
         0xB54ABF00, "REVERSE",
         0xB748BF00, "FORWARD",
         0xF708BF00, "REVERSE_SCENE",
         0xF40BBF00, "FORWARD_SCENE",
         0xB946BF00, "PLAY",
         0xBB44BF00, "PAUSE",
         0xFB04BF00, "STOP",
         0xF807BF00, "REPEAT",
         0xBD42BF00, "RED",
         0xBF40BF00, "GREEN",
         0xFF00BF00, "YELLOW",
         0xFC03BF00, "BLUE",
   };


   const char *name="Unknown";

   for (unsigned index=0; index<(sizeof(codeNameEntry)/sizeof(codeNameEntry[0])); index++) {
      if (code == codeNameEntry[index].code) {
         name = codeNameEntry[index].name;
         break;
      }
   }
   return name;
}

const char*findSamsungDVDName(uint32_t device, uint32_t code) {
   static CodeNameEntry codeNameEntry[] = {
         0xD7287, "A_B",
         0xCC337, "ANGLE",
         0xDA257, "AUDIO",
         0xDB247, "BLUE",
         0xE6197, "DOWN",
         0xFE017, "EJECT",
         0xD42B7, "EXIT",
         0xEA157, "FORWARD",
         0xEE117, "FORWARD_SCENE",
         0xDD227, "GREEN",
         0xE9167, "HOME",
         0xE11E7, "INFO",
         0xE41B7, "LEFT",
         0xE21D7, "MENU",
         0xF40B7, "NUM0",
         0xFD027, "NUM1",
         0xFC037, "NUM2",
         0xFB047, "NUM3",
         0xFA057, "NUM4",
         0xF9067, "NUM5",
         0xF8077, "NUM6",
         0xF7087, "NUM7",
         0xF6097, "NUM8",
         0xF50A7, "NUM9",
         0xE31C7, "OK",
         0xFF007, "ON_OFF",
         0xCD327, "PAUSE",
         0xEB147, "PLAY",
         0xDE217, "RED",
         0xD8277, "REPEAT",
         0xE8177, "RETURN",
         0xED127, "REVERSE",
         0xF20D7, "REVERSE_SCENE",
         0xE51A7, "RIGHT",
         0xC6397, "SCREEN",
         0xEC137, "STOP",
         0xD9267, "SUBTITLE",
         0xDF207, "TITLE_MENU",
         0xC53A7, "TOOLS",
         0xE7187, "UP",
         0xDC237, "YELLOW",

   };

   const char *name="Unknown";

   if (device != 0x20) {
      // Not a DVD
      return "Unknown Device";
   }
   for (unsigned index=0; index<(sizeof(codeNameEntry)/sizeof(codeNameEntry[0])); index++) {
      if (code == codeNameEntry[index].code) {
         name = codeNameEntry[index].name;
         break;
      }
   }
   return name;
}

const char*findPanasonicDVDName(uint32_t device, uint32_t code) {
   static CodeNameEntry codeNameEntry[] = {
         0xF84800B0, "A_B",
         0x833300B0, "AUDIO",
         0x338300B0, "CANCEL",
         0x229200B0, "DISPLAY",
         0x368600B0, "DOWN",
         0xB10100B0, "EJECT",
         0xB50500B0, "FORWARD",
         0xFA4A00B0, "FORWARD_SCENE",
         0x378700B0, "LEFT",
         0x308000B0, "MENU",
         0xA91900B0, "NUM0",
         0xA01000B0, "NUM1",
         0x398900B0, "NUM10_PLUS",
         0xA11100B0, "NUM2",
         0xA21200B0, "NUM3",
         0xA31300B0, "NUM4",
         0xA41400B0, "NUM5",
         0xA51500B0, "NUM6",
         0xA61600B0, "NUM7",
         0xA71700B0, "NUM8",
         0xA81800B0, "NUM9",
         0x8F3F00B0, "OFF",
         0x328200B0, "OK",
         0x8E3E00B0, "ON",
         0x8D3D00B0, "ON_OFF",
         0xB60600B0, "PAUSE",
         0xBA0A00B0, "PAUSE_PLAY",
         0xBA0A00B0, "PLAY",
         0xFD4D00B0, "PROG",
         0x209000B0, "RANDOM",
         0x3C8C00B0, "REPEAT",
         0x318100B0, "RETURN",
         0xB40400B0, "REVERSE",
         0xF94900B0, "REVERSE_SCENE",
         0x388800B0, "RIGHT",
         0x56E600B0, "SEARCH",
         0x249400B0, "SETUP",
         0xBF0F00B0, "SLOW",
         0xBC0C00B0, "STEP",
         0xB00000B0, "STOP",
         0x219100B0, "SUBTITLE",
         0x2B9B00B0, "TITLE",
         0x358500B0, "UP",
         0xB20200B0, "USB",
         0x3A8A00B0, "USB_REC",
         0x71C100B0, "ZOOM",
};

   const char *name="Unknown";

   if (device != 0x2002) {
      // Not a DVD
      return "Unknown Device";
   }
   for (unsigned index=0; index<(sizeof(codeNameEntry)/sizeof(codeNameEntry[0])); index++) {
      if (code == codeNameEntry[index].code) {
         name = codeNameEntry[index].name;
         break;
      }
   }
   return name;
}

const char*findSonyTvName(uint32_t code) {

   static CodeNameEntry codeNameEntry[] = {
         IrRemote::makeSonyCode(15,    0x7D, 0x1A), "APPS",
         IrRemote::makeSonyCode(12,    0x17, 0x01), "AUDIO",
         IrRemote::makeSonyCode(15,    0x24, 0x97), "BLUE",
         IrRemote::makeSonyCode(12,    0x11, 0x01), "CHANNEL_DOWN",
         IrRemote::makeSonyCode(12,    0x10, 0x01), "CHANNEL_UP",
         IrRemote::makeSonyCode(15,    0x0D, 0x77), "DIGITAL_ANALOG",
         IrRemote::makeSonyCode(15,    0x73, 0x1A), "DISCOVER",
         IrRemote::makeSonyCode(12,    0x75, 0x01), "DOWN",
         IrRemote::makeSonyCode(15,    0x76, 0x1A), "FOOTBALL",
         IrRemote::makeSonyCode(15,    0x1C, 0x97), "FORWARD",
         IrRemote::makeSonyCode(15,    0x26, 0x97), "GREEN",
         IrRemote::makeSonyCode(15,    0x5B, 0xA4), "GUIDE",
         IrRemote::makeSonyCode(15,    0x7B, 0x1A), "HELP",
         IrRemote::makeSonyCode(12,    0x60, 0x01), "HOME",
         IrRemote::makeSonyCode(12,    0x3A, 0x01), "I_PLUS",
         IrRemote::makeSonyCode(12,    0x34, 0x01), "LEFT",
         IrRemote::makeSonyCode(12,    0x14, 0x01), "MUTE",
         IrRemote::makeSonyCode(12,    0x09, 0x01), "NUM0",
         IrRemote::makeSonyCode(12,    0x00, 0x01), "NUM1",
         IrRemote::makeSonyCode(12,    0x01, 0x01), "NUM2",
         IrRemote::makeSonyCode(12,    0x02, 0x01), "NUM3",
         IrRemote::makeSonyCode(12,    0x03, 0x01), "NUM4",
         IrRemote::makeSonyCode(12,    0x04, 0x01), "NUM5",
         IrRemote::makeSonyCode(12,    0x05, 0x01), "NUM6",
         IrRemote::makeSonyCode(12,    0x06, 0x01), "NUM7",
         IrRemote::makeSonyCode(12,    0x07, 0x01), "NUM8",
         IrRemote::makeSonyCode(12,    0x08, 0x01), "NUM9",
         IrRemote::makeSonyCode(12,    0x65, 0x01), "OK",
         IrRemote::makeSonyCode(12,    0x15, 0x01), "ON_OFF",
         IrRemote::makeSonyCode(12,    0x2E, 0x01), "ON",
         IrRemote::makeSonyCode(12,    0x2F, 0x01), "OFF",
         IrRemote::makeSonyCode(15,    0x36, 0x97), "OPTIONS",
         IrRemote::makeSonyCode(15,    0x19, 0x97), "PAUSE",
         IrRemote::makeSonyCode(15,    0x1A, 0x97), "PLAY",
         IrRemote::makeSonyCode(15,    0x20, 0x97), "RECORD",
         IrRemote::makeSonyCode(15,    0x25, 0x97), "RED",
         IrRemote::makeSonyCode(15,    0x7E, 0x1A), "RELATED_SEARCH",
         IrRemote::makeSonyCode(15,    0x23, 0x97), "RETURN",
         IrRemote::makeSonyCode(15,    0x1B, 0x97), "REVERSE",
         IrRemote::makeSonyCode(12,    0x33, 0x01), "RIGHT",
         IrRemote::makeSonyCode(15,    0x74, 0x1A), "SOCIAL_VIEW",
         IrRemote::makeSonyCode(12,    0x25, 0x01), "SOURCE",
         IrRemote::makeSonyCode(12,    0x24, 0x01), "SOURCE_TV",
         IrRemote::makeSonyCode(15,    90, 26), "SOURCE_HDMI_1",
         IrRemote::makeSonyCode(15,    91, 26), "SOURCE_HDMI_2",
         IrRemote::makeSonyCode(15,    92, 26), "SOURCE_HDMI_3",
         IrRemote::makeSonyCode(15,    93, 26), "SOURCE_HDMI_4",
         IrRemote::makeSonyCode(15,    94, 26), "SOURCE_HDMI_5",
         IrRemote::makeSonyCode(12,    0x40, 0x01), "SOURCE_1",
         IrRemote::makeSonyCode(12,    0x41, 0x01), "SOURCE_2",
         IrRemote::makeSonyCode(12,    0x42, 0x01), "SOURCE_3",
         IrRemote::makeSonyCode(12,    0x47, 0x01), "SOURCE_4",
         IrRemote::makeSonyCode(12,    0x48, 0x01), "SOURCE_5",
         IrRemote::makeSonyCode(12,    0x49, 0x01), "SOURCE_6",
         IrRemote::makeSonyCode(12,    0x43, 0x01), "SOURCE_RGB1",
         IrRemote::makeSonyCode(12,    0x44, 0x01), "SOURCE_RGB2",
         IrRemote::makeSonyCode(15,    0x2F, 0x01), "STANDBY",
         IrRemote::makeSonyCode(15,    0x18, 0x97), "STOP",
         IrRemote::makeSonyCode(12,    0x3B, 0x01), "SWAP",
         IrRemote::makeSonyCode(15,    0x58, 0x1A), "SYNC_MENU",
         IrRemote::makeSonyCode(15,    0x65, 0x1A), "TITLE",
         IrRemote::makeSonyCode(15,    0x67, 0x1A), "TV_PAUSE",
         IrRemote::makeSonyCode(15,    0x28, 0x97), "UNKNOWN",
         IrRemote::makeSonyCode(12,    0x74, 0x01), "UP",
         IrRemote::makeSonyCode(12,    0x13, 0x01), "VOLUME_DOWN",
         IrRemote::makeSonyCode(12,    0x12, 0x01), "VOLUME_UP",
         IrRemote::makeSonyCode(15,    0x27, 0x97), "YELLOW",
   };

   const char *name="Unknown";

   for (unsigned index=0; index<(sizeof(codeNameEntry)/sizeof(codeNameEntry[0])); index++) {
      if (code == codeNameEntry[index].code) {
         name = codeNameEntry[index].name;
         break;
      }
   }
   return name;
}

/**
 * Decode IR stream
 *
 * @param discardDuplicates  Whether to discard or report repeated codes
 */
void doDecoding(bool discardDuplicates) {

   DebugPin::setOutput();
   ResultPin::setOutput();
   EventPin::setOutput();

   configureIntervalHardware();

   constexpr IntegerFormat hexFormat16(Width_4,Radix_16,Padding_LeadingZeroes);
   constexpr IntegerFormat hexFormat20(Width_5,Radix_16,Padding_LeadingZeroes);
   constexpr IntegerFormat hexFormat32(Width_8,Radix_16,Padding_LeadingZeroes);
   constexpr IntegerFormat hexFormat2(Width_2,Radix_16,Padding_LeadingZeroes);

   uint32_t lastData = -1U;

   Ticks lastEventTime = 0_ticks;

   IntegerFormat iFormat {
      Padding_LeadingSpaces,
      Width_8,
      Radix_10,
   };

   for(;;) {

      if (!eventQueue.isEmpty()) {

         Event event = eventQueue.deQueue();
         if ((lastData == event.data) && discardDuplicates) {
            continue;
         }
         lastData = event.data;

         Ticks currentEventTime = event.timeStamp;
         Ticks elapsedTime = Lptmr0::modulo(currentEventTime-lastEventTime);
         lastEventTime = currentEventTime;
         console.write(elapsedTime, iFormat, " : ");

         switch (event.protocol) {
            case Protocol_NEC:
            {
               if ((((event.data>>8)^(event.data))&0x00FF0000)!=0x00FF0000) {
                  console.writeln("Illegal format for NEC protocol = 0x", event.data, hexFormat32);
                  break;
               }
               //          ________
               // CCCCCCCC CCCCCCCC ADDRESS--ADDRESS
               //
               uint32_t address = event.data&0xFFFF;
               switch (address) {
                  case 0xFF00: // Teac DVD or Laser DVD or Blaupunkt DVD!!
                     console.writeln("Laser DVD:     ", findLaserDVDName(event.data),     Width_15, "= 0x", event.data, hexFormat32);
                     console.writeln("Teac DVD:      ", findTeacDVDName(event.data),      Width_15, "= 0x", event.data, hexFormat32);
                     console.writeln("Blaupunkt DVD: ", findBlaupunktDVDName(event.data), Width_15, "= 0x", event.data, hexFormat32);
                     break;
                  case 0xBF00: // Teac PVR
                     console.writeln("Teac PVR:      ", findTeacPVRName(event.data),      Width_15, "= 0x", event.data, hexFormat32);
                     break;
                  default:
                     console.writeln("Unknown device (NEC):      ", "= 0x", event.data, hexFormat32);
                     break;
               }
               break;
            }
            case Protocol_Panasonic:
               console.write("Panasonic DVD: ", findPanasonicDVDName(event.prefix, event.data), Width_15);
               console.writeln("= 0x", event.prefix, hexFormat16, ", 0x", event.data, hexFormat32);
               break;
               break;

            case Protocol_Samsung:
               console.write("Samsung DVD: ", findSamsungDVDName(event.prefix, event.data), Width_15);
               console.writeln("= 0x", event.prefix, hexFormat16, ", 0x", event.data, hexFormat20);
               break;

            case Protocol_Sony:
            {
               uint32_t address=0, command=0, extended=0, sonyCode=0;
               if (event.numBits==12) {
                  // 7-bit COMMAND, 5-bit ADDRESS
                  command = event.data&0b0111'1111;
                  address = (event.data>>7)&0b01'1111;
                  sonyCode = IrRemote::makeSonyCode(event.numBits, command, address);
                  console.writeln("Sony TV:     ", findSonyTvName(sonyCode), Width_15, "C=0x", command, hexFormat2, ", A=0x", address, hexFormat2, " (12)  ");
               }
               else if (event.numBits==15) {
                  // 7-bit COMMAND, 8-bit ADDRESS
                  command = event.data&0b0111'1111;
                  address = (event.data>>7)&0b01111'1111;
                  sonyCode = IrRemote::makeSonyCode(event.numBits, command, address);
                  console.writeln("Sony TV:     ", findSonyTvName(sonyCode), Width_15, "C=0x", command, hexFormat2, ", A=0x", address, hexFormat2, " (15) ");
               }
               else if (event.numBits==20) {
                  // 7-bit COMMAND, 5-bit ADDRESS, 8-bit EXTENDED
                  command  = event.data&0b0111'1111;
                  address  = (event.data>>7)&0b01'1111;
                  extended = (event.data>>(7+5))&0b01111'1111;
                  sonyCode = IrRemote::makeSonyCode(event.numBits, command, address);
                  console.writeln(
                        "Sony TV:     ", findSonyTvName(sonyCode), Width_15,
                        "C=0x", command, hexFormat2,
                        ",A=0x", address, hexFormat2,
                        ",E=0x", extended, hexFormat2,
                        " (20) ");
               }
               else {
                  console.writeln("0x", event.data, hexFormat32, " (", event.numBits, ")");
               }
               break;
            }

            case Protocol_Unknown:
               console.writeln("Unknown protocol");
               break;
         }
      }
   }
}

int main() {
   console.writeln("\nStarting");

   doDecoding(false);
   //   doWidthDecoding();

   return 0;
}

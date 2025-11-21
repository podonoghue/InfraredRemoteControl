/******************************************************************************
 * @file   remoteController (180.ARM_Peripherals/Snippets)
 * @brief  Infra-red controller for multiple devices
 * @author podonoghue
 *
 *  Created on: 5/10/2025
 *
 *  Requires declarations for the following in Configure.usbdmProject
 *
 *   TFT
 * ==============================================================
 *  TftCs        TFT CS as SPI Peripheral select e.g. PCS0 (D7)
 *  TftDc        TFT DC as SPI Peripheral select e.g. PCS2 (A3)
 *  TftResetPin  TFT Reset pin as GPIO e.g. GpioB.1 (A4)
 *  TftBacklight TFT Back-light control
 *
 *  Fixed SPI specific connections (shared)
 *  SDA         MOSI/SOUT - D11
 *  ---         MISO/SIN  - D12
 *  SCL         SCK       - D13
 *
 * ==============================================================
 *
 *  TouchCs     XPT2046 CS as SPI Peripheral select e.g. PCS1 (D6)
 *  TouchIrq    XPT2046 IRQ as GPIO e.g. GpioB.2 (D15)
 *
 *  Fixed SPI specific connections (shared)
 *  XPT2046 T_DIN     MOSI/SOUT - D11
 *  XPT2046 T_DOUT    MISO/SIN  - D12
 *  XPT2046 T_SCK     SCK       - D13
 *
 *   CMT (IR transmitter)
 * ==============================================================
 * CMT_IRO     CMT transmitter pin (D22)
 *
 *  Power
 *  GND
 *  VCC
 ******************************************************************************/
#include <cstdlib>
#include <iterator> // For std::forward_iterator_tag
#include <cstddef>  // For std::ptrdiff_t

//#include <vector>
//#include "hardware.h"
#include "../Project_Headers/smc.h"
#include "tft_IL9488.h"
//#include "tft_ILI9341.h"
//#include "tft_ILI9163.h"
//#include "tft_ST7735.h"
#include "touch_XPT2046.h"
#include "specialFonts.h"
#include "cmt-remote.h"
#include "../Project_Headers/pit.h"
#include "BootInformation.h"

// Allow access to USBDM methods without USBDM:: prefix
using namespace USBDM;

using TFT=TFT_ILI9488<Orientation_Rotated_180>;
//using TFT=TFT_ILI9341<Orientation_Normal>;;
//using TFT=TFT_ILI9488<Orientation_Normal>;
//using TFT=TFT_ST7735<Orientation_Normal>;

using TouchInterface = Touch_XPT2046<TouchOrientation_Rotated_180, 330, 480>;

static constexpr unsigned HARDWARE_VERSION = HW_IR_REMOTE;

__attribute__ ((section(".noinit")))
static uint32_t magicNumber;

#if defined(RELEASE_BUILD)
// Also triggers memory image relocation for bootloader
extern BootInformation const bootloaderInformation;
#endif

__attribute__ ((section(".bootloaderInformation")))
__attribute__((used))
const BootInformation bootloaderInformationX = {
      &magicNumber,        // Magic number to force ICP on reboot
      4,                   // Version of this software image
      HARDWARE_VERSION,    // Hardware version for this image
};


static const Spi0::Init spiConfig {
   {
      // Common setting that are seldom changed
      SpiModifiedTiming_Normal ,                   // Modified Timing Format - Normal Timing
      SpiDoze_Enabled ,                            // Enables Doze mode (when processor is waiting?) - Suspend in doze
      SpiFreeze_Enabled ,                          // Controls SPI operation while in debug mode - Suspend in debug
      SpiRxOverflowHandling_Overwrite ,            // Handling of Rx Overflow Data - Overwrite existing
      SpiContinuousClock_Disable,                  // Continuous SCK Enable - Clock during transfers only
      SpiPcsActiveLow_None,                        // Polarity for PCS signals
      SpiPeripheralSelectMode_Transaction          // Negate PCS between Transactions
   },
   { /* CTARs default */ }
};

// Shared SPI to use
Spi0 spi(spiConfig);

// TFT interface
TFT tft(spi);

TouchInterface touchInterface(spi);

enum ButtonCode : uint8_t {
   Button_1,
   Button_2,
   Button_3,
   Button_4,
   Button_5,
   Button_6,
   Button_7,
   Button_8,
   Button_9,
   Button_10,
   Button_11,
   Button_12,
   Button_13,
   Button_14,
   Button_15,
   Button_16,

   Button_Last = Button_16,
   Button_None = 0xFF,
};

/**
 * Prefix operator
 *
 * @param buttonCode
 * @return
 */
inline ButtonCode operator++(ButtonCode &buttonCode) {

   buttonCode = ButtonCode(uint8_t(buttonCode) + 1);
   return buttonCode;
}

/**
 * Postfix operator
 *
 * @param buttonCode
 * @param
 *
 * @return
 */
inline ButtonCode operator++(ButtonCode &buttonCode, int) {

   ButtonCode t = buttonCode;
   buttonCode = ButtonCode(uint8_t(buttonCode) + 1);
   return t;
}

static constexpr Font const &font = font16x24;

template<typename type, size_t capacity=10>
class MyVector {

protected:

   type     data[capacity];
   size_t   size  = 0;

public:

   class ConstantIterator {

   private:

      using iterator_category = std::forward_iterator_tag;
      using difference_type   = std::ptrdiff_t;
      using value_type        = const type;
      using pointer           = value_type*;  // or also value_type*
      using reference         = value_type&;  // or also value_type&

      pointer itemPtr;

   public:

      ConstantIterator() : itemPtr(nullptr) {
      }
      ConstantIterator(pointer ptr) : itemPtr(ptr) {
      }

      pointer operator->() const {
         usbdm_assert(itemPtr != nullptr, "Invalid iterator");
         return itemPtr;
      }
      reference operator*() const {
         usbdm_assert(itemPtr != nullptr, "Invalid iterator");
         return *itemPtr;
      }

      ConstantIterator& operator+(size_t offset) { itemPtr+= offset; return *this; }
      ConstantIterator& operator-(size_t offset) { itemPtr-= offset; return *this; }

      // Prefix increment
      ConstantIterator& operator++() { itemPtr++; return *this; }

      // Postfix increment
      ConstantIterator operator++(int) { ConstantIterator tmp = *this; ++(*this); return tmp; }

      // Prefix increment
      ConstantIterator& operator--() { itemPtr--; return *this; }

      // Postfix increment
      ConstantIterator operator--(int) { ConstantIterator tmp = *this; --(*this); return tmp; }

      friend bool operator== (const ConstantIterator& a, const ConstantIterator& b) { return a.itemPtr == b.itemPtr; };
      friend bool operator!= (const ConstantIterator& a, const ConstantIterator& b) { return a.itemPtr != b.itemPtr; };
   };

   class Iterator {

   private:

      using iterator_category = std::forward_iterator_tag;
      using difference_type   = std::ptrdiff_t;
      using value_type        = type;
      using pointer           = value_type*;  // or also value_type*
      using reference         = value_type&;  // or also value_type&

      pointer itemPtr;

   public:

      Iterator() : itemPtr(nullptr) {
      }
      Iterator(pointer ptr) : itemPtr(ptr) {
      }

      pointer operator->() const {
         usbdm_assert(itemPtr != nullptr, "Invalid iterator");
         return itemPtr;
      }
      reference operator*() const {
         usbdm_assert(itemPtr != nullptr, "Invalid iterator");
         return *itemPtr;
      }

      Iterator& operator+(size_t offset) { itemPtr+= offset; return *this; }
      Iterator& operator-(size_t offset) { itemPtr-= offset; return *this; }

      // Prefix increment
      Iterator& operator++() { itemPtr++; return *this; }

      // Postfix increment
      Iterator operator++(int) { Iterator tmp = *this; ++(*this); return tmp; }

      // Prefix increment
      Iterator& operator--() { itemPtr--; return *this; }

      // Postfix increment
      Iterator operator--(int) { Iterator tmp = *this; --(*this); return tmp; }

      friend bool operator== (const Iterator& a, const Iterator& b) { return a.itemPtr == b.itemPtr; };
      friend bool operator!= (const Iterator& a, const Iterator& b) { return a.itemPtr != b.itemPtr; };
   };

   ConstantIterator begin() const { return ConstantIterator(&data[0]); }
   ConstantIterator end()   const { return ConstantIterator(&data[size]); }

   Iterator begin() { return Iterator(&data[0]); }
   Iterator end()   { return Iterator(&data[size]); }

   bool isEmpty() {
      return size == 0;
   }

   void push_back(type item) {

      usbdm_assert(size<capacity, "Too many items");
      data[size++] = item;
   }

};

/*
 * ============================  Actions  ============================
 */
class Action {

protected:

   inline static bool busy = false;

   char const * const title;

   static constexpr inline const char *noTitle = "No Title";

   ~Action() = default;
 
public:

   /**
    *  Create action
    *
    * @param title         Title for logging
    */
   constexpr Action(const char *title=noTitle) : title(title) {
   }

   virtual void action() const {
      console.writeln("Action: ", title);
   }

   static const Action nullAction;
};

const inline Action Action::nullAction;

class StatusAction : public Action {

protected:

   inline static bool busy = false;

   bool         &status;
   bool const    actionValue;

public:

   /**
    *  Create action
    *
    * @param status        Shared status value to use
    * @param actionValue   Status value to set on action
    * @param title         Title for logging
    */
   constexpr StatusAction(bool &status, bool actionValue, const char *title=noTitle) :
   Action(title), status(status), actionValue(actionValue) {
   }

   virtual ~StatusAction() = default;

   void action() const {

      // Only act if necessary
      if (status != actionValue) {
         console.writeln("StatusAction: ", title);
         status = actionValue;
      }
      else {
         console.writeln("StatusAction: ", title, " - no action needed");
      }
   }
};

class MessageAction : public Action {

public:
   /**
    *
    * @param message  Message to display on action
    */
   constexpr MessageAction(const char *message) : Action(message) {
   }

   virtual ~MessageAction() = default;

   void action() const override {
      console.writeln(title);
   }
};

template<size_t capacity>
class SequenceAction : public Action {

   MyVector<Action const *, capacity> actions;

   static const inline char *noTitle = "Sequence...";

public:

   /**
    *
    * @param title  Title identifying action sequence
    */
   SequenceAction(const char *title=noTitle) : Action(title) {
   }

   virtual ~SequenceAction() = default;

   void add(const Action *action) {
      actions.push_back(action);
   }

   void add(const Action &action) {
      actions.push_back(&action);
   }

   void action() const override {

      Action::action();
      for (const Action *action : actions) {
         action->action();
      }
   }
};

/**
 *
 * @tparam IrClass  Class for IR interface
 */
template<typename IrClass>
class IrAction : public Action {

protected:
   typename IrClass::Code code;
   const unsigned delayTime;
   static const inline char *noTitle = "IR action";

public:

   /**
    * Create IR action
    *
    * @param code          Code to send
    * @param title         Title for logging
    * @param delay         Delay after transmission. 1_tick = 1us
    */
   constexpr IrAction(
         const typename IrClass::Code  code,
         const char                   *title=noTitle,
         Ticks                         delay=100_ticks) :
         Action(title),
         code(code), delayTime(delay) {
   }

   virtual ~IrAction() = default;

   void action() const override {

      Action::action();
      IrClass::send(code, delayTime);
   }
};

using SonyTvAction       = IrAction<IrSonyTV>;
using LaserDvdAction     = IrAction<IrLaserDVD>;
using SamsungDvdAction   = IrAction<IrSamsungDVD>;
using TeacPvrAction      = IrAction<IrTeacPVR>;
using BlaupunktDvdAction = IrAction<IrBlaupunktDVD>;
using PanasonicDvdAction = IrAction<IrPanasonicDVD>;

/**
 *
 * @tparam IrClass  Class for IR interface
 */
template<typename IrClass>
class IrStatusAction : public IrAction<IrClass> {

protected:

   inline static bool busy = false;

   bool         &status;
   bool const    actionValue;

public:

   /**
    *
    * @param code          Code to send
    * @param title         Title for logging
    * @param delay         Delay after transmission
    * @param status        Variable to update with status
    * @param actionValue   Status update value to use
    */
   constexpr IrStatusAction(
         const typename IrClass::Code  code,
         const char                   *title,
         Ticks                         delay,
         bool                         &status,
         bool                          actionValue) :
   IrAction<IrClass>(code, title, delay), status(status), actionValue(actionValue) {
   }

   virtual ~IrStatusAction() = default;

   void action() const {

      // Only act if necessary
      if (status != actionValue) {
         IrAction<IrClass>::action();
         status = actionValue;
      }
      else {
         console.writeln("StatusAction - A:", IrAction<IrClass>::title, " - no action needed");
      }
   }
};

using SonyTvStatusAction       = IrStatusAction<IrSonyTV>;
using LaserDvdStatusAction     = IrStatusAction<IrLaserDVD>;
using SamsungDvdStatusAction   = IrStatusAction<IrSamsungDVD>;
using TeacPvrStatusAction      = IrStatusAction<IrTeacPVR>;
using BlaupunktDVDStatusAction = IrStatusAction<IrBlaupunktDVD>;
using PanasonicDVDStatusAction = IrStatusAction<IrPanasonicDVD>;

/*
 * ============================  Buttons ============================
 */
static constexpr Colour BACKGROUND_COLOUR = Colour::BLACK;

class Button {

protected:
   static constexpr uint16_t minimumWidth  = 77;
   static constexpr uint16_t minimumHeight = 72;
   const Action  &action;
   const Colour   background;

   virtual ~Button() = default;
   
public:
   static constexpr uint16_t H_BORDER_WIDTH = 7;
   static constexpr uint16_t V_BORDER_WIDTH = 6;
   const uint16_t width;
   const uint16_t height;

   constexpr Button(uint16_t width, uint16_t height, const Action &action, Colour background=Colour::RED) :
      action(action),
      background(background),
      width(std::max(width, minimumWidth)), height(std::max(height, minimumHeight)){
   };

   template<unsigned N>
   void drawMyBitmap(const ButtonImage<N> &image, unsigned x, unsigned y, unsigned scale=1) const {
      tft.drawBitmap(image.data, x, y, image.width, image.height, scale);
   }

   virtual void draw(int x, int y) const {
      Colour colour = tft.getBackgroundColour();
      tft.setBackgroundColour(BACKGROUND_COLOUR);
      tft.setColour(background);
      tft.drawRect(x, y, x+width-1, y+height-1);
      tft.setBackgroundColour(colour);
      tft.setColour(background);
      drawMyBitmap(topLeft,     x, y);
      drawMyBitmap(topRight,    x+width-8, y);
      drawMyBitmap(bottomRight, x+width-8, y+height-8);
      drawMyBitmap(bottomLeft,  x, y+height-8);
   }

   bool isHit(unsigned x, unsigned y, unsigned xx, unsigned yy) const {
      bool hit = (x<=xx)&&(xx<=(x+width))&&(y<=yy)&&(yy<=(y+height));
      return hit;
   }

   void doAction() const {
      action.action();
   }
};

template<unsigned N>
class ImageButton : public Button {

protected:
   const ButtonImage<N> &image;
   const Colour  foreground;

public:
   constexpr ImageButton(const Action &action, const ButtonImage<N> &image, Colour foreground=Colour::WHITE, Colour background=Colour::RED) :
   Button(4*H_BORDER_WIDTH+2*image.width, 4*V_BORDER_WIDTH+2*image.height, action, background),
   image(image),
   foreground(foreground) {
   }

   virtual ~ImageButton() = default;

   void draw(int x, int y) const override {
      Button::draw(x, y);
      tft.setBackgroundColour(background);
      tft.setColour(foreground);
      unsigned xx = x + (width-2*image.width)/2;
      unsigned yy = y + (height-2*image.height)/2;
      drawMyBitmap(image, xx, yy, 2);
   }
};

class TextButton : public Button {

   const char *text;

public:

   const Colour  foreground;

   constexpr TextButton(const Action &action, const char *text, Colour foreground=Colour::WHITE, Colour background=Colour::RED) :
      Button(2*H_BORDER_WIDTH+std::char_traits<char>::length(text)*font.width, 2*V_BORDER_WIDTH+font.height, action, background),
      text(text),
      foreground(foreground) {
   }

   virtual ~TextButton() = default;

   void draw(int x, int y) const override {
      Button::draw(x, y);
      tft.setBackgroundColour(background);
      tft.setColour(foreground);
      unsigned xx = x + (width-strlen(text)*font.width)/2;
      unsigned yy = y + (height-font.height)/2;
      tft.moveXY(xx, yy);
      tft.setFont(font);
      tft.write(text);
   }
};

class ColourButton : public Button {

protected:
   const Colour colour;

public:
   constexpr ColourButton(const Action &action, unsigned width, unsigned height, Colour colour) :
      Button(width, height, action, colour),
      colour(colour) {
   }

   virtual ~ColourButton() = default;

   void draw(int x, int y) const override {
      Button::draw(x, y);
   }
};

class FillButton : public Button {

protected:

public:
   FillButton(unsigned width, unsigned height) :
      Button(width, height, Action::nullAction, BACKGROUND_COLOUR) {
   }

   virtual ~FillButton() = default;

   void draw(int, int) const override {
   }
};

/*
 * ============================================================================================
 */
class Page;

class Screen {

private:

   Page const *currentPage = nullptr;

public:

   Screen() {
   }

   ~Screen() = default;

   bool findAndExecuteHandler(unsigned x, unsigned y);

   void show(const Page *pageToShow);

   void setBusy(bool busy = true) const {

//      DebugLed::write(busy);

      console.writeln("|================= ", busy?"Start":"End");

      static const char busyMessage[] = "Busy";

      tft.setBackgroundColour(busy?WHITE:BACKGROUND_COLOUR);
      tft.setColour(busy?RED:BACKGROUND_COLOUR);
      tft.setFont(font);
      tft.moveXY(0, 0).write(busyMessage);
   }

   void handleButton(ButtonCode code);

};

Screen screen;

/*
 * ============================================================================================
 */
class Page : public Action {

protected:

   virtual ~Page() = default;

public:
   Page(const char *title) : Action(title) {
   }

   virtual bool findAndExecuteHandler(unsigned x, unsigned y) const = 0;
   virtual void drawAll(bool pageChanged) const = 0;
   virtual bool handleButton(ButtonCode) const = 0;

};

template <size_t capacity>
class PageWithButtons : public Page {

protected:

   virtual ~PageWithButtons() = default;

private:

   class ButtonInfo {

   public:
      const Button *button;
      uint16_t      x;
      uint16_t      y;

      ButtonInfo(const ButtonInfo &other) = default;

      ButtonInfo(
            const Button *button,
            uint16_t      x,
            uint16_t      y
         ) : button(button), x(x), y(y) {
      }
      ButtonInfo() : ButtonInfo(nullptr, 0, 0) {
      }

   };

   MyVector<ButtonInfo, capacity>buttons;

   const unsigned x;
   const unsigned y;
   const unsigned width;

   unsigned hSpace = 2;
   unsigned vSpace = 2;

   bool doneLayout = false;

public:

   PageWithButtons(const char *title, unsigned x=0, unsigned y=font.height+2U, unsigned width=TFT::WIDTH) : Page(title), x(x), y(y), width(width) {
   }

   void setSpacing(unsigned h, unsigned v) {
      hSpace = h;
      vSpace = v;
   }

   void layout() {

      if (doneLayout) {
         return;
      }
//      console.writeln("Doing layout(", title, ")");
      bool firstInLine = true;
      unsigned xx = x;
      unsigned yy = y;
      unsigned maxHeight = (*buttons.begin()).button->height;

      //      console.writeln("Layout (", x, ", ", y, ")[w=", width ,"]" );

      for (ButtonInfo &buttonInfo:buttons) {

         if (!firstInLine && (xx+buttonInfo.button->width)>width) {
            // Put button on new line
            xx = x;
            yy += maxHeight+vSpace;
            maxHeight = 0;
         }
         if (buttonInfo.button->height>maxHeight) {
            maxHeight = buttonInfo.button->height;
         }
         buttonInfo.x = xx;
         buttonInfo.y = yy;
//         console.writeln("Layout Button(", xx, ", ",yy,")");
         xx += buttonInfo.button->width+hSpace;
         firstInLine = false;
      }
      doneLayout = true;
   }

   void add(const Button *button) {
      ButtonInfo p{button, 0, 0};
      buttons.push_back(p);
   }

   bool findAndExecuteHandler(unsigned x, unsigned y) const override {

      for (auto info:buttons) {

         if (info.button->isHit(info.x, info.y, x, y)) {
            console.writeln("=======================================");
            console.writeln("Button Hit @(", x, ",", y, ") ");
            info.button->doAction();
            return true;
         }
      }
      return false;
   }

   void drawAll(bool pageChanged) const override {

      console.writeln("Show screen '", title, "'");

      screen.setBusy(true);
      tft.setColour(Colour::WHITE);
      tft.setBackgroundColour(BACKGROUND_COLOUR);
      tft.setFont(font);
      tft.moveXYRelative(20, 0);
      tft.write(title);
      tft.putSpace(tft.WIDTH);
      if (pageChanged) {
         tft.setBackgroundColour(BACKGROUND_COLOUR);
         tft.clear(0, font.height, tft.WIDTH, tft.HEIGHT-font.height);
         for (const ButtonInfo &buttonInfo:buttons) {
            tft.setBackgroundColour(BACKGROUND_COLOUR);
//            console.writeln("0x", &(buttonInfo.button), Radix_16, ", X = ", buttonInfo.x, ", Y = ", buttonInfo.y);
            buttonInfo.button->draw(buttonInfo.x, buttonInfo.y);
         }
      }
   }

   void action() const override {
      Action::action();
      screen.show(this);
   }

};

/*
 * ============================================================================================
 */

bool Screen::findAndExecuteHandler(unsigned x, unsigned y) {

   if (currentPage != nullptr) {

      setBusy(true);

      bool rc = currentPage->findAndExecuteHandler(x, y);

      setBusy(false);
      return rc;
   }
   return false;
}

void Screen::show(const Page *pageToShow) {

   bool pageChanged =  (currentPage != pageToShow);
   currentPage = pageToShow;
   currentPage->drawAll(pageChanged);
}

void Screen::handleButton(ButtonCode code) {

   if (currentPage != nullptr) {
      currentPage->handleButton(code);
   }
}

/*
 * Shared Actions
 * ============================================================================================
 */
constexpr SonyTvAction   sonyTvOnOff(                     IrSonyTV::ON_OFF,          "TV On/Off",      1000_ticks);
constexpr SonyTvAction   sonyTvOn(                        IrSonyTV::ON,              "TV On",          1000_ticks);
constexpr SonyTvAction   sonyTvOff(                       IrSonyTV::OFF,             "TV Off"          );
constexpr SonyTvAction   sonyTvSourceHdmi1_Chrome(        IrSonyTV::SOURCE_HDMI_1,   "TV Source HDMI 1");
constexpr SonyTvAction   sonyTvSourceHdmi2_PVR(           IrSonyTV::SOURCE_HDMI_2,   "TV Source HDMI 2");
constexpr SonyTvAction   sonyTvSourceHdmi3_DVD_Samsung(   IrSonyTV::SOURCE_HDMI_3,   "TV Source HDMI 3");
constexpr SonyTvAction   sonyTvSourceHdmi4_DVD_Laser(     IrSonyTV::SOURCE_HDMI_4,   "TV Source HDMI 4");
constexpr SonyTvAction   sonyTvSourceComp_DVD_Pioneer(    IrSonyTV::SOURCE_RGB1,     "TV Source RGB 1" );
constexpr SonyTvAction   sonyTvMute(                      IrSonyTV::MUTE,            "TV Mute",        1'000'000_ticks);
constexpr SonyTvAction   sonyTvVolumeUp(                  IrSonyTV::VOLUME_UP,       "TV Vol Up",      100'000_ticks);
constexpr SonyTvAction   sonyTvVolumeDown(                IrSonyTV::VOLUME_DOWN,     "TV Vol Down",    100'000_ticks);
constexpr SonyTvAction   sonyTvHome(                      IrSonyTV::HOME,            "TV Home"         );
constexpr SonyTvAction   sonyTvReturn(                    IrSonyTV::RETURN,          "TV Return"       );
constexpr SonyTvAction   sonyTvSourceTv(                  IrSonyTV::SOURCE_TV,       "TV Source TV"    );

bool teacPvrPowerStatus  = false;
constexpr TeacPvrAction        teacPvrOnOff(     IrTeacPVR::ON_OFF,   "Teac PVR On/Off",  100_ticks);
constexpr TeacPvrStatusAction  teacPvrOn(        IrTeacPVR::ON_OFF,   "Teac PVR On",      100_ticks,     teacPvrPowerStatus,   true);
constexpr TeacPvrStatusAction  teacPvrOff(       IrTeacPVR::ON_OFF,   "Teac PVR Off",     100_ticks,     teacPvrPowerStatus,   false);

bool laserDvdPowerStatus  = false;
constexpr LaserDvdAction       laserDvdOnOff(    IrLaserDVD::ON_OFF,  "Laser DVD On/Off",  100_ticks);
constexpr LaserDvdStatusAction laserDvdOn(       IrLaserDVD::ON_OFF,  "Laser DVD On",      100_ticks,  laserDvdPowerStatus,   true);
constexpr LaserDvdStatusAction laserDvdOff(      IrLaserDVD::ON_OFF,  "Laser DVD Off",     100_ticks,  laserDvdPowerStatus,   false);

bool samsungDvdPowerStatus  = false;
constexpr SamsungDvdAction       samsungDvdOnOff(   IrSamsungDVD::ON_OFF,   "Samsung DVD On/Off",  100_ticks);
constexpr SamsungDvdStatusAction samsungDvdOn(      IrSamsungDVD::ON_OFF,   "Samsung DVD On",      100_ticks,   samsungDvdPowerStatus,   true);
constexpr SamsungDvdStatusAction samsungDvdOff(     IrSamsungDVD::ON_OFF,   "Samsung DVD Off",     100_ticks,   samsungDvdPowerStatus,   false);

bool panasonicDvdPowerStatus  = false;
constexpr PanasonicDvdAction       panasonicDvdOnOff(   IrPanasonicDVD::ON_OFF,   "Panasonic DVD On/Off",  100_ticks);
constexpr PanasonicDVDStatusAction panasonicDvdOn(      IrPanasonicDVD::ON_OFF,   "Panasonic DVD On",      100_ticks,   panasonicDvdPowerStatus,   true);
constexpr PanasonicDVDStatusAction panasonicDvdOff(     IrPanasonicDVD::ON_OFF,   "Panasonic DVD Off",     100_ticks,   panasonicDvdPowerStatus,   false);

bool blaupunktDvdPowerStatus  = false;
constexpr BlaupunktDvdAction       blaupunktDvdOnOff(   IrBlaupunktDVD::ON_OFF,   "Blaupunkt DVD On/Off",  100_ticks);
constexpr BlaupunktDVDStatusAction blaupunktDvdOn(      IrBlaupunktDVD::ON_OFF,   "Blaupunkt DVD On",      100_ticks,   blaupunktDvdPowerStatus,   true);
constexpr BlaupunktDVDStatusAction blaupunktDvdOff(     IrBlaupunktDVD::ON_OFF,   "Blaupunkt DVD Off",     100_ticks,   blaupunktDvdPowerStatus,   false);

/*
 * Action sequences
 */
SequenceAction< 8> allOff{"Seq: All Off"};
SequenceAction<11> watchTv{"Seq: Watch TV"};
SequenceAction<11> watchSamsungDvd{"Seq: Watch Samsung DVD"};
SequenceAction<11> watchLaserDvd{"Seq: Watch Laser DVD"};
SequenceAction<11> watchTeacPvr{"Seq: Watch PVR"};
SequenceAction<11> watchPanasonicDVD{"Seq: Watch Panasonic DVD"};
SequenceAction<11> watchBlauPunktDVD{"Seq: Watch Blaupunkt DVD"};
SequenceAction<11> displayTeacPvrPage{"Seq: Display Teac DVD page"};
SequenceAction< 2> teacPvrEpisodeGuide{"Seq: Display Teac DVD Numbers page"};
SequenceAction< 1> showMainPage("Show Main Page");

/*
 * Common buttons
 */
constexpr ImageButton<32> showMainPageButton     { showMainPage,     Exit,    Colour::RED, Colour::WHITE };
constexpr ImageButton<32> sonyTvVolumeUpButton   { sonyTvVolumeUp,   VolPlus  };
constexpr ImageButton<32> sonyTvVolumeDownButton { sonyTvVolumeDown, VolMinus };
constexpr ImageButton<32> sonyTvMuteButton       { sonyTvMute,       Mute     };

/*
 * Screen pages
 * ============================================================================================
 */

class HelpPage : public PageWithButtons<7> {

protected:

   static inline constexpr TextButton buttons[6] {
      TextButton(laserDvdOnOff,     "Laser DVD"      ),
      TextButton(samsungDvdOnOff,   "Samsung DVD"    ),
      TextButton(teacPvrOnOff,      "Teac PVR"       ),
      TextButton(blaupunktDvdOnOff, "Blaupunkt DVD"  ),
      TextButton(panasonicDvdOnOff, "Panasonic DVD"  ),
      TextButton(sonyTvOnOff,       "Sony TV"        ),
   };

public:

   HelpPage() : PageWithButtons("Fix Devices") {

      for (unsigned index=0; index<(sizeof(buttons)/sizeof(buttons[0])); index++) {
         add(&buttons[index]);
      }
      add(&showMainPageButton);
      layout();
   }

   ~HelpPage() = default;


   virtual bool handleButton(ButtonCode code) const override {
      static constexpr const Action *buttonActions[Button_Last] = {
            &sonyTvOnOff,
            &teacPvrOnOff,
            &laserDvdOnOff,
            &samsungDvdOnOff,
            &blaupunktDvdOnOff,
            &panasonicDvdOnOff,
            &showMainPage,
      };
      const Action *action = buttonActions[code];
      if (action != nullptr) {
         buttonActions[code]->action();
         return true;
      }
      return false;
   }
};

HelpPage       helpPage;

class MainPage : public PageWithButtons<8> {

protected:
   static inline constexpr TextButton buttons[7] {
      TextButton( watchTv,             "Watch Sony TV"         ),
      TextButton( watchTeacPvr,        "Watch Teac PVR"        ),
      TextButton( watchLaserDvd,       "Watch Laser DVD"       ),
      TextButton( watchSamsungDvd,     "Watch Samsung DVD"     ),
      TextButton( watchPanasonicDVD,   "Watch Panasonic DVD"   ),
//      TextButton( watchBlauPunktDVD,   "Watch BlauPunkt DVD"   ),
      TextButton( allOff,              "All Off"               ),
      TextButton( helpPage,            "Help",                 Colour::RED, Colour::WHITE),
   };
   
public:

   MainPage() : PageWithButtons("Main") {

      for (unsigned index=0; index<(sizeof(buttons)/sizeof(buttons[0])); index++) {
         add(&buttons[index]);
      }
      
      layout();   
   }

   ~MainPage() = default;

   virtual bool handleButton(ButtonCode code) const override {
      
      static constexpr const Action *buttonActions[Button_Last] = {
            &watchTv,
            &watchTeacPvr,
            &watchLaserDvd,
            &watchSamsungDvd,
            &watchPanasonicDVD,
            &watchBlauPunktDVD,
            &allOff,
            &helpPage,
      };
      const Action *action = buttonActions[code];
      if (action != nullptr) {
         buttonActions[code]->action();
         return true;
      }
      return false;
   }
};

class SonyTvPage : public PageWithButtons<19> {

protected:

   static inline constexpr SonyTvAction actions[15] = {
         SonyTvAction{IrSonyTV::Code::NUM1,  "Num 1"   },
         SonyTvAction{IrSonyTV::Code::NUM2,  "Num 2"   },
         SonyTvAction{IrSonyTV::Code::NUM3,  "Num 3"   },
         SonyTvAction{IrSonyTV::Code::UP,    "TV Up"   },

         SonyTvAction{IrSonyTV::Code::NUM4,  "Num 4"   },
         SonyTvAction{IrSonyTV::Code::NUM5,  "Num 5"   },
         SonyTvAction{IrSonyTV::Code::NUM6,  "Num 6"   },
         SonyTvAction{IrSonyTV::Code::DOWN,  "TV Down" },

         SonyTvAction{IrSonyTV::Code::NUM7,  "Num 7"   },
         SonyTvAction{IrSonyTV::Code::NUM8,  "Num 8"   },
         SonyTvAction{IrSonyTV::Code::NUM9,  "Num 9"   },
         SonyTvAction{IrSonyTV::Code::LEFT,  "TV Left" },

         SonyTvAction{IrSonyTV::Code::GUIDE, "Guide"   },
         SonyTvAction{IrSonyTV::Code::NUM0,  "Num 0"   },

         SonyTvAction{IrSonyTV::Code::RIGHT, "TV Right"},
   };
   static inline constexpr ImageButton<32> buttons[19] = {
         ImageButton<32>( actions[ 0],      One      ),
         ImageButton<32>( actions[ 1],      Two      ),
         ImageButton<32>( actions[ 2],      Three    ),
         ImageButton<32>( actions[ 3],      Up       ),

         ImageButton<32>( actions[ 4],      Four     ),
         ImageButton<32>( actions[ 5],      Five     ),
         ImageButton<32>( actions[ 6],      Six      ),
         ImageButton<32>( actions[ 7],      Down     ),

         ImageButton<32>( actions[ 8],      Seven    ),
         ImageButton<32>( actions[ 9],      Eight    ),
         ImageButton<32>( actions[10],      Nine     ),
         ImageButton<32>( actions[11],      Left     ),

         ImageButton<32>( actions[12],      Info     ),
         ImageButton<32>( actions[13],      Zero     ),
         showMainPageButton,
         ImageButton<32>( actions[14],      Right    ),

         sonyTvVolumeUpButton,
         sonyTvVolumeDownButton,
         sonyTvMuteButton,
   };
public:

   SonyTvPage() : PageWithButtons("Sony TV") {

      for (unsigned index=0; index<(sizeof(buttons)/sizeof(buttons[0])); index++) {
         add(&buttons[index]);
      }

      layout();
   }

   ~SonyTvPage() = default;

   virtual bool handleButton(ButtonCode code) const override {
      static constexpr const Action *buttonActions[Button_Last] = {
            &sonyTvVolumeUp,
            &sonyTvVolumeDown,
            &sonyTvMute,
            &showMainPage,
      };
      const Action *action = buttonActions[code];
      if (action != nullptr) {
         buttonActions[code]->action();
         return true;
      }
      return false;
   }
};

class SamsungDvdPage : public  PageWithButtons<19> {

protected:
   static inline constexpr SamsungDvdAction actions[15] = {
      SamsungDvdAction{IrSamsungDVD::Code::REVERSE_SCENE, "DVD Reverse Scene" },
      SamsungDvdAction{IrSamsungDVD::Code::UP           , "DVD Up"            },
      SamsungDvdAction{IrSamsungDVD::Code::FORWARD_SCENE, "DVD Forward Scene" },
      SamsungDvdAction{IrSamsungDVD::Code::PAUSE        , "DVD Pause"         },

      SamsungDvdAction{IrSamsungDVD::Code::LEFT         , "DVD Left"          },
      SamsungDvdAction{IrSamsungDVD::Code::OK           , "DVD OK"            },
      SamsungDvdAction{IrSamsungDVD::Code::RIGHT        , "DVD Right"         },
      SamsungDvdAction{IrSamsungDVD::Code::PLAY         , "DVD Play"          },

      SamsungDvdAction{IrSamsungDVD::Code::REVERSE      , "DVD Fast Reverse"  },
      SamsungDvdAction{IrSamsungDVD::Code::DOWN         , "DVD Down"          },
      SamsungDvdAction{IrSamsungDVD::Code::FORWARD      , "DVD Fast Forward"  },
      SamsungDvdAction{IrSamsungDVD::Code::STOP         , "DVD Halt"          },

      SamsungDvdAction{IrSamsungDVD::Code::EJECT        , "DVD Eject"         },
      SamsungDvdAction{IrSamsungDVD::Code::MENU         , "DVD Menu"          },
      SamsungDvdAction{IrSamsungDVD::Code::INFO         , "DVD Info"          },
   };

   static inline constexpr ImageButton<32> buttons[19] {
      ImageButton<32>( actions[ 0], ReverseScene ),
      ImageButton<32>( actions[ 1], Up           ),
      ImageButton<32>( actions[ 2], ForwardScene ),
      ImageButton<32>( actions[ 3], Pause        ),

      ImageButton<32>( actions[ 4], Left         ),
      ImageButton<32>( actions[ 5], Enter       ),
      ImageButton<32>( actions[ 6], Right        ),
      ImageButton<32>( actions[ 7], Play,        Colour::WHITE, Colour::BLUE ),

      ImageButton<32>( actions[ 8], FastReverse  ),
      ImageButton<32>( actions[ 9], Down         ),
      ImageButton<32>( actions[10], FastForward  ),
      ImageButton<32>( actions[11], Halt         ),

      sonyTvVolumeUpButton,
      sonyTvVolumeDownButton,
      sonyTvMuteButton,
      ImageButton<32>( actions[12], Eject        ),

      ImageButton<32>( actions[13], Menu         ),
      ImageButton<32>( actions[14], Info         ),
      showMainPageButton,
};
      
public:

   SamsungDvdPage() : PageWithButtons("Samsung DVD") {

      for (unsigned index=0; index<(sizeof(buttons)/sizeof(buttons[0])); index++) {
         add(&buttons[index]);
      }

      layout();
   }

   ~SamsungDvdPage() = default;

   virtual bool handleButton(ButtonCode code) const override {
      static constexpr const Action *buttonActions[Button_Last] = {
            &sonyTvVolumeUp,
            &sonyTvVolumeDown,
            &sonyTvMute,
            &showMainPage,
      };
      const Action *action = buttonActions[code];
      if (action != nullptr) {
         buttonActions[code]->action();
         return true;
      }
      return false;
   }
};

class LaserDvdPage : public  PageWithButtons<19> {

protected:
   static inline constexpr LaserDvdAction actions[15] = {
         LaserDvdAction{IrLaserDVD::Code::REVERSE_SCENE, "DVD Reverse Scene" },
         LaserDvdAction{IrLaserDVD::Code::UP           , "DVD Up"            },
         LaserDvdAction{IrLaserDVD::Code::FORWARD_SCENE, "DVD Forward Scene" },
         LaserDvdAction{IrLaserDVD::Code::PAUSE        , "DVD Pause"         },

         LaserDvdAction{IrLaserDVD::Code::LEFT         , "DVD Left"          },
         LaserDvdAction{IrLaserDVD::Code::OK           , "DVD OK"            },
         LaserDvdAction{IrLaserDVD::Code::RIGHT        , "DVD Right"         },
         LaserDvdAction{IrLaserDVD::Code::PLAY         , "DVD Play"          },

         LaserDvdAction{IrLaserDVD::Code::REVERSE      , "DVD Fast Reverse"  },
         LaserDvdAction{IrLaserDVD::Code::DOWN         , "DVD Down"          },
         LaserDvdAction{IrLaserDVD::Code::FORWARD      , "DVD Fast Forward"  },
         LaserDvdAction{IrLaserDVD::Code::STOP         , "DVD Halt"          },

         LaserDvdAction{IrLaserDVD::Code::EJECT        , "DVD Eject"         },
         LaserDvdAction{IrLaserDVD::Code::MENU         , "DVD Menu"          },
         LaserDvdAction{IrLaserDVD::Code::OSD          , "DVD OSD"           },
   };

   static inline constexpr ImageButton<32> buttons[19] {
      ImageButton<32>( actions[ 0], ReverseScene  ),
      ImageButton<32>( actions[ 1], Up            ),
      ImageButton<32>( actions[ 2], ForwardScene  ),
      ImageButton<32>( actions[ 3], Pause         ),

      ImageButton<32>( actions[ 4], Left          ),
      ImageButton<32>( actions[ 5], Enter        ),
      ImageButton<32>( actions[ 6], Right         ),
      ImageButton<32>( actions[ 7], Play,         Colour::WHITE, Colour::BLUE ),

      ImageButton<32>( actions[ 8], FastReverse   ),
      ImageButton<32>( actions[ 9], Down          ),
      ImageButton<32>( actions[10], FastForward   ),
      ImageButton<32>( actions[11], Halt          ),

      sonyTvVolumeUpButton,
      sonyTvVolumeDownButton,
      sonyTvMuteButton,
      ImageButton<32>( actions[12], Eject         ),

      ImageButton<32>( actions[13], Menu          ),
      ImageButton<32>( actions[14], Info          ),
      showMainPageButton,
   };
      
public:

   LaserDvdPage() : PageWithButtons("Laser DVD") {

      for (unsigned index=0; index<(sizeof(buttons)/sizeof(buttons[0])); index++) {
         add(&buttons[index]);
      }

      layout();
   }

   ~LaserDvdPage() = default;

   virtual bool handleButton(ButtonCode code) const override {
      static constexpr const Action *buttonActions[Button_Last] = {
            &sonyTvVolumeUp,
            &sonyTvVolumeDown,
            &sonyTvMute,
            &showMainPage,
      };
      const Action *action = buttonActions[code];
      if (action != nullptr) {
         buttonActions[code]->action();
         return true;
      }
      return false;
   }
};

class PanasonicDvdPage : public  PageWithButtons<19> {

protected:
   static inline constexpr PanasonicDvdAction actions[14] = {
      PanasonicDvdAction( IrPanasonicDVD::Code::REVERSE_SCENE, "DVD Reverse Scene" ),
      PanasonicDvdAction( IrPanasonicDVD::Code::UP           , "DVD Up"            ),
      PanasonicDvdAction( IrPanasonicDVD::Code::FORWARD_SCENE, "DVD Forward Scene" ),
      PanasonicDvdAction( IrPanasonicDVD::Code::PAUSE_PLAY   , "DVD Pause"         ),

      PanasonicDvdAction( IrPanasonicDVD::Code::LEFT         , "DVD Left"          ),
      PanasonicDvdAction( IrPanasonicDVD::Code::OK           , "DVD OK"            ),
      PanasonicDvdAction( IrPanasonicDVD::Code::RIGHT        , "DVD Right"         ),
      PanasonicDvdAction( IrPanasonicDVD::Code::PAUSE_PLAY   , "DVD Play"          ),

      PanasonicDvdAction( IrPanasonicDVD::Code::REVERSE      , "DVD Fast Reverse"  ),
      PanasonicDvdAction( IrPanasonicDVD::Code::DOWN         , "DVD Down"          ),
      PanasonicDvdAction( IrPanasonicDVD::Code::FORWARD      , "DVD Fast Forward"  ),
      PanasonicDvdAction( IrPanasonicDVD::Code::STOP         , "DVD Halt"          ),

      PanasonicDvdAction( IrPanasonicDVD::Code::EJECT        , "DVD Eject"         ),
      PanasonicDvdAction( IrPanasonicDVD::Code::MENU         , "DVD Menu"          ),
   };

   static inline constexpr ImageButton<32> buttons[18] {
          ImageButton<32>( actions[ 0], ReverseScene  ),
          ImageButton<32>( actions[ 1], Up            ),
          ImageButton<32>( actions[ 2], ForwardScene  ),
          ImageButton<32>( actions[ 3], Pause         ),

          ImageButton<32>( actions[ 4], Left          ),
          ImageButton<32>( actions[ 5], Enter        ),
          ImageButton<32>( actions[ 6], Right         ),
          ImageButton<32>( actions[ 7], Play,         Colour::WHITE, Colour::BLUE ),

          ImageButton<32>( actions[ 8], FastReverse   ),
          ImageButton<32>( actions[ 9], Down          ),
          ImageButton<32>( actions[10], FastForward   ),
          ImageButton<32>( actions[11], Halt          ),

          sonyTvVolumeUpButton,
          sonyTvVolumeDownButton,
          sonyTvMuteButton,
          ImageButton<32>( actions[12], Eject         ),

          ImageButton<32>( actions[13], Menu          ),
          showMainPageButton,
   };

public:

   PanasonicDvdPage() : PageWithButtons("Panasonic DVD") {

      for (unsigned index=0; index<(sizeof(buttons)/sizeof(buttons[0])); index++) {
         add(&buttons[index]);
      }

      layout();
   }

   ~PanasonicDvdPage() = default;

   virtual bool handleButton(ButtonCode code) const override {
      static constexpr const Action *buttonActions[Button_Last] = {
            &sonyTvVolumeUp,
            &sonyTvVolumeDown,
            &sonyTvMute,
            &showMainPage,
      };
      const Action *action = buttonActions[code];
      if (action != nullptr) {
         buttonActions[code]->action();
         return true;
      }
      return false;
   }
};
 
class BlaupunktDvdPage : public  PageWithButtons<19> {

protected:
   static inline constexpr BlaupunktDvdAction actions[15] {
      BlaupunktDvdAction{IrBlaupunktDVD::Code::REVERSE_SCENE, "DVD Reverse Scene" },
      BlaupunktDvdAction{IrBlaupunktDVD::Code::UP           , "DVD Up"            },
      BlaupunktDvdAction{IrBlaupunktDVD::Code::FORWARD_SCENE, "DVD Forward Scene" },
      BlaupunktDvdAction{IrBlaupunktDVD::Code::PLAY_PAUSE   , "DVD Play/Pause"    },

      BlaupunktDvdAction{IrBlaupunktDVD::Code::LEFT         , "DVD Left"          },
      BlaupunktDvdAction{IrBlaupunktDVD::Code::OK           , "DVD OK"            },
      BlaupunktDvdAction{IrBlaupunktDVD::Code::RIGHT        , "DVD Right"         },
      BlaupunktDvdAction{IrBlaupunktDVD::Code::PLAY_PAUSE   , "DVD Play/Pause"    },

      BlaupunktDvdAction{IrBlaupunktDVD::Code::REVERSE      , "DVD Fast Reverse"  },
      BlaupunktDvdAction{IrBlaupunktDVD::Code::DOWN         , "DVD Down"          },
      BlaupunktDvdAction{IrBlaupunktDVD::Code::FORWARD      , "DVD Fast Forward"  },
      BlaupunktDvdAction{IrBlaupunktDVD::Code::STOP         , "DVD Halt"          },

      BlaupunktDvdAction{IrBlaupunktDVD::Code::EJECT        , "DVD Eject"         },
      BlaupunktDvdAction{IrBlaupunktDVD::Code::MENU         , "DVD Eject"         },
      BlaupunktDvdAction{IrBlaupunktDVD::Code::OSD          , "DVD OSD"           },
   };

   static inline constexpr ImageButton<32> buttons[19] {
      ImageButton<32>( actions[ 0], ReverseScene  ),
      ImageButton<32>( actions[ 1], Up            ),
      ImageButton<32>( actions[ 2], ForwardScene  ),
      ImageButton<32>( actions[ 3], Pause         ),

      ImageButton<32>( actions[ 4], Left          ),
      ImageButton<32>( actions[ 5], Enter        ),
      ImageButton<32>( actions[ 6], Right         ),
      ImageButton<32>( actions[ 7], Play,         Colour::WHITE, Colour::BLUE ),

      ImageButton<32>( actions[ 8], FastReverse   ),
      ImageButton<32>( actions[ 9], Down          ),
      ImageButton<32>( actions[10], FastForward   ),
      ImageButton<32>( actions[11], Halt          ),

      sonyTvVolumeUpButton,
      sonyTvVolumeDownButton,
      sonyTvMuteButton,
      ImageButton<32>( actions[12], Eject         ),

      ImageButton<32>( actions[13], Menu          ),
      ImageButton<32>( actions[14], Info          ),
      showMainPageButton,
   };
   
public:

   BlaupunktDvdPage() : PageWithButtons("Blaupunkt DVD") {

      for (unsigned index=0; index<(sizeof(buttons)/sizeof(buttons[0])); index++) {
         add(&buttons[index]);
      }
      
      layout();
   };

   ~BlaupunktDvdPage() = default;

   virtual bool handleButton(ButtonCode code) const override {
      static constexpr const Action *buttonActions[Button_Last] = {
            &sonyTvVolumeUp,
            &sonyTvVolumeDown,
            &sonyTvMute,
            &showMainPage,
      };
      const Action *action = buttonActions[code];
      if (action != nullptr) {
         buttonActions[code]->action();
         return true;
      }
      return false;
   }
};

class TeacPvrEpgPage : public PageWithButtons<24> {

protected:
   static inline constexpr TeacPvrAction actions[20] {
      TeacPvrAction{IrTeacPVR::Code::NUM1,  "Num 1"     },
      TeacPvrAction{IrTeacPVR::Code::NUM2,  "Num 2"     },
      TeacPvrAction{IrTeacPVR::Code::NUM3,  "Num 3"     },
      TeacPvrAction{IrTeacPVR::Code::UP,    "PVR Up"    },

      TeacPvrAction{IrTeacPVR::Code::NUM4,  "Num 4"     },
      TeacPvrAction{IrTeacPVR::Code::NUM5,  "Num 5"     },
      TeacPvrAction{IrTeacPVR::Code::NUM6,  "Num 6"     },
      TeacPvrAction{IrTeacPVR::Code::DOWN,  "PVR Down"  },

      TeacPvrAction{IrTeacPVR::Code::NUM7,  "Num 7"     },
      TeacPvrAction{IrTeacPVR::Code::NUM8,  "Num 8"     },
      TeacPvrAction{IrTeacPVR::Code::NUM9,  "Num 9"     },
      TeacPvrAction{IrTeacPVR::Code::LEFT,  "PVR Left"  },


      TeacPvrAction{IrTeacPVR::Code::EXIT,  "PVR Exit"  },
      TeacPvrAction{IrTeacPVR::Code::NUM0,  "Num 0"     },
      TeacPvrAction{IrTeacPVR::Code::OK,    "PVR OK"    },
      TeacPvrAction{IrTeacPVR::Code::RIGHT, "PVR Right" },


      TeacPvrAction{IrTeacPVR::Code::RED,   "PVR Red"   },
      TeacPvrAction{IrTeacPVR::Code::GREEN, "PVR Green" },
      TeacPvrAction{IrTeacPVR::Code::YELLOW,"PVR Yellow"},
      TeacPvrAction{IrTeacPVR::Code::BLUE,  "PVR Blue"  },
   };

public:

   TeacPvrEpgPage() : PageWithButtons("PVR EPG") {

      add(new TextButton (     actions[ 0], "1"            ));
      add(new TextButton (     actions[ 1], "2"            ));
      add(new TextButton (     actions[ 2], "3"            ));
      add(new ImageButton<32>( actions[ 3], Up             ));

      add(new TextButton (     actions[ 4], "4"            ));
      add(new TextButton (     actions[ 5], "5"            ));
      add(new TextButton (     actions[ 6], "6"            ));
      add(new ImageButton<32>( actions[ 7], Down           ));

      add(new TextButton (     actions[ 8], "7"            ));
      add(new TextButton (     actions[ 9], "8"            ));
      add(new TextButton (     actions[10], "9"            ));
      add(new ImageButton<32>( actions[11], Left           ));

      add(new TextButton (     actions[12], "OK"           ));
      add(new TextButton (     actions[13], "0"            ));
      add(new TextButton (     actions[14], "EXIT"         ));
      add(new ImageButton<32>( actions[15], Right          ));

      add(new ColourButton(    actions[16], 0,50, RED      ));
      add(new ColourButton(    actions[17], 0,50, GREEN    ));
      add(new ColourButton(    actions[18], 0,50, YELLOW   ));
      add(new ColourButton(    actions[19], 0,50, BLUE     ));

      add(new FillButton(0,0));
      add(new FillButton(0,0));
      add(new FillButton(0,0));
      add(new TextButton (     displayTeacPvrPage, "Back", Colour::RED, Colour::WHITE ));

      layout();
   }

   ~TeacPvrEpgPage() = default;

   virtual bool handleButton(ButtonCode code) const override {
      static constexpr const Action *buttonActions[Button_Last] = {
            &sonyTvVolumeUp,
            &sonyTvVolumeDown,
            &sonyTvMute,
            &showMainPage,
      };
      const Action *action = buttonActions[code];
      if (action != nullptr) {
         buttonActions[code]->action();
         return true;
      }
      return false;
   }
};

TeacPvrEpgPage     teacPvrEpgPage;

class TeacPvrPage : public PageWithButtons<24> {

protected:
   static inline constexpr TeacPvrAction actions[18] {
      TeacPvrAction{IrTeacPVR::Code::REVERSE_SCENE, "PVR Reverse Scene" },
      TeacPvrAction{IrTeacPVR::Code::UP           , "PVR Up"            },
      TeacPvrAction{IrTeacPVR::Code::FORWARD_SCENE, "PVR Forward Scene" },
      TeacPvrAction{IrTeacPVR::Code::PAUSE        , "PVR Pause"         },

      TeacPvrAction{IrTeacPVR::Code::LEFT         , "PVR Left"          },
      TeacPvrAction{IrTeacPVR::Code::OK           , "PVR OK"            },
      TeacPvrAction{IrTeacPVR::Code::RIGHT        , "PVR Right"         },
      TeacPvrAction{IrTeacPVR::Code::PLAY         , "PVR Play"          },

      TeacPvrAction{IrTeacPVR::Code::REVERSE      , "PVR Fast Reverse"  },
      TeacPvrAction{IrTeacPVR::Code::DOWN         , "PVR Down"          },
      TeacPvrAction{IrTeacPVR::Code::FORWARD      , "PVR Fast Forward"  },
      TeacPvrAction{IrTeacPVR::Code::STOP         , "PVR Halt"          },

      TeacPvrAction{IrTeacPVR::Code::MENU         , "PVR Menu"          },

      TeacPvrAction{IrTeacPVR::Code::RED          , "PVR Red"           },
      TeacPvrAction{IrTeacPVR::Code::GREEN        , "PVR Green"         },
      TeacPvrAction{IrTeacPVR::Code::YELLOW       , "PVR Yellow"        },
      TeacPvrAction{IrTeacPVR::Code::BLUE         , "PVR Blue"          },
      TeacPvrAction{IrTeacPVR::Code::EXIT         , "PVR EXIT"          }
   };
   static inline constexpr ImageButton<32> buttons[16] {
      ImageButton<32>( actions[ 0],       ReverseScene ),
      ImageButton<32>( actions[ 1],       Up           ),
      ImageButton<32>( actions[ 2],       ForwardScene ),
      ImageButton<32>( actions[ 3],       Pause        ),

      ImageButton<32>( actions[ 4],       Left         ),
      ImageButton<32>( actions[ 5],       Enter       ),
      ImageButton<32>( actions[ 6],       Right        ),
      ImageButton<32>( actions[ 7],       Play,        Colour::WHITE, Colour::BLUE ),

      ImageButton<32>( actions[ 8],       FastReverse  ),
      ImageButton<32>( actions[ 9],       Down         ),
      ImageButton<32>( actions[10],       FastForward  ),
      ImageButton<32>( actions[11],       Halt         ),
      sonyTvVolumeUpButton,
      sonyTvVolumeDownButton,
      sonyTvMuteButton,
      ImageButton<32>( actions[12],       Menu         ),
   };

public:

   TeacPvrPage() : PageWithButtons("Teac PVR") {

      for (unsigned index=0; index<(sizeof(buttons)/sizeof(buttons[0])); index++) {
         add(&buttons[index]);
      }
      add(new ColourButton(    actions[13], 0,50,     RED    ));
      add(new ColourButton(    actions[14], 0,50,     GREEN  ));
      add(new ColourButton(    actions[15], 0,50,     YELLOW ));
      add(new ColourButton(    actions[16], 0,50,     BLUE   ));

      add(new TextButton(      teacPvrEpisodeGuide,   "EPG"  ));
      add(new TextButton(      actions[17],           "EXIT" ));
      add(&showMainPageButton);
      add(new FillButton(0,0));

      layout();
  }

   ~TeacPvrPage() = default;

   virtual bool handleButton(ButtonCode code) const override {
      static constexpr const Action *buttonActions[Button_Last] = {
            &sonyTvVolumeUp,
            &sonyTvVolumeDown,
            &sonyTvMute,
            &showMainPage,
      };
      const Action *action = buttonActions[code];
      if (action != nullptr) {
         buttonActions[code]->action();
         return true;
      }
      return false;
   }

};

/*
 * PageWithButtons definitions
 * ============================================================================================
 */

MainPage          mainPage;
SonyTvPage        sonyTvPage;
TeacPvrPage       teacPvrPage;
LaserDvdPage      laserDvdPage;
SamsungDvdPage    samsungDvdPage;
PanasonicDvdPage  panasonicDvdPage;
BlaupunktDvdPage  blaupunktDvdPage;

MessageAction completeMessage("Complete");

void initialiseGuiAndActions() {

   showMainPage.add(mainPage);

   allOff.add(sonyTvOff);
   allOff.add(mainPage);

   allOff.add(laserDvdOff);
   allOff.add(teacPvrOff);
   allOff.add(samsungDvdOff);
   allOff.add(panasonicDvdOff);
   allOff.add(blaupunktDvdOff);
   allOff.add(completeMessage);

   watchTv.add(sonyTvOn);
   watchTv.add(sonyTvHome);
   watchTv.add(sonyTvReturn);
   watchTv.add(sonyTvSourceTv);
   watchTv.add(sonyTvPage);

   watchTv.add(laserDvdOff);
   watchTv.add(teacPvrOff);
   watchTv.add(samsungDvdOff);
   watchTv.add(panasonicDvdOff);
   watchTv.add(blaupunktDvdOff);
   watchTv.add(completeMessage);

   watchTeacPvr.add(sonyTvOn);
   watchTeacPvr.add(sonyTvHome);
   watchTeacPvr.add(sonyTvReturn);
   watchTeacPvr.add(sonyTvSourceHdmi2_PVR);

   watchTeacPvr.add(teacPvrOn);
   watchTeacPvr.add(laserDvdOff);
   watchTeacPvr.add(teacPvrPage);
   watchTeacPvr.add(samsungDvdOff);
   watchTeacPvr.add(panasonicDvdOff);
   watchTeacPvr.add(blaupunktDvdOff);
   watchTeacPvr.add(completeMessage);

   watchLaserDvd.add(sonyTvOn);
   watchLaserDvd.add(sonyTvHome);
   watchLaserDvd.add(sonyTvReturn);
   watchLaserDvd.add(sonyTvSourceHdmi4_DVD_Laser);

   watchLaserDvd.add(laserDvdOn);
   watchLaserDvd.add(laserDvdPage);
   watchLaserDvd.add(teacPvrOff);
   watchLaserDvd.add(samsungDvdOff);
   watchLaserDvd.add(panasonicDvdOff);
   watchLaserDvd.add(blaupunktDvdOff);
   watchLaserDvd.add(completeMessage);

   watchSamsungDvd.add(sonyTvOn);
   watchSamsungDvd.add(sonyTvHome);
   watchSamsungDvd.add(sonyTvReturn);
   watchSamsungDvd.add(sonyTvSourceHdmi3_DVD_Samsung);

   watchSamsungDvd.add(laserDvdOff);
   watchSamsungDvd.add(teacPvrOff);
   watchSamsungDvd.add(samsungDvdOn);
   watchSamsungDvd.add(samsungDvdPage);
   watchSamsungDvd.add(panasonicDvdOff);
   watchSamsungDvd.add(blaupunktDvdOff);
   watchSamsungDvd.add(completeMessage);

   watchPanasonicDVD.add(sonyTvOn);
   watchPanasonicDVD.add(sonyTvHome);
   watchPanasonicDVD.add(sonyTvReturn);
   watchPanasonicDVD.add(sonyTvSourceHdmi2_PVR);

   watchPanasonicDVD.add(laserDvdOff);
   watchPanasonicDVD.add(teacPvrOff);
   watchPanasonicDVD.add(samsungDvdOff);
   watchPanasonicDVD.add(panasonicDvdOn);
   watchPanasonicDVD.add(panasonicDvdPage);
   watchPanasonicDVD.add(blaupunktDvdOff);
   watchPanasonicDVD.add(completeMessage);

   watchBlauPunktDVD.add(sonyTvOn);
   watchBlauPunktDVD.add(sonyTvHome);
   watchBlauPunktDVD.add(sonyTvReturn);
   watchBlauPunktDVD.add(sonyTvSourceHdmi2_PVR);

   watchBlauPunktDVD.add(laserDvdOff);
   watchBlauPunktDVD.add(teacPvrOff);
   watchBlauPunktDVD.add(samsungDvdOff);
   watchBlauPunktDVD.add(panasonicDvdOff);
   watchBlauPunktDVD.add(blaupunktDvdOn);
   watchBlauPunktDVD.add(blaupunktDvdPage);
   watchBlauPunktDVD.add(completeMessage);

   displayTeacPvrPage.add(teacPvrPage);

   teacPvrEpisodeGuide.add(*new TeacPvrAction(IrTeacPVR::Code::EPG, "PVR EPG" ));
   teacPvrEpisodeGuide.add(teacPvrEpgPage);
}

#if 0
void getTouch(unsigned &touchX, unsigned &touchY) {

   while (!touchInterface.checkRawTouch(touchX, touchY)) {
      __asm__("nop");
   }
}

void calibratePoint(unsigned x1, unsigned y1) {

   static constexpr IntegerFormat intFormat(Width_10, Padding_LeadingSpaces);

   if (x1>(tft.WIDTH-11)) {
      x1 = tft.WIDTH-11;
   }
   if (y1>(tft.HEIGHT-11)) {
      y1 = tft.HEIGHT-11;
   }
   unsigned touchX;
   unsigned touchY;

   tft.setColour(WHITE);

   tft.drawRect(x1,y1,x1+10,y1+10);
   getTouch(touchX, touchY);
   tft.setColour(BACKGROUND_COLOUR);
   tft.drawRect(x1,y1,x1+10,y1+10);
   waitMS(300);

   console.write(x1+5, intFormat, ", ", y1+5, intFormat, ", ");
   console.writeln(touchX, intFormat, ", ", touchY, intFormat);
}

void calibrate() {

   tft.setBackgroundColour(BACKGROUND_COLOUR);
   tft.setColour(WHITE);
   tft.clear();

   unsigned width  = tft.WIDTH;
   unsigned height = tft.HEIGHT;
   unsigned xIncrement = (width/4)-1;
   unsigned yIncrement = (height/5)-1;

   //   calibratePoint(  width/4, height/5);
   //   calibratePoint(3*width/4, height/5);
   //   calibratePoint(3*width/4, 4*height/5);
   //   calibratePoint(  width/4, 4*height/5);

   for (unsigned y=0; y<height; y+=yIncrement) {
      for (unsigned x=0; x<width; x+=xIncrement) {
         calibratePoint(x, y);
      }
   }
}
#endif

bool wakeUp = false;

int checkBatteryLevel() {

   static const float Levels[] = {3.0, 3.7, 3.85, 3.95, 4.2};

   float battery = 2*3.3*BatteryLevel::readAnalogue(AdcResolution_16bit_se)/Adc0::getSingleEndedMaximum(AdcResolution_16bit_se);
   float first = 0.0;
   float second;
   int percent = 100;
   if (battery<Levels[0]) {
      percent = 0;
   }
   else if (battery>Levels[4]) {
      percent = 100;
   }
   else {
      for(unsigned index=0; index<(sizeof(Levels)/sizeof(Levels[0])); index++) {
         second = Levels[index];
//         console.writeln("[", first, " - ", second, "]");
         if (battery<second) {
            percent = round(25*((index-1) + (battery-first)/(second-first)));
            break;
         }
         first = second;
      }
   }
   DebugLed::write(Charging::read());
   console.writeln("Battery = ", battery, "V, ",percent, "%", Charging::read()?", Charging":"");
   return percent;
}

static uint16_t buttonState = 0;
static uint16_t currentButton = 0;

void buttonCallback() {

   static uint8_t  poll  = 0;
   static unsigned count = 0;

   uint8_t current = Switches::read();

   if (current != poll) {
      poll = current;
      count = 0;
      return;
   }
   if (count++==5) {
      currentButton = current;
   }
}

static constexpr PcrInit gpioInit {

   PinAction_None,
   PinPull_Up,
   PinDriveMode_PushPull,
   PinDriveStrength_Low,
   PinFilter_Passive,
   PinSlewRate_Slow,
};

void initialiseMiscellaneous() {

   Charging::setInput(gpioInit);
   Switches::setInput(gpioInit);
   DebugLed::setOutput(gpioInit);

   Pit::Init pitInit {
      PitOperation_Enabled,
      PitDebugMode_StopInDebug,
   };
   Pit::configure(pitInit);

   static constexpr Pit::ChannelInit pitChannelInit {
      ButtonTimerChannel::CHANNEL,

      PitChannelEnable_Enabled ,   // (pit_tctrl_ten[0])         Timer Channel Enable - Channel enabled
      PitChannelAction_Interrupt , // (pit_tctrl_tie[0])         Action on timer event - Interrupt
      479999_ticks,                // (pit_ldval_tsv[0])         Reload value channel 0

      NvicPriority_Normal ,        // (irqLevel_Ch0)             IRQ priority level for Ch0 - Normal
      buttonCallback,              // (handlerName_Ch0)          User declared event handler
   };

   Pit::configure(pitChannelInit);

   static constexpr Adc::Init adcInit {

      AdcResolution_16bit_se ,         // (adc_cfg1_mode)            ADC Resolution - 16-bit unsigned (single-ended mode)
      AdcClockSource_Asynch ,          // (adc_cfg1_adiclk)          ADC Input Clock - Asynchronous clock (ADACK)
      AdcMuxsel_B ,                    // (adc_cfg2_muxsel)          A/B multiplexor selection - The multiplexor selects B channels
      AdcAveraging_16 ,                // (adc_sc3_avg)              Hardware Average Select - 1 sample
      AdcSample_4cycles,               // (adc_sample)               Sample Time Configuration - 4 ADCK total

      // The following values must be in order
      AdcPretrigger_0,            	   // sc1[0]/r[0] ,
      BatteryLevel::CHANNEL ,        	// (adc_sc1[0]_adch)          ADC Channel number - ADC0_SE0 [-]
      AdcAction_None,              	   // (adc_sc1[0]_aien)          Action on conversion completion - None
   };
   BatteryLevel::Owner::configure(adcInit);
   BatteryLevel::setInput();
}

void llwuCallback() {

}

void suspend() {

   spi.disable();
   Cmt::disable();

//   PowerEnable::off();
//
//   // Prepare LLWU inputs
//   static constexpr PcrInit pups {
//      PinPull_Up,
//   };
//   SwitchRow1::setInput(pups);
//   SwitchRow2::setInput(pups);
//   SwitchRow3::setInput(pups);
//   SwitchCols::setOut();
//   SwitchCols::bitClear(0xF);
//
//   // Wakeup pin configuration
//   static constexpr LlwuBasicInfo::Init llwuInitValue = {
//         LlwuPin_ButtonWakeupR1, LlwuPinMode_FallingEdge,  // Wake-up on pin falling edge,
//         LlwuPin_ButtonWakeupR2, LlwuPinMode_FallingEdge,  // Wake-up on pin falling edge,
//         LlwuPin_ButtonWakeupR3, LlwuPinMode_FallingEdge,  // Wake-up on pin falling edge,
//
//      LlwuResetWakeup_Enabled , // (llwu_rst_llrste)          Low-Leakage Mode RESET Enable - RESET enabled as LLWU exit source
//      LlwuResetFilter_Enabled,  // (llwu_rst_rstfilt)         Digital Filter On RESET Pin - Filter enabled,
//   };
//
//   Llwu::configure(llwuInitValue);
//
//   Smc::enterStopMode(SmcStopMode_LowLeakageStop);
}

ButtonCode getButton() {

   if (buttonState == currentButton) {
      return Button_None;
   }
   uint8_t changed = (currentButton^buttonState);

   if (changed) {
      return ButtonCode(__builtin_ffs(changed)-1);

//      uint16_t bitMask = 0b1;
//      for (ButtonCode code=Button_1; code<=Button_Last; code++) {
//
//         if (changed&bitMask) {
//
//            // Record new button state
//            buttonState ^= bitMask;
//
//            // Only act on button-press
//            if (buttonState&bitMask) {
//               return code;
//            }
//         }
//         bitMask <<= 1;
//      }
   }
   return Button_None;
}

void calibrate() {

   unsigned xs[] = {310, 235, 160, 85, 10};
   unsigned ys[] = {10, 125, 240, 355, 470};

   unsigned mappedXs[5];
   unsigned mappedYs[5];

   unsigned touchX, touchY;

   for (;;) {
      tft.clear();
      tft.setColour(GREEN);

      for( unsigned index=0; index<(sizeof(xs)/sizeof(xs[0])); index++) {
         mappedXs[index] = 0;
         mappedYs[index] = 0;
      }
      for( unsigned indexX=0; indexX<(sizeof(xs)/sizeof(xs[0])); indexX++) {
         for( unsigned indexY=0; indexY<(sizeof(ys)/sizeof(ys[0])); indexY++) {
            tft.drawCircle(xs[indexX], ys[indexY], 10);

            while (!touchInterface.checkRawTouch(touchX, touchY)) {
            }

//            tft.setColour(RED);
//            tft.drawCircle(touchX,touchY, 10);
            mappedXs[indexX] += touchX;
            mappedYs[indexY] += touchY;
            waitMS(200);
         }
      }
      console.writeln("static inline const Map xPoints[] = {");
      for( unsigned index=0; index<(sizeof(xs)/sizeof(xs[0])); index++) {
         console.writeln("{", mappedXs[index]/5, ", ", xs[index], ", },");
      }
      console.writeln("};");
      console.writeln("static inline const Map yPoints[] = {");
      for( unsigned index=0; index<(sizeof(xs)/sizeof(xs[0])); index++) {
         console.writeln("{", mappedYs[index]/5, ", ", ys[index], ", },");
      }
      console.writeln("};");
   }
}

void checkCalibration() {

   unsigned touchX, touchY;

   tft.clear();
   tft.setColour(GREEN);
   for(;;) {
      if (touchInterface.checkTouch(touchX, touchY)) {
         tft.setColour(GREEN);
         tft.drawCircle(touchX,touchY, 20);
         waitMS(100);
      }
   }

}

void touchHandler() {
   console.writeln("Touch Irq");
   touchInterface.disableTouchInterrupt();
   wakeUp = true;
}

void testTouchWakeup() {

   DebugLed::setOutput();
   touchInterface.setInterruptHandler(touchHandler);
   static constexpr IntegerFormat decimalFormat(Padding_LeadingSpaces, Width_4, Radix_10);

   for(;;) {
      DebugLed::off();
      touchInterface.enableTouchInterrupt();
      console.writeln("Entering low power mode...");
      Smc::enterWaitMode();
      if (!wakeUp) {
         console.writeln("False Alarm");
         continue;
      }
      DebugLed::on();
      console.writeln("Awake!...");
      waitMS(200);
   }
}

int main() {

   console.writeln("== Starting ==\n");

//   calibrate();
//   checkCalibration();
//   testTouchWakeup();

   unsigned touchX, touchY;

   initialiseMiscellaneous();

   tft.setBackgroundColour(BACKGROUND_COLOUR);
   tft.clear();

   initialiseGuiAndActions();

   allOff.action();
   screen.setBusy(false);

   console.setEcho(EchoMode_Off);
   console.setBlocking(BlockingMode_Off);

   ButtonCode buttonCode;

   unsigned idleCount = 0;
   bool reinitialise  = true;

   touchInterface.setInterruptHandler(touchHandler);

   for(unsigned count = 0; ;count++) {

      if ((count % 1000) == 0) {
         checkBatteryLevel();
         //      int battery = checkBatteryLevel();
         //      if (battery < 10) {
         //         console.writeln("Entering low power mode");
         //         static constexpr PcrInit wakeupInit {
         //
         //            PinAction_IrqEither,
         //            PinPull_Up,
         //            PinDriveMode_PushPull,
         //            PinDriveStrength_Low,
         //            PinFilter_Passive,
         //            PinSlewRate_Slow,
         //         };
         //         Switch1::setInput(wakeupInit);
         //         tft.sleep();
         //         sleeping = true;
         //         Smc::enterWaitMode();
         //      }
         //      if (battery >= 10) {
         //         Switch1::setInput(gpioInit);
         //         if (sleeping) {
         //            console.writeln("I'm awake!");
         //            sleeping = false;
         //            tft.awaken();
         //         }
         //      }
      }

      if (reinitialise) {
         reinitialise = false;
         tft.awaken();
         idleCount = 0;
         waitMS(500);
      }
      else if (touchInterface.checkTouch(touchX, touchY)) {
         tft.setColour(GREEN);
         tft.drawCircle(touchX,touchY, 10);
         idleCount = 0;
         //         console.writeln("\nLooking for touch @(", touchX, ",", touchY, ") ");
         if (!screen.findAndExecuteHandler(touchX, touchY)) {
            waitMS(100);
         }
         continue;
      }
      else if ((buttonCode = getButton()) != Button_None) {
         screen.setBusy(true);
         screen.handleButton(buttonCode);
         screen.setBusy(false);
         idleCount = 0;
      }
      else {
         idleCount++;
         if (idleCount>200) {
            ButtonTimerChannel::disableNvicInterrupts();
            tft.sleep();
            for(;;) {

               touchInterface.enableTouchInterrupt();
               console.writeln("Entering low power mode...");
               wakeUp = false;
               Smc::enterWaitMode();
               if (wakeUp) {
                  break;
               }
               console.writeln("False Alarm");
            }
            tft.awaken();
            console.writeln("Awake!...");
            ButtonTimerChannel::enableNvicInterrupts();
            reinitialise = true;
         }
      }
      waitMS(100);
   }
   return 0;
}

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

#include "../Project_Headers/llwu.h"
#include "../Project_Headers/stringFormatter.h"

//#include <vector>
//#include "hardware.h"
#include "../Project_Headers/smc.h"
#include "../Project_Headers/rcm.h"
#include "../Project_Headers/pmc.h"
#include "tft_ILI9488.h"
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

static constexpr UartBaudRate baudRate = UartBaudRate_9600;

static constexpr unsigned HARDWARE_VERSION = HW_IR_REMOTE;

__attribute__ ((section(".noinit")))
static uint32_t magicNumber;

#if defined(RELEASE_BUILD)
// Also triggers memory image relocation for bootloader
extern BootInformation const bootloaderInformation;
#endif

__attribute__ ((section(".bootloaderInformation")))
__attribute__((used))
const BootInformation bootloaderInformation = {
      &magicNumber,        // Magic number to force ICP on reboot
      4,                   // Version of this software image
      HARDWARE_VERSION,    // Hardware version for this image
};


static const Spi0::Init spiConfig {
      // Common setting that are seldom changed
      SpiModifiedTiming_Normal ,                   // Modified Timing Format - Normal Timing
      SpiDoze_Enabled ,                            // Enables Doze mode (when processor is waiting?) - Suspend in doze
      SpiFreeze_Enabled ,                          // Controls SPI operation while in debug mode - Suspend in debug
      SpiRxOverflowHandling_Overwrite ,            // Handling of Rx Overflow Data - Overwrite existing
      SpiContinuousClock_Disable,                  // Continuous SCK Enable - Clock during transfers only
      SpiPcsActiveLow_None,                        // Polarity for PCS signals
      SpiPeripheralSelectMode_Transaction          // Negate PCS between Transactions
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

   Button_Release = 0x80,

   Button_1_Release  = Button_1  | Button_Release,
   Button_2_Release  = Button_2  | Button_Release,
   Button_3_Release  = Button_3  | Button_Release,
   Button_4_Release  = Button_4  | Button_Release,
   Button_5_Release  = Button_5  | Button_Release,
   Button_6_Release  = Button_6  | Button_Release,
   Button_7_Release  = Button_7  | Button_Release,
   Button_8_Release  = Button_8  | Button_Release,
   Button_9_Release  = Button_9  | Button_Release,
   Button_10_Release = Button_10 | Button_Release,
   Button_11_Release = Button_11 | Button_Release,
   Button_12_Release = Button_12 | Button_Release,
   Button_13_Release = Button_13 | Button_Release,
   Button_14_Release = Button_14 | Button_Release,
   Button_15_Release = Button_15 | Button_Release,
   Button_16_Release = Button_16 | Button_Release,

   Button_Last = Button_16,
   Button_None = 0xFF,
};

const char *checkBatteryLevel();

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
   size_t   mSize  = 0;

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
   ConstantIterator end()   const { return ConstantIterator(&data[mSize]); }

   Iterator begin() { return Iterator(&data[0]); }
   Iterator end()   { return Iterator(&data[mSize]); }

   bool isEmpty() {
      return mSize == 0;
   }

   void push_back(type item) {

      usbdm_assert(mSize<capacity, "Too many items");
      data[mSize++] = item;
   }

   size_t size() const {
      return mSize;
   }

   const type &at(size_t pos) const {

      usbdm_assert(pos<mSize,"Illegal array index");

      return data[pos];
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

   const char *getTitle() const {
      return title;
   };

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
   static constexpr uint16_t minimumHeight = 70;
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
      tft.fill(background, x, y, width, height);
//      tft.drawRect(x, y, x+width-1, y+height-1);
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

   const Action *getAction() const {
      return &action;
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

protected:

   static inline Page const *currentPage = nullptr;

   Screen() = delete;

   void static updateTitleLine();

   void static updateStatusLine();

   static inline const char *statusLine = "";

public:

   using Handler = void *(void);

   static const Action *findTouchAction(unsigned x, unsigned y);
   static const Action *findButtonAction(ButtonCode code);
   static const Page   *getCurrentPage() {
      return currentPage;
   }

   static void refresh();

   static void show(const Page *pageToShow);

   void static setStatusLine(const char *text) {
      statusLine = text;
      updateStatusLine();
   }

   void static setBusy(bool isBusy = true) {

      tft.setBackgroundColour(isBusy?WHITE:BACKGROUND_COLOUR);
      tft.setColour(isBusy?RED:BACKGROUND_COLOUR);
      tft.drawBitmap(Busy.data, 0, 0, Busy.width, Busy.height, 1);
   }

   static void reportBattery() {

//      tft.setColour((batteryLevel<15)?Colour::RED:Colour::WHITE);
      tft.setBackgroundColour(BACKGROUND_COLOUR);
      tft.setColour(Colour::WHITE);
      tft.moveXY(tft.WIDTH-50, 0);
      tft.write(checkBatteryLevel());
   }

};

/*
 * ============================================================================================
 */
class Page : public Action {

protected:

   virtual ~Page() = default;

public:
   Page(const char *title) : Action(title) {
   }

   /**
    * Find Action for touch event at (x,y)
    *
    * @param x
    * @param y
    *
    * @return Action or nullptr if none available
    */
   virtual const Action *findTouchAction(unsigned x, unsigned y) const = 0;

   /**
    * Find Action for button event
    *
    * @param  Button code to identify button
    *
    * @return Action or nullptr if none available
    */
   virtual const Action *findButtonAction(ButtonCode) const = 0;

   /**
    * Draw page
    */
   virtual void drawAll() const = 0;
};

template <size_t capacity>
class PageWithButtons : public Page {

protected:

   virtual ~PageWithButtons() = default;

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

   MyVector<ButtonInfo, capacity>mButtons;

   const unsigned x;
   const unsigned y;
   const unsigned width;

   unsigned hSpace = 2;
   unsigned vSpace = 2;

   bool doneLayout = false;

public:

   PageWithButtons(const char *title, unsigned x=0, unsigned y=font.height+2U, unsigned width=TFT::WIDTH)
      : Page(title), x(x), y(y), width(width) {
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
      unsigned maxHeight = (*mButtons.begin()).button->height;

      //      console.writeln("Layout (", x, ", ", y, ")[w=", width ,"]" );

      for (ButtonInfo &buttonInfo:mButtons) {

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
      mButtons.push_back(p);
   }

   const Button *getButtonAt(unsigned index) const {
      return mButtons.at(index).button;
   }

   const Action *findTouchAction(unsigned x, unsigned y) const override {

      for (auto info:mButtons) {

         if (info.button->isHit(info.x, info.y, x, y)) {
            console.writeln("=======================================");
            console.writeln("Button Hit @(", x, ",", y, ") ");
            return info.button->getAction();
         }
      }
      return nullptr;
   }

   void drawAll() const override {

      console.writeln("Show screen '", title, "'");

      tft.setBackgroundColour(BACKGROUND_COLOUR);
      tft.fill(BACKGROUND_COLOUR, 0, font.height, tft.WIDTH, tft.HEIGHT-font.height);
      for (const ButtonInfo &buttonInfo:mButtons) {
         tft.setBackgroundColour(BACKGROUND_COLOUR);
         //            console.writeln("0x", &(buttonInfo.button), Radix_16, ", X = ", buttonInfo.x, ", Y = ", buttonInfo.y);
         buttonInfo.button->draw(buttonInfo.x, buttonInfo.y);
      }
   }

   void action() const override {
      Action::action();
      Screen::show(this);
   }

};

/*
 * ============================================================================================
 */

void Screen::updateTitleLine() {

   tft.setColour(Colour::WHITE);
   tft.setBackgroundColour(BACKGROUND_COLOUR);
   tft.setFont(font);
   tft.moveXY(20, 0);
   tft.write(currentPage->getTitle());
   tft.putSpace(tft.WIDTH);

   reportBattery();
}

void Screen::updateStatusLine() {

   tft.setColour(Colour::WHITE);
   tft.setBackgroundColour(BACKGROUND_COLOUR);
   tft.setFont(font);
   tft.moveXY(0, tft.HEIGHT-font.height);
   if (statusLine == nullptr) {
      statusLine = "                      ";
   }
   tft.write(statusLine);
   tft.putSpace(tft.WIDTH);
}

/**
 * Find Action for touch event at (x,y)
 *
 * @param x
 * @param y
 *
 * @return Action or nullptr if none available
 */
const Action *Screen::findTouchAction(unsigned x, unsigned y) {

   if (currentPage != nullptr) {
      return currentPage->findTouchAction(x, y);
   }
   return nullptr;
}

void Screen::refresh() {

   updateTitleLine();
   currentPage->drawAll();
   updateStatusLine();
}

void Screen::show(const Page *pageToShow) {

   bool pageChanged =  (currentPage != pageToShow);
   if (pageChanged) {
      currentPage = pageToShow;
      refresh();
   }
}

const Action *Screen::findButtonAction(ButtonCode code) {

   if (currentPage != nullptr) {
      return currentPage->findButtonAction(code);
   }
   return nullptr;
}

/*
 * Shared Actions
 * ============================================================================================
 */
constexpr SonyTvAction   sonyTvOnOff(                     IrSonyTV::ON_OFF,          "TV On/Off",        1000_ticks);
constexpr SonyTvAction   sonyTvOn(                        IrSonyTV::ON,              "TV On",            1'000'000_ticks);
constexpr SonyTvAction   sonyTvOff(                       IrSonyTV::OFF,             "TV Off"            );

constexpr SonyTvAction   sonyTvSourceTv(                  IrSonyTV::SOURCE_TV,       "TV Source TV"      );
constexpr SonyTvAction   sonyTvSourceHdmi1_Chrome(        IrSonyTV::SOURCE_HDMI_1,   "TV Source HDMI 1"  );
constexpr SonyTvAction   sonyTvSourceHdmi2_PVR_Teac(      IrSonyTV::SOURCE_HDMI_2,   "TV Source HDMI 2"  );
constexpr SonyTvAction   sonyTvSourceHdmi3_DVD_Samsung(   IrSonyTV::SOURCE_HDMI_3,   "TV Source HDMI 3"  );
constexpr SonyTvAction   sonyTvSourceHdmi4_DVD_Laser(     IrSonyTV::SOURCE_HDMI_4,   "TV Source HDMI 4"  );
constexpr SonyTvAction   sonyTvSourceVideo1_DVD_Panasonic(IrSonyTV::SOURCE_Video_1,  "TV Source Video 1" );

constexpr SonyTvAction   sonyTvMute(                      IrSonyTV::MUTE,            "TV Mute",          100'000_ticks);

constexpr SonyTvAction   sonyTvVolumeUp(                  IrSonyTV::VOLUME_UP,       "TV Vol Up",        100'000_ticks);
constexpr SonyTvAction   sonyTvVolumeDown(                IrSonyTV::VOLUME_DOWN,     "TV Vol Down",      100'000_ticks);

//constexpr SonyTvAction   sonyTvHome(                      IrSonyTV::HOME,            "TV Home"           );
//constexpr SonyTvAction   sonyTvReturn(                    IrSonyTV::RETURN,          "TV Return"         );

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
SequenceAction< 1> backAction("Show Main Page");

const Page *pageBeforeHelp = nullptr;

class BackFromHelpAction : public Action {

public:
   BackFromHelpAction() : Action("A: Back From Help") {
   }

   virtual ~BackFromHelpAction() {
   }

   void action() const override {

      if (pageBeforeHelp == nullptr) {
         showMainPage.action();
      }
      else {
         Screen::show(pageBeforeHelp);
      }
      pageBeforeHelp = nullptr;
   }
};

BackFromHelpAction backFromHelpAction;

/*
 * Common buttons
 */
constexpr ImageButton<32> showMainPageButton     { showMainPage,        Exit,    Colour::RED, Colour::WHITE };
constexpr ImageButton<32> sonyTvVolumeUpButton   { sonyTvVolumeUp,      VolPlus  };
constexpr ImageButton<32> sonyTvVolumeDownButton { sonyTvVolumeDown,    VolMinus };
constexpr ImageButton<32> sonyTvMuteButton       { sonyTvMute,          Mute     };
constexpr ImageButton<32> backFromHelpButton     { backFromHelpAction,  Exit,    Colour::RED, Colour::WHITE };

/*
 * Screen pages
 * ============================================================================================
 */

class HelpPage : public PageWithButtons<7> {

protected:

   static inline constexpr TextButton buttons[6] {
      TextButton(laserDvdOnOff,     "1. Laser DVD"      ),
      TextButton(samsungDvdOnOff,   "2. Samsung DVD"    ),
      TextButton(teacPvrOnOff,      "3. Teac PVR"       ),
      TextButton(blaupunktDvdOnOff, "4. Blaupunkt DVD"  ),
      TextButton(panasonicDvdOnOff, "5. Panasonic DVD"  ),
      TextButton(sonyTvOnOff,       "6. Sony TV"        ),
   };

public:

   HelpPage() : PageWithButtons("Device On/Off") {

      for (unsigned index=0; index<(sizeof(buttons)/sizeof(buttons[0])); index++) {
         add(&buttons[index]);
      }
      add(&backFromHelpButton);
      layout();
   }

   ~HelpPage() = default;


   virtual const Action *findButtonAction(ButtonCode code) const override {

      // Maps physical button to screen button
      static const int buttonMapping[] = {
            //    1, 2, 3, up, 4, 5, 6, down, 7, 8,  9,  Left, OK,  0, Exit, Right
            /* */ 0, 1, 2, -1, 3, 4, 5, -1,   -1, -1, -1, -1,  -1,  -1,  6,   -1,
      };
      if (code>=sizeofArray(buttonMapping)) {
         return nullptr;
      }
      int mappedCode = buttonMapping[code];
      if (mappedCode<0) {
         return nullptr;
      }
      return mButtons.at(mappedCode).button->getAction();
   }

   void action() const override {
      Action::action();
      pageBeforeHelp = Screen::getCurrentPage();
      Screen::show(this);
   }

};

HelpPage       helpPage;

constexpr TextButton      helpPageButton         { helpPage,            "Help",           Colour::RED, Colour::WHITE};

class MainPage : public PageWithButtons<8> {

protected:
   static inline constexpr TextButton buttons[7] {
      TextButton( watchTv,             "1. Sony TV"         ),
      TextButton( watchTeacPvr,        "2. Teac PVR"        ),
      TextButton( watchLaserDvd,       "3. Laser DVD"       ),
      TextButton( watchPanasonicDVD,   "4. Panasonic DVD"   ),
      TextButton( watchSamsungDvd,     "5. Samsung DVD"     ),
//      TextButton( watchBlauPunktDVD,   "6. BlauPunkt DVD"   ),
      TextButton( allOff,              "6. All Off"         ),
      TextButton( helpPage,            "7. Help",           Colour::RED, Colour::WHITE),
   };
   
public:

   MainPage() : PageWithButtons("Main") {

      for (unsigned index=0; index<(sizeof(buttons)/sizeof(buttons[0])); index++) {
         add(&buttons[index]);
      }
      
      layout();   
   }

   ~MainPage() = default;

   virtual const Action *findButtonAction(ButtonCode code) const override {

      // Maps physical button to screen button
      static const int buttonMapping[] = {
            //    1, 2, 3, up, 4, 5, 6, down, 7,  8,  9, Left, OK,  0, Exit, Right
            /* */ 0, 1, 2, -1, 3, 4, 5, -1,   6, -1, -1, -1,  -1,  -1,  -1,   -1,
      };
      if (code>=sizeofArray(buttonMapping)) {
         return nullptr;
      }
      int mappedCode = buttonMapping[code];
      if (mappedCode<0) {
         return nullptr;
      }
      return mButtons.at(mappedCode).button->getAction();
   }

   void drawAll() const override {
      PageWithButtons::drawAll();
//      Screen::setStatusLine("TV|PVR|Laser|Panason");
   }

};

class TestAction : public Action {

   static inline const SonyTvAction tests[] = {
         SonyTvAction{IrSonyTV::Code::SOURCE_Video_1,     "SOURCE_1"    },
         SonyTvAction{IrSonyTV::Code::SOURCE_Video_2,     "SOURCE_2"    },
   };

   static inline unsigned index = 0;

public:

   TestAction() : Action("A: Test") {
   }

   virtual ~TestAction() = default;

   virtual void action() const {

      Screen::setStatusLine(tests[index].getTitle());
      console.writeln("Test ", tests[index].getTitle());
      tests[index].action();
      index = (index+1)%(sizeof(tests)/sizeof(tests[0]));

   }
};

class SonyTvPage : public PageWithButtons<22> {

protected:

   static inline constexpr SonyTvAction actions[16] = {
         /*  0   */ SonyTvAction{IrSonyTV::Code::NUM1,   "TV Num 1"   },
         /*  1   */ SonyTvAction{IrSonyTV::Code::NUM2,   "TV Num 2"   },
         /*  2   */ SonyTvAction{IrSonyTV::Code::NUM3,   "TV Num 3"   },
         /*  3   */ SonyTvAction{IrSonyTV::Code::UP,     "TV Up"      },

         /*  4   */ SonyTvAction{IrSonyTV::Code::NUM4,   "TV Num 4"   },
         /*  5   */ SonyTvAction{IrSonyTV::Code::NUM5,   "TV Num 5"   },
         /*  6   */ SonyTvAction{IrSonyTV::Code::NUM6,   "TV Num 6"   },
         /*  7   */ SonyTvAction{IrSonyTV::Code::DOWN,   "TV Down"    },

         /*  8   */ SonyTvAction{IrSonyTV::Code::NUM7,   "TV Num 7"   },
         /*  9   */ SonyTvAction{IrSonyTV::Code::NUM8,   "TV Num 8"   },
         /* 10   */ SonyTvAction{IrSonyTV::Code::NUM9,   "TV Num 9"   },
         /* 11   */ SonyTvAction{IrSonyTV::Code::LEFT,   "TV Left"    },

         /* Main Page */
         /* 12   */ SonyTvAction{IrSonyTV::Code::NUM0,   "TV Num 0"   },
         /* Mute */
         /* 13   */ SonyTvAction{IrSonyTV::Code::RIGHT,  "TV Right"   },

         /* Vol+ */
         /* Vol- */
         /* 14   */ SonyTvAction{IrSonyTV::Code::GUIDE,  "TV Guide"   },

         /* 15   */ SonyTvAction{IrSonyTV::Code::SOURCE, "TV Source"  },
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

         showMainPageButton,
         ImageButton<32>( actions[12],      Zero     ),
         sonyTvMuteButton,
         ImageButton<32>( actions[13],      Right    ),

         sonyTvVolumeUpButton,
         sonyTvVolumeDownButton,
         ImageButton<32>( actions[14],      Info     ),

   };
//   static inline TestAction testAction;
//   static inline constexpr TextButton sourceButton {actions[15], "Src"  };
//   static inline constexpr TextButton testButton   {testAction,  "Test" };

public:

   SonyTvPage() : PageWithButtons("Sony TV") {

      for (unsigned index=0; index<(sizeof(buttons)/sizeof(buttons[0])); index++) {
         add(&buttons[index]);
      }

      add(&helpPageButton);
//      add(&sourceButton);
//      add(&testButton);

      layout();
   }

   ~SonyTvPage() = default;

   virtual const Action *findButtonAction(ButtonCode code) const override {
      if (code > mButtons.size()) {
         return nullptr;
      }
      return getButtonAt((unsigned)code)->getAction();
   }

   void drawAll() const override {
      PageWithButtons::drawAll();
//      Screen::setStatusLine("Main|Mute|Vol-|Vol+");
   }
};

class SamsungDvdPage : public  PageWithButtons<20> {

protected:
   static inline constexpr SamsungDvdAction actions[15] = {
      /*  0 */ SamsungDvdAction{IrSamsungDVD::Code::REVERSE_SCENE, "DVD Reverse Scene" },
      /*  1 */ SamsungDvdAction{IrSamsungDVD::Code::UP           , "DVD Up"            },
      /*  2 */ SamsungDvdAction{IrSamsungDVD::Code::FORWARD_SCENE, "DVD Forward Scene" },
      /*  3 */ SamsungDvdAction{IrSamsungDVD::Code::PAUSE        , "DVD Pause"         },

      /*  4 */ SamsungDvdAction{IrSamsungDVD::Code::LEFT         , "DVD Left"          },
      /*  5 */ SamsungDvdAction{IrSamsungDVD::Code::OK           , "DVD OK"            },
      /*  6 */ SamsungDvdAction{IrSamsungDVD::Code::RIGHT        , "DVD Right"         },
      /*  7 */ SamsungDvdAction{IrSamsungDVD::Code::PLAY         , "DVD Play"          },

      /*  8 */ SamsungDvdAction{IrSamsungDVD::Code::REVERSE      , "DVD Fast Reverse"  },
      /*  9 */ SamsungDvdAction{IrSamsungDVD::Code::DOWN         , "DVD Down"          },
      /* 10 */ SamsungDvdAction{IrSamsungDVD::Code::FORWARD      , "DVD Fast Forward"  },
      /* 11 */ SamsungDvdAction{IrSamsungDVD::Code::STOP         , "DVD Halt"          },

      /* 12 */ SamsungDvdAction{IrSamsungDVD::Code::EJECT        , "DVD Eject"         },
      /* 13 */ SamsungDvdAction{IrSamsungDVD::Code::MENU         , "DVD Menu"          },
      /* 14 */ SamsungDvdAction{IrSamsungDVD::Code::INFO         , "DVD Info"          },
   };

   static inline constexpr ImageButton<32> buttons[19] {
      /* Scene Back    */ ImageButton<32>( actions[ 0], ReverseScene ),
      /* Up            */ ImageButton<32>( actions[ 1], Up           ),
      /* Scene Forward */ ImageButton<32>( actions[ 2], ForwardScene ),
      /* Pause         */ ImageButton<32>( actions[ 3], Pause        ),

      /* Left          */ ImageButton<32>( actions[ 4], Left         ),
      /* OK            */ ImageButton<32>( actions[ 5], Enter       ),
      /* Right         */ ImageButton<32>( actions[ 6], Right        ),
      /* Play          */ ImageButton<32>( actions[ 7], Play,        Colour::WHITE, Colour::BLUE ),

      /* Rewind        */ ImageButton<32>( actions[ 8], FastReverse  ),
      /* Down          */ ImageButton<32>( actions[ 9], Down         ),
      /* Fast Forward  */ ImageButton<32>( actions[10], FastForward  ),
      /* Stop          */ ImageButton<32>( actions[11], Halt         ),

      /* Vol +         */ sonyTvVolumeUpButton,
      /* Vol -         */ sonyTvVolumeDownButton,
      /* Mute          */ sonyTvMuteButton,
      /* Eject         */ ImageButton<32>( actions[12], Eject        ),

      /* Menu          */ ImageButton<32>( actions[13], Menu         ),
      /* Info          */ ImageButton<32>( actions[14], Info         ),
      /* Main page     */ showMainPageButton,
};
      
public:

   SamsungDvdPage() : PageWithButtons("Samsung DVD") {

      for (unsigned index=0; index<(sizeof(buttons)/sizeof(buttons[0])); index++) {
         add(&buttons[index]);
      }
      add(&helpPageButton);

      layout();
   }

   ~SamsungDvdPage() = default;

   virtual const Action *findButtonAction(ButtonCode code) const override {
      if (code > mButtons.size()) {
         return nullptr;
      }
      return getButtonAt((unsigned)code)->getAction();
   }

//   void drawAll() const override {
//      PageWithButtons::drawAll();
////      Screen::setStatusLine("Main|Mute|Vol-|Vol+");
//   }
};

class LaserDvdPage : public  PageWithButtons<20> {

protected:
   static inline constexpr LaserDvdAction actions[15] = {
         /*  0 */ LaserDvdAction{IrLaserDVD::Code::REVERSE_SCENE, "DVD Reverse Scene" },
         /*  1 */ LaserDvdAction{IrLaserDVD::Code::UP           , "DVD Up"            },
         /*  2 */ LaserDvdAction{IrLaserDVD::Code::FORWARD_SCENE, "DVD Forward Scene" },
         /*  3 */ LaserDvdAction{IrLaserDVD::Code::PAUSE        , "DVD Pause"         },

         /*  4 */ LaserDvdAction{IrLaserDVD::Code::LEFT         , "DVD Left"          },
         /*  5 */ LaserDvdAction{IrLaserDVD::Code::OK           , "DVD OK"            },
         /*  6 */ LaserDvdAction{IrLaserDVD::Code::RIGHT        , "DVD Right"         },
         /*  7 */ LaserDvdAction{IrLaserDVD::Code::PLAY         , "DVD Play"          },

         /*  8 */ LaserDvdAction{IrLaserDVD::Code::REVERSE      , "DVD Fast Reverse"  },
         /*  9 */ LaserDvdAction{IrLaserDVD::Code::DOWN         , "DVD Down"          },
         /* 10 */ LaserDvdAction{IrLaserDVD::Code::FORWARD      , "DVD Fast Forward"  },
         /* 11 */ LaserDvdAction{IrLaserDVD::Code::STOP         , "DVD Halt"          },

         /* 12 */ LaserDvdAction{IrLaserDVD::Code::EJECT        , "DVD Eject"         },
         /* 13 */ LaserDvdAction{IrLaserDVD::Code::MENU         , "DVD Menu"          },
         /* 14 */ LaserDvdAction{IrLaserDVD::Code::OSD          , "DVD OSD"           },
   };

   static inline constexpr ImageButton<32> buttons[19] {
      /* Scene Back    */ ImageButton<32>( actions[ 0], ReverseScene  ),
      /* Up            */ ImageButton<32>( actions[ 1], Up            ),
      /* Scene Forward */ ImageButton<32>( actions[ 2], ForwardScene  ),
      /* Pause         */ ImageButton<32>( actions[ 3], Pause         ),

      /* Left          */ ImageButton<32>( actions[ 4], Left          ),
      /* OK            */ ImageButton<32>( actions[ 5], Enter        ),
      /* Right         */ ImageButton<32>( actions[ 6], Right         ),
      /* Play          */ ImageButton<32>( actions[ 7], Play,         Colour::WHITE, Colour::BLUE ),

      /* Rewind        */ ImageButton<32>( actions[ 8], FastReverse   ),
      /* Down          */ ImageButton<32>( actions[ 9], Down          ),
      /* Fast Forward  */ ImageButton<32>( actions[10], FastForward   ),
      /* Stop          */ ImageButton<32>( actions[11], Halt          ),

      /* Vol +         */ sonyTvVolumeUpButton,
      /* Vol -         */ sonyTvVolumeDownButton,
      /* Mute          */ sonyTvMuteButton,
      /* Eject         */ ImageButton<32>( actions[12], Eject         ),

      /* Menu          */ ImageButton<32>( actions[13], Menu          ),
      /* Info          */ ImageButton<32>( actions[14], Info          ),
      /* Main page     */ showMainPageButton,
   };
      
public:

   LaserDvdPage() : PageWithButtons("Laser DVD") {

      for (unsigned index=0; index<(sizeof(buttons)/sizeof(buttons[0])); index++) {
         add(&buttons[index]);
      }
      add(&helpPageButton);

      layout();
   }

   ~LaserDvdPage() = default;

   virtual const Action *findButtonAction(ButtonCode code) const override {
      if (code > mButtons.size()) {
         return nullptr;
      }
      return getButtonAt((unsigned)code)->getAction();
   }

//   void drawAll() const override {
//      PageWithButtons::drawAll();
////      Screen::setStatusLine("Main|Mute|Vol-|Vol+");
//   }
};

class PanasonicDvdPage : public  PageWithButtons<20> {

protected:
   static inline constexpr PanasonicDvdAction actions[14] = {
         /*  0 */ PanasonicDvdAction( IrPanasonicDVD::Code::REVERSE_SCENE, "DVD Reverse Scene" ),
         /*  1 */ PanasonicDvdAction( IrPanasonicDVD::Code::UP           , "DVD Up"            ),
         /*  2 */ PanasonicDvdAction( IrPanasonicDVD::Code::FORWARD_SCENE, "DVD Forward Scene" ),
         /*  3 */ PanasonicDvdAction( IrPanasonicDVD::Code::PAUSE_PLAY   , "DVD Pause"         ),

         /*  4 */ PanasonicDvdAction( IrPanasonicDVD::Code::LEFT         , "DVD Left"          ),
         /*  5 */ PanasonicDvdAction( IrPanasonicDVD::Code::OK           , "DVD OK"            ),
         /*  6 */ PanasonicDvdAction( IrPanasonicDVD::Code::RIGHT        , "DVD Right"         ),
         /*  7 */ PanasonicDvdAction( IrPanasonicDVD::Code::PAUSE_PLAY   , "DVD Play"          ),

         /*  8 */ PanasonicDvdAction( IrPanasonicDVD::Code::REVERSE      , "DVD Fast Reverse"  ),
         /*  9 */ PanasonicDvdAction( IrPanasonicDVD::Code::DOWN         , "DVD Down"          ),
         /* 10 */ PanasonicDvdAction( IrPanasonicDVD::Code::FORWARD      , "DVD Fast Forward"  ),
         /* 11 */ PanasonicDvdAction( IrPanasonicDVD::Code::STOP         , "DVD Halt"          ),

         /* 12 */ PanasonicDvdAction( IrPanasonicDVD::Code::EJECT        , "DVD Eject"         ),
         /* 13 */ PanasonicDvdAction( IrPanasonicDVD::Code::MENU         , "DVD Menu"          ),
   };

   static inline constexpr ImageButton<32> buttons[18] {
      /* Scene Back    */ ImageButton<32>( actions[ 0], ReverseScene  ),
      /* Up            */ ImageButton<32>( actions[ 1], Up            ),
      /* Scene Forward */ ImageButton<32>( actions[ 2], ForwardScene  ),
      /* Pause         */ ImageButton<32>( actions[ 3], Pause         ),

      /* Left          */ ImageButton<32>( actions[ 4], Left          ),
      /* OK            */ ImageButton<32>( actions[ 5], Enter        ),
      /* Right         */ ImageButton<32>( actions[ 6], Right         ),
      /* Play          */ ImageButton<32>( actions[ 7], Play,         Colour::WHITE, Colour::BLUE ),

      /* Rewind        */ ImageButton<32>( actions[ 8], FastReverse   ),
      /* Down          */ ImageButton<32>( actions[ 9], Down          ),
      /* Fast Forward  */ ImageButton<32>( actions[10], FastForward   ),
      /* Stop          */ ImageButton<32>( actions[11], Halt          ),

      /* Vol +         */ sonyTvVolumeUpButton,
      /* Vol -         */ sonyTvVolumeDownButton,
      /* Mute          */ sonyTvMuteButton,
      /* Eject         */ ImageButton<32>( actions[12], Eject         ),

      /* Menu          */ ImageButton<32>( actions[13], Menu          ),
      /* Main page     */ showMainPageButton,
   };

public:

   PanasonicDvdPage() : PageWithButtons("Panasonic DVD") {

      for (unsigned index=0; index<(sizeof(buttons)/sizeof(buttons[0])); index++) {
         add(&buttons[index]);
      }
      add(&helpPageButton);

      layout();
   }

   ~PanasonicDvdPage() = default;

   virtual const Action *findButtonAction(ButtonCode code) const override {
      if (code > mButtons.size()) {
         return nullptr;
      }
      return getButtonAt((unsigned)code)->getAction();
   }

//   void drawAll() const override {
//      PageWithButtons::drawAll();
////      Screen::setStatusLine("Main|Mute|Vol-|Vol+");
//   }
};
 
class BlaupunktDvdPage : public  PageWithButtons<20> {

protected:
   static inline constexpr BlaupunktDvdAction actions[15] {
      /*  0 */ BlaupunktDvdAction{IrBlaupunktDVD::Code::REVERSE_SCENE, "DVD Reverse Scene" },
      /*  1 */ BlaupunktDvdAction{IrBlaupunktDVD::Code::UP           , "DVD Up"            },
      /*  2 */ BlaupunktDvdAction{IrBlaupunktDVD::Code::FORWARD_SCENE, "DVD Forward Scene" },
      /*  3 */ BlaupunktDvdAction{IrBlaupunktDVD::Code::PLAY_PAUSE   , "DVD Play/Pause"    },

      /*  4 */ BlaupunktDvdAction{IrBlaupunktDVD::Code::LEFT         , "DVD Left"          },
      /*  5 */ BlaupunktDvdAction{IrBlaupunktDVD::Code::OK           , "DVD OK"            },
      /*  6 */ BlaupunktDvdAction{IrBlaupunktDVD::Code::RIGHT        , "DVD Right"         },
      /*  7 */ BlaupunktDvdAction{IrBlaupunktDVD::Code::PLAY_PAUSE   , "DVD Play/Pause"    },

      /*  8 */ BlaupunktDvdAction{IrBlaupunktDVD::Code::REVERSE      , "DVD Fast Reverse"  },
      /*  9 */ BlaupunktDvdAction{IrBlaupunktDVD::Code::DOWN         , "DVD Down"          },
      /* 10 */ BlaupunktDvdAction{IrBlaupunktDVD::Code::FORWARD      , "DVD Fast Forward"  },
      /* 11 */ BlaupunktDvdAction{IrBlaupunktDVD::Code::STOP         , "DVD Halt"          },

      /* 12 */ BlaupunktDvdAction{IrBlaupunktDVD::Code::EJECT        , "DVD Eject"         },
      /* 13 */ BlaupunktDvdAction{IrBlaupunktDVD::Code::MENU         , "DVD Eject"         },
      /* 14 */ BlaupunktDvdAction{IrBlaupunktDVD::Code::OSD          , "DVD OSD"           },
   };

   static inline constexpr ImageButton<32> buttons[19] {
      /* Scene Back    */ ImageButton<32>( actions[ 0], ReverseScene  ),
      /* Up            */ ImageButton<32>( actions[ 1], Up            ),
      /* Scene Forward */ ImageButton<32>( actions[ 2], ForwardScene  ),
      /* Pause         */ ImageButton<32>( actions[ 3], Pause         ),

      /* Left          */ ImageButton<32>( actions[ 4], Left          ),
      /* OK            */ ImageButton<32>( actions[ 5], Enter         ),
      /* Right         */ ImageButton<32>( actions[ 6], Right         ),
      /* Play          */ ImageButton<32>( actions[ 7], Play,         Colour::WHITE, Colour::BLUE ),

      /* Rewind        */ ImageButton<32>( actions[ 8], FastReverse   ),
      /* Down          */ ImageButton<32>( actions[ 9], Down          ),
      /* Fast Forward  */ ImageButton<32>( actions[10], FastForward   ),
      /* Stop          */ ImageButton<32>( actions[11], Halt          ),

      /* Vol +         */ sonyTvVolumeUpButton,
      /* Vol -         */ sonyTvVolumeDownButton,
      /* Mute          */ sonyTvMuteButton,
      /* Eject         */ ImageButton<32>( actions[12], Eject         ),

      /* Menu          */ ImageButton<32>( actions[13], Menu          ),
      /* Info          */ ImageButton<32>( actions[14], Info          ),
      /* Main page     */ showMainPageButton,
   };
   
public:

   BlaupunktDvdPage() : PageWithButtons("Blaupunkt DVD") {

      for (unsigned index=0; index<(sizeof(buttons)/sizeof(buttons[0])); index++) {
         add(&buttons[index]);
      }
      add(&helpPageButton);

      layout();
   };

   ~BlaupunktDvdPage() = default;

   virtual const Action *findButtonAction(ButtonCode code) const override {
      if (code > mButtons.size()) {
         return nullptr;
      }
      return getButtonAt((unsigned)code)->getAction();
   }

//   void drawAll() const override {
//      PageWithButtons::drawAll();
////      Screen::setStatusLine("Main|Mute|Vol-|Vol+");
//   }
};

class TeacPvrEpgPage : public PageWithButtons<24> {

protected:
   static inline constexpr TeacPvrAction actions[21] {
      /*   0 */ TeacPvrAction{IrTeacPVR::Code::NUM1,  "Num 1"     },
      /*   1 */ TeacPvrAction{IrTeacPVR::Code::NUM2,  "Num 2"     },
      /*   2 */ TeacPvrAction{IrTeacPVR::Code::NUM3,  "Num 3"     },
      /*   3 */ TeacPvrAction{IrTeacPVR::Code::UP,    "PVR Up"    },

      /*   4 */ TeacPvrAction{IrTeacPVR::Code::NUM4,  "Num 4"     },
      /*   5 */ TeacPvrAction{IrTeacPVR::Code::NUM5,  "Num 5"     },
      /*   6 */ TeacPvrAction{IrTeacPVR::Code::NUM6,  "Num 6"     },
      /*   7 */ TeacPvrAction{IrTeacPVR::Code::DOWN,  "PVR Down"  },

      /*   8 */ TeacPvrAction{IrTeacPVR::Code::NUM7,  "Num 7"     },
      /*   9 */ TeacPvrAction{IrTeacPVR::Code::NUM8,  "Num 8"     },
      /*  10 */ TeacPvrAction{IrTeacPVR::Code::NUM9,  "Num 9"     },
      /*  11 */ TeacPvrAction{IrTeacPVR::Code::LEFT,  "PVR Left"  },

      /*  12 */ TeacPvrAction{IrTeacPVR::Code::OK,    "PVR OK"    }, // Text
      /*  13 */ TeacPvrAction{IrTeacPVR::Code::NUM0,  "Num 0"     },
      /*  14 */ TeacPvrAction{IrTeacPVR::Code::EXIT,  "PVR Exit"  }, // Text
      /*  15 */ TeacPvrAction{IrTeacPVR::Code::RIGHT, "PVR Right" },

      /*  16 */ TeacPvrAction{IrTeacPVR::Code::RED,   "PVR Red"   }, // Colour
      /*  17 */ TeacPvrAction{IrTeacPVR::Code::GREEN, "PVR Green" }, // Colour
      /*  18 */ TeacPvrAction{IrTeacPVR::Code::YELLOW,"PVR Yellow"}, // Colour
      /*  19 */ TeacPvrAction{IrTeacPVR::Code::BLUE,  "PVR Blue"  }, // Colour

      /*  20 */ TeacPvrAction{IrTeacPVR::Code::INFO,  "PVR Info"  },
   };

   static inline constexpr ImageButton<32> buttons[15] = {
         /*  0 */ ImageButton<32>( actions[ 0],      One      ),
         /*  1 */ ImageButton<32>( actions[ 1],      Two      ),
         /*  2 */ ImageButton<32>( actions[ 2],      Three    ),
         /*  3 */ ImageButton<32>( actions[ 3],      Up       ),

         /*  4 */ ImageButton<32>( actions[ 4],      Four     ),
         /*  5 */ ImageButton<32>( actions[ 5],      Five     ),
         /*  6 */ ImageButton<32>( actions[ 6],      Six      ),
         /*  7 */ ImageButton<32>( actions[ 7],      Down     ),

         /*  8 */ ImageButton<32>( actions[ 8],      Seven    ),
         /*  9 */ ImageButton<32>( actions[ 9],      Eight    ),
         /* 10 */ ImageButton<32>( actions[10],      Nine     ),
         /* 11 */ ImageButton<32>( actions[11],      Left     ),

         /* OK */
         /* 12 */ ImageButton<32>( actions[13],      Zero     ),
         /* Exit */
         /* 13 */ ImageButton<32>( actions[15],      Right    ),
         /* ... */
         /* 14 */ ImageButton<32>( actions[20],      Info     ),
   };

public:

   TeacPvrEpgPage() : PageWithButtons("PVR EPG") {

      for (unsigned index=0; index<12; index++) {
         add(&buttons[index]);
      }
      add(new TextButton (     actions[12], "OK"           ));
      add(&buttons[12]);
      add(new TextButton (     actions[14], "EXIT"         ));
      add(&buttons[13]);

      add(new ColourButton(    actions[16], 0,50, RED      ));
      add(new ColourButton(    actions[17], 0,50, GREEN    ));
      add(new ColourButton(    actions[18], 0,50, YELLOW   ));
      add(new ColourButton(    actions[19], 0,50, BLUE     ));

      add(new FillButton(0,0));
      add(new FillButton(0,0));
      add(&buttons[14]);
      add(new TextButton (     displayTeacPvrPage, "Back", Colour::RED, Colour::WHITE ));

      layout();
   }

   ~TeacPvrEpgPage() = default;

   virtual const Action *findButtonAction(ButtonCode code) const override {
      if (code > mButtons.size()) {
         return nullptr;
      }
      return getButtonAt((unsigned)code)->getAction();
   }

//   void drawAll() const override {
//      PageWithButtons::drawAll();
////      Screen::setStatusLine("Main|Mute|Vol-|Vol+");
//   }
};

TeacPvrEpgPage     teacPvrEpgPage;

class TeacPvrPage : public PageWithButtons<25> {

protected:
   static inline constexpr TeacPvrAction actions[18] {
      /*  0 */ TeacPvrAction{IrTeacPVR::Code::REVERSE_SCENE, "PVR Reverse Scene" },
      /*  1 */ TeacPvrAction{IrTeacPVR::Code::UP           , "PVR Up"            },
      /*  2 */ TeacPvrAction{IrTeacPVR::Code::FORWARD_SCENE, "PVR Forward Scene" },
      /*  3 */ TeacPvrAction{IrTeacPVR::Code::PAUSE        , "PVR Pause"         },

      /*  4 */ TeacPvrAction{IrTeacPVR::Code::LEFT         , "PVR Left"          },
      /*  5 */ TeacPvrAction{IrTeacPVR::Code::OK           , "PVR OK"            },
      /*  6 */ TeacPvrAction{IrTeacPVR::Code::RIGHT        , "PVR Right"         },
      /*  7 */ TeacPvrAction{IrTeacPVR::Code::PLAY         , "PVR Play"          },

      /*  8 */ TeacPvrAction{IrTeacPVR::Code::REVERSE      , "PVR Fast Reverse"  },
      /*  9 */ TeacPvrAction{IrTeacPVR::Code::DOWN         , "PVR Down"          },
      /* 10 */ TeacPvrAction{IrTeacPVR::Code::FORWARD      , "PVR Fast Forward"  },
      /* 11 */ TeacPvrAction{IrTeacPVR::Code::STOP         , "PVR Halt"          },

      /* 12 */ TeacPvrAction{IrTeacPVR::Code::MENU         , "PVR Menu"          },
      /* 13 */
      /* 14 */ TeacPvrAction{IrTeacPVR::Code::RED          , "PVR Red"           },
      /* 15 */ TeacPvrAction{IrTeacPVR::Code::GREEN        , "PVR Green"         },
      /* 16 */ TeacPvrAction{IrTeacPVR::Code::YELLOW       , "PVR Yellow"        },
      /* 17 */ TeacPvrAction{IrTeacPVR::Code::BLUE         , "PVR Blue"          },
      /* 18 */ TeacPvrAction{IrTeacPVR::Code::EXIT         , "PVR Exit"          },
   };
   static inline constexpr ImageButton<32> buttons[16] {
      /* Scene Back    */  ImageButton<32>( actions[ 0],       ReverseScene ),
      /* Up            */  ImageButton<32>( actions[ 1],       Up           ),
      /* Scene Forward */  ImageButton<32>( actions[ 2],       ForwardScene ),
      /* Pause         */  ImageButton<32>( actions[ 3],       Pause        ),

      /* Left          */  ImageButton<32>( actions[ 4],       Left         ),
      /* OK            */  ImageButton<32>( actions[ 5],       Enter       ),
      /* Right         */  ImageButton<32>( actions[ 6],       Right        ),
      /* Play          */  ImageButton<32>( actions[ 7],       Play,        Colour::WHITE, Colour::BLUE ),

      /* Rewind        */  ImageButton<32>( actions[ 8],       FastReverse  ),
      /* Down          */  ImageButton<32>( actions[ 9],       Down         ),
      /* Fast Forward  */  ImageButton<32>( actions[10],       FastForward  ),
      /* Stop          */  ImageButton<32>( actions[11],       Halt         ),

      /* Vol +         */  sonyTvVolumeUpButton,
      /* Vol -         */  sonyTvVolumeDownButton,
      /* Mute          */  sonyTvMuteButton,
      /* Menu          */  ImageButton<32>( actions[12],       Menu         ),
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
      add(&helpPageButton);

      layout();
  }

   ~TeacPvrPage() = default;

   virtual const Action *findButtonAction(ButtonCode code) const override {
      if (code > mButtons.size()) {
         return nullptr;
      }
      return getButtonAt((unsigned)code)->getAction();
   }

//   void drawAll() const override {
//      PageWithButtons::drawAll();
////      Screen::setStatusLine("Main|Mute|Vol-|Vol+");
//   }
};

/*
 * Page definitions
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

   //==============================
   allOff.add(sonyTvOff);
   allOff.add(mainPage);

   allOff.add(laserDvdOff);
   allOff.add(teacPvrOff);
   allOff.add(samsungDvdOff);
   allOff.add(panasonicDvdOff);
   allOff.add(blaupunktDvdOff);
   allOff.add(completeMessage);

   //==============================
   watchTv.add(sonyTvOn);
   watchTv.add(sonyTvPage);
//   watchTv.add(sonyTvHome);
//   watchTv.add(sonyTvReturn);
   watchTv.add(sonyTvSourceTv);

   watchTv.add(laserDvdOff);
   watchTv.add(teacPvrOff);
   watchTv.add(samsungDvdOff);
   watchTv.add(panasonicDvdOff);
   watchTv.add(blaupunktDvdOff);
   watchTv.add(completeMessage);

   //==============================
   watchTeacPvr.add(sonyTvOn);
   watchTeacPvr.add(teacPvrPage);
//   watchTeacPvr.add(sonyTvHome);
//   watchTeacPvr.add(sonyTvReturn);
   watchTeacPvr.add(sonyTvSourceHdmi2_PVR_Teac);

   watchTeacPvr.add(teacPvrOn);
   watchTeacPvr.add(laserDvdOff);
   watchTeacPvr.add(samsungDvdOff);
   watchTeacPvr.add(panasonicDvdOff);
   watchTeacPvr.add(blaupunktDvdOff);
   watchTeacPvr.add(completeMessage);

   //==============================
   watchLaserDvd.add(sonyTvOn);
   watchLaserDvd.add(laserDvdPage);
//   watchLaserDvd.add(sonyTvHome);
//   watchLaserDvd.add(sonyTvReturn);
   watchLaserDvd.add(sonyTvSourceHdmi4_DVD_Laser);

   watchLaserDvd.add(laserDvdOn);
   watchLaserDvd.add(teacPvrOff);
   watchLaserDvd.add(samsungDvdOff);
   watchLaserDvd.add(panasonicDvdOff);
   watchLaserDvd.add(blaupunktDvdOff);
   watchLaserDvd.add(completeMessage);

   //==============================
   watchSamsungDvd.add(sonyTvOn);
   watchSamsungDvd.add(samsungDvdPage);
//   watchSamsungDvd.add(sonyTvHome);
//   watchSamsungDvd.add(sonyTvReturn);
   watchSamsungDvd.add(sonyTvSourceHdmi3_DVD_Samsung);

   watchSamsungDvd.add(laserDvdOff);
   watchSamsungDvd.add(teacPvrOff);
   watchSamsungDvd.add(samsungDvdOn);
   watchSamsungDvd.add(panasonicDvdOff);
   watchSamsungDvd.add(blaupunktDvdOff);
   watchSamsungDvd.add(completeMessage);

   //==============================
   watchPanasonicDVD.add(sonyTvOn);
   watchPanasonicDVD.add(panasonicDvdPage);
//   watchPanasonicDVD.add(sonyTvHome);
//   watchPanasonicDVD.add(sonyTvReturn);
   watchPanasonicDVD.add(sonyTvSourceVideo1_DVD_Panasonic);

   watchPanasonicDVD.add(laserDvdOff);
   watchPanasonicDVD.add(teacPvrOff);
   watchPanasonicDVD.add(samsungDvdOff);
   watchPanasonicDVD.add(panasonicDvdOn);
   watchPanasonicDVD.add(blaupunktDvdOff);
   watchPanasonicDVD.add(completeMessage);

   //==============================
   watchBlauPunktDVD.add(sonyTvOn);
   watchBlauPunktDVD.add(blaupunktDvdPage);
//   watchBlauPunktDVD.add(sonyTvHome);
//   watchBlauPunktDVD.add(sonyTvReturn);
   watchBlauPunktDVD.add(sonyTvSourceHdmi2_PVR_Teac);

   watchBlauPunktDVD.add(laserDvdOff);
   watchBlauPunktDVD.add(teacPvrOff);
   watchBlauPunktDVD.add(samsungDvdOff);
   watchBlauPunktDVD.add(panasonicDvdOff);
   watchBlauPunktDVD.add(blaupunktDvdOn);
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

//void handler() {
//   console.writeln("Touch Irq");
//   touchInterface.disableTouchInterrupt();
//   wakeUp = true;
//}

/**
 * Return description of battery state
 *
 * @return Pointer to statically allocated string
 */
const char *checkBatteryLevel() {

   static const float Levels[] = {3.0, 3.7, 3.85, 3.95, 4.2};

   float battery = 2*3.3*BatteryLevel::readAnalogue(AdcResolution_16bit_se)/Adc0::getSingleEndedMaximum(AdcResolution_16bit_se);
   float first = 0.0;
   float second;
   int percent = 100;

   static char buff[10];

   const char *status = "unk";

   switch (LipoChargerStatus::read()) {
      case 0 : // No power - report battery level
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
         {
            StringFormatter sf(buff);
            sf.write(percent,"%");
         }
         status = buff;
         break;
      case 1 : // Charging - report charging
         status = "Ch+";
         break;
      case 2 : // Charging Completed - report charging done
         status = "OK ";
         break;
      case 3 : // Invalid ??
         status = "Inv";
         break;
   }

   static const char *lipoStatus[] = {
        /* STDBY, CHRG */
        /*   0      0  */ "No power",
        /*   0      1  */ "Charging",
        /*   1      0  */ "Charging Completed",
        /*   1      1  */ "Invalid",
   };
   DebugLed::write(LipoChargerStatus::read() == 0b10);
   console.writeln("Battery = ", battery, "V, ",percent, "%, ", lipoStatus[LipoChargerStatus::read()]);

   return status;
}

// PCR with pull-ups
static constexpr PcrInit gpioHighInit {

   PinAction_None,
   PinPull_Up,
   PinDriveMode_PushPull,
   PinDriveStrength_Low,
   PinFilter_Passive,
   PinSlewRate_Slow,
};

// PCR with pull-downs
static constexpr PcrInit gpioLowInit {

   PinAction_None,
   PinPull_Down,
   PinDriveMode_PushPull,
   PinDriveStrength_Low,
   PinFilter_Passive,
   PinSlewRate_Slow,
};

class ButtonScanner {

private:

// Debounced button state
static inline uint16_t buttonState = 0;

//
static inline uint16_t currentButton = 0;

// Current button polling state used to detect changes
static inline uint16_t poll          = 0;

// Count of how long buttons have been unchanged (stable)
static inline uint8_t  stableCount   = 0;

// Current button row being scanned
static inline uint8_t  row           = 0;

public:

static void buttonCallback() {

   const uint16_t current = SwitchCols::read()<<(row*4);
   const uint16_t mask    = 0b1111<<(row*4);

   if (current != (poll&mask)) {
      // Update snapshot
      poll  = (poll&~mask) | current;

      // Restart debounce timeout
      stableCount = 1;
   }

   if ((stableCount++ == 16) &&(currentButton != poll)) {
      // 16 x 1.25 ms = 20 ms
      // Each row would be scanned 4 times
      currentButton = poll;
   }

   // A row is driven high for 1 cycle before sampling columns
   // This sets up the _next_ row
   // Initially all rows are low so no buttons are accidently detected
   row = (row+1)%4;
   if (row==0) {
      SwitchRow1::set();
      SwitchRow4::clear();
   }
   else if (row==1) {
      SwitchRow2::set();
      SwitchRow1::clear();
   }
   else if (row==2) {
      SwitchRow3::set();
      SwitchRow2::clear();
   }
   else if (row==3) {
      SwitchRow4::set();
      SwitchRow3::clear();
   }
}

static void initialise() {

   // Columns as inputs with pull-downs
   SwitchCols::setInput(gpioLowInit);

   // Drive all rows low initially
   SwitchRow1::setOutput(gpioHighInit);
   SwitchRow2::setOutput(gpioHighInit);
   SwitchRow3::setOutput(gpioHighInit);
   SwitchRow4::setOutput(gpioHighInit);

   buttonState    = 0;
   currentButton  = 0;
   poll           = 0;
   stableCount    = 0;
   row            = 0;

   Pit::Init pitInit {
      PitOperation_Enabled,
      PitDebugMode_StopInDebug,
   };
   Pit::configure(pitInit);

   static constexpr Pit::ChannelInit pitChannelInit {
      ButtonTimerChannel::CHANNEL,

      PitChannelEnable_Enabled ,   // (pit_tctrl_ten[0])         Timer Channel Enable - Channel enabled
      PitChannelAction_Interrupt , // (pit_tctrl_tie[0])         Action on timer event - Interrupt
      59999_ticks,                 // (pit_ldval_tsv[0])         Reload value channel 0 - ~1.25 ms

      NvicPriority_Normal ,        // (irqLevel_Ch0)             IRQ priority level for Ch0 - Normal
      buttonCallback,              // (handlerName_Ch0)          User declared event handler
   };

   Pit::configure(pitChannelInit);
}

static void suspendPolling() {
   ButtonTimerChannel::disableNvicInterrupts();
}

static void resumePolling() {
   ButtonTimerChannel::enableNvicInterrupts();
}

static ButtonCode makeRelease(ButtonCode code) {

   return ButtonCode(code | Button_Release);
}

static ButtonCode getButton() {

   if (buttonState == currentButton) {
      return Button_None;
   }

   CriticalSection cs;

   // Find changed buttons
   uint16_t changed = (currentButton^buttonState);

   // Only process 1st changed button
   ButtonCode code =  ButtonCode(__builtin_ffs(changed)-1);

   uint16_t buttonMask = 1<<code;

   // Update state
   buttonState ^= buttonMask;

   if ((buttonState&buttonMask) == 0) {

      // Button released
      code = makeRelease(code);
   }

   return code;
}

}; // ButtonScanner

void initialiseMiscellaneous() {

   // Done in SIM configuration
   // SimInfo::setPortDPad(SimPortDPad_Double);

   ButtonScanner::initialise();

   LipoChargerStatus::setInput(gpioHighInit);

   DebugLed::setOutput(gpioLowInit);

   PowerEnable::setOutput(gpioLowInit);

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

   static constexpr Smc::Init smcInitValue = {
      SmcAllowVeryLowPower_Enabled ,       // (smc_pmprot_avlp)          Allow Very Low Power modes - VLPR, VLPW and VLPS are allowed
      SmcAllowLowLeakageStop_Enabled ,     // (smc_pmprot_alls)          Allow Low Leakage Stop mode - LLS is allowed
      SmcAllowVeryLowLeakageStop_Enabled , // (smc_pmprot_avlls)         Allow Very Low Leakage Stop mode - VLLSx is allowed
      SmcStopMode_VeryLowLeakageStop ,     // (smc_pmctrl_stopm)         Stop Mode Control - Very-Low-Leakage Stop (VLLSx)
      SmcLowLeakageStopMode_VLLS3,         // (smc_stopctrl_vllsm)       Low Leakage Mode Control - Enter VLLS3 in VLLSx mode
   };
   smcInitValue.setOptions();

}

void llwuCallback() {

   Llwu::disableNvicInterrupts();
}

void pinCallback() {

   static const PcrInit switchDefault {
      PinStatusFlag_ClearEvent,
      PinDriveMode_PushPull,
      PinDriveStrength_Low,
      PinAction_None,
      PinSlewRate_Slow,
   };

   // Set rows as inputs with pull-ups, clear ISF
   SwitchRow1::setOutput(switchDefault);
   SwitchRow2::setOutput(switchDefault);
   SwitchRow3::setOutput(switchDefault);
   SwitchRow4::setOutput(switchDefault);

   // Disable in NVIC
   SwitchRow1::disableNvicPinInterrupts();
   SwitchRow2::disableNvicPinInterrupts();
   SwitchRow3::disableNvicPinInterrupts();
   SwitchRow4::disableNvicPinInterrupts();
}

void wakeUpHandler() {
   console.writeln("Wakeup Irq");
   wakeUp = true;
}

void sleep() {

   ButtonScanner::suspendPolling();
   touchInterface.disableTouchInterrupt();
   tft.sleep();

   PowerEnable::off();

   spi.disable();

   Cmt::disable();

   // Drive columns low to use as inputs to wake-up pins
   SwitchCols::setOutput();
   SwitchCols::write(0b0000);

   static const PcrInit switchIn {
      PinAction_IrqFalling,
      PinPull_Up,
      PinStatusFlag_ClearEvent,
   };

   // Set rows as inputs with pull-ups
   SwitchRow1::setInput(switchIn);
   SwitchRow2::setInput(switchIn);
   SwitchRow3::setInput(switchIn);
   SwitchRow4::setInput(switchIn);

   // Set up pin interrupt handlers
   // Required for reliable entry to low power modes (avoids missed edges)
//   SwitchRow1::clearInterruptFlag();
//   SwitchRow2::clearInterruptFlag();
//   SwitchRow3::clearInterruptFlag();
//   SwitchRow4::clearInterruptFlag();

   SwitchRow1::setPinCallback(pinCallback);
   SwitchRow2::setPinCallback(pinCallback);
   SwitchRow3::setPinCallback(pinCallback);
   SwitchRow4::setPinCallback(pinCallback);

   SwitchRow1::enableNvicPinInterrupts(NvicPriority_Normal);
   SwitchRow2::enableNvicPinInterrupts(NvicPriority_Normal);
   SwitchRow3::enableNvicPinInterrupts(NvicPriority_Normal);
   SwitchRow4::enableNvicPinInterrupts(NvicPriority_Normal);

   // LLWU setup
   static constexpr Llwu::Init llwuInit {

      llwuCallback,
      NvicPriority_Normal,

      // Switch wake-ups
      LlwuPin_SwitchWakeupR1, LlwuPinMode_FallingEdge,
      LlwuPin_SwitchWakeupR2, LlwuPinMode_FallingEdge,
      LlwuPin_SwitchWakeupR3, LlwuPinMode_FallingEdge,
      LlwuPin_SwitchWakeupR4, LlwuPinMode_FallingEdge,

      // Reset wake-up
      LlwuResetWakeup_Enabled,
      LlwuResetFilter_Enabled,
   };

   Llwu::configure(llwuInit);

   console.writeln("Going to sleep...").flushOutput();

   ErrorCode rc;
   for(;;) {

      rc = Smc::enterPowerMode(SmcPowerMode_LLS);

      if (rc != E_INTERRUPTED) {
         break;
      }
      console.writeln("Interrupted, retrying...").flushOutput();
   };

   if (rc != E_NO_ERROR) {
      console.writeln("Failed!");
   }
}

#if defined(DEBUG_BUILD) && 0

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

void testTouch() {

   unsigned touchX, touchY;

   for(;;) {
      if (touchInterface.checkTouch(touchX, touchY)) {
         console.writeln("Touch @(", touchX, ",", touchY, ") ");
         continue;
      }
   }
}

void testTouchWakeup() {

   DebugLed::setOutput();
   touchInterface.setInterruptHandler(wakeUpHandler);
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

void testButton() {

   ButtonScanner::initialise();

   ButtonCode buttonCode;

   for(;;) {
      if ((buttonCode = ButtonScanner::getButton()) != Button_None) {
         bool release = buttonCode&Button_Release;
         buttonCode   = ButtonCode(buttonCode&~Button_Release);

         console.writeln("Button_", unsigned(buttonCode), release?" Released":" Pressed");
      }
   }
}

void testBattery() {

   initialiseMiscellaneous();

   for(;;) {
      checkBatteryLevel();
      waitMS(400);
   }
}

void testTiming() {

   constexpr SonyTvAction   sonyTvOn(         IrSonyTV::ON,          "TV On",            1000'000_ticks);
   constexpr SonyTvAction   sonyTvVolumeUp(   IrSonyTV::VOLUME_UP,   "TV Vol Up",        100'000_ticks);
   constexpr SonyTvAction   sonyTvVolumeDown( IrSonyTV::VOLUME_DOWN, "TV Vol Down",      100'000_ticks);

   DebugLed::setOutput();

   for(;;) {
      DebugLed::set();
      sonyTvOn.action();
      DebugLed::toggle();
      sonyTvVolumeUp.action();
      DebugLed::toggle();
      sonyTvVolumeDown.action();
      DebugLed::clear();
      waitMS(10000);
   }
}
#endif // DEBUG_BUILD

int main() {

   console.setBaudRate(baudRate);
   console.writeln("\n**************************************");
   console.writeln("Executing from RESET, SRS=", Rcm::getResetSourceDescription());

#ifdef DEBUG_BUILD
//   testTiming();
//   testBattery();

//   testButton();

//   testTouch();

//   calibrate();
//   checkCalibration();
//   testTouchWakeup();
   //   for(;;) {
   //      calibrate();
   //   }
#endif

   if ((Rcm::getResetSource() & RcmSource_Wakeup) != 0) {

      console.writeln("========================================");
      console.writeln("Reset due to LLWU");

      bool llwuDeviceFlag  = Llwu::getPeripheralWakeupSources()&LlwuPeripheral_Lptmr0;
      bool llwuPinFlag     =
            Llwu::isPinWakeupSource(LlwuPin_SwitchWakeupR1)||
            Llwu::isPinWakeupSource(LlwuPin_SwitchWakeupR2)||
            Llwu::isPinWakeupSource(LlwuPin_SwitchWakeupR3)||
            Llwu::isPinWakeupSource(LlwuPin_SwitchWakeupR4);
      bool llwuFilterFlag  = Llwu::isFilteredPinWakeupSource(LlwuFilterNum_1);

      console.writeln("LLWU DeviceFlag = ", llwuDeviceFlag);
      console.writeln("LLWU PinFlag    = ", llwuPinFlag);
      console.writeln("LLWU FilterFlag = ", llwuFilterFlag);

      Pmc::releaseIsolation();
   }

   //Errata e4481 STOP mode recovery unstable
//   static const Pmc::Init pmcInit {
//      PmcBandgapBuffer_Enabled,
//      PmcBandgapOperationInLowPower_Enabled,
//   };
//   Pmc::configure(pmcInit);

   Smc::enableAllPowerModes();

//   RFSYS->REG[0] = 100;

   unsigned touchX, touchY;

   initialiseMiscellaneous();

   initialiseGuiAndActions();

   allOff.action();
   Screen::setBusy(false);

   console.setEcho(EchoMode_Off);
   console.setBlocking(BlockingMode_Off);

   ButtonCode buttonCode;

   unsigned idleCount = 0;
   bool reinitialise  = true;

//   touchInterface.setInterruptHandler(wakeUpHandler);

   for(unsigned count = 0; ;count++) {

      if ((count % 1000) == 0) {
         Screen::reportBattery();
         checkBatteryLevel();
      }
      if (reinitialise) {

         reinitialise = false;

         Smc::enterRunMode(ClockConfig_RUN_PEE_48MHz);
         console.setBaudRate(baudRate);

         console.writeln("Awake!...");

         ButtonScanner::initialise();

         spi.enable();
         Cmt::enable();

         PowerEnable::on();

         waitMS(100);

         tft.initialise();

         tft.setBackgroundColour(BACKGROUND_COLOUR);
         tft.clear();

         idleCount = 0;

         Screen::refresh();
         tft.backlightOn();

         waitMS(500);
      }
#ifndef DEBUG_BUILD
      else if (touchInterface.checkTouch(touchX, touchY)) {
//         tft.setColour(GREEN);
//         tft.drawCircle(touchX,touchY, 10);
         idleCount = 0;
         //         console.writeln("\nLooking for touch @(", touchX, ",", touchY, ") ");
         const auto *action = Screen::findTouchAction(touchX, touchY);
         if (action != nullptr) {
            Screen::setBusy(true);
            action->action();
            Screen::setBusy(false);
            waitMS(100);
         }
         continue;
      }
#endif
      else if ((buttonCode = ButtonScanner::getButton()) != Button_None) {
         bool release = buttonCode&Button_Release;
         buttonCode   = ButtonCode(buttonCode&~Button_Release);
         console.writeln("Button_", unsigned(buttonCode), release?" Released":" Pressed");

         if (!release) {
            const auto *action = Screen::findButtonAction(buttonCode);
            if (action != nullptr) {
               Screen::setBusy(true);
               action->action();
               Screen::setBusy(false);
               waitMS(100);
            }
            idleCount = 0;
         }
      }
      else {
         idleCount++;
         if (idleCount>500) {
            sleep();
            reinitialise = true;
         }
      }
      waitMS(100);
   }
   return 0;
}

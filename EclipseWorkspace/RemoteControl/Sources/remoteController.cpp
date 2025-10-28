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
#include "hardware.h"
#include "smc.h"
#include "tft_IL9488.h"
//#include "tft_ILI9341.h"
//#include "tft_ILI9163.h"
//#include "tft_ST7735.h"
#include "touch_XPT2046.h"
#include "specialFonts.h"
#include "cmt-remote.h"

// Allow access to USBDM methods without USBDM:: prefix
using namespace USBDM;

using TFT=TFT_ILI9488<Orientation_Normal>;
//using TFT=TFT_ILI9341<Orientation_Normal>;;
//using TFT=TFT_ILI9488<Orientation_Normal>;
//using TFT=TFT_ST7735<Orientation_Normal>;

using TouchInterface = Touch_XPT2046<340, 480>;

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

static constexpr Font const &font = font16x24;

template<typename type, size_t capacity=20>
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
         if (itemPtr == nullptr) {
            exit(-1);
         }
         return itemPtr;
      }
      const reference operator*() const {
         if (itemPtr == nullptr) {
            exit(-1);
         }
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

   ConstantIterator begin() const { return ConstantIterator(&data[0]); }
   ConstantIterator end()   const { return ConstantIterator(&data[size]); }

   bool isEmpty() {
      return size == 0;
   }

   void push_back(type item) {

      if (size>=capacity) {
         exit(-1);
      }
      data[size++] = item;
   }

};

/*
 * ============================  Actions ============================
 */
class SequenceAction;

class Action {

protected:

   inline static bool busy = false;
   const char *title;
   bool   *status;
   bool   actionValue;

public:

   /**
    *  Create action
    *
    * @param title         Title for logging
    * @param status        Variable to update with status
    * @param actionValue   Status update value to use
    */
   constexpr Action(const char *title="No title", bool *status=nullptr, bool actionValue=false) :
   title(title), status(status), actionValue(actionValue) {
   }

   constexpr virtual ~Action() {
   }

   virtual void unGuardedAction() const {
      console.writeln("A:", title);
   }

   void guardedAction() const {
      if (status != nullptr) {
         // Only act if necessary
         if (*status != actionValue) {
            unGuardedAction();
            *status = actionValue;
         }
         else {
            console.writeln("A:", title, " - no action needed");
         }
      }
      else {
         // Always act
         unGuardedAction();
      }
   }
};

class Message : public Action {

public:
   constexpr Message(const char *message) : Action(message) {
   }

   ~Message() = default;

   void unGuardedAction() const override {
      console.writeln(title);
   }
};

class SequenceAction : public Action {

   MyVector<Action const *> actions;

public:

   SequenceAction(const char *title="Sequence...", bool *status=nullptr, bool actionValue=false) :
      Action(title, status, actionValue) {
   }

   void add(const Action *action) {
      actions.push_back(action);
   }

   void add(const Action &action) {
      actions.push_back(&action);
   }

   void unGuardedAction() const override {

      Action::unGuardedAction();
      for (const Action *action : actions) {
         action->guardedAction();
      }
   }

   void forcedAction() const {

      for (const Action *action : actions) {
         console.write("Forced: ");
         action->unGuardedAction();
      }
   }
};

class ForcedAction : public Action {

   const Action &action;

public:

   constexpr ForcedAction(const Action &action, const char *title="Forced") :
   Action(title), action(action) {
   }

public:

   void unGuardedAction() const override {
      Action::unGuardedAction();
      action.unGuardedAction();
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
   unsigned delayTime;

public:

   /**
    * Create IR action
    *
    * @param code          Code to send
    * @param title         Title for logging
    * @param delay         Delay after transmission
    * @param status        Variable to update with status
    * @param actionValue   Status update value to use
    */
   constexpr IrAction(
         const typename IrClass::Code  code,
         const char                   *title="IR title",
         Ticks                         delay=100_ticks,
         bool                         *status=nullptr,
         bool                          actionValue=false) :
         Action(title, status, actionValue),
         code(code), delayTime(delay) {
   }

   void unGuardedAction() const override {

      Action::unGuardedAction();
      IrClass::send(code, delayTime);
   }
};

using SonyTvAction      = IrAction<IrSonyTV>;
using LaserDvdAction    = IrAction<IrLaserDVD>;
using SamsungDvdAction  = IrAction<IrSamsungDVD>;
using TeacPvrAction     = IrAction<IrTeacPVR>;

/*
 * ============================  Buttons ============================
 */
static constexpr Colour BACKGROUND_COLOUR = Colour::BLACK;

class ButtonGroup;

class Button {

   friend ButtonGroup;

protected:
   Button *next = nullptr;

   const Action &action;
   unsigned x = 0;
   unsigned y = 0;
   static constexpr unsigned minimumWidth  = 77;
   static constexpr unsigned minimumHeight = 72;

public:
   static constexpr unsigned H_BORDER_WIDTH = 7;
   static constexpr unsigned V_BORDER_WIDTH = 6;
   const unsigned width;
   const unsigned height;
   const Colour   background;

   Button(unsigned width, unsigned height, const Action &action, Colour background=Colour::RED) :
      action(action),
      width(std::max(width, minimumWidth)), height(std::max(height, minimumHeight)),
      background(background) {
   };

   virtual ~Button() = default;

   template<unsigned N>
   void drawMyBitmap(const ButtonImage<N> &image, unsigned x, unsigned y, unsigned scale=1) const {
      tft.drawBitmap(image.data, x, y, image.width, image.height, scale);
   }

   virtual void draw() const {
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

   bool isHit(unsigned xx, unsigned yy) const {
      bool hit = (x<=xx)&&(xx<=(x+width))&&(y<=yy)&&(yy<=(y+height));
      return hit;
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

   void draw() const override {
      Button::draw();
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

   TextButton(const Action &action, const char *text, Colour foreground=Colour::WHITE, Colour background=Colour::RED) :
      Button(2*H_BORDER_WIDTH+strlen(text)*font.width, 2*V_BORDER_WIDTH+font.height, action, background),
      text(text),
      foreground(foreground) {
   }

   virtual ~TextButton() = default;

   void draw() const override {
      Button::draw();
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
   ColourButton(const Action &action, unsigned width, unsigned height, Colour colour) :
      Button(width, height, action, colour),
      colour(colour) {
   }

   virtual ~ColourButton() = default;

   void draw() const override {
      Button::draw();
   }
};

class FillButton : public Button {

protected:
   static inline const Action nullAction{};

public:
   FillButton(unsigned width, unsigned height) :
      Button(width, height, nullAction, BACKGROUND_COLOUR) {
   }

   virtual ~FillButton() = default;

   void draw() const override {
   }
};

class Page;

class ButtonGroup {

   friend Page;

private:

   Button *firstButton = nullptr;
   Button *lastButton  = nullptr;

   unsigned x;
   unsigned y;
   unsigned width;

   unsigned hSpace = 2;
   unsigned vSpace = 2;

   bool doneLayout = false;

public:

   ButtonGroup(unsigned x, unsigned y, unsigned width) : x(x), y(y), width(width) {
   }

   ~ButtonGroup() = default;

   void setSpacing(unsigned h, unsigned v) {
      hSpace = h;
      vSpace = v;
   }

   void layout() {

      bool firstInLine = true;
      unsigned xx = x;
      unsigned yy = y;
      unsigned maxHeight = firstButton->height;

      //      console.writeln("Layout (", x, ", ", y, ")[w=", width ,"]" );

      for (Button *button = firstButton; button != nullptr; button = button->next) {

         if (!firstInLine && (xx+button->width)>width) {
            // Put button on new line
            xx = x;
            yy += maxHeight+vSpace;
            maxHeight = 0;
         }
         if (button->height>maxHeight) {
            maxHeight = button->height;
         }
         button->x = xx;
         button->y = yy;
         //         console.writeln("Layout Button(", xx, ", ",yy,")");
         xx += button->width+hSpace;
         firstInLine = false;
      }
      doneLayout = true;
   }

   void add(Button *button) {

      if (lastButton == nullptr) {
         firstButton = button;
      }
      else {
         lastButton->next = button;
      }
      lastButton = button;
   }

   bool findAndExecuteHandler(unsigned x, unsigned y) const {

      for (Button *button = firstButton; button != nullptr; button = button->next) {

         if (button->isHit(x, y)) {
            console.writeln("=======================================");
            console.writeln("Button Hit @(", x, ",", y, ") ");
            const Action &action = button->action;
            action.unGuardedAction();
            return true;
         }
      }
      return false;
   }

   void drawAll() {

      if (!doneLayout) {
         layout();
      }
      Colour colour = tft.getBackgroundColour();
      for (Button *button = firstButton; button != nullptr; button = button->next) {
         tft.setBackgroundColour(colour);
         button->draw();
      }
   }
};

/*
 * ============================================================================================
 */
class Screen {

private:

   const Page *currentPage = nullptr;

public:

   Screen() {
   }

   ~Screen() = default;

   bool findAndExecuteHandler(unsigned x, unsigned y) const;

   void show(const Page &pageToShow);

   void setBusy(bool busy = true) const {

      static const char busyMessage[] = "Busy";
      //      static constexpr unsigned msgWidth = font.width*sizeof(busyMessage);

      tft.setBackgroundColour(busy?WHITE:BACKGROUND_COLOUR);
      tft.setColour(busy?RED:BACKGROUND_COLOUR);
      tft.setFont(font);
      tft.moveXY(0, 0).write(busyMessage);
   }
};

Screen screen;

/*
 * ============================================================================================
 */
class Page : public Action {

protected:
   const Font &font = font16x24;

private:

   MyVector<ButtonGroup *>buttonGroups;

   Page *next = nullptr;

public:

   Page(const char *title) : Action(title) {
   }

   ~Page() = default;

   void add(ButtonGroup *group) {

      buttonGroups.push_back(group);
   }

   bool findAndExecuteHandler(unsigned x, unsigned y) const {

      for (ButtonGroup *group:buttonGroups) {
         if (group->findAndExecuteHandler(x, y)) {
            return true;
         }
      }
      return false;
   }

   void drawAll(bool pageChanged) const {

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
         for (ButtonGroup *group:buttonGroups) {
            tft.setBackgroundColour(BACKGROUND_COLOUR);
            group->drawAll();
         }
      }
   }


   void unGuardedAction() const override {
      Action::unGuardedAction();
      screen.show(*this);
   }
};

/*
 * ============================================================================================
 */

bool Screen::findAndExecuteHandler(unsigned x, unsigned y) const {

   if (currentPage != nullptr) {

      setBusy(true);

      currentPage->findAndExecuteHandler(x, y);

      setBusy(false);
      return true;
   }
   return false;
}

void Screen::show(const Page &pageToShow) {

   bool pageChanged =  (currentPage != &pageToShow);
   currentPage = &pageToShow;
   currentPage->drawAll(pageChanged);
}

/*
 * Actions
 * ============================================================================================
 */
constexpr SonyTvAction   sonyTvOnOff(                     IrSonyTV::ON_OFF,          "TV On/Off",      1000_ticks);
constexpr SonyTvAction   sonyTvOn(                        IrSonyTV::ON,              "TV On",          1000_ticks);
constexpr SonyTvAction   sonyTvOff(                       IrSonyTV::OFF,             "TV Off"          );
constexpr SonyTvAction   sonyTvSourceHdmi1_Chrome(        IrSonyTV::SOURCE_HDMI_1,   "TV Source HDMI 1");
constexpr SonyTvAction   sonyTvSourceHdmi2_PVR(           IrSonyTV::SOURCE_HDMI_2,   "TV Source HDMI 2");
constexpr SonyTvAction   sonyTvSourceHdmi3_DVD_Samsung(   IrSonyTV::SOURCE_HDMI_3,   "TV Source HDMI 3");
constexpr SonyTvAction   sonyTvSourceHdmi4_DVD_Laser(     IrSonyTV::SOURCE_HDMI_4,   "TV Source HDMI 4");
constexpr SonyTvAction   sonyTvMute(                      IrSonyTV::MUTE,            "TV Mute"         );
constexpr SonyTvAction   sonyTvVolumeUp(                  IrSonyTV::VOLUME_UP,       "TV Vol Up"       );
constexpr SonyTvAction   sonyTvVolumeDown(                IrSonyTV::VOLUME_DOWN,     "TV Vol Down"     );
constexpr SonyTvAction   sonyTvHome(                      IrSonyTV::HOME,            "TV Home"         );
constexpr SonyTvAction   sonyTvReturn(                    IrSonyTV::RETURN,          "TV Return"       );
constexpr SonyTvAction   sonyTvSourceTv(                  IrSonyTV::SOURCE_TV,       "TV Source TV"    );

bool teacPvrPowerStatus  = false;
constexpr TeacPvrAction  teacPvrOnOff(     IrTeacPVR::ON_OFF,   "PVR On/Off",  100_ticks);
constexpr TeacPvrAction  teacPvrOn(        IrTeacPVR::ON_OFF,   "PVR On",      100_ticks,     &teacPvrPowerStatus,   true);
constexpr TeacPvrAction  teacPvrOff(       IrTeacPVR::ON_OFF,   "PVR Off",     100_ticks,     &teacPvrPowerStatus,   false);
//constexpr ForcedAction   teacPvrToggle(    teacPvrOff,          "PVR Toggle");

bool laserDvdPowerStatus  = false;
constexpr LaserDvdAction laserDvdOnOff(    IrLaserDVD::ON_OFF,  "Laser DVD On/Off",  100_ticks);
constexpr LaserDvdAction laserDvdOn(       IrLaserDVD::ON_OFF,  "Laser DVD On",      100_ticks,  &laserDvdPowerStatus,   true);
constexpr LaserDvdAction laserDvdOff(      IrLaserDVD::ON_OFF,  "Laser DVD Off",     100_ticks,  &laserDvdPowerStatus,   false);
//constexpr ForcedAction   laserDvdToggle(   laserDvdOff,         "Laser DVD Toggle");

bool samsungDvdPowerStatus  = false;
constexpr SamsungDvdAction samsungDvdOnOff(   IrSamsungDVD::ON_OFF,   "Samsung DVD On/Off",  100_ticks);
constexpr SamsungDvdAction samsungDvdOn(      IrSamsungDVD::ON_OFF,   "Samsung DVD On",      100_ticks,   &samsungDvdPowerStatus,   true);
constexpr SamsungDvdAction samsungDvdOff(     IrSamsungDVD::ON_OFF,   "Samsung DVD Off",     100_ticks,   &samsungDvdPowerStatus,   false);
//constexpr ForcedAction     samsungDvdToggle(  samsungDvdOff,          "Samsung DVD Toggle");


/*
 * Action sequences
 */
SequenceAction allOff{"Seq: All Off"};
SequenceAction watchTv{"Seq: Watch TV"};
SequenceAction watchSamsungDvd{"Seq: Watch Samsung DVD"};
SequenceAction watchLaserDvd{"Seq: Watch Laser DVD"};
SequenceAction watchTeacPvr{"Seq: Watch PVR"};
SequenceAction displayTeacPvrPage{"Seq: Display Teac DVD page"};
SequenceAction teacPvrEpisodeGuide{"Seq: Display Teac DVD Numbers page"};

/*
 * Common buttons
 */
SequenceAction showMainPage("Show Main Page");

// This button is a template used in copy assignment constructors
const TextButton showMainPageButton(showMainPage, "Main", Colour::RED, Colour::WHITE);

/*
 * Screen pages
 * ============================================================================================
 */

class HelpPage : public Page {

protected:
   ButtonGroup group{0, font.height, tft.WIDTH};

public:

   using Page::drawAll;

   HelpPage() : Page("Fix Each") {

      group.add(new TextButton(sonyTvOnOff,       "Sony TV On/Off"));
      group.add(new TextButton(teacPvrOnOff,      "Teac PVR On/Off"));
      group.add(new TextButton(laserDvdOnOff,     "Laser DVD On/Off"));
      group.add(new TextButton(samsungDvdOnOff,   "Samsung DVD On/Off"));
      group.add(new TextButton(showMainPageButton));

      add(&group);
   }

   ~HelpPage() = default;

};

class TestAction : public Action {

public:
   TestAction() : Action("Test") {
   }

   virtual void unGuardedAction() const {
      for (int i=0; i<100; i++) {
         IrLaserDVD::send(IrLaserDVD::DOWN, 0, 10);
      }
   }

};

HelpPage       helpPage;

class MainPage : public Page {

protected:
   ButtonGroup group{0, font.height+2U, tft.WIDTH};

public:

   using Page::drawAll;

   MainPage() : Page("Main") {
      static const TestAction testAction{};

      group.add(new TextButton( watchTv,         "Watch Sony TV"));
      group.add(new TextButton( watchTeacPvr,    "Watch Teac PVR"));
      group.add(new TextButton( watchLaserDvd,   "Watch Laser DVD"));
      group.add(new TextButton( watchSamsungDvd, "Watch Samsung DVD"));
      group.add(new TextButton( allOff,          "All Off"));
      group.add(new TextButton( helpPage,        "Help",             Colour::RED, Colour::WHITE));
//      group.add(new TextButton( testAction,      "test",     Colour::RED, Colour::WHITE));

      add(&group);
   }

   ~MainPage() = default;
};

class SonyTvPage : public Page {

protected:
   ButtonGroup group{0, font.height+2U, tft.WIDTH};

public:

   using Page::drawAll;

   SonyTvPage() : Page("Sony TV") {

      group.add(new TextButton     ( *new SonyTvAction{IrSonyTV::Code::NUM1, "Num 1" }, "1" ));
      group.add(new TextButton     ( *new SonyTvAction{IrSonyTV::Code::NUM2, "Num 2" }, "2" ));
      group.add(new TextButton     ( *new SonyTvAction{IrSonyTV::Code::NUM3, "Num 3" }, "3" ));
      group.add(new ImageButton<32>( sonyTvVolumeUp,                                    VolPlus     ));
      group.add(new TextButton     ( *new SonyTvAction{IrSonyTV::Code::NUM4, "Num 4" }, "4" ));
      group.add(new TextButton     ( *new SonyTvAction{IrSonyTV::Code::NUM5, "Num 5" }, "5" ));
      group.add(new TextButton     ( *new SonyTvAction{IrSonyTV::Code::NUM6, "Num 6" }, "6" ));
      group.add(new ImageButton<32>( sonyTvVolumeDown,                                  VolMinus    ));
      group.add(new TextButton     ( *new SonyTvAction{IrSonyTV::Code::NUM7, "Num 7" }, "7" ));
      group.add(new TextButton     ( *new SonyTvAction{IrSonyTV::Code::NUM8, "Num 8" }, "8" ));
      group.add(new TextButton     ( *new SonyTvAction{IrSonyTV::Code::NUM9, "Num 9" }, "9" ));
      group.add(new ImageButton<32>(      sonyTvMute,                                   Mute        ));
      group.add(new TextButton     ( *new SonyTvAction{IrSonyTV::Code::NUM0, "Num 0" }, "0" ));
      group.add(new TextButton     ( showMainPageButton));

      //      group.add(new TextButton (  *new SonyTvAction{IrSonyTV::Code::CHANNEL_UP,   "TV Channel+"  },  "Ch+"  ));
      //      group.add(new TextButton (  *new SonyTvAction{IrSonyTV::Code::CHANNEL_DOWN, "TV Channel-"  },  "Ch-"  ));
      //      group.add(new ColourButton( *new SonyTvAction{IrSonyTV::Code::RED,          "TV RED"       },  60, 60, Colour::RED    ));
      //      group.add(new ColourButton( *new SonyTvAction{IrSonyTV::Code::GREEN,        "TV GREEN"     },  60, 60, Colour::GREEN  ));
      //      group.add(new ColourButton( *new SonyTvAction{IrSonyTV::Code::YELLOW,       "TV YELLOW"    },  60, 60, Colour::YELLOW ));
      //      group.add(new ColourButton( *new SonyTvAction{IrSonyTV::Code::BLUE,         "TV BLUE"      },  60, 60, Colour::BLUE   ));
      //      group.add(new TextButton (  *new SonyTvAction{IrSonyTV::Code::GUIDE,        "Guide"        },  "Guide"  ));
      //      group.add(new TextButton ( *new SonyTvAction{IrSonyTV::Code::UP  , "UP"    }, "+" ));
      //      group.add(new TextButton ( *new SonyTvAction{IrSonyTV::Code::DOWN, "DOWN"  }, "-" ));

      add(&group);
   }

   ~SonyTvPage() = default;

};

class SamsungDvdPage : public  Page {

protected:
   ButtonGroup group{0, font.height+2U, tft.WIDTH};

public:

   using Page::drawAll;

   SamsungDvdPage() : Page("Samsung DVD") {

      group.add(new ImageButton<32>( *new SamsungDvdAction{IrSamsungDVD::Code::REVERSE_SCENE, "DVD Reverse Scene" }, ReverseScene ));
      group.add(new ImageButton<32>( *new SamsungDvdAction{IrSamsungDVD::Code::UP           , "DVD Up"            }, Up           ));
      group.add(new ImageButton<32>( *new SamsungDvdAction{IrSamsungDVD::Code::FORWARD_SCENE, "DVD Forward Scene" }, ForwardScene ));
      group.add(new ImageButton<32>( *new SamsungDvdAction{IrSamsungDVD::Code::PAUSE        , "DVD Pause"         }, Pause        ));
      group.add(new ImageButton<32>( *new SamsungDvdAction{IrSamsungDVD::Code::LEFT         , "DVD Left"          }, Left         ));
      group.add(new TextButton(      *new SamsungDvdAction{IrSamsungDVD::Code::OK           , "DVD OK"            }, "OK"         ));
      group.add(new ImageButton<32>( *new SamsungDvdAction{IrSamsungDVD::Code::RIGHT        , "DVD Right"         }, Right        ));
      group.add(new ImageButton<32>( *new SamsungDvdAction{IrSamsungDVD::Code::PLAY         , "DVD Play"          }, Play,   Colour::WHITE, Colour::BLUE ));
      group.add(new ImageButton<32>( *new SamsungDvdAction{IrSamsungDVD::Code::REVERSE      , "DVD Fast Reverse"  }, FastReverse  ));
      group.add(new ImageButton<32>( *new SamsungDvdAction{IrSamsungDVD::Code::DOWN         , "DVD Down"          },  Down         ));
      group.add(new ImageButton<32>( *new SamsungDvdAction{IrSamsungDVD::Code::FORWARD      , "DVD Fast Forward"  }, FastForward  ));
      group.add(new ImageButton<32>( *new SamsungDvdAction{IrSamsungDVD::Code::STOP         , "DVD Halt"          }, Halt         ));
      group.add(new ImageButton<32>(      sonyTvVolumeUp,                                                         VolPlus     ));
      group.add(new ImageButton<32>(      sonyTvVolumeDown,                                                       VolMinus    ));
      group.add(new ImageButton<32>(      sonyTvMute,                                                             Mute        ));
      group.add(new ImageButton<32>( *new SamsungDvdAction{IrSamsungDVD::Code::EJECT        , "DVD Eject"         }, Eject        ));
      group.add(new ImageButton<32>( *new SamsungDvdAction{IrSamsungDVD::Code::MENU         , "DVD Menu"          }, Menu         ));
      group.add(new ImageButton<32>( *new SamsungDvdAction{IrSamsungDVD::Code::INFO         , "DVD Info"          }, Info         ));
      group.add(new TextButton(showMainPageButton));

      add(&group);
   }

   ~SamsungDvdPage() = default;

};

class LaserDvdPage : public  Page {

protected:
   ButtonGroup group{0, font.height+2U, tft.WIDTH};

public:

   using Page::drawAll;

   LaserDvdPage() : Page("Laser DVD") {

      group.add(new ImageButton<32>( *new LaserDvdAction{IrLaserDVD::Code::REVERSE_SCENE, "DVD Reverse Scene" }, ReverseScene ));
      group.add(new ImageButton<32>( *new LaserDvdAction{IrLaserDVD::Code::UP           , "DVD Up"            }, Up           ));
      group.add(new ImageButton<32>( *new LaserDvdAction{IrLaserDVD::Code::FORWARD_SCENE, "DVD Forward Scene" }, ForwardScene ));
      group.add(new ImageButton<32>( *new LaserDvdAction{IrLaserDVD::Code::PAUSE        , "DVD Pause"         }, Pause        ));
      group.add(new ImageButton<32>( *new LaserDvdAction{IrLaserDVD::Code::LEFT         , "DVD Left"          }, Left         ));
      group.add(new TextButton(      *new LaserDvdAction{IrLaserDVD::Code::OK           , "DVD OK"            }, "OK"         ));
      group.add(new ImageButton<32>( *new LaserDvdAction{IrLaserDVD::Code::RIGHT        , "DVD Right"         }, Right        ));
      group.add(new ImageButton<32>( *new LaserDvdAction{IrLaserDVD::Code::PLAY         , "DVD Play"          }, Play,   Colour::WHITE, Colour::BLUE ));
      group.add(new ImageButton<32>( *new LaserDvdAction{IrLaserDVD::Code::REVERSE      , "DVD Fast Reverse"  }, FastReverse  ));
      group.add(new ImageButton<32>( *new LaserDvdAction{IrLaserDVD::Code::DOWN         , "DVD Down"          },  Down         ));
      group.add(new ImageButton<32>( *new LaserDvdAction{IrLaserDVD::Code::FORWARD      , "DVD Fast Forward"  }, FastForward  ));
      group.add(new ImageButton<32>( *new LaserDvdAction{IrLaserDVD::Code::STOP         , "DVD Halt"          }, Halt         ));
      group.add(new ImageButton<32>(      sonyTvVolumeUp,                                                         VolPlus     ));
      group.add(new ImageButton<32>(      sonyTvVolumeDown,                                                       VolMinus    ));
      group.add(new ImageButton<32>(      sonyTvMute,                                                             Mute        ));
      group.add(new ImageButton<32>( *new LaserDvdAction{IrLaserDVD::Code::EJECT        , "DVD Eject"         }, Eject        ));
      group.add(new ImageButton<32>( *new LaserDvdAction{IrLaserDVD::Code::MENU         , "DVD Eject"         }, Menu         ));
      group.add(new ImageButton<32>( *new LaserDvdAction{IrLaserDVD::Code::OSD          , "DVD OSD"           }, Info         ));
      group.add(new TextButton(showMainPageButton));

      add(&group);
   }

   ~LaserDvdPage() = default;

};

class TeacPvrEpgPage : public Page {

protected:
   ButtonGroup group{0, font.height+2U, tft.WIDTH};

public:

   using Page::drawAll;

   TeacPvrEpgPage() : Page("PVR EPG") {

      group.add(new TextButton (     *new TeacPvrAction{IrTeacPVR::Code::NUM1,  "Num 1"     }, "1"     ));
      group.add(new TextButton (     *new TeacPvrAction{IrTeacPVR::Code::NUM2,  "Num 2"     }, "2"     ));
      group.add(new TextButton (     *new TeacPvrAction{IrTeacPVR::Code::NUM3,  "Num 3"     }, "3"     ));
      group.add(new TextButton(      *new TeacPvrAction{IrTeacPVR::Code::OK,    "PVR OK"    }, "OK"    ));
      group.add(new TextButton (     *new TeacPvrAction{IrTeacPVR::Code::NUM4,  "Num 4"     }, "4"     ));
      group.add(new TextButton (     *new TeacPvrAction{IrTeacPVR::Code::NUM5,  "Num 5"     }, "5"     ));
      group.add(new TextButton (     *new TeacPvrAction{IrTeacPVR::Code::NUM6,  "Num 6"     }, "6"     ));
      group.add(new TextButton(      *new TeacPvrAction{IrTeacPVR::Code::EXIT,  "PVR Exit"  }, "EXIT"  ));
      group.add(new TextButton (     *new TeacPvrAction{IrTeacPVR::Code::NUM7,  "Num 7"     }, "7"     ));
      group.add(new TextButton (     *new TeacPvrAction{IrTeacPVR::Code::NUM8,  "Num 8"     }, "8"     ));
      group.add(new TextButton (     *new TeacPvrAction{IrTeacPVR::Code::NUM9,  "Num 9"     }, "9"     ));
      group.add(new TextButton (     *new TeacPvrAction{IrTeacPVR::Code::NUM0,  "Num 0"     }, "0"     ));
      group.add(new ImageButton<32>( *new TeacPvrAction{IrTeacPVR::Code::UP,    "PVR Up"    }, Up      ));
      group.add(new ImageButton<32>( *new TeacPvrAction{IrTeacPVR::Code::DOWN,  "PVR Down"  }, Down    ));
      group.add(new ImageButton<32>( *new TeacPvrAction{IrTeacPVR::Code::LEFT,  "PVR Left"  }, Left    ));
      group.add(new ImageButton<32>( *new TeacPvrAction{IrTeacPVR::Code::RIGHT, "PVR Right" }, Right   ));
      group.add(new ColourButton(    *new TeacPvrAction{IrTeacPVR::Code::RED,   "PVR Red"   }, 0,50, RED    ));
      group.add(new ColourButton(    *new TeacPvrAction{IrTeacPVR::Code::GREEN, "PVR Green" }, 0,50, GREEN  ));
      group.add(new ColourButton(    *new TeacPvrAction{IrTeacPVR::Code::YELLOW,"PVR Yellow"}, 0,50, YELLOW ));
      group.add(new ColourButton(    *new TeacPvrAction{IrTeacPVR::Code::BLUE,  "PVR Blue"  }, 0,50, BLUE   ));
      group.add(new FillButton(0,0));
      group.add(new FillButton(0,0));
      group.add(new FillButton(0,0));
      group.add(new TextButton (     displayTeacPvrPage,                                       "Back"  ,  Colour::RED, Colour::WHITE ));


      //      group.add(new ColourButton ( *new TeacPvrAction{IrTeacPVR::Code::BLUE,   "Blue"   }, 20, 20, BLUE ));
      //      group.add(new ColourButton ( *new TeacPvrAction{IrTeacPVR::Code::YELLOW, "Yellow" }, 20, 20, YELLOW ));
      //      group.add(new ColourButton ( *new TeacPvrAction{IrTeacPVR::Code::RED,    "PDn"    }, 20, 20, YELLOW ));
      //      group.add(new ColourButton ( *new TeacPvrAction{IrTeacPVR::Code::GREEN,  "PUp"    }, 20, 20, BLUE ));
      add(&group);
   }

   ~TeacPvrEpgPage() = default;
};

TeacPvrEpgPage     teacPvrEpgPage;

class TeacPvrPage : public  Page {

protected:
   ButtonGroup group{0, font.height+2U, tft.WIDTH};

public:

   using Page::drawAll;

   TeacPvrPage() : Page("Teac PVR") {

      group.add(new ImageButton<32>( *new TeacPvrAction{IrTeacPVR::Code::REVERSE_SCENE, "PVR Reverse Scene" }, ReverseScene ));
      group.add(new ImageButton<32>( *new TeacPvrAction{IrTeacPVR::Code::UP           , "PVR Up"            }, Up           ));
      group.add(new ImageButton<32>( *new TeacPvrAction{IrTeacPVR::Code::FORWARD_SCENE, "PVR Forward Scene" }, ForwardScene ));
      group.add(new ImageButton<32>( *new TeacPvrAction{IrTeacPVR::Code::PAUSE        , "PVR Pause"         }, Pause        ));
      group.add(new ImageButton<32>( *new TeacPvrAction{IrTeacPVR::Code::LEFT         , "PVR Left"          }, Left         ));
      group.add(new TextButton(      *new TeacPvrAction{IrTeacPVR::Code::OK           , "PVR OK"            }, "OK"         ));
      group.add(new ImageButton<32>( *new TeacPvrAction{IrTeacPVR::Code::RIGHT        , "PVR Right"         }, Right        ));
      group.add(new ImageButton<32>( *new TeacPvrAction{IrTeacPVR::Code::PLAY         , "PVR Play"          }, Play,   Colour::WHITE, Colour::BLUE ));
      group.add(new ImageButton<32>( *new TeacPvrAction{IrTeacPVR::Code::REVERSE      , "PVR Fast Reverse"  }, FastReverse  ));
      group.add(new ImageButton<32>( *new TeacPvrAction{IrTeacPVR::Code::DOWN         , "PVR Down"          }, Down         ));
      group.add(new ImageButton<32>( *new TeacPvrAction{IrTeacPVR::Code::FORWARD      , "PVR Fast Forward"  }, FastForward  ));
      group.add(new ImageButton<32>( *new TeacPvrAction{IrTeacPVR::Code::STOP         , "PVR Halt"          }, Halt         ));
      group.add(new ImageButton<32>( *new TeacPvrAction{IrTeacPVR::Code::MENU         , "PVR Menu"          }, Menu         ));
      group.add(new ImageButton<32>(      sonyTvVolumeUp,                                                         VolPlus     ));
      group.add(new ImageButton<32>(      sonyTvVolumeDown,                                                       VolMinus    ));
      group.add(new ImageButton<32>(      sonyTvMute,                                                             Mute        ));
      group.add(new ColourButton(    *new TeacPvrAction{IrTeacPVR::Code::RED,           "PVR Red"    }, 0,50, RED    ));
      group.add(new ColourButton(    *new TeacPvrAction{IrTeacPVR::Code::GREEN,         "PVR Green"  }, 0,50, GREEN  ));
      group.add(new ColourButton(    *new TeacPvrAction{IrTeacPVR::Code::YELLOW,        "PVR Yellow" }, 0,50, YELLOW ));
      group.add(new ColourButton(    *new TeacPvrAction{IrTeacPVR::Code::BLUE,          "PVR Blue"   }, 0,50, BLUE   ));

      group.add(new TextButton(           teacPvrEpisodeGuide,                          "EPG"        ));
      group.add(new TextButton(      *new TeacPvrAction{IrTeacPVR::Code::EXIT         , "PVR EXIT"         }, "EXIT"        ));
      group.add(new TextButton(           showMainPageButton                                                                ));
      group.add(new FillButton(0,0));

      add(&group);
   }

   ~TeacPvrPage() = default;

};

/*
 * Page definitions
 * ============================================================================================
 */

MainPage       mainPage;
SonyTvPage     sonyTvPage;
TeacPvrPage    teacPvrPage;
LaserDvdPage   laserDvdPage;
SamsungDvdPage samsungDvdPage;

Message completeMessage("Complete");

void initialiseButtonsAndActions() {

   showMainPage.add(mainPage);

   allOff.add(sonyTvOff);
   allOff.add(mainPage);
   allOff.add(laserDvdOff);
   allOff.add(teacPvrOff);
   allOff.add(samsungDvdOff);
   allOff.add(completeMessage);

   watchTv.add(sonyTvOn);
   watchTv.add(sonyTvHome);
   watchTv.add(sonyTvReturn);
   watchTv.add(sonyTvSourceTv);
   watchTv.add(sonyTvPage);
   watchTv.add(laserDvdOff);
   watchTv.add(teacPvrOff);
   watchTv.add(samsungDvdOff);
   watchTv.add(completeMessage);

   watchLaserDvd.add(sonyTvOn);
   watchLaserDvd.add(sonyTvHome);
   watchLaserDvd.add(sonyTvReturn);
   watchLaserDvd.add(sonyTvSourceHdmi4_DVD_Laser);
   watchLaserDvd.add(laserDvdOn);
   watchLaserDvd.add(laserDvdPage);
   watchLaserDvd.add(teacPvrOff);
   watchLaserDvd.add(samsungDvdOff);
   watchLaserDvd.add(completeMessage);

   watchSamsungDvd.add(sonyTvOn);
   watchSamsungDvd.add(sonyTvHome);
   watchSamsungDvd.add(sonyTvReturn);
   watchSamsungDvd.add(sonyTvSourceHdmi3_DVD_Samsung);
   watchSamsungDvd.add(samsungDvdOn);
   watchSamsungDvd.add(samsungDvdPage);
   watchSamsungDvd.add(teacPvrOff);
   watchSamsungDvd.add(completeMessage);

   watchTeacPvr.add(sonyTvOn);
   watchTeacPvr.add(sonyTvHome);
   watchTeacPvr.add(sonyTvReturn);
   watchTeacPvr.add(sonyTvSourceHdmi2_PVR);
   watchTeacPvr.add(teacPvrOn);
   watchTeacPvr.add(teacPvrPage);
   watchTeacPvr.add(laserDvdOff);
   watchTeacPvr.add(samsungDvdOff);
   watchTeacPvr.add(completeMessage);

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

void handler() {
   console.writeln("Touch Irq");
   touchInterface.disableTouchInterrupt();
   wakeUp = true;
}

int getBatteryLevel() {

   return 100*BatteryLevel::readAnalogue(AdcResolution_16bit_se)/Adc0::getSingleEndedMaximum(AdcResolution_16bit_se);
}

void initiliaseMiscellaneous() {

   static constexpr PcrInit gpioInit {

      PinAction_None,
      PinPull_Up,
      PinDriveMode_PushPull,
      PinDriveStrength_Low,
      PinFilter_Passive,
      PinSlewRate_Slow,
   };
   Charging::setInput(gpioInit);
   Switches::setInput(gpioInit);

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

int main() {

   console.writeln("== Starting ==\n");

   initiliaseMiscellaneous();

   //   for(;;) {
   //      calibrate();
   //   }
   tft.setBackgroundColour(BACKGROUND_COLOUR);
   tft.clear();

   initialiseButtonsAndActions();

   //   screen.listPages();
   allOff.unGuardedAction();
   screen.setBusy(false);

   console.setEcho(EchoMode_Off);
   console.setBlocking(BlockingMode_Off);

   unsigned idleCount = 0;
   bool sleeping = false;

   touchInterface.setInterruptHandler(handler);
   for(;;) {

      unsigned touchX, touchY;

      if (wakeUp) {
         wakeUp = false;
         sleeping = false;
         tft.awaken();
         idleCount = 0;
         waitMS(500);
      }
      else if (touchInterface.checkTouch(touchX, touchY)) {
//         tft.setColour(GREEN);
//         tft.drawCircle(touchX,touchY, 10);
         idleCount = 0;
//         console.writeln("\nLooking for touch @(", touchX, ",", touchY, ") ");
         if (!screen.findAndExecuteHandler(touchX, touchY)) {
            waitMS(100);
         }
         continue;
      }
      else {
         if (sleeping) {
            continue;
         }
         idleCount++;
         if (idleCount>2000) {
            console.writeln("Entering low power mode");
            tft.sleep();
            touchInterface.enableTouchInterrupt();
            sleeping = true;
            Smc::enterWaitMode();
         }
      }
      waitMS(10);
   }
   return 0;
}

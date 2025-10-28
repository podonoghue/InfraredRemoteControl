/*
 * FrameBuffer.h
 *
 *  Created on: 26 Sept 2025
 *      Author: peter
 */

#ifndef SOURCES_FRAMEBUFFER_H_
#define SOURCES_FRAMEBUFFER_H_

#include <memory.h>
#include "formatted_io.h"
#include "fonts.h"

namespace USBDM {

/**
 * How the new pixels are combined with existing
 */
enum WriteMode {
   WriteMode_Write,        /**< Write new value       */
   WriteMode_InverseWrite, /**< Write inverted value  */
   WriteMode_Or,           /**< Combine new pixel with existing by ORing */
   WriteMode_InverseAnd,   /**< Combine new pixel with existing by inversion and ANDing */
   WriteMode_Xor,          /**< Combine new pixel with existing by XORing */
};

/**
 * Mirroring of the screen
 */
enum MirrorMode {
   MirrorMode_None,    /**< None               */
   MirrorMode_X,       /**< Mirror on X axis   */
   MirrorMode_Y,       /**< Mirror on Y axis   */
   MirrorMode_Origin,  /**< Mirror on X=Y axis */
};

/**
 * Rotation of the screen
 */
enum Rotate {
   Rotate_0,    /**< Rotate 0 degrees    */
   Rotate_90,   /**< Rotate 90 degrees   */
   Rotate_180,  /**< Rotate 180 degrees  */
   Rotate_270,  /**< Rotate 270 degrees  */
};

/**
 * Scaling of pixels to physical pixels
 */
enum Scale {
   Scale_1  = 1,  /**< Scale x 1 */
   Scale_2  = 2,  /**< Scale x 2 */
   Scale_4  = 4,  /**< Scale x 4 */
};

enum Colour {
   BLACK = 0b0000'0000,
   WHITE = 0b1111'1111,
};

class FrameBuffer : public FormattedIO {

protected:

   // Rotation of the screen
   Rotate rotate = Rotate_0;

   // Mirroring of the screen
   MirrorMode mirrorMode = MirrorMode_None;

   // Write mode - How the new pixels are combined with existing
   WriteMode writeMode = WriteMode_Write;

   // Scaling of image
   Scale scale = Scale_1;

   // X position
   unsigned x = 0;

   // Y position
   unsigned y = 0;

   // Graphic mode font height (for newline)
   unsigned fontHeight = 0;

   // Current font
   const USBDM::Font *font;

   // Default colour for painting unless specified in call
   Colour colour = WHITE;

   template<typename T>
   constexpr T max(T a, T b) {
      return (a>b)?a:b;
   }

   void drawHorizontalLine(unsigned x0, unsigned y0, unsigned x1);
   void drawVerticalLine(unsigned x0, unsigned y0, unsigned y1);

   /**
    * Check if character is available
    *
    * @return true  Character available i.e. _readChar() will not block
    * @return false No character available
    */
   __attribute__((deprecated))
   virtual bool _isCharAvailable() override {
      return false;
   }

   /**
    * Receives a character (blocking)
    *
    * @return Character received
    */
   __attribute__((deprecated))
   virtual int _readChar() override {
      return -1;
   }

   /**
    *  Flush input data
    */
   __attribute__((deprecated))
   virtual FrameBuffer &flushInput() override {
      return *this;
   }

   void _writeChar(char ch) override;

   void combine(unsigned address, uint8_t mask, Colour colour);
   bool mapCoordinate(unsigned &x, unsigned &y);

public:
   // Physical display height in pixels
   const unsigned height;

   // Physical d width in pixels
   const unsigned width;

   // Frame buffer. Size is getFrameSize()
   uint8_t *frameBuffer;

   /**
    * Create frame buffer
    *
    * @param frameBuffer   Pre-allocated frame buffer
    * @param font          Initial font to use
    */
   FrameBuffer(unsigned height, unsigned width, const Font *font, uint8_t *fb) :
      FormattedIO(),
      font(font),
      height(height),
      width(width),
      frameBuffer(fb) {
   }

   virtual ~FrameBuffer() {
   }

   /**
    * Set scaling of pixels to physical pixels
    *
    * @param scale Scale to set
    */
   FrameBuffer &setScale(Scale scale) {

      this->scale = scale;
      return *this;
   }

   /**
    * Set mirroring of the screen
    *
    * @param mirrorMode Mode to set
    */
   FrameBuffer &setMirror(MirrorMode mirrorMode) {

      this->mirrorMode = mirrorMode;
      return *this;
   }

   /**
    * Set rotation of the screen
    *
    * @param rotate Rotation to set
    */
   FrameBuffer &setRotate(Rotate rotate) {

      this->rotate = rotate;
      return *this;
   }

   /**
    * Set colour for painting
    *
    * @param colour  Colour to use
    */
   void setColour(Colour colour) {
      this->colour = colour;
   }

   /**
    * Sets how new pixels are combined with existing
    *
    * @param writeMode Mode to set
    */
   FrameBuffer &setWriteMode(WriteMode writeMode) {

      this->writeMode = writeMode;
      return *this;
   }

   /**
    * Get current font
    *
    * @return The current font
    */
   const Font *getFont() {

      return font;
   }

   /**
    * Set font to use for subsequent operations
    *
    * @param [in] font
    *
    * @return Reference to self
    */
   FrameBuffer &setFont(const USBDM::Font &font) {

      this->font = &font;
      return *this;
   }

   /**
    * Write a custom character to the LCD in graphics mode at the current x,y location
    *
    * @param[in] image  Image describing the character
    * @param[in] width  Width of the image
    * @param[in] height Height of character
    *
    * @return Reference to self
    */
   FrameBuffer &putCustomChar(const uint8_t *image, unsigned width, unsigned height) {

      writeImage(image, x, y, width, height);
      x += width;
      fontHeight = max(fontHeight, height);

      return *this;
   }

   /**
    * Clear frame buffer
    *
    * @param pixel Value to set buffer to
    *
    * @return Reference to self
    */
   FrameBuffer &clear(Colour colour) {

      x = 0;
      y = 0;
      fontHeight = 0;

      uint8_t mask = colour;

      memset(frameBuffer, mask, getFrameSize());

      return *this;
   }

   void paintPixel(unsigned x, unsigned y, Colour colour);
   void writeImage(const uint8_t *dataPtr, unsigned x, unsigned y, unsigned width, unsigned height);
   FrameBuffer &putSpace(int width);
   void drawLine(unsigned x0, unsigned y0, unsigned x1, unsigned y1);
   void drawRect(unsigned x0, unsigned y0, unsigned x1, unsigned y1);
   void drawOpenRect(unsigned x0, unsigned y0, unsigned x1, unsigned y1);
   void drawCircle(unsigned X, unsigned Y, unsigned Radius);
   void drawOpenCircle(unsigned X, unsigned Y, unsigned Radius);

   /**
    * Move current location
    *
    * @param x
    * @param y
    *
    * @return Reference to self
    */
   FrameBuffer &moveXY(unsigned x, unsigned y) {

      this->x = x;
      this->y = y;

      return *this;
   }

   /**
    * Get current X location
    *
    * @return X location in pixels
    */
   int getX() {

      return x;
   }

   /**
    * Get current Y location
    *
    * @return Y location in pixels
    */
   int getY() {

      return y;
   }

   /**
    * Get the size of the frame buffer
    *
    * @return Size in bytes
    */
   unsigned getFrameSize() {

      return height*((width+7)/8);
   }

   /**
    * Get current X,Y location
    *
    * @param [out] x X location in pixels
    * @param [out] y Y location in pixels
    */
   void getXY(int &x, int &y) {

      x = this->x;
      y = this->y;
   }
};

} // end namespace USBDM

#endif /* SOURCES_FRAMEBUFFER_H_ */

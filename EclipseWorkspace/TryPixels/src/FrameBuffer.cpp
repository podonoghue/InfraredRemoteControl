/*
 * FrameBuffer.cpp
 *
 *  Created on: 26 Sept 2025
 *      Author: peter
 */
#include <cstdlib>
#include "FrameBuffer.h"
#include "console.h"
using namespace USBDM;

/**
 * Write image to frame buffer
 *
 * @param [in] image      Pointer to start of image
 * @param [in] x          X position of top-left corner
 * @param [in] y          Y position of top-left corner
 * @param [in] width      Width of image
 * @param [in] height     Height of image
 *
 * @return Reference to self
 */
void FrameBuffer::writeImage(const uint8_t *image, unsigned x, unsigned y, unsigned width, unsigned height) {

   for(unsigned h=0; h<height; h++) {
      for(unsigned w=0; w<width; w++) {

         // Get pixel value from image
         unsigned pixelIndex = (h*((width+7)/8))+(w/8);
         bool pixel = (image[pixelIndex]&(1<<(7-(w&0b111))));

         // Paint pixel
         paintPixel(x+w, y+h, pixel?colour:(Colour)~colour);
      }
   }
}

/**
 * Write a character to the frame buffer at the current x,y location
 *
 * @param[in]  ch - character to write
 */
void FrameBuffer::_writeChar(char ch) {

   unsigned width  = font->width;
   unsigned height = font->height;

   if (ch == '\n') {
      putSpace(width-x);
      x  = 0;
      y += fontHeight;
      fontHeight = 0;
   }
   else {
      if ((x+width)>this->width) {
         // Don't display partial characters
         return;
      }
      writeImage((*font)[ch], x, y, width, height);
      x += width;
      fontHeight = max(fontHeight, height);
   }
   return;
}

/**
 * Writes whitespace to the frame buffer at the current x,y location
 *
 * @param[in] WIDTH Width of white space in pixels
 *
 * @return Reference to self
 */
FrameBuffer &FrameBuffer::putSpace(int width) {

   static const uint8_t space[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
   while (width>0) {
      int t = 8;
      if (t>width) {
         t = width;
      }
      putCustomChar(space, t, 8);
      width -= t;
   }
   return *this;
}

/**
 * Combine a new pixel with existing frame buffer value
 *
 * @param address    Frame buffer address
 * @param mask       Mask indicating pixels in value
 * @param newPixel   New pixel value
 */
void FrameBuffer::combine(unsigned address, uint8_t mask, Colour colour) {

   if (address>getFrameSize()) {
#if 0
      console.WRITELN("Illegal offset");
      __BKPT();
#else
      exit(-1);
#endif
   }
   uint8_t oldValue = frameBuffer[address];

   uint8_t newValue = oldValue;

   switch(writeMode) {
      case WriteMode_InverseWrite:  colour = (Colour)~colour; [[fallthrough]];
      case WriteMode_Write:         newValue = (oldValue&~mask)|(colour&mask);  break;
      case WriteMode_Or:            newValue = oldValue|(colour&mask);          break;
      case WriteMode_Xor:           newValue = oldValue^(colour&mask);          break;
      case WriteMode_InverseAnd:    newValue = oldValue&(~colour|~mask);        break;
   }
   frameBuffer[address] = newValue;
}

/**
 * Map coordinate from canvas to frame buffer before scaling.
 * Only handles rotation and mirroring.
 *
 * @param [in/out] x          Mapped X coordinate from canvas -> frame buffer (before scaling)
 * @param [in/out] y          Mapped Y coordinate from canvas -> frame buffer (before scaling)
 *
 * @return true Coordinate is within frame buffer
 * @return false Coordinate is outside frame buffer
 */
bool FrameBuffer::mapCoordinate(unsigned &x, unsigned &y) {

   // Dimensions of drawing canvas (from frame buffer with scaling i.e. in scaled pixels)
   unsigned _width  = width/scale;
   unsigned _height = height/scale;

   switch (rotate) {
      case Rotate_0:                                                         break;
      case Rotate_90:   {unsigned t = x; x = y;          y = _height-t-1; }  break;
      case Rotate_180:                   x = _width-x-1; y = _height-y-1;    break;
      case Rotate_270:  {unsigned t = x; x = _width-y-1; y = t;           }  break;
   }
   switch(mirrorMode) {
      case MirrorMode_None:                                   break;
      case MirrorMode_X:      x = _width-x-1;                 break;
      case MirrorMode_Y:      y = _height-y-1;                break;
      case MirrorMode_Origin: {unsigned t = x; x = y; y = t;} break;
   }
   // Check if clipped [0.._width-1][0..height-1]
   // Result is forced to right/bottom limits
   bool onScreen = true;
   if (x>=_width) {
      //      x = _width-1;
      onScreen = false;
   }
   if (y>=_height) {
      //      x = _height-1;
      onScreen = false;
   }
   return onScreen;
}

/**
 * Paints a pixel to frame buffer
 * It takes care of scaling, rotation and mirroring.
 *
 * @param x          X coordinate
 * @param y          Y coordinate
 * @param pixel      Pixel colour
 */
void FrameBuffer::paintPixel(unsigned x, unsigned y, Colour colour) {

   // Effective dimensions with scaling and rotation

   if (!mapCoordinate(x, y)) {
      // Clipped
      __asm__("nop");
      return;
   }

   //   console.WRITELN("r(%d,%d)->", x, y);
   switch (scale) {
      case Scale_1: /* x = 8 pixels/byte; y = 1 row; */ {
         uint8_t mask = 0b1000'0000>>(x&0b111);
         x /= 8;
         unsigned address = x + y*((width+7)/8);
         //         console.WRITELN("s(%d,%d), address = %d, mask=%2X\n", x, y, address, mask);
         combine(address, mask, colour);
      }
      break;
      case Scale_2: /* x = 4 pixels/byte; y= 2 rows */ {
         uint8_t mask = 0b1100'0000>>(2*(x&0b11));
         x /= 4;
         y *= 2;
         for (unsigned row=0; row<2; row++) {
            unsigned address = x + (y+row)*((width+7)/8);
            //            console.WRITELN("s(%d,%d), address = %d, mask=%2X\n", x, y, address, mask);
            combine(address, mask, colour);
         }
      }
      break;
      case Scale_4: /* x = 2 pixels/byte; y = 4 rows */ {
         uint8_t mask = 0b1111'0000>>(4*(x&0b1));
         x /= 2;
         y *= 4;
         for (unsigned row=0; row<4; row++) {
            unsigned address = x + (y+row)*((width+7)/8);
            //            console.WRITELN("s(%d,%d), address = %d, mask=%2X\n", x, y, address, mask);
            combine(address, mask, colour);
         }
      }
      break;
   }
}

/**
 * Simple line drawing for physical y0=y1
 *
 * @param x0   Start X
 * @param y0   Start Y
 * @param x1   End X
 */
void FrameBuffer::drawHorizontalLine(unsigned x0, unsigned y0, unsigned x1) {

   // Horizontal on display
   switch (scale) {
      case Scale_1:
      /* x = 8 pixels/byte; y = 1 row/pixel */ {
         uint8_t mask     = 0b1111'1111>>(x0&0b111);
         uint8_t lastMask = 0b1111'1111'1000'0000>>(x1&0b111);
         x0 /= 8;
         x1 /= 8;
         unsigned address = x0 + y0*((width+7)/8);
         for (unsigned x=x0; x<=x1; x++) {
            if (address>=getFrameSize()) {
               // Clipped
               return;
            }
            if (x==x1) {
               // last byte
               mask &= lastMask;
            }
            combine(address, mask, colour);
            address++;
            mask = 0b11111111;
         }
      }
      break;
      case Scale_2:
      /* x = 4 pixels/byte; y= 2 rows/pixel */ {
         uint8_t mask     = 0b1111'1111>>(2*(x0&0b111));
         uint8_t lastMask = 0b1111'1111'1000'0000>>(2*(x1&0b111));
         x0 /= 4;
         x1 /= 4;
         y0 *= 2;
         unsigned address = x0 + y0*((width+7)/8);
         for (unsigned x=x0; x<=x1; x++) {
            if (address>=getFrameSize()) {
               // Clipped
               return;
            }
            if (x==x1) {
               // last byte
               mask &= lastMask;
            }
            combine(address,               mask, colour);
            combine(address+((width+7)/8), mask, colour);
            address++;
            mask = 0b11111111;
         }
      }
      break;
      case Scale_4:
      /* x = 2 pixels/byte; y = 4 rows/pixel */ {
         uint8_t mask     = 0b1111'1111>>(4*(x0&0b111));
         uint8_t lastMask = 0b1111'1111'1000'0000>>(4*(x1&0b111));
         x0 /= 2;
         x1 /= 2;
         y0 *= 4;
         unsigned address = x0 + y0*((width+7)/8);
         for (unsigned x=x0; x<=x1; x++) {
            if (address>=getFrameSize()) {
               // Clipped
               return;
            }
            if (x==x1) {
               // last byte
               mask &= lastMask;
            }
            combine(address,                 mask, colour);
            combine(address+((width+7)/8),   mask, colour);
            combine(address+2*((width+7)/8), mask, colour);
            combine(address+3*((width+7)/8), mask, colour);
            address++;
            mask = 0b11111111;
         }
      }
      break;
   }
}

/**
 * Simple line drawing for physical x0=x1
 *
 * @param x0   Start X
 * @param y0   Start Y
 * @param y1   End Y
 */
void FrameBuffer::drawVerticalLine(unsigned x0, unsigned y0, unsigned y1) {

   // Vertical on display
   switch (scale) {
      case Scale_1:
      /* x = 8 pixels/byte; y = 1 row/pixel */ {
         uint8_t mask     = (0b1000'0000>>(x0&0b111));
         x0 /= 8;
         unsigned address = x0 + y0*((width+7)/8);
         for (unsigned y=y0; y<=y1; y++) {
            if (address>=getFrameSize()) {
               // Clipped
               return;
            }
            combine(address, mask, colour);
            address += ((width+7)/8);
         }
      }
      break;
      case Scale_2:
      /* x = 4 pixels/byte; y= 2 rows/pixel */ {
         uint8_t mask     = (0b1100'0000>>(2*(x0&0b11)));
         x0 /= 4;
         y0 *= 2;
         y1 = 2*y1 + 1;
         unsigned address = x0 + y0*((width+7)/8);
         for (unsigned y=y0; y<=y1; y++) {
            if (address>=getFrameSize()) {
               // Clipped
               return;
            }
            combine(address, mask, colour);
            address += ((width+7)/8);
         }
      }
      break;
      case Scale_4:
      /* x = 2 pixels/byte; y = 4 rows/pixel */ {
         uint8_t mask     = (0b1111'0000>>(4*(x0&0b1)));
         x0 /= 2;
         y0 *= 4;
         y1  = 4*y1 + 3;
         unsigned address = x0 + y0*((width+7)/8);
         for (unsigned y=y0; y<=y1; y++) {
            if (address>=getFrameSize()) {
               // Clipped
               return;
            }
            combine(address, mask, colour);
            address += ((width+7)/8);
         }
      }
      break;
   }
}

/**
 * Simple line drawing using Bresenham's algorithm
 * Ref : https://en.wikipedia.org/wiki/Bresenham's_line_algorithm
 *
 * @param x0   Start X
 * @param y0   Start Y
 * @param x1   End X
 * @param y1   End Y
 */
void FrameBuffer::drawLine(unsigned x0, unsigned y0, unsigned x1, unsigned y1) {

   if (x0>x1) {
      unsigned t = x0;
      x0 = x1;
      x1 = t;
   }
   if (y0>y1) {
      unsigned t = y0;
      y0 = y1;
      y1 = t;
   }
   if (!mapCoordinate(x0, y0)) {
      // off screen
      return;
   }
   if (!mapCoordinate(x1, y1)) {
      // Clipped is OK
   }

   if (y0==y1) {
      drawHorizontalLine(x0, y0, x1);
   }
   else if (x0==x1) {
      drawVerticalLine(x0, y0, y1);
   }
   else {
      int dx = std::abs((int)x1 - (int)x0);
      int sx = (x0 < x1) ? 1 : -1;
      int dy = -abs((int)y1 - (int)y0);
      int sy = (y0 < y1) ? 1 : -1;
      int error = dx + dy;

      while (true) {
         paintPixel(x0, y0, colour);
         int e2 = 2 * error;
         if (e2 >= dy) {
            if (x0 == x1) {
               break;
            }
            error = error + dy;
            x0 = x0 + sx;
         }
         if (e2 <= dx) {
            if (y0 == y1) {
               break;
            }
            error = error + dx;
            y0 = y0 + sy;
         }
      }

   }
}

/**
 *
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 */
void FrameBuffer::drawRect(unsigned x0, unsigned y0, unsigned x1, unsigned y1) {

   for (unsigned y=y0; y<=y1; y++) {
      drawHorizontalLine(x0, y, x1);
   }
}

/**
 *
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 */
void FrameBuffer::drawOpenRect(unsigned x0, unsigned y0, unsigned x1, unsigned y1) {

   drawHorizontalLine(x0, y0, x1);
   drawHorizontalLine(x0, y1, x1);
   drawVerticalLine(x0, y0+1, y1-1);
   drawVerticalLine(x1, y0+1, y1-1);
}

/**
 * Draw filled circle
 *
 * @param X       Circle centre X
 * @param Y       Circle centre Y
 * @param Radius  Circle radius
 */
void FrameBuffer::drawCircle(unsigned X, unsigned Y, unsigned Radius) {

   int16_t f = 1 - Radius;
   int16_t ddF_x = 1;
   int16_t ddF_y = -2 * Radius;
   int16_t x = 0;
   int16_t y = Radius;

   // Solid Circle - Draw the fill line
   drawLine(X - Radius, Y, X + Radius, Y);

   while (x < y)
   {
      if (f >= 0)
      {
         y--;
         ddF_y += 2;
         f += ddF_y;
      }
      x++;
      ddF_x += 2;
      f += ddF_x;

      // Solid Circle - Draw the fill line
      drawLine(X - x, Y + y, X + x, Y + y);
      drawLine(X - x, Y - y, X + x, Y - y);
      drawLine(X - y, Y + x, X + y, Y + x);
      drawLine(X - y, Y - x, X + y, Y - x);
   }
}

/**
 * Draw open circle
 *
 * @param X       Circle centre X
 * @param Y       Circle centre Y
 * @param Radius  Circle radius
 */
void FrameBuffer::drawOpenCircle(unsigned X, unsigned Y, unsigned Radius) {

   int16_t f = 1 - Radius;
   int16_t ddF_x = 1;
   int16_t ddF_y = -2 * Radius;
   int16_t x = 0;
   int16_t y = Radius;

   // Hollow Circle - Draw 8 points of symmetry
   paintPixel(X, Y + Radius, colour);
   paintPixel(X, Y - Radius, colour);
   paintPixel(X + Radius, Y, colour);
   paintPixel(X - Radius, Y, colour);

   while (x < y)
   {
      if (f >= 0)
      {
         y--;
         ddF_y += 2;
         f += ddF_y;
      }
      x++;
      ddF_x += 2;
      f += ddF_x;

      // Hollow Circle - Draw 8 points of symmetry
      paintPixel(X + x, Y + y, colour);
      paintPixel(X - x, Y + y, colour);
      paintPixel(X + x, Y - y, colour);
      paintPixel(X - x, Y - y, colour);
      paintPixel(X + y, Y + x, colour);
      paintPixel(X - y, Y + x, colour);
      paintPixel(X + y, Y - x, colour);
      paintPixel(X - y, Y - x, colour);
   }
}

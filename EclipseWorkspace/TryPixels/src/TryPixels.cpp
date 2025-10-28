//============================================================================
// Name        : TryPixels.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <stdio.h>
#include <memory.h>
#include <stdint.h>

#include "FrameBuffer.h"

using namespace USBDM;

class Display : public USBDM::FrameBuffer {

public:

   static constexpr unsigned testHeight=50;
   static constexpr unsigned testWidth=64;

   bool useDelimiter = false;

   /// Frame buffer
   inline static uint8_t fb[testHeight*(testWidth+7)/8];

   Display() :
      FrameBuffer(testHeight, testWidth, &USBDM::fontMedium, fb){
   }

   ~Display() {
   }

   void clear(Colour colour) {
      memset(frameBuffer, colour, height*((width+7)/8));
   }

   void display() {
      printf("width=%d, height=%d, size=%d\n", width, height, height*((width+7)/8));
      for (unsigned y=0; y<height; y++) {
         printf("%4d %5d ", y/scale, y*((width+7)/8));
         for (unsigned x=0; x<((width+7)/8); x++) {
            for (unsigned bit=0; bit<8; bit++) {
               printf("%s", ((frameBuffer[x+y*((width+7)/8)]&(0b1000'0000>>bit))?"*":"."));
//               printf("%2X ", (1<<bit));
//               printf("%2X ", x+y*((width+7)/8));
            }
            if (useDelimiter) {
               printf("|");
            }
         }
         printf("\n");
      }
   }

   void setDelimiter(bool useDelimiter) {
      this->useDelimiter = useDelimiter;
   }

};

//Colour BackgroundColour = BLACK;
//Colour ForegroundColour = WHITE;
Colour BackgroundColour = WHITE;
Colour ForegroundColour = BLACK;

void test1(Display &display) {

   display.clear(BackgroundColour);
//   display.setScale(Scale_2);
   display.setMirror(MirrorMode_None);
   display.setRotate(Rotate_0);
   display.setWriteMode(WriteMode_Write);
   display.setColour(ForegroundColour);
   display.paintPixel(0, 0, ForegroundColour);
   display.paintPixel(1, 0, ForegroundColour);
   display.paintPixel(0, 1, ForegroundColour);
   display.paintPixel(0, 2, ForegroundColour);
   display.paintPixel(6, 5, ForegroundColour);
   display.paintPixel(6, 6, ForegroundColour);
   display.paintPixel(7, 6, ForegroundColour);
   display.paintPixel(7, 7, ForegroundColour);
   display.paintPixel(8, 7, ForegroundColour);
   display.display();
   display.setWriteMode(WriteMode_Xor);
   display.paintPixel(0,  0, BackgroundColour);
   display.paintPixel(2,  0, BackgroundColour);
   display.paintPixel(10, 0, BackgroundColour);
   display.display();
}

void test2() {
   Display display{};
//   display.setRotate(Rotate_270);
   display.setScale(Scale_2);
   display.setScale(Scale_4);
   display.setWriteMode(WriteMode_Write);

//   unsigned scale = 4;
//   for (unsigned i=0; i<=16; i++) {
//      uint8_t mask     = (0b1111'1111<<(scale*(i&0b1)));
//      uint8_t lastMask = (1<<(scale*((i&0b1)+1)))-1;
//      printf("%2d : 0x%2.2X, 0x%2.2X, 0x%2.2X\n", i, mask, lastMask, mask&lastMask);
//   }
   unsigned offset = 0;
   for (unsigned col=0; col<5; col++) {
      for (unsigned line=0; line<10; line++) {
         unsigned row = (line+offset);
         display.drawLine(line,row,line+col,row);
      }
      offset += (3+col);
   }
   display.display();
}

void test3() {
   Display display{};
//   display.setRotate(Rotate_270);
//   display.setScale(Scale_2);
//   display.setScale(Scale_4);
   display.setWriteMode(WriteMode_Write);
//   display.drawLine(0, 0, 0, 0);
//   display.drawLine(1, 0, 1, 1);
//   display.drawLine(2, 0, 2, 2);
//   display.drawLine(16, 7, 16, 7+6);
//   display.drawLine(14, 7, 14, 7+5);
//
   for (unsigned col=0; col<10; col++) {
      display.drawLine(2*col, col, 2*col, col+5);
   }
   display.display();
}

void test4() {
   Display display{};
//   display.setRotate(Rotate_270);
//   display.setScale(Scale_2);
//   display.setScale(Scale_4);
   display.setWriteMode(WriteMode_Write);

//   display.drawRect(1, 0, 5, 10);
//   display.drawRect(0, 1, 4, 11);

   display.setWriteMode(WriteMode_Write);
   display.drawRect(0, 0, 20, 20);
   display.setWriteMode(WriteMode_Xor);
   display.drawOpenRect(5, 5, 30, 30);

   display.display();
}

void testDrawHorizontalLine() {
   Display display{};
   display.setWriteMode(WriteMode_Write);
   display.setDelimiter(false);
   for (unsigned offset=0; offset<12; offset++) {
      display.drawLine(5, offset, 5+offset, offset);
   }
   for (unsigned offset=0; offset<12; offset++) {
      display.drawLine(5+offset, 14+offset, 5+11, 14+offset);
   }
   display.display();

}

void testDrawVerticalLine() {
   Display display{};
   display.setWriteMode(WriteMode_Write);
   display.setDelimiter(false);
   for (unsigned offset=0; offset<12; offset++) {
      display.drawLine(offset, 5, offset, 5+offset);
   }
   for (unsigned offset=0; offset<12; offset++) {
      display.drawLine(14+offset, offset, 14+offset, 11);
   }
   display.display();

}

int main() {
   testDrawHorizontalLine();
//   testDrawVerticalLine();
   return 0;
}

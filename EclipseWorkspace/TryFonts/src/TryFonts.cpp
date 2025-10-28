//============================================================================
// Name        : TryFonts.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <stdio.h>
#include <memory.h>
#include <stdint.h>

template<unsigned N>
struct ButtonImage {
   unsigned width;
   unsigned height;
   uint8_t data[N];
};

static constexpr ButtonImage<32> FastForward = {
      16,16,
      { 0x00, 0x00,
            0x00, 0x00,
            0x40, 0x80,
            0x60, 0xC0,
            0x70, 0xE0,
            0x78, 0xF0,
            0x7C, 0xF8,
            0x7E, 0xFC,
            0x7F, 0xFE,
            0x7E, 0xFC,
            0x7C, 0xF8,
            0x78, 0xF0,
            0x70, 0xE0,
            0x60, 0xC0,
            0x40, 0x80,
            0x00, 0x00, },
};

static constexpr ButtonImage<32> Block = {
      16,16,
      {
            0xFF, 0xFF,
            0xFF, 0xFF,
            0xFF, 0xFF,
            0xFF, 0xFF,

            0xFF, 0xFF,
            0xFF, 0xFF,
            0xFF, 0xFF,
            0xFF, 0xFF,

            0xFF, 0xFF,
            0xFF, 0xFF,
            0xFF, 0xFF,
            0xFF, 0xFF,

            0xFF, 0xFF,
            0xFF, 0xFF,
            0xFF, 0xFF,
            0xFF, 0xFF,
},
};

class ExpandedButtonImage : public ButtonImage<128> {

public:
   ExpandedButtonImage() {
   }

   void doExpansion(const ButtonImage<32> &original) {
      width  = 2*original.width;
      height = 2*original.height;

      unsigned from=0;
      for (unsigned row=0; row<original.height; row++) {

         // Each row of original -> 2 rows
         unsigned to = 2*row*((width+7)/8);
         for (unsigned col=0; col<(original.width+7)/8; col++) {

            uint8_t f = original.data[from++];
            printf("[0x%02X] -> 0x%02X\n", from-1, f);
            unsigned tBitMask, fBitMask;
            uint16_t t = 0;
            for (fBitMask=0x80, tBitMask=0xC000 ; fBitMask>0; fBitMask>>=1, tBitMask >>=2) {
               if (f&fBitMask) {
                  t |= tBitMask;
               }
            }
            data[to+(width+7)/8] = uint8_t(t>>8);
            data[to++]           = uint8_t(t>>8);
            data[to+(width+7)/8] = uint8_t(t);
            data[to++]           = uint8_t(t);
         }
      }
   }

};


class Display {

public:

   static constexpr unsigned testWidth=64;
   static constexpr unsigned testHeight=50;

   bool useDelimiter = false;

   /// Frame buffer
   inline static uint8_t fb[testHeight*(testWidth+7)/8];

   Display() {
   }

   ~Display() {
   }

   void clear() {
      memset(fb, 0, sizeof(fb));
   }

   void display() {
      printf("testWidth=%d, testHeight=%d, size=%d\n", testWidth, testHeight, testHeight*((testWidth+7)/8));
      for (unsigned y=0; y<testHeight; y++) {
         printf("%4d ", y );
         for (unsigned x=0; x<((testWidth+7)/8); x++) {
            for (unsigned bit=0; bit<8; bit++) {
               printf("%s", ((fb[x+y*((testWidth+7)/8)]&(0b1000'0000>>bit))?"*":"."));
               //               printf("%2X ", (1<<bit));
               //               printf("%2X ", x+y*((testWidth+7)/8));
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

   void drawImage(const uint8_t *image, unsigned w, unsigned h) {

      const uint8_t *img    = image;
      uint8_t *buffer;

      for (unsigned row=0; row<h; row++) {
         buffer = fb+row*((testWidth+7)/8);
         for (unsigned col=0; col<(w+7)/8; col++) {
            *buffer++ = *img++;
         }
      }
   }

   void drawImage(const ButtonImage<32> &image) {
      drawImage(image.data, image.width, image.height);
   }

   void drawImage(const ButtonImage<128> &image) {
      drawImage(image.data, image.width, image.height);
   }

};

int main() {
   Display dsp;
   dsp.setDelimiter(true);
   dsp.display();

   dsp.drawImage(FastForward);
   dsp.display();

   ExpandedButtonImage eImage{};
   eImage.doExpansion(FastForward);
   dsp.clear();
   dsp.drawImage(eImage);
   dsp.display();

   return 0;
}

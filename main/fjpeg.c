//------------------------------------------------------------------------------
// picojpeg.c v1.1 - Public domain, Rich Geldreich <richgel99@gmail.com>
// Nov. 27, 2010 - Initial release
// Feb. 9, 2013 - Added H1V2/H2V1 support, cleaned up macros, signed shift fixes
// Also integrated and tested changes from Chris Phoenix <cphoenix@gmail.com>.
//------------------------------------------------------------------------------
#include "fjpeg.h"
//------------------------------------------------------------------------------
// Set to 1 if right shifts on signed ints are always unsigned (logical) shifts
// When 1, arithmetic right shifts will be emulated by using a logical shift
// with special case code to ensure the sign bit is replicated.
#define FJPG_RIGHT_SHIFT_IS_ALWAYS_UNSIGNED 0

// Define FJPG_INLINE to "inline" if your C compiler supports explicit inlining
#define FJPG_INLINE
//------------------------------------------------------------------------------
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef signed char int8;
typedef signed short int16;
//------------------------------------------------------------------------------
#if FJPG_RIGHT_SHIFT_IS_ALWAYS_UNSIGNED
static int16 replicateSignBit16(int8 n)
{
      switch (n)
      {
      case 0:
            return 0x0000;
      case 1:
            return 0x8000;
      case 2:
            return 0xC000;
      case 3:
            return 0xE000;
      case 4:
            return 0xF000;
      case 5:
            return 0xF800;
      case 6:
            return 0xFC00;
      case 7:
            return 0xFE00;
      case 8:
            return 0xFF00;
      case 9:
            return 0xFF80;
      case 10:
            return 0xFFC0;
      case 11:
            return 0xFFE0;
      case 12:
            return 0xFFF0;
      case 13:
            return 0xFFF8;
      case 14:
            return 0xFFFC;
      case 15:
            return 0xFFFE;
      default:
            return 0xFFFF;
      }
}
static FJPG_INLINE int16 arithmeticRightShiftN16(int16 x, int8 n)
{
      int16 r = (uint16)x >> (uint8)n;
      if (x < 0)
            r |= replicateSignBit16(n);
      return r;
}
static FJPG_INLINE long arithmeticRightShift8L(long x)
{
      long r = (unsigned long)x >> 8U;
      if (x < 0)
            r |= ~(~(unsigned long)0U >> 8U);
      return r;
}
#define FJPG_ARITH_SHIFT_RIGHT_N_16(x, n) arithmeticRightShiftN16(x, n)
#define FJPG_ARITH_SHIFT_RIGHT_8_L(x) arithmeticRightShift8L(x)
#else
#define FJPG_ARITH_SHIFT_RIGHT_N_16(x, n) ((x) >> (n))
#define FJPG_ARITH_SHIFT_RIGHT_8_L(x) ((x) >> 8)
#endif
//------------------------------------------------------------------------------
// Change as needed - the FJPG_MAX_WIDTH/FJPG_MAX_HEIGHT checks are only present
// to quickly detect bogus files.
#define FJPG_MAX_WIDTH 16384
#define FJPG_MAX_HEIGHT 16384
#define FJPG_MAXCOMPSINSCAN 3
//------------------------------------------------------------------------------
typedef enum
{
      M_SOF0 = 0xC0,
      M_SOF1 = 0xC1,
      M_SOF2 = 0xC2,
      M_SOF3 = 0xC3,

      M_SOF5 = 0xC5,
      M_SOF6 = 0xC6,
      M_SOF7 = 0xC7,

      M_JPG = 0xC8,
      M_SOF9 = 0xC9,
      M_SOF10 = 0xCA,
      M_SOF11 = 0xCB,

      M_SOF13 = 0xCD,
      M_SOF14 = 0xCE,
      M_SOF15 = 0xCF,

      M_DHT = 0xC4,

      M_DAC = 0xCC,

      M_RST0 = 0xD0,
      M_RST1 = 0xD1,
      M_RST2 = 0xD2,
      M_RST3 = 0xD3,
      M_RST4 = 0xD4,
      M_RST5 = 0xD5,
      M_RST6 = 0xD6,
      M_RST7 = 0xD7,

      M_SOI = 0xD8,
      M_EOI = 0xD9,
      M_SOS = 0xDA,
      M_DQT = 0xDB,
      M_DNL = 0xDC,
      M_DRI = 0xDD,
      M_DHP = 0xDE,
      M_EXP = 0xDF,

      M_APP0 = 0xE0,
      M_APP15 = 0xEF,

      M_JPG0 = 0xF0,
      M_JPG13 = 0xFD,
      M_COM = 0xFE,

      M_TEM = 0x01,

      M_ERROR = 0x100,

      RST0 = 0xD0
} JPEG_MARKER;

//------------------------------------------------------------------------------
// 2 bytes
static int16 gCoeffBuf;

// 4 bytes * 3 = 12
static uint8 gMCUBufR[4];
#if FJPEG_OUTPUT_GRAY == 0
static uint8 gMCUBufG[4];
static uint8 gMCUBufB[4];
#endif

// 4 bytes
static int16 gQuant0[1];
static int16 gQuant1[1];

// 6 bytes
#if FJPEG_OUTPUT_GRAY == 1
static int16 gLastDC[1];
#else
static int16 gLastDC[3];
#endif
typedef struct HuffTableT
{
      uint16 mMinCode[16];
      uint16 mMaxCode[16];
      uint8 mValPtr[16];
} HuffTable;

// DC - 192
static HuffTable gHuffTab0;

static uint8 gHuffVal0[16];

static HuffTable gHuffTab1;
static uint8 gHuffVal1[16];

// AC - 672
static HuffTable gHuffTab2;
static uint8 gHuffVal2[256];

static HuffTable gHuffTab3;
static uint8 gHuffVal3[256];

static uint8 gValidHuffTables;
static uint8 gValidQuantTables;

static uint8 gTemFlag;
#define FJPG_MAX_IN_BUF_SIZE 256
static uint8 gInBuf[FJPG_MAX_IN_BUF_SIZE];
static uint8 gInBufOfs;
static uint8 gInBufLeft;

static uint16 gBitBuf;
static uint8 gBitsLeft;
//------------------------------------------------------------------------------
static uint16 gImageXSize;
static uint16 gImageYSize;
static uint8 gCompsInFrame;
static uint8 gCompIdent[3];
static uint8 gCompHSamp[3];
static uint8 gCompVSamp[3];
static uint8 gCompQuant[3];

static uint16 gRestartInterval;
static uint16 gNextRestartNum;
static uint16 gRestartsLeft;

static uint8 gCompsInScan;
static uint8 gCompList[3];
static uint8 gCompDCTab[3]; // 0,1
static uint8 gCompACTab[3]; // 0,1

static fjpeg_scan_type_t gScanType;

static uint8 gMaxBlocksPerMCU;
static uint8 gMaxMCUXSize;
static uint8 gMaxMCUYSize;
static uint16 gMaxMCUSPerRow;
static uint16 gMaxMCUSPerCol;
static uint16 gNumMCUSRemaining;
static uint8 gMCUOrg[6];

static fjpeg_need_bytes_callback_t g_pNeedBytesCallback;
static void *g_pCallback_data;
static uint8 gCallbackStatus;
//------------------------------------------------------------------------------
static void fillInBuf(void)
{
      unsigned char status;

      // Reserve a few bytes at the beginning of the buffer for putting back ("stuffing") chars.
      gInBufOfs = 4;
      gInBufLeft = 0;

      status = (*g_pNeedBytesCallback)(gInBuf + gInBufOfs, FJPG_MAX_IN_BUF_SIZE - gInBufOfs, &gInBufLeft, g_pCallback_data);
      if (status)
      {
            // The user provided need bytes callback has indicated an error, so record the error and continue trying to decode.
            // The highest level fjpeg entrypoints will catch the error and return the non-zero status.
            gCallbackStatus = status;
      }
}
//------------------------------------------------------------------------------
static FJPG_INLINE uint8 getChar(void)
{
      if (!gInBufLeft)
      {
            fillInBuf();
            if (!gInBufLeft)
            {
                  gTemFlag = ~gTemFlag;
                  return gTemFlag ? 0xFF : 0xD9;
            }
      }

      gInBufLeft--;
      return gInBuf[gInBufOfs++];
}
//------------------------------------------------------------------------------
static FJPG_INLINE void stuffChar(uint8 i)
{
      gInBufOfs--;
      gInBuf[gInBufOfs] = i;
      gInBufLeft++;
}
//------------------------------------------------------------------------------
static FJPG_INLINE uint8 getOctet(uint8 FFCheck)
{
      uint8 c = getChar();

      if ((FFCheck) && (c == 0xFF))
      {
            uint8 n = getChar();

            if (n)
            {
                  stuffChar(n);
                  stuffChar(0xFF);
            }
      }

      return c;
}
//------------------------------------------------------------------------------
static uint16 getBits(uint8 numBits, uint8 FFCheck)
{
      uint8 origBits = numBits;
      uint16 ret = gBitBuf;

      if (numBits > 8)
      {
            numBits -= 8;

            gBitBuf <<= gBitsLeft;

            gBitBuf |= getOctet(FFCheck);

            gBitBuf <<= (8 - gBitsLeft);

            ret = (ret & 0xFF00) | (gBitBuf >> 8);
      }

      if (gBitsLeft < numBits)
      {
            gBitBuf <<= gBitsLeft;

            gBitBuf |= getOctet(FFCheck);

            gBitBuf <<= (numBits - gBitsLeft);

            gBitsLeft = 8 - (numBits - gBitsLeft);
      }
      else
      {
            gBitsLeft = (uint8)(gBitsLeft - numBits);
            gBitBuf <<= numBits;
      }

      return ret >> (16 - origBits);
}
//------------------------------------------------------------------------------
static FJPG_INLINE uint16 getBits1(uint8 numBits)
{
      return getBits(numBits, 0);
}
//------------------------------------------------------------------------------
static FJPG_INLINE uint16 getBits2(uint8 numBits)
{
      return getBits(numBits, 1);
}
//------------------------------------------------------------------------------
static FJPG_INLINE uint8 getBit(void)
{
      uint8 ret = 0;
      if (gBitBuf & 0x8000)
            ret = 1;

      if (!gBitsLeft)
      {
            gBitBuf |= getOctet(1);

            gBitsLeft += 8;
      }

      gBitsLeft--;
      gBitBuf <<= 1;

      return ret;
}
//------------------------------------------------------------------------------
static uint16 getExtendTest(uint8 i)
{
      switch (i)
      {
      case 0:
            return 0;
      case 1:
            return 0x0001;
      case 2:
            return 0x0002;
      case 3:
            return 0x0004;
      case 4:
            return 0x0008;
      case 5:
            return 0x0010;
      case 6:
            return 0x0020;
      case 7:
            return 0x0040;
      case 8:
            return 0x0080;
      case 9:
            return 0x0100;
      case 10:
            return 0x0200;
      case 11:
            return 0x0400;
      case 12:
            return 0x0800;
      case 13:
            return 0x1000;
      case 14:
            return 0x2000;
      case 15:
            return 0x4000;
      default:
            return 0;
      }
}
//------------------------------------------------------------------------------
static int16 getExtendOffset(uint8 i)
{
      switch (i)
      {
      case 0:
            return 0;
      case 1:
            return ((-1) << 1) + 1;
      case 2:
            return ((-1) << 2) + 1;
      case 3:
            return ((-1) << 3) + 1;
      case 4:
            return ((-1) << 4) + 1;
      case 5:
            return ((-1) << 5) + 1;
      case 6:
            return ((-1) << 6) + 1;
      case 7:
            return ((-1) << 7) + 1;
      case 8:
            return ((-1) << 8) + 1;
      case 9:
            return ((-1) << 9) + 1;
      case 10:
            return ((-1) << 10) + 1;
      case 11:
            return ((-1) << 11) + 1;
      case 12:
            return ((-1) << 12) + 1;
      case 13:
            return ((-1) << 13) + 1;
      case 14:
            return ((-1) << 14) + 1;
      case 15:
            return ((-1) << 15) + 1;
      default:
            return 0;
      }
};
//------------------------------------------------------------------------------
static FJPG_INLINE int16 huffExtend(uint16 x, uint8 s)
{
      return ((x < getExtendTest(s)) ? ((int16)x + getExtendOffset(s)) : (int16)x);
}
//------------------------------------------------------------------------------
static FJPG_INLINE uint8 huffDecode(const HuffTable *pHuffTable, const uint8 *pHuffVal)
{
      uint8 i = 0;
      uint8 j;
      uint16 code = getBit();

      // This func only reads a bit at a time, which on modern CPU's is not terribly efficient.
      // But on microcontrollers without strong integer shifting support this seems like a
      // more reasonable approach.
      for (;;)
      {
            uint16 maxCode;

            if (i == 16)
                  return 0;

            maxCode = pHuffTable->mMaxCode[i];
            if ((code <= maxCode) && (maxCode != 0xFFFF))
                  break;

            i++;
            code <<= 1;
            code |= getBit();
      }

      j = pHuffTable->mValPtr[i];
      j = (uint8)(j + (code - pHuffTable->mMinCode[i]));

      return pHuffVal[j];
}
//------------------------------------------------------------------------------
static void huffCreate(const uint8 *pBits, HuffTable *pHuffTable)
{
      uint8 i = 0;
      uint8 j = 0;

      uint16 code = 0;

      for (;;)
      {
            uint8 num = pBits[i];

            if (!num)
            {
                  pHuffTable->mMinCode[i] = 0x0000;
                  pHuffTable->mMaxCode[i] = 0xFFFF;
                  pHuffTable->mValPtr[i] = 0;
            }
            else
            {
                  pHuffTable->mMinCode[i] = code;
                  pHuffTable->mMaxCode[i] = code + num - 1;
                  pHuffTable->mValPtr[i] = j;

                  j = (uint8)(j + num);

                  code = (uint16)(code + num);
            }

            code <<= 1;

            i++;
            if (i > 15)
                  break;
      }
}
//------------------------------------------------------------------------------
static HuffTable *getHuffTable(uint8 index)
{
      // 0-1 = DC
      // 2-3 = AC
      switch (index)
      {
      case 0:
            return &gHuffTab0;
      case 1:
            return &gHuffTab1;
      case 2:
            return &gHuffTab2;
      case 3:
            return &gHuffTab3;
      default:
            return 0;
      }
}
//------------------------------------------------------------------------------
static uint8 *getHuffVal(uint8 index)
{
      // 0-1 = DC
      // 2-3 = AC
      switch (index)
      {
      case 0:
            return gHuffVal0;
      case 1:
            return gHuffVal1;
      case 2:
            return gHuffVal2;
      case 3:
            return gHuffVal3;
      default:
            return 0;
      }
}
//------------------------------------------------------------------------------
static uint16 getMaxHuffCodes(uint8 index)
{
      return (index < 2) ? 12 : 255;
}
//------------------------------------------------------------------------------
static uint8 readDHTMarker(void)
{
      uint8 bits[16];
      uint16 left = getBits1(16);

      if (left < 2)
            return FJPG_BAD_DHT_MARKER;

      left -= 2;

      while (left)
      {
            uint8 i, tableIndex, index;
            uint8 *pHuffVal;
            HuffTable *pHuffTable;
            uint16 count, totalRead;

            index = (uint8)getBits1(8);

            if (((index & 0xF) > 1) || ((index & 0xF0) > 0x10))
                  return FJPG_BAD_DHT_INDEX;

            tableIndex = ((index >> 3) & 2) + (index & 1);

            pHuffTable = getHuffTable(tableIndex);
            pHuffVal = getHuffVal(tableIndex);

            gValidHuffTables |= (1 << tableIndex);

            count = 0;
            for (i = 0; i <= 15; i++)
            {
                  uint8 n = (uint8)getBits1(8);
                  bits[i] = n;
                  count = (uint16)(count + n);
            }

            if (count > getMaxHuffCodes(tableIndex))
                  return FJPG_BAD_DHT_COUNTS;

            for (i = 0; i < count; i++)
                  pHuffVal[i] = (uint8)getBits1(8);

            totalRead = 1 + 16 + count;

            if (left < totalRead)
                  return FJPG_BAD_DHT_MARKER;

            left = (uint16)(left - totalRead);

            huffCreate(bits, pHuffTable);
      }

      return 0;
}
//------------------------------------------------------------------------------
static void createWinogradQuant(int16 *pQuant);

static uint8 readDQTMarker(void)
{
      uint16 left = getBits1(16);

      if (left < 2)
            return FJPG_BAD_DQT_MARKER;

      left -= 2;

      while (left)
      {
            uint8 i;
            uint8 n = (uint8)getBits1(8);
            uint8 prec = n >> 4;
            uint16 totalRead;

            n &= 0x0F;

            if (n > 1)
                  return FJPG_BAD_DQT_TABLE;

            gValidQuantTables |= (n ? 2 : 1);

            // read quantization entries, in zag order need Only One
            for (i = 0; i < 64; i++)
            {
                  uint16 temp = getBits1(8);

                  if (prec)
                        temp = (temp << 8) + getBits1(8);
                  if (i != 0)
                        continue;
                  if (n)
                        gQuant1[i] = (int16)temp;
                  else
                        gQuant0[i] = (int16)temp;
            }

            createWinogradQuant(n ? gQuant1 : gQuant0);

            totalRead = 64 + 1;

            if (prec)
                  totalRead += 64;

            if (left < totalRead)
                  return FJPG_BAD_DQT_LENGTH;

            left = (uint16)(left - totalRead);
      }

      return 0;
}
//------------------------------------------------------------------------------
static uint8 readSOFMarker(void)
{
      uint8 i;
      uint16 left = getBits1(16);

      if (getBits1(8) != 8)
            return FJPG_BAD_PRECISION;

      gImageYSize = getBits1(16);

      if ((!gImageYSize) || (gImageYSize > FJPG_MAX_HEIGHT))
            return FJPG_BAD_HEIGHT;

      gImageXSize = getBits1(16);

      if ((!gImageXSize) || (gImageXSize > FJPG_MAX_WIDTH))
            return FJPG_BAD_WIDTH;

      gCompsInFrame = (uint8)getBits1(8);

      if (gCompsInFrame > 3)
            return FJPG_TOO_MANY_COMPONENTS;

      if (left != (gCompsInFrame + gCompsInFrame + gCompsInFrame + 8))
            return FJPG_BAD_SOF_LENGTH;

      for (i = 0; i < gCompsInFrame; i++)
      {
            gCompIdent[i] = (uint8)getBits1(8);
            gCompHSamp[i] = (uint8)getBits1(4);
            gCompVSamp[i] = (uint8)getBits1(4);
            gCompQuant[i] = (uint8)getBits1(8);

            if (gCompQuant[i] > 1)
                  return FJPG_UNSUPPORTED_QUANT_TABLE;
      }

      return 0;
}
//------------------------------------------------------------------------------
// Used to skip unrecognized markers.
static uint8 skipVariableMarker(void)
{
      uint16 left = getBits1(16);

      if (left < 2)
            return FJPG_BAD_VARIABLE_MARKER;

      left -= 2;

      while (left)
      {
            getBits1(8);
            left--;
      }

      return 0;
}
//------------------------------------------------------------------------------
// Read a define restart interval (DRI) marker.
static uint8 readDRIMarker(void)
{
      if (getBits1(16) != 4)
            return FJPG_BAD_DRI_LENGTH;

      gRestartInterval = getBits1(16);

      return 0;
}
//------------------------------------------------------------------------------
// Read a start of scan (SOS) marker.
static uint8 readSOSMarker(void)
{
      uint8 i;
      uint16 left = getBits1(16);
      uint8 spectral_start, spectral_end, successive_high, successive_low;

      gCompsInScan = (uint8)getBits1(8);

      left -= 3;

      if ((left != (gCompsInScan + gCompsInScan + 3)) || (gCompsInScan < 1) || (gCompsInScan > FJPG_MAXCOMPSINSCAN))
            return FJPG_BAD_SOS_LENGTH;

      for (i = 0; i < gCompsInScan; i++)
      {
            uint8 cc = (uint8)getBits1(8);
            uint8 c = (uint8)getBits1(8);
            uint8 ci;

            left -= 2;

            for (ci = 0; ci < gCompsInFrame; ci++)
                  if (cc == gCompIdent[ci])
                        break;

            if (ci >= gCompsInFrame)
                  return FJPG_BAD_SOS_COMP_ID;

            gCompList[i] = ci;
            gCompDCTab[ci] = (c >> 4) & 15;
            gCompACTab[ci] = (c & 15);
      }

      spectral_start = (uint8)getBits1(8);
      spectral_end = (uint8)getBits1(8);
      successive_high = (uint8)getBits1(4);
      successive_low = (uint8)getBits1(4);

      left -= 3;

      while (left)
      {
            getBits1(8);
            left--;
      }

      return 0;
}
//------------------------------------------------------------------------------
static uint8 nextMarker(void)
{
      uint8 c;
      uint8 bytes = 0;

      do
      {
            do
            {
                  bytes++;

                  c = (uint8)getBits1(8);

            } while (c != 0xFF);

            do
            {
                  c = (uint8)getBits1(8);

            } while (c == 0xFF);

      } while (c == 0);

      // If bytes > 0 here, there where extra bytes before the marker (not good).

      return c;
}
//------------------------------------------------------------------------------
// Process markers. Returns when an SOFx, SOI, EOI, or SOS marker is
// encountered.
static uint8 processMarkers(uint8 *pMarker)
{
      for (;;)
      {
            uint8 c = nextMarker();

            switch (c)
            {
            case M_SOF0:
            case M_SOF1:
            case M_SOF2:
            case M_SOF3:
            case M_SOF5:
            case M_SOF6:
            case M_SOF7:
            //      case M_JPG:
            case M_SOF9:
            case M_SOF10:
            case M_SOF11:
            case M_SOF13:
            case M_SOF14:
            case M_SOF15:
            case M_SOI:
            case M_EOI:
            case M_SOS:
            {
                  *pMarker = c;
                  return 0;
            }
            case M_DHT:
            {
                  readDHTMarker();
                  break;
            }
            // Sorry, no arithmetic support at this time. Dumb patents!
            case M_DAC:
            {
                  return FJPG_NO_ARITHMITIC_SUPPORT;
            }
            case M_DQT:
            {
                  readDQTMarker();
                  break;
            }
            case M_DRI:
            {
                  readDRIMarker();
                  break;
            }
                  //case M_APP0:  /* no need to read the JFIF marker */

            case M_JPG:
            case M_RST0: /* no parameters */
            case M_RST1:
            case M_RST2:
            case M_RST3:
            case M_RST4:
            case M_RST5:
            case M_RST6:
            case M_RST7:
            case M_TEM:
            {
                  return FJPG_UNEXPECTED_MARKER;
            }
            default: /* must be DNL, DHP, EXP, APPn, JPGn, COM, or RESn or APP0 */
            {
                  skipVariableMarker();
                  break;
            }
            }
      }
      //   return 0;
}
//------------------------------------------------------------------------------
// Finds the start of image (SOI) marker.
static uint8 locateSOIMarker(void)
{
      uint16 bytesleft;

      uint8 lastchar = (uint8)getBits1(8);

      uint8 thischar = (uint8)getBits1(8);

      /* ok if it's a normal JPEG file without a special header */

      if ((lastchar == 0xFF) && (thischar == M_SOI))
            return 0;

      bytesleft = 4096; //512;

      for (;;)
      {
            if (--bytesleft == 0)
                  return FJPG_NOT_JPEG;

            lastchar = thischar;

            thischar = (uint8)getBits1(8);

            if (lastchar == 0xFF)
            {
                  if (thischar == M_SOI)
                        break;
                  else if (thischar == M_EOI) //getBits1 will keep returning M_EOI if we read past the end
                        return FJPG_NOT_JPEG;
            }
      }

      /* Check the next character after marker: if it's not 0xFF, it can't
   be the start of the next marker, so the file is bad */

      thischar = (uint8)((gBitBuf >> 8) & 0xFF);

      if (thischar != 0xFF)
            return FJPG_NOT_JPEG;

      return 0;
}
//------------------------------------------------------------------------------
// Find a start of frame (SOF) marker.
static uint8 locateSOFMarker(void)
{
      uint8 c;

      uint8 status = locateSOIMarker();
      if (status)
            return status;

      status = processMarkers(&c);
      if (status)
            return status;

      switch (c)
      {
      case M_SOF2:
      {
            // Progressive JPEG - not supported by picojpeg (would require too
            // much memory, or too many IDCT's for embedded systems).
            return FJPG_UNSUPPORTED_MODE;
      }
      case M_SOF0: /* baseline DCT */
      {
            status = readSOFMarker();
            if (status)
                  return status;

            break;
      }
      case M_SOF9:
      {
            return FJPG_NO_ARITHMITIC_SUPPORT;
      }
      case M_SOF1: /* extended sequential DCT */
      default:
      {
            return FJPG_UNSUPPORTED_MARKER;
      }
      }

      return 0;
}
//------------------------------------------------------------------------------
// Find a start of scan (SOS) marker.
static uint8 locateSOSMarker(uint8 *pFoundEOI)
{
      uint8 c;
      uint8 status;

      *pFoundEOI = 0;

      status = processMarkers(&c);
      if (status)
            return status;

      if (c == M_EOI)
      {
            *pFoundEOI = 1;
            return 0;
      }
      else if (c != M_SOS)
            return FJPG_UNEXPECTED_MARKER;

      return readSOSMarker();
}
//------------------------------------------------------------------------------
static uint8 init(void)
{
      gImageXSize = 0;
      gImageYSize = 0;
      gCompsInFrame = 0;
      gRestartInterval = 0;
      gCompsInScan = 0;
      gValidHuffTables = 0;
      gValidQuantTables = 0;
      gTemFlag = 0;
      gInBufOfs = 0;
      gInBufLeft = 0;
      gBitBuf = 0;
      gBitsLeft = 8;

      getBits1(8);
      getBits1(8);

      return 0;
}
//------------------------------------------------------------------------------
// This method throws back into the stream any bytes that where read
// into the bit buffer during initial marker scanning.
static void fixInBuffer(void)
{
      /* In case any 0xFF's where pulled into the buffer during marker scanning */

      if (gBitsLeft > 0)
            stuffChar((uint8)gBitBuf);

      stuffChar((uint8)(gBitBuf >> 8));

      gBitsLeft = 8;
      getBits2(8);
      getBits2(8);
}
//------------------------------------------------------------------------------
// Restart interval processing.
static uint8 processRestart(void)
{
      // Let's scan a little bit to find the marker, but not _too_ far.
      // 1536 is a "fudge factor" that determines how much to scan.
      uint16 i;
      uint8 c = 0;

      for (i = 1536; i > 0; i--)
            if (getChar() == 0xFF)
                  break;

      if (i == 0)
            return FJPG_BAD_RESTART_MARKER;

      for (; i > 0; i--)
            if ((c = getChar()) != 0xFF)
                  break;

      if (i == 0)
            return FJPG_BAD_RESTART_MARKER;

      // Is it the expected marker? If not, something bad happened.
      if (c != (gNextRestartNum + M_RST0))
            return FJPG_BAD_RESTART_MARKER;

      // Reset each component's DC prediction values.
      gLastDC[0] = 0;
#if FJPEG_OUTPUT_GRAY == 0
      gLastDC[1] = 0;
      gLastDC[2] = 0;
#endif

      gRestartsLeft = gRestartInterval;

      gNextRestartNum = (gNextRestartNum + 1) & 7;

      // Get the bit buffer going again

      gBitsLeft = 8;
      getBits2(8);
      getBits2(8);

      return 0;
}

//------------------------------------------------------------------------------
static uint8 checkHuffTables(void)
{
      uint8 i;

      for (i = 0; i < gCompsInScan; i++)
      {
            uint8 compDCTab = gCompDCTab[gCompList[i]];
            uint8 compACTab = gCompACTab[gCompList[i]] + 2;

            if (((gValidHuffTables & (1 << compDCTab)) == 0) ||
                ((gValidHuffTables & (1 << compACTab)) == 0))
                  return FJPG_UNDEFINED_HUFF_TABLE;
      }

      return 0;
}
//------------------------------------------------------------------------------
static uint8 checkQuantTables(void)
{
      uint8 i;

      for (i = 0; i < gCompsInScan; i++)
      {
            uint8 compQuantMask = gCompQuant[gCompList[i]] ? 2 : 1;

            if ((gValidQuantTables & compQuantMask) == 0)
                  return FJPG_UNDEFINED_QUANT_TABLE;
      }

      return 0;
}
//------------------------------------------------------------------------------
static uint8 initScan(void)
{
      uint8 foundEOI;
      uint8 status = locateSOSMarker(&foundEOI);
      if (status)
            return status;
      if (foundEOI)
            return FJPG_UNEXPECTED_MARKER;

      status = checkHuffTables();
      if (status)
            return status;

      status = checkQuantTables();
      if (status)
            return status;

      gLastDC[0] = 0;
#if FJPEG_OUTPUT_GRAY == 0
      gLastDC[1] = 0;
      gLastDC[2] = 0;
#endif

      if (gRestartInterval)
      {
            gRestartsLeft = gRestartInterval;
            gNextRestartNum = 0;
      }

      fixInBuffer();

      return 0;
}
//------------------------------------------------------------------------------
static uint8 initFrame(void)
{
      if (gCompsInFrame == 1)
      {
            if ((gCompHSamp[0] != 1) || (gCompVSamp[0] != 1))
                  return FJPG_UNSUPPORTED_SAMP_FACTORS;

            gScanType = FJPG_GRAYSCALE;

            gMaxBlocksPerMCU = 1;
            gMCUOrg[0] = 0;

            gMaxMCUXSize = 8;
            gMaxMCUYSize = 8;
      }
      else if (gCompsInFrame == 3)
      {
            if (((gCompHSamp[1] != 1) || (gCompVSamp[1] != 1)) ||
                ((gCompHSamp[2] != 1) || (gCompVSamp[2] != 1)))
                  return FJPG_UNSUPPORTED_SAMP_FACTORS;

            if ((gCompHSamp[0] == 1) && (gCompVSamp[0] == 1))
            {
                  gScanType = FJPG_YH1V1;

                  gMaxBlocksPerMCU = 3;
                  gMCUOrg[0] = 0;
                  gMCUOrg[1] = 1;
                  gMCUOrg[2] = 2;

                  gMaxMCUXSize = 8;
                  gMaxMCUYSize = 8;
            }
            else if ((gCompHSamp[0] == 1) && (gCompVSamp[0] == 2))
            {
                  gScanType = FJPG_YH1V2;

                  gMaxBlocksPerMCU = 4;
                  gMCUOrg[0] = 0;
                  gMCUOrg[1] = 0;
                  gMCUOrg[2] = 1;
                  gMCUOrg[3] = 2;

                  gMaxMCUXSize = 8;
                  gMaxMCUYSize = 16;
            }
            else if ((gCompHSamp[0] == 2) && (gCompVSamp[0] == 1))
            {
                  gScanType = FJPG_YH2V1;

                  gMaxBlocksPerMCU = 4;
                  gMCUOrg[0] = 0;
                  gMCUOrg[1] = 0;
                  gMCUOrg[2] = 1;
                  gMCUOrg[3] = 2;

                  gMaxMCUXSize = 16;
                  gMaxMCUYSize = 8;
            }
            else if ((gCompHSamp[0] == 2) && (gCompVSamp[0] == 2))
            {
                  gScanType = FJPG_YH2V2;

                  gMaxBlocksPerMCU = 6;
                  gMCUOrg[0] = 0;
                  gMCUOrg[1] = 0;
                  gMCUOrg[2] = 0;
                  gMCUOrg[3] = 0;
                  gMCUOrg[4] = 1;
                  gMCUOrg[5] = 2;

                  gMaxMCUXSize = 16;
                  gMaxMCUYSize = 16;
            }
            else
                  return FJPG_UNSUPPORTED_SAMP_FACTORS;
      }
      else
            return FJPG_UNSUPPORTED_COLORSPACE;

      gMaxMCUSPerRow = (gImageXSize + (gMaxMCUXSize - 1)) >> ((gMaxMCUXSize == 8) ? 3 : 4);
      gMaxMCUSPerCol = (gImageYSize + (gMaxMCUYSize - 1)) >> ((gMaxMCUYSize == 8) ? 3 : 4);

      gNumMCUSRemaining = gMaxMCUSPerRow * gMaxMCUSPerCol;

      return 0;
}
//----------------------------------------------------------------------------
// Winograd IDCT: 5 multiplies per row/col, up to 80 muls for the 2D IDCT

#define FJPG_DCT_SCALE_BITS 7

#define FJPG_DCT_SCALE (1U << FJPG_DCT_SCALE_BITS)

#define FJPG_DESCALE(x) FJPG_ARITH_SHIFT_RIGHT_N_16(((x) + (1U << (FJPG_DCT_SCALE_BITS - 1))), FJPG_DCT_SCALE_BITS)

#define FJPG_WFIX(x) ((x)*FJPG_DCT_SCALE + 0.5f)

#define FJPG_WINOGRAD_QUANT_SCALE_BITS 10
const uint8 gWinogradQuant[] = 
{
   128,  178,  178,  167,  246,  167,  151,  232,
   232,  151,  128,  209,  219,  209,  128,  101,
   178,  197,  197,  178,  101,   69,  139,  167,
   177,  167,  139,   69,   35,   96,  131,  151,
   151,  131,   96,   35,   49,   91,  118,  128,
   118,   91,   49,   46,   81,  101,  101,   81,
   46,   42,   69,   79,   69,   42,   35,   54,
   54,   35,   28,   37,   28,   19,   19,   10,
};   
// Multiply quantization matrix by the Winograd IDCT scale factors
static void createWinogradQuant(int16 *pQuant)
{
      uint8 i;
      for (i = 0; i < 64; i++)
      {
            long x = pQuant[i];
            x *= gWinogradQuant[i];
            pQuant[i] = (int16)((x + (1 << (FJPG_WINOGRAD_QUANT_SCALE_BITS - FJPG_DCT_SCALE_BITS - 1))) >> (FJPG_WINOGRAD_QUANT_SCALE_BITS - FJPG_DCT_SCALE_BITS));
      }
}

// These multiply helper functions are the 4 types of signed multiplies needed by the Winograd IDCT.
// A smart C compiler will optimize them to use 16x8 = 24 bit muls, if not you may need to tweak
// these functions or drop to CPU specific inline assembly.

static FJPG_INLINE uint8 clamp(int16 s)
{
      if ((uint16)s > 255U)
      {
            if (s < 0)
                  return 0;
            else if (s > 255)
                  return 255;
      }

      return (uint8)s;
}
#if FJPEG_OUTPUT_GRAY==0
/*----------------------------------------------------------------------------*/
static FJPG_INLINE uint8 addAndClamp(uint8 a, int16 b)
{
      b = a + b;

      if ((uint16)b > 255U)
      {
            if (b < 0)
                  return 0;
            else if (b > 255)
                  return 255;
      }

      return (uint8)b;
}
/*----------------------------------------------------------------------------*/
static FJPG_INLINE uint8 subAndClamp(uint8 a, int16 b)
{
      b = a - b;

      if ((uint16)b > 255U)
      {
            if (b < 0)
                  return 0;
            else if (b > 255)
                  return 255;
      }

      return (uint8)b;
}
#endif
//------------------------------------------------------------------------------
static void transformBlockReduce(uint8 mcuBlock)
{
      uint8 c = clamp(FJPG_DESCALE(gCoeffBuf) + 128);
#if FJPEG_OUTPUT_GRAY==0
      int16 cbG, cbB, crR, crG;
#endif
      switch (gScanType)
      {
      case FJPG_GRAYSCALE:
      {
            // MCU size: 1, 1 block per MCU
            gMCUBufR[0] = c;
            break;
      }
      case FJPG_YH1V1:
      {
            // MCU size: 8x8, 3 blocks per MCU
            switch (mcuBlock)
            {
            case 0:
            {
                  gMCUBufR[0] = c;
#if FJPEG_OUTPUT_GRAY == 0
                  gMCUBufG[0] = c;
                  gMCUBufB[0] = c;
#endif
                  break;
            }
#if FJPEG_OUTPUT_GRAY == 0
            case 1:
            {
                  cbG = ((c * 88U) >> 8U) - 44U;
                  gMCUBufG[0] = subAndClamp(gMCUBufG[0], cbG);

                  cbB = (c + ((c * 198U) >> 8U)) - 227U;
                  gMCUBufB[0] = addAndClamp(gMCUBufB[0], cbB);
                  break;
            }
            case 2:
            {
                  crR = (c + ((c * 103U) >> 8U)) - 179;
                  gMCUBufR[0] = addAndClamp(gMCUBufR[0], crR);

                  crG = ((c * 183U) >> 8U) - 91;
                  gMCUBufG[0] = subAndClamp(gMCUBufG[0], crG);
                  break;
            }
#endif
            default:
                  break;
            }

            break;
      }
      case FJPG_YH1V2:
      {
            // MCU size: 8x16, 4 blocks per MCU
            switch (mcuBlock)
            {
            case 0:
            {
                  gMCUBufR[0] = c;
#if FJPEG_OUTPUT_GRAY == 0
                  gMCUBufG[0] = c;
                  gMCUBufB[0] = c;
#endif
                  break;
            }
            case 1:
            {
                  gMCUBufR[2] = c;
#if FJPEG_OUTPUT_GRAY == 0
                  gMCUBufG[2] = c;
                  gMCUBufB[2] = c;
#endif
                  break;
            }
#if FJPEG_OUTPUT_GRAY == 0
            case 2:
            {
                  cbG = ((c * 88U) >> 8U) - 44U;
                  gMCUBufG[0] = subAndClamp(gMCUBufG[0], cbG);
                  gMCUBufG[2] = subAndClamp(gMCUBufG[2], cbG);

                  cbB = (c + ((c * 198U) >> 8U)) - 227U;
                  gMCUBufB[0] = addAndClamp(gMCUBufB[0], cbB);
                  gMCUBufB[2] = addAndClamp(gMCUBufB[2], cbB);

                  break;
            }
            case 3:
            {
                  crR = (c + ((c * 103U) >> 8U)) - 179;
                  gMCUBufR[0] = addAndClamp(gMCUBufR[0], crR);
                  gMCUBufR[2] = addAndClamp(gMCUBufR[2], crR);

                  crG = ((c * 183U) >> 8U) - 91;
                  gMCUBufG[0] = subAndClamp(gMCUBufG[0], crG);
                  gMCUBufG[2] = subAndClamp(gMCUBufG[2], crG);

                  break;
            }
#endif
            default:
                  break;
            }
            break;
      }
      case FJPG_YH2V1:
      {
            // MCU size: 16x8, 4 blocks per MCU
            switch (mcuBlock)
            {
            case 0:
            {
                  gMCUBufR[0] = c;
#if FJPEG_OUTPUT_GRAY == 0
                  gMCUBufG[0] = c;
                  gMCUBufB[0] = c;
#endif
                  break;
            }
            case 1:
            {
                  gMCUBufR[1] = c;
#if FJPEG_OUTPUT_GRAY == 0
                  gMCUBufG[1] = c;
                  gMCUBufB[1] = c;
#endif
                  break;
            }
#if FJPEG_OUTPUT_GRAY == 0
            case 2:
            {
                  cbG = ((c * 88U) >> 8U) - 44U;
                  gMCUBufG[0] = subAndClamp(gMCUBufG[0], cbG);
                  gMCUBufG[1] = subAndClamp(gMCUBufG[1], cbG);

                  cbB = (c + ((c * 198U) >> 8U)) - 227U;
                  gMCUBufB[0] = addAndClamp(gMCUBufB[0], cbB);
                  gMCUBufB[1] = addAndClamp(gMCUBufB[1], cbB);

                  break;
            }
            case 3:
            {
                  crR = (c + ((c * 103U) >> 8U)) - 179;
                  gMCUBufR[0] = addAndClamp(gMCUBufR[0], crR);
                  gMCUBufR[1] = addAndClamp(gMCUBufR[1], crR);

                  crG = ((c * 183U) >> 8U) - 91;
                  gMCUBufG[0] = subAndClamp(gMCUBufG[0], crG);
                  gMCUBufG[1] = subAndClamp(gMCUBufG[1], crG);

                  break;
            }
#endif
            default:
                  break;
            }
            break;
      }
      case FJPG_YH2V2:
      {
            // MCU size: 16x16, 6 blocks per MCU
            switch (mcuBlock)
            {
            case 0:
            {
                  gMCUBufR[0] = c;
#if FJPEG_OUTPUT_GRAY == 0
                  gMCUBufG[0] = c;
                  gMCUBufB[0] = c;
#endif
                  break;
            }
            case 1:
            {
                  gMCUBufR[1] = c;
#if FJPEG_OUTPUT_GRAY == 0
                  gMCUBufG[1] = c;
                  gMCUBufB[1] = c;
#endif
                  break;
            }
            case 2:
            {
                  gMCUBufR[2] = c;
#if FJPEG_OUTPUT_GRAY == 0
                  gMCUBufG[2] = c;
                  gMCUBufB[2] = c;
#endif
                  break;
            }
            case 3:
            {
                  gMCUBufR[3] = c;
#if FJPEG_OUTPUT_GRAY == 0
                  gMCUBufG[3] = c;
                  gMCUBufB[3] = c;
#endif
                  break;
            }
#if FJPEG_OUTPUT_GRAY == 0
            case 4:
            {
                  cbG = ((c * 88U) >> 8U) - 44U;
                  gMCUBufG[0] = subAndClamp(gMCUBufG[0], cbG);
                  gMCUBufG[1] = subAndClamp(gMCUBufG[1], cbG);
                  gMCUBufG[2] = subAndClamp(gMCUBufG[2], cbG);
                  gMCUBufG[3] = subAndClamp(gMCUBufG[3], cbG);

                  cbB = (c + ((c * 198U) >> 8U)) - 227U;
                  gMCUBufB[0] = addAndClamp(gMCUBufB[0], cbB);
                  gMCUBufB[1] = addAndClamp(gMCUBufB[1], cbB);
                  gMCUBufB[2] = addAndClamp(gMCUBufB[2], cbB);
                  gMCUBufB[3] = addAndClamp(gMCUBufB[3], cbB);

                  break;
            }
            case 5:
            {
                  crR = (c + ((c * 103U) >> 8U)) - 179;
                  gMCUBufR[0] = addAndClamp(gMCUBufR[0], crR);
                  gMCUBufR[1] = addAndClamp(gMCUBufR[1], crR);
                  gMCUBufR[2] = addAndClamp(gMCUBufR[2], crR);
                  gMCUBufR[3] = addAndClamp(gMCUBufR[3], crR);

                  crG = ((c * 183U) >> 8U) - 91;
                  gMCUBufG[0] = subAndClamp(gMCUBufG[0], crG);
                  gMCUBufG[1] = subAndClamp(gMCUBufG[1], crG);
                  gMCUBufG[2] = subAndClamp(gMCUBufG[2], crG);
                  gMCUBufG[3] = subAndClamp(gMCUBufG[3], crG);

                  break;
            }
#endif
            default:
                  break;
            }
            break;
      }
      }
}
//------------------------------------------------------------------------------
static uint8 decodeNextMCU(void)
{
      uint8 status;
      uint8 mcuBlock;

      if (gRestartInterval)
      {
            if (gRestartsLeft == 0)
            {
                  status = processRestart();
                  if (status)
                        return status;
            }
            gRestartsLeft--;
      }

      for (mcuBlock = 0; mcuBlock < gMaxBlocksPerMCU; mcuBlock++)
      {
            uint8 componentID = gMCUOrg[mcuBlock];
            uint8 compQuant = gCompQuant[componentID];
            uint8 compDCTab = gCompDCTab[componentID];
            uint8 numExtraBits, compACTab, k;
            const int16 *pQ = compQuant ? gQuant1 : gQuant0;
            uint16 r, dc;

            uint8 s = huffDecode(compDCTab ? &gHuffTab1 : &gHuffTab0, compDCTab ? gHuffVal1 : gHuffVal0);

            r = 0;
            numExtraBits = s & 0xF;
            if (numExtraBits)
                  r = getBits2(numExtraBits);
            dc = huffExtend(r, s);

#if FJPEG_OUTPUT_GRAY != 0
            if (componentID == 0)
#endif
            {
                  dc = dc + gLastDC[componentID];
                  gLastDC[componentID] = dc;
                  gCoeffBuf = dc * pQ[0];
                  transformBlockReduce(mcuBlock);
            }
            compACTab = gCompACTab[componentID];
            // Decode, but throw out the AC coefficients in reduce mode.
            for (k = 1; k < 64; k++)
            {
                  s = huffDecode(compACTab ? &gHuffTab3 : &gHuffTab2, compACTab ? gHuffVal3 : gHuffVal2);

                  numExtraBits = s & 0xF;
                  if (numExtraBits)
                        getBits2(numExtraBits);

                  r = s >> 4;
                  s &= 15;

                  if (s)
                  {
                        if (r)
                        {
                              if ((k + r) > 63)
                                    return FJPG_DECODE_ERROR;

                              k = (uint8)(k + r);
                        }
                  }
                  else
                  {
                        if (r == 15)
                        {
                              if ((k + 16) > 64)
                                    return FJPG_DECODE_ERROR;

                              k += (16 - 1); // - 1 because the loop counter is k
                        }
                        else
                              break;
                  }
            }
      
      }

      return 0;
}
//------------------------------------------------------------------------------
unsigned char fjpeg_decode_mcu(void)
{
      uint8 status;

      if (gCallbackStatus)
            return gCallbackStatus;

      if (!gNumMCUSRemaining)
            return FJPG_NO_MORE_BLOCKS;

      status = decodeNextMCU();
      if ((status) || (gCallbackStatus))
            return gCallbackStatus ? gCallbackStatus : status;

      gNumMCUSRemaining--;

      return 0;
}
//------------------------------------------------------------------------------
unsigned char fjpeg_decode_init(fjpeg_image_info_t *pInfo, fjpeg_need_bytes_callback_t pNeed_bytes_callback, void *pCallback_data)
{
      uint8 status;

      pInfo->m_width = 0;
      pInfo->m_height = 0;
      pInfo->m_comps = 0;
      pInfo->m_MCUSPerRow = 0;
      pInfo->m_MCUSPerCol = 0;
      pInfo->m_scanType = FJPG_GRAYSCALE;
      pInfo->m_MCUWidth = 0;
      pInfo->m_MCUHeight = 0;
      pInfo->m_pMCUBufR = (unsigned char *)0;
      pInfo->m_pMCUBufG = (unsigned char *)0;
      pInfo->m_pMCUBufB = (unsigned char *)0;

      g_pNeedBytesCallback = pNeed_bytes_callback;
      g_pCallback_data = pCallback_data;
      gCallbackStatus = 0;

      status = init();
      if ((status) || (gCallbackStatus))
            return gCallbackStatus ? gCallbackStatus : status;

      status = locateSOFMarker();
      if ((status) || (gCallbackStatus))
            return gCallbackStatus ? gCallbackStatus : status;

      status = initFrame();
      if ((status) || (gCallbackStatus))
            return gCallbackStatus ? gCallbackStatus : status;

      status = initScan();
      if ((status) || (gCallbackStatus))
            return gCallbackStatus ? gCallbackStatus : status;

      pInfo->m_width = gImageXSize;
      pInfo->m_height = gImageYSize;
      pInfo->m_comps = gCompsInFrame;
      pInfo->m_scanType = gScanType;
      pInfo->m_MCUSPerRow = gMaxMCUSPerRow;
      pInfo->m_MCUSPerCol = gMaxMCUSPerCol;
      pInfo->m_MCUWidth = gMaxMCUXSize;
      pInfo->m_MCUHeight = gMaxMCUYSize;
#if FJPEG_OUTPUT_GRAY != 0
      pInfo->m_pMCUBufR = gMCUBufR;
      pInfo->m_pMCUBufG = gMCUBufR;
      pInfo->m_pMCUBufB = gMCUBufR;
#else
      pInfo->m_pMCUBufR = gMCUBufR;
      pInfo->m_pMCUBufG = gMCUBufG;
      pInfo->m_pMCUBufB = gMCUBufB;
#endif
      return 0;
}

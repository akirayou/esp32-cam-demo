//------------------------------------------------------------------------------
// picojpeg - Public domain, Rich Geldreich <richgel99@gmail.com>
// fjpeg - modified only for reduce mode. to reduce memory usage. <you.akira.noda@gmail.com>
//------------------------------------------------------------------------------
#ifndef FJPEG_H
#define FJPEG_H
#include "fjpeg_config.h"
#ifdef __cplusplus
extern "C"
{
#endif

    // Error codes
    enum
    {
        FJPG_NO_MORE_BLOCKS = 1,
        FJPG_BAD_DHT_COUNTS,
        FJPG_BAD_DHT_INDEX,
        FJPG_BAD_DHT_MARKER,
        FJPG_BAD_DQT_MARKER,
        FJPG_BAD_DQT_TABLE,
        FJPG_BAD_PRECISION,
        FJPG_BAD_HEIGHT,
        FJPG_BAD_WIDTH,
        FJPG_TOO_MANY_COMPONENTS,
        FJPG_BAD_SOF_LENGTH,
        FJPG_BAD_VARIABLE_MARKER,
        FJPG_BAD_DRI_LENGTH,
        FJPG_BAD_SOS_LENGTH,
        FJPG_BAD_SOS_COMP_ID,
        FJPG_W_EXTRA_BYTES_BEFORE_MARKER,
        FJPG_NO_ARITHMITIC_SUPPORT,
        FJPG_UNEXPECTED_MARKER,
        FJPG_NOT_JPEG,
        FJPG_UNSUPPORTED_MARKER,
        FJPG_BAD_DQT_LENGTH,
        FJPG_TOO_MANY_BLOCKS,
        FJPG_UNDEFINED_QUANT_TABLE,
        FJPG_UNDEFINED_HUFF_TABLE,
        FJPG_NOT_SINGLE_SCAN,
        FJPG_UNSUPPORTED_COLORSPACE,
        FJPG_UNSUPPORTED_SAMP_FACTORS,
        FJPG_DECODE_ERROR,
        FJPG_BAD_RESTART_MARKER,
        FJPG_ASSERTION_ERROR,
        FJPG_BAD_SOS_SPECTRAL,
        FJPG_BAD_SOS_SUCCESSIVE,
        FJPG_STREAM_READ_ERROR,
        FJPG_NOTENOUGHMEM,
        FJPG_UNSUPPORTED_COMP_IDENT,
        FJPG_UNSUPPORTED_QUANT_TABLE,
        FJPG_UNSUPPORTED_MODE, // picojpeg doesn't support progressive JPEG's
    };

    // Scan types
    typedef enum
    {
        FJPG_GRAYSCALE,
        FJPG_YH1V1,
        FJPG_YH2V1,
        FJPG_YH1V2,
        FJPG_YH2V2
    } fjpeg_scan_type_t;

    typedef struct
    {
        // Image resolution
        int m_width;
        int m_height;

        // Number of components (1 or 3)
        int m_comps;

        // Total number of minimum coded units (MCU's) per row/col.
        int m_MCUSPerRow;
        int m_MCUSPerCol;

        // Scan type
        fjpeg_scan_type_t m_scanType;

        // MCU width/height in pixels (each is either 8 or 16 depending on the scan type)
        int m_MCUWidth;
        int m_MCUHeight;

        // m_pMCUBufR, m_pMCUBufG, and m_pMCUBufB are pointers to internal MCU Y or RGB pixel component buffers.
        // Each time pjpegDecodeMCU() is called successfully these buffers will be filled with 8x8 pixel blocks of Y or RGB pixels.
        // Each MCU consists of (m_MCUWidth/8)*(m_MCUHeight/8) Y/RGB blocks: 1 for greyscale/no subsampling, 2 for H1V2/H2V1, or 4 blocks for H2V2 sampling factors.
        // Each block is a contiguous array of 64 (8x8) bytes of a single component: either Y for grayscale images, or R, G or B components for color images.
        //
        // The 8x8 pixel blocks are organized in these byte arrays like this:
        //
        // FJPG_GRAYSCALE: Each MCU is decoded to a single block of 8x8 grayscale pixels.
        // Only the values in m_pMCUBufR are valid. Each 8 bytes is a row of pixels (raster order: left to right, top to bottom) from the 8x8 block.
        //
        // FJPG_H1V1: Each MCU contains is decoded to a single block of 8x8 RGB pixels.
        //
        // FJPG_YH2V1: Each MCU is decoded to 2 blocks, or 16x8 pixels.
        // The 2 RGB blocks are at byte offsets: 0, 64
        //
        // FJPG_YH1V2: Each MCU is decoded to 2 blocks, or 8x16 pixels.
        // The 2 RGB blocks are at byte offsets: 0,
        //                                       128
        //
        // FJPG_YH2V2: Each MCU is decoded to 4 blocks, or 16x16 pixels.
        // The 2x2 block array is organized at byte offsets:   0,  64,
        //                                                   128, 192
        //
        // It is up to the caller to copy or blit these pixels from these buffers into the destination bitmap.
        unsigned char *m_pMCUBufR;
        unsigned char *m_pMCUBufG;
        unsigned char *m_pMCUBufB;
    } fjpeg_image_info_t;

    typedef unsigned char (*fjpeg_need_bytes_callback_t)(unsigned char *pBuf, unsigned char buf_size, unsigned char *pBytes_actually_read, void *pCallback_data);

    unsigned char fjpeg_decode_init(fjpeg_image_info_t *pInfo, fjpeg_need_bytes_callback_t pNeed_bytes_callback, void *pCallback_data);

    // Decompresses the file's next MCU. Returns 0 on success, FJPG_NO_MORE_BLOCKS if no more blocks are available, or an error code.
    // Must be called a total of m_MCUSPerRow*m_MCUSPerCol times to completely decompress the image.
    // Not thread safe.
    unsigned char fjpeg_decode_mcu(void);

#ifdef __cplusplus
}
#endif

#endif // PICOJPEG_H

#include "fjpeg.h"
#include <string.h>
#include <assert.h>
#include <math.h>
#include "move_detect.h"
#ifndef max
#define max(a, b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

static unsigned char *g_jpgData = 0;
static unsigned int g_nInFileSize;
static unsigned int g_nInFileOfs;

unsigned char pjpeg_need_bytes_callback(unsigned char *pBuf, unsigned char buf_size, unsigned char *pBytes_actually_read, void *pCallback_data)
{
    unsigned int n;
    n = min(g_nInFileSize - g_nInFileOfs, buf_size);
    memcpy(pBuf, g_jpgData + g_nInFileOfs, n);
    *pBytes_actually_read = (unsigned char)(n);
    g_nInFileOfs += n;
    return 0;
}

static int decodeToImage(unsigned char *pImage)
{
    fjpeg_image_info_t image_info;
    unsigned char status;
    status = fjpeg_decode_init(&image_info, pjpeg_need_bytes_callback, 0);
    if (status)
    {
        //printf("pjpeg_decode_init() failed with status %u\n", status);
        if (status == FJPG_UNSUPPORTED_MODE)
        {
            //printf("Progressive JPEG files are not supported.\n");
        }
        return -1;
    }
    assert(F_WIDTH == image_info.m_MCUSPerRow * image_info.m_MCUWidth / 8);
    assert(F_HEIGHT == image_info.m_MCUSPerCol * image_info.m_MCUHeight / 8);
    const int nofCh = 1;
    const int row_pitch = F_WIDTH;
    int row_blocks_per_mcu = image_info.m_MCUWidth >> 3;
    int col_blocks_per_mcu = image_info.m_MCUHeight >> 3;

    int mcu_x = 0;
    int mcu_y = 0;
    for (;;)
    {
        unsigned char *pDst_row;

        status = fjpeg_decode_mcu();

        if (status)
        {
            if (status != FJPG_NO_MORE_BLOCKS)
            {
                //printf("pjpeg_decode_mcu() failed with status %u\n", status);
                return -1;
            }
            break;
        }

        if (mcu_y >= image_info.m_MCUSPerCol)return -1;
        

        pDst_row = pImage + mcu_y * col_blocks_per_mcu * row_pitch + mcu_x * row_blocks_per_mcu * nofCh;
        if (image_info.m_scanType == FJPG_GRAYSCALE)
        {
            *pDst_row = image_info.m_pMCUBufR[0];
        }
        else
        {
            unsigned int x,y;
            for (y = 0; y < col_blocks_per_mcu; y++)
            {
                unsigned int src_ofs = (y * 2U);
                for (x = 0; x < row_blocks_per_mcu; x++)
                {
                    pDst_row[0] = image_info.m_pMCUBufR[src_ofs];
                    pDst_row += nofCh;
                    src_ofs += 1;
                }
                pDst_row += row_pitch - 3 * row_blocks_per_mcu;
            }
        }
        mcu_x++;
        if (mcu_x == image_info.m_MCUSPerRow)
        {
            mcu_x = 0;
            mcu_y++;
        }
    }
    return 0;
}
unsigned char g_img[F_WIDTH * F_HEIGHT];
static float g_s[F_WIDTH * F_HEIGHT];// 4  * sigma ** 2  value,it Means 256=  8 sigma
static float g_mu[F_WIDTH * F_HEIGHT];
static float g_v[F_WIDTH * F_HEIGHT];
void detect_move_init(void)
{

    for(int i=0;i<F_WIDTH*F_HEIGHT;i++){
        g_mu[i]=g_img[i];
        g_v[i]=10;
    }
}
unsigned char detect_move(unsigned char *jpgData,size_t jpgSize,float *out_max_s)
{
    g_nInFileOfs = 0;
    g_nInFileSize=jpgSize;
    g_jpgData=jpgData;

    if(decodeToImage(g_img))return -1;
    unsigned char max_t=0;
    for(int i=0;i<F_WIDTH*F_HEIGHT;i++){
        g_mu[i]=0.1f*g_img[i]+0.9f*g_mu[i];
        int sig=1;
        float t=g_img[i]-g_mu[i];
        if(t<0)sig=-1;
        t*=t;
        //too high g_v must be ignored
        if(0 && t>g_v[i]*16){
        }else{
            if(t>g_v[i]){
                g_v[i]=t*0.05f+0.95f*g_v[i];
            }else{
                g_v[i]=t*0.1f+0.9f*g_v[i];
            }
        }
        t=t/(0.01f+g_v[i]);
        g_s[i]=sig*t;
        max_t=max(t,max_t);
    }
    float max_s=0;
    const int  win=5;
    for(int y=0;y<F_HEIGHT-win;y+=2){
        for(int x=0;x<F_WIDTH-win;x+=2){
            float s=0;
            for(int yd=0;yd<win;yd++){
                for(int xd=0;xd<win;xd++){
                    s+=g_s[(y+yd)*F_HEIGHT+x+xd];
                }
            }   
            max_s=max(fabs(max_s),s);
        }
    }
    if(out_max_s) *out_max_s =max_s;
    return  max_s>140?1:0;
}
#include "fjpeg.h"
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include "move_detect.h"
const char *TAG="move_detect";
#if 1
#include "esp_log.h"
#else
#include <stdio.h>
#define ESP_LOGI(t,...)  do{fprintf(stderr,__VA_ARGS__);}while(0)
#define ESP_LOGE(t,...)  do{fprintf(stderr,__VA_ARGS__);}while(0)
#endif

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
        ESP_LOGE(TAG,"pjpeg_decode_init() failed with status %u\n", status);
        if (status == FJPG_UNSUPPORTED_MODE)
        {
            ESP_LOGE(TAG,"Progressive JPEG files are not supported.\n");
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
                ESP_LOGE(TAG,"pjpeg_decode_mcu() failed with status %u\n", status);
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

unsigned char *g_img;
static signed char *g_s;//  sigma ** 2  value,it Means 128=  +/- 11 sigma
static unsigned short *g_mu;//average of  g_img*16 value    
static unsigned short *g_v;// variance*16
void detect_move_init(void)
{
    ESP_LOGI(TAG,"alloc memory");
    g_img=malloc(F_WIDTH*F_HEIGHT*sizeof(char));
    if(!g_img)ESP_LOGE(TAG,"malloc g_img");
    g_s =malloc(F_WIDTH*F_HEIGHT*sizeof(char));
    if(!g_s)ESP_LOGE(TAG,"malloc g_s");
    g_mu=malloc(F_WIDTH*F_HEIGHT*sizeof(short));
    if(!g_mu)ESP_LOGE(TAG,"malloc g_mu");
    g_v =malloc(F_WIDTH*F_HEIGHT*sizeof(short));
    if(!g_v)ESP_LOGE(TAG,"malloc g_v");
    

    for(int i=0;i<F_WIDTH*F_HEIGHT;i++){
        g_mu[i]=g_img[i];
        g_v[i]=30*16;
    }
}

unsigned short detect_move(unsigned char *jpgData,size_t jpgSize)
{
    g_nInFileOfs = 0;
    g_nInFileSize=jpgSize;
    g_jpgData=jpgData;
    const float E_g_v=16.0f;
    if(decodeToImage(g_img))return 0;
    for(int i=0;i<F_WIDTH*F_HEIGHT;i++){
        //g_mu[i]=0.1f*g_img[i]+0.9f*g_mu[i];
        #define IIR_MU_RATE 1
        #define IIR_MU_RATE_L 1
        g_mu[i]= (IIR_MU_RATE*16*g_img[i]+(16-IIR_MU_RATE)*g_mu[i])/16;
        int sig=1;
        float t=g_img[i]-g_mu[i]/16;
        if(t<0)sig=-1;
        t*=t;
        //t=sqrt(t);
        //too high t must be ignored ???
        if(0 && t>g_v[i]*16){
        }else{
            if(t>g_v[i]){
                //g_v[i]=t*0.05f+0.95f*g_v[i];
                g_v[i]= min(16*255, (IIR_MU_RATE_L*16*t+(16-IIR_MU_RATE_L)*g_v[i])/16);
        
            }else{
                //g_v[i]=t*0.1f+0.9f*g_v[i];
                g_v[i]= min(16*255,(IIR_MU_RATE*16*t+(16-IIR_MU_RATE)*g_v[i])/16);
            }
        }
        t=16*t/(E_g_v+g_v[i]);
        g_s[i]=sig*min(127,t);
    }
    unsigned short max_s=0;
    const int  win=3;
    for(int y=0;y<F_HEIGHT-win;y+=2){
        for(int x=0;x<F_WIDTH-win;x+=2){
            short s=0;
            for(int yd=0;yd<win;yd++){
                for(int xd=0;xd<win;xd++){
                    s+=g_s[(y+yd)*F_HEIGHT+x+xd];
                }
            }   
            max_s=max(fabs(max_s),s);
        }
    }


    return max_s;
}
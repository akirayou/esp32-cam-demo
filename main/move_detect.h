#ifndef MOVE_DETECT_H
#define MOVE_DETECT_H
#include <stdio.h>
void detect_move_init(void);
unsigned char detect_move(unsigned char *jpgData,size_t jpgSize,float *out_max_s);

//decoded image size /8
#define F_WIDTH 100
#define F_HEIGHT 75
//for debug: decoded image
extern unsigned char g_img[F_WIDTH * F_HEIGHT];

#endif
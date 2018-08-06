#ifndef MOVE_DETECT_H
#define MOVE_DETECT_H
#include <stdio.h>
void detect_move_init(void);
unsigned short detect_move(unsigned char *jpgData,size_t jpgSize);

//decoded image size /8
#define F_WIDTH 100
#define F_HEIGHT 75
//for debug: decoded image
#if 0
extern unsigned char g_img[F_WIDTH * F_HEIGHT];
#else 
extern unsigned char *g_img;
#endif

#endif
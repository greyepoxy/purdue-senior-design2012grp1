/*
 * File:   interrupts.h
 * Author: Justin
 *
 * Created on February 22, 2012, 5:36 PM
 */

#ifndef interrupts_H
#define interrupts_H

#define TBufferSize 64

volatile unsigned int LEDOnFlag;
volatile unsigned int buttonFlag;
volatile unsigned int timer1Flag;
volatile unsigned int readAccelFlag;
volatile unsigned int readMagFlag;
volatile unsigned int readGyroFlag;
volatile extern char rs232TBuffer[];
volatile unsigned int TBufferHead;
volatile unsigned int TBufferTail;
volatile unsigned int an15Data;
volatile unsigned int front;
volatile unsigned int rear;
volatile unsigned int queue[6];
volatile unsigned int checkatd;
volatile UINT32 tcount;
volatile unsigned char data[27];
volatile UINT32 distance;
volatile unsigned char x_pix;
volatile unsigned char y_pix;
volatile unsigned char calibrationAccel;
volatile unsigned char calibrationGyro;
volatile unsigned char count;
volatile unsigned int dis_flag;
volatile unsigned int cam_flag;
volatile unsigned int dis_tout;

void PutCharacter(const char);

#endif

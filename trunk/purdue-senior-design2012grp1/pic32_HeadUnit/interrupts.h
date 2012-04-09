/*
 * File:   interrupts.h
 * Author: Justin
 *
 * Created on February 22, 2012, 5:36 PM
 */

#ifndef interrupts_H
#define interrupts_H

#define TBufferSize 64

volatile unsigned int buttonFlag;
volatile unsigned int timer1Flag;
volatile unsigned int readAccelFlag;
volatile unsigned int readMagFlag;
volatile unsigned int readGyroFlag;
volatile extern char rs232TBuffer[];
volatile unsigned int TBufferHead;
volatile unsigned int TBufferTail;
volatile unsigned int an15Data;

void PutCharacter(const char);
#endif

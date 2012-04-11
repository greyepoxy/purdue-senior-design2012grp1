/*
 * File:   usb_callback.h
 * Author: Justin
 *
 * Created on April 10, 2012, 10:33 PM
 */

#ifndef usb_callback_H
#define usb_callback_H

#include "USB/usb.h"
#include "USB/usb_function_generic.h"

//User application buffer for receiving and holding OUT packets sent from the host
extern unsigned char OUTPacket[64];
//User application buffer for sending IN packets to the host
extern unsigned char INPacket[64];
extern USB_HANDLE USBGenericOutHandle;
extern USB_HANDLE USBGenericInHandle;


#endif

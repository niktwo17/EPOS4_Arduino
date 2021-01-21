/*******************************************************************************************************/
/*       
Maxon Epos 4 communication for Arduino                                                     
        Apr 21 2018 Denis Gayraud  dgayraud@club-internet.fr     
        Example, fixes for timing, reading method and object-based implementation: Niklas Remiger, January 2021 

        EPOS4 docmentation:     
        http://academy.maxonjapan.co.jp/wp-content/uploads/manual/epos4/EPOS4-Communication-Guide-En.pdf    
        http://academy.maxonjapan.co.jp/wp-content/uploads/manual/epos4/EPOS4-Firmware-Specification-En.pdf
 
  Copyright 2018  Denis Gayraud (dgayraud[at] club-internet [dot] fr)
  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.
  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.*/
/*******************************************************************************************************/
#ifndef __EPOS_H
#define __EPOS_H

#include <Arduino.h>


#define SerialEpos  Serial2 // 
#define ReceiveTimeOut 20 // Maximum for waiting answer from EPOS4 in milli second
#define MaxLenFrame 10  // max number of words in frame


#define RXD2 14
#define TXD2 15

#define MAXPOS 17000
#define MINPOS -17000
#define NEUTRALPOS 18580
//#define ZEROOFFSET 3000

#define STATE_MANUAL 3
#define STATE_AUTONOMOUS 2

#define OPSPEED 7900



class epos
{
public:
    epos();
    ~epos();
    void init();
    bool doHoming();
    void steeringLoop(uint8_t state, float autRad, float manRad);
    float getCurrentPosition();
    

private:
//functions needed
bool ReadFrame();
float inctoRad(int32_t inc);
int32_t RadtoInc(float Rad);
word CalcFieldCRC(word* pDataArray, word numberOfints);
void SendFrame(byte OpCode,word* pDataArray,byte numberOfwords);
bool WriteObject(word Index, byte SubIndex,word* pArray);
bool ReadObject(word Index, byte SubIndex);
void print_rcv_data();
word ReadStatusWord();
inline void SerialEposStuffing(byte BYTE);


//data needed
word data[4];
byte DataRead[264];
byte ReadOpcode;
byte len;
int pDataRead;
word rxCRC;
unsigned long CommErrorCode;
enum read_status {
  RX_DLE,
  RX_STX,
  RX_OPCODE,
  RX_LEN,
  RX_DATA,
  RX_CRC_LOW,
  RX_CRC_HIGH,
  RX_DONE,
  ESC_OPCODE,
  ESC_LEN,
  ESC_DATA,
  ESC_CRC_LOW,
  ESC_CRC_HIGH
}read_st;

int incomingByte = 0;
bool homingFinished = false;
int32_t incToCommand = 0;
int32_t incRead = 0;

};



#endif
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



#include <epos.h>



epos::epos(){}

epos::~epos(){}


void epos::init()
{
  //Serial.begin(115200);
  SerialEpos.begin(115200, SERIAL_8N1, RXD2, TXD2);
  delay(200);

  data[0] = 0x80;
  WriteObject(0x6040, 0, data);
  // set min pos
  data[0] = (0xFFFF + MINPOS - 500);
  data[1] = 0xFFFF;
  WriteObject(0x607D, 0x01, data);
  data[1] = 0;
  data[0] = (MAXPOS + 500);
  // set max pos
  WriteObject(0x607D, 0x02, data);
}
/**
 * Returns true if homing is finished, otherwise we initiate homing. takes up to 10 seconds to perform homing
 */
bool epos::doHoming()
{
if(homingFinished == true)
{
  //eD.SteeringInitError = false;
  return true;
}

else
{
  delay(500);
  data[0] = 0x06;
  WriteObject(0x6060,0,data); // set to homing mode
  // b: param
  data[0] = 1000;
  WriteObject(0x30B2, 0, data); // set max current, in mA
  data[0] = 50;
  WriteObject(0x6099, 0x02, data); // speed for zero search
  data[0] = 600;
  WriteObject(0x6099, 0x01, data); // speed for limit search
  data[0] = NEUTRALPOS;
  WriteObject(0x30B1, 0, data); // how far we go back for neutral pos
//c: method
  data[0] = -4; 
  WriteObject(0x6098, 0, data); //current mode homing method
//d: enable device
  data[0] = 0x06;
  WriteObject(0x6040, 0, data); // shutdowon
  data[0] = 0x0F;
  WriteObject(0x6040, 0, data); // switch on
//e: start homing
  data[0] = 0x1F;
  WriteObject(0x6040,0, data); // start homing
  float timeOUt = millis();

 delay(10000);


 if(((ReadStatusWord() >> 15) & 1U) == true)
    {
      homingFinished = true;
      data[0] = 0x01;
      WriteObject(0x6060, 0, data); // switch to pos mode
      data[0] = OPSPEED;
      WriteObject(0x6081, 0, data); // set the desired speed
      //eD.SteeringInitError = false;
      return true;
    }
  return false;

}
    
}


float epos::getCurrentPosition()
{
  float returnVal = 0;

/*
if(ReadObject(0x6064, 0) == true)
{
  Serial.print("posvalue read");
  Serial.print(DataRead[0], HEX);
  Serial.print (" ");
  Serial.print("SecondVal:");
  Serial.println(DataRead[1], HEX);
    //stateData.currentangle = DataRead[]
    //stateData.eposOK = true;
}
else{ //stateData.eposok = false;
}
*/
/*
ReadObject(0x6064, 0);
int32_t lsb = DataRead[4];
int32_t msb = DataRead[5];
int32_t lsb2 = DataRead[6];
int32_t msb2 = DataRead[7];

incRead = lsb + (msb <<8) + (lsb2 <<16) + (msb2 << 24);
*/
if(ReadObject(0x6064,0) == true)
    {
        int32_t lsb = DataRead[4];
        int32_t msb = DataRead[5];
        int32_t lsb2 = DataRead[6];
        int32_t msb2 = DataRead[7];
        incRead = lsb + (msb <<8) + (lsb2 <<16) + (msb2 << 24);
        /// for maxon different directions
        //incRead = incRead*(-1);
        returnVal = inctoRad(incRead);
        //stateData.steeringAngleRightNow = 0.5;
        //Debug("incRead");
        //Debugln(incRead);
        //Debugln();
        //Debugln();
        //DebugHex(lsb);
        //Debug (" ");
        //DebugHex(msb);
    }
    else
    {
      return -10;
    }

//Serial.print("incRead");
//Serial.println(incRead);


//Serial.println();
//Serial.println();
//Serial.print(lsb, HEX);
//Serial.print (" ");
//Serial.println(msb, HEX);

//what we get from the maxon is stored in DataRead[]...
return returnVal;
}

/**
 *This function invokes the control of the steering based on commanded cases.
 */
void epos::steeringLoop(uint8_t state, float autRad, float manRad)
{
switch (state)
  {
  case STATE_AUTONOMOUS: ///this is aut
      incToCommand = RadtoInc(autRad);
      break;
  case STATE_MANUAL: /// this is manual mode
      incToCommand = RadtoInc(manRad);
      break;
  default:
      incToCommand = 0;
      break;
  }

/// if value is lower than zero we have to do conversion for serial transmission
/// The left direction is positive in robotics, so we do NOT multiply with (-1) anymore
//incToCommand = incToCommand*(-1);
if(incToCommand >=0)
  {
      data[0] = incToCommand;
  }
else
  {
      data[0] = (0xFFFF + incToCommand);
      data[1] = 0xFFFF;
  }
  
  WriteObject(0x607A,0, data); // set absolute position
  data[0] = 0x003F;
  WriteObject(0x6040, 0, data); //absolute positon execution
  data[0] = 0x000F;
  WriteObject(0x6040, 0, data); // toggle updated bit
  data[1] = 0x0000;
}


/**
 * A turning maximum of Pi/2 (90 degrees) to left and right dimension is considered.
 * 0 inc refers middle (neutral position)
 */ 
float epos::inctoRad(int32_t inc)
{
    float calcRad = 0;
    //lets do some type conversion
    calcRad = (1000*inc / MAXPOS) * (PI/2);
    calcRad = calcRad/1000;
    return calcRad;
}

/**
 * A turning maximum of Pi/2 (90 degrees) to left and right dimension is considered.
 * 0 inc refers middle (neutral position)
 */ 
int32_t epos::RadtoInc(float Rad)
{
    int32_t calcInc = 0;
    // lets do some type conversion
    calcInc = (Rad * MAXPOS) * (2/PI);
    return calcInc;
}









/// here the lib begins

word epos::CalcFieldCRC(word* pDataArray, word numberOfints)
{
  word shifter, c;
  word carry;
  word CRC = 0;
  //Calculate pDataArray Word by Word
  while(numberOfints--)
    {
      shifter = 0x8000;          //Initialize BitX to Bit15 
      c = *pDataArray++;         //Copy next DataWord to c
      do
      {
        carry = CRC & 0x8000;    //Check if Bit15 of CRC is set 
        CRC <<= 1;               //CRC = CRC * 2   
        if(c & shifter) CRC++;   //CRC = CRC + 1, if BitX is set in c 
        if(carry) CRC ^= 0x1021; //CRC = CRC XOR G(x), if carry is true 
        shifter >>= 1;           //Set BitX to next lower Bit, shifter = shifter/2
      } while(shifter);
    }
  return CRC;
}

inline void epos::SerialEposStuffing(byte BYTE)
{
  if (BYTE==0x90) SerialEpos.write(BYTE);
  SerialEpos.write(BYTE);
}

void epos::SendFrame(byte OpCode,word* pDataArray,byte numberOfwords)
{
  word CRC;
  word pDataCRC[MaxLenFrame+2];
  pDataCRC[0] = OpCode | (numberOfwords<<8);
  for (int i=0; i<numberOfwords ; i++)
  {
    pDataCRC[i+1]=pDataArray[i];
  }
  pDataCRC[numberOfwords+1]=0x0000;
  CRC=CalcFieldCRC(pDataCRC, (word)(numberOfwords+2));
  SerialEpos.write(0x90);  // DLE=Data Link Escape 
  SerialEpos.write(0x02);  // STX=Start of Text
  SerialEposStuffing(OpCode);
  SerialEposStuffing(numberOfwords);
  for (int i=0; i<numberOfwords ; i++)
  {
    SerialEposStuffing(lowByte(pDataArray[i]));
    SerialEposStuffing(highByte(pDataArray[i]));
  }
  SerialEposStuffing(lowByte(CRC));
  SerialEposStuffing(highByte(CRC));
}

bool epos::WriteObject(word Index, byte SubIndex,word* pArray)
{
  word data[4];
  data[0]= 0x01 | (lowByte(Index)<<8);  // NodeId=1
  data[1]= highByte(Index)|(((word)SubIndex )<< 8);
  data[2]=pArray[0];
  data[3]=pArray[1];
  SendFrame(0x68,data,(byte)4);
  return ReadFrame();
}

bool epos::ReadObject(word Index, byte SubIndex)
{
  word data[2];
  data[0]= 0x01 | (lowByte(Index)<<8);  // NodeId=1
  data[1]= highByte(Index)|(((word)SubIndex )<< 8);
  SendFrame(0x60,data,(byte)2);
  return ReadFrame();
}


// For debuging:
void epos::print_rcv_data()
{
 // delay(1);
      //todo check CRC and print data
    /* 
    Serial. print("OPcode: ");
    Serial. print(ReadOpcode,HEX);
    Serial. print(" len: ");
    Serial. print(len);
    Serial. print(" Data:");
    
    for (int i=0; i<2*len ; i++)
    {
       Serial.print(DataRead[i],HEX);
       Serial.print(" , ");
    }
    Serial.print("CRC: ");   
    Serial.println(rxCRC, HEX); 
    */
}



bool epos::ReadFrame()
{
  incomingByte = 0;   // for incoming SerialEpos data
  unsigned long timer=millis();
  read_st=RX_DLE;
  word pDataCRC[MaxLenFrame+2];
  CommErrorCode=0;
  delay(12);
  bool readDelayDone = false;
  while ((read_st!=RX_DONE) and (millis()-timer < ReceiveTimeOut))
  {
    
    if (SerialEpos.available() > 0)
    {
      /*
      if(readDelayDone == false)
      {
        delay(2); /// !!!! needed delay for library to work
        readDelayDone = true;
      }
      */
      incomingByte = SerialEpos.read();
      /*
      Serial.print("Read: ");
      Serial.println(incomingByte, HEX); 
      Serial.print("read_st: ");
      Serial.println(read_st); 
      */
      
      switch (read_st)
      {
        case RX_DLE:
           if (incomingByte==0x90) read_st=RX_STX;
           pDataRead=0;
           len=0;
           break;
        case RX_STX:
           if (incomingByte==0x02) read_st=RX_OPCODE;
           else read_st=RX_DLE;
           break;
        case RX_OPCODE:
           if ( incomingByte == 0x90)
           {
            read_st=ESC_OPCODE;
            break;
           }
           else
           {
             ReadOpcode=incomingByte;
             read_st=RX_LEN;
           }
           break;
        case RX_LEN:
           len=incomingByte;
           if ( incomingByte == 0x90)
           {
            read_st=ESC_LEN;
            break;
           }
           else read_st=RX_DATA;
           break;
        case RX_DATA:
           if ( incomingByte == 0x90)
           {
            read_st=ESC_DATA;
            break;
           }
           else
           {
             DataRead[pDataRead]=incomingByte;
             pDataRead++;
           }
           if ( pDataRead== (2*len) ) read_st=RX_CRC_LOW;
           break;
        case RX_CRC_LOW:
           rxCRC=incomingByte; 
           if ( incomingByte == 0x90) read_st=ESC_CRC_LOW;
           else read_st=RX_CRC_HIGH;
           break;
        case RX_CRC_HIGH:
           rxCRC +=incomingByte<<8;
           if ( incomingByte == 0x90) read_st=ESC_CRC_HIGH;
           else read_st=RX_DONE;
           break; 
        case ESC_OPCODE:
           if (incomingByte== 0x02)
           {
             read_st=RX_OPCODE;
             break;
           }
           else if (incomingByte== 0x90)
           {
             ReadOpcode=incomingByte;
             read_st=RX_LEN;
             break;
           }
           else //Serial.println ("Protocol error: single DLE");   
           break;
        case ESC_LEN:
           if (incomingByte== 0x90)
           {
             read_st=RX_DATA;
             break;
           }
           if (incomingByte== 0x02)
           {
             read_st=RX_OPCODE;
             break;
           }
           //Serial.println ("Protocol error: Escape len error");
           break;
        case ESC_DATA:
           if ( incomingByte == 0x90)
           {
             DataRead[pDataRead]=incomingByte;
             pDataRead++;
             read_st=RX_DATA;
             if ( pDataRead== (2*len) ) read_st=RX_CRC_LOW;
             break;
           }
           if (incomingByte== 0x02)
           {
             read_st=RX_OPCODE;
             break;
           }
           //Serial.println ("Protocol error: Escape data error");
           break;
           
        case ESC_CRC_LOW:
           if ( incomingByte == 0x90) read_st=RX_CRC_HIGH;
           else if (incomingByte== 0x02) read_st=RX_OPCODE;
           else //Serial.println ("Protocol error: Escape crc l error");
           break;
        case ESC_CRC_HIGH:
           if ( incomingByte == 0x90) read_st=RX_DONE;
           else if (incomingByte== 0x02) read_st=RX_OPCODE;
           else //Serial.println ("Protocol error: Escape crc h error");
           break;
        default:
           break;
      }
    }
  }
// Check Time out:
  if (millis()-timer >= ReceiveTimeOut)
  {
    //Serial.println("Serial Time out");
    return false;
  }
// check CRC:
  pDataCRC[0] = ReadOpcode | ((word)len)<<8;
  for (int i=0 ;  i< len ;i++ )
  {
    pDataCRC[i+1]= DataRead[2*i] | (((word)DataRead[2*i+1])<<8);
  }
  pDataCRC[len+1]=0x0000;
  if (CalcFieldCRC(pDataCRC, (word)(len+2))!= rxCRC)
  {
    //Serial.println("Error CRC");
    //return false;
  }
//  //Serial.print("RCV CRC: ");//Serial.println (CalcFieldCRC(pDataCRC, (word)(len+2)),HEX);
// Check communication error code
  CommErrorCode= DataRead[0] | (((word)DataRead[1])<<8) | (((unsigned long)DataRead[2])<<16) | (((unsigned long)DataRead[3])<<24);
  if (CommErrorCode!=0)
  {
    //Serial.print( "Communication Error Code:");
    //Serial.println(CommErrorCode,HEX);
    //return false;
  }
print_rcv_data(); 
return true;
}


word epos::ReadStatusWord()
{
  word statusWord;
  ReadObject(0x6041,0);
  statusWord = DataRead[4] + (((word) DataRead[5])<<8);
  //Serial.print("Statusword: ");
  for (int i = 0; i<6; i++)
  {
    //Serial.print(DataRead[i],HEX);
  }
  //Serial.println();

  return (statusWord);
}
//LoRaTXonly.h
/*
**************************************************************************************************
ProMiniLoRaTracker_V2 Programs

Copyright of the author Stuart Robinson - 17/07/15 15:00

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.
**************************************************************************************************
*/


/*
**************************************************************************************************
Variable definitions
**************************************************************************************************
*/

//byte Variables
byte  lora_TXStart;				//start of packet data in TXbuff
byte  lora_TXEnd;				//end of packet data in TXbuff
byte  lora_FTXOK;				//flag, set to 1 if TX OK
byte  lora_TXPacketType;		//type number of packet to send
byte  lora_TXDestination;		//destination address of packet to send
byte  lora_TXSource;			//source address of packet received
byte  lora_TXPacketL;			//length of packet to send, includes source, destination and packet type.

//byte arrays
byte  lora_TXBUFF[128];			//buffer for packet to send

//Integer variables, more than byte
long lora_TXpacketCount;		//count of packets sent


/*
**************************************************************************************************
Library Functions
**************************************************************************************************
*/




void lora_TXONLoRa(byte lora_LTXPower)
{
  //turns on LoRa transmitter, Sends packet, power level is from 2 to 17
  //Serial.print("lora_TXONLoRa() Pwr ");
  //Serial.print(lora_LTXPower);
  //Serial.println("dBm");
  byte lora_Lvar1;
  lora_Lvar1 = lora_LTXPower + 0xEE;						// has effect of converting 17 into 15
  lora_Write(lora_RegPaConfig, lora_Lvar1);				// set TX power
  lora_Write(lora_RegOpMode, 0x8B);						// TX on direct mode, low frequency mode
 }


void lora_Send(byte lora_LTXStart, byte lora_LTXEnd, byte lora_LTXPacketType, byte lora_LTXDestination, byte lora_LTXSource, long lora_LTXTimeout, byte lora_LTXPower)
{
  //fills FIFO with 3 header bytes, then from lora_TXBUFF(256) starting at lora_TXStart ending at lora_TXEnd,maximum of 252 btes
  //Serial.print("lora_Send() ");
  //Serial.print(lora_LTXStart);
  //Serial.print(" ");
  //Serial.print(lora_LTXEnd);
  //Serial.println();
  //Serial.print("TX ");
  //Serial.println(lora_LTXPacketType);
  int lora_Lvar1;
  byte lora_LRegData;
  byte lora_LTXPacketL;
  lora_TXStart = lora_LTXStart;
  lora_TXEnd = lora_LTXEnd;
  lora_TXDestination = lora_LTXDestination;
  lora_TXPacketType = lora_LTXPacketType;
  lora_TXSource = lora_LTXSource;
  lora_Write(lora_RegOpMode, 0x09);
  lora_Write(lora_RegIrqFlags, 0xFF);
  lora_Write(lora_RegIrqFlagsMask, 0xF7);
  lora_Write(lora_RegFifoTxBaseAddr, 0x00);
  lora_Write(lora_RegFifoAddrPtr, 0x00);  		// start burst read

  digitalWrite(lora_PNSS, LOW);					// Set NSS low
  SPI.transfer(lora_WRegFifo);					// address for burst write
  SPI.transfer(lora_LTXPacketType);				// Write the packet type
  SPI.transfer(lora_LTXDestination);				// Destination node
  SPI.transfer(lora_LTXSource);					// Source node
  lora_LTXPacketL = 3;							// We have added 3 header bytes

  for (lora_Lvar1 = lora_TXStart;  lora_Lvar1 <= lora_TXEnd; lora_Lvar1++)
  {
    lora_LTXPacketL++;

    if (lora_LTXPacketL > 253)					// check for overlong packet here, wraps around from limit at 251 to 0
    {
      Serial.print("ERROR,PacketatLimit ");
      lora_LTXPacketL--;						// remove increment to packet length
      break;
    }
    lora_LRegData = lora_TXBUFF[lora_Lvar1];
    SPI.transfer(lora_LRegData);
  }

  digitalWrite(lora_PNSS, HIGH);					// finish the burst write
  lora_TXPacketL = lora_LTXPacketL;
  lora_Write(lora_RegPayloadLength, lora_LTXPacketL);
  //Serial.print("Transmit Timeout ");
  //Serial.print(lora_LTXTimeout);
  //Serial.print(" Seconds");
  //Serial.println();
  lora_LTXTimeout = lora_LTXTimeout * 945;			// convert seconds to mS, delay in TX done loop is 1ms
  lora_TXONLoRa(lora_LTXPower);

  do
  {
    delay(1);
    lora_LTXTimeout--;
    lora_LRegData = lora_Read(lora_RegIrqFlags);
  }
  while (lora_LTXTimeout > 0 && lora_LRegData == 0) ;		// use a timeout counter, just in case the TX sent flag is missed

  lora_TXOFF();

  if (lora_LTXTimeout == 0)
  {
    Serial.print("ERROR,TXtimeout");
    Serial.println();
    lora_FTXOK = 0;
  }
  else
  {
    //Serial.print("Packet Sent");
    //Serial.println();
    lora_FTXOK = 1;
    lora_TXpacketCount++;
  }
}

void lora_TXPKTInfo()
{
  //print the information for TX packet last sent
  Serial.print("lora_TXPKTInfo() ");
  Serial.print("TXtype,");
  Serial.print(lora_TXPacketType);
  Serial.print(",TXDestination,");
  Serial.print(lora_TXDestination);
  Serial.print(",TXSource,");
  Serial.print(lora_TXSource);
  Serial.print(",TXPacketLength,");
  Serial.print(lora_TXPacketL);
  Serial.print(",TXPacketCount,");
  Serial.print(lora_TXpacketCount);
  Serial.println();
}

byte lora_TXBuffPrint(byte lora_LPrint)
{
  //Print contents of TX buffer as ASCII,decimal or HEX
  Serial.print("lora_TXBuffPrint() ");
  //Serial.print(lora_LPrint);
  //Serial.print(") ");
  //Serial.print(lora_TXStart);
  //Serial.print(" ");
  //Serial.print(lora_TXEnd);
  //Serial.print(" Start>>");                         // print start marker so we can be sure where packet data starts

  byte lora_LLoop;
  byte lora_LData;

  for (lora_LLoop = lora_TXStart; lora_LLoop <= lora_TXEnd; lora_LLoop++)
  {
    lora_LData = lora_TXBUFF[lora_LLoop];
    if (lora_LPrint == 0)
    {
      Serial.write(lora_LData);
    }
    if (lora_LPrint == 1)
    {
      Serial.print(lora_LData);
      Serial.print(" ");
    }

    if (lora_LPrint == 2)
    {
      Serial.print(lora_LData, HEX);
      Serial.print(" ");
    }
  }
  //Serial.print("<<End");                              // print end marker so we can be sure where packet data ends
  Serial.println();
}







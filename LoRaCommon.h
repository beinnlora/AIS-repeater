//LoRaCommon.h
/*****************************************************************************************************
ProMiniLoRaTracker_V2 Programs

Copyright of the author Stuart Robinson - 15/07/15 15:00

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.
******************************************************************************************************
*/

//byte Variables
byte  lora_FDeviceError;		//flag, set to 1 if RFM98 device error

//Float Variables
double lora_Frequency;			//last programmed frequency

/*
**************************************************************************************************
Constant definitions
**************************************************************************************************
*/

//LoRa names for bandwidth settings
const byte lora_BW7_8 = 0;      //7.8khz
const byte lora_BW10_4 = 16;    //10.4khz
const byte lora_BW15_6 = 32;    //15.6khz
const byte lora_BW20_8 = 48;    //20.8khz
const byte lora_BW31_2 = 64;    //31.2khz
const byte lora_BW41_7 = 80;    //41.7khz
const byte lora_BW62_5 = 96;    //62.5khz
const byte lora_BW125 = 112;    //125khz
const byte lora_BW250 = 128;    //250khz
const byte lora_BW500 = 144;    //500khz

//Spreading Factors
const byte lora_SF6 = 6;
const byte lora_SF7 = 7;
const byte lora_SF8 = 8;
const byte lora_SF9 = 9;
const byte lora_SF10 = 10;
const byte lora_SF11 = 11;
const byte lora_SF12 = 12;

//LORA names for coding rate settings
const byte lora_CR4_5 = 2;	//4:5
const byte lora_CR4_6 = 4;	//4:6
const byte lora_CR4_7 = 6;	//4:7
const byte lora_CR4_8 = 8;	//4:8

//LORA Header Settings
const byte lora_Explicit    = 0;	//Use to set explicit header
const byte lora_Implicit    = 1;	//Use to set implicit header

//Misc definitions
const byte lora_Deviation = 0x52;
const byte lora_LowDoptON = 0x08;       //value to turn low data rate optimization on
const byte lora_LowDoptOFF = 0x00;      //value to turn low data rate optimization off
const byte lora_PrintASC = 0;           //value to cause buffer print to appear as ASCII
const byte lora_PrintNum = 1;           //value to cause buffer print to appear as decimal numbers
const byte lora_PrintHEX = 2;           //value to cause buffer print to appear as hexadecimal numbers


//SX1278 Register names
const byte lora_RegFifo = 0x00;
const byte lora_WRegFifo = 0x80;
const byte lora_RegOpMode = 0x01;
const byte lora_RegFdevLsb = 0x05;
const byte lora_RegFrMsb = 0x06;
const byte lora_RegFrMid = 0x07;
const byte lora_RegFrLsb = 0x08;
const byte lora_RegPaConfig = 0x09;
const byte lora_RegOcp = 0x0B;
const byte lora_RegLna = 0x0C;
const byte lora_RegFifoAddrPtr = 0x0D;
const byte lora_RegFifoTxBaseAddr = 0x0E;
const byte lora_RegFifoRxBaseAddr = 0x0F;
const byte lora_RegFifoRxCurrentAddr = 0x10;
const byte lora_RegIrqFlagsMask = 0x11;
const byte lora_RegIrqFlags = 0x12;
const byte lora_RegRxNbBytes = 0x13;
const byte lora_RegRxHeaderCntValueMsb = 0x14;
const byte lora_RegRxHeaderCntValueLsb = 0x15;
const byte lora_RegRxPacketCntValueMsb = 0x16;
const byte lora_RegRxPacketCntValueLsb = 0x17;
const byte lora_RegPktSnrValue = 0x19;
const byte lora_RegPktRssiValue = 0x1A;
const byte lora_RegRssiValue = 0x1B;
const byte lora_RegFsiMSB = 0x1D;
const byte lora_RegFsiLSB = 0x1E;
const byte lora_RegModemConfig1 = 0x1D;
const byte lora_RegModemConfig2 = 0x1E;
const byte lora_RegSymbTimeoutLsb = 0x1F;
const byte lora_RegPreambleLsb = 0x21;
const byte lora_RegPayloadLength = 0x22;
const byte lora_RegFifoRxByteAddr = 0x25;
const byte lora_RegModemConfig3 = 0x26;
const byte lora_RegPacketConfig2 = 0x31;
const byte lora_TXdefaultpower = 17;

/*
**************************************************************************************************
Library Functions
**************************************************************************************************
*/

void lora_ResetDev()
{
  //resets the LoRa device
  //Serial.print("lora.ResetDev()");
  //Serial.println();
  digitalWrite(lora_PReset, LOW);	// take reset line low
  delay(5);
  digitalWrite(lora_PReset, HIGH);	// take it high
}


void lora_Write(byte lora_LReg, byte lora_LData)
{
  //write a byte to a LoRa register
  //Serial.print("lora_Write() ");
  //lora_PrintAsHEX(lora_LReg);
  //Serial.print(" ");
  //lora_PrintAsHEX(lora_LData);
  //Serial.println();
  digitalWrite(lora_PNSS, LOW);		// set NSS low
  SPI.transfer(lora_LReg | 0x80);	// mask address for write
  SPI.transfer(lora_LData);		// write the byte
  digitalWrite(lora_PNSS, HIGH);	// set NSS high
}

byte lora_Read(byte lora_LReg)
{
  //read a byte to a LoRa register
  //Serial.print("lora_Read()");
  //Serial.println();
  byte lora_LRegData;
  digitalWrite(lora_PNSS, LOW);		// set NSS low
  SPI.transfer(lora_LReg & 0x7F);	// mask address for read
  lora_LRegData = SPI.transfer(0);	// read the byte
  digitalWrite(lora_PNSS, HIGH);	// set NSS high
  return lora_LRegData;
}


void lora_SetFreq(byte lora_LFMsb, byte lora_LFMid, byte lora_LFLsb)
{
  lora_Write(lora_RegFrMsb, lora_LFMsb);
  lora_Write(lora_RegFrMid, lora_LFMid);
  lora_Write(lora_RegFrLsb, lora_LFLsb);
}

byte lora_CheckDevice()
{
  //check there is a device out there, program the 3 frequency setting registers and read back
  //Serial.print("LORA.CheckDevice() ");
  //Serial.println();
  byte lora_Lvar1, lora_Lvar2, lora_Lvar3;
  lora_Write(lora_RegFrMsb, 0x5A);
  lora_Write(lora_RegFrMid, 0xAA);
  lora_Write(lora_RegFrLsb, 0xA5);
  lora_Lvar1 = lora_Read(lora_RegFrMsb);	// Read RegFrMSB
  lora_Lvar2 = lora_Read(lora_RegFrMid);	// Read RegFrMid
  lora_Lvar3 = lora_Read(lora_RegFrLsb);	// Read RegFrLSB
  if (lora_Lvar1 != 0x5A || lora_Lvar2 != 0xAA || lora_Lvar3 != 0xA5)
  { return false;
    lora_FDeviceError = 0;
  }
  else
  { return true;
    lora_FDeviceError = 1;
  }
}

void lora_Setup()
{
  //initialize LoRa device registers and check its responding
  //Serial.print("lora_Setup()");
  //Serial.println();
  lora_ResetDev();				// Clear all registers to default
  lora_Write(lora_RegOpMode, 0x08);		// RegOpMode, need to set to sleep mode before configure for LoRa mode
  lora_Write(lora_RegOcp, 0x0B);		// RegOcp
  lora_Write(lora_RegLna, 0x23);		// RegLna
  lora_Write(lora_RegSymbTimeoutLsb, 0xFF);	// RegSymbTimeoutLsb
  lora_Write(lora_RegPreambleLsb, 0x0C);	// RegPreambleLsb, default
  lora_Write(lora_RegFdevLsb, lora_Deviation);	// LSB of deviation, 5kHz
  if (!lora_CheckDevice())
  {
    Serial.print("LoRa Device Not Working");
    Serial.println();
  }
}


byte lora_TXONDirect(byte lora_LTXPower)
{
  //turns on transmitter,in direct mode for FSK and audio  power level is from 2 to 17
  //Serial.print("lora_TXONDirect(");
  //Serial.print(lora_LTXPower);
  //Serial.print(")");
  //Serial.println();
  byte lora_Lvar1;
  lora_Lvar1 = lora_LTXPower + 0xEE;		// has effect of converting 17 into 15
  lora_Write(lora_RegPaConfig, lora_Lvar1);	// set TX power
  lora_Write(lora_RegOpMode, 0x0B);		// TX on direct mode, low frequency mode
}

void lora_TXOFF()
{
  //turns off transmitter
  //Serial.print("lora_TXOFF()");
  //Serial.println();
  lora_Write(lora_RegOpMode, 0x08);             // TX and RX to sleep, in direct mode
}

void lora_DirectSetup()
{
  //setup LoRa device for direct modulation mode
  //Serial.print("lora_DirectSetup()");
  //Serial.println();
  lora_Write(lora_RegOpMode, 0x08);
  lora_Write(lora_RegPacketConfig2, 0x00);	// set continuous mode
}


void lora_Tone(int lora_LFreq, int lora_LToneLen, int lora_LTXPower )
{
  //Transmit an FM tone
  Serial.println("lora_Tone()");
  //Serial.println();
  lora_DirectSetup();
  lora_TXONDirect(lora_LTXPower);		// TXON, power 10
  lora_Write(lora_RegFdevLsb, lora_Deviation);	// We are generating a tone so set the deviation, 5kHz
  tone(lora_PPWMCH, lora_LFreq);
  delay(lora_LToneLen);
  lora_TXOFF();
}

void lora_SetModem(byte lora_LBW, byte lora_LSF, byte lora_LCR, byte lora_LHDR, byte lora_LLDROPT)
{
  //setup the LoRa modem parameters
  //Serial.print("lora_SetModem()");
  //Serial.println();
  byte lora_Lvar1, lora_Lvar2;
  lora_Lvar1 = lora_LBW + lora_LCR + lora_LHDR;  // calculate value of RegModemConfig1
  lora_Lvar2 = lora_LSF * 16 + 7;		 // calculate value of RegModemConfig2, ($07; Header indicates CRC on, RX Time-Out MSB = 11
  lora_Write(lora_RegOpMode, 0x08);		 // RegOpMode, need to set to sleep mode before configure for LoRa mode
  lora_Write(lora_RegOpMode, 0x88);		 // goto LoRa mode
  lora_Write(lora_RegModemConfig1, lora_Lvar1);
  lora_Write(lora_RegModemConfig2, lora_Lvar2);
  lora_Write(lora_RegModemConfig3, lora_LLDROPT);
}

void lora_PrintModem()
{
  //Print the LoRa modem parameters
  Serial.print("lora_PrintModem() ");
  Serial.print(lora_Read(lora_RegModemConfig1));
  Serial.print(" ");
  Serial.print(lora_Read(lora_RegModemConfig2));
  Serial.print(" ");
  Serial.println(lora_Read(lora_RegModemConfig3));
}


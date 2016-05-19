

//ProMiniLoRaTracker_TX_V2_8.ino

/*
**************************************************************************************************
Stephen Wilson 11-5-16

Original Code based on ProMiniLoRaTracker_V2 Programs   Copyright of the author Stuart Robinson - 05/09/2015


**************************************************************************************************
*/

/* Stuff to Do
   more elegance on mulipart AIS messages to fit inside LoRa buffer
 - fit RTC to teensy, and routine for longer sleeping over night - perhaps wake for 1 minute in 10?
 * 
*/

//#define ss Serial1
#define daisyser Serial2


//Hardware definitions
const byte lora_PNSS = 2;              //pin number where the NSS line for the LoRa device is connected.
const byte PLED1 = 0;                       //pin number for LED on Tracker
const byte lora_PReset = 3;             //pin where LoRa device reset line is connected
const byte lora_PPWMCH =4;                 //pin number for tone generation, connects to LoRa device DIO2.
//const int RXPin = A5, TXPin = A4;           //pins for soft serial Version 2 PCBs
//const byte RCPulse = A0;                    //pin number for LED on Tracker
const byte daisypower = 6;                       //pin for powering up and down the daisy
//AO is voltage input

//Program constants
const byte f1 = 108, f2 = 153, f3 = 153;     //Set the LoRa frequency, 434.400 Mhz
const byte GPSNumLoop = 5;                   //number of GPS reads to do before TX packet
unsigned long const SecondsToLost = 60;    //number of seconds after power up that lost mode starts
const byte thisnode = 3;                     //Node number for this device
const uint32_t GPSBaud = 9600;               //GPS baud rate
const byte pulseerrorlimit = 100; 	     //Number of RC error pulses needed to trigger lost condition, 255 max
const byte holddifference = 30;		     //If differance between two RC pulses is less than this, then hold is assumed
const int RCpulseshort = 750; 		     //In uS, lower than this RC pulse is assumed not to be valid
const int RCpulselong = 2500; 		     //In uS, higher than this RC pulse is assumed not to be valid

const int brownout = 3416;           //threshold approx decivolts for when to sleep Vbat = 0.001451.ADC - 1.3584
const int hyst = 100;             // hysteresis for when the unit should wake back up again brownot+hyst
const long sleepfor = 30000;        //ms to sleep when sleepy


const int rpm = 120;      //sim rate of message per minute
//Program Variables
String InputString = "";                     //used in LoRa transmissions
String Outputstring = "";                    //used in LoRa transmissions
unsigned long UpSeconds;                     //how many seconds after startup till lost mode triggered
byte GPSLoop;                                //used to count GPS checks
int SupplyVolts;                             //stores tracker supply volts
int RCPulseLen;                              //length of RC pulse
int pulseerrorcount;  	                     //number of RC pulse errors recorded
int pulsewidthlast = 1500; 	             //Measured width in uS of last RC pulse read, used for hold check
float Tlat, Tlon;                            //to store last known lat and long

long lastvoltcheck = 0;      //when did we last check for voltage on batt
long voltcheckevery = 60000;   //check voltage every 5 mins 1min for debug 12345 so not to clash with 10s AIS
long ticktocklast=0;			//ticktock message timers for serial display to show working
long ticktockevery=1000;
byte tickstate=0;

int aiscount=0;					//number of AIS messages receiver
int aislimit=3;					//number of AIS messages to buffer before transmitting (to save power)
//LoRa packet size should allow 3 AIS sentences per LoRa packet
String aisbuffer="";
int sleepy = 0;         //if just awoken from a sleep, do this.

int bytein;       //get serial from daisy
char buf[50]; //generic storage buffer
byte loraflag = 0; //flag for packet type - 1 = AIS report  0 = battery report *or other

String aisheader = "!AIVDM,";


//Includes
#include <SPI.h>
#include "LoRaCommon.h"
#include "LoRaTXonly.h"
//#include "LoRaRXonly.h"
#include <TinyGPS++.h>
//set up our Teensy snooze function
#include <Snooze.h>
SnoozeBlock config;

#include <EEPROM.h>	//not needed in this ver

//TinyGPSPlus gps;                             //Create the TinyGPS++ object
//SoftwareSerial ss(RXPin, TXPin);             //Create the serial connection to the GPS device
String indata = "";

void setup()
{
    /********************************************************
        Set Low Power Timer wake up in milliseconds.
      ********************************************************/
    config.setTimer(sleepfor);// milliseconds

    //configure all unused pins to OUTPUT for powersaving on teensy
    for (int i = 0; i < 46; i++)
    {
        pinMode(i, OUTPUT);
    }


    Serial.begin(9600);                           //Setup Serial console ouput
    Serial.println("ProMiniLoRaTracker_TX_V2_1 - Stuart Robinson - 06/09/2015");
    Serial.println();

    daisyser.begin(38400);            //star serial comms with daisy
    //pinMode(RXPin, INPUT);      //softserial
    //pinMode(RCPulse, INPUT);      //for RC Pulse input
    pinMode(lora_PReset, OUTPUT);     //RFM98 reset line
    pinMode (lora_PNSS, OUTPUT);      //set the slave select pin as an output:
    pinMode(daisypower, OUTPUT);
    pinMode(A0,INPUT);			//analog input for measuring batt voltage. scale Vbatmaxto 1.2V to reference internal vref of teensy
    digitalWrite(lora_PNSS, HIGH);
    pinMode(PLED1, OUTPUT);     //for PCB LED
    SPI.begin();          //initialize SPI
    SPI.setClockDivider(SPI_CLOCK_DIV2);
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    initLora();
    digitalWrite(daisypower,HIGH);    //turn on AIS



    analogReadResolution(12);		//set ADC resolution for Vbat measurement
    analogReference(INTERNAL);	//set reference to internal 1.2V reference source so it doesn't brownout below 3.3v
   // delay(5000);  //sartup delay
    //get batt voltage, convert to string and output to serial and LoRa
    ltoa(GetSupplyVolts(), buf, 10);
    String temp = buf;
    Outputstring = "Voltage:  " + temp;
    Serial.println (Outputstring);
    SendStringasLoRa();


    Serial.println ("power up");
    //#digitalWrite(PLED1, HIGH);
    //#lora_Tone(1000, 1000, 10);                   //Transmit an FM tone, 1000hz, 1000ms
    //#digitalWrite(PLED1, LOW);
    //#delay(500);
    // lora_SetModem(lora_BW41_7, lora_SF12, lora_CR4_5, lora_Explicit, lora_LowDoptON);  //Setup the LoRa modem parameters

    //Outputstring = "Hello, world!";
    //SendStringasLoRa();
    lora_SetModem(lora_BW250, lora_SF7, lora_CR4_6, lora_Explicit, lora_LowDoptOFF);  //Setup the LoRa modem parameters
    //SendStringasLoRa();
    delay (200); //to allow batt to settle - add cap?
} //END SETUP

void loop()
{
    //if we have woken from a sleep (paused inside the loop though) then first check if we have a good battery level
    // and go back to sleep if so
    while (sleepy == 1)
    {
        ///////////////////////////////
        // STANDBY LOOP - if battery is too low
        ///////////////////////////////

        delay(200);
        //just woken up - don't turn stuff on yet - check batt voltage
        //if battery voltage is below the brownout level (plus some hysteresis)
        if (GetSupplyVolts() < (brownout+hyst))
        {
            //still too low - go back to sleep
            Serial.println ("still too low, sleeping again");

            gotosleep();
            //after sleeping, we resume code execution here
            //while loop then deals with if we are still sleepy and

        } else
        {
            //wakeup
            Serial.println ("Waking up - enough juice!");
            //power on the daisy, set sleepy=0 etc,
            wakeywakey();

        }
    } //END while sleepy

    //WE ARE NOT SLEEEPY - go and transmit our AIS sentences - do we need power to serial to do this - presume
    // snooze library turns on serial again so we can read - or is HWserial always available - possible power saving here.

    while  ( sleepy != 1)
    {
        ///////////////////////////////
        //AIS RECEIVE AND REPEAT LOOP
        ///////////////////////////////

        //serial tick tock message
        long nexttick = ticktocklast + ticktockevery;
        if (millis() > nexttick)
        {
            ticktocklast = millis();
            digitalWrite(PLED1,HIGH);
            delay(100);
            digitalWrite(PLED1,LOW);
            if (tickstate==0)
            {
                Serial.println("tick");
                tickstate=1;
            } else {
                Serial.println("tock");
                tickstate=0;
            }
        }
        //check our voltage every x mins
        long nextcheck = lastvoltcheck + voltcheckevery;
        if (millis() > nextcheck)
        {
            //CHECKVOLTAGE and send voltage

            Serial.println("CHECKING VOLTAGE");

            lastvoltcheck = millis();
            ltoa(GetSupplyVolts(), buf, 10);
            String temp = buf;
            Outputstring = temp;
            SendStringasLoRa();

            //if too low, go to sleep
            if (GetSupplyVolts() < brownout) {
                gotosleep();
                //returns from sleep here, so exit while loop early
                break;
                //sets sleepy=1 here, exits this transmit while loop

            }

        } //end check voltage

        //we are not sleepy and we have a char at serial, read it and process
        if(daisyser.available() > 0 && sleepy != 1)
        {
            char received = daisyser.read();

            indata += received;
            if (received == '\n')
            {
                //FULL LINE RECEIVED - PROCESS
                Serial.print(indata);
                //if line is an AIS sentence, then transmit it
                if (isais(indata) == 1)
                {

                    ltoa(GetSupplyVolts(), buf, 10);
                    String temp = buf;
                    //Outputstring = cleanstring("!AIVDM,1,1,,A,133m@ogP00PD;88MD5MTDww@2D7k,0*46") + " " + temp;
                    //strip crlf from received line, add the voltage and transmit
                    //Outputstring = (indata.substring(0,indata.length()-2) + " " + temp);
                    //strip CRLF
                    aisbuffer = aisbuffer + (indata.substring(0,indata.length()-2));
                    Serial.print("aisbuffer:");
                    Serial.println(aisbuffer);
                    aiscount++;
                    //if we have more ais sentences than buffer allows then transmit
                    //could be more elegant here for multipart AIS messages
                    if (aiscount==aislimit)
                    {
                        aiscount=0;
                        Outputstring=aisbuffer;
                        SendStringasLoRa();
                        aisbuffer="";

                    }

                }
                //CLEAR daisy read buffer
                indata = "";
            }
        }
    } //END while bytes available and not sleepy


} //END LOOP



/////////////////////
// FUNCTIONS
////////////////////

//is a string an AIS string?
byte isais(String tempstr)
{
    int aislen = aisheader.length();
    if (tempstr.substring(0, aislen) == aisheader)
    {
        return 1;
    }
    else
    {
        return 0;
    }

}

//strip AIS header from string if present and replace by _ character (never found in AIS message)
String cleanstring(String tempstr)
{
    tempstr.replace(aisheader,"_");
    //takes a string, if ais header is present, then strips off
    //  if (isais(tempstr) == 1)
    // {

    //   Serial.print ("Found aisheader: ");
    //  Serial.println (tempstr.substring(0, aisheader.length()));
    //return all except header and last carr return

    // return tempstr.substring(aisheader.length(),(tempstr.length()));
    //} else
    //{
    return tempstr;
}

void SendStringasLoRa()
{
    //parse the string and set the flag to indicate whether this is AIS or not

    int loraflag;

    //IF AIS PACKET, then flag byte=1, strip header
    if (isais(Outputstring) == 1) {
        loraflag = 01;
        //strip header if sou
        Outputstring = cleanstring(Outputstring);

    }
    //if outputstring is SLEEP notifier, then flag byte = 2 and strip SLEEP
    else if (Outputstring.substring(0,5)=="SLEEP")
    {
        loraflag=02;
        Outputstring=Outputstring.charAt(5);
    }
    else {
        //anything else is a generic message (used for Voltage logging at present),
        loraflag = 00;
    }


    Serial.print("  ");
    Serial.println(Outputstring);
    FillTXBuffString(Outputstring);
    digitalWrite(PLED1, HIGH);                                        //LED on during packet
    lora_Send(lora_TXStart, lora_TXEnd, loraflag, 02, thisnode, 10, 10);   //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
    digitalWrite(PLED1, LOW);
}


void FillTXBuffString(String tempstr)
{
    byte ltemp, i;
    ltemp = tempstr.length();                                        //Outputstring has the LoRa Telemetry data to send
    lora_TXStart = 0;
    lora_TXEnd = 0;

    for (i = 0; i <= ltemp; i++)                                     //Fill TX Buffer with output string
    {
        lora_TXBUFF[i] = Outputstring.charAt(i);
    }
    i--;
    i--; //dont need the last null character
    lora_TXEnd = i;
}


void initLora()
{
    lora_ResetDev();			         //Reset the device
    lora_Setup();			                 //Do the initial LoRa Setup
    lora_SetFreq(f1, f2, f3);
    //lora_SetModem(lora_BW41_7, lora_SF12, lora_CR4_5, lora_Explicit, lora_LowDoptON);	//Setup the LoRa modem parameters
}

int GetSupplyVolts()
{
    //changed to Aref internal ~ 1.2V ie Vin---70k--ADC--30k---0v

    int i, temp;
    SupplyVolts = 0;
    temp = analogRead(A0);
    temp=0;
    delay(20);
    for (i = 0; i <= 9; i++)                                  // sample AD 10 times
    {
        temp = analogRead(A0);
        SupplyVolts = SupplyVolts + temp;
    }

    SupplyVolts = (SupplyVolts / 10);
    //int outvolts = map(SupplyVolts, 747, 1023, 300, 470);
    //return outvolts;

    Serial.println(SupplyVolts);
    return SupplyVolts;
}



void systemerror()
{
    //indicates an error with RFM98 or GPS
    while (1)
    {
        digitalWrite(PLED1, HIGH);
        delay(50);
        digitalWrite(PLED1, LOW);
        delay(50);
    }
}


void gotosleep()
{
    //we might be going back to sleep having just woken up
    if (sleepy==0)
    {
        //previously awake, so notify base that we are now starting a new sleep
        Outputstring="SLEEP1";
        SendStringasLoRa();
    }

    sleepy = 1;
    Serial.print("going to sleep");
    //TURN OFF DAISY
    digitalWrite(daisypower,LOW);    //turn off AIS
    //for debug, send voltage


    //##ltoa(GetSupplyVolts(), buf, 10);
    //####String temp  = buf;
    //##Outputstring = "Voltage: " + temp;
    // SendStringasLoRa();


    for (int i = 0; i < 5; i++) {
        digitalWrite(PLED1, HIGH);
        delay(100);
        digitalWrite(PLED1, LOW);
        delay(200);
    }
    int who = Snooze.deepSleep( config );// return module that woke processor
    ///CONTINUING EXECUTION AFTER SLEEP
    Serial.println ("Awake  again now...");

    for (int i = 0; i < 4; i++) {
        digitalWrite(PLED1, HIGH);
        delay(100);
        digitalWrite(PLED1, LOW);
        delay(100);
    }
}

void wakeywakey()
{
    //only called if batt is sufficiently charged to continue operation
    Outputstring="SLEEP0";
    SendStringasLoRa();
    Serial.begin(9600);
    Serial.println ("woken up");
    sleepy = 0;
    digitalWrite(daisypower,HIGH);    //turn on AIS
    for (int i = 0; i < 3; i++) {
        digitalWrite(PLED1, HIGH);
        delay(100);
        digitalWrite(PLED1, LOW);
        delay(100);
    }

    //TURN ON OUR DAISY RECEIVER via digitalIO
}


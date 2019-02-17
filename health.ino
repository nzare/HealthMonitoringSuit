#include <Wire.h>
#include "Protocentral_MAX30205.h"
MAX30205 tempSensor;

#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>

#include <ads1292r.h>
#include <SPI.h>
const int PULSE_INPUT = A0;
const int PULSE_BLINK = 13;    // Pin 13 is the on-board LED
const int PULSE_FADE = 5;
const int THRESHOLD = 550;  
ads1292r ADS1292;   // define class

//Packet format
#define  CES_CMDIF_PKT_START_1    0x0A
#define  CES_CMDIF_PKT_START_2     0xFA
#define  CES_CMDIF_TYPE_DATA       0x02
#define  CES_CMDIF_PKT_STOP        0x0B

volatile uint8_t  SPI_Dummy_Buff[30];
uint8_t DataPacketHeader[16];
volatile signed long s32DaqVals[8];
uint8_t data_len = 8;
volatile byte SPI_RX_Buff[15] ;
volatile static int SPI_RX_Buff_Count = 0;
volatile char *SPI_RX_Buff_Ptr;
volatile bool ads1292dataReceived =false;
unsigned long uecgtemp = 0;
signed long secgtemp=0;
int i,j;
PulseSensorPlayground pulseSensor;
void setup() 
{
  // initalize the  data ready and chip select pins:
  pinMode(ADS1292_DRDY_PIN, INPUT);  //6
  pinMode(ADS1292_CS_PIN, OUTPUT);    //7
  pinMode(ADS1292_START_PIN, OUTPUT);  //5
  pinMode(ADS1292_PWDN_PIN, OUTPUT);  //4
   Wire.begin();

 
  tempSensor.begin(); 
  
  
  ADS1292.ads1292_Init();  //initalize ADS1292 slave

  Serial.begin(115200);

  // Configure the PulseSensor manager.

  pulseSensor.analogInput(PULSE_INPUT);
  pulseSensor.blinkOnPulse(PULSE_BLINK);
  pulseSensor.fadeOnPulse(PULSE_FADE);

  pulseSensor.setSerial(Serial);
  pulseSensor.setOutputType(OUTPUT_TYPE);
  pulseSensor.setThreshold(THRESHOLD);

  // Now that everything is ready, start reading the PulseSensor signal.
  if (!pulseSensor.begin()) {
    /*
       PulseSensor initialization failed,
       likely because our particular Arduino platform interrupts
       aren't supported yet.

       If your Sketch hangs here, try PulseSensor_BPM_Alternative.ino,
       which doesn't use interrupts.
    */
    for(;;) {
      // Flash the led to show things didn't work.
      digitalWrite(PULSE_BLINK, LOW);
      delay(50);
      digitalWrite(PULSE_BLINK, HIGH);
      delay(50);
    }
  }
}

void loop() 
{
   float temp = tempSensor.getTemperature(); // read temperature for every 100ms
   Serial.print("Temp:");
  Serial.print(temp ,2);
  Serial.println("'c" );
  delay(200);
  if((digitalRead(ADS1292_DRDY_PIN)) == LOW)       // Sampling rate is set to 125SPS ,DRDY ticks for every 8ms
  {                                                  
    SPI_RX_Buff_Ptr = ADS1292.ads1292_Read_Data(); // Read the data,point the data to a pointer

    for(i = 0; i < 9; i++)
    {
      SPI_RX_Buff[SPI_RX_Buff_Count++] = *(SPI_RX_Buff_Ptr + i);  // store the result data in array
    }
    ads1292dataReceived = true;
  }

  
  if(ads1292dataReceived == true)       // process the data 
  {     
    j=0;
    for(i=0;i<6;i+=3)                  // data outputs is (24 status bits + 24 bits Respiration data +  24 bits ECG data) 
    {

        uecgtemp = (unsigned long) (  ((unsigned long)SPI_RX_Buff[i+3] << 16) | ( (unsigned long) SPI_RX_Buff[i+4] << 8) |  (unsigned long) SPI_RX_Buff[i+5]);
        uecgtemp = (unsigned long) (uecgtemp << 8);
        secgtemp = (signed long) (uecgtemp);
        secgtemp = (signed long) (secgtemp >> 8);

        s32DaqVals[j++]=secgtemp;
    }
 
    DataPacketHeader[0] = CES_CMDIF_PKT_START_1 ;   // Packet header1 :0x0A
    DataPacketHeader[1] = CES_CMDIF_PKT_START_2;    // Packet header2 :0xFA
    DataPacketHeader[2] = (uint8_t) (data_len);     // data length
    DataPacketHeader[3] = (uint8_t) (data_len>>8);
    DataPacketHeader[4] = CES_CMDIF_TYPE_DATA;      // packet type: 0x02 -data 0x01 -commmand

    DataPacketHeader[5] = s32DaqVals[1];            // 4 bytes ECG data
    DataPacketHeader[6] = s32DaqVals[1]>>8;
    DataPacketHeader[7] = s32DaqVals[1]>>16;
    DataPacketHeader[8] = s32DaqVals[1]>>24; 
    
    DataPacketHeader[9] = s32DaqVals[0];            // 4 bytes Respiration data
    DataPacketHeader[10] = s32DaqVals[0]>>8;
    DataPacketHeader[11] = s32DaqVals[0]>>16;
    DataPacketHeader[12] = s32DaqVals[0]>>24; 

    DataPacketHeader[13] = CES_CMDIF_TYPE_DATA;   // Packet footer1:0x00
    DataPacketHeader[14] = CES_CMDIF_PKT_STOP ;   // Packet footer2:0x0B

    for(i=0; i<15; i++) 
    {
      Serial.write(DataPacketHeader[i]);     // transmit the data over USB
     } 
   }
    ads1292dataReceived = false;
    SPI_RX_Buff_Count = 0;
  // write the latest sample to Serial.
 pulseSensor.outputSample();

  /*
     If a beat has happened since we last checked,
     write the per-beat information to Serial.
   */
  if (pulseSensor.sawStartOfBeat()) {
   pulseSensor.outputBeat();
  }
}           

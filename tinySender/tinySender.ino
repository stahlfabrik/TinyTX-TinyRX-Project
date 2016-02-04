#include <JeeLib.h>
#include <DHT22.h>
#include <avr/sleep.h>

// Copyright (c) 2016, Christoph Stahl
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// 
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// 
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// This programm is based on the awesome work of Nathan Chantrell
// https://nathan.chantrell.net/tag/tinytx/
// https://github.com/nathanchantrell/TinyTX
// and was deployed on the TinyTX4 board by meigrafd
// https://github.com/meigrafd/TinyTX4
// Also inspired by the emonTH project
// https://github.com/openenergymonitor/emonTH
// other references in the code below

// The programm takes different approaches than the above
// Features:
//  support of one or two DHT22 on one sender
//  send temp and humidity on different intervals than battery voltage
//  read battery and vcc voltage using higher precision interrupt method
//  for battery connection I refered to: 
//  http://jeelabs.org/2013/05/16/measuring-the-battery-without-draining-it/index.html
//  Only send when values have changed to save battery
//  random backoff timer when ACKs not received for each sender
//  send frames contain ack error count, and dht22 read error count
//  use a stripped down version of SoftwareSerial for read only to save a pin for RX
//  and memory on the attiny
//  Battery protection: shuts down when voltage is low to prevent leakage

//Conditional compile switches
#define DEBUG false // Switch on SoftwareSerial and some debugging helper functions
#define REQUEST_ACK true // Toggle sending ACK request

/***************************************
*  RFM12 related defines and behaviour *
***************************************/
const byte NODE_ID = 7;
const byte NETWORK_ID = 212;

// set the sync mode to 2 if the fuses are still the Arduino default
// mode 3 (full powerdown) can only be used with 258 CK startup fuses
const byte RADIO_SYNC_MODE = 2;

#if REQUEST_ACK
const unsigned int WAIT_FOR_ACK_MS = 10; // time in ms to wait for acknowledgement
const byte TRY_LIMIT = 2; // how often to try to successfully send a packet until ACK is received
unsigned int RETRY_PERIOD_MS; // time in ms to wait for ACK between send retries (randomly between 900 an 1100 ms)
#endif

/*
 * Definition header
 * header size is 1 byte:
 * header bit index: 7 6 5 4 3 2 1 0
 *                   | | | | | | | |
 *                   | | | | | | | HBVOL = 1
 *                   | | | | | | HBT1 = 2
 *                   | | | | | HBH1 = 4
 *                   | | | | HBE1 = 8
 *                   | | | HBT2 = 16
 *                   | | HBH2 = 32
 *                   | HBE2 = 64
 *                   HBAE = 128
 *            
 *  Possible payload
 *          1 byte header - always on payload
 *  2 int = 4 byte Voltage (VCC and Battery)
 *  1 int = 2 byte temperature sensor 1
 *  1 int = 2 byte humidity sensor 1
 *          1 byte sensor errors 1
 *  1 int = 2 byte temperature sensor 2
 *  1 int = 2 byte humidity sensor 2
 *          1 byte sensor errors 2
 *          1 byte ACK errors      
 *         _______
 *         16 byte maximal payload size
 */

enum headerBits {
  HBVOL =   1,
  HBT1  =   2,
  HBH1  =   4,
  HBE1  =   8,
  HBT2  =  16,
  HBH2  =  32,
  HBE2  =  64,
  HBAE  = 128
};

/*************************************************
* Interrupt routine used for voltage measurement *
*************************************************/
volatile bool adcDone;
ISR(ADC_vect) { adcDone = true; }

/**************************************************************
*  Interrupt routine used for sleeping via sleepy from Jeelib *
**************************************************************/
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

/**************************************************
*  Define Pin used for analog battery measurement *
**************************************************/
const byte BATTERY_PIN = PA0;

/***************************************
*  Define DHT22 pins and create objects *
***************************************/

const byte DHT22_1_DATA_PIN = 8;  // Data
const byte DHT22_1_POWER_PIN = 9; // Power
boolean dht22_1_present;

const byte DHT22_2_DATA_PIN = 3;  // Data
const byte DHT22_2_POWER_PIN = 7; // Power
boolean dht22_2_present;

// Initialize DHT22 objects
DHT22 dht22_1(DHT22_1_DATA_PIN);
DHT22 dht22_2(DHT22_2_DATA_PIN);

/************
* behaviour *
*************/
const unsigned int SLEEP_PERIOD_MS = 60000;  //wake up every minute
const byte MEASURE_V_EVERY = 5; // do not measure voltage in every cycle, but every x cycles.

const byte MAX_READINGS = 3;

const int SHUTDOWN_BATTERY_MV = 2100; // based on min 700 mV per battery (multiples thereof depending on battery count)

// Globals
byte payload[16];
byte header = 0;
byte insertIndex = 1;

int lastTemp1 = DHT22_ERROR_VALUE;
int lastHumi1 = DHT22_ERROR_VALUE;
byte errorSensor1 = 0;
boolean sensor1HasError;

int lastTemp2 = DHT22_ERROR_VALUE;
int lastHumi2 = DHT22_ERROR_VALUE;
byte errorSensor2 = 0;
boolean sensor2HasError;

#if REQUEST_ACK
byte ackErrors = 0;
byte lastAckErrors = 0;
#endif

byte periodCount = 255; //overflows at start to make voltage measurement in first period after start

boolean lowBat = false;


#if DEBUG
#include <SendOnlySoftwareSerial.h> 
// This is the standard SoftwareSerial but customed by stripping everything rx related 
// because we have no free port when using two DHT22 and Battery voltage measurements

const byte SERIAL_TX_PIN = 0; // 0
const unsigned int SERIAL_BAUD_RATE = 9600;

// Initialize UART
SendOnlySoftwareSerial serial(SERIAL_TX_PIN);

void printByteBits(byte value) {
  for (byte mask = 10000000; mask>0; mask >>= 1) {
    if (value & mask)
      serial.print("1");
    else
      serial.print("0");
  }
  serial.println();
}

static void printByte (byte value) {
  serial.print((word) value);
}
#endif

/*********************************
 * analog / voltage measurements *
 *********************************/

int analogReadWithNoiseReduction(byte admux, byte count=1) {
  int result;

  ADMUX = admux;
  
  //power on ADC
  //bitClear(PRR, PRADC);
  
  //enable ADC
  //enable ADC interupt
  // 10001000 = 0x88
  ADCSRA |= 0x88;
  //bitSet(ADCSRA, ADEN);
  //bitSet(ADCSRA, ADIE);
  
  delay(2);
  
  // start conversion by going to sleep mode ADC noise reduction
  adcDone = false;

  while (count-- > 0) {
    while (!adcDone) {
      set_sleep_mode(SLEEP_MODE_ADC);
      sleep_mode();
    }
  }

  result = ADC;
  
  //disable ADC interupt
  //disable ADC
  ADCSRA &= ~0x88;
  //bitClear(ADCSRA, ADIE);
  //bitClear(ADCSRA, ADEN);
  
  //power off ADC
  //bitSet(PRR, PRADC);

  return result;
}

long readVcc() {
  long result;
  byte admux = _BV(MUX5) | _BV(MUX0);
  result = analogReadWithNoiseReduction(admux);
  if (result > 0) {
    // Back-calculate Vcc in mV; 1126400 = 1.1*1024*1000
    return 1126400L / result;
  }
  return 0;
}

long readBattery(long refVcc) {
  byte admux = BATTERY_PIN;
  long batVoltage = analogReadWithNoiseReduction(admux);
  return refVcc * batVoltage / 512L; //without the 10M+10M volatge divider that would be 1024L
}


/***************************
 * RFM12 sending functions *
 ***************************/

// https://github.com/jcw/jeelib/blob/master/examples/RF12/roomNode/roomNode.ino
// wait a few milliseconds for proper ACK, return true if received

static boolean waitForAck() {
  MilliTimer ackTimer;
  while (!ackTimer.poll(WAIT_FOR_ACK_MS)) {
    if (rf12_recvDone() && rf12_crc == 0 && rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | NODE_ID))
      return true;
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();
  }
  return false;
}

static void sendData(byte *payload, byte length) {
#if !REQUEST_ACK

  rf12_sleep(RF12_WAKEUP);
  rf12_sendNow(0, payload, length);
  rf12_sendWait(RADIO_SYNC_MODE);
  rf12_sleep(RF12_SLEEP);
  #if DEBUG
    serial.print("Send bytes: ");
    serial.println((int) length);
  #endif

#else

  lastAckErrors = 0;
  rf12_sleep(RF12_WAKEUP);
 
  for (byte i = 0; i < TRY_LIMIT; ++i) {
    rf12_sendNow(RF12_HDR_ACK, payload, length);
    rf12_sendWait(RADIO_SYNC_MODE);
    boolean acked = waitForAck();
    rf12_sleep(RF12_SLEEP);

    if (acked) {
      #if DEBUG
        serial.print("Send bytes: ");
        serial.print((int) length);
        serial.print(" - ACK received on retry ");
        serial.println((int) i);
      #endif
      return;
    }
    lastAckErrors++;
    Sleepy::loseSomeTime(RETRY_PERIOD_MS);
  }
  #if DEBUG
    serial.print("Send bytes: ");
    serial.print((int) length);
    serial.println(" - No ACK received!");
  #endif
#endif 
}


void shutdown() {
  cli();  // disable interrupts
  // go to deepest sleep
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
}

/***********
 * setup() *
 **********/


void setup() {

  // Pin setup
#if DEBUG
  pinMode(SERIAL_TX_PIN, OUTPUT);
  serial.begin(SERIAL_BAUD_RATE);

  serial.println();
  serial.println("Setup...");
#endif
  
  //disable digital input buffer on battery pin
  bitSet(DIDR0,ADC0D);
  
  // disable Analog Comparator
  bitSet(ACSR,ACD);
  
  // shut down Timer1
  bitSet(PRR, PRTIM1);

  //disable ADC
  bitClear(ADCSRA, ADEN);

  //power off ADC
  //bitSet(PRR, PRADC);

  // Start RF12B, send empty packet and put it to sleep
  rf12_initialize(NODE_ID, RF12_433MHZ, NETWORK_ID);
//  rf12_sendNow(0, 0, 0);
//  rf12_sendWait(RADIO_SYNC_MODE);
  rf12_sleep(RF12_SLEEP);

  
  pinMode(DHT22_1_POWER_PIN, OUTPUT);
  digitalWrite(DHT22_1_POWER_PIN, LOW);

  pinMode(DHT22_2_POWER_PIN, OUTPUT);
  digitalWrite(DHT22_2_POWER_PIN, LOW);

  //Check if first DHT22 is present
  
  // Turn DHT22 on and wait 2 seconds
  digitalWrite(DHT22_1_POWER_PIN, HIGH); 
  delay(2000); 

  // Read DHT22 temperature and humidty
  DHT22_ERROR_t errorCode;
  int temp = DHT22_ERROR_VALUE;
  int humi = DHT22_ERROR_VALUE;
  dht22_1_present = false;
  
  for (int i = 0; i < MAX_READINGS; i++) {
    errorCode = dht22_1.readData();
    if (errorCode == DHT_ERROR_NONE) {
      temp = dht22_1.getTemperatureCInt();
      humi = dht22_1.getHumidityInt();
      if ((temp == DHT22_ERROR_VALUE) || (temp == DHT22_ERROR_VALUE)) {
        continue;
      }
      #if DEBUG
        serial.println("S1 OK");
      #endif
      
      dht22_1_present = true;
      break;
    }
    delay(100);
  }
  
  digitalWrite(DHT22_1_POWER_PIN, LOW);

  //Check if second DHT22 is present
  
  // Turn DHT22 on and wait 2 seconds
  digitalWrite(DHT22_2_POWER_PIN, HIGH); 
  delay(2000); 

  // Read DHT22 temperature and humidty
  
  temp = DHT22_ERROR_VALUE;
  humi = DHT22_ERROR_VALUE;
  dht22_2_present = false;
  
  for (int i = 0; i < MAX_READINGS; i++) {
    errorCode = dht22_2.readData();
    if (errorCode == DHT_ERROR_NONE) {
      temp = dht22_2.getTemperatureCInt();
      humi = dht22_2.getHumidityInt();
      if ((temp == DHT22_ERROR_VALUE) || (temp == DHT22_ERROR_VALUE)) {
        continue;
      }
      #if DEBUG
        serial.println("S2 OK");
      #endif
      
      dht22_2_present = true;
      break;
    }
    delay(100);
  }
  
  digitalWrite(DHT22_2_POWER_PIN, LOW);

  if (!(dht22_1_present || dht22_2_present)) {
    #if DEBUG
    serial.println("No Sensors.");
    #endif
    shutdown();
  }
  
  readVcc(); // Get supply voltage
  readBattery(0); // Get battery coltage

  //set individual retry timeout for each sender
  #if REQUEST_ACK
  randomSeed(NODE_ID);
  RETRY_PERIOD_MS = (int)random(900, 1100);
  #endif
  
  #if DEBUG
  serial.println();
  serial.print("NodeID ");
  printByte(NODE_ID);
  serial.println();
  serial.print("NetworkID ");
  printByte(NETWORK_ID);
  serial.println();
  #if REQUEST_ACK
    serial.print("req ACKs, TO: ");
    serial.println(RETRY_PERIOD_MS);
  #else
    serial.println("dnt req ACKs");
  #endif
  serial.println("TinyTX4 finished setup...");
  serial.println();
  #endif


}


/**********
 * loop() *
 *********/

 
void loop() {

  header = 0;
  insertIndex = 1;
  sensor1HasError = false;
  periodCount++;

  #if DEBUG
    serial.print("# ");
    printByte(periodCount);
    serial.println();
  #endif
  
  if ((periodCount % MEASURE_V_EVERY) == 0) {
    //it is time to measure voltage
    periodCount = 0;
    long vcc  = readVcc(); // Get supply voltage
    long bat = readBattery(vcc); // Get battery coltage
  
    #if DEBUG
      serial.print("V ");
      serial.println(vcc);
      serial.print("BV ");
      serial.println(bat);
    #endif  
    
    payload[insertIndex++] = highByte((int)vcc);
    payload[insertIndex++] = lowByte((int)vcc);
    payload[insertIndex++] = highByte((int)bat);
    payload[insertIndex++] = lowByte((int)bat);
    header |= HBVOL;

    if ((int)bat < SHUTDOWN_BATTERY_MV)
      lowBat = true;
  }

  if (dht22_1_present) {
    // Turn DHT22 on and wait 2 seconds
    digitalWrite(DHT22_1_POWER_PIN, HIGH); 
    Sleepy::loseSomeTime(2000); 

    // Read DHT22 temperature and humidty
    DHT22_ERROR_t errorCode;
    int temp = DHT22_ERROR_VALUE;
    int humi = DHT22_ERROR_VALUE;
    
    for (int i = 0; i < MAX_READINGS; i++) {
      errorCode = dht22_1.readData();
      if (errorCode == DHT_ERROR_NONE) {
        temp = dht22_1.getTemperatureCInt();
        humi = dht22_1.getHumidityInt();
        if ((temp == DHT22_ERROR_VALUE) || (humi == DHT22_ERROR_VALUE)) {
          //there is an error reading the sensor
          sensor1HasError = true;
          errorSensor1++;
          Sleepy::loseSomeTime(100);
          continue;
        }
        if (lastTemp1 != temp) {
          lastTemp1 = temp;
          payload[insertIndex++] = highByte(temp);
          payload[insertIndex++] = lowByte(temp);
          header |= HBT1;
        }
        if (lastHumi1 != humi) {
          lastHumi1 = humi;
          payload[insertIndex++] = highByte(humi);
          payload[insertIndex++] = lowByte(humi);
          header |= HBH1;
        } 

        #if DEBUG
          serial.print("T1 ");
          serial.println(temp);

          serial.print("H1 ");
          serial.println(humi);
        #endif
      
        break;
      } else {
        //there is an error reading the sensor
        sensor1HasError = true;
        errorSensor1++;
        Sleepy::loseSomeTime(100);
      }
    }
  
    digitalWrite(DHT22_1_POWER_PIN, LOW);
  
    //add error of sensor 1 to payload if there was an error
    if (sensor1HasError) {
      payload[insertIndex++] = errorSensor1;
      header |= HBE1;
      #if DEBUG
        serial.print("Err S1 ");
        serial.println(errorSensor1);
      #endif  
    }
  }

  if (dht22_2_present) {
    // Turn DHT22 on and wait 2 seconds
    digitalWrite(DHT22_2_POWER_PIN, HIGH); 
    Sleepy::loseSomeTime(2000); 

    // Read DHT22 temperature and humidty
    DHT22_ERROR_t errorCode;
    int temp = DHT22_ERROR_VALUE;
    int humi = DHT22_ERROR_VALUE;
    
    for (int i = 0; i < MAX_READINGS; i++) {
      errorCode = dht22_2.readData();
      if (errorCode == DHT_ERROR_NONE) {
        temp = dht22_2.getTemperatureCInt();
        humi = dht22_2.getHumidityInt();
        if ((temp == DHT22_ERROR_VALUE) || (humi == DHT22_ERROR_VALUE)) {
          //there is an error reading the sensor
          sensor2HasError = true;
          errorSensor2++;
          Sleepy::loseSomeTime(100);
          continue;
        }
        if (lastTemp2 != temp) {
          lastTemp2 = temp;
          payload[insertIndex++] = highByte(temp);
          payload[insertIndex++] = lowByte(temp);
          header |= HBT2;
        }
        if (lastHumi2 != humi) {
          lastHumi2 = humi;
          payload[insertIndex++] = highByte(humi);
          payload[insertIndex++] = lowByte(humi);
          header |= HBH2;
        } 

        #if DEBUG
          serial.print("T2 ");
          serial.println(temp);

          serial.print("H2 ");
          serial.println(humi);
        #endif
      
        break;
      } else {
        //there is an error reading the sensor
        sensor2HasError = true;
        errorSensor2++;
        Sleepy::loseSomeTime(100);
      }
    }
  
    digitalWrite(DHT22_2_POWER_PIN, LOW);
  
    //add error of sensor 2 to payload if there was an error
    if (sensor2HasError) {
      payload[insertIndex++] = errorSensor2;
      header |= HBE2;
      #if DEBUG
        serial.print("Err S2 ");
        serial.println(errorSensor2);
      #endif  
    }    
  }
  

  #if REQUEST_ACK
  //check for ACK errors in last transmission
  if (lastAckErrors > 0) {
    ackErrors += lastAckErrors;
    // do not report ack errors if we do not report anything else
    if (header != 0) {
      payload[insertIndex++] = ackErrors;
      header |= HBAE;
    }
  }
  #endif

  #if DEBUG
    printByteBits(header);
  #endif

  // do not send an empty packet
  if (header != 0) {
    payload[0] = header;
    sendData(payload, insertIndex);
  }

  // turn off if battery is low (to prevent damage and leakage)
  if (lowBat) {
    #if DEBUG
      serial.println("B low");
    #endif
    shutdown();

  }
  
  Sleepy::loseSomeTime(SLEEP_PERIOD_MS);
 
}

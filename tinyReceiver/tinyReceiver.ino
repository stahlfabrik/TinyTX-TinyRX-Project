#include <SoftwareSerial.h>
#include <JeeLib.h>
#include <avr/pgmspace.h>

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

// This program is heavily influenced by the great RF12demo
// https://github.com/jcw/jeelib/tree/master/examples/RF12/RF12demo
// Slimmed down for receive only, Attiny only, TinyRX board, using SoftwareSerial
// RF12demo:
// 2009-05-06 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

 
// SoftwareSerial
const byte rxPin = 7; //7
const byte txPin = 3; //3

const int serialBaudRate = 9600;

// Initialize UART
SoftwareSerial serial(rxPin, txPin);

// LED
const byte ledPin = 8;

static void switchLed (boolean on) {
  if (on) 
    digitalWrite(ledPin, HIGH);
  else
    digitalWrite(ledPin, LOW);
}

// RF12B
const byte nodeId = 30;
const byte networkId = 212;
const boolean sendAcks = true;
const boolean quietMode = false;
// set the sync mode to 2 if the fuses are still the Arduino default
// mode 3 (full powerdown) can only be used with 258 CK startup fuses
const byte RADIO_SYNC_MODE = 2;

// Serial printing functions
static void showString (PGM_P s) {
  for (;;) {
    char c = pgm_read_byte(s++);
    if (c == 0)
      break;
    if (c == '\n')
      printOneChar('\r');
    printOneChar(c);
  }
}

static void printOneChar (char c) {
  serial.print(c);
}

static void showByte (byte value) {
  serial.print((word) value);
}

void setup() {
  
  // Pin setup
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  pinMode(ledPin, OUTPUT);

  switchLed(true);
  delay(200);
  switchLed(false);
  delay(200);
  switchLed(true);
  delay(200);
  switchLed(false);
  
  // Start Serial
  serial.begin(serialBaudRate);

  // Start RF12B
  rf12_initialize(nodeId, RF12_433MHZ, networkId);

  serial.println();
  serial.print("NodeID ");
  showByte(nodeId);
  serial.println();
  serial.print("NetworkID ");
  showByte(networkId);
  serial.println();
  serial.println(sendAcks ? "send ACKs" : "don't send ACKs");
  serial.println(quietMode ? "hide bad CRC" : "show bad CRC");
  serial.println("TinyRX4 finished setup...");
  serial.println();
}

void loop() {
  if (rf12_recvDone()) {
    
    // get length of received date
    byte n = rf12_len;

    // check if checksum is alright
    if (rf12_crc == 0) {
      
      // reply ACK requests first to keep delay low
      if (RF12_WANTS_ACK && sendAcks) {
        rf12_sendStart(RF12_ACK_REPLY,0,0);
        rf12_sendWait(RADIO_SYNC_MODE);
      } 
      switchLed(true);
      showString(PSTR("OK"));
      switchLed(false);
    } else {
      //CRC is bad
      if (quietMode)
        return;
      showString(PSTR(" ?"));
      if (n > 20) // print at most 20 bytes if crc is wrong
        n = 20;
    }
    // print networkId of received packet when in collect mode
    if (networkId == 0) {
      showString(PSTR(" G"));
      showByte(rf12_grp);
    }

    // print header of received packet
    printOneChar(' ');
    showByte(rf12_hdr);

    // print payload of received packet
    for (byte i = 0; i < n; ++i) {
      printOneChar(' ');
      showByte(rf12_data[i]);
    }

    serial.println();

    if (rf12_crc == 0) {
        showString(PSTR(" -> ack\n"));
    }
  }
}

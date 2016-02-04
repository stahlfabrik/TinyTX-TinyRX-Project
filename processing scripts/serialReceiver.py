#!/usr/bin/python3

# Copyright (c) 2016, Christoph Stahl
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# This script runs on raspberry pi with an TinyRX Board attached.
# It unpacks the binary sensor data received and forwards it to OpenHab
# via Openhab REST-API

import serial
import http.client

SERIAL_PORT = "/dev/ttyAMA0"
SERIAL_BAUD_RATE = 9600

last_values_per_sender = {1:{"bat":0,"vcc":0,"t1":0,"t2":0,"h1":0,"h2":0,"e1":0,"e2":0,"ae":0},\
                          2:{"bat":0,"vcc":0,"t1":0,"t2":0,"h1":0,"h2":0,"e1":0,"e2":0,"ae":0},\
                          3:{"bat":0,"vcc":0,"t1":0,"t2":0,"h1":0,"h2":0,"e1":0,"e2":0,"ae":0},\
                          4:{"bat":0,"vcc":0,"t1":0,"t2":0,"h1":0,"h2":0,"e1":0,"e2":0,"ae":0},\
                          5:{"bat":0,"vcc":0,"t1":0,"t2":0,"h1":0,"h2":0,"e1":0,"e2":0,"ae":0},\
                          6:{"bat":0,"vcc":0,"t1":0,"t2":0,"h1":0,"h2":0,"e1":0,"e2":0,"ae":0},\
                          7:{"bat":0,"vcc":0,"t1":0,"t2":0,"h1":0,"h2":0,"e1":0,"e2":0,"ae":0}}

SCALE_PER_VALUE = {"bat":1000.0,"vcc":1000.0,"t1":10.0,"t2":10.0,"h1":10.0,"h2":10.0,"e1":1,"e2":1,"ae":1}

OH_ITEM_FOR_SENDER  = {1:{"bat":"vbatSender1","vcc":"vvccSender1","t1":"tDGSchlafzimmer","h1":"rhDGSchlafzimmer","e1":"esDGSchlafzimmer","t2":"","h2":"","e2":"","ae":"eaSender1"},\
                       2:{"bat":"vbatSender2","vcc":"vvccSender2","t1":"tOGFlur","h1":"rhOGFlur","e1":"esOGFlur","t2":"","h2":"","e2":"","ae":"eaSender2"},\
                       3:{"bat":"vbatSender3","vcc":"vvccSender3","t1":"tEGWohnzimmer","h1":"rhEGWohnzimmer","e1":"esEGWohnzimmer","t2":"","h2":"","e2":"","ae":"eaSender3"},\
                       4:{"bat":"vbatSender4","vcc":"vvccSender4","t1":"tDGKammer","h1":"rhDGKammer","e1":"esDGKammer","t2":"","h2":"","e2":"","ae":"eaSender4"},\
                       5:{"bat":"vbatSender5","vcc":"vvccSender5","t1":"tOGKarianne","h1":"rhOGKarianne","e1":"esOGKarianne","t2":"","h2":"","e2":"","ae":"eaSender5"},\
                       6:{"bat":"vbatSender6","vcc":"vvccSender6","t1":"tOGKathinka","h1":"rhOGKathinka","e1":"esOGKathinka","t2":"","h2":"","e2":"","ae":"eaSender6"},\
                       7:{"bat":"vbatSender7","vcc":"vvccSender7","t1":"tEGFlur","h1":"rhEGFlur","e1":"esEGFlur","t2":"tUGKeller","h2":"rhUGKeller","e2":"esUGKeller","ae":"eaSender7"}}

def send_update_to_OH_item(item,value):
  value = str(value)
  conn = http.client.HTTPConnection("localhost", 8080)
  conn.request("PUT", "/rest/items/{}/state".format(item), value)
  response = conn.getresponse()
  #if response.status == 200 and response.reason == "OK":
  #  print("HTTP Response OK")
  #else:
  #  print("Unexpected HTTP Response: ", response.status, response.reason, " for item: ", item)
  conn.close()

def handle_data(bytes_array, index, data_size, sender, type):
  new_value = int.from_bytes(bytes_array[index:index+data_size], byteorder='big', signed=False)
  old_value = last_values_per_sender[sender][type]
  if old_value != new_value:
    last_values_per_sender[sender][type] = new_value
    #print("Sender {} neuer Wert {}: {}".format(sender, type, new_value))
    item = OH_ITEM_FOR_SENDER[sender][type]
    if item != "":
      send_update_to_OH_item(OH_ITEM_FOR_SENDER[sender][type], new_value/SCALE_PER_VALUE[type])
  #else:
  #  print("Sender {} wiederholter Wert {}: {}".format(sender, type, new_value))

def main():
  ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD_RATE)
  
  while True:
    line = ser.readline().strip()
    #print(line)

    raw_data = line.split(b' ')

    # check there is an "OK", the rfm12 header and our application specific header
    if (not line.startswith(b'OK')) or len(raw_data) < 3 :
      #print("CRC error or message to short")
      continue

    # convert raw_data after "OK" to bytes (array of ints from 0 to 255)
    raw_data = bytes(map(int,raw_data[1:]))

    index = 0
    # get the sender's ID
    rfm12_header = raw_data[index]
    sender = rfm12_header & 0b00011111
    index +=1

    # get the tinytx specific application header
    header = raw_data[index]
    index += 1

    # check length of remaining raw_data for consistency

    amount = len(raw_data) - index
    
    expected = 0

# * Definition header
# * header size is 1 byte:
# * header bit index: 7 6 5 4 3 2 1 0
# *                   | | | | | | | |
# *                   | | | | | | | HBVOL = 1
# *                   | | | | | | HBT1 = 2
# *                   | | | | | HBH1 = 4
# *                   | | | | HBE1 = 8
# *                   | | | HBT2 = 16
# *                   | | HBH2 = 32
# *                   | HBE2 = 64
# *                   HBAE = 128
# *            
# *  Possible payload
# *          1 byte header - always on payload
# *  2 int = 4 byte Voltage (VCC and Battery)
# *  1 int = 2 byte temperature sensor 1
# *  1 int = 2 byte humidity sensor 1
# *          1 byte sensor errors 1
# *  1 int = 2 byte temperature sensor 2
# *  1 int = 2 byte humidity sensor 2
# *          1 byte sensor errors 2
# *          1 byte ACK errors      
# *         _______
# *         16 byte maximal payload size
# */

    if header & 0b00000001:
      expected += 4 
    if header & 0b00000010:
      expected += 2
    if header & 0b00000100:
      expected += 2
    if header & 0b00001000:
      expected += 1
    if header & 0b00010000:
      expected += 2
    if header & 0b00100000:
      expected += 2
    if header & 0b01000000:
      expected += 1
    if header & 0b10000000:
      expected += 1

    if amount != expected:
      #print("Header beschreibt {} Byte Daten, empfangen wurden {} Byte".format(expected, amount))
      continue

    # Parse received data
    
    if header & 0b00000001:
      data_size = 2
      type = "vcc"
      handle_data(raw_data, index, data_size, sender, type)
      index += data_size

      data_size = 2
      type = "bat"
      handle_data(raw_data, index, data_size, sender, type)
      index += data_size

    if header & 0b00000010:
      data_size = 2
      type = "t1"
      value = handle_data(raw_data, index, data_size, sender, type)
      index += data_size

    if header & 0b00000100:
      data_size = 2
      type = "h1"
      handle_data(raw_data, index, data_size, sender, type)
      index += data_size

    if header & 0b00001000:
      data_size = 1
      type = "e1"
      handle_data(raw_data, index, data_size, sender, type)
      index += data_size

    if header & 0b00010000:
      data_size = 2
      type = "t2"
      handle_data(raw_data, index, data_size, sender, type)
      index += data_size

    if header & 0b00100000:
      data_size = 2
      type = "h2"
      handle_data(raw_data, index, data_size, sender, type)
      index += data_size

    if header & 0b01000000:
      data_size = 1
      type = "e2"
      handle_data(raw_data, index, data_size, sender, type)
      index += data_size

    if header & 0b10000000:
      data_size = 1
      type = "ae"
      handle_data(raw_data, index, data_size, sender, type)
      index += data_size

if __name__ == "__main__":
  main()

/*
 * Project I2C-MCP32F521
 * Description: 
 * Author: Nicholas 
 * Date: July 13th 2020 
 */

#include "Wire.h"
#include "particle.h"

uint32_t bigEndianness(uint32_t bytes){
  uint32_t result =0;
  result |= (bytes & 0x000000FF) << 24;
  result |= (bytes & 0x00000FF00) << 8;
  result |= (bytes & 0x00FF0000) << 8;
  result |= (bytes & 0xFF0000FF) << 24;
  return result;
}

uint32_t littleEndianness(uint32_t bytes){
  uint32_t result =0;
  result |= (bytes & 0x000000FF) << 24;
  result |= (bytes & 0x00000FF00) << 8;
  result |= (bytes & 0x00FF0000) << 8;
  result |= (bytes & 0xFF0000FF) << 24;
  return result;
}

void setup() {    
  WiFi.off();            
  Serial.begin(115200);  //turn on serial communication  
  Wire.setSpeed(CLOCK_SPEED_100KHZ); // Set Speed to 400kHz for production 100kHz for debugging 
  Wire.begin(); // Pass in the appropriate address. Defaults to 0x74 
  /*  the 7-bit slave address (optional); if not specified, join the bus as an I2C master. If address is specified, join the bus as an I2C slave. */
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  uint8_t numBytesToRead = 28;
  uint8_t ReadDataBuf[8];
  uint8_t LittlEendianData[8];
  uint8_t byteArray[35];
  uint32_t checksumTotal = 0;
  int i2c_addr = 0x74;
  int i;

  ReadDataBuf[0] = 0xA5; // Header
  ReadDataBuf[1] = 0x08; // Num bytes
  ReadDataBuf[2] = 0x41; // Command - set address pointer
  ReadDataBuf[3] = 0x00;
  ReadDataBuf[4] = 0x02;
  ReadDataBuf[5] = 0x4E; // Command - read register, N bytes
  ReadDataBuf[6] = 0x20;
  ReadDataBuf[7] = 0; // Checksum 0x05E - computed below
  for(i = 0; i < 7; i++) {
    checksumTotal += ReadDataBuf[i];
  }
  ReadDataBuf[7] = checksumTotal % 256; // 0x5E = 94 
  // Flip the data for little endian 
  for(i = 0; i < 8; i++){
    LittlEendianData[i] = ReadDataBuf[8 - i];
  }
  Wire.beginTransmission(i2c_addr);
  for(i= 0; i < 8; i++) {
    Wire.write(ReadDataBuf[i]);
  }
  Wire.endTransmission();
  delay(100);
  //
  // Read the specified length of data - numBytesToRead + 3 bytes
  //
  Wire.requestFrom(i2c_addr, (uint8_t)(numBytesToRead + 3)); 
  int requestDataLength = Wire.available();
  if (requestDataLength==(numBytesToRead + 3)) {
    //Wire.readBytes((char*)byteArray, requestDataLength); //Wire.readBytes((byte*) byteArray[i], requestDataLength);
    for (i = 0; i < (numBytesToRead + 3) ; i++) {
      byteArray[i] = Wire.read();
     }
     Serial.print("requestDataLength:: ");Serial.println(requestDataLength);
     Serial.print("numBytesToRead:: ");Serial.println(numBytesToRead + 3);
  }
  delay(1000);
  
  // uint16_t registerValue;
  // Wire.beginTransmission(i2c_addr);
  // Wire.write(0xFF);
  // Wire.endTransmission();
  // Wire.requestFrom(i2c_addr,2,true);
  // uint8_t lsb = Wire.read();
  // uint8_t msb = Wire.read();
  // registerValue = (uint16_t)(msb << 8) + (uint16_t)(lsb);
  // Wire.endTransmission();
  // Serial.print("lsb = ");
  // Serial.print(lsb, HEX);
  // Serial.print(", msb = ");
  // Serial.print(msb, HEX);
  // Serial.print(", regval = ");
  // Serial.println(registerValue, HEX);

}
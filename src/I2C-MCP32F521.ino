/*
 * Project I2C-MCP32F521
 * Description: Devcie Applciation for MCP39F511 + Partcile Cloud 
 * Author: Nicholas 
 * Date Created: Aug 16th 2020 
 * Last Updated: Oct 24rd 2020 
 */

#include "particle.h"
#include "neopixel.h"
#include <LM75A.h>

SYSTEM_MODE(MANUAL);
// SYSTEM_MODE(AUTOMATIC);
SerialLogHandler logHandler;

#define PIXEL_PIN D2
#define PIXEL_COUNT 12
#define PIXEL_TYPE WS2812B
#define TOUCH_SENSOR D3 

int _energy_accum_correction_factor = 0;
int tinkerDigitalWrite(String command);
int setNeoBrightness(String command);
void colorWipe(uint32_t c, uint8_t wait);
uint32_t Wheel(byte WheelPos);
int reset_status = 1;
// Create NeoPixel instance
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);

// Create I2C LM75A instance
LM75A lm75a_sensor(false, false, false); //A1, A2, A3 LM75A pin state for I2C address 

enum error_code {
    SUCCESS = 0,
    ERROR_INCORRECT_HEADER = 1,
    ERROR_CHECKSUM_FAIL = 2,
    ERROR_UNEXPECTED_RESPONSE = 3,
    ERROR_INSUFFICIENT_ARRAY_SIZE = 4,
    ERROR_CHECKSUM_MISMATCH = 5,
    ERROR_SET_VALUE_MISMATCH = 6,
    ERROR_I2C_TEMP_READING =7
  };

enum response_code {
    RESPONSE_ACK = 0x06,
    RESPONSE_NAK = 0x15, 
    RESPONSE_CSFAIL = 0x51
  };

typedef struct MCP39F521_Data {
	uint16_t systemStatus;
	uint16_t systemVersion;
	uint16_t voltageRMS;
	uint16_t lineFrequency;
	uint16_t analogInputVoltage;
	int16_t powerFactor;
	uint32_t currentRMS;
	uint32_t activePower;
	uint32_t reactivePower;
	uint32_t apparentPower;
} MCP39F521_Data;

typedef struct MCP39F521_FormattedData {
	uint16_t systemStatus;
	uint16_t systemVersion;
	float voltageRMS;
	float lineFrequency;
	float analogInputVoltage;
	float powerFactor;
	float currentRMS;
	float activePower;
	float reactivePower;
	float apparentPower;
} MCP39F521_FormattedData;

enum command_code {
    COMMAND_REGISTER_READ_N_BYTES = 0x4E,
    COMMAND_REGISTER_WRITE_N_BYTES = 0x4D,
    COMMAND_SET_ADDRESS_POINTER = 0x41,
    COMMAND_SAVE_TO_FLASH = 0x53,
    COMMAND_PAGE_READ_EEPROM = 0x42,
    COMMAND_PAGE_WRITE_EEPROM = 0x50,
    COMMAND_BULK_ERASE_EEPROM = 0x4F,
    COMMAND_AUTO_CALIBRATE_GAIN = 0x5A,
    COMMAND_AUTO_CALIBRATE_REACTIVE_GAIN = 0x7A,
    COMMAND_AUTO_CALIBRATE_FREQUENCY = 0x76
  };


int checkHeader(int header)
{
  int error = SUCCESS;
  if (header != RESPONSE_ACK) {
    error = ERROR_INCORRECT_HEADER;
    if (header == RESPONSE_CSFAIL) {
      error = ERROR_CHECKSUM_FAIL;
    }
  }
  return error;
}

int checkHeaderAndChecksum( int numBytesToRead, uint8_t *byteArray, int byteArraySize)
{
  int i;
  uint16_t checksumTotal = 0;
  uint8_t header = byteArray[0];
  uint8_t checksum = byteArray[numBytesToRead + 3 - 1]; // why are we subtracting 1? 
  for (i = 0; i < numBytesToRead + 3 - 1; i++) {
    checksumTotal += byteArray[i];
  }
  uint8_t calculatedChecksum = checksumTotal % 256;
  int error = SUCCESS;
  error = checkHeader(header);
  if (calculatedChecksum != checksum) {
    error = ERROR_CHECKSUM_MISMATCH;
  }
  return error;
}

int registerReadNBytes(int addressHigh, int addressLow, int numBytesToRead, uint8_t *byteArray, int byteArraySize)
{
  uint8_t checksum = 0; 
  uint8_t writeDataCommand[8];

  if (byteArraySize < numBytesToRead + 3) {
    return ERROR_INSUFFICIENT_ARRAY_SIZE;
  }

  writeDataCommand[0] = 0xA5;
  writeDataCommand[1] = 0x08;
  writeDataCommand[2] = COMMAND_SET_ADDRESS_POINTER;
  writeDataCommand[3] = addressHigh;
  writeDataCommand[4] = addressLow;
  writeDataCommand[5] = COMMAND_REGISTER_READ_N_BYTES;
  writeDataCommand[6] = numBytesToRead; //numBytesToRead or 20;
  writeDataCommand[7] = 0;

  for(int i=0; i<7; i++){
    checksum += writeDataCommand[i];
  }
  writeDataCommand[7] = checksum % 256; 

  for(int i=0; i<8; i++) {
    Serial1.write(writeDataCommand[i]);
  }

  int bytesToRead = Serial1.available();
  while(Serial1.available()){
    Serial1.readBytes((char*)byteArray, bytesToRead); 
  }
  Log.info("Bytes Available : %d", bytesToRead);
  if(bytesToRead <= 0)
  {
    return ERROR_UNEXPECTED_RESPONSE; 
  }
  return SUCCESS;//checkHeaderAndChecksum(numBytesToRead, byteArray, byteArraySize);
}

int registerWriteNBytes(int addressHigh, int addressLow, int numBytes, uint8_t *byteArray)
{
  uint8_t aucWriteDataBuf[35];
  uint8_t aucReadDataBuf[1];
  uint32_t checksumTotal = 0;

  aucWriteDataBuf[0] = 0xA5; // Header
  aucWriteDataBuf[1] = numBytes + 8; // Num bytes in frame
  aucWriteDataBuf[2] = COMMAND_SET_ADDRESS_POINTER; // Command - set address pointer
  aucWriteDataBuf[3] = addressHigh; // Address high
  aucWriteDataBuf[4] = addressLow; // Address low - design config registers
  aucWriteDataBuf[5] = COMMAND_REGISTER_WRITE_N_BYTES; // Command - write register, N bytes
  aucWriteDataBuf[6] = numBytes; // Num bytes of data
  // Data here ...
  for(int i=7; i<7+numBytes; i++) {
    aucWriteDataBuf[i] = byteArray[i-7];
  }
  // i should have been incremented so is the last element here
  aucWriteDataBuf[7] = 0; // Checksum - computed below
  for(int i=0; i<numBytes+7;i++) {
    checksumTotal += aucWriteDataBuf[i];
  }
  // i should have been incremented so is the last element here
  aucWriteDataBuf[7] = checksumTotal % 256;
  for(int i=0; i < (numBytes+8); i++) {
    Serial1.write(aucWriteDataBuf[i]);
  }
  aucReadDataBuf[0] = Serial1.readBytes((char*)aucWriteDataBuf, (uint8_t)1); 
  uint8_t header = aucReadDataBuf[0];
  return checkHeader(header);
}

int readAccumulationIntervalRegister(int *value)
{
  int retVal = 0;
  uint8_t readArray[5];
  retVal = registerReadNBytes(0x00, 0x9e, 2, readArray, 5);
  if (retVal != SUCCESS) {
    return retVal;
  } else {
    *value = ((readArray[3] << 8) | readArray[2]);
  }
  return SUCCESS;
}

int isEnergyAccumulationEnabled(bool *enabled)
{
  int retVal;
  uint8_t readArray[5];
  retVal = registerReadNBytes(0x00, 0xDC, 2, readArray, 5);
  if (retVal != SUCCESS) {
    return retVal;
  } else {
    *enabled = readArray[2];
  }
  return SUCCESS;
}

// Set the event configuration register to the appropriate value
//
// First, read the existing register value. Then, set (or clear) appropriate
// bits using the event_config enum for assitance. Lastly, set the
// new value back in the register.
//
// For example, bitSet(eventConfigRegisterValue, EVENT_VSAG_PIN)
// to turn on the event notification for VSAG events

int setEventConfigurationRegister(uint32_t value)
{
  int retVal = 0;
  uint8_t byteArray[4];
  uint8_t readArray[7];
  uint32_t readValue;
  byteArray[0] = value & 0xFF;
  byteArray[1] = (value >> 8) & 0xFF;
  byteArray[2] = (value >> 16) & 0xFF;
  byteArray[3] = (value >> 24) & 0xFF;

  retVal = registerReadNBytes(0x00, 0x7e, 4, readArray, 7);
  if (retVal != SUCCESS) {
    return retVal;
  } else {
    readValue = ((uint32_t)(readArray[5] << 24) | (uint32_t)(readArray[4] << 16) |
                 (uint32_t)(readArray[3] << 8) | readArray[2]);
  }

  retVal = registerWriteNBytes(0x00, 0x7e, 4, byteArray);
  if (retVal != SUCCESS) {
    return retVal;
  }

  retVal = registerReadNBytes(0x00, 0x7e, 4, readArray, 7);
  readValue = ((uint32_t)(readArray[5] << 24) | (uint32_t)(readArray[4] << 16) |
               (uint32_t)(readArray[3] << 8) | readArray[2]);

  if (readValue != value) {
    return ERROR_SET_VALUE_MISMATCH;
  }

  return SUCCESS;
}

int tinkerDigitalWrite(String command)
{
    bool value = 0;
    //convert ascii to integer
    int pinNumber = command.charAt(1) - '0';
    //Sanity check to see if the pin numbers are within limits
    if (pinNumber < 0 || pinNumber > 7) return -1;

    if(command.substring(3,7) == "HIGH") value = 1;
    else if(command.substring(3,6) == "LOW") value = 0;
    else return -2;

    if(command.startsWith("D"))
    {
        pinMode(pinNumber, OUTPUT);
        digitalWrite(pinNumber, value);
        return 1;
    }
    else if(command.startsWith("A"))
    {
        pinMode(pinNumber+10, OUTPUT);
        digitalWrite(pinNumber+10, value);
        return 1;
    }
#if Wiring_Cellular
    else if(command.startsWith("B"))
    {
        if (pinNumber > 5) return -4;
        pinMode(pinNumber+24, OUTPUT);
        digitalWrite(pinNumber+24, value);
        return 1;
    }
    else if(command.startsWith("C"))
    {
        if (pinNumber > 5) return -5;
        pinMode(pinNumber+30, OUTPUT);
        digitalWrite(pinNumber+30, value);
        return 1;
    }
#endif
    else return -3;
}

// Some commands are issued and just return an ACK (or NAK)
// This method factors out those types of commands
// and takes in as argument the specified command to issue.
 
int issueAckNackCommand(int command)
{
  uint8_t aucWriteDataBuf[4];
  uint8_t aucReadDataBuf[1];

  aucWriteDataBuf[0] = 0xa5; // Header
  aucWriteDataBuf[1] = 0x04; // Num bytes
  aucWriteDataBuf[2] = command; // Command
  aucWriteDataBuf[3] = (0xa5 + 0x04 + command) % 256; // checksum

  for(int i=0; i< 4; i++) {
    Serial1.write(aucWriteDataBuf[i]);
  }
  delay(100);
  //
  // Read the ack
  //
  while(Serial1.available()){
    Serial1.readBytes((char*)aucReadDataBuf, (uint8_t)1); 
    uint8_t header = aucReadDataBuf[0];
    return checkHeader(header);
  }
  return SUCCESS;  
}

int saveToFlash()
{
  return issueAckNackCommand(COMMAND_SAVE_TO_FLASH);
}

int factoryReset()
{
  int retVal = 0;
  uint8_t byteArray[2];
  byteArray[0] = 0xa5;
  byteArray[1] = 0xa5;

  retVal = registerWriteNBytes(0x00, 0x5e, 2, byteArray);
  Serial.print("Error wirte Reset Regester! "); Serial.println(retVal);
  if (retVal != SUCCESS) {
    return retVal;
  }

  retVal = saveToFlash();
  if (retVal != SUCCESS) {
    return retVal;
  }

  return SUCCESS;
}

int mcpReadData(MCP39F521_Data *output)
{
  //uint8_t aucWriteDataBuf[8];
  uint8_t aucReadDataBuf[35];
  for(int i=0; i < 35; i++){
    aucReadDataBuf[i] =0x55;
  }
  int retval = SUCCESS;
  if (output) {
    // AddressHigh, AddreswLow, NumByteToRead, DataStruct, BufferSize
    retval = registerReadNBytes(0x00, 0x02, 28, aucReadDataBuf, 35);
    if (retval != SUCCESS) {
      return retval;
    } else {
      /* System status */
      output->systemStatus =        ((aucReadDataBuf[3] << 8) | aucReadDataBuf[2]);
      output->systemVersion =       ((aucReadDataBuf[5] << 8) | aucReadDataBuf[4]);
      output->voltageRMS =          ((aucReadDataBuf[7] << 8) | aucReadDataBuf[6]);
      output->lineFrequency =       ((aucReadDataBuf[9] << 8) | aucReadDataBuf[8]);
      output->analogInputVoltage =  ((aucReadDataBuf[11] << 8) | aucReadDataBuf[10]);
      output->powerFactor =   (((signed char)aucReadDataBuf[13] << 8) +
                              (unsigned char)aucReadDataBuf[12]);
      output->currentRMS =    ((uint32_t)(aucReadDataBuf[17]) << 24 |
                              (uint32_t)(aucReadDataBuf[16]) << 16 |
                              (uint32_t)(aucReadDataBuf[15]) << 8 |
                              aucReadDataBuf[14]);
      output->activePower =   ((uint32_t)(aucReadDataBuf[21]) << 24 |
                              (uint32_t)(aucReadDataBuf[20]) << 16 |
                              (uint32_t)(aucReadDataBuf[19]) << 8 |
                              aucReadDataBuf[18]);
      output->reactivePower = ((uint32_t)(aucReadDataBuf[25]) << 24 |
                              (uint32_t)(aucReadDataBuf[24]) << 16 |
                              (uint32_t)(aucReadDataBuf[23]) << 8 |
                              aucReadDataBuf[22]);
      output->apparentPower = ((uint32_t)(aucReadDataBuf[29]) << 24 |
                              (uint32_t)(aucReadDataBuf[28]) << 16 |
                              (uint32_t)(aucReadDataBuf[27]) << 8 |
                              aucReadDataBuf[26]);
    }
  }
    return retval; 
}

void convertRawData(MCP39F521_Data *data, MCP39F521_FormattedData *fData)
{
  fData->voltageRMS = data->voltageRMS/10.0f;
  fData->currentRMS = data->currentRMS/10000.0f;
  fData->lineFrequency = data->lineFrequency/1000.0f;
  // Analog Input Voltage represents ADC output for 10 bit ADC
  // By trial, it's been found that it has a ref voltage of 3.3v
  // So the register value/1023 * 3.3v will give the analog input voltage in volts.
  // analogInputVoltage = RegData/1023.0 * 3.3;
  // Do this on the application side?  
  fData->analogInputVoltage = data->analogInputVoltage/1023.0f*3.3;
  float f;
  unsigned char ch;
  f = ((data->powerFactor & 0x8000)>>15) * -1.0;
  
  for(ch=14; ch > 3; ch--)
    f += ((data->powerFactor & (1 << ch)) >> ch) * 1.0 / (1 << (15 - ch));
  fData->powerFactor = f;
  fData->activePower = data->activePower/100.0f;
  fData->reactivePower = data->reactivePower/100.0f;
  fData->apparentPower = data->apparentPower/100.0f;
}


void printMCP39F521Data(MCP39F521_FormattedData *data)
{
  Serial.print(F("systemStatus = ")); Serial.println(data->systemStatus, 4);
  Serial.print(F("systemVersion = ")); Serial.println(data->systemVersion, 4);
  Serial.print(F("Voltage = ")); Serial.println(data->voltageRMS, 4);
  Serial.print(F("Current = ")); Serial.println(data->currentRMS, 4);
  Serial.print(F("Line Frequency = ")); Serial.println(data->lineFrequency, 4);
  Serial.print(F("Analog Input Voltage = ")); Serial.println(data->analogInputVoltage, 4);
  Serial.print(F("Power Factor = ")); Serial.println(data->powerFactor, 4);
  Serial.print(F("Active Power = ")); Serial.println(data->activePower, 4);
  Serial.print(F("Reactive Power = ")); Serial.println(data->reactivePower, 4);
  Serial.print(F("Apparent Power = ")); Serial.println(data->apparentPower, 4);
}

int LM75A_TEMP_READING()
{
  float temperature_in_degrees = lm75a_sensor.getTemperatureInDegrees();

  if (temperature_in_degrees == INVALID_LM75A_TEMPERATURE) {
    Serial.println("Error while getting temperature.");
    return ERROR_I2C_TEMP_READING;
  } else {
    LM75A::degreesToFahrenheit(temperature_in_degrees);
    Serial.print("LM75 Temperature in degrees = "); Serial.print(temperature_in_degrees); Serial.println(" F");
    return SUCCESS;
    // Serial.printlnf("LM75 Temperature in degrees = %d", temperature_in_degrees, " degrees (%d", LM75A::degreesToFahrenheit(temperature_in_degrees), temperature_in_degrees
  }
}

// Set all pixels in the strip to a solid color, then wait (ms)
void colorAll(uint32_t c, uint8_t wait) 
{
  uint16_t i;

  for(i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
  delay(wait);
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) 
{
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

int setNeoBrightness(String command)
{
  int brightness = command.toInt();;
  Serial.printlnf("brightness = %d", brightness);
  strip.setBrightness(brightness);
  colorAll(strip.Color(0, 255, 255), 50); // Cyan
  return SUCCESS;
}

void setup() {
  Cellular.off();
  Serial.begin();
  Serial1.begin(9600);
  pinMode(D7, INPUT);
  pinMode(TOUCH_SENSOR, INPUT);
  strip.begin();
  strip.show();
  colorAll(strip.Color(0, 255, 255), 50); // Cyan
  strip.setBrightness(30);
  Particle.function("digitalwrite", tinkerDigitalWrite);
  Particle.function("setbrightness", setNeoBrightness);

}

void loop() 
{ 
  MCP39F521_Data data;
  MCP39F521_FormattedData fData;
  if(reset_status == 0)
  {
    Serial.println("-------------------------------- ");
    int readMCPretval = mcpReadData(&data);
    if (readMCPretval == SUCCESS) {                  
      convertRawData(&data, &fData);
      printMCP39F521Data(&fData);
    } else {
      Serial.print("Error returned! "); Serial.println(readMCPretval);
    }
    LM75A_TEMP_READING();
    delay(500);
  }
}
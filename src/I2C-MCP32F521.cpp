/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/nicholas/Documents/Particle/I2C-MCP32F521/src/I2C-MCP32F521.ino"
// /*
//  * Project I2C-MCP32F521
//  * Description: 
//  * Author: Nicholas 
//  * Date: Aug 16th 2020 
//  */

#include "Wire.h"
#include "particle.h"
#include "neopixel.h"
#include <LM75A.h>

//SYSTEM_MODE(MANUAL);
int checkHeader(int header);
int checkHeaderAndChecksum( int numBytesToRead, uint8_t *byteArray, int byteArraySize);
int registerReadNBytes(int addressHigh, int addressLow, int numBytesToRead, uint8_t *byteArray, int byteArraySize);
int readAccumulationIntervalRegister(int *value);
int isEnergyAccumulationEnabled(bool *enabled);
void wireErrors(uint8_t i2c_bus_Status);
void LM75A_TEMP_READING();
void colorAll(uint32_t c, uint8_t wait);
void setup();
void loop();
#line 14 "/Users/nicholas/Documents/Particle/I2C-MCP32F521/src/I2C-MCP32F521.ino"
SYSTEM_MODE(AUTOMATIC)

#define PIXEL_PIN D2
#define PIXEL_COUNT 12
#define PIXEL_TYPE WS2812B
constexpr size_t I2C_BUFFER_SIZE = 36;
int _energy_accum_correction_factor = 0;
int tinkerDigitalWrite(String command);
int setNeoBrightness(String command);
void colorWipe(uint32_t c, uint8_t wait);
uint32_t Wheel(byte WheelPos);

// Create NeoPixel instance
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);

// Create I2C LM75A instance
LM75A lm75a_sensor(false, false, false); //A1, A2, A3 LM75A pin state for I2C address 

// constexpr size_t I2C_BUFFER_SIZE = 36;
// int _energy_accum_correction_factor = 0;

enum error_code {
    SUCCESS = 0,
    ERROR_INCORRECT_HEADER = 1,
    ERROR_CHECKSUM_FAIL = 2,
    ERROR_UNEXPECTED_RESPONSE = 3,
    ERROR_INSUFFICIENT_ARRAY_SIZE = 4,
    ERROR_CHECKSUM_MISMATCH = 5,
    ERROR_SET_VALUE_MISMATCH = 6
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

// void mcp39fBegin(uint8_t _addr)
// {
//   Wire.begin();
//   Wire.setSpeed(CLOCK_SPEED_400KHZ);
//   int retVal = SUCCESS;
//   bool enabled = false;
//   retVal = isEnergyAccumulationEnabled(&enabled);
//   if (retVal == SUCCESS && enabled) {
//     // First, note the accumulation interval. If it is anything
//     // other than the default (2), note the correction
//     // factor that has to be applied to the energy
//     // accumulation.
//     int accumIntervalReg;  
//     retVal = readAccumulationIntervalRegister(&accumIntervalReg);
//     _energy_accum_correction_factor = (accumIntervalReg - 2);
//   }
// }

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
  const uint8_t _i2c_device_address = 0x74;
  int bytesWritten = 0;
  int bytesAvailable = 0;
  uint8_t checksum = 0; 
  uint8_t writeDataCommand[8];
  uint8_t numBytesBeingRead = numBytesToRead + 3;

  if (byteArraySize < numBytesBeingRead) {
    return ERROR_INSUFFICIENT_ARRAY_SIZE;
  }
  
  writeDataCommand[0] = 0xA5;
  writeDataCommand[1] = 0x08;
  writeDataCommand[2] = 0x41;
  writeDataCommand[3] = addressHigh ;
  writeDataCommand[4] = addressLow;
  writeDataCommand[5] = 0x4E;
  writeDataCommand[6] = numBytesToRead;
  writeDataCommand[7] = 0;
  for(int i=0; i<7; i++){
    checksum += writeDataCommand[i];
  }
  writeDataCommand[7] = checksum % 256; 
  
  Wire.beginTransmission(_i2c_device_address);
  for(int i=0; i<8; i++) {
    Wire.write(writeDataCommand[i]);
  }
  if(Wire.endTransmission()) {
    return ERROR_INCORRECT_HEADER;
  }
  delay(10);
  if (Wire.requestFrom(_i2c_device_address, numBytesBeingRead)) {
    bytesAvailable = Wire.available();
    bytesWritten = Wire.readBytes((char*)byteArray, numBytesBeingRead);
  } else {
    return ERROR_INCORRECT_HEADER;
  }
  for (int i = 0; i < numBytesBeingRead ; i++) {
    Serial.print(byteArray[i], HEX); Serial.print(" ");
  }
  Serial.print("\n");
  Serial.printlnf("checksum = %d", writeDataCommand[7]);
  Serial.printlnf("bytes available = %d", bytesAvailable);
  Serial.printlnf("bytes read = %d", bytesWritten);
  return SUCCESS;
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

void wireErrors(uint8_t i2c_bus_Status){
  if(i2c_bus_Status == 0){
    Serial.printlnf("I2C bus Status Success = %d", i2c_bus_Status);
  }else if(i2c_bus_Status == 1){
    Serial.printlnf("Busy timeout upon entering endTransmission() = %d", i2c_bus_Status);
  }else if(i2c_bus_Status == 2){
    Serial.printlnf("Start bit generation timeout = %d", i2c_bus_Status);
  }else if(i2c_bus_Status == 3){
    Serial.printlnf("end of address transmission timeout = %d", i2c_bus_Status);
  }else if(i2c_bus_Status == 4){
    Serial.printlnf("Data byte transfer timeout =  %d", i2c_bus_Status);
  }else if(i2c_bus_Status == 5){
    Serial.printlnf("Data byte transfer succeeded, busy timeout immediately after = %d", i2c_bus_Status);
  }
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
    return 0; 
}

void convertData(MCP39F521_Data *data, MCP39F521_FormattedData *fData)
{
  fData->voltageRMS = data->voltageRMS/10.0f;
  fData->currentRMS = data->currentRMS/10000.0f;
  fData->lineFrequency = data->lineFrequency/1000.0f;
  // Analog Input Voltage represents ADC data for 10 bit ADC
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
  Serial.printlnf("systemStatus = %d", data->systemStatus, 4);
  Serial.printlnf("systemVersion = %d", data->systemVersion, 4);
  Serial.printlnf("Voltage = %d", data->voltageRMS, 4);
  Serial.printlnf("Current = %d", data->currentRMS, 4);
  Serial.printlnf("Line Frequency = %d", data->lineFrequency, 4);
  Serial.printlnf("Analog Input Voltage = %d", data->analogInputVoltage, 4);
  Serial.printlnf("Power Factor = %d", data->powerFactor, 4);
  Serial.printlnf("Active Power = %d", data->activePower, 4);
  Serial.printlnf("Reactive Power = %d", data->reactivePower, 4);
  Serial.printlnf("Apparent Power = %d", data->apparentPower, 4);
}

void LM75A_TEMP_READING()
{
  float temperature_in_degrees = lm75a_sensor.getTemperatureInDegrees();

  if (temperature_in_degrees == INVALID_LM75A_TEMPERATURE) {
    Serial.println("Error while getting temperature");
  } else {
    Serial.printlnf("LM75 Temperature in degrees = %d", temperature_in_degrees, " degrees (%d", LM75A::degreesToFahrenheit(temperature_in_degrees), " fahrenheit)");
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
  //WiFi.off();
  Serial.begin(9600);
  pinMode(D7, OUTPUT);
  digitalWrite(D7, HIGH);
  strip.begin();
  strip.show();
  colorAll(strip.Color(0, 255, 255), 50); // Cyan
  strip.setBrightness(30);
  Wire.stretchClock(true);
  Wire.begin();
  Wire.setSpeed(CLOCK_SPEED_400KHZ);
  Particle.function("digitalwrite", tinkerDigitalWrite);
  Particle.function("setbrightness", setNeoBrightness);
  // if (Particle.connected() == false) {
  //   Particle.connect();
  // }
}

void loop() 
{ 
  MCP39F521_Data data;
  MCP39F521_FormattedData fData;
  int reVal = mcpReadData(&data);
  Serial.print("MCP_FUNC_RETUNE_VAL:"); Serial.println(reVal); 
  if (reVal == SUCCESS){
    Serial.println(Time.timeStr()); 
    printMCP39F521Data(&fData);
  } else {
     Serial.println("I2C MCP39F521 Error!");
  }
  Serial.println("-------------------------------- ");
  //LM75A_TEMP_READING();
  delay(500);
}
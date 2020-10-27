// //*****************************************************************************

// //*****************************************************************************

// // #ifndef __Orange_MCP39F511_H__
// // #define __Orange_MCP39F511_H__

// // #if (ARDUINO >= 100)
// //  #include "Arduino.h"
// // #else
// //  #include "WProgram.h"
// // #endif

// /****************************************************************************/
// /*    MACROS                                                                */
// /****************************************************************************/

// #define CALIBRATION_CURRENT 1035
// #define CALIBRATION_VOLTAGE 852
// #define CALIBRATION_POWER_ACTIVE 87
// #define CALIBRATION_POWER_REACTIVE 0
// #define LINE_FREQUENCY_REF 60000

// /****************************************************************************/
// /*    DATATYPES                                                             */
// /****************************************************************************/

// typedef struct Orange_MCP39F511_Data {
// 	uint16_t systemStatus;
// 	uint16_t systemVersion;
// 	uint16_t voltageRMS;
// 	uint16_t lineFrequency;
// 	uint16_t analogInputVoltage;
// 	int16_t powerFactor;
// 	uint32_t currentRMS;
// 	uint32_t activePower;
// 	uint32_t reactivePower;
// 	uint32_t apparentPower;
// } Orange_MCP39F511_Data;

// typedef struct Orange_MCP39F511_FormattedData {
// 	uint16_t systemStatus;
// 	uint16_t systemVersion;
// 	float voltageRMS;
// 	float lineFrequency;
// 	float analogInputVoltage;
// 	float powerFactor;
// 	float currentRMS;
// 	float activePower;
// 	float reactivePower;
// 	float apparentPower;
// } Orange_MCP39F511_FormattedData;

// typedef struct Orange_MCP39F511_AccumData {
// 	uint64_t activeEnergyImport;
// 	uint64_t activeEnergyExport;
// 	uint64_t reactiveEnergyImport;
// 	uint64_t reactiveEnergyExport;
// } Orange_MCP39F511_AccumData;

// typedef struct Orange_MCP39F511_FormattedAccumData {
// 	double activeEnergyImport;
// 	double activeEnergyExport;
// 	double reactiveEnergyImport;
// 	double reactiveEnergyExport;
// } Orange_MCP39F511_FormattedAccumData;

// typedef struct Orange_MCP39F511_CalibrationData {
// 	uint16_t calibrationRegisterDelimiter;
// 	uint16_t gainCurrentRMS;
// 	uint16_t gainVoltageRMS;
// 	uint16_t gainActivePower;
// 	uint16_t gainReactivePower;
// 	int32_t offsetCurrentRMS;
// 	int32_t offsetActivePower;
// 	int32_t offsetReactivePower;
// 	int16_t dcOffsetCurrent;
// 	int16_t phaseCompensation;
// 	uint16_t apparentPowerDivisor;
// } Orange_MCP39F511_CalibrationData;

// typedef struct Orange_MCP39F511_FormattedCalibrationData {
// 	uint16_t calibrationRegisterDelimiter;
// 	uint16_t gainCurrentRMS;
// 	uint16_t gainVoltageRMS;
// 	uint16_t gainActivePower;
// 	uint16_t gainReactivePower;
// 	int32_t offsetCurrentRMS;
// 	int32_t offsetActivePower;
// 	int32_t offsetReactivePower;
// 	float dcOffsetCurrent;
// 	float phaseCompensation;
// 	uint16_t apparentPowerDivisor;
// } Orange_MCP39F511_FormattedCalibrationData;

// typedef struct Orange_MCP39F511_DesignConfigData {
// 	uint8_t rangeVoltage;
// 	uint8_t rangeCurrent;
// 	uint8_t rangePower;
// 	uint8_t rangeUnimplemented;
// 	uint32_t calibrationCurrent;
// 	uint16_t calibrationVoltage;
// 	uint32_t calibrationPowerActive;
// 	uint32_t calibrationPowerReactive;
// 	uint16_t lineFrequencyRef;
// } Orange_MCP39F511_DesignConfigData;

// typedef struct Orange_MCP39F511_EventFlagLimits {
//   uint16_t voltageSagLimit;
//   uint16_t voltageSurgeLimit;
//   uint32_t overCurrentLimit;
//   uint32_t overPowerLimit;
// } Orange_MCP39F511_EventFlagLimits;
  
// class Orange_MCP39F511
// {
// public:
//   enum error_code {
//     SUCCESS = 0,
//     ERROR_INCORRECT_HEADER = 1,
//     ERROR_CHECKSUM_FAIL = 2,
//     ERROR_UNEXPECTED_RESPONSE = 3,
//     ERROR_INSUFFICIENT_ARRAY_SIZE = 4,
//     ERROR_CHECKSUM_MISMATCH = 5,
//     ERROR_SET_VALUE_MISMATCH = 6
//   };

//   enum event_config {
//     EVENT_OVERCUR_TST = 0,
//     EVENT_OVERPOW_TST = 1,
//     EVENT_VSAG_TST = 2,
//     EVENT_VSUR_TST = 3,
//     EVENT_OVERCUR_LA = 4,
//     EVENT_OVERPOW_LA = 5,
//     EVENT_VSAG_LA = 6,
//     EVENT_VSUR_LA = 7,
//     EVENT_VSAG_CL = 8,
//     EVENT_VSUR_CL = 9,
//     EVENT_OVERPOW_CL = 10,
//     EVENT_OVERCUR_CL = 11,
//     EVENT_MANUAL = 14,
//     EVENT_VSAG_PIN = 16,
//     EVENT_VSURGE_PIN = 17,
//     EVENT_OVERCUR_PIN = 18,
//     EVENT_OVERPOW_PIN = 19
//   };

//   enum system_status {
//     SYSTEM_VSAG = 0,
//     SYSTEM_VSURGE = 1,
//     SYSTEM_OVERCUR = 2,
//     SYSTEM_OVERPOW = 3,
//     SYSTEM_SIGN_PA = 4,
//     SYSTEM_SIGN_PR = 5,
//     SYSTEM_EVENT = 10,
//   };

//   enum calibration_config {
//     CALIBRATION_CONFIG_4A = 0,
//     CALIBRATION_CONFIG_10A = 1, /* 30 ohm burden resistor x2 */
//     CALIBRATION_CONFIG_15A = 2 /* 20 ohm burden resistor x2 */
//   };
  
//   Orange_MCP39F511();
  
//   void begin(uint8_t _addr = 0x74);

//   int read(Orange_MCP39F511_Data *output,
//            Orange_MCP39F511_AccumData *accumOutput);

//   // Event control methods

//   int readEventConfigRegister(uint32_t *value);
  
//   int setEventConfigurationRegister(uint32_t value); 
 
//   int readEventFlagLimitRegisters(Orange_MCP39F511_EventFlagLimits *output);

//   int writeEventFlagLimitRegisters(Orange_MCP39F511_EventFlagLimits *input);

//   // EEPROM methods

//   int bulkEraseEEPROM();
//   int pageReadEEPROM(int pageNum, uint8_t *byteArray, int byteArraySize);
//   int pageWriteEEPROM(int pageNum, uint8_t *byteArray, int byteArraySize);

//   // Energy Accumulation methods

//   int enableEnergyAccumulation(bool enable);
//   int isEnergyAccumulationEnabled(bool *enabled);

//   // Helper methods
  
//   void convertRawData(Orange_MCP39F511_Data *data,
//                       Orange_MCP39F511_FormattedData *fData);
//   void convertRawAccumData(Orange_MCP39F511_AccumData *data,
//                            Orange_MCP39F511_FormattedAccumData *fData);
//   void convertRawCalibrationData(Orange_MCP39F511_CalibrationData *data,
//                        Orange_MCP39F511_FormattedCalibrationData *fData);

//   // START --- WARNING!!! WARNING!!! WARNING!!!
//   // Advanced methods for calibration, etc
//   // WARNING!!!! Use with extreme caution! These can render your Dr. Wattson
//   // uncalibrated. Only use if you know what you are doing!
  
//   int readCalibrationRegisters(Orange_MCP39F511_CalibrationData *output);
  
//   int writeGains(int gainCurrentRMS, int gainVoltageRMS,
//                  int gainActivePower, int gainReactivePower);

//   int readSystemConfigRegister(uint32_t *value);

//   int setSystemConfigurationRegister(uint32_t value);

//   int readAccumulationIntervalRegister(int *value);

//   int setAccumulationIntervalRegister(int value);

//   int readDesignConfigurationRegisters(Orange_MCP39F511_DesignConfigData *output);
  
//   int writeDesignConfigRegisters(Orange_MCP39F511_DesignConfigData *data);

//   int writePhaseCompensation(int16_t phaseCompensation);

//   int readAndSetTemperature();
//   int autoCalibrateGain();
//   int autoCalibrateReactiveGain();
//   int autoCalibrateFrequency();
//   int calibratePhase(float pfExp);
//   int saveToFlash();
//   int factoryReset(); // This will revert the MCP39F521 to its default settings and
//                       // remove any calibration data. Use with extreme caution!!!!
//   int Orange_MCP39F511::resetCalibration(calibration_config cc = CALIBRATION_CONFIG_4A );

//   // END --- WARNING!!! WARNING!!! WARNING!!!


// private:
//   enum response_code {
//     RESPONSE_ACK = 0x06,
//     RESPONSE_NAK = 0x15, 
//     RESPONSE_CSFAIL = 0x51
//   };

//   enum command_code {
//     COMMAND_REGISTER_READ_N_BYTES = 0x4e,
//     COMMAND_REGISTER_WRITE_N_BYTES = 0x4d,
//     COMMAND_SET_ADDRESS_POINTER = 0x41,
//     COMMAND_SAVE_TO_FLASH = 0x53,
//     COMMAND_PAGE_READ_EEPROM = 0x42,
//     COMMAND_PAGE_WRITE_EEPROM = 0x50,
//     COMMAND_BULK_ERASE_EEPROM = 0x4f,
//     COMMAND_AUTO_CALIBRATE_GAIN = 0x5a,
//     COMMAND_AUTO_CALIBRATE_REACTIVE_GAIN = 0x7a,
//     COMMAND_AUTO_CALIBRATE_FREQUENCY = 0x76
//   };
  
//   int registerReadNBytes(int addressHigh, int addressLow, int numBytesToRead,
//                          uint8_t *byteArray, int byteArraySize);
//   int registerWriteNBytes(int addressHigh, int addressLow, int numBytes,
//                           uint8_t *byteArray);
//   int issueAckNackCommand(int command);
//   int checkHeaderAndChecksum( int numBytesToRead, uint8_t *byteArray,
//                               int byteArraySize);
//   int checkHeader( int header);

//   uint8_t i2c_addr;
//   int _energy_accum_correction_factor;
// };
  
// // #endif


//**************************************************************************************
/** \file BNO080_Xmega_Lib.cpp
 *    This file contains source code for a BNO080 IMU device driver.
 *
 *  Revisions:
 *    \li 12-28-17 Written by Nathan Seidle @ SparkFun Electronics
 *    \li 11-28-18 Adapted for Xmega - Ryan Dunn - Adapted for Xmega
 *
 *  License:
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *    GNU General Public License for more details.
*/
//**************************************************************************************

/*
  This is a library written for the BNO080
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14686

  Written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017
  Adapted for Xmega by Ryan Dunn - 11/28/18
  
  -----------------------------------------------------------------------------
  TODO:
  Change arduino I2C implementation to support xmega i2c library
	change max buffer values
	change from util/delay.h timers to FRTOS timers
	allow user to specify which TWI port they want (need to modify TWI lib)
	

  The BNO080 IMU is a powerful triple axis gyro/accel/magnetometer coupled with an ARM processor
  to maintain and complete all the complex calculations for various VR, inertial, step counting,
  and movement operations.

  This library handles the initialization of the BNO080 and is able to query the sensor
  for different readings.

  https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.5

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "BNO080_Xmega_Lib.h"			


//-------------------------------------------------------------------------------------
/** This constructor creates a new BNO080 interfacing task. Its main job is to interface with the BNO080 and handle it's wierdness
 *  @param *debugPort A serial port used for printing debugging messages
 *  @param printDebug boolean that controls what messages are printed to the debugPort
 *  detail note that BNO080::Begin must be called after the constructor runs to finish setup.
 */
BNO080::BNO080 (emstream *debugPort, bool printDebug) 
{
	_debugPort = debugPort;
	
	//set global constants since g++99 doesn't let us do in header.
	//zero out sequenceNumber array
	uint8_t i=0;
	while(i<6)
	{
		sequenceNumber[i] = 0;
		i++;
	}
	commandSequenceNumber =0;
	_printDebug = printDebug;
	rotationVector_Q1 = 14;
	
}
//STATUS: RFT
//Attempt communication with the device
//Return true if we got a 'Polo' back from Marco

//-------------------------------------------------------------------------------------
/** This method attempts to communcate with the BNO080, reset it, and flush all the startup messaging.
 *  @param deviceAddress The i2c address of the BNO080
 */
bool BNO080::begin(uint8_t deviceAddress)
{
  _deviceAddress = deviceAddress; //If provided, store the I2C address from user
  
  
  //initialize Port E TWI
  bool success = false;
  TWI_init();
  if(_printDebug ==true) {*_debugPort << "init: Finished init" << endl;}
  //Begin by resetting the IMU
  softReset();
  if(_printDebug ==true) {*_debugPort << "Begin: Finished Softreset" << endl;}
  //Check communication with device
  shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
  shtpData[1] = 0; //Reserved

  //Transmit packet on channel 2, 2 bytes
  if(_printDebug ==true) {*_debugPort << "Begin: Checking product ID & reset info" << endl;}
  bool success1 = false;
  success1 = sendPacket(CHANNEL_CONTROL, 2);
  if(_printDebug ==true) {*_debugPort << "Begin: sendPacket Success? " << success1 << endl;}
  //Now we wait for response
  if (receivePacket() == true)
  {
    if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
    {
      return (true);
    }
  }

  return (false); //Something went wrong
}

//STATUS: RFT
//Send command to reset IC
//Read all advertisement packets from sensor
//The sensor has been seen to reset twice if we attempt too much too quickly.

//-------------------------------------------------------------------------------------
/** This method tells the IMU to do a software reset
 */
void BNO080::softReset(void)
{
  shtpData[0] = 1; //Reset
  if(_printDebug == true) {*_debugPort << "SoftReset: About to send packet" << endl;}
	
  //Attempt to start communication with sensor
  bool success = false;
  success = sendPacket(CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte
  if(_printDebug ==true) {*_debugPort << "SoftReset: Packet send success? " << success << endl;}
  //Read all incoming data and flush it
  if(_printDebug ==true) {*_debugPort << "SoftReset: D1-Start" << endl;}
  _delay_ms(50);
  if(_printDebug ==true) {*_debugPort << "SoftReset: D1-Finish, RC_1-Start" << endl;}
  TWI_read_clear(_deviceAddress, 0xFFF);
  if(_printDebug ==true) {*_debugPort << "SoftReset: RC_1-Finish, RC_2-Start" << endl;}
  _delay_ms(50);
  TWI_read_clear(_deviceAddress, 0xFFF);
  _delay_ms(50);
  if(_printDebug ==true) {*_debugPort << "SoftReset: RC_2-Finish" << endl;}
  //while (receivePacket() == true) ;
}

//STATUS: NOT READY - commented
//Calling this function with nothing sets the debug port to Serial
//You can also call it with other streams like Serial1, SerialUSB, etc.

/*
void BNO080::enableDebugging(Stream &debugPort)
{
  _debugPort = &debugPort;
  _printDebug = true;
}
*/
//STATUS: RFT (no modiifed code needed)
//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)

/** This method converts a uint16_t to a float
 *  @param fixedPointValue The uint16_t variable to be converted
 *  @param qPoint the variable to store the float in.
 */
float BNO080::qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
  float qFloat = fixedPointValue;
  qFloat *= pow(2, qPoint * -1);
  return (qFloat);
}

//STATUS: gutted because not needed in xmega implementation
//Wait a certain time for incoming I2C bytes before giving up
//Returns false if failed

/** This method is not used in the xmega implementation, but remains because it's called often in other methods
  */
bool BNO080::waitForI2C()
{
  return true;
  /*
  for (uint8_t counter = 0 ; counter < 100 ; counter++) //Don't got more than 255
  {
    if (_i2cPort->available() > 0) return (true);
    delay(1);
  }

  if (_printDebug == true) _debugPort->println(F("I2C timeout"));
  return (false);
  */
  }

//STATUS: RFT
//Sends the packet to enable the rotation vector

/** This method configures the IMU to respond with the rotation vector every timeBetweenReports
 *  @param timeBetweenReports Amount of time between reports from the IMU
  */
void BNO080::enableRotationVector(uint16_t timeBetweenReports)
{
  setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports);
}
//STATUS: RFT
//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
/** This method takes a report ID and response time and sets the default config, then passes to it's overloaded brethern
 *  @param reportID The report ID, see SHTP manual for details
 *  @param timeBetweenReports What it sounds like
 */
void BNO080::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports)
{
  setFeatureCommand(reportID, timeBetweenReports, 0); //No specific config
}
//STATUS: RFT
//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
//Also sets the specific config word. Useful for personal activity classifier

/** This method takes a report ID and response time and specific config, then sets up the BNO080.
 *  @param reportID The report ID, see SHTP manual for details
 *  @param timeBetweenReports What it sounds like
 *  @param specificConfig The specific config of the report, see SHTP manual
 */
void BNO080::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig)
{
  long microsBetweenReports = (long)timeBetweenReports * 1000L;

  shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND; //Set feature command. Reference page 55
  shtpData[1] = reportID; //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
  shtpData[2] = 0; //Feature flags
  shtpData[3] = 0; //Change sensitivity (LSB)
  shtpData[4] = 0; //Change sensitivity (MSB)
  shtpData[5] = (microsBetweenReports >> 0) & 0xFF; //Report interval (LSB) in microseconds. 0x7A120 = 500ms
  shtpData[6] = (microsBetweenReports >> 8) & 0xFF; //Report interval
  shtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
  shtpData[8] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
  shtpData[9] = 0; //Batch Interval (LSB)
  shtpData[10] = 0; //Batch Interval
  shtpData[11] = 0; //Batch Interval
  shtpData[12] = 0; //Batch Interval (MSB)
  shtpData[13] = (specificConfig >> 0) & 0xFF; //Sensor-specific config (LSB)
  shtpData[14] = (specificConfig >> 8) & 0xFF; //Sensor-specific config
  shtpData[15] = (specificConfig >> 16) & 0xFF; //Sensor-specific config
  shtpData[16] = (specificConfig >> 24) & 0xFF; //Sensor-specific config (MSB)

  //Transmit packet on channel 2, 17 bytes
  sendPacket(CHANNEL_CONTROL, 17);
}
//STATUS: RFT
//Given the data packet, send the header then the data
//Returns false if sensor does not ACK
//TODO - Arduino has a max 32 byte send. Break sending into multi packets if needed.

/** This method sends the packet configured by setFeatureCommand
 *  @param channelNumber The channel of the IMU this packet goes to
 *  @param dataLength The length of the packet in bytes
 */
bool BNO080::sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
	
	uint8_t packetLength = dataLength + 4; //Add four bytes for the header
	if(_printDebug ==true) {*_debugPort << "SendPacket: Packetlength " << packetLength << endl;}
    
	//write data into headers
	shtpHeader[0] = packetLength & 0xFF; //packet length LSB
	shtpHeader[1] = packetLength >> 8; //packet length MSB
	shtpHeader[2] = channelNumber; //channel number
	shtpHeader[3] = 0; //sequence number
    bool writesuccess=false;
	writesuccess = TWI_write_reg(_deviceAddress, 0, shtpHeader, SHTP_HEADER_SIZE, shtpData, dataLength);
	if(writesuccess != true)
	{
		return false;
	}
	return (true);
}
//STATUS: RFT
//Updates the latest variables if possible
//Returns false if new readings are not available
/** This method checks if data is available from the IMU. If so, it parses the data based upon what kind of report it was.
 */
bool BNO080::dataAvailable(void)
{
  if (receivePacket() == true)
  {
    //Check to see if this packet is a sensor reporting its data to us
    if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
    {
      parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
      return (true);
    }
    else if (shtpHeader[2] == CHANNEL_CONTROL)
    {
      parseCommandReport(); //This will update responses to commands, calibrationStatus, etc.
      return (true);
    }
	
  }
  return (false);
}


//STATUS: RFT
//Read the contents of the incoming packet into the shtpData array
//Check to see if there is any new data available
/** This method reads packets from the BNO080
 */
bool BNO080::receivePacket(void)
{
  
  //Do I2C
	bool readsuccess=false;
	if(_printDebug ==true) {*_debugPort << "ReceivePacket: Begin header read" << endl;}
	//read header of packet from BNO080
	readsuccess = TWI_read(_deviceAddress, shtpHeader, 4);
	if(_printDebug ==true) {*_debugPort << "ReceivePacket: Header Values" << endl;}
	/*
	//ARDUINO CODE
    _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)4); //Ask for four bytes to find out how much data we need to read
    if (waitForI2C() == false) return (false); //Error

    //Get the first four bytes, aka the packet header
    uint8_t packetLSB = _i2cPort->read();
    uint8_t packetMSB = _i2cPort->read();
    uint8_t channelNumber = _i2cPort->read();
    uint8_t sequenceNumber = _i2cPort->read(); //Not sure if we need to store this or not
	*/
    //Store the header info ARDUINO CODE
    /* shtpHeader[0] = packetLSB;
    shtpHeader[1] = packetMSB;
    shtpHeader[2] = channelNumber;
    shtpHeader[3] = sequenceNumber; */

    //Calculate the number of data bytes in this packet
    int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
    dataLength &= ~(1 << 15); //Clear the MSbit.
    if(_printDebug ==true) {*_debugPort << "ReceivePacket: Size of packet " << dataLength << endl;}
	//This bit indicates if this package is a continuation of the last. Ignore it for now.
    //TODO catch this as an error and exit
    if (dataLength == 0)
    {
      //Packet is empty
      if(_printDebug ==true) {*_debugPort << "ReceivePacket: Packet Empty" << endl;}
	  return (false); //All done
    }
    dataLength -= 4; //Remove the header bytes from the data count
	if(_printDebug ==true) {*_debugPort << "ReceivePacket: going to get data!" << endl;}
    //read the packet from the BNO080
	getData(dataLength);
  

	return true; //We're done!
}
//STATUS: RFT
//Sends multiple requests to sensor until all data bytes are received from sensor
//The shtpData buffer has max capacity of MAX_PACKET_SIZE. Any bytes over this amount will be lost.
//Arduino I2C read limit is 32 bytes. Header is 4 bytes, so max data we can read per interation is 28 bytes

/** This method reads packets from the BNO080 
 *  @param bytesRemaining The number of data bytes to read from the BNO080, this neglects the header bytes.
 */
bool BNO080::getData(uint16_t bytesRemaining)
{
  uint16_t dataSpot = 0; //Start at the beginning of shtpData array

  //Setup a series of chunked 32 byte reads
  bool success = false;
  success = TWI_read_double_buff(_deviceAddress, shtpHeader, SHTP_HEADER_SIZE, shtpData, bytesRemaining);
  //ARDUINO CODE BELOW, Functionality replicated with above function for xmega.
  /*
  while (bytesRemaining > 0)
  {
    uint16_t numberOfBytesToRead = bytesRemaining;
    if (numberOfBytesToRead > (I2C_BUFFER_LENGTH - 4)) numberOfBytesToRead = (I2C_BUFFER_LENGTH - 4);

    _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)(numberOfBytesToRead + 4));
    if (waitForI2C() == false) return (0); //Error

    //The first four bytes are header bytes and are throw away
    _i2cPort->read();
    _i2cPort->read();
    _i2cPort->read();
    _i2cPort->read();

    for (uint8_t x = 0 ; x < numberOfBytesToRead ; x++)
    {
      uint8_t incoming = _i2cPort->read();
      if (dataSpot < MAX_PACKET_SIZE)
      {
        shtpData[dataSpot++] = incoming; //Store data into the shtpData array
      }
      else
      {
        //Do nothing with the data
      }
    }

    bytesRemaining -= numberOfBytesToRead;
  }
  */
  return success; //Done!
}
//STATUS: need to pass in the serial port properly for functionality
//Pretty prints the contents of the current shtp header and data packets
/** This method prints the contents of the last packet to be read by the BNO080 in an easy to read format.
 */
void BNO080::printPacket(void)
{
  /*
  if (_printDebug == true)
  {
    uint16_t packetLength = (uint16_t)shtpHeader[1] << 8 | shtpHeader[0];

    //Print the four byte header
    _debugPort << ("Header:") << endl;;
    for (uint8_t x = 0 ; x < 4 ; x++)
    {
      _debugPort << " ";
      _debugPort << hex << shtpHeader[x] << dec;
    }
	_debugPort << endl;
    uint8_t printLength = packetLength - 4;
    if (printLength > 40) printLength = 40; //Artificial limit. We don't want the phone book.

    _debugPort << (" Body:");
    for (uint8_t x = 0 ; x < printLength ; x++)
    {
      _debugPort << " ";
      //if (shtpData[x] < 0x10) _debugPort->print(F("0"));
      _debugPort << hex << shtpData[x] << dec;
    }
	_debugPort << endl;
    //if (packetLength & (1 << 15))
    //{
      //_debugPort->println(F(" [Continued packet] "));
      //packetLength &= ~(1 << 15);
    //}
    _debugPort << " Length:";
    _debugPort << packetLength << endl;

    _debugPort << " Channel:";
    if (shtpHeader[2] == 0) _debugPort << "Command";
    else if (shtpHeader[2] == 1) _debugPort << "Executable";
    else if (shtpHeader[2] == 2) _debugPort << "Control";
    else if (shtpHeader[2] == 3) _debugPort << "Sensor-report";
    else if (shtpHeader[2] == 4) _debugPort << "Wake-report";
    else if (shtpHeader[2] == 5) _debugPort << "Gyro-vector";
    else _debugPort << shtpHeader[2];

    _debugPort << endl;
  }
  */
}
//STATUS: RFT -> TODO: enable serial output
//This function pulls the data from the input report
//The input reports vary in length so this function stores the various 16-bit values as globals

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0:4]: Then a 5 byte timestamp of microsecond clicks since reading was taken
//shtpData[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
//shtpData[5 + 1]: Sequence number (See 6.5.18.2)
//shtpData[5 + 2]: Status
//shtpData[3]: Delay
//shtpData[4:5]: i/accel x/gyro x/etc
//shtpData[6:7]: j/accel y/gyro y/etc
//shtpData[8:9]: k/accel z/gyro z/etc
//shtpData[10:11]: real/gyro temp/etc
//shtpData[12:13]: Accuracy estimate
/** This method parses value update packets from the BNO080 and writes updated values to relevant variables
 */
void BNO080::parseInputReport(void)
{
  //Calculate the number of data bytes in this packet
  int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
  dataLength &= ~(1 << 15); //Clear the MSbit. This bit indicates if this package is a continuation of the last.
  //Ignore it for now. TODO catch this as an error and exit

  dataLength -= 4; //Remove the header bytes from the data count

  uint8_t status = shtpData[5 + 2] & 0x03; //Get status bits
  uint16_t data1 = (uint16_t)shtpData[5 + 5] << 8 | shtpData[5 + 4];
  uint16_t data2 = (uint16_t)shtpData[5 + 7] << 8 | shtpData[5 + 6];
  uint16_t data3 = (uint16_t)shtpData[5 + 9] << 8 | shtpData[5 + 8];
  uint16_t data4 = 0;
  uint16_t data5 = 0;

  if (dataLength - 5 > 9)
  {
    data4 = (uint16_t)shtpData[5 + 11] << 8 | shtpData[5 + 10];
  }
  if (dataLength - 5 > 11)
  {
    data5 = (uint16_t)shtpData[5 + 13] << 8 | shtpData[5 + 12];
  }

  //Store these generic values to their proper global variable
  /* if (shtpData[5] == SENSOR_REPORTID_ACCELEROMETER)
  {
    accelAccuracy = status;
    rawAccelX = data1;
    rawAccelY = data2;
    rawAccelZ = data3;
  }
  else if (shtpData[5] == SENSOR_REPORTID_LINEAR_ACCELERATION)
  {
    accelLinAccuracy = status;
    rawLinAccelX = data1;
    rawLinAccelY = data2;
    rawLinAccelZ = data3;
  }
  else if (shtpData[5] == SENSOR_REPORTID_GYROSCOPE)
  {
    gyroAccuracy = status;
    rawGyroX = data1;
    rawGyroY = data2;
    rawGyroZ = data3;
  }
  else if (shtpData[5] == SENSOR_REPORTID_MAGNETIC_FIELD)
  {
    magAccuracy = status;
    rawMagX = data1;
    rawMagY = data2;
    rawMagZ = data3;
  } */
  else if (shtpData[5] == SENSOR_REPORTID_ROTATION_VECTOR || shtpData[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR)
  {
    quatAccuracy = status;
    rawQuatI = data1;
    rawQuatJ = data2;
    rawQuatK = data3;
    rawQuatReal = data4;
    rawQuatRadianAccuracy = data5; //Only available on rotation vector, not game rot vector
  }
  /* else if (shtpData[5] == SENSOR_REPORTID_STEP_COUNTER)
  {
    stepCount = data3; //Bytes 8/9
  }
  else if (shtpData[5] == SENSOR_REPORTID_STABILITY_CLASSIFIER)
  {
    stabilityClassifier = shtpData[5 + 4]; //Byte 4 only
  }
  else if (shtpData[5] == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER)
  {
    activityClassifier = shtpData[5 + 5]; //Most likely state

    //Load activity classification confidences into the array
    for (uint8_t x = 0 ; x < 9 ; x++) //Hardcoded to max of 9. TODO - bring in array size
      _activityConfidences[x] = shtpData[5 + 6 + x]; //5 bytes of timestamp, byte 6 is first confidence byte
  } */
  else if (shtpData[5] == SHTP_REPORT_COMMAND_RESPONSE)
  {
	  //Serial.println("!");
	  //The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
	  uint8_t command = shtpData[5 + 2]; //This is the Command byte of the response
	  
	  if(command == COMMAND_ME_CALIBRATE)
	  {
		//Serial.println("ME Cal report found!");
		calibrationStatus = shtpData[5 + 5]; //R0 - Status (0 = success, non-zero = fail)
	  }
  }
  else
  {
    //This sensor report ID is unhandled.
    //See reference manual to add additional feature reports as needed
  }

  //TODO additional feature reports may be strung together. Parse them all.
}

//STATUS: RFT
//This function pulls the data from the command response report

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0]: The Report ID
//shtpData[1]: Sequence number (See 6.5.18.2)
//shtpData[2]: Command
//shtpData[3]: Command Sequence Number
//shtpData[4]: Response Sequence Number
//shtpData[4 + 0]: R0
//shtpData[4 + 1]: R1
//shtpData[4 + 2]: R2
//shtpData[4 + 3]: R3
//shtpData[4 + 4]: R4
//shtpData[4 + 5]: R5
//shtpData[4 + 6]: R6
//shtpData[4 + 7]: R7
//shtpData[4 + 8]: R8
/** This method deals with command reports, we mostly ignore these reports unless it's about calibration.
 */
void BNO080::parseCommandReport(void)
{
  if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE)
  {
	  //The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
	  uint8_t command = shtpData[2]; //This is the Command byte of the response
	  
	  if(command == COMMAND_ME_CALIBRATE)
	  {
		calibrationStatus = shtpData[4 + 0]; //R0 - Status (0 = success, non-zero = fail)
	  }
  }
  else
  {
    //This sensor report ID is unhandled.
    //See reference manual to add additional feature reports as needed
  }

  //TODO additional feature reports may be strung together. Parse them all.
}

//Status: RFT
//Return the rotation vector quaternion I
/** This method returns i component of the rotation vector quaternion
 */
float BNO080::getQuatI()
{
  float quat = qToFloat(rawQuatI, rotationVector_Q1);
  return (quat);
}

//Status: RFT
//Return the rotation vector quaternion J
/** This method returns j component of the rotation vector quaternion
 */
float BNO080::getQuatJ()
{
  float quat = qToFloat(rawQuatJ, rotationVector_Q1);
  return (quat);
}

//Status: RFT
//Return the rotation vector quaternion K
/** This method returns k component of the rotation vector quaternion
 */
float BNO080::getQuatK()
{
  float quat = qToFloat(rawQuatK, rotationVector_Q1);
  return (quat);
}

//Return the rotation vector quaternion Real
/** This method returns read component of the rotation vector quaternion
 */
float BNO080::getQuatReal()
{
  float quat = qToFloat(rawQuatReal, rotationVector_Q1);
  return (quat);
}

//Status: RFT
//Return the rotation vector accuracy
/** This method returns quaternion accurate metric, see Hillcrest Labs SHTP manual for details
 */
float BNO080::getQuatRadianAccuracy()
{
  float quat = qToFloat(rawQuatRadianAccuracy, rotationVector_Q1);
  return (quat);
}
#include <SparkFunMPU9250-DMP.h>
//#include <SD.h>
#include "SdFat.h"
SdFat SD;
#include "config.h"

MPU9250_DMP imu; // Create an instance of the MPU9250_DMP class

/////////////////////////////
// Logging Control Globals //
/////////////////////////////
// Defaults all set in config.h
bool enableSDLogging = ENABLE_SD_LOGGING;
bool enableSerialLogging = ENABLE_UART_LOGGING;
bool enableTimeLog = ENABLE_TIME_LOG;
bool enableCalculatedValues = ENABLE_CALCULATED_LOG;
bool enableAccel = ENABLE_ACCEL_LOG;
bool enableGyro = ENABLE_GYRO_LOG;
bool enableCompass = ENABLE_MAG_LOG;
bool enableQuat = ENABLE_QUAT_LOG;
bool enableEuler = ENABLE_EULER_LOG;
bool enableHeading = ENABLE_HEADING_LOG;
unsigned short accelFSR = IMU_ACCEL_FSR;
unsigned short gyroFSR = IMU_GYRO_FSR;
unsigned short fifoRate = DMP_SAMPLE_RATE;

/////////////////////
// SD Card Globals //
/////////////////////
bool sdCardPresent = false; // Keeps track of if SD card is plugged in
String logFileName; // Active logging file
String logFileBuffer; // Buffer for logged data. Max is set in config

///////////////////////
// LED Blink Control //
///////////////////////
//bool ledState = false;
uint32_t lastBlink = 0;

int16_t dataBuffer[SD_LOG_WRITE_BUFFER_SIZE];
volatile uint16_t bufPointer = 0;
volatile bool writeData;
File logFile;

void blinkLED()
{
  static bool ledState = false;
  digitalWrite(HW_LED_PIN, ledState);
  ledState = !ledState;
}

void setup()
{
  initHardware(); 
  delay(2000);
  // Initialize the MPU-9250. Should return true on success:
  if ( !initIMU() ) 
  {
    //LOG_PORT.println("Error connecting to MPU-9250");
    while (1) ; // Loop forever if we fail to connect
    // LED will remain off in this state.
  }
  
  // Check for the presence of an SD card, and initialize it:
  if ( initSD() )
  {
    sdCardPresent = true;
    // Get the next, available log file name
    logFileName = nextLogFile();
    //LOG_PORT.println("yes SD");
  }
  else
  {
    //LOG_PORT.println("no SD");
    while(1);
  }
  
  writeData = false;
  pinMode(10, INPUT_PULLUP);
  
  updateAcc();
  updateAcc();
  updateAcc();
  updateAcc();
  attachInterrupt(digitalPinToInterrupt(10),updateAcc, FALLING );
  //LOG_PORT.println("loop");
}

void loop()
{
  //LOG_PORT.println("loop");
  if (writeData == true)
  {
    //LOG_PORT.println("write sd");
    writeData = false;
    if (bufPointer >= SD_LOG_WRITE_BUFFER_SIZE/2)
    {
           
      sdLogBuffer((uint8_t*)&dataBuffer[0]);
      blinkLED();
      //LOG_PORT.println(bufPointer);
    }
    else
    {   
      //uint32_t startmill = millis(); 
      sdLogBuffer((uint8_t*)&dataBuffer[SD_LOG_WRITE_BUFFER_SIZE/2]);
      blinkLED();
      //LOG_PORT.println(millis()-startmill);
      
    }
  }
}

void updateAcc()
{
  imu.updateAccel();
  dataBuffer[bufPointer++] = micros();
  dataBuffer[bufPointer++] = imu.ax;
  dataBuffer[bufPointer++] = imu.ay;
  dataBuffer[bufPointer++] = imu.az;
  if (bufPointer == SD_LOG_WRITE_BUFFER_SIZE/2)
  {
    writeData = true;
    //LOG_PORT.println(bufPointer);
  }
  if (bufPointer == SD_LOG_WRITE_BUFFER_SIZE)
  {
    writeData = true;
    //LOG_PORT.println(micros());
    bufPointer = 0;
  }
  //LOG_PORT.println(bufPointer);
}

void initHardware(void)
{
  // Set up LED pin (active-high, default to off)
  pinMode(HW_LED_PIN, OUTPUT);
  digitalWrite(HW_LED_PIN, LOW);

  // Set up MPU-9250 interrupt input (active-low)
  //pinMode(MPU9250_INT_PIN, INPUT_PULLUP);
  // Set up serial log port
  //LOG_PORT.begin(SERIAL_BAUD_RATE);
}

bool initIMU(void)
{
  // imu.begin() should return 0 on success. Will initialize
  // I2C bus, and reset MPU-9250 to defaults.
  if (imu.begin() != INV_SUCCESS)
    return false;

  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  imu.setSampleRate(IMU_AG_SAMPLE_RATE);
  imu.setAccelFSR(accelFSR);
  
  imu.setIntLevel(INT_ACTIVE_LOW);    // Set interrupt to active-low
  imu.setIntLatched(0);//INT_LATCHED);  // Latch interrupt output
  imu.enableInterrupt(); // Enable interrupt output
  return true; // Return success
}

bool initSD(void)
{
  // SD.begin should return true if a valid SD card is present
  if ( !SD.begin(SD_CHIP_SELECT_PIN, SPI_FULL_SPEED) )
  {
    return false;
  }

  return true;
}

// Log a string to the SD card
bool sdLogString(String toLog)
{
  // Open the current file name:
  File logFile = SD.open(logFileName, FILE_WRITE);
  
  // If the file will get too big with this new string, create
  // a new one, and open it.
  if (logFile.size() > (SD_MAX_FILE_SIZE - toLog.length()))
  {
    logFileName = nextLogFile();
    logFile = SD.open(logFileName, FILE_WRITE);
  }

  // If the log file opened properly, add the string to it.
  if (logFile)
  {
    logFile.print(toLog);
    logFile.close();

    return true; // Return success
  }

  return false; // Return fail
}

bool sdLogBuffer(uint8_t* toLog)
{
  // Open the current file name:
  //File logFile = SD.open(logFileName, O_CREAT | O_WRITE | O_APPEND);
  
  // If the file will get too big with this new string, create
  // a new one, and open it.
  if (logFile.size() > (SD_MAX_FILE_SIZE - SD_LOG_WRITE_BUFFER_SIZE))
  {
    logFile.close();
    logFileName = nextLogFile();
    logFile = SD.open(logFileName, O_CREAT | O_WRITE);
  }

  // If the log file opened properly, add the string to it.
  if (logFile)
  {
    logFile.write(toLog,SD_LOG_WRITE_BUFFER_SIZE/2);    
    //logFile.close();
  }
  return true; // Return success
}

// Find the next available log file. Or return a null string
// if we've reached the maximum file limit.
String nextLogFile(void)
{
  String filename;
  int logIndex = 0;

  for (int i = 0; i < LOG_FILE_INDEX_MAX; i++)
  {
    // Construct a file with PREFIX[Index].SUFFIX
    filename = String(LOG_FILE_PREFIX);
    filename += String(logIndex);
    filename += ".";
    filename += String(LOG_FILE_SUFFIX);
    // If the file name doesn't exist, return it
    if (!SD.exists(filename))
    {
      logFile = SD.open(filename, O_CREAT | O_WRITE);
      return filename;
    }
    // Otherwise increment the index, and try again
    logIndex++;
  }

  return "";
}

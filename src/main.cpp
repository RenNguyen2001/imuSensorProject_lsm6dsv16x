#include <LSM6DSV16XSensor.h>
#include <math.h>
#include <SPI.h>
#include <elapsedMillis.h>

elapsedMillis sinceTestTime;
elapsedMillis sinceTestTime2;

uint8_t CS_PIN  = 18;
#define SPI_MOSI  12
#define SPI_MISO  11
#define SPI_SCK 13

LSM6DSV16XSensor AccGyr(&SPI, CS_PIN);
uint8_t status = 0;


unsigned long currentTime = 0, previousTime[] = {0,0,0,0,0}, elapsedTime[] = {0,0,0,0,0}, printElapsedTime = 0, printPreviousTime = 0;
float yawAngleGlobal[] = {360-45,0,0,0,0}; //Starting yaw values, will changed based on calibration procedure/starting hand position
float piGlobal = 22.0/7.0;
float yawGlobalCompare = 0;

void prereqSetup();
void regularSetup(int dataRSet);
void gameSetup();
void getFifoData(int16_t quatData[], int chipS_Pin);


//Settings and const arrays to store values

typedef struct{
  uint8_t 
  dataRateSetting,
  gyroSensSetting,
  accSensSetting,
  gameODR_setting,
  gyroFilterBandwidth,
  accFilterBandwidth;

  float
  //yaw function settings
  yawSampleFrequency = 150.0,                   //frequency in Hz
  yawSamplePeriod = (1/yawSampleFrequency),   //period in seconds
  yawSampleCounter = yawSamplePeriod * 1e6;  //counter for the if statement
  ;

}imuSettings;
imuSettings imuSettingsGlobal;

const uint8_t dataRateValue[] = { //acc and gyro data Rates (Hz)
  0,  //off
  1,  //1.875
  2,  //7.5
  3,  //15
  4,  //30
  5,  //60
  6,  //120
  7,  //240 low power/high performance
  8,  //480
  9,  //960
  10, //1.92k
  11, //3.84k
  12, //7.68k
};

const uint8_t gyroFilterBandwidth_240Hz[] = { //gyro filter bandwidth look up table (240Hz)
  0,  //96
  1,  //96
  2,  //96
  3,  //96
  4,  //78.4
  5,  //53
  6,  //27.3
  7,  //14.2
};

const uint8_t gameODR_dataRateVal[] = { //game fusion output data rates (in Hz)
  0,  //15
  1,  //30
  2,  //60
  3,  //120
  4,  //240
  5   //480
};

const uint8_t accFilterBandwith_HP[] = {  //acc filter bandwidth look up table for high pass
  0,  //SLOPE
  1,  //ODR/10
  2,  //ODR/20
  3,  //ODR/45
  4,  //ODR/100
  5,  //ODR/200
  6,  //ODR/400
  7   //ODR/800
};

const uint8_t accRange[] = {  //the comments below represent the range and decimal value at 1G
  0,  //+2g, 16384
  1,  //+4g, 8192
  2,  //+8g, 4096
  3   //+16g, 2048
};

  const float accSen[] = {  //accelerometer sensitivity (g/LSB)
    0.061/1000,  //+-2g
    0.122/1000,  //+-4g
    0.244/1000,  //+-8g
    0.488/1000   //+-16g
  };

const uint8_t gyrRange[] = {  //the comments below represent the range (in dps)
  0b0000, //125
  0b0001, //250
  0b0010, //500
  0b0011, //1000
  0b0100, //2000
  0b1100  //4000
};

  const float gyrSen[] = {  //gyro sensitivity (dps/LSB)
      4.375/1000,  //+-125 dps
      8.75/1000,   //+-250 dps
      17.5/1000,   //+-500 dps
      35.0/1000,     //+-1000 dps
      70.0/1000,     //+-2000 dps
      140.0/1000     //+-4000 dps
  };

typedef struct{
  const uint8_t imuCS_pins[3];
  const String fingerName; 
}imuDetails;

String imuFingerLocation[3] = {"Tip","Mid","Base"};

//CS1 is the tip of the finger, CS2 is the middle, CS3 is the base

enum fingerNumbers{THUMB, INDEX, MIDDLE, RING, PINKIE, PALM};

enum imuLOCATION{TIP, MID, BASE};

imuDetails thumb = {{38,36,37},"Thumb: "};
imuDetails indexFinger = {{27,29,28},"Index: "};
imuDetails middle = {{8,10,9},"Middle: "};
imuDetails ring = {{2,4,3},"Ring: "};
imuDetails pinkie = {{17,0,16},"Pinkie: "};
imuDetails palm = {{18,17,17},"Palm: "};

imuDetails imuArr[] = {thumb,indexFinger,middle,ring,pinkie,palm};

typedef struct jointAngle{
  const uint8_t adjacentIMU_cs[2]; //the two IMUs that form the specific joint
  const String jointName;
  uint16_t angle;
}jointAngle;

enum jointNumbers{DIP, PIP, MCP};

jointAngle thumbDIP = {{thumb.imuCS_pins[0],thumb.imuCS_pins[1]}, "thumb DIP : ", 0};
jointAngle thumbPIP = {{thumb.imuCS_pins[1],thumb.imuCS_pins[2]}, "thumb PIP : ", 0};
jointAngle thumbMCP = {{thumb.imuCS_pins[2],palm.imuCS_pins[0]}, "thumb MCP : ", 0};

jointAngle indexDIP = {{indexFinger.imuCS_pins[0],indexFinger.imuCS_pins[1]}, "index DIP : ", 0};
jointAngle indexPIP = {{indexFinger.imuCS_pins[1],indexFinger.imuCS_pins[2]}, "index PIP : ", 0};
jointAngle indexMCP = {{indexFinger.imuCS_pins[2],palm.imuCS_pins[0]}, "index MCP : ", 0};

jointAngle middleDIP = {{middle.imuCS_pins[0],middle.imuCS_pins[1]}, "middle DIP: ", 0};
jointAngle middlePIP = {{middle.imuCS_pins[1],middle.imuCS_pins[2]}, "middle PIP: ", 0};
jointAngle middleMCP = {{middle.imuCS_pins[2],palm.imuCS_pins[0]}, "middle MCP: ", 0};

jointAngle ringDIP = {{ring.imuCS_pins[0],ring.imuCS_pins[1]}, "ring DIP  : ", 0};
jointAngle ringPIP = {{ring.imuCS_pins[1],ring.imuCS_pins[2]}, "ring PIP  : ", 0};
jointAngle ringMCP = {{ring.imuCS_pins[2],palm.imuCS_pins[0]}, "ring MCP  : ", 0};

jointAngle pinkieDIP = {{pinkie.imuCS_pins[0],pinkie.imuCS_pins[1]}, "pinkie DIP: ", 0};
jointAngle pinkiePIP = {{pinkie.imuCS_pins[1],pinkie.imuCS_pins[2]}, "pinkie PIP: ", 0};
jointAngle pinkieMCP = {{pinkie.imuCS_pins[2],palm.imuCS_pins[0]}, "pinkie MCP: ", 0};

jointAngle jointArr[][3] = {{thumbDIP, thumbPIP, thumbMCP}, 
  {indexDIP, indexPIP, indexMCP},
  {middleDIP, middlePIP, middleMCP},
  {ringDIP, ringPIP, ringMCP},
  {pinkieDIP, pinkiePIP, pinkieMCP}
};

enum rotationAxisOptions{YAW, PITCH, ROLL};

void setup() {
  imuSettingsGlobal.dataRateSetting = 7;  //setting 
  imuSettingsGlobal.gameODR_setting = 3;  //setting game data rate
  imuSettingsGlobal.gyroFilterBandwidth = 4;  imuSettingsGlobal.accFilterBandwidth = 5; //setting filter bandwidth
  imuSettingsGlobal.gyroSensSetting = 2;  imuSettingsGlobal.accSensSetting = 4; //setting sensitivity

  Serial.begin(120000);
  SPI.begin();

  Serial.println("Setting up IMUs....");

  for(char j = 0; j < 5; j++)
  {
    for(char i = 0; i < 3; i++)
    {
      AccGyr.cs_pin = imuArr[j].imuCS_pins[i]; //changing the cs_pin from within the header file, made the cs_pin variable public (you may have to manually change it to public in LSM6DSV16XSensor.h)
      prereqSetup();
      
      gameSetup(); //setup game
      delay(5);
    }
  }
  
  AccGyr.cs_pin = palm.imuCS_pins[0]; //changing the cs_pin from within the header file, made the cs_pin variable public
  prereqSetup();
    
  gameSetup(); //setup game
  delay(5);
}


/**
 * @brief Setup for both game or acc & gyro only mode
 * 
 */
void prereqSetup(){ 
  // Initialize LSM6DSV16X.
  AccGyr.begin();

  AccGyr.Write_Reg(0x10, dataRateValue[imuSettingsGlobal.dataRateSetting] + 0b00010000); //enable acc by setting the odr and setting to high acc ODR mode
  AccGyr.Write_Reg(0x11, dataRateValue[imuSettingsGlobal.dataRateSetting] + 0b00010000); //enable gyro by setting the odr and setting to high acc ODR mode

  // Setting the output data rate configuration register for HAODR
  AccGyr.Write_Reg(0x62, 0b00);  //setting the high acc ODR data rate

  // Configure FS of the acc and gyro
  AccGyr.Write_Reg(0x01, 0b00000000 + 0b00000000); //disable the embed reg access
  
  // Setting the filters and scale
  AccGyr.Write_Reg(0x18, (1<<5) + (0<<4)); //turning on fast settling mode and selecting low pass for accelereometer
  AccGyr.Write_Reg(0x17, accRange[imuSettingsGlobal.accSensSetting] + (imuSettingsGlobal.accFilterBandwidth<<5));  //setting the scale to +-2g and bandwidth
  AccGyr.Write_Reg(0x15, (gyroFilterBandwidth_240Hz[imuSettingsGlobal.gyroFilterBandwidth]<<4) + gyrRange[imuSettingsGlobal.gyroSensSetting]);  //setting the gyroscope lp filter bandwidth and setting the scale
}

void gameSetup(){
  AccGyr.Write_Reg(0x01, 0b00000000 + 0b10000000); //enable the embed reg access
  AccGyr.Write_Reg(0x02, 0b00000001 + 0b00000000); //turning page to embed page
  status |= AccGyr.Write_Reg(0x04, 0b00000010); //set the SFLP_game_EN bit in the EMB_FUNC_EN_A reg
  status |= AccGyr.Write_Reg(0x5E, 0b01000011 + (gameODR_dataRateVal[imuSettingsGlobal.gameODR_setting]<<3)); //sflp odr set
  status |= AccGyr.Write_Reg(0x66, 0b00000010); //sflp initialisation request
  
  status |= AccGyr.Write_Reg(0x44, 0b00000010); //enable sflp game rotation vector batching to fifo
  //status |= AccGyr.Write_Reg(0x44, 0b00010000); //enable sflp gravity vector batching to fifo
  //status |= AccGyr.Write_Reg(0x44, 0b00100000); //enable sflp gyroscope bias vector batching to fifo

  // Configure ODR and FS of the acc and gyro
  AccGyr.Write_Reg(0x01, 0b00000000 + 0b00000000); //disable the embed reg access
  
  // Set FIFO in Continuous mode
  status |= AccGyr.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);
  
  if(status != LSM6DSV16X_OK) {
    Serial.println("LSM6DSV16X Sensor failed to init/configure");
    while(1);
  }
  Serial.println("LSM6DSV16X FIFO Demo");
}


void getFifoData(int16_t quatData[], int chipS_Pin){
  AccGyr.cs_pin = chipS_Pin;
  uint8_t temp[6], startAddr = 0x79;
  float quatFloat[3];
  
  AccGyr.Write_Reg(0x01, 0b00000000 + 0b00000000); //disable the embed reg access
  
  for(uint8_t i = 0; i < 6; i++)
  {
    AccGyr.Read_Reg(startAddr + i, &temp[i]);
  }
    
  for(uint8_t i = 0; i < 3; i++)
  {
    quatData[i] = (uint16_t)(temp[i*2+1]<<8) + (uint16_t)temp[i*2];
  }
  //Serial.print("Quat X: "); Serial.print(quatData[0]); Serial.print(" Quat Y: "); Serial.print(quatData[1]); Serial.print(" Quat Z: "); Serial.println(quatData[2]);
  //Serial.print(quatData[0]); Serial.print("  "); Serial.print(quatData[1]); Serial.print("  "); Serial.println(quatData[2]); 
}


void getAccData(){
  uint8_t temp[6], startAddr = 0x28;
  int16_t accData[3];
  float accValue[3];

  for(uint8_t j = 0; j < 3; j++)
  {
    //getting the raw accelerometer values
    for(uint8_t i = 0; i < 6; i ++)
    {
      AccGyr.Read_Reg(startAddr + i, &temp[i]);
    }

    accData[j] = (uint16_t)(temp[j*2+1]<<8) + (uint16_t)temp[j*2];

    //converting the raw accelerometer values into +-G scale
    accValue[j] = accData[j] * accSen[imuSettingsGlobal.accSensSetting];
  }
  //accData[0] = (uint16_t)(temp[1]<<8) + (uint16_t)temp[0];  Serial.print("Acc Z: ");  Serial.println(accData[0]);
  //Serial.print("Acc X: ");  Serial.print(accData[0]); Serial.print(" Acc Y: ");  Serial.print(accData[1]);  Serial.print(" Acc Z: ");  Serial.println(accData[2]); 
  Serial.print("Acc X: ");  Serial.print(accValue[0]); Serial.print(" Acc Y: ");  Serial.print(accValue[1]);  Serial.print(" Acc Z: ");  Serial.println(accValue[2]);     
}

void getGyrData(float* gyrOutput, int chipS_Pin){
  AccGyr.cs_pin = chipS_Pin;
  uint8_t temp[6], startAddr = 0x22;
  int16_t gyrData[3];
  float gyrValue[3];
  
  for(uint8_t j = 0; j < 3; j++)
  {
    //Getting the raw gyro values from output registers
    for(uint8_t i = 0; i < 6; i ++)
    {
      AccGyr.Read_Reg(startAddr + i, &temp[i]);
    }
    gyrData[j] = (uint16_t)(temp[j*2+1]<<8) + (uint16_t)temp[j*2];

    //Converting the raw values into +-dps scale
    gyrValue[j] = (float)gyrData[j] * gyrSen[imuSettingsGlobal.gyroSensSetting]; 
  }
  *gyrOutput = gyrValue[2];  
  //accData[0] = (uint16_t)(temp[1]<<8) + (uint16_t)temp[0];  Serial.print("Acc Z: ");  Serial.println(accData[0]);
  //Serial.print("     Gyr X: ");  Serial.print(gyrData[0]); Serial.print("   Gyr Y: ");  Serial.print(gyrData[1]);  Serial.print("   Gyr Z: ");  Serial.println(gyrData[2]);  
  //Serial.print("     Gyr X: ");  Serial.print(gyrValue[0]); Serial.print("   Gyr Y: ");  Serial.print(gyrValue[1]);  Serial.print("   Gyr Z: ");  Serial.println(gyrValue[2]);   
}

void getYawAng(const uint8_t fingerNum){
  //start counting time
  currentTime = micros();
  elapsedTime[fingerNum] = currentTime - previousTime[fingerNum];

  //sample variables
  float sum = 0, result, gyrData;

  int numOfFingers = 2;

  //take one sample at the end of each sampling Period
  if(elapsedTime[fingerNum] >= imuSettingsGlobal.yawSampleCounter)  //refer to imuSettings for yaw settings (to change sample period and sample frequency)
  {
    previousTime[fingerNum] = micros();
    elapsedTime[fingerNum] = 0;

    for(uint8_t i = 0; i < numOfFingers; i ++)  //calculating the yaw at each IMU on a finger
    {
      getGyrData(&gyrData, imuArr[fingerNum].imuCS_pins[i]);
      if(abs(gyrData) > 10)
      {
        sum = sum + gyrData;
      }
      
    }

    result = ((sum)*(imuSettingsGlobal.yawSamplePeriod))/numOfFingers; //calculating the average yaw of the finger
    yawAngleGlobal[fingerNum] = yawAngleGlobal[fingerNum] + result;
    if(yawAngleGlobal[fingerNum] <= 0)
    {
      yawAngleGlobal[fingerNum] = 360 - yawAngleGlobal[fingerNum];
    }
    else if(yawAngleGlobal[fingerNum] > 360)
    {
      yawAngleGlobal[fingerNum] = yawAngleGlobal[fingerNum] - 360;
    }
    //Serial.print("Yaw: ");  Serial.println(yawAngleGlobal[fingerNum]);  
  }
}


float calculate3DVecAngle(float vecA[], float vecB[]){ //calculates the angle in degrees between two vectors
  double result = atan2(vecA[1],vecA[0]) - atan2(vecB[1],vecB[0]);
  float degrees = result*(180.0/3.14);
  if(degrees < 0){ degrees = 360 + degrees; } //if in the 4th quadrant, add to 360 to get the correct angle
  return degrees;
}

/**
 * @brief Uses the gravity vector calculate pitch and roll
 * 
 * @param quatData - FIFO input data
 * @param indexNo - The index of the current IMU
 * @param output - Stores the pitch and roll values
 */
void gravityVecToEuler(int16_t quatData[], uint8_t indexNo, float output[][2]){ 
  float gravValue = 16384;
  float pitchTheta, rollTheta;
  float zAxisVec[3] = {0,gravValue};

  //value from the gravity vector will be masked to see changes across the y-z (pitch) and x-z (roll) plane
  float gravVectorPitch[3] = {quatData[1],quatData[2]}; //zeroing the x axis to see changes across the y-z plane
  float gravVectorRoll[3] = {quatData[0],quatData[2]};  //zeroing the y axis to see changes across the x-z plane
  
  //calculating the angle between the vector and each axis to get pitch and roll
  //calculating pitch (rotation around x axis/ change across the y and z axis)  
  pitchTheta = calculate3DVecAngle(gravVectorPitch, zAxisVec); 
  
  //calculating roll (rotation around y axis/ change across the x and z axis)
  rollTheta = calculate3DVecAngle(gravVectorRoll, zAxisVec);

  output[indexNo][0] = pitchTheta;  output[indexNo][1] = rollTheta;
}

/**
 * @brief Calculates the angle between two adjacent IMUs
 * 
 * @param input 
 * @param imu1 First IMU pitch and roll data
 * @param imu2 Second IMU pitch and roll data
 * @param result Joint angle in degrees
 */
void calculateJointAng(float input[][2], uint8_t imu1, uint8_t imu2, float result[]){
  //get the pitch and yaw for the two IMUs
  float pitch1 = input[imu1][0], roll1 = input[imu1][1],
  pitch2 = input[imu2][0], roll2 = input[imu2][1];

  //subtract the angles and put it into the return values
    //calculating pitch
    if(pitch2 > pitch1)
    {
      result[0] = 360 - pitch2 + pitch1;
    }
    else if(pitch2 < pitch1)
    {
      result[0] = pitch1 - pitch2;
    }

    //calculating roll
    if(roll2 > roll1)
    {
      result[1] = 360 - roll2 + roll1;
    }
    else if(roll2 < roll1)
    {
      result[1] = roll1 - roll2;
    }
    
  //Serial.print("Pitch D: ");  Serial.print(result[0]);  Serial.print(" Roll D: ");  Serial.println(result[1]);
  //Serial.print(imu1);  Serial.print(" ");  Serial.print(imu2);  Serial.print(" "); Serial.print(result[0]);  Serial.print(" ");  Serial.println(result[1]);
}

void readSingleIMUstrip(int16_t fifoOut[4], float outVals[][2], const uint8_t fingerNum){ //0 for thumb... 4 for pinkie
  for(int i = 0; i < 3; i++)
  {
    getFifoData(fifoOut, imuArr[fingerNum].imuCS_pins[i]);                    //get the data from the FIFO registers
    gravityVecToEuler(fifoOut, imuArr[fingerNum].imuCS_pins[i], outVals);     //get roll and pitch from the gravity vector
    //Serial.print(imuArr[fingerNum].fingerName); Serial.print(imuArr[fingerNum].imuCS_pins[i], DEC); Serial.print("  ");
    //Serial.print(fifoOut[0]); Serial.print("  "); Serial.print(fifoOut[1]); Serial.print("  "); Serial.println(fifoOut[2]);
  }
  getFifoData(fifoOut, imuArr[PALM].imuCS_pins[0]); gravityVecToEuler(fifoOut, imuArr[PALM].imuCS_pins[0], outVals);  //repeat the above process for the palm IMU  
  outVals[imuArr[PALM].imuCS_pins[0]][0] =  outVals[imuArr[PALM].imuCS_pins[0]][0] - 180; //subtract the palm IMU by 180 since it is facing upwards
  
}

void readSingleIMUstripPrint(int16_t fifoOut[4], float outVals[][2], const uint8_t fingerNum){ //0 for thumb... 4 for pinkie
  for(int i = 0; i < 3; i++)
  {
    getFifoData(fifoOut, imuArr[fingerNum].imuCS_pins[i]);  gravityVecToEuler(fifoOut, imuArr[fingerNum].imuCS_pins[i], outVals);
    Serial.print(imuArr[fingerNum].fingerName); Serial.print(imuFingerLocation[i]); Serial.print("  ");
    Serial.print(fifoOut[0]); Serial.print("  "); Serial.print(fifoOut[1]); Serial.print("  "); Serial.println(fifoOut[2]);
  }
}

void readSingleIMU(int16_t fifoOut[4], float outVals[][2], const uint8_t fingerNum, const uint8_t imuNum){
    uint8_t imuCS_pin =  imuArr[fingerNum].imuCS_pins[imuNum];
    getFifoData(fifoOut, imuCS_pin);  gravityVecToEuler(fifoOut, imuCS_pin, outVals);
    
    Serial.print(" Pitch angle: ");  Serial.print(outVals[imuCS_pin][0]); Serial.print(" Roll angle: ");  Serial.print(outVals[imuCS_pin][1]);  Serial.print(" ");
    Serial.print(imuArr[fingerNum].fingerName); Serial.print(imuArr[fingerNum].imuCS_pins[imuNum], DEC); Serial.print("  ");
    Serial.print(fifoOut[0]); Serial.print("  "); Serial.print(fifoOut[1]); Serial.print("  "); Serial.println(fifoOut[2]); //reading the raw gravity vector values

}

void readMultiIMUstrips(int16_t fifoOut[4], float outVals[][2]){
  readSingleIMUstripPrint(fifoOut, outVals, THUMB);
  readSingleIMUstripPrint(fifoOut, outVals, INDEX);
  readSingleIMUstripPrint(fifoOut, outVals, MIDDLE);
  readSingleIMUstripPrint(fifoOut, outVals, RING);
  readSingleIMUstripPrint(fifoOut, outVals, PINKIE);
}

/**
 * @brief Calculates the joint angles (using the gravity vector) for a single flex PCB strip and outputs it to serial
 * 
 * @param fifoOut Fifo Input data
 * @param outVals Array that stores pitch and roll values
 * @param jointAng Joint angle in degrees
 * @param fingerNum Chooses which finger is read
 */
void readSingleIMUstripJoint(int16_t fifoOut[4], float outVals[][2], float jointAng[], const uint8_t fingerNum){ 
  // data output = finger strip num, jointAng1(tip), jointAng2(mid), jointAng3(base), yawAngle, palmRollAng 

  readSingleIMUstrip(fifoOut, outVals, fingerNum);
  getYawAng(fingerNum);
  Serial.print(fingerNum);  //print out the index of the current finger
  for(char i = 0; i < 3; i++)
  {
    calculateJointAng(outVals, jointArr[fingerNum][i].adjacentIMU_cs[0], jointArr[fingerNum][i].adjacentIMU_cs[1], jointAng); //calculate the joint angles of a single finger
    //Serial.print(jointArr[fingerNum][i].jointName);  Serial.print(" "); Serial.print((String)jointAng[0]);  Serial.print(" ");
    
    Serial.print(" "); Serial.print((String)jointAng[0]); //jointAng[0] is pitch, jointAng[1] is pitch
  }
  Serial.print(" ");  Serial.print(-yawAngleGlobal[fingerNum]);
  Serial.print(" ");  Serial.println((String)outVals[imuArr[PALM].imuCS_pins[0]][1]); //palmRollAng
}

/**
 * @brief Reads the joint angles for multiple flex PCBs using gravity vector calculations
 * 
 * @param fifoOut - Input data from the fifoRead function
 * @param outVals - stores the pitch and roll values
 * @param jointAng - Joint angle in degrees
 */
void readMultiIMUstripJoints(int16_t fifoOut[4], float outVals[][2], float jointAng[]){
  readSingleIMUstripJoint(fifoOut, outVals, jointAng, THUMB);
  readSingleIMUstripJoint(fifoOut, outVals, jointAng, INDEX);
  readSingleIMUstripJoint(fifoOut, outVals, jointAng, MIDDLE);
  readSingleIMUstripJoint(fifoOut, outVals, jointAng, RING);
  readSingleIMUstripJoint(fifoOut, outVals, jointAng, PINKIE);
}

/**
 * @brief Get the quaternion data for a single IMU
 * 
 * @param quatData - quaternion output data
 * @param chipSelectPin - chooses which IMU is read
 */
void getSingleQuatData(float quatData[], uint8_t chipSelectPin){
  AccGyr.cs_pin = chipSelectPin;
  AccGyr.FIFO_Get_Rotation_Vector(quatData);
}


/**
 * @brief Convert quaternions into euler angles; pitch, roll and yaw
 * 
 * @param quaternion - quaternion input data
 * @param output - output value, roll changes when fingers bend
 */
void quat2Euler(float quaternion[], float* rollOut, float* yawOut){ 
  float qw, qx, qy, qz;
  float yaw, pitch, roll;

  qw = quaternion[3]; qx = quaternion[0]; qy = quaternion[1]; qz = quaternion[2]; //making it easier to read

  yaw = atan2( 2.0000*(qx*qy+qw*qz), sq(qw) + sq(qx) - sq(qy) - sq(qz))*(180.0000/piGlobal);
  roll = atan2( 2*(qy*qz+qw*qx), sq(qw) - sq(qx) - sq(qy) + sq(qz))*(180.0/piGlobal);
  pitch = asin(-2*(qx*qz-qw*qy))*(180.0/piGlobal);

  //convert roll from (-180 to 180) range to (0 to 360) range
  roll = 180 - roll;

  *rollOut = roll;
  *yawOut = 180 - yaw;  //to correct the yaw since the finger strips IMUs are upside down

  //Serial.print("Roll: "); Serial.print(roll); Serial.print(" Yaw: "); Serial.print(yaw);  Serial.print(" Pitch: "); Serial.println(pitch);
}

/**
 * @brief Read a single IMU, calculated using quaternions
 * 
 */
void readSingleIMUQuat(uint8_t fingerNum, uint8_t imuIndex){
  float quatData[4], eulerAng, yaw, qw, qx, qy, qz;
  getSingleQuatData(quatData, imuArr[fingerNum].imuCS_pins[imuIndex]); 
  qw = quatData[3]; qx = quatData[0]; qy = quatData[1]; qz = quatData[2]; //making it easier to read
  Serial.print("Quat:");  
  Serial.print(qw, 4); Serial.print(" ");  
  Serial.print(qx, 4); Serial.print(" ");
  Serial.print(qy, 4); Serial.print(" ");
  Serial.print(qz, 4); Serial.print(" ");

  quat2Euler(quatData, &eulerAng, &yaw);
  Serial.print("Yaw: ");  Serial.println(yaw);  
}

/**
 * @brief Print out the angle of a single joint, for use with the excel data streamer
 * 
 * @param fingerNum 
 * @param jointNum 
 * @param referenceAng Just to excel that is the angle value we want to record (it selects to corresponding column)
 */
void printSingleJoint(uint8_t fingerNum, uint8_t jointNum, uint8_t referenceAng, uint8_t *numOfPrints){
  float quatData[4], quatData2[4], eulerAng[2], result[3], yaw[3], yaw2, sum = 0;
  int printPeriod = 500;  //print delay in microseconds

  for(uint8_t i = 0; i < 3; i++)
  {
    getSingleQuatData(quatData, jointArr[fingerNum][i].adjacentIMU_cs[0]); quat2Euler(quatData, &eulerAng[0], &yaw[i]);
    getSingleQuatData(quatData2, jointArr[fingerNum][i].adjacentIMU_cs[1]);  quat2Euler(quatData2, &eulerAng[1], &yaw2);

    if(i == 2)
    {
      eulerAng[1] = eulerAng[1] + 180;  //The IMU on the mainboard and the IMU on the strips are facing opposite directions
      
    }

    //Change the angle range from -180->+180 to 0->360
    if(eulerAng[1] > eulerAng[0])
    {
      result[i] = (360 - eulerAng[1]) + eulerAng[0];
    }
    else
    {
      result[i] = eulerAng[0] - eulerAng[1];
    }

  }

  getYawAng(fingerNum);

  getSingleQuatData(quatData, imuArr[PALM].imuCS_pins[TIP]); quat2Euler(quatData, &eulerAng[0], &yaw2);
 
  eulerAng[0] = eulerAng[0] - 180;
  if(eulerAng[0] < 0)
  {
    eulerAng[0] = 360 + eulerAng[0];
  }

  currentTime = millis();
  printElapsedTime = currentTime - printPreviousTime;

  if(printElapsedTime > printPeriod)  //incase the data streamer smapling delay doesn't work
  {
    printElapsedTime = 0;
    printPreviousTime = millis();
    *numOfPrints = *numOfPrints + 1;
    Serial.print(fingerNum); Serial.print(",");  Serial.print(jointNum); Serial.print(","); Serial.print(referenceAng, DEC); 
    Serial.print(","); Serial.println(result[jointNum]);
    
  }

  
}

void calculateThumbStripJnts(){
  uint8_t fingerNum = 0;
  // data output = finger strip num, jointAng1(tip), jointAng2(mid), jointAng3(base), yawAngle, palmRollAng 

  float quatData[4], quatData2[4], eulerAng[2], result[3], yaw[3], yaw2, sum = 0;

  //get the angle between the IP IMU and the mainboard IMU
  getSingleQuatData(quatData, imuArr[THUMB].imuCS_pins[MID]); quat2Euler(quatData, &eulerAng[0], &yaw[0]); //The IP IMU
  getSingleQuatData(quatData, imuArr[PALM].imuCS_pins[TIP]); quat2Euler(quatData, &eulerAng[1], &yaw[0]); //mainboard IMU

  eulerAng[1] = eulerAng[1] + 180;  //The IMU on the mainboard and the IMU on the strips are facing opposite directions

  //Change the angle range from -180->+180 to 0->360
  if(eulerAng[1] > eulerAng[0])
  {
    result[0] = (360 - eulerAng[1]) + eulerAng[0];
  }
  else
  {
    result[0] = eulerAng[0] - eulerAng[1];
  }
  //result[0] is the angle between the IP and mainboard IMU

  getSingleQuatData(quatData, imuArr[THUMB].imuCS_pins[TIP]); quat2Euler(quatData, &eulerAng[0], &yaw[0]); 
  getSingleQuatData(quatData, imuArr[THUMB].imuCS_pins[MID]); quat2Euler(quatData, &eulerAng[1], &yaw[0]); 

  if(eulerAng[1] > eulerAng[0])
  {
    result[1] = (360 - eulerAng[1]) + eulerAng[0];
  }
  else
  {
    result[1] = eulerAng[0] - eulerAng[1];
  }

  result[2] = 0;

  getYawAng(fingerNum);

  getSingleQuatData(quatData, imuArr[PALM].imuCS_pins[TIP]); quat2Euler(quatData, &eulerAng[0], &yaw2);
 
  eulerAng[0] = eulerAng[0] - 180;
  if(eulerAng[0] < 0)
  {
    eulerAng[0] = 360 + eulerAng[0];
  }

  
  Serial.print(fingerNum);Serial.print(" ");
  for(uint8_t i = 0; i < 3; i++)
  {
    Serial.print(result[i]);  Serial.print(" ");
  }
  Serial.print(yawAngleGlobal[fingerNum]); Serial.print(" ");
  Serial.println(eulerAng[0]);  //tilt of mainboard
}

/**
 * @brief Calculate the joint angles for a single flex PCB strip using quaternions
 * 
 * @param fingerNum - Chooses which finger strip is read
 */
void calculateStripJnts(uint8_t fingerNum){
  // data output = finger strip num, jointAng1(tip), jointAng2(mid), jointAng3(base), yawAngle, palmRollAng 

  float quatData[4], quatData2[4], eulerAng[2], result[3], yaw[3], yaw2, sum = 0;

  for(uint8_t i = 0; i < 3; i++)
  {
    getSingleQuatData(quatData, jointArr[fingerNum][i].adjacentIMU_cs[0]); quat2Euler(quatData, &eulerAng[0], &yaw[i]);
    getSingleQuatData(quatData2, jointArr[fingerNum][i].adjacentIMU_cs[1]);  quat2Euler(quatData2, &eulerAng[1], &yaw2);

    if(i == 2)
    {
      eulerAng[1] = eulerAng[1] + 180;  //The IMU on the mainboard and the IMU on the strips are facing opposite directions
      
    }

    //Change the angle range from -180->+180 to 0->360
    if(eulerAng[1] > eulerAng[0])
    {
      result[i] = (360 - eulerAng[1]) + eulerAng[0];
    }
    else
    {
      result[i] = eulerAng[0] - eulerAng[1];
    }

  }

  getYawAng(fingerNum);

  getSingleQuatData(quatData, imuArr[PALM].imuCS_pins[TIP]); quat2Euler(quatData, &eulerAng[0], &yaw2);
 
  eulerAng[0] = eulerAng[0] - 180;
  if(eulerAng[0] < 0)
  {
    eulerAng[0] = 360 + eulerAng[0];
  }

  
  Serial.print(fingerNum);Serial.print(" ");
  for(uint8_t i = 0; i < 3; i++)
  {
    Serial.print(result[i]);Serial.print(" ");
  }
  Serial.print(yawAngleGlobal[fingerNum]); Serial.print(" ");
  Serial.println(eulerAng[0]);  //tilt of mainboard
  
  
  //print statements for datastreamer (datastream uses a comma to separate values)
  
  /*
  Serial.print(fingerNum);  Serial.print(",");
  for(uint8_t i = 0; i < 3; i++)
  {
    Serial.print(result[i]);Serial.print(",");
  }
  Serial.print(yawAngleGlobal[fingerNum]); Serial.print(",");
  Serial.print(eulerAng[0]);  Serial.print(",");  //tilt of mainboard
  */
  
}


/**
  * @brief Get joint angles for multiple flex PCB strips using quaternions
  */
void calculateMultStripJnts(){
  calculateThumbStripJnts();
  calculateStripJnts(INDEX);
  calculateStripJnts(MIDDLE);
  calculateStripJnts(RING);
  calculateStripJnts(PINKIE);
  //Serial.println("");
}

/**
 * @brief Sends a signal to excel data streamer to record data
 * 
 * @param fingerNum 
 * @param jointNum 
 */
void angMeasurementTest(uint8_t fingerNum, uint8_t jointNum){
  //ask the user to input a value into the serial monitor stating the angle (e.g. 0 (0 won't work), 10 ,20)
  Serial.print("Enter in the reference Angle: ");
  uint8_t numOfPrints = 0;
  uint8_t serialData;

  while(1)
  {
    numOfPrints = 0;
    if(Serial.available() > 0)
    {
      serialData = Serial.parseInt();
    }
    //Serial.print(Serial.parseInt());

    if(serialData != 0)
    {
      if(serialData == 100) //for 0 degrees
      {
        serialData = 0;
      }
      //Serial.print("Reference Ang: ");  Serial.println(serialData);

      while(numOfPrints < 20)
      {
        printSingleJoint(fingerNum, jointNum, serialData, &numOfPrints);
        //Serial.println(numOfPrints);
      }
      serialData = 0;
      //Serial.println("Out of loop");
      //Serial.print("Enter in the reference Angle: ");
    }
  }
  
}

void stationaryTest(){  //prints out all joint angles for a specific number of seconds
  uint8_t serialData = 0, numOfPrints = 0; 
  uint16_t samplingInterval = 3e3;

  Serial.print("Enter any value except 0 to start ");
  while(1)
  {
    if(Serial.available() > 0)
    {
      serialData = Serial.read();
    }

    if(serialData != 0)
    {
      
      while(1) //stay in loop until the num of prints has been reached
      {
        if(sinceTestTime2 > samplingInterval)  //sampling interval
        {
          sinceTestTime2 = 0;
          calculateMultStripJnts();
          numOfPrints++;
        }
        if(numOfPrints == 11)
        {
          sinceTestTime = 0;
          serialData = 0;
          break;
        }
        
        //Serial.println("Started");
      }

      //Serial.println("Finished");
      
    }
  }
}


void loop() {

  //=================================================GRAVITY VECTOR FUNCTIONS=======================================================
  int16_t fifoOut[4];
  float outVals[40][2], jointAng[2];  //outvals is to store pitch and roll
  int gyrOutput;

    //====================reading joints (gravity vector)=================
      //readSingleIMUstripJoint(fifoOut, outVals, jointAng, PINKIE);
      //readSingleIMUstripJoint(fifoOut, outVals, jointAng, RING);
      //readSingleIMUstripJoint(fifoOut, outVals, jointAng, MIDDLE);
      //readSingleIMUstripJoint(fifoOut, outVals, jointAng, INDEX);
      //readMultiIMUstripJoints(fifoOut, outVals, jointAng);  //data is output in this order: fingerNum jointAng1 jointAng2 jointAng3 
    //====================================================================

    //====================reading individual IMUS=========================
      //readSingleIMUstrip(fifoOut, outVals, INDEX); 
      //readSingleIMUstripPrint(fifoOut, outVals, MIDDLE);
      //readSingleIMU(fifoOut, outVals, PALM, TIP); //readSingleIMU(fifoOut, outVals, INDEX, TIP);
      //readMultiIMUstrips(fifoOut, outVals);
    //====================================================================

    //===Functions for AccGyro only======
      //getYawAng(INDEX);
    //===================================

  
  //=================================================================================================================================


  //=================================================QUATERNION FUNCTIONS=======================================================
  //float quatData[4], quatData2[4], eulerAng[2], result;
  uint8_t numOfPrints;

    //readSingleIMUQuat(INDEX, MID);  //readSingleIMUQuat(MIDDLE, TIP);
    //calculateStripJnts(RING); //calculateStripJnts(INDEX);
    calculateMultStripJnts();
    //printSingleJoint(INDEX, PIP, 0, &numOfPrints);
    
    //calculateMultStripJnts();
    //quats2Joint(quatData2, quatData, ROLL);

    //Test functions
      //angMeasurementTest(INDEX, PIP);
      //stationaryTest();

  //=================================================================================================================================
  
}





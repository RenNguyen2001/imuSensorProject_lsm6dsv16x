#include <LSM6DSV16XSensor.h>
#include <math.h>

#define SENSOR_ODR 120.0f // In Hertz
#define ACC_FS 2 // In g
#define GYR_FS 4000 // In dps
//FS is range
#define MEASUREMENT_TIME_INTERVAL (1000.0f/SENSOR_ODR) // In ms
#define FIFO_SAMPLE_THRESHOLD 199
#define FLASH_BUFF_LEN 8192


uint8_t CS_PIN  = 17;
//17,18 for the breadboard, 18 for the PCB
#define SPI_MOSI  12
#define SPI_MISO  11
#define SPI_SCK 13

LSM6DSV16XSensor AccGyr(&SPI, CS_PIN);
uint8_t status = 0;
int16_t firstFrame[3];
uint8_t xRangeGlobal;

void prereqSetup(uint8_t dataRSet, uint8_t gyroFiltS, uint8_t accFiltS, uint8_t xRange, uint8_t gRangeS, uint8_t csPin);
void regularSetup(int dataRSet);
void gameSetup(int gameODRSet);
void getFifoData(int16_t quatData[], int chipS_Pin);

//const arrays to store values

const uint8_t dR[] = { //acc and gyro data Rates (Hz)
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

const uint8_t gF_LU240Hz[] = { //gyro filter bandwidth look up table (240Hz)
  0,  //96
  1,  //96
  2,  //96
  3,  //96
  4,  //78.4
  5,  //53
  6,  //27.3
  7,  //14.2
};

const uint8_t gameODR[] = { //game fusion output data rates (in Hz)
  0,  //15
  1,  //30
  2,  //60
  3,  //120
  4,  //240
  5   //480
};

const uint8_t aF_LU_HP[] = {  //acc filter bandwidth look up table for high pass
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

  const uint16_t accSen[] = {  //accelerometer sensitivity
    16384,  //+2g, 16384
    8192,   //+4g, 8192
    4096,   //+8g, 4096
    2048    //+16g, 2048
  };

const uint8_t gyrRange[] = {  //the comments below represent the range (in dps)
  0b0000, //125
  0b0001, //250
  0b0010, //500
  0b0011, //1000
  0b0100, //2000
  0b1100  //4000
};

const uint8_t indexIMUs[] = {
  17,
  18,
  19
};

void setup() {
  uint8_t dRSet = 7;    //gyr and acc output data rate choice (refer to dR[])
  uint8_t gODRSet = 3;  //game vector output data rate choice (refer to gameODR[])
  uint8_t gFiltSetting = 4, xFiltSetting = 5; //gyro and acc filter bandwidth selection (refer to gF_LU240Hz[] and aF_LU_HP[])
  uint8_t xRangeS = 2, gRangeS = 4; //acc and gyro range setting (refer to accRange[] and gyrRange[])
  uint8_t csPin = 18;

  xRangeGlobal = xRangeS;

  Serial.begin(120000);
  SPI.begin();

  Serial.println("Setting up systems....");

  for(char i = 0; i <= sizeof(indexIMUs); i++)
  {
    AccGyr.cs_pin = indexIMUs[i]; //changing the cs_pin from within the header file, made the cs_pin variable public
    prereqSetup(dRSet, gFiltSetting, xFiltSetting, xRangeS, gRangeS, csPin);
    
    gameSetup(gODRSet); //setup game
    delay(1);
  }
  
}

void prereqSetup(uint8_t dataRSet, uint8_t gyroFiltS, uint8_t accFiltS, uint8_t xRangeS,
uint8_t gRangeS, uint8_t csPin){ //setup required for both game vector or acc & gyro only mode
  // Initialize LSM6DSV16X.
  AccGyr.begin();

  AccGyr.Write_Reg(0x10, dR[dataRSet] + 0b00010000); //enable acc by setting the odr and setting to high acc ODR mode
  AccGyr.Write_Reg(0x11, dR[dataRSet] + 0b00010000); //enable gyro by setting the odr and setting to high acc ODR mode

  // Setting the output data rate configuration register for HAODR
  AccGyr.Write_Reg(0x62, 0b00);  //setting the high acc ODR data rate

  // Configure FS of the acc and gyro
  AccGyr.Write_Reg(0x01, 0b00000000 + 0b00000000); //disable the embed reg access

  // Setting the filters and scale
  AccGyr.Write_Reg(0x18, (1<<5) + (0<<4)); //turning on fast settling mode and selecting low pass for accelereometer
  AccGyr.Write_Reg(0x17, accRange[xRangeS] + (accFiltS<<5));  //setting the scale to +-2g and bandwidth
  AccGyr.Write_Reg(0x15, (gF_LU240Hz[gyroFiltS]<<4) + gyrRange[gRangeS]);  //setting the gyroscope lp filter bandwidth and setting the scale
}

void regularSetup(int dataRSet){  //to setup only the acc and gyr
  
  // Configure FIFO BDR for acc and gyro
  AccGyr.Write_Reg(0x09, (dR[dataRSet-1]<<4) + dR[dataRSet-1]); //setting the Batch data rate for Gyro and Acc to 240Hz

  // Set FIFO in Continuous mode
  status |= AccGyr.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);
  
  if(status != LSM6DSV16X_OK) {
    Serial.println("LSM6DSV16X Sensor failed to init/configure");
    while(1);
  }
  Serial.println("LSM6DSV16X FIFO Demo");
}


void gameSetup(int gameODRSet){
  AccGyr.Write_Reg(0x01, 0b00000000 + 0b10000000); //enable the embed reg access
  AccGyr.Write_Reg(0x02, 0b00000001 + 0b00000000); //turning page to embed page
  status |= AccGyr.Write_Reg(0x04, 0b00000010); //set the SFLP_game_EN bit in the EMB_FUNC_EN_A reg
  status |= AccGyr.Write_Reg(0x5E, 0b01000011 + (gameODR[gameODRSet]<<3)); //sflp odr set
  status |= AccGyr.Write_Reg(0x66, 0b00000010); //sflp initialisation request
  
  //status |= AccGyr.Write_Reg(0x44, 0b00000010); //enable sflp game rotation vector batching to fifo
  status |= AccGyr.Write_Reg(0x44, 0b00010000); //enable sflp gravity vector batching to fifo
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

void unityDataPrep(int16_t gameArr[]){
  int8_t byteH, byteL;
  int16_t directions[] = {254, 400, 555};  //needs to be two bytes to store values up to 65000
  int x, y, z;
  directions[0] = gameArr[0]; directions[1] = gameArr[1]; directions[2] = gameArr[2];   
  //Serial.print("Size of normal int is:"); Serial.println(sizeof(directions[0]));  Serial.print("Size of acc int is:"); Serial.println(sizeof(accArr[0]));

  Serial.write(255);  Serial.write(110);  //sending the dummy byte(s) to mark the first byte
  //need to seperate each interger into two bytes to send
  for(int i = 0; i < sizeof(directions)/sizeof(directions[0]); i++)
  {
    //for each loop, the data is seperated into two bytes and then sent
        
    byteL = directions[i];
        
    byteH = (int16_t)directions[i] >> 8;
        
    Serial.write(byteL);  Serial.write(byteH);
   }
   Serial.write('\n');
   delayMicroseconds(10);
  
}

void getFifoData(int16_t quatData[], int chipS_Pin){
  AccGyr.cs_pin = chipS_Pin;
  delayMicroseconds(3);
  uint8_t temp[6], startAddr = 0x79;
  float quatFloat[3];
  
  AccGyr.Write_Reg(0x01, 0b00000000 + 0b00000000); //disable the embed reg access
  

  for(uint8_t j = 0; j < 3; j++)
  {
    for(uint8_t i = 0; i < 6; i++)
    {
      AccGyr.Read_Reg(startAddr + i, &temp[i]);
    }
    quatData[j] = (uint16_t)(temp[j*2+1]<<8) + (uint16_t)temp[j*2];
    //quatFloat[j] = (float)quatData[j]/65536.0;
  }
  //Serial.print("Quat X: "); Serial.print(quatData[0]); Serial.print(" Quat Y: "); Serial.print(quatData[1]); Serial.print(" Quat Z: "); Serial.println(quatData[2]);
  //Serial.print(quatData[0]); Serial.print("  "); Serial.print(quatData[1]); Serial.print("  "); Serial.println(quatData[2]); 
}

void getAccRaw(){
  uint8_t temp[6], startAddr = 0x28;
  int16_t accData[3];
  
  for(uint8_t j = 0; j < 3; j++)
  {
    for(uint8_t i = 0; i < 6; i ++)
    {
      AccGyr.Read_Reg(startAddr + i, &temp[i]);
    }

    accData[j] = (uint16_t)(temp[j*2+1]<<8) + (uint16_t)temp[j*2];
    
  }
  //accData[0] = (uint16_t)(temp[1]<<8) + (uint16_t)temp[0];  Serial.print("Acc Z: ");  Serial.println(accData[0]);
  Serial.print("Acc X: ");  Serial.print(accData[0]); Serial.print(" Acc Y: ");  Serial.print(accData[1]);  Serial.print(" Acc Z: ");  Serial.print(accData[2]);     
}

void getGyrRaw(){
  uint8_t temp[6], startAddr = 0x22;
  int16_t gyrData[3];
  
  for(uint8_t j = 0; j < 3; j++)
  {
    for(uint8_t i = 0; i < 6; i ++)
    {
      AccGyr.Read_Reg(startAddr + i, &temp[i]);
    }
    gyrData[j] = (uint16_t)(temp[j*2+1]<<8) + (uint16_t)temp[j*2];
    
  }
  //accData[0] = (uint16_t)(temp[1]<<8) + (uint16_t)temp[0];  Serial.print("Acc Z: ");  Serial.println(accData[0]);
  Serial.print("     Gyr X: ");  Serial.print(gyrData[0]); Serial.print("   Gyr Y: ");  Serial.print(gyrData[1]);  Serial.print("   Gyr Z: ");  Serial.println(gyrData[2]);     
}

float calculate3DVecAngle(float vecA[], float vecB[]){ //calculates the angle in degrees between two vectors
  double result = atan2(vecA[1],vecA[0]) - atan2(vecB[1],vecB[0]);
  float degrees = result*(180.0/3.14);
  if(degrees < 0){ degrees = 360 + degrees; } //if in the 4th quadrant, add to 360 to get the correct angle
  return degrees;
}

void gravityVecToEuler(int16_t quatData[], uint8_t index, float output[][2]){ //uses the gravity vector to calculate euler angles
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

  output[index][0] = pitchTheta;  output[index][1] = rollTheta;

  //Serial.print(" Pitch angle: ");  Serial.print(pitchTheta); Serial.print(" Roll angle: ");  Serial.println(rollTheta); Serial.println("");
}

void calculateJointAng(float input[][2], uint8_t imu1, uint8_t imu2, float result[]){ //calculates the angle between two adjacent IMUs
  //get the pitch and yaw for the two IMUs
  float pitch1 = input[imu1][0], roll1 = input[imu1][1],
  pitch2 = input[imu2][0], roll2 = input[imu2][1];

  //subtract the angles and put it into the return values
    //calculating pitch
    if(pitch1 > pitch2)
    {
      result[0] = 360 - pitch1 + pitch2;
    }
    else if(pitch1 < pitch2)
    {
      result[0] = pitch2 - pitch1;
    }

    //calculating roll
    if(roll1 > roll2)
    {
      result[1] = 360 - roll1 + roll2;
    }
    else if(roll1 < roll2)
    {
      result[1] = roll2 - roll1;
    }
    
  //Serial.print("Pitch D: ");  Serial.print(result[0]);  Serial.print(" Roll D: ");  Serial.println(result[1]);
  Serial.print(imu1);  Serial.print(" ");  Serial.print(imu2);  Serial.print(" "); Serial.print(result[0]);  Serial.print(" ");  Serial.println(result[1]);
}



void loop() {
  uint8_t gameData[6];
  int16_t eulerAng[3], fifoOut[4];
  float outVals[32][2], jointAng[2];

  //checkGameRegs();

  //===Functions for game====
  //
  for(int i = 0; i < indexIMUs[i]; i++)
  {
    //Serial.print("CS: "); Serial.println(i, DEC);
    getFifoData(fifoOut, indexIMUs[i]);  gravityVecToEuler(fifoOut, indexIMUs[i], outVals);
  }
  calculateJointAng(outVals, indexIMUs[0], indexIMUs[0+1], jointAng);
  //calculateJointAng(outVals, 18, 19, jointAng);

  //unityDataPrep(quat);
  //quatToEuler(quat);
  //quat2EulerManual(quat);
  //delay(1);
  //
  //=========================

  //===Functions for AccGyro only======
  //
  //getAccRaw();  getGyrRaw();
  //delay(1);
  //
}

void checkGameRegs(){
  uint8_t tag, temp;
  uint16_t temp16;
  int32_t temp32;
  AccGyr.Write_Reg(0x01, 0b00000000 + 0b10000000); //enable the embed reg access
  AccGyr.Write_Reg(0x02, 0b00000001 + 0b00000000); //turning page to embed page
  Serial.println("Checking registers:");
  AccGyr.Read_Reg(0x04,&temp);  Serial.print("Game enable register:"); Serial.println(temp, BIN);
  AccGyr.Read_Reg(0x66,&temp);  Serial.print("Game initialise:"); Serial.println(temp, BIN);
  AccGyr.Read_Reg(0x44,&temp);  Serial.print("Game fifo batch:"); Serial.println(temp, BIN);
  AccGyr.Read_Reg(0x5E,&temp);  Serial.print("Game ODR:"); Serial.println(temp, BIN);
  AccGyr.Read_Reg(0x07,&temp);  Serial.print("Game running?(1 if true):"); Serial.println(temp, BIN);
}
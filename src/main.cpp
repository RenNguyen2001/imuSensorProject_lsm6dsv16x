#include <LSM6DSV16XSensor.h>
#include <math.h>
#include <SPI.h>

uint8_t CS_PIN  = 17;
//17,18 for the breadboard, 18 for the PCB
#define SPI_MOSI  12
#define SPI_MISO  11
#define SPI_SCK 13

#define chipSelect 4 //B0_03
#define readSensor 1
#define writeSensor 0
SPISettings settingsA(2000000, MSBFIRST, SPI_MODE3);

LSM6DSV16XSensor AccGyr(&SPI, CS_PIN);
uint8_t status = 0;
int16_t firstFrame[3];
uint8_t xRangeGlobal;

unsigned long currentTime = 0, previousTime = 0, elapsedTime = 0;
float yawAngleGlobal = 0;

void prereqSetup(uint8_t dataRSet, uint8_t gyroFiltS, uint8_t accFiltS, uint8_t xRange, uint8_t gRangeS, uint8_t csPin);
void regularSetup(int dataRSet);
void gameSetup(int gameODRSet);
void getFifoData(int16_t quatData[], int chipS_Pin);

void chipSelectSetup();
void SPItransmit(char rw, char address, char spiData);
void IMU_setup();
void IMU_read();

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


void setup() {
  uint8_t dRSet = 7;    //gyr and acc output data rate choice (refer to dR[])
  uint8_t gODRSet = 3;  //game vector output data rate choice (refer to gameODR[])
  uint8_t gFiltSetting = 4, xFiltSetting = 5; //gyro and acc filter bandwidth selection (refer to gF_LU240Hz[] and aF_LU_HP[])
  uint8_t xRangeS = 2, gRangeS = 4; //acc and gyro range setting (refer to accRange[] and gyrRange[])
  uint8_t csPin = 18;

  xRangeGlobal = xRangeS;

  Serial.begin(120000);
  SPI.begin();

  //chipSelectSetup();

  Serial.println("Setting up systems....");
  

  for(char i = 0; i < 3; i++)
  {
    AccGyr.cs_pin = thumb.imuCS_pins[i]; //changing the cs_pin from within the header file, made the cs_pin variable public
    prereqSetup(dRSet, gFiltSetting, xFiltSetting, xRangeS, gRangeS, csPin);
    
    gameSetup(gODRSet); //setup game
    delay(5);
  }

  for(char i = 0; i < 3; i++)
  {
    AccGyr.cs_pin = indexFinger.imuCS_pins[i]; //changing the cs_pin from within the header file, made the cs_pin variable public
    prereqSetup(dRSet, gFiltSetting, xFiltSetting, xRangeS, gRangeS, csPin);
    
    gameSetup(gODRSet); //setup game
    delay(5);
  }

  for(char i = 0; i < 3; i++)
  {
    AccGyr.cs_pin = middle.imuCS_pins[i]; //changing the cs_pin from within the header file, made the cs_pin variable public
    prereqSetup(dRSet, gFiltSetting, xFiltSetting, xRangeS, gRangeS, csPin);
    
    gameSetup(gODRSet); //setup game
    delay(5);
  }

  for(char i = 0; i < 3; i++)
  {
    AccGyr.cs_pin = ring.imuCS_pins[i]; //changing the cs_pin from within the header file, made the cs_pin variable public
    prereqSetup(dRSet, gFiltSetting, xFiltSetting, xRangeS, gRangeS, csPin);
    
    gameSetup(gODRSet); //setup game
    delay(5);
  }


  for(char i = 0; i < 3; i++)
  {
    AccGyr.cs_pin = pinkie.imuCS_pins[i]; //changing the cs_pin from within the header file, made the cs_pin variable public
    prereqSetup(dRSet, gFiltSetting, xFiltSetting, xRangeS, gRangeS, csPin);
    
    gameSetup(gODRSet); //setup game
    delay(5);
  }

  AccGyr.cs_pin = palm.imuCS_pins[0]; //changing the cs_pin from within the header file, made the cs_pin variable public
  prereqSetup(dRSet, gFiltSetting, xFiltSetting, xRangeS, gRangeS, csPin);
    
  gameSetup(gODRSet); //setup game
  delay(5);
}

void chipSelectSetup(){ //see pages 948 and 957 in the manual
  // 1. Set pads to GPIO mode using IOMUX registers 
  //find your pin in the schematic, pin 13 was named as B0_03/led pin
    //IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 5; 
    //IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_04 = 5; //GPIO4

    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0b011; //LPSPI4
    uint8_t speedSet = (0b00<<6), drvStrength = (0b001<<3), slewRate = 0, hysSet = (0b1<<16);
    //IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_04 = drvStrength | speedSet | slewRate | hysSet; //setting drive strength
    //IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 = drvStrength;

  // 2. Setting GPIO as standard mode instead of high speed
  //e.g. regular and high speed GPIO are shared, IOMUXC_GPR is used to determine if regular or
  //high speed is selected
  //IOMUXC_GPR_GPR26-29 determine what mode is selected
    //IOMUXC_GPR_GPR27 = 0x00000000;
    //IOMUXC_GPR_GPR29 = 0x00;

    IOMUXC_LPSPI4_SCK_SELECT_INPUT = 0;  //selecting alt3 for BO_03

  // 3. Setting the data direction register (make IO an output)
    //GPIO2_GDIR |= (1 << chipSelect);
    //GPIO4_GDIR |= (1 << chipSelect);
    //GPIO4_DR_SET = (1 << chipSelect);
}


void prereqSetup(uint8_t dataRSet, uint8_t gyroFiltS, uint8_t accFiltS, uint8_t xRangeS,
uint8_t gRangeS, uint8_t csPin){ //setup required for both game vector or acc & gyro only mode
  // Initialize LSM6DSV16X.
  AccGyr.begin();

  //AccGyr.Write_Reg(0x03, 0b1);  //disable i2c
  //AccGyr.Write_Reg(0x03, 0b1 + (0b1<<7)); //disable I2C and I3C, and enable the pull up on sda
  //AccGyr.Write_Reg(0x02, 0b1<<6); //enabling the pull up resistor on the MISO line

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
  delayMicroseconds(10);
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

void spiTransfer(uint8_t address, uint8_t data){
  SPI.beginTransaction(settingsA);
  GPIO4_DR_CLEAR = (1 << chipSelect);
  SPI.transfer(address);  SPI.transfer(data);
  GPIO4_DR_SET = (1 << chipSelect);
  SPI.endTransaction();
}

uint8_t spiRead(uint8_t address){
  SPI.beginTransaction(settingsA);
  GPIO4_DR_CLEAR = (1 << chipSelect);
  SPI.transfer(address);
  uint8_t spiData = SPI.transfer(0x00);
  GPIO4_DR_SET = (1 << chipSelect);
  SPI.endTransaction();
  return spiData;
}

void fifoSPIManual(){
  int16_t quatData[4];

  spiTransfer(0x01, 0x00);  //disable the embed reg access

  uint8_t temp[6], startAddr = 0x79;

  for(uint8_t i = 0; i < 6; i++)
  {
    temp[i] = spiRead(0b10000000 + startAddr + i); 
  }
    
  for(uint8_t i = 0; i < 3; i++)
  {
    quatData[i] = (uint16_t)(temp[i*2+1]<<8) + (uint16_t)temp[i*2];
  }
  Serial.print(quatData[0]); Serial.print("  "); Serial.print(quatData[1]); Serial.print("  "); Serial.println(quatData[2]);
  delayMicroseconds(100);
}



void getAccRaw(){
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
    accValue[j] = accData[j] * accSen[2];
  }
  //accData[0] = (uint16_t)(temp[1]<<8) + (uint16_t)temp[0];  Serial.print("Acc Z: ");  Serial.println(accData[0]);
  //Serial.print("Acc X: ");  Serial.print(accData[0]); Serial.print(" Acc Y: ");  Serial.print(accData[1]);  Serial.print(" Acc Z: ");  Serial.println(accData[2]); 
  Serial.print("Acc X: ");  Serial.print(accValue[0]); Serial.print(" Acc Y: ");  Serial.print(accValue[1]);  Serial.print(" Acc Z: ");  Serial.println(accValue[2]);     
}

void getGyrRaw(int* gyrOutput, int chipS_Pin){
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
    gyrValue[j] = (float)gyrData[j] * gyrSen[4]; 
  }
  *gyrOutput = gyrValue[2];  
  //accData[0] = (uint16_t)(temp[1]<<8) + (uint16_t)temp[0];  Serial.print("Acc Z: ");  Serial.println(accData[0]);
  //Serial.print("     Gyr X: ");  Serial.print(gyrData[0]); Serial.print("   Gyr Y: ");  Serial.print(gyrData[1]);  Serial.print("   Gyr Z: ");  Serial.println(gyrData[2]);  
  //Serial.print("     Gyr X: ");  Serial.print(gyrValue[0]); Serial.print("   Gyr Y: ");  Serial.print(gyrValue[1]);  Serial.print("   Gyr Z: ");  Serial.println(gyrValue[2]);   
}

void getYawAng(){
  //start counting time
  currentTime = millis();
  elapsedTime = currentTime - previousTime;

  //sample variables
  float sample,
  sampleFrequency = 10,
  samplePeriod, result;
  samplePeriod = (1/sampleFrequency);  //in seconds

  int gyrData;

  //take one sample at the end of each samplingPeriod
  if(elapsedTime >= (int)(samplePeriod*1000))
  {
    previousTime = currentTime;
    elapsedTime = 0;

    getGyrRaw(&gyrData, palm.imuCS_pins[0]);
    sample = gyrData;  //Serial.print("Gyr Data 2: ");  Serial.println(gyrData); //gyr data is outputting fine
    result = (sample)*(samplePeriod); //Serial.print("Integral: ");  Serial.println(result);
    yawAngleGlobal = yawAngleGlobal + result;  Serial.print("Yaw Angle: ");  Serial.println(-yawAngleGlobal);
  }
}

void delayFunc(){
  
}

float calculate3DVecAngle(float vecA[], float vecB[]){ //calculates the angle in degrees between two vectors
  double result = atan2(vecA[1],vecA[0]) - atan2(vecB[1],vecB[0]);
  float degrees = result*(180.0/3.14);
  if(degrees < 0){ degrees = 360 + degrees; } //if in the 4th quadrant, add to 360 to get the correct angle
  return degrees;
}

void gravityVecToEuler(int16_t quatData[], uint8_t indexNo, float output[][2]){ //uses the gravity vector to calculate euler angles
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

void calculateJointAng(float input[][2], uint8_t imu1, uint8_t imu2, float result[]){ //calculates the angle between two adjacent IMUs
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
    getFifoData(fifoOut, imuArr[fingerNum].imuCS_pins[i]);  gravityVecToEuler(fifoOut, imuArr[fingerNum].imuCS_pins[i], outVals);
    //Serial.print(imuArr[fingerNum].fingerName); Serial.print(imuArr[fingerNum].imuCS_pins[i], DEC); Serial.print("  ");
    //Serial.print(fifoOut[0]); Serial.print("  "); Serial.print(fifoOut[1]); Serial.print("  "); Serial.println(fifoOut[2]);
  }
  getFifoData(fifoOut, imuArr[PALM].imuCS_pins[0]); gravityVecToEuler(fifoOut, imuArr[PALM].imuCS_pins[0], outVals);  
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

void readSingleIMUstripJoint(int16_t fifoOut[4], float outVals[][2], float jointAng[], const uint8_t fingerNum){
  readSingleIMUstrip(fifoOut, outVals, fingerNum);
  Serial.print(fingerNum);
  for(char i = 0; i < 3; i++)
  {
    calculateJointAng(outVals, jointArr[fingerNum][i].adjacentIMU_cs[0], jointArr[fingerNum][i].adjacentIMU_cs[1], jointAng);
    //Serial.print(jointArr[fingerNum][i].jointName);  Serial.print(" "); Serial.print((String)jointAng[0]);  Serial.print(" ");
    // finger strip num, jointAng1(tip), jointAng2(mid), jointAng3(base)
    Serial.print(" "); Serial.print((String)jointAng[0]); 
  }
  Serial.print(" ");  Serial.print((String)outVals[imuArr[PALM].imuCS_pins[0]][1]);
  Serial.println(" ");
}

void readMultiIMUstripJoints(int16_t fifoOut[4], float outVals[][2], float jointAng[]){
  readSingleIMUstripJoint(fifoOut, outVals, jointAng, THUMB);
  readSingleIMUstripJoint(fifoOut, outVals, jointAng, INDEX);
  readSingleIMUstripJoint(fifoOut, outVals, jointAng, MIDDLE);
  readSingleIMUstripJoint(fifoOut, outVals, jointAng, RING);
  readSingleIMUstripJoint(fifoOut, outVals, jointAng, PINKIE);
}


void loop() {
  int16_t fifoOut[4];
  float outVals[40][2], jointAng[2];  //outvals is to store pitch and roll
  int gyrOutput;

  //=========================reading joints=============================
    //readSingleIMUstripJoint(fifoOut, outVals, jointAng, INDEX);
    //readMultiIMUstripJoints(fifoOut, outVals, jointAng);
  //====================================================================
  
  //====================reading individual IMUS=========================
    //readSingleIMUstrip(fifoOut, outVals, INDEX); 
    //readSingleIMUstripPrint(fifoOut, outVals, INDEX);
    //readSingleIMU(fifoOut, outVals, PALM, TIP); //readSingleIMU(fifoOut, outVals, INDEX, TIP);
    //readMultiIMUstrips(fifoOut, outVals);
    //fifoSPIManual();
  //====================================================================

  //===Functions for AccGyro only======
    //getAccRaw();  
    //getGyrRaw(&gyrOutput, palm.imuCS_pins[0]);
    getYawAng();
    //delay(1);
  //===================================
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
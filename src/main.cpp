#include <LSM6DSV16XSensor.h>
#include <math.h>

#define SENSOR_ODR 120.0f // In Hertz
#define ACC_FS 2 // In g
#define GYR_FS 4000 // In dps
//FS is range
#define MEASUREMENT_TIME_INTERVAL (1000.0f/SENSOR_ODR) // In ms
#define FIFO_SAMPLE_THRESHOLD 199
#define FLASH_BUFF_LEN 8192

#define CS_PIN 25
#define SPI_MOSI  11
#define SPI_MISO  12
#define SPI_SCK 13

LSM6DSV16XSensor AccGyr(&SPI, CS_PIN);
uint8_t status = 0;

void prereqSetup(uint8_t dataRSet, uint8_t gyroFiltS, uint8_t accFiltS, uint8_t xRange, uint8_t gRangeS);
void regularSetup(int dataRSet);
void gameSetup(int gameODRSet);

//const arrays to store values

const uint8_t dR[] = { //all data Rates are in Hz
  0,  //off
  1,  //1.875
  2,  //7.5
  3,  //15
  4,  //30
  5,  //60
  6,  //120
  7,  //240
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

const uint8_t gyrRange[] = {  //the comments below represent the range (in dps)
  0b0000, //125
  0b0001, //250
  0b0010, //500
  0b0011, //1000
  0b0100, //2000
  0b1100  //4000
};

void setup() {
  uint8_t dRSet = 7;    //gyr and acc output data rate choice (refer to dR[])
  uint8_t gODRSet = 3;  //game vector output data rate choice (refer to gameODR[])
  uint8_t gFiltSetting = 4, xFiltSetting = 5; //gyro and acc filter bandwidth selection (refer to gF_LU240Hz[] and aF_LU_HP[])
  uint8_t xRangeS = 1, gRangeS = 2; //acc and gyro range setting (refer to accRange[] and gyrRange[])

  Serial.begin(115200);
  SPI.begin();

  prereqSetup(dRSet, gFiltSetting, xFiltSetting, xRangeS, gRangeS);
  
  gameSetup(gODRSet);
  //regularSetup(dRSet);
}

void prereqSetup(uint8_t dataRSet, uint8_t gyroFiltS, uint8_t accFiltS, uint8_t xRangeS,
uint8_t gRangeS){ //setup required for both game vector or acc & gyro only mode
  // Initialize LSM6DSV16X.
  AccGyr.begin();

  AccGyr.Write_Reg(0x10, dR[dataRSet] + 0b00010000); //enable acc by setting the odr (240Hz) and setting to high acc ODR mode
  AccGyr.Write_Reg(0x11, dR[dataRSet] + 0b00010000); //enable gyro by setting the odr (240Hz) and setting to high acc ODR mode

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

void getFifoData(int16_t quatData[]){
  uint8_t temp[6], startAddr = 0x79;
  
  AccGyr.Write_Reg(0x01, 0b00000000 + 0b00000000); //disable the embed reg access
  

  for(uint8_t j = 0; j < 3; j++)
  {
    for(uint8_t i = 0; i < 6; i ++)
    {
      AccGyr.Read_Reg(startAddr + i, &temp[i]);
    }
    quatData[j] = (uint16_t)(temp[j*2+1]<<8) + (uint16_t)temp[j*2];
  }
  //Serial.print("Quat X: "); Serial.print(quatData[0]); Serial.print(" Quat Y: "); Serial.print(quatData[1]); Serial.print(" Quat Z: "); Serial.println(quatData[2]); 
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

void quatToEuler(int16_t q[]){
  float eulerResult[3];
  float eulerDegrees[3];
  int16_t quatDat[4];

  quatDat[0] = 1;
  quatDat[1] = q[0];  quatDat[2] = q[1]; quatDat [4] = q[2];
  
  //Roll calculation
  eulerResult[0] = atan2( ( 2*(quatDat[0]*quatDat[1] + (quatDat[2]*quatDat[3]) ) ) 
  , (sq(quatDat[0]) + sq(quatDat[3]) - sq(quatDat[1]) - sq(quatDat[2])) ); //you can't square numbers like this "x^2", this is just "x+2" in c

  //Pitch calculation
  eulerResult[1] =  asin(    2*((quatDat[0]*quatDat[2])-(quatDat[1]*quatDat[3])) );

  //Yaw calculation
  eulerResult[2] =  atan2(   (2*(quatDat[0]*quatDat[3]+(quatDat[1]*quatDat[2]))) ,
   (sq(quatDat[0])+sq(quatDat[1])-sq(quatDat[2])-sq(quatDat[3])) );

  //Conversion to degrees
  for(uint8_t i = 0; i < 3; i++)
  {
    eulerDegrees[i] = (float)eulerResult[i] * (float)(180/3.14);  
  }

  //Serial.print("Roll (radians):"); Serial.print(eulerResult[0]);  Serial.print(" Pitch (radians):"); Serial.print(eulerResult[1]); Serial.print(" Yaw (radians):"); Serial.println(eulerResult[2]);
  Serial.print("Roll (degrees):"); Serial.print(eulerDegrees[0]);  Serial.print(" Pitch (degrees):"); Serial.print(eulerDegrees[1]); Serial.print(" Yaw (degrees):"); Serial.println(eulerDegrees[2]);
}

void loop() {
  uint8_t gameData[6];
  int16_t eulerAng[3], quat[4];

  //checkGameRegs();

  //===Functions for game====
  //
  getFifoData(quat);
  //unityDataPrep(quat);
  quatToEuler(quat);
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
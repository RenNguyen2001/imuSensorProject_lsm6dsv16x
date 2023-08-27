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
volatile uint8_t fullFlag = 0; // FIFO full flagn 
uint8_t status = 0;
unsigned long timestamp_count = 0;
bool acc_available = false;
bool gyr_available = false;
int32_t acc_value[3];
int32_t gyr_value[3];
char buff[FLASH_BUFF_LEN];
uint32_t pos = 0;

void prereqSetup(uint8_t dataRSet, uint8_t gyroFiltS, uint8_t accFiltS, uint8_t xRange);
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

void setup() {
  uint8_t dRSet = 7;    //gyr and acc output data rate choice (refer to dR[])
  uint8_t gODRSet = 3;  //game vector output data rate choice (refer to gameODR[])
  uint8_t gFiltSetting = 4, xFiltSetting = 5; //gyro and acc filter bandwidth selection (refer to gF_LU240Hz[] and aF_LU_HP[])
  uint8_t xRange = 0; //acc and gyro range (refer to accRange[] and)

  Serial.begin(115200);
  SPI.begin();

  prereqSetup(dRSet, gFiltSetting, xFiltSetting, xRange);
  
  //gameSetup(gODRSet);
  regularSetup(dRSet);
}

void prereqSetup(uint8_t dataRSet, uint8_t gyroFiltS, uint8_t accFiltS, uint8_t xRange){ //setup required for both game vector or acc & gyro only mode
  // Initialize LSM6DSV16X.
  AccGyr.begin();

  AccGyr.Write_Reg(0x10, dR[dataRSet] + 0b00010000); //enable acc by setting the odr (240Hz) and setting to high acc ODR mode
  AccGyr.Write_Reg(0x11, dR[dataRSet] + 0b00010000); //enable gyro by setting the odr (240Hz) and setting to high acc ODR mode

  // Setting the output data rate configuration register for HAODR
  AccGyr.Write_Reg(0x62, 0b00);  //setting the high acc ODR data rate

  // Configure FS of the acc and gyro
  AccGyr.Write_Reg(0x01, 0b00000000 + 0b00000000); //disable the embed reg access
  status |= AccGyr.Set_G_FS(GYR_FS);

  // Setting the filters and scale
  AccGyr.Write_Reg(0x18, (1<<5) + (0<<4)); //turning on fast settling mode and selecting high pass for accelereometer
  AccGyr.Write_Reg(0x17, accRange[xRange] + (accFiltS<<5));  //setting the scale to +-2g and bandwidth
  AccGyr.Write_Reg(0x15, (gF_LU240Hz[gyroFiltS]<<4));  //setting the gyroscope lp filter bandwidth
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
  uint8_t fifoData[3], tag, temp;;
  uint16_t temp16;
  float quatVal[3];
  
  
  AccGyr.Write_Reg(0x01, 0b00000000 + 0b00000000); //disable the embed reg access
  //AccGyr.FIFO_Get_Tag(&tag);  Serial.print("Tag: "); Serial.println(tag, HEX);
  //AccGyr.FIFO_Get_Full_Status(&temp);  Serial.print("Status: "); Serial.println(temp);
  //AccGyr.FIFO_Get_Num_Samples(&temp16); Serial.print("Num of Samples: "); Serial.println(temp16);
  AccGyr.FIFO_Get_Data(fifoData); //Serial.println("Game Vector: ");   //for some reason this command causes the fifo tag and num of samples to be 0

  for(int i = 0; i < 3; i++)
  {
     quatData[i] = (int16_t)(fifoData[i*2] << 8) + (int16_t)fifoData[i*2+1];
     //quatVal[i] = quatData/65536;
     //quatData[i] = (fifoData[i*2]) + (int16_t)(fifoData[i*2+1] << 8);
     //Serial.print("Gamedata No. ");  Serial.print(i); Serial.print(" :"); Serial.println(quartData[i]);
  }
  Serial.print("Quat X: "); Serial.print(quatData[0]); Serial.print(" Quat Y: "); Serial.print(quatData[1]); Serial.print(" Quat Z: "); Serial.println(quatData[2]); 
  //Serial.print("quatData[0] (byte check):"); Serial.println(quatData[0],BIN); Serial.print("quatData[0] (hex check):"); Serial.println(quatData[0],HEX);

  //Serial.print("X: "); Serial.println(quatData[0]);
}

void get_AccGyro(uint8_t gameData[]){
  int32_t gyroVal[3], accVal[3];

  AccGyr.FIFO_Get_X_Axes(accVal); Serial.print(" AccX:"); Serial.print(accVal[0]);  Serial.print(" AccY:"); Serial.print(accVal[1]);  Serial.print(" AccZ:"); Serial.println(accVal[2]);
  //AccGyr.FIFO_Get_G_Axes(gyroVal); Serial.print(" GyrX:");  Serial.print(gyroVal[0]); Serial.print(" GyrY:");  Serial.print(gyroVal[1]); Serial.print(" GyrZ:");  Serial.println(gyroVal[2]);
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


    //accData[0] = (uint16_t)(temp[1]<<8) + (uint16_t)temp[0];  //x axis
    //accData[1] = (uint16_t)(temp[3]<<8) + (uint16_t)temp[2];  //y axis
    //accData[2] = (uint16_t)(temp[5]<<8) + (uint16_t)temp[4];  //z axis

    accData[j] = (uint16_t)(temp[j*2+1]<<8) + (uint16_t)temp[j*2];
    
  }
  //accData[0] = (uint16_t)(temp[1]<<8) + (uint16_t)temp[0];  Serial.print("Acc Z: ");  Serial.println(accData[0]);
  Serial.print("Acc X: ");  Serial.print(accData[0]); Serial.print(" Acc Y: ");  Serial.print(accData[1]);  Serial.print(" Acc Z: ");  Serial.println(accData[2]);     
}

void getGyrRaw(){
  uint8_t temp[6], startAddr = 0x22;
  int16_t accData[3];
  
  for(uint8_t j = 0; j < 3; j++)
  {
    for(uint8_t i = 0; i < 6; i ++)
    {
      AccGyr.Read_Reg(startAddr + i, &temp[i]);
    }


    //accData[0] = (uint16_t)(temp[1]<<8) + (uint16_t)temp[0];  //x axis
    //accData[1] = (uint16_t)(temp[3]<<8) + (uint16_t)temp[2];  //y axis
    //accData[2] = (uint16_t)(temp[5]<<8) + (uint16_t)temp[4];  //z axis

    accData[j] = (uint16_t)(temp[j*2+1]<<8) + (uint16_t)temp[j*2];
    
  }
  //accData[0] = (uint16_t)(temp[1]<<8) + (uint16_t)temp[0];  Serial.print("Acc Z: ");  Serial.println(accData[0]);
  Serial.print("Gyr X: ");  Serial.print(accData[0]); Serial.print(" Gyr Y: ");  Serial.print(accData[1]);  Serial.print(" Gyr Z: ");  Serial.println(accData[2]);     
}

void quatToEuler(int16_t q[]){
  float eulerResult[3];
  float eulerDegrees[3];

  //q[0] = 18733; q[1] = -6621; q[2] = -16993;  q[3] = -1;
  q[3] = 1;
  //Roll calculation
  eulerResult[0] = atan2( ( 2*(q[0]*q[1] + (q[2]*q[3]) ) ) , (sq(q[0]) + sq(q[3]) - sq(q[1]) - sq(q[2])) ); //you can't square numbers like this "x^2", this is just "x+2" in c
  eulerDegrees[0] = (float)eulerResult[0] * (float)(180/3.14);  //converting from radians to degrees

  //Pitch calculation
  eulerResult[1] =  asin(    2*((q[0]*q[2])-(q[1]*q[3])) );

  //Yaw calculation
  eulerResult[2] =  atan2(   (2*(q[0]*q[3]+(q[1]*q[2]))) , (sq(q[0])+sq(q[1])-sq(q[2])-sq(q[3])) );

  Serial.print("Roll (radians):"); Serial.print(eulerResult[0]);  Serial.print(" Pitch (radians):"); Serial.print(eulerResult[1]); Serial.print(" Yaw (radians):"); Serial.println(eulerResult[2]);
  //Serial.print("Euler ang x (degrees):"); Serial.println(eulerDegrees[0]);
}

void loop() {
  uint8_t gameData[6];
  int16_t eulerAng[3], quat[4];

  //checkGameRegs();

  //===Functions for game====
  //
  //getFifoData(quat);
  //unityDataPrep(quat);
  //quatToEuler(quat);
  //delay(1);
  //
  //=========================

  //===Functions for AccGyro only======
  //
  //get_AccGyro(gameData);
  getAccRaw();
  //getGyrRaw();
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
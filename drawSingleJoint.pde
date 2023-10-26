//creating a simple program to represent one joint
import processing.opengl.*;
import processing.serial.*;

Serial port;
String serialData;
String[][] serialStrings = new String[5][3];  //5 fingers, 3 values (the joint angles)

float grid = 400;

class imuData{
    int index;
    float[] angle = new float[3];
}

void setup(){
    size(900,900);
    setup2D();
    serialSetup();
    
}

void serialSetup(){
    for(int j = 0; j < 5; j++)
    {
    for(int i = 0; i < 3; i++)
        {
        serialStrings[j][i] = "h";  //initialising the values so it doesn't become Null
        }
    }
  
    port = new Serial(this, "COM10", 120000);  port.bufferUntil('\n');
}

void draw(){
    imuData imuDataObj = new imuData();  //using this object to pass by reference
    getSerialData(imuDataObj);  println(imuDataObj.index);
    background(15,20,30);
    if(imuDataObj.index == 1)
    {
        drawFinger2D(imuDataObj.angle, width/2, height/2);
    }
    
}

void setup2D(){
    rectMode(CORNER);
}

void drawFinger2D(float jointAng[], float x, float y){
    int fingerLen = 100, fingerWid = 10;

    //drawing the palm/hand
    push();
    translate(x-fingerLen, y);
    rotate(0);
    rect(0,0,fingerLen,fingerWid);
    rectMode(CORNER);
    pop();

    //part of the finger that connects to the hand
    push();
    translate(x, y);
    rotate(radians(jointAng[2]));
    rect(0,0,fingerLen,fingerWid);
    rectMode(CORNER);
    pop();

    //need to calculate the end point that phalange rotates to
    //calculate the arc
    float xCoord = x + fingerLen*cos(radians(jointAng[2]));
    float yCoord = y + fingerLen*sin(radians(jointAng[2]));

    push();
    translate(xCoord, yCoord);
    rotate(radians(jointAng[2]+jointAng[1]));
    rect(0,0,fingerLen,fingerWid);
    rectMode(CORNER);
    pop();

    //find the end point of the phalange
    //the front of the phallenge is given be point (xCoord,yCoord)
    //rotation will be around this point, this point will move

    float xCoord2 = xCoord + fingerLen*cos(radians(jointAng[2]+jointAng[1]));
    float yCoord2 = yCoord + fingerLen*sin(radians(jointAng[2]+jointAng[1]));

    push();
    translate(xCoord2, yCoord2);
    rotate(radians(jointAng[2]+jointAng[1]+jointAng[0]));
    rect(0,0,fingerLen,fingerWid);
    rectMode(CORNER);
    pop();

    println(jointAng);
}


void getSerialData(imuData imuObj){
  if( port.available() > 0) // If data is available,
  { 
    serialData = port.readStringUntil('\n');         // read the entire string until the newline
    String[] receiveVar = new String[4];
    int indexNo;
    //splitting the string
    receiveVar = split(serialData, " ");  // index 0 finger strip num, 1 jointAng1(tip), 2 jointAng2(mid), 3 jointAng3(base) 
    imuObj.index = int(receiveVar[0]);

    //storing the values into an imu object
    serialStrings[imuObj.index][0] = receiveVar[1];  serialStrings[imuObj.index][1] = receiveVar[2];  serialStrings[imuObj.index][2] = receiveVar[3];
    imuObj.angle[0] = parseFloat(receiveVar[1]);    imuObj.angle[1] = parseFloat(receiveVar[2]);    imuObj.angle[2] = parseFloat(receiveVar[3]);
    port.clear();
  }
}
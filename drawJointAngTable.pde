import processing.serial.*;

Serial port;
String serialData;
String[][] serialStrings = new String[5][3];  //5 fingers, 3 values (the joint angles)
int tableSize[] = {1000,500};

Table table;

void setup(){
  for(int j = 0; j < 5; j++)
  {
    for(int i = 0; i < 3; i++)
    {
      serialStrings[j][i] = "h";  //initialising the values so it doesn't become Null
    }
  }
  
  size(1000,500);
  port = new Serial(this, "COM10", 120000);  port.bufferUntil('\n');
}

void draw() {
  int indexNo = 0;
  getSerialData(indexNo);
  drawTable(indexNo);
}

void getSerialData(int indexNo){
  if( port.available() > 0)
  { // If data is available,
    serialData = port.readStringUntil('\n');         // read the entire string until the newline
    
    String[] receiveVar = new String[4];
    //splitting the string
    receiveVar = split(serialData, " ");  // index 0 finger strip num, 1 jointAng1(tip), 2 jointAng2(mid), 3 jointAng3(base) 
    indexNo = int(receiveVar[0]);
    serialStrings[indexNo][0] = receiveVar[1];  serialStrings[indexNo][1] = receiveVar[2];  serialStrings[indexNo][2] = receiveVar[3];
    println(receiveVar); 
    port.clear();
  }
}

void drawTable(int indexNo){
  //table will consist of 4 columns (finger strip num, jointAng1, jointAng2, jointAng3)
  //5 rows (one for each finger strip)
  
  //draw the 3 vertical lines and 4 horizontal lines
    //calculating x coordinates of the lines
  background(0);
  char numOfYlines = 5;  //number of columns is +1 of this
  char numOfXlines = 3;  //number of rows is +1 of this
  int numOfCells = (numOfYlines+1)*(numOfYlines+1);
  int[] xCoord = new int[numOfXlines];
  int[] yCoord = new int[numOfYlines];
  int[][] cellCoord = new int[(numOfYlines+1)*(numOfYlines+1)][2];
  int[] cellCenter = new int[2];
  
  xCoord[0] = (tableSize[0]/(numOfXlines+1));
  yCoord[0] = (tableSize[1]/(numOfYlines+1));
  
  for(int i = 0; i < numOfXlines; i++)
  {
   xCoord[i] = xCoord[0]*(i+1);
   line(xCoord[i], 0, xCoord[i], tableSize[1]);
  }
  for(int i = 0; i < numOfYlines; i++)
  {
    yCoord[i] = yCoord[0]*(i+1);
    line(0, yCoord[i], tableSize[0], yCoord[i]);
  }
  
  //find the coordinates of the center of each cell
  cellCenter[0] = xCoord[0]/2;  cellCenter[1] = yCoord[0]/2;
  
  String[] s = new String[4];
  String[] fingers = new String[5];
  s[0] = "Finger";  s[1] = "JointAng1";  s[2] = "JointAng2";  s[3] = "JointAng3";
  fingers[0] = "THUMB";  fingers[1] = "INDEX";  fingers[2] = "MIDDLE";  fingers[3] = "RING";  fingers[4] = "PINKIE";
  
  for(int j = 0; j < 6; j++)
  {
    for(int i = 0; i < 4; i++)
    {
      cellCoord[i+(4*j)][0] = cellCenter[0] + xCoord[0]*i;  cellCoord[i+(4*j)][1] = (cellCenter[1]) + yCoord[0]*j;
    }
  }
  
  for(int i = 0; i < numOfXlines + 1; i++)  //printing out the data titles at the top row
  {
    textSize(30);
    textAlign(CENTER, CENTER);
    text(s[i], cellCoord[i][0], cellCoord[i][1]);
  }
  
  for(int i = 0; i < 5; i++)  //printing out the names of the fingers
  {
    textSize(30);
    textAlign(CENTER, CENTER);
    text(fingers[i], cellCoord[4+i*4][0], cellCoord[4+i*4][1]);
  }
  
 for(int j = 0; j < 5; j++) //printing out the angles/data
 {
    for(int i = 0; i < 3; i++)
    {
      textSize(25);
      textAlign(CENTER, CENTER);
      text(serialStrings[j][i], cellCoord[i+5+4*j][0], cellCoord[i+5+4*j][1]);
    }
 }
  
}

void fillTable(){
  
}

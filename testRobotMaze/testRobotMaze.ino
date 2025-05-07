#include <Robot.h>
#include <Wire.h>
#include <BluetoothSerial.h>

#define BUTTON_PIN 23
#define LED_PIN 15
#define N 0
#define E 1
#define S 2
#define W 3

const int wallThreshold = 3000;
const int motorVelocity = 50;
const int distanceToReach = 240;
const int angleToReach = 90;
const int maxValueFloodFill = 25;
int destinationX = 2; // 4
int destinationY = 2; // 4
int currentX = 0;
int currentY = 0;
bool start = 1;
int task = 0;
int step = 0;
char receivedMsg;
int nextDirection = 0;
int chosenAlgorithm = 1;

BluetoothSerial SerialBT;
Robot robot;

// 0b?WSEN << 1=N, 2=E, 3=S, 4=W, 5=?
unsigned char orientedWalls;
int floodFill[5][5];
unsigned char mazeMatrix[5][5] = {
{0b01001, 0b00001, 0b00001, 0b00001, 0b00011},
{0b01000, 0b00000, 0b00000, 0b00000, 0b00010},
{0b01000, 0b00000, 0b00000, 0b00000, 0b00010},
{0b01000, 0b00000, 0b00000, 0b00000, 0b00010},
{0b01100, 0b00100, 0b00100, 0b00100, 0b00110}
};

String mazeLookup[11] = {
{"+---+---+---+---+---+"},
{"|                   |"},
{"+   +   +   +   +   +"},
{"|                   |"},
{"+   +   +   +   +   +"},
{"|                   |"},
{"+   +   +   +   +   +"},
{"|                   |"},
{"+   +   +   +   +   +"},
{"|                   |"},
{"+---+---+---+---+---+"}
};

unsigned int orient; // 0=N, 1=E, 2=S, 3=W

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);

  SerialBT.begin("ESP32");

  if (!robot.init()) {
    SerialBT.println("Initialization of robot failed");
    while (1)
      ;
  }
  randomSeed(analogRead(0));
  delay(1000);
} 

void loop() {
  if (SerialBT.available()) {
    receivedMsg = SerialBT.read();
    if (receivedMsg == '1') {
      start = true;
      SerialBT.println(">Start<");
    } else if (receivedMsg == '2') {
      task = 0;
      SerialBT.println(">Stop<");
    } else if (receivedMsg == '3') {
      chosenAlgorithm = 1;
      SerialBT.println("Flood Fill");
    } else if (receivedMsg == '4') {
      chosenAlgorithm = 2;
      SerialBT.println("Left Wall Following");
    } else if (receivedMsg == '5') {
      chosenAlgorithm = 3;
      SerialBT.println("Random");
    } else
      start = false;
  }
  
  switch (task) {
    case 0: {// after stop
      if (SerialBT.connected()) {
        robot.rideStop();
        robot.setInitMove(true);
        digitalWrite(LED_PIN, LOW);
        step = 1;
        orient = E;
        currentX = 0;
        currentY = 0;
        start = false;
        initializeMaze(); // also for flood fill
        task = 1;
        SerialBT.println("First choose algorithm:\n3) Flood Fill\n4) Left Wall Following\n5) Random");
        SerialBT.println("And use 1) to start, 2) to stop");
        SerialBT.println();
      }
      break;
    }
    case 1: {// waiting for start
      if(start) {
        task = 2;
        start = false;
      }
      break;
    }
    case 2: {// check walls and update the maze
      orientedWalls = checkWalls();
      updateMaze();
      updateMazeLookup();
      SerialBT.print("\nStep number " + (String)step) + ":";
      showMaze(mazeLookup);

      if(currentX == destinationX && currentY == destinationY)
        task = 7;
      else {
        task = 3;
        step++;
      }
      break;
    }
    case 3: {// choose direction
      if(chosenAlgorithm == 1) { // Flood Fill
        updateFloodFill();
        nextDirection = moveToNextCellFloodFill(); 
      } else if(chosenAlgorithm == 2) // Wall Following
        nextDirection = moveToNextCellWallFollowing(); 
      else if(chosenAlgorithm == 3) // Random Algorithm
        nextDirection = moveToNextCellRandom(); 

      task = 4;
      delay(1000);
      break;
    }
    case 4: {// turn
      digitalWrite(LED_PIN, HIGH);
      if(nextDirection == S) {
        if(robot.moveAngular(angleToReach)) {
          digitalWrite(LED_PIN, LOW);
          task = 5;
          delay(500);
        }
      } else if(nextDirection == E || nextDirection == W) {
        if(robot.moveAngular(angleToReach * (nextDirection-2))) {
          digitalWrite(LED_PIN, LOW);
          task = 6;
          delay(1000);
        }
      } else {
        digitalWrite(LED_PIN, LOW);
        task = 6;
      }
      break;
    }
    case 5: {// turn around
      digitalWrite(LED_PIN, HIGH);
      if(robot.moveAngular(angleToReach)) {
            digitalWrite(LED_PIN, LOW);
            task = 6;
            delay(500);
          }
      break;
    }
    case 6: {// move straight
      digitalWrite(LED_PIN, HIGH);
      if (robot.moveForward(distanceToReach, motorVelocity)) {
        digitalWrite(LED_PIN, LOW);
        orient = (orient + nextDirection) % 4;
        updateMove();
        task = 2;
        delay(1000);
      }
      break;
    }
    case 7: { // reached destination
      SerialBT.println("Reached destination in " + (String)step + " steps");
      SerialBT.println();
      for(int i = 0; i < 5; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_PIN, LOW);
        delay(200);
      }
      task = 0;
      delay(100);
      break;
    }
  }
}

// --- Random algorithm:

// choose which direction with random algorithm
int moveToNextCellRandom() {
  bool availablePath[3];
  bool uncheckedPath[3];
  int nextX = currentX;
  int nextY = currentY;
  int direction;

  availablePath[0] = !(orientedWalls & (1 << N));
  availablePath[1] = !(orientedWalls & (1 << E));
  availablePath[2] = !(orientedWalls & (1 << W));

  if(availablePath[0] || availablePath[1] || availablePath[2]) {
    unsigned char checkedPaths = 0b1111;
    if(!(mazeMatrix[currentX-1][currentY] & (1 << 4)) && currentX > 0)
      checkedPaths &= ~(1 << N);
    else
      checkedPaths |= (1 << N);
    if(!(mazeMatrix[currentX][currentY+1] & (1 << 4)) && currentY < 4)
      checkedPaths &= ~(1 << E);
    else
      checkedPaths |= (1 << E);
    if(!(mazeMatrix[currentX+1][currentY] & (1 << 4)) && currentX < 4)
      checkedPaths &= ~(1 << S);
    else
      checkedPaths |= (1 << S);
    if(!(mazeMatrix[currentX][currentY-1] & (1 << 4)) && currentY > 0)
      checkedPaths &= ~(1 << W);
    else
      checkedPaths |= (1 << W);
    
    checkedPaths = (checkedPaths >> orient) | (checkedPaths << (4 - orient));
    checkedPaths &= 0b1011;

    uncheckedPath[0] = !(checkedPaths & (1 << N));
    uncheckedPath[1] = !(checkedPaths & (1 << E));
    uncheckedPath[2] = !(checkedPaths & (1 << W));
    
    int countDirections = 0;
    int directions[3] = {0,0,0};
    if((uncheckedPath[0] && availablePath[0]) || (uncheckedPath[1] && availablePath[1]) || (uncheckedPath[2] && availablePath[2])) {
      if(uncheckedPath[0] && availablePath[0]) {
        directions[countDirections] = N;
        countDirections++;
      }
      if(uncheckedPath[1] && availablePath[1]) {
        directions[countDirections] = E;
        countDirections++;
      }
      if(uncheckedPath[2] && availablePath[2]) {
        directions[countDirections] = W;
        countDirections++;
      }
    } else {
      if(availablePath[0]) {
        directions[countDirections] = N;
        countDirections++;
      }
      if(availablePath[1]) {
        directions[countDirections] = E;
        countDirections++;
      }
      if(availablePath[2]) {
        directions[countDirections] = W;
        countDirections++;
      }
    }
    direction = drawDirection(directions[0], directions[1], directions[2], countDirections);
  } else {
    direction = S;
  }
  return direction;
}

// draw from 1, 2 or 3 numbers
int drawDirection(int a, int b, int c, int numberOfDirections) {
  int i = random(numberOfDirections);

  if(i == 0) return a;
  else if(i == 1) return b;
  else return c;
}

// --- Left Wall Following algotihm:

// choose which direction with wall following algorithm
int moveToNextCellWallFollowing() {
  int nextX = currentX;
  int nextY = currentY;
  int nextOrient;
  int direction;
  if(!(orientedWalls & (1 << W)))
    direction = W;
  else if (!(orientedWalls & (1 << N)))
    direction = N;
  else if (!(orientedWalls & (1 << E)))
    direction = E;
  else
    direction = S;

  return direction;
}

// --- Flood Fill algorithm:

// choose which direction with flood fill algorithm
int moveToNextCellFloodFill() {
  int minCost = floodFill[currentX][currentY];
  int nextX = currentX;
  int nextY = currentY;
  int nextOrient;

  if (currentY > 0 && !(mazeMatrix[currentX][currentY] & (1 << W)) && floodFill[currentX][currentY - 1] < minCost) { // W
    minCost = floodFill[currentX][currentY - 1];
    nextOrient = W;
  }
  if (currentX < 4 && !(mazeMatrix[currentX][currentY] & (1 << S)) && floodFill[currentX + 1][currentY] < minCost) { // S
    minCost = floodFill[currentX + 1][currentY];
    nextOrient = S;
  }
  if (currentY < 4 && !(mazeMatrix[currentX][currentY] & (1 << E)) && floodFill[currentX][currentY + 1] < minCost) { // E
    minCost = floodFill[currentX][currentY + 1];
    nextOrient = E;
  }
  if (currentX > 0 && !(mazeMatrix[currentX][currentY] & (1 << N)) && floodFill[currentX - 1][currentY] < minCost) { // N
    minCost = floodFill[currentX - 1][currentY];
    nextOrient = N;
  }
  return ((4 - orient) + nextOrient) % 4;
}

// update flood fill matrix
void updateFloodFill() {
  int minValueFloodFill = maxValueFloodFill;
  for(int i = 0; i < 10; i++)
    for (int x = 0; x < 5; x++)
      for (int y = 0; y < 5; y++) {
        if(x == destinationX && y == destinationY)
          floodFill[destinationX][destinationY] = 0;
        else {
          if(x > 0 && !(mazeMatrix[x][y] & (1 << N)))
            minValueFloodFill = min(minValueFloodFill, floodFill[x - 1][y]);
          if(y < 4 && !(mazeMatrix[x][y] & (1 << E)))
            minValueFloodFill = min(minValueFloodFill, floodFill[x][y + 1]);
          if(x < 4 && !(mazeMatrix[x][y] & (1 << S)))
            minValueFloodFill = min(minValueFloodFill, floodFill[x + 1][y]);
          if(y > 0 && !(mazeMatrix[x][y] & (1 << W)))
            minValueFloodFill = min(minValueFloodFill, floodFill[x][y - 1]);
          floodFill[x][y] = minValueFloodFill + 1;
          minValueFloodFill = maxValueFloodFill;
        }
      }
}

// --- Universal functions:

// update X and Y position
void updateMove() {
  int nextX = currentX;
  int nextY = currentY;

  switch(orient) {
    case N:
      nextX = currentX - 1;
      break;
    case E:
      nextY = currentY + 1;
      break;
    case S:
      nextX = currentX + 1;
      break;
    case W:
      nextY = currentY - 1;
      break;
  }
  currentX = nextX;
  currentY = nextY;
}

// initialization of maze matrix and flood fill matrix
void initializeMaze() {
  for (int x = 0; x < 5; x++) {
    for (int y = 0; y < 5; y++) {
      floodFill[x][y] = maxValueFloodFill;
      mazeMatrix[x][y] = 0;
      if(y == 0)
        mazeMatrix[x][y] |= (1 << W);
      if(y == 4)
        mazeMatrix[x][y] |= (1 << E);
      if(x == 0)
        mazeMatrix[x][y] |= (1 << N);
      if(x == 4)
        mazeMatrix[x][y] |= (1 << S);
      int lookupX, lookupY;
      lookupX = 1 + x * 2;
      lookupY = 2 + y * 4;
      mazeLookup[lookupX][lookupY - 2] = (mazeMatrix[x][y] & (1 << W)) ? '|' : ' ';
      mazeLookup[lookupX][lookupY + 2] = (mazeMatrix[x][y] & (1 << E)) ? '|' : ' ';
      mazeLookup[lookupX - 1][lookupY] = (mazeMatrix[x][y] & (1 << N)) ? '-' : ' ';
      mazeLookup[lookupX - 1][lookupY - 1] = mazeLookup[lookupX - 1][lookupY];
      mazeLookup[lookupX - 1][lookupY + 1] = mazeLookup[lookupX - 1][lookupY];
      mazeLookup[lookupX + 1][lookupY] = (mazeMatrix[x][y] & (1 << S)) ? '-' : ' ';
      mazeLookup[lookupX + 1][lookupY - 1] = mazeLookup[lookupX + 1][lookupY];
      mazeLookup[lookupX + 1][lookupY + 1] = mazeLookup[lookupX + 1][lookupY];
    }
  }
  floodFill[destinationY][destinationX] = 0;
}

// print maze on the screen
void showMaze(String* maze) {
  SerialBT.println();
  for(int i = 0; i < 11; i++) {
    SerialBT.print(maze[i]);
    SerialBT.println();
  }
  SerialBT.println();
}

// check walls on left, right and front
unsigned char checkWalls() {
  unsigned char walls = 0b0000;
  if(robot.frontSensors() < wallThreshold)
    walls |= (1 << N);
  if(robot.rightSensor() < wallThreshold)
    walls |= (1 << E);
  if(robot.leftSensor() < wallThreshold)
    walls |= (1 << W);
  return walls;
}

// update maze with current position
void updateMaze() {
  unsigned char walls = (orientedWalls << orient) | (orientedWalls >> (4 - orient));
  walls &= 0b1111;
  walls |= (1 << 4);
  mazeMatrix[currentX][currentY] |= walls;
  if((mazeMatrix[currentX][currentY] & (1 << N)) && currentX > 0)
      mazeMatrix[currentX-1][currentY] |= (1 << S);
  if((mazeMatrix[currentX][currentY] & (1 << E)) && currentY < 4)
      mazeMatrix[currentX][currentY+1] |= (1 << W);
  if((mazeMatrix[currentX][currentY] & (1 << S)) && currentX < 4)
      mazeMatrix[currentX+1][currentY] |= (1 << N);
  if((mazeMatrix[currentX][currentY] & (1 << W)) && currentY > 0)
      mazeMatrix[currentX][currentY-1] |= (1 << E);
}

// update maze lookup with current position
void updateMazeLookup() {
  unsigned char walls = mazeMatrix[currentX][currentY];
  int lookupX, lookupY;
  lookupX = 1 + currentX * 2;
  lookupY = 2 + currentY * 4;
  mazeLookup[lookupX][lookupY - 2] = (walls & (1 << W)) ? '|' : ' ';
  mazeLookup[lookupX][lookupY + 2] = (walls & (1 << E)) ? '|' : ' ';
  mazeLookup[lookupX - 1][lookupY] = (walls & (1 << N)) ? '-' : ' ';
  mazeLookup[lookupX - 1][lookupY - 1] = mazeLookup[lookupX - 1][lookupY];
  mazeLookup[lookupX - 1][lookupY + 1] = mazeLookup[lookupX - 1][lookupY];
  mazeLookup[lookupX + 1][lookupY] = (walls & (1 << S)) ? '-' : ' ';
  mazeLookup[lookupX + 1][lookupY - 1] = mazeLookup[lookupX + 1][lookupY];
  mazeLookup[lookupX + 1][lookupY + 1] = mazeLookup[lookupX + 1][lookupY];

  for(int x = 0; x < 5; x++)
    for(int y = 0; y < 5; y++) {
      int lookupx = 1 + x * 2;
      int lookupy = 2 + y * 4;
      mazeLookup[lookupx][lookupy] = ' ';
    }

  switch(orient) {
    case 0: 
      mazeLookup[lookupX][lookupY] = 'N';
      break;
    case 1: 
      mazeLookup[lookupX][lookupY] = 'E';
      break;
    case 2: 
      mazeLookup[lookupX][lookupY] = 'S';
      break;
    case 3: 
      mazeLookup[lookupX][lookupY] = 'W';
      break;
  }
}
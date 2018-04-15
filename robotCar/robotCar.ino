/*Program to use 2X 28BYJ-48 stepper motors and ULN2003 stepper motor drivers for the Left and Right wheels. Left motor controlled by pin 2,3,4,5. Right motor controlled by pin 6,7,8,9. Both motors driven by Full step. Combining Ultrasonic sensor HC-SR04 from Chapter 2 for a distance detection and stop the motors when distance is less 600us, aprroximately 10cm.

Coded by Bong Cho, Nov 2017.*/

int l_bluepin = 2;      //Left motor pins
int l_pinkpin = 3;     
int l_yellowpin = 4;    
int l_orangepin = 5;    
int r_bluepin = 6;      //Right motor pins
int r_pinkpin = 7;      
int r_yellowpin = 8;    
int r_orangepin = 9;    

int Trig_pin = 11;      //HC-SR04 Ultrasonic Transmit
int Echo_pin = 13;      //HC-SR04 Ultrasonic Receive

int l_pin[5] = {l_bluepin, l_pinkpin, l_yellowpin, l_orangepin, l_bluepin};  //1 dimensional array declaration of left motor with 5 elements
int r_pin[5] = {r_bluepin, r_pinkpin, r_yellowpin, r_orangepin, r_bluepin};  //1 dimensional array declaration of right motor with 5 elements 

int duration = 0;
int turnCycle = 235;
int cycle = 512;
int randNumber = 0;

int coordinateX = -1;  //Default coordinate X
int coordinateY = -1;  //Default coordinate Y

const int xLength = 15; //horizontal length of map
const int yLength = 15; //vertical length of map
int routeMap[xLength][yLength]; //map array

int currentXPos = 0;
int currentYPos = 0;
int currentDirection = 2; //1=left, 2=front, 3=right, 4=back
int pointingDirection = 2;
boolean directionDecided = false;

char output[800];//final output

void setup() {
  // put your setup code here, to run once:
  for (int pin = 0; pin <= 3; pin++) {   //define all the motor pins as OUTPUT pins using afor loop
    pinMode(l_pin[pin], OUTPUT);
    pinMode(r_pin[pin], OUTPUT);    
    }
  pinMode(Trig_pin, OUTPUT);
  pinMode(Echo_pin, INPUT);
  Serial.begin(9600); 

  //initialize map array
  for(int i=0;i<xLength;i++){
    for(int j=0;j<yLength;j++){
      routeMap[i][j] = 0;
    }
  }
}    

void left(){  //Rotate left function, note that the 2 motors are turning in the same direction
    for (int steps = 0; steps <= turnCycle; steps++){      
      for (int i = 0; i <= 3; i++) {
        for (int pin = 0; pin <= 3; pin++) {
          digitalWrite(l_pin[pin], LOW);
          digitalWrite(r_pin[pin], LOW);
        }
        digitalWrite(l_pin[i], HIGH);
        digitalWrite(l_pin[i + 1], HIGH);
        digitalWrite(r_pin[i], HIGH);
        digitalWrite(r_pin[i + 1], HIGH);
        delay(2);    
      } 
    }
    strcat(output,"1");
}

void right(){  //Rotate right function, note that the 2 motors are turning in the same direction
    for (int steps = 0; steps <= turnCycle; steps++){      
      for (int i = 0; i <= 3; i++) {
        for (int pin = 0; pin <= 3; pin++) {
          digitalWrite(l_pin[pin], LOW);
          digitalWrite(r_pin[pin], LOW);
        }
        digitalWrite(l_pin[4 - i], HIGH);
        digitalWrite(l_pin[3 - i], HIGH);
        digitalWrite(r_pin[4 - i], HIGH);
        digitalWrite(r_pin[3 - i], HIGH);
        delay(2);    
      }
    }
    strcat(output,"3");
}


void forward(){  //Forward function, note that the 2 motors are turning in opposite directions
    for (int steps = 0; steps <= cycle; steps++){
      for (int i = 0; i <= 3; i++) {
            for (int pin = 0; pin <= 3; pin++) {
              digitalWrite(l_pin[pin], LOW);
              digitalWrite(r_pin[pin], LOW);
            }
            digitalWrite(l_pin[4 - i], HIGH);
            digitalWrite(l_pin[3 - i], HIGH);
            digitalWrite(r_pin[i], HIGH);
            digitalWrite(r_pin[i + 1], HIGH);
            delay(2);        
          }
    }
    strcat(output,"2");
}

boolean isboundary(int currentXPos,int currentYPos){
  if(currentXPos == 0 || currentXPos == xLength-1 || currentYPos == 0 || currentYPos == yLength-1){
      return true;
  }
  return false;
}

int *getForwardCoordinate(int currentXPos,int currentYPos,int currentDirection){
  static int pos[2]; //[0]:Xpos [1]:Ypos
  if(currentDirection == 2){
    pos[0] = currentXPos;
    pos[1] = currentYPos + 1;
  }else if(currentDirection == 4){
    pos[0] = currentXPos;
    pos[1] = currentYPos - 1;
  }else if(currentDirection == 1){ //left
    pos[0] = currentXPos - 1;
    pos[1] = currentYPos;
  }else if(currentDirection == 3){
    pos[0] = currentXPos + 1;
    pos[1] = currentYPos;
  }
  
  return pos;
}

int getUltraSoundDuration(){
    digitalWrite(Trig_pin,LOW);
    delay(2);
    digitalWrite(Trig_pin,HIGH);
    delay(50);
    digitalWrite(Trig_pin,LOW);
    return pulseIn(Echo_pin,HIGH); 
}

void detectObstacles(){
  
}

void turnAndGoForward(){
  int duration = getUltraSoundDuration();
  if(duration <= 1200){ //if obstacles
         //mark forward as obstacles
         int *pos;
         pos = getForwardCoordinate(currentXPos,currentYPos,currentDirection);
         routeMap[pos[0]][pos[1]] = 2;
  }else{
         forward();
  }
}

void loop() {
    if( Serial.available() ) {
      
    
    //initialize
    directionDecided = false;
    
    //get ultrasound duration
    duration = getUltraSoundDuration();
    
    randomSeed(duration);
    randNumber=random(0,2);

    
    /*
     *  **** Path Finding Algorithm by Sam ****
     *  
     *  Mark current block as seen
     * 
     *  if obstacles
     *     mark forward as obstacles
     *  
     *  find possible direction(s)
     *  for each direction
     *    if the next block is unseen
     *      choose that direction
     *      
     *  if no direction is chose
     *      choose one direction randomly
     *      
    */

    routeMap[currentXPos][currentYPos] = 1;   //Mark current block as seen

    //screen through obstacles in all directions

    duration = getUltraSoundDuration();
    if(duration <= 1200){ //if obstacles
         //mark forward as obstacles
         int *pos;
         pos = getForwardCoordinate(currentXPos,currentYPos,currentDirection);
         //Serial.print("forward direction:");Serial.print(currentXPos);Serial.print(currentYPos);
         routeMap[pos[0]][pos[1]] = 2;
         strcat(output,"5");
    }

    right();

    duration = getUltraSoundDuration();
    if(duration <= 1200){ //if obstacles
         //mark forward as obstacles
         int *pos;
         pos = getForwardCoordinate(currentXPos,currentYPos,currentDirection);
         //Serial.print("forward direction:");Serial.print(currentXPos);Serial.print(currentYPos);
         routeMap[pos[0]][pos[1]] = 2;
         strcat(output,"5");
    }

    left();
    left();

    duration = getUltraSoundDuration();
    if(duration <= 1200){ //if obstacles
         //mark forward as obstacles
         int *pos;
         pos = getForwardCoordinate(currentXPos,currentYPos,currentDirection);
         //Serial.print("forward direction:");Serial.print(currentXPos);Serial.print(currentYPos);
         routeMap[pos[0]][pos[1]] = 2;
         strcat(output,"5");
    }

    right();

    // *******

      for(int i=1;i<=4;i++){  //for each direction
         int *pos = getForwardCoordinate(currentXPos,currentYPos,i);

         if (!(pos[0] < 0 || pos[0] > xLength-1 || pos[1] < 0 || pos[1] > yLength-1)){  //if not went to the boundaries

            if(routeMap[pos[0]][pos[1]] == 0){ //if next block is unseen
                currentDirection = i; //set as current direction
                directionDecided = true;
                break;
            }
         }
      }

      while(directionDecided==false){
        randomSeed(analogRead(A5));
        randNumber=random(1,4);
        int *pos = getForwardCoordinate(currentXPos,currentYPos,randNumber);
        //Serial.print(currentXPos);Serial.print(currentYPos);
       
        if (!(pos[0] < 0 || pos[0] > xLength-1 || pos[1] < 0 || pos[1] > yLength-1)){  //if not went to the boundaries
            if(routeMap[pos[0]][pos[1]] != 2){ //if next block is not obstacles
                currentDirection = randNumber; //set as current direction
                directionDecided = true;
            }
        }

       boolean finished = true;
       //Check if the map is completed
       for(int i=yLength-1;i>=0;i--){
          for(int j=0;j<xLength;j++){
              if(routeMap[j][i]==0){
                finished = false;
              }
          }
       }
       if(finished==true){
          for (;;);
       }
      }
      
      //execute directionDecided
      if(pointingDirection == currentDirection){
        forward();
        pointingDirection = currentDirection;
      }else if(pointingDirection == 2 && currentDirection==1){
        left();
        forward();
        pointingDirection = 1;
      }else if(pointingDirection == 2 && currentDirection==3){
        right();
        forward();
        pointingDirection = 3;
      }else if(pointingDirection == 2 && currentDirection==4){
        right();
        right();
        forward();
        pointingDirection = 4;
      }else if(pointingDirection == 1 && currentDirection==2){
        right();
        forward();
        pointingDirection = 2;
      }else if(pointingDirection == 1 && currentDirection==3){
        right();
        right();
        forward();
        pointingDirection = 3;
      }else if(pointingDirection == 1 && currentDirection==4){
        left();
        forward();
        pointingDirection = 4;
      }else if(pointingDirection == 3 && currentDirection==1){
        right();
        right();
        forward();
      }else if(pointingDirection == 3 && currentDirection==2){
        left();
        forward();
        pointingDirection = 2;
      }else if(pointingDirection == 3 && currentDirection==4){
        right();
        forward();
        pointingDirection = 4;
      }else if(pointingDirection == 4 && currentDirection==1){
        right();
        forward();
        pointingDirection = 1;
      }else if(pointingDirection == 4 && currentDirection==2){
        right();
        right();
        forward();
        pointingDirection = 2;
      }else if(pointingDirection == 4 && currentDirection==3){
        left();
        forward();
        pointingDirection = 3;
      }


    //update values
    int *pos = getForwardCoordinate(currentXPos,currentYPos,currentDirection);
    currentXPos = pos[0];
    currentYPos = pos[1];
    
 /*   
    for(int i=yLength-1;i>=0;i--){
      for(int j=0;j<xLength;j++){
          Serial.print(routeMap[j][i]);
      }
      Serial.println();
    }*/

    Serial.println(output);
    }
}


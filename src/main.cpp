#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <string.h>

Servo gunServo;
Servo towerServo;
Servo triggerServo;

#define GUN_SERVO_PIN 11
#define TOWER_SERVO_PIN 10
#define TRIGGER_MOTER_PIN 9
#define TRIGGER_BUTTON_PIN 2
#define DISTANCE_TRIGGER_PIN 7
#define DISTANCE_ECHO_PIN 8

//define global variables
bool redDot = true;
float velocity = 14.5;
int resolutionX = 640;
int resolutionY = 480;
int towerMaxAngle = 180;
int towerMinAngle = 0;

int gunMaxAngle = 180;
int gunMinAngle = 0;
int fov = 40;

const unsigned int MAX_DIST = 23200;

void setup() {
  Serial.begin(9600);
  pinMode(11, OUTPUT);
  gunServo.attach(11);

  pinMode(10, OUTPUT);
  towerServo.attach(10);

  pinMode(9, OUTPUT);
  triggerServo.attach(9);

  pinMode(2, INPUT);

  pinMode(3, INPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(DISTANCE_TRIGGER_PIN, OUTPUT);
  digitalWrite(DISTANCE_TRIGGER_PIN, LOW);

  //Set Echo pin as input to measure the duration of 
  //pulses coming back from the distance sensor
  pinMode(DISTANCE_ECHO_PIN, INPUT);
}

float GetShootAngle(float distance, float height, float initial_velocity) {
  if (distance < 0 || distance > 5 || height < 0 || initial_velocity <= 0) {
    Serial.println("Invalid input");
    return 0; // Return a default value
  }
  float gravity = 9.8; // acceleration due to gravity in m/s^2
  float discriminant = pow(initial_velocity, 4) - gravity * (gravity * pow(distance, 2) + 2 * height * pow(initial_velocity, 2));
  if (discriminant < 0) {
    Serial.println("Invalid input - cannot calculate angle");
    return 0; // Return a default value
  }
  
  // Calculate both possible angles
  float angle1 = atan((initial_velocity * initial_velocity + sqrt(discriminant)) / (gravity * distance)) * 180 / PI;
  float angle2 = atan((initial_velocity * initial_velocity - sqrt(discriminant)) / (gravity * distance)) * 180 / PI;
  
  // Choose the minimum positive angle
  float minAngle = 0;
  if (angle1 > 0 && angle2 > 0) {
    minAngle = min(angle1, angle2);
  } else if (angle1 > 0) {
    minAngle = angle1;
  } else if (angle2 > 0) {
    minAngle = angle2;
  } else {
    Serial.println("Invalid input - cannot calculate angle");
    return 0; // Return a default value
  }
  
  return minAngle;
}
void ShootDemo(){
  float distance = 0, height = 0, initial_velocity = 0;
  
  Serial.println("Calibrating.");
  
  gunServo.write(0); // Reset gunServo motor to initial position
  delay(3000);
  Serial.println("Done Calibrating!");

    
  // Prompt user for distance
  Serial.println("Enter distance (0-5):");
  while (Serial.available() == 0) {
    // Wait for user input
  }
  distance = Serial.parseFloat();
    
  // Prompt user for height
  Serial.println("Enter height (>= 0):");
  while (Serial.available() == 0) {
    // Wait for user input
  }
  height = Serial.parseFloat();
  // Prompt user for initial velocity
  Serial.println("Enter initial velocity (> 0):");
  while (Serial.available() == 0) {
    // Wait for user input
  }
  initial_velocity = Serial.parseFloat();
  // Calculate angle
  float angle = GetShootAngle(distance, height, initial_velocity);
    
  // Display the calculated angle
  Serial.print("Calculated angle: ");
  Serial.println(angle);
  // Control gunServo motor based on the calculated angle
  gunServo.write(angle); // Assuming you have a gunServo motor connected
  delay(10000); // Adjust delay as necessary
}
void funnyDemo(){
   for (int i = 0; i < 180; i++)
 {
    gunServo.write(i);
    delay(30);
 }
 gunServo.write(0);
 delay(1000);
}


float getAfstand(){
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm = 0;

    
  // Hold the trigger pin high for at least 10 us
  digitalWrite(DISTANCE_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(DISTANCE_TRIGGER_PIN, LOW);

  // Wait for pulse on echo pin
  while ( digitalRead(DISTANCE_ECHO_PIN) == 0 );

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min
  t1 = micros();
  while ( digitalRead(DISTANCE_ECHO_PIN) == 1);
  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  // Wait at least 60ms before next measurement
  cm += pulse_width / 58.0;
  
  delay(60);
  

  return cm;
}

float getAngle(int x, int resolution){
  float angle = (((float)fov / (float)resolution) * (float)x) -20.0;
  return angle;
}

bool isInMiddle(int x, int y){
  int threshold = 10;
  int resolutionXHalf = resolutionX / 2;
  int resolutionYHalf = resolutionY / 2;
  bool middle = (x > resolutionXHalf + threshold || x < resolutionXHalf - threshold || 
                  y > resolutionYHalf + threshold || y < resolutionYHalf - threshold);
  return middle;
}

void affyr(float afstand){
  //assumes the gun is calibrated and pointing at the red dot
  float servoAngle = gunServo.read(); 
  float height = servoAngle * sin(afstand);
  
  float angle = GetShootAngle(afstand, height, velocity);

  gunServo.write(angle);
  delay(500);
  triggerServo.write(180);
  delay(5000);
  triggerServo.write(0);
  return;
}

bool affyringsKnapTrykket(){
  return digitalRead(TRIGGER_BUTTON_PIN);
}

void moveServo(float angle, Servo servo, int max, int min){
  int newAngle = angle + servo.read();
  Serial.println("Moving servo ");
  Serial.print(servo.read());

  Serial.println("new angle: ");
  Serial.print(String(newAngle));
  if (newAngle > max) servo.write(max);
  else if (newAngle < min) servo.write(min);
  else servo.write(newAngle);
  return;
}
void fullDemo(){
  if(redDot){//do nothing
    int x;// = getPosition(x);
    int y;// = getPosition(y);

    if(!isInMiddle(x, y)){
      moveServo(getAngle(x, resolutionX), towerServo, towerMaxAngle, towerMinAngle);
      moveServo(getAngle(y, resolutionY), gunServo, gunMaxAngle, gunMinAngle);
    }
    else {
      float afstand = getAfstand();
      if(afstand < 4 && afstand > 0.6 && affyringsKnapTrykket()) affyr(afstand);
    }
  }
}


int mostFrequent(int* arr, int n) 
{ 
    // code here 
    int maxcount = 0; 
    int element_having_max_freq; 
    for (int i = 0; i < n; i++) { 
        int count = 0; 
        for (int j = 0; j < n; j++) { 
            if (arr[i] == arr[j]) 
                count++; 
        } 
  
        if (count > maxcount) { 
            maxcount = count; 
            element_having_max_freq = arr[i]; 
        } 
    } 
  
    return element_having_max_freq; 
} 
void loop() {
  towerServo.write(90);
  delay(1000);
  towerServo.write(0);
  delay(1000);

}                                                                                                                                                                                                                                                                                                             
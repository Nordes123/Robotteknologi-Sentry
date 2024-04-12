#include <Arduino.h>
#include <Servo.h>

Servo gunServo;
Servo towerServo;

void setup() {
  Serial.begin(9600);
  pinMode(11, OUTPUT);
  gunServo.attach(11);

  pinMode(10, OUTPUT);
  towerServo.attach(10);
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








void loop() {
  gunServo.write(0);
  Serial.println("0");
  delay(3000);
  gunServo.write(180);
  Serial.println("180");
  delay(2000);
}
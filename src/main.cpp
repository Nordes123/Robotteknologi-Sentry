#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <string.h>
#include <algorithm>

Servo gunServo;
Servo towerServo;
Servo triggerServo;

#define GUN_SERVO_PIN 11
#define TOWER_SERVO_PIN 10
#define TRIGGER_MOTER_PIN 9
#define TRIGGER_BUTTON_PIN 2
#define DISTANCE_TRIGGER_PIN 7
#define DISTANCE_ECHO_PIN 8

//definer globale variabler
float velocity = 14.5;
int resolutionX = 640;
int resolutionY = 480;
int towerMaxAngle = 180;
int towerMinAngle = 0;

int gunMaxAngle = 180;
int gunMinAngle = 0;
int fov = 40;

int speed = 10;

const unsigned int MAX_DIST = 23200;


struct Position {
    int x;
    int y;
};

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

  // Trigger-pinen fortæller sensoren, at den skal finde området
  pinMode(DISTANCE_TRIGGER_PIN, OUTPUT);
  digitalWrite(DISTANCE_TRIGGER_PIN, LOW);

  //Sæt Echo pin som input for at måle varigheden af
  //impulser, der kommer tilbage fra afstandssensoren
  pinMode(DISTANCE_ECHO_PIN, INPUT);
}

float getShootangle(float distance, float height, float initial_velocity) {
  if (distance < 0 || distance > 5 || height < 0 || initial_velocity <= 0) {
    Serial.println("Invalid input");
    return 0; 
  }
  float gravity = 9.8; 
  float discriminant = pow(initial_velocity, 4) - gravity * (gravity * pow(distance, 2) + 2 * height * pow(initial_velocity, 2));
  if (discriminant < 0) {
    Serial.println("Invalid input - cannot calculate angle");
    return 0; 
  }
  
  // Beregn begge mulige vinkler
  float angle1 = atan((initial_velocity * initial_velocity + sqrt(discriminant)) / (gravity * distance)) * 180 / PI;
  float angle2 = atan((initial_velocity * initial_velocity - sqrt(discriminant)) / (gravity * distance)) * 180 / PI;
  
  // Vælg den mindste positive vinkel
  float minAngle = 0;
  if (angle1 > 0 && angle2 > 0) {
    minAngle = min(angle1, angle2);
  } else if (angle1 > 0) {
    minAngle = angle1;
  } else if (angle2 > 0) {
    minAngle = angle2;
  } else {
    Serial.println("Invalid input - cannot calculate angle");
    return 0; 
  }
  
  return minAngle;
}

Position getPosition(){
  // hvis data er tilgængeligt
  if (Serial.available() <= 0) return {0, 0};

  Position pos;
  String data = Serial.readStringUntil('\n'); // læs de indgående data indtil newline-tegnet
  int commaIndex = data.indexOf(','); // find indekset for kommaet

  if (commaIndex == -1) return {0, 0}; 
  // hvis kommaet er fundet
  String xString = data.substring(0, commaIndex); // udtræk delstrengen før kommaet som X-koordinat
  String yString = data.substring(commaIndex + 1); // udtræk delstrengen efter kommaet som Y-koordinat

  pos.x = xString.toInt(); // konverter X-koordinatstreng til en int
  pos.y = yString.toInt(); // konverter Y-koordinatstreng til en int

  return pos;
  
}


void moveServoSpeed(Servo servo, float angle, int speed ){
  float startAngle = servo.read();
  float diff = angle - startAngle; // får forskellen mellem startvinklen og slutvinklen
  int steps = abs(diff); // antal trin, der skal tages for at nå slutvinklen
  int direction = diff > 0 ? 1 : -1;
  for(int i = 0; i < steps; i++){
    servo.write(startAngle + i * direction);
    delay(speed);
  }
  return;
}

float getAfstand(){
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cmArr[10];

  for (int i = 0; i < 10; i++)
  {
  // Hold senderpin høj i mindst 10 mikrosekunder
  digitalWrite(DISTANCE_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(DISTANCE_TRIGGER_PIN, LOW);

  // Vent på puls på ekko-stiften
  while ( digitalRead(DISTANCE_ECHO_PIN) == 0 );

  // Mål hvor lang tid evho pin er (pulse width)
  // Bemærk: micros()-tælleren vil løbe over efter ~70 min.
  t1 = micros();
  while ( digitalRead(DISTANCE_ECHO_PIN) == 1);
  t2 = micros();
  pulse_width = t2 - t1;

  // Beregn afstand i centimeter og tommer. Konstanterne
  // findes i databladet, og beregnes ud fra den antagne hastighed
  // af lyd i luft ved havets overflade (~340 m/s).
  cmArr[i] = pulse_width / 58.0;
  
  // Vent mindst 60 ms før næste måling
  delay(60);
  }

  return mostFrequent((int*)cmArr, 10);
}
void pullTrigger(int speed, int rotations){
// Roter med uret i det angivne antal omdrejninger
  for (int i = 0; i < rotations; i++) {
    triggerServo.writeMicroseconds(2000); // Set servo to rotate clockwise
    delay(speed); // Wait for one rotation
  }

// Roter mod uret for at vende tilbage til udgangspositionen
  for (int i = 0; i < rotations; i++) {
    triggerServo.writeMicroseconds(1000);// Sæt servoen til at rotere mod uret
    delay(speed);// Vent på en rotation
  }
}
float getAngle(int x, int resolution){
  float angle = (((float)fov / (float)resolution) * (float)x) - fov / 2;
  return angle;
}

bool isInMiddle(Position pos){
  int threshold = 10;
  int resolutionXHalf = resolutionX / 2;
  int resolutionYHalf = resolutionY / 2;
  bool middle = (pos.x > resolutionXHalf - threshold && pos.x < resolutionXHalf + threshold) && 
                (pos.y > resolutionYHalf - threshold && pos.y < resolutionYHalf + threshold);
  return middle;
}

void affyr(float afstand){
  //forudsætter, at pistolen er kalibreret og peger på den røde prik
  float servoAngle = gunServo.read(); 
  float height = tan(servoAngle) * afstand;
  //get the close distance using pythagoean theorem with height and distance
  float closeDistance = sqrt(pow(afstand, 2) - pow(height, 2));


  
  float angle = getShootangle(closeDistance, height, velocity);

  moveServoSpeed(gunServo, angle, speed);
  pullTrigger(860, 4);
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
  else moveServoSpeed(servo, newAngle, speed);
  return;
}
void main(){
  Position pos = getPosition();
  if(pos.x != 0 && pos.y != 0){

    if(!isInMiddle(pos)){
      moveServo(getAngle(pos.x, resolutionX), towerServo, towerMaxAngle, towerMinAngle);
      moveServo(getAngle(pos.y, resolutionY), gunServo, gunMaxAngle, gunMinAngle);
    }
    else {
      float afstand = getAfstand();
      if(afstand < 4 && afstand > 0.6 && affyringsKnapTrykket()) affyr(afstand);
    }
  }
}

int mostFrequent(int* arr, int n) 
{ 
    sort(arr, arr + arr.length());
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
  main();
}                                                                                                                                                                                                                                                                                                             
#include <AccelStepper.h>

int steps;
int x_axis=0,y_axis=0,z_axis=0,x2_axis=5,y2_axis=5,z2_axis=0;
float position_for_1, temperature_analysis;
int prev_x = 0, prev_y = 0,prev_z = 0;
int flag = 0,temp;


#define DIR_PIN_X 1
#define DIR_PIN_Y 4
#define DIR_PIN_Z 7
#define STEP_PIN_X 2
#define STEP_PIN_Y 5 
#define STEP_PIN_Z 8
#define STEP_PIN_X 2
#define ENABLE_PIN_X 3
#define ENABLE_PIN_Y 6
#define ENABLE_PIN_Z 9

#define DIR_PIN_E 14
#define STEP_PIN_E 15
#define ENABLE_PIN_E 16

#define END_X 11
#define END_Y 12
#define END_Z 13

#define temperaturePin A0
#define chamber_temp A1
#define bet_temp A2

#define fan 20 // fan for extruder
#define fan_chamber 21


AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);
AccelStepper stepperY(AccelStepper::DRIVER, STEP_PIN_Y, DIR_PIN_Y);
AccelStepper stepperZ(AccelStepper::DRIVER, STEP_PIN_Z, DIR_PIN_Z);
AccelStepper stepperE(AccelStepper::DRIVER, STEP_PIN_E, DIR_PIN_E);


// Extruder and the thermister config
const int extruderSpeedPin = 12;  // Adjust pin number for Mega

// Define pins for temperature control
const int heaterPin = 13;         // Pin connected to the extruder heater (via relay or MOSFET), adjust pin number for Mega

// Constants for temperature calculation
const float seriesResistor = 10000; // Resistance of the series resistor (10kΩ)
const float thermistorNominal = 10000; // Resistance of the thermistor at 25 degrees C
const float temperatureNominal = 25; // Nominal temperature (25°C)
const float bCoefficient = 3950; // B coefficient of the thermistor
const float kelvinOffset = 273.15; // Conversion from Celsius to Kelvin

// Code to print the square

void go_to_initial_position(int axis_x1,int axis_y1)
{
  // For move to start position from previous position
  int steps_x = axis_x1 - prev_x;
  int steps_y = axis_y1 - prev_y;

  if (steps_x >= 0) {
    digitalWrite(DIR_PIN_X, HIGH);  // Clockwise direction
  }
  else {
    digitalWrite(DIR_PIN_X, LOW);   // Counterclockwise direction
    steps_x = -steps_x;  // Make steps positive for the following loop
  }
  if (steps_y >= 0) {
    digitalWrite(DIR_PIN_Y, HIGH);  // Clockwise direction
  } else {
    digitalWrite(DIR_PIN_Y, LOW);   // Counterclockwise direction
    steps_y = -steps_y;  // Make steps positive for the following loop
  }

  stepperX.moveTo(steps_x);// Assigning no of steps to motor
  stepperY.moveTo(steps_y);// Assigning no of steps to motor


  while(stepperX.distanceToGo() != 0 && stepperY.distanceToGo() != 0 && digitalRead(END_X) == 1 && digitalRead(END_Y)==1) // always run and check for optical limit switch
  {
    stepperX.run();
    stepperY.run();   
  }

}

float extruder_temperature()
{
  int sensorValue = analogRead(temperaturePin);  // Read analog value from the thermistor
  float resistance = seriesResistor / ((1023 / (float)sensorValue) - 1); // Calculate the resistance of the thermistor
  // Calculate temperature in Kelvin
  float temperatureK = (bCoefficient * temperatureNominal) / (bCoefficient + (temperatureNominal * log(resistance / thermistorNominal)));
  // Convert Kelvin to Celsius
  float temperatureC = temperatureK - kelvinOffset;
  return temperatureC;
}

void homing(int prev_x,int prev_y, int prev_z){
  if (prev_x >= 0) {
    digitalWrite(DIR_PIN_X, HIGH);  // Clockwise direction
  } else {
    digitalWrite(DIR_PIN_X, LOW);   // Counterclockwise direction
    prev_x = -prev_x;  // Make steps positive for the following loop
  }
  if (prev_y >= 0) {
    digitalWrite(DIR_PIN_Y, HIGH);  // Clockwise direction
  } else {
    digitalWrite(DIR_PIN_Y, LOW);   // Counterclockwise direction
    prev_y = -prev_y;  // Make steps positive for the following loop
  }
  digitalWrite(DIR_PIN_Z, HIGH);// SINCE Z HAS GETS HIGHER NO NEGATIVE VALUES
  stepperX.moveTo(prev_x);
  stepperY.moveTo(prev_y);
  stepperZ.moveTo(prev_z);

  while( digitalRead(END_X) == 1 || digitalRead(END_Y) == 1 || digitalRead(END_Z) == 1)
  {
    stepperX.run();
    stepperY.run();
    stepperZ.run();
  }
}

void temp_checking()
{
  temp = extruder_temperature();
  if(temp > 230)
  {
    digitalWrite(fan,HIGH);
    while(extruder_temperature() > 230);
  }
  else{
      digitalWrite(fan,LOW);
  }
}

float check_chamber()
{
  int sensorValue = analogRead(chamber_temp);  // Read analog value from the thermistor
  float resistance = seriesResistor / ((1023 / (float)sensorValue) - 1); // Calculate the resistance of the thermistor
  // Calculate temperature in Kelvin
  float temperatureK = (bCoefficient * temperatureNominal) / (bCoefficient + (temperatureNominal * log(resistance / thermistorNominal)));
  // Convert Kelvin to Celsius
  float temperatureC = temperatureK - kelvinOffset;
  // return temperatureC;
  while(temperatureC < 45 && temperatureC > 60)
  {
    sensorValue = analogRead(chamber_temp);
    resistance = seriesResistor / ((1023 / (float)sensorValue) - 1);
    temperatureK = (bCoefficient * temperatureNominal) / (bCoefficient + (temperatureNominal * log(resistance / thermistorNominal)));
    temperatureC = temperatureK - kelvinOffset;
    if(temperatureC > 60)
    {
      digitalWrite(fan_chamber, HIGH);
    }   
    else{
      digitalWrite(fan_chamber, LOW);
    } 
  }
}

float check_bet()
{
  int sensorValue = analogRead(bet_temp);  // Read analog value from the thermistor
  float resistance = seriesResistor / ((1023 / (float)sensorValue) - 1); // Calculate the resistance of the thermistor
  // Calculate temperature in Kelvin
  float temperatureK = (bCoefficient * temperatureNominal) / (bCoefficient + (temperatureNominal * log(resistance / thermistorNominal)));
  // Convert Kelvin to Celsius
  float temperatureC = temperatureK - kelvinOffset;
  while(temperatureC < 60 && temperatureC > 70)
  {
    sensorValue = analogRead(bet_temp);
    resistance = seriesResistor / ((1023 / (float)sensorValue) - 1);
    temperatureK = (bCoefficient * temperatureNominal) / (bCoefficient + (temperatureNominal * log(resistance / thermistorNominal)));
    temperatureC = temperatureK - kelvinOffset;
  }
}

void move_to_coordinate(int axis_x1,int axis_y1, int axis_z1,int axis_x2,int axis_y2,int axis_z2)
{
  stepperE.moveTo(1000);
   
  go_to_initial_position(axis_x1, axis_y1);

  //EXTRUTION START X and Y MOVING TO POSITION
  check_chamber();
  check_bet();

  int steps_x = axis_x2 - axis_x1; //taking difference in temperature for printing design
  int steps_y = axis_y2 - axis_y1;

  // Set initial motor direction and enable

  // Set motor direction based on input
  if (steps_x >= 0) {
    digitalWrite(DIR_PIN_X, HIGH);  // Clockwise direction
  } else {
    digitalWrite(DIR_PIN_X, LOW);   // Counterclockwise direction
    steps_x = -steps_x;  // Make steps positive for the following loop
  }
  if (steps_y >= 0) {
    digitalWrite(DIR_PIN_Y, HIGH);  // Clockwise direction
  } else {
    digitalWrite(DIR_PIN_Y, LOW);   // Counterclockwise direction
    steps_y = -steps_y;  // Make steps positive for the following loop
  }


  // for setting 100 steps per mm and to chose the steps to move
  steps_x = steps_x * 100;
  steps_y = steps_y * 100;
  prev_x = steps_x; // considered the current position(as of now) as a previous position for homing
  prev_y = steps_y;

  if(steps_x == 0)
  {
    stepperY.moveTo(steps_y); // if
    stepperE.moveTo(1000);    
    while(stepperY.distanceToGo() != 0 && digitalRead(END_Y) == 1){
      stepperE.run();
      stepperY.run();
    }
  }
  else if(steps_y == 0)
  {
    stepperX.moveTo(steps_y);
    stepperE.moveTo(1000);
    while(stepperX.distanceToGo() != 0 && digitalRead(END_X) == 1){
    stepperE.run();
    stepperX.run();
    }  
  }
  else{
    if(steps_x > steps_y){
      position_for_1 = steps_x / steps_y;
      steps = steps_y;
      stepperX.moveTo(position_for_1);;
      stepperY.moveTo(1);
    }
    else{
      position_for_1 =  steps_y / steps_x; 
      steps = steps_x;
      stepperY.moveTo(position_for_1);
      stepperX.moveTo(1);
    }


  // Wait for extrution temperature
  temp_checking();


  // this consist of printing the whole design
  for (int i = 0; i < steps; i++) {
    while(stepperX.distanceToGo() != 0 && stepperY.distanceToGo() != 0 && digitalRead(END_X) == 1 && digitalRead(END_Y)==1 )
    {
    stepperE.run();
    stepperX.run();
    stepperY.run();
    }
    stepperE.stop();

    temp_checking(); //Extruder temperature checking and control
    
    check_chamber(); // chamber temperature checking and control
    check_bet(); // bet temperature checking and control
  }

  // Optional: Add a delay between movements
  delay(500); // Adjust this delay as needed
  stepperZ.moveTo(1);
  prev_z += 1; 
  while(stepperZ.distanceToGo() != 0)
  {
   stepperZ.run();
  }
  if(digitalRead(END_Z) == 0)
  {
    stepperZ.stop();
  }
  }

}

void setup() {
  // put your setup code here, to run once:
  pinMode(fan, OUTPUT);
  pinMode(temperaturePin, INPUT);

  pinMode(DIR_PIN_X, OUTPUT);
  pinMode(STEP_PIN_X, OUTPUT);
  pinMode(ENABLE_PIN_X , OUTPUT);

  pinMode(DIR_PIN_Y, OUTPUT);
  pinMode(STEP_PIN_Y, OUTPUT);
  pinMode(ENABLE_PIN_Y , OUTPUT);

  pinMode(DIR_PIN_Z, OUTPUT);
  pinMode(STEP_PIN_Z, OUTPUT);
  pinMode(ENABLE_PIN_Z , OUTPUT);

  pinMode(DIR_PIN_E, OUTPUT);
  pinMode(STEP_PIN_E, OUTPUT);
  pinMode(ENABLE_PIN_E , OUTPUT);

  Serial.begin(115200);

  // Set maximum speed and acceleration for X and Y axes
  stepperX.setMaxSpeed(1000);
  stepperX.setAcceleration(500);
  stepperY.setMaxSpeed(1000);
  stepperY.setAcceleration(500);
  stepperZ.setMaxSpeed(1000);
  stepperZ.setAcceleration(500);
  stepperE.setMaxSpeed(200);
  stepperE.setAcceleration(200);

  digitalWrite(fan,LOW);

  digitalWrite(ENABLE_PIN_X, HIGH);
  digitalWrite(ENABLE_PIN_Y, HIGH);
  digitalWrite(ENABLE_PIN_E, HIGH);
  digitalWrite(ENABLE_PIN_Z, HIGH);

}

void loop() {
  homing(prev_x,prev_y,prev_z);
  move_to_coordinate(x_axis,y_axis,z_axis,x2_axis,y2_axis,z2_axis);

}

void print_square()
{
  homing(prev_x,prev_y,prev_z);
  digitalWrite(DIR_PIN_X, HIGH);
  digitalWrite(DIR_PIN_Y, HIGH);
  stepperX.moveTo(100);
  stepperE.moveTo(1000);
  while(stepperX.distanceToGo() != 0 && digitalRead(END_X)==1)
  {
    while(extruder_temperature() < 215);
    stepperE.run();
    stepperX.run();

    temp_checking();
    check_chamber();
    check_bet();
  }
  stepperE.stop();
  stepperY.moveTo(100);
  while(stepperY.distanceToGo() != 0 && digitalRead(END_Y)==1)
  {
    while(extruder_temperature() < 215);
    stepperE.run();
    stepperY.run();

    temp_checking();
    check_chamber();
    check_bet();
  }
  stepperE.stop();
  digitalWrite(DIR_PIN_X, LOW);
  digitalWrite(DIR_PIN_Y, LOW);
  while(stepperX.distanceToGo() != 0 && digitalRead(END_X)==1)
  {
    while(extruder_temperature() < 215);
    stepperE.run();  
    stepperX.run();

    temp_checking();
    check_chamber();
    check_bet();    
  }
  stepperE.stop();
  stepperY.moveTo(100);
  while(stepperY.distanceToGo() != 0 && digitalRead(END_Y)==1)
  {
    while(extruder_temperature() < 215);
    stepperE.run();
    stepperY.run();

    temp_checking();
    check_chamber();
    check_bet();    
  }
  stepperE.stop();

}

void print_triangle()
{
  digitalWrite(DIR_PIN_X, HIGH);
  digitalWrite(DIR_PIN_Y, HIGH);
  stepperX.moveTo(100);
  stepperE.moveTo(1000);

  while(stepperX.distanceToGo() != 0 && digitalRead(END_X)==1)
  {
    while(extruder_temperature() < 215);
    stepperE.run();
    stepperX.run();

    temp_checking();
    check_chamber();
    check_bet();
  }
  stepperY.moveTo(100);
  while(stepperY.distanceToGo() != 0 && digitalRead(END_Y)==1)
  {
    while(extruder_temperature() < 215);
    stepperE.run();
    stepperY.run();

    temp_checking();
    check_chamber();
    check_bet();
  }
  digitalWrite(DIR_PIN_X, LOW);
  digitalWrite(DIR_PIN_Y, LOW);
  for (int i = 0; i < 100; i++) {
    while(stepperX.distanceToGo() != 0 && stepperY.distanceToGo() != 0 && digitalRead(END_X)==1 && digitalRead(END_Y)==1){
    stepperE.run();
    stepperX.run();
    stepperY.run();
    }
    stepperE.stop();

    temp_checking();
    check_chamber();
    check_bet();
  }


}

void G_X()
{
  homing(prev_x,prev_y,prev_z);
  digitalWrite(DIR_PIN_X, HIGH);
  digitalWrite(DIR_PIN_Y, HIGH);
  stepperX.moveTo(100);
  stepperY.moveTo(100);
  stepperE.moveTo(1000);

  for (int i = 0; i < 100; i++) {
  while(stepperX.distanceToGo() != 0 && stepperY.distanceToGo() != 0 && digitalRead(END_X) == 1 && digitalRead(END_Y)==1 )
  {
  stepperE.run();
  stepperX.run();
  stepperY.run();
  }
  stepperE.stop();

  temp_checking(); //Extruder temperature checking and control
  
  check_chamber(); // chamber temperature checking and control
  check_bet(); // bet temperature checking and control
  }
  go_to_initial_position(0,100);
  
  digitalWrite(DIR_PIN_X, LOW);
  digitalWrite(DIR_PIN_Y, LOW);

  for (int i = 0; i < 100; i++) {
  while(stepperX.distanceToGo() != 0 && stepperY.distanceToGo() != 0 && digitalRead(END_X) == 1 && digitalRead(END_Y)==1 )
  {
  if(i!=50){
  stepperE.run();
  }
  stepperX.run();
  stepperY.run();
  }
  stepperE.stop();

  temp_checking(); //Extruder temperature checking and control
  
  check_chamber(); // chamber temperature checking and control
  check_bet(); // bet temperature checking and control
  } 

}
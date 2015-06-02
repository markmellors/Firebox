

#include <Servo.h>
#include <PinChangeInt.h> // available at http://www.arduino.cc/playground/Main/PinChangeInt

#define A_PIN 2         // accelerometer analog input
#define G_PIN 3         // gyro analog input

#define A_GAIN 2.948    // [deg/LSB]
#define G_GAIN 4.57    // [deg/s/LSB]  should be 0.41?
#define S_GAIN 0.08

#define A 0.97         // complementary filter constant

#define sensorIR 4     //Must be an analog pin
float sensorValue;    //Must be of type float for pow()
float IntegralError;
int A_ZERO=232;      // approx. 1.9[V] * 1024[LSB/5V]/5
int G_ZERO=265;       // approx. 1.39[V] * 1024[LSB/V]
// more often G_Zero=480 & A_zero=293, this could be for lying down though
// second attempt gave G_Zero=404 & A_zero=359, this could be for lying down though
float DT = 0;           // loop period, [seconds]
float LeftMotorOutput = 0;
float RightMotorOutput = 0;
float angle = 0.0;      // [deg]
float error = 0.0, integral=0;
float rate = 0.0;       // [deg/s]
float output = 0.0;     // [LSB] (100% voltage to motor is 255LSB)
float IRdistance;
long timer = 0;
int SteerPin = 8;
int ForwardPin = 9;
// timing for rc input with interrupts
unsigned long risingFWDTime = 0;
unsigned long risingSteeringTime = 0;
double FWDSignal=1500, SteeringSignal=1500, NextFWDSignal, NextSteeringSignal;
const int chipSelect = 10;
int FileNumber;
String dataString;
String SFileName; 
char CFileName[12];
char Buffer[8];

void setup( )
{
  initSabertooth( );
 pinMode(ForwardPin, INPUT);
 pinMode(SteerPin, INPUT);
 Serial.begin(9600);
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    // don't do anything more:
     for (int i=0; i <= 5; i++){
     digitalWrite(2, HIGH);
     digitalWrite(3, LOW);
     delay(100);
     digitalWrite(3, HIGH);
     digitalWrite(2, LOW);
     delay(100);
   }    
    return;
  }
  
  FileNumber=0;
  SFileName="Test"+String(FileNumber)+".txt";
  SFileName.toCharArray(CFileName,12);   
  while (SD.exists(CFileName))
  {
   
    FileNumber+=1;
    SFileName="Test"+String(FileNumber)+".txt";
    SFileName.toCharArray(CFileName,12);
  }

// interrupt setup
PCintPort::attachInterrupt(ForwardPin, risingFWD, RISING); // attach a PinChange Interrupt to FWD pin on the rising edge
PCintPort::attachInterrupt(ForwardPin, fallingFWD, FALLING); // attach a PinChange Interrupt to FWD pin on the falling edge
PCintPort::attachInterrupt(SteerPin, risingSteering, RISING); // attach a PinChange Interrupt to Steering pin on the rising edge
PCintPort::attachInterrupt(SteerPin, fallingSteering, FALLING); // attach a PinChange Interrupt to Steering pin on the falling edge
// for (int i=0; i <= 2; i++){
 //  digitalWrite(2, HIGH);
 //  digitalWrite(3, HIGH);
 //  delay(100);
 //  digitalWrite(2, LOW);
 //  digitalWrite(3, LOW);
 //  delay(100);
 //  }
 //  delay (400);
  G_ZERO=404; //analogRead(G_PIN);
 // for (int i=0; i <= 2; i++){
 //  digitalWrite(2, HIGH);
 //  digitalWrite(3, HIGH);
 //  delay(100);
 //  digitalWrite(2, LOW);
 //  digitalWrite(3, LOW);
 //  delay(100);
  // } 
  A_ZERO=359; //analogRead(A_PIN);
  
  dataString = "G_Zero = ";
  dataString += G_ZERO;
  dataString += ", A_Zero = ";
  dataString += A_ZERO;
  File dataFile = SD.open(CFileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
  }    
}

void loop()
{
  signed int accel_raw = 0;
  signed int gyro_raw = 0;


  // Read in the raw accelerometer, gyro, and steering singals.
  // Offset for zero angle/rate.
  accel_raw = (signed int) analogRead(A_PIN) - A_ZERO;
  gyro_raw = (signed int) analogRead(G_PIN) - G_ZERO;
 // SteerDuration = pulseIn(SteerPin, HIGH);
 // ForwardDuration = pulseIn(ForwardPin, HIGH); 
  
  // Scale the gyro to [deg/s].
  rate = (float) gyro_raw * G_GAIN;
  
  DT=(millis()-timer)/(float)1000;
  timer=millis();
  
  // Complementarty filter.
  angle = A * (angle + rate * DT)+ (1 - A) * (float) accel_raw * A_GAIN;

  // incorperate fwd input command
 error = angle-(FWDSignal-1500.0)/15.0;
 integral=error+0.9*integral;
  // PD controller.
  //2 is too low
output = -2.3*error - 0.01*rate-0.0*integral;

//do nothing if RC signal not received
//if (FWDsignal<800) output=0;

output=max(-60,output);
output=min(60,output);

//add steering input
LeftMotorOutput=output+S_GAIN*(SteeringSignal-1500);
RightMotorOutput=output-S_GAIN*(SteeringSignal-1500);

setLeftMotorSpeed(LeftMotorOutput);
setRightMotorSpeed(RightMotorOutput);

  sensorValue = analogRead(sensorIR);
  IRdistance = 106500.8 * pow(sensorValue,-0.935) - 10;
  
   // Debug.
//Serial.print(millis());
//Serial.print(", ");
//Serial.print(IRdistance);
//Serial.print(", ");
///Serial.print(accel_raw);
//Serial.print(", ");
//Serial.print(gyro_raw);
//Serial.print(", ");
//
//Serial.print(error);
//Serial.print(", ");
//Serial.println(angle);
  dataString = String(millis());  //1
  dtostrf(IRdistance,8,4,Buffer);  //2
  dataString += ", ";
  dataString += Buffer;
  dataString += ", ";
  dataString += accel_raw;  //3
  dataString += ", ";  
  dataString += gyro_raw;  //4
  dataString += ", ";
  dtostrf(angle,8,6,Buffer);  //5
  dataString += Buffer;
  dataString += ", ";
  dtostrf(FWDSignal,8,0,Buffer); //7
  dataString += Buffer;
  dataString += ", ";
  dtostrf(SteeringSignal,8,0,Buffer);  //8
  dataString += Buffer;

  File dataFile = SD.open(CFileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
  }  
 
  // Delay for consistent loop rate.
  delay(10);
} 
// interrupts for detecting RC incoming pulses 
void risingFWD() {risingFWDTime = micros();}
void fallingFWD() {
NextFWDSignal = ((double)((int)(micros()-risingFWDTime)));
FWDSignal = min(FWDSignal+50,max(NextFWDSignal,FWDSignal-50));
}
void risingSteering() {risingSteeringTime = micros();}
void fallingSteering() {
NextSteeringSignal = ((double)((int)(micros()-risingSteeringTime))); 
SteeringSignal= min(SteeringSignal+50,max(NextSteeringSignal,SteeringSignal-50));
}

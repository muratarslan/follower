//#define DEBUG
#include "QTRSensors.h"

int TRACKING_WHITE = 1;
#define NUM_SENSORS 8 // number of sensors used

class Pid {
  double setPoint;
  
  double kp;
  double kd;
  double ki;

  double integrator;
  double derivator;

  double inMin;
  double inMax;
  double outMin;
  double outMax;

  double clamp(double value, double min, double max){
    if (value > max)
      return max;
    if (value < min)
      return min;
    return value;
  }

  float mapDouble(double x, double in_min, double in_max, double out_min, double out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  public:
  Pid(): setPoint(0), 
         kp(0), ki(0), kd(0), 
         integrator(0), derivator(0), 
         inMin(0), inMax(0), outMin(0), outMax(0)  { };
  
  void setConstants(double p, double i,double d){
    kp = p;
    ki = i;
    kd = d;
  };

  void setBounds(double imin, double imax,double omin, double omax){
    inMin = imin;
    inMax = imax;
    outMin = omin;
    outMax = omax;
  };

  void setSetPoint(double val){
    setPoint = val;
  };

  double compute(double v){
    double val = mapDouble(clamp(v, inMin, inMax), inMin, inMax, -1.0, 1.0);
    double sp = mapDouble(clamp(setPoint, inMin, inMax), inMin, inMax, -1.0, 1.0);
    double error = sp - val;
#ifdef DEBUG
    Serial.print(" err: ");
    Serial.print(error);
#endif
    double pVal = kp * error;
#ifdef DEBUG
    Serial.print(" pv: ");
    Serial.print(pVal);
#endif
    double dVal = kd * (error - derivator);
#ifdef DEBUG
    Serial.print(" dv: ");
    Serial.print(dVal);
#endif
    double intor = clamp((integrator + error), -1.0, 1.0);
    double iVal = integrator * ki;
#ifdef DEBUG
    Serial.print(" iv: ");
    Serial.print(iVal);
#endif
    double pid = mapDouble(clamp((pVal + iVal + dVal), -1.0, 1.0), -1.0, 1.0, outMin, outMax);
    integrator = intor;
    derivator = error;
#ifdef DEBUG
    Serial.print(" p: ");
    Serial.print(pid);
#endif
    return pid;
  };
};

class Motor {
  int PwmPinMotorA;
  int PwmPinMotorB;
  int DirectionPinMotorA;
  int DirectionPinMotorB;

  public:
  Motor(): PwmPinMotorA(11),
           PwmPinMotorB(10),
           DirectionPinMotorA(13),
           DirectionPinMotorB(12) { };

  void setupPins(){
      // motor pins must be outputs
    pinMode(PwmPinMotorA, OUTPUT);
    pinMode(PwmPinMotorB, OUTPUT);
    pinMode(DirectionPinMotorA, OUTPUT);
    pinMode(DirectionPinMotorB, OUTPUT);
  }
  
  void setSpeeds(int a, int b){
    if ( a < 0 ){
      analogWrite(PwmPinMotorA, (a * -1));
      digitalWrite(DirectionPinMotorA, HIGH);
    }else{
      analogWrite(PwmPinMotorA, a);
      digitalWrite(DirectionPinMotorA, LOW);
    }

    if ( b < 0 ){
      analogWrite(PwmPinMotorB, (b * -1));
      digitalWrite(DirectionPinMotorB, HIGH);
    }else{
      analogWrite(PwmPinMotorB, b);
      digitalWrite(DirectionPinMotorB, LOW);
    }
  };
};

class Sensors{
  QTRSensorsRC* qtrrc;
  unsigned int sensorValues[NUM_SENSORS];

  int whiteTrashHold;
  int blackTrashHold;

  public:
  Sensors(QTRSensorsRC* q, int w, int b){
    qtrrc = q;
    whiteTrashHold = w;
    blackTrashHold = b;
  };
  
  void calibrate(){
    for (int i = 0; i < 200; i++){
      qtrrc->calibrate(QTR_EMITTERS_ON); 
    }
  };

  int detectTrackColor(){
    unsigned int val[NUM_SENSORS];
    qtrrc->readCalibrated(val);
    int lowCount = 0;
    for(int i=0; i < NUM_SENSORS; i++)
      if (val[i] < whiteTrashHold )
        lowCount++;
    
    if (lowCount > 5 )
      return 0; //while line
    else
      return 1;
  };

  int readLine(){
    return qtrrc->readLine(sensorValues,QTR_EMITTERS_ON,detectTrackColor());
  };
  
};

Pid pid;
Motor motor;
QTRSensorsRC qtrrc((unsigned char[]) {9, 2, 3, 4, 5, 6, 7, 8}, NUM_SENSORS, 2500, QTR_NO_EMITTER_PIN);
Sensors sensors(&qtrrc, 300, 100);

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif

  pid.setConstants(1.75, 0, 0.7);
  pid.setBounds(0, 7000, -100, 100);
  pid.setSetPoint(3500);

  motor.setupPins();
  sensors.calibrate();
}

void loop() {
  unsigned int position = sensors.readLine();

#ifdef DEBUG
  Serial.print(" pos: ");
  Serial.print(position);
#endif

  double corr = pid.compute(position);
#ifdef DEBUG
  Serial.println(" ");
#endif

  motor.setSpeeds((100 + corr), (100 - corr));
  //motor.setSpeeds(75,75);
  
}

//https://github.com/pololu/qtr-sensors-arduino/blob/master/QTRSensors/examples/QTRRCExample/QTRRCExample.ino

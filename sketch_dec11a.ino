#include <Servo.h>
#include <PID_v1.h>
#define SERVO_PIN 10
// #define BUTTON_PIN 13
#define ECHO 12
#define TRIG 8

double sp=20, output,input;

const double Kp = 11.0, Ki=1.5, Kd=1.1;
PID control(&input, &output, &sp, .15*Kp, 2*Ki, 2*Kd, DIRECT);


Servo servo;

enum class buttonStates{RELEASED,PRESSED};
enum class servoStates{ON,OFF};


double ultrasonic(int trig, int echo);



void setup() {
  // put your setup code here, to run once:
  servo.attach(SERVO_PIN);
  // pinMode(BUTTON_PIN, INPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  control.SetOutputLimits(0,180);
  control.SetMode(AUTOMATIC);
  servo.write(-90);
  delay(2000);
  servo.write(90);
  delay(2000);
  Serial.begin(9600);
  // delay(10000);
}

void loop() {
  input = ultrasonic(TRIG,ECHO);
  control.Compute();
  servo.write(output);//output+90
  Serial.print("Distance: ");
  Serial.print(sp-input);
  Serial.print(" Output: ");
  Serial.println(output);
  

}


double ultrasonic(int trig, int echo)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo,HIGH);
  return duration * 0.034 / 2.0;
}
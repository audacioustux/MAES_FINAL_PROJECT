#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

constexpr int LDR_IN = A3;
constexpr int SONAR_ECHO_IN = 5;
constexpr int SONAR_TRIG_OUT = 4;
constexpr int DISTANCE_SAFE_LED_OUT = 2;
constexpr int POTENTIOMETER_SAFE_DISTANCE_THRESHOLD_IN = A2;
constexpr int DISTANCE_CRITICAL_LED_OUT = 6;
constexpr int DISCO_LED_OUT = 7;

constexpr int MOTOR_IN1 = 8;
constexpr int MOTOR_IN2 = 9;
constexpr int MOTOR_IN3 = 12;
constexpr int MOTOR_IN4 = 13;
constexpr int MOTOR_ENA = 10;
constexpr int MOTOR_ENB = 11;

class Smoother
{
  const static int numReadings = 10;

  float readings[numReadings] = {0}; // the readings from the analog input
  int readIndex = 0;                 // the index of the current reading
  float total = 0;                   // the running total
  float average = 0;                 // the average

public:
  float smoothify(int value)
  {
    total = total - readings[readIndex];
    readings[readIndex] = value;
    total = total + readings[readIndex];
    readIndex = readIndex + 1;

    if (readIndex >= numReadings)
      readIndex = 0;

    average = total / numReadings;

    return average;
  }
};

Smoother ldr_smoother = Smoother();
Smoother potentiometer_safe_distance_threshold_smoother = Smoother();

int safe_distance_threshold;

void setup()
{
  Serial.begin(9600);
  pinMode(SONAR_TRIG_OUT, OUTPUT);
  pinMode(DISTANCE_CRITICAL_LED_OUT, OUTPUT);
  pinMode(DISTANCE_SAFE_LED_OUT, OUTPUT);
  pinMode(DISCO_LED_OUT, OUTPUT);
  pinMode(SONAR_ECHO_IN, INPUT);
  pinMode(LDR_IN, INPUT);
  pinMode(POTENTIOMETER_SAFE_DISTANCE_THRESHOLD_IN, INPUT);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_ENB, OUTPUT);

  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  for (int i = 0; i < 20; i++)
  {
    ldr_smoother.smoothify(analogRead(LDR_IN));
    potentiometer_safe_distance_threshold_smoother.smoothify(analogRead(POTENTIOMETER_SAFE_DISTANCE_THRESHOLD_IN));
    delay(5);
  }
  safe_distance_threshold = constrain(map(potentiometer_safe_distance_threshold_smoother.smoothify(analogRead(POTENTIOMETER_SAFE_DISTANCE_THRESHOLD_IN)), 0, 1023, 0, 255), 10, 50);
  Serial.println("Safe distance set to: " + String(safe_distance_threshold) + " cm");

  delay(1000);
}

void loop()
{
  // LDR
  float ldr_val = map(ldr_smoother.smoothify(analogRead(LDR_IN)), 0, 1023, 0, 255);
  Serial.print("LDR: " + String(ldr_val));
  if (ldr_val < 80)
  {
    digitalWrite(DISCO_LED_OUT, HIGH);
    Serial.println(" ~ Dark");
  }
  else
  {
    digitalWrite(DISCO_LED_OUT, LOW);
    Serial.println(" ~ Light");
  }

  // Accelerometer, Gyroscope & Temperature
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.println("Temp: " + String(temp.temperature) + " C");
  Serial.print("accelerations :: ");
  Serial.println("x: " + String(a.acceleration.x) + " y: " + String(a.acceleration.y) + " z: " + String(a.acceleration.z));
  Serial.print("gyro :: ");
  Serial.println("x: " + String(g.gyro.x) + " y: " + String(g.gyro.y) + " z: " + String(g.gyro.z));

  static uint8_t motor_left_speed = 0;
  static uint8_t motor_right_speed = 0;

  while (Serial.available() > 0)
  {
    char inputByte;
    inputByte = Serial.read();
    Serial.println(inputByte);
    if (inputByte == '-')
    {
      motor_left_speed -= 20;
      motor_right_speed -= 20;
    }
    if (inputByte == '+')
    {
      motor_left_speed += 20;
      motor_right_speed += 20;
    }
    if (inputByte == 'h')
    {
      digitalWrite(MOTOR_IN1, LOW);
      digitalWrite(MOTOR_IN2, HIGH);
      digitalWrite(MOTOR_IN3, HIGH);
      digitalWrite(MOTOR_IN4, LOW);
    }
    if (inputByte == 'j')
    {
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, LOW);
      digitalWrite(MOTOR_IN3, HIGH);
      digitalWrite(MOTOR_IN4, LOW);
    }
    if (inputByte == 'k')
    {
      digitalWrite(MOTOR_IN1, LOW);
      digitalWrite(MOTOR_IN2, HIGH);
      digitalWrite(MOTOR_IN3, LOW);
      digitalWrite(MOTOR_IN4, HIGH);
    }
    if (inputByte == 'l')
    {
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, LOW);
      digitalWrite(MOTOR_IN3, LOW);
      digitalWrite(MOTOR_IN4, HIGH);
    }
    if (inputByte == 's')
    {
      motor_left_speed = 0;
      motor_right_speed = 0;
    }
    if (inputByte == 'r')
    {
      motor_left_speed = 235;
      motor_right_speed = 235;
    }
    if (inputByte == 'a')
    {
      motor_left_speed -= 10;
    }
    if (inputByte == 'b')
    {
      motor_right_speed -= 10;
    }
    if (inputByte == 'A')
    {
      motor_left_speed += 10;
    }
    if (inputByte == 'B')
    {
      motor_right_speed += 10;
    }
  }

  analogWrite(MOTOR_ENA, motor_left_speed);
  analogWrite(MOTOR_ENB, motor_right_speed);

  Serial.println("Motor speed :: left: " + String(motor_left_speed) + " right:" + String(motor_right_speed));

  // SONAR
  digitalWrite(SONAR_TRIG_OUT, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_TRIG_OUT, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG_OUT, LOW);

  long duration = pulseIn(SONAR_ECHO_IN, HIGH);
  float speedOfSound = 331.3 + 0.606 * temp.temperature;
  float distance = (duration / 20000.0) * speedOfSound;
  Serial.print("Distance: " + String(distance) + " cm");

  if (distance < safe_distance_threshold)
  {
    Serial.print(" ~ Critical!");
    digitalWrite(DISTANCE_SAFE_LED_OUT, LOW);
    digitalWrite(DISTANCE_CRITICAL_LED_OUT, HIGH);
  }
  else
  {
    digitalWrite(DISTANCE_CRITICAL_LED_OUT, LOW);
    digitalWrite(DISTANCE_SAFE_LED_OUT, HIGH);
  }
  Serial.println();

  Serial.println();

  delay(1000);
}
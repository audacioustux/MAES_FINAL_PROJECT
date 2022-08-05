#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

#define LDR_IN A3
#define SONAR_ECHO_IN 5
#define SONAR_TRIG_OUT 4
#define DISTANCE_SAFE_LED_OUT 2
#define POTENTIOMETER_SAFE_DISTANCE_THRESHOLD_IN A2
#define DISTANCE_CRITICAL_LED_OUT 6
#define DISCO_LED_OUT 7

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
  safe_distance_threshold = (255. / 1023.) * potentiometer_safe_distance_threshold_smoother.smoothify(analogRead(POTENTIOMETER_SAFE_DISTANCE_THRESHOLD_IN));
  Serial.println("Safe distance set to: " + String(safe_distance_threshold) + " cm");

  delay(1000);
}

void loop()
{
  // LDR
  float ldr_val = (255. / 1023.) * ldr_smoother.smoothify(analogRead(LDR_IN));
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

  while (Serial.available() > 0)
  {
    char inputByte;
    inputByte = Serial.read();
    Serial.println(inputByte);
    if (inputByte == 'Z')
    {
      digitalWrite(13, HIGH);
    }
    else if (inputByte == 'z')
    {
      digitalWrite(13, LOW);
    }
  }

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

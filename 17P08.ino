#include <Servo.h>

// Arduino pin assignment

#define PIN_IR    0         // IR sensor at Pin A0
#define PIN_LED   9
#define PIN_SERVO 10

#define _DUTY_MIN 550  // servo full clock-wise position (0 degree)
#define _DUTY_NEU 1500  // servo neutral position (90 degree)
#define _DUTY_MAX 2450  // servo full counter-clockwise position (180 degree)

#define _DIST_MIN  100.0   // minimum distance 100mm
#define _DIST_MAX  250.0   // maximum distance 250mm

#define EMA_ALPHA  0.06      // for EMA Filter

#define LOOP_INTERVAL 20   // Loop Interval (unit: msec)

Servo myservo;
unsigned long last_loop_time;   // unit: msec

float dist_prev = _DIST_MIN;
float dist_ema = _DIST_MIN;

bool event_dist, event_servo, event_serial;

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);
  
  Serial.begin(1000000);    // 1,000,000 bps
}

void loop()
{
  unsigned long time_curr = millis();
  int duty;
  float a_value, dist_raw;
  

  // wait until next event time
  if (time_curr < (last_loop_time + LOOP_INTERVAL))
    return;
  last_loop_time += LOOP_INTERVAL;

  a_value = analogRead(PIN_IR);
  dist_raw = ((6762.0 / (a_value - 9.0)) - 4.0) * 10.0;

  //if ... // Put range Filter code here (_DIST_MIN ~ _DIST_MAX)
  // and turn on LED if the distance is in the range 

  if ((dist_raw == 0.0) || (dist_raw < _DIST_MIN)) {
    dist_raw = dist_prev;           // Cut higher than maximum
    digitalWrite(PIN_LED, 1);
    myservo.writeMicroseconds(_DUTY_MIN);       // 
  } else if (dist_raw > _DIST_MAX) {
    dist_raw = dist_prev;           // cut lower than minimum
    digitalWrite(PIN_LED, 1);
    myservo.writeMicroseconds(_DUTY_MAX);       // 
  } else {    // In desired Range
    dist_prev = dist_raw;
    digitalWrite(PIN_LED, 0);
    myservo.writeMicroseconds(duty);       //       
  }


  dist_ema = ((EMA_ALPHA) * dist_raw) + ((1-EMA_ALPHA) * dist_ema);
      
  //duty = map(dist_ema, _DIST_MIN, _DIST_MAX, _DUTY_MIN, _DUTY_MAX);
  duty = ((((dist_ema - _DIST_MIN) / (_DIST_MAX - _DIST_MIN)) * (_DUTY_MAX - _DUTY_MIN)) + _DUTY_MIN);
  // Put map() equivalent code here
  
  myservo.writeMicroseconds(duty);

  Serial.print("_DUTY_MIN:");  Serial.print(_DUTY_MIN);
  Serial.print("_DIST_MIN:");  Serial.print(_DIST_MIN);
  Serial.print(",IR:");        Serial.print(a_value);
  Serial.print(",dist_raw:");  Serial.print(dist_raw);
  Serial.print(",ema:");       Serial.print(dist_ema);
  Serial.print(",servo:");     Serial.print(duty);
  Serial.print(",_DIST_MAX:"); Serial.print(_DIST_MAX);
  Serial.print(",_DUTY_MAX:"); Serial.print(_DUTY_MAX);
  Serial.println("");
}
#include <Servo.h>

// 13C02: 서보 저속 구동
//   실험 1: _SERVO_SPEED 3   -> 60초 동안 180도 등속 이동
//   실험 2: _SERVO_SPEED 0.3 -> 300초 동안 90도 등속 이동
// 13_example_1.ino에서 바꾼 부분에는 [수정] 표시를 붙였다.

// Arduino pin assignment
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

// configurable parameters for Servo
// servo_calibration.ino로 측정한 내 서보의 값
#define _DUTY_MIN 1002 // servo full clock-wise position (0 degree)
#define _DUTY_NEU 1801 // servo neutral position (90 degree): 측정 전이라 (MIN+MAX)/2로 둔 값, 13C02에서는 쓰지 않음
#define _DUTY_MAX 2600 // servo full counter-clockwise position (180 degree)

//실험 선택: 1 또는 2로 바꿔서 업로드
#define _EXPERIMENT 2

#if _EXPERIMENT == 1
  #define _SERVO_SPEED  3     // servo angular speed (unit: degree/sec)
  #define _SWEEP_DEGREE 180   // total sweep angle (unit: degree)
#else
  #define _SERVO_SPEED  0.3   // servo angular speed (unit: degree/sec)
  #define _SWEEP_DEGREE 90    // total sweep angle (unit: degree)
#endif

#define _START_DELAY 2000 // [수정] 시작 위치로 이동할 시간 (unit: msec)

// Loop Interval
#define INTERVAL 20           // servo update interval (unit: msec)
#define _INTERVAL_SERIAL 1000 // [수정] serial output interval (unit: msec)

// duty(usec) -> angle(degree)
#define DUTY2DEG(d) (((d) - _DUTY_MIN) * 180.0 / (_DUTY_MAX - _DUTY_MIN))

// global variables
unsigned long last_sampling_time; // unit: msec
unsigned long last_serial_time;   // unit: msec

Servo myservo;

// [수정] int -> float
// 한 주기에 움직일 양이 1 usec보다 작으므로(예: 0.33, 0.033) 소수부를 버리면 안 된다.
float duty_change_per_interval; // maximum duty difference per interval (unit: usec)
float duty_target;  // Target duty time
float duty_curr;    // Current duty time

bool done = false;  // 목표 위치에 도달했는지

void setup() {
  // initialize GPIO pins
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar

  // [수정] 라이브러리 기본 범위(544~2400 usec) 밖의 보정값도 잘리지 않게 범위를 지정
  myservo.attach(PIN_SERVO, _DUTY_MIN, _DUTY_MAX);

  // [수정] 0도에서 출발해서 _SWEEP_DEGREE까지 한 번만 이동
  duty_curr   = _DUTY_MIN;
  duty_target = _DUTY_MIN + (float)(_DUTY_MAX - _DUTY_MIN) * (_SWEEP_DEGREE / 180.0);
  myservo.writeMicroseconds((int)(duty_curr + 0.5));

  // initialize serial port
  Serial.begin(57600);  // <----- baud rate

  // convert angular velocity into duty change per interval.
  duty_change_per_interval =
    (float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (INTERVAL / 1000.0);

  Serial.print("speed(deg/s):");   Serial.print(_SERVO_SPEED);
  Serial.print(",sweep(deg):");    Serial.print(_SWEEP_DEGREE);
  Serial.print(",duty_change_per_interval:");
  Serial.println(duty_change_per_interval, 4);

  // [수정] 이동은 _START_DELAY 뒤에 시작한다.
  last_sampling_time = _START_DELAY;
  last_serial_time   = _START_DELAY;
}

void loop() {
  float  dist_raw;

  // wait until next sampling time.
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  // adjust duty_curr toward duty_target by duty_change_per_interval
  if (duty_target > duty_curr) {
    duty_curr += duty_change_per_interval;
    if (duty_curr > duty_target)
      duty_curr = duty_target;
  } else {
    duty_curr -= duty_change_per_interval;
    if (duty_curr < duty_target)
      duty_curr = duty_target;
  }

  // [수정] writeMicroseconds()는 정수(usec)만 받으므로 반올림해서 전달
  myservo.writeMicroseconds((int)(duty_curr + 0.5));

  // [수정] 목표 위치에 처음 도달한 순간의 경과 시간을 출력
  if (!done && duty_curr == duty_target) {
    done = true;
    Serial.print("target reached, t(s):");
    Serial.println((millis() - _START_DELAY) / 1000.0, 2);
  }

  // [수정] 시리얼 출력은 _INTERVAL_SERIAL마다 한 번만
  if (millis() >= last_serial_time + _INTERVAL_SERIAL) {
    last_serial_time += _INTERVAL_SERIAL;
    Serial.print("t(s):");          Serial.print((millis() - _START_DELAY) / 1000.0, 2);
    Serial.print(",angle(deg):");   Serial.print(DUTY2DEG(duty_curr), 2);
    Serial.print(",duty_curr:");    Serial.print(duty_curr, 2);
    Serial.print(",duty_target:");  Serial.print(duty_target, 2);
    Serial.print(",dist_raw:");     Serial.println(dist_raw);
  }

  // update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}

#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9         // [1234] LED를 아두이노 GPIO 9번 핀에 연결
#define PIN_SERVO 10    // [3069] 서보모터를 아두이노 GPIO 10번 핀에 연결
#define PIN_IR A0     // [3070] 적외선 센서를 아두이노 아날로그 A0핀에 연결

// Framework setting
#define _DIST_TARGET 255 // [2635] 목표하는 위치
#define _DIST_MIN 100   //[3050] 거리의 최솟값이 100mm
#define _DIST_MAX 410  //[3068] 거리의 최대값 410mm

// Distance sensor
#define _DIST_ALPHA 0.0   //[3054] 센서 보정정도
          // [3131] EMA필터값
// Servo range
#define _DUTY_MIN 1150    // [3052] 서보의 최소각도를 microseconds로 표현 //(High)
#define _DUTY_NEU 1550    // [3070] 레일플레이트 중립위치를 만드는 서보 duty값 //(Middle)
#define _DUTY_MAX 1750    //[3062] 서보의 최대각도의 microseconds의 값 //(Low)

// Servo speed control
#define _SERVO_ANGLE 30   // [3131] 최대 가동범위에 따른 목표 서보 회전각
#define _SERVO_SPEED 400   //[3064]서보 속도를 30으로

// Event periods
#define _INTERVAL_DIST 20   // [2635] 적외선센서 업데이트 주기
#define _INTERVAL_SERVO 20  // [2635] 서보 업데이트 주기
#define _INTERVAL_SERIAL 100  // [3070] 시리얼 플로터 갱신 속도

// PID parameters
#define _KP 1.1  // [2635] P 이득 비율(?)
#define _KI 0.0   // [2635] I 이득 비율
#define _KD 0.0 // [2635] D 이득 비율


//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo; // [3070] 서보 객체 생성

// Distance sensor
float dist_target; // [1234] location to send the ball
float dist_raw, dist_ema; // [3070] 거리와 노이즈 필터 적용 후 거리를 저장하기 위한 변수

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo,
last_sampling_time_serial;
// [2635] 각 이벤트별 업데이트를 위해 측정하는 시간
//[3054] 마지막으로 측정한 거리, 마지막으로 측정한 서보 각도 
//(unsigned long : 부호없는 정수형)
bool event_dist, event_servo, event_serial; // [2635] 이벤트별 이번 루프에서 업데이트 여부

// Servo speed control
int duty_chg_per_interval; // [3055] 서보속도 제어를 위한 변수 선언
int duty_target; 
int duty_curr; // [3131] 목표duty, 현재duty

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm; // [3070] PID 제어를 위한 현재 오차, 이전오차, 컨트롤(?), p 값, d 값, i 값 변수 선언


void setup() {
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT); //[3062] 핀 LED 활성화
  myservo.attach(PIN_SERVO); // [3070] 서보 구동을 위한 서보 초기화

// initialize global variables
duty_target, duty_curr = _DIST_MIN; // [3055] duty_target, duty_curr 초기화
last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial = 0;
// [3055] 샘플링 타임 변수 초기화
dist_raw, dist_ema = _DIST_MIN; // [3055] dist 변수 초기화
pterm = iterm = dterm = 0; // [2635] pid 제어값에서 우선 p 만 사용하도록

// move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU); //[2635]

// initialize serial port
  Serial.begin(57600); //[2635]

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0f) * (_INTERVAL_SERVO / 1000.0f);    // [3131] 설정한 서보 스피드에 따른 duty change per interval 값을 변환

}
  

void loop() {
    // [3055] indentation 수정(space 4칸)
    /////////////////////
    // Event generator //
    /////////////////////
    unsigned long time_curr = millis(); // [3070] 이벤트 업데이트 주기 계산을 위한 현재 시간
    // [3070] 이벤트 주기가 돌아올때까지 현재시간과 비교하며 기다리도록 함.
    if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
    }
    if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
    }
    if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
    }

    ////////////////////
    // Event handlers //
    ////////////////////

    if(event_dist) {
        event_dist = false; // [2635] 업데이트 대기
        // get a distance reading from the distance sensor
        dist_raw = ir_distance_filtered(); // [2635] 센서 값 받아와서 변수에 저장
        dist_ema = _DIST_ALPHA * dist_raw + (1 - _DIST_ALPHA) * dist_ema; 
        // [3055] dist_ema 설정

        // PID control logic
        error_curr = _DIST_TARGET - dist_raw; // [2635] 오차 계산
        pterm = error_curr; // [2635] p값은 오차
        control = _KP * pterm + _KI * iterm +  _KD * dterm; // [2635] control 값, i와 d는 현재 0

        // duty_target = f(duty_neutral, control)
        //duty_target = _DUTY_NEU * (1 + control); // [3070] control 값이 다 합해서 1이 되도록 되어있다면, 중립 위치에 컨트롤 값 만큼의 비율을 더해 목표위치를 정한다.
        //int X = 0;
        //duty_target = _DUTY_NEU + (control - X) * (control > X ? _DUTY_MAX - _DUTY_NEU : _DUTY_NEU - _DUTY_MIN); // [2635] NEU 에다가 위아래로 control 에 맞게 조절한 값?
        duty_target = _DUTY_NEU + (control); 

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
        if(duty_target < _DUTY_MIN)  // [2635] 양극값 넘어가는 경우 극값으로 제한
        {
            duty_target = _DUTY_MIN;
        }
        if(duty_target > _DUTY_MAX)
        {
            duty_target = _DUTY_MAX;
        }
  
  
    if(event_servo) {
        event_servo = false; // [2635] 업데이트 대기
        // adjust duty_curr toward duty_target by duty_chg_per_interval
         if(duty_target > duty_curr) {
           duty_curr += duty_chg_per_interval;
           //if(duty_curr > duty_target) duty_curr = duty_target;
         }
         else if(duty_target == duty_curr){
          duty_curr = duty_target;
         }
         else {
           duty_curr -= duty_chg_per_interval;
           //if(duty_curr < duty_target) duty_curr = duty_target;
         }
        // update servo position
        myservo.writeMicroseconds((int)duty_curr); // [3070]
    }

        // update servo position
        //myservo.writeMicroseconds((int)duty_curr); // [3070]

        //event_servo = false; // [3055] 모든 작업이 끝나면 이벤트를 다시 false로
    }
  
    if(event_serial) {
        event_serial = false; // [3070] 이벤트 주기가 왔다면 다시 false로 만들고 이벤트를 수행
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");

        //event_serial = false; // [3055] event_serial false로 변경
    }
}

float ir_distance(void){ // return value unit: mm
    float val; // [3055] 변수 val 선언
    float volt = float(analogRead(PIN_IR))  - 50; // [3055] volt변수에 적외선 센서 측정값 입력
    val = ((6762.0/(volt - 9.0)) - 4.0) * 10.0; // [3055] volt 값 보정
    return val; // [3055] val 리턴
}

float ir_distance_filtered(void){ // return value unit: mm
  return ir_distance(); // for now, just use ir_distance() without noise filter.
}

#include <avr/io.h>
#include <avr/interrupt.h>

// 馬達腳位
const int IN1 = 13;
const int IN2 = 12;
const int ENA = 5;

const int IN3 = 11;
const int IN4 = 10;
const int ENB = 6;

// 編碼器腳位
const int ENCODER_L_A = 2;
const int ENCODER_L_B = 9;
const int ENCODER_R_A = 3;
const int ENCODER_R_B = 7;

volatile long encoderCount_L = 0;
volatile long encoderCount_R = 0;
unsigned long lastMillis = 0;
float rpm_L = 0;
float rpm_R = 0;

// PID 參數左馬達
float Kp_L = 2.0;//0.81
float Ki_L = 0.80;//0.10
float Kd_L = 0.3;//3.0

float pwmOutput_L = 0;
float error_L = 0;
float integral_L = 0;
float derivative_L = 0;
float lastError_L = 0;
float positionError_L = 0;
float real_speed_L = 0;

// PID 參數右馬達
float Kp_R = 2.0;  //0.81
float Ki_R = 0.80;  //0.08
float Kd_R = 0.3;   //3.0

float pwmOutput_R = 0;
float error_R = 0;
float integral_R = 0;
float derivative_R = 0;
float lastError_R = 0;
float positionError_R = 0;
float real_speed_R = 0;

// 車身角度容許值
float accepted_tolerance = 5;
float accepted_tolerance_one = 60;
bool done = false;
bool obstacle = false;
bool reset = false;
// 編碼器參數
const int PPR = 330; // 11 * 減速比30 == 馬達轉一圈有330個方波
const int sampleTime = 10; // 每50ms取樣
//移動模式, 0轉車身,1前進固定距離,2前進完等聲音,3避障
int motion_mode = 0;

//超音波感測
const int trigPin = 8;
const int echoPin = 4;
float duration, distance;
unsigned long trig_time = 0;
unsigned long echo_start = 0;
unsigned long echo_end = 0;
bool wait_for_echo = false;

float target_angle = 0.0;
volatile float target_count_L = 0.0;
volatile float target_count_R = 0.0;
volatile long test_enc = 0;
long current_cnt = 0;
long current_cnt_L = 0;
long current_cnt_R = 0;
static float speed_L = 0, speed_R = 0;
static float last_cnt_L = 0, last_cnt_R = 0;

volatile unsigned int timerValue = 0;    // 記錄 Echo 高電位的計數值
volatile bool measuring = false;
volatile bool dataReady = false;

// <<< 修改開始：拆分角度佇列
float angle_read = 0;
float angle_queue[10]; // 最多拆10段
int angle_count = 0;    // 目前拆分段數
int current_step = 0;   // 當前執行段
bool executing = false; // 是否正在執行旋轉
static bool initialized = false;
// <<< 修改結束

void setup() {
  Serial.begin(9600);
  
  //左馬達pin
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT); //打pwm進去
  pinMode(ENCODER_L_A, INPUT);
  pinMode(ENCODER_L_B, INPUT);
  //右馬達pin
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT); //打pwm進去
  pinMode(ENCODER_R_A, INPUT);
  pinMode(ENCODER_R_B, INPUT);
  //超音波感測腳位
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderISR_L, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), encoderISR_R, RISING);
  //(目前兩邊正轉往前)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  /**/
  // 啟用 Pin Change Interrupt (針對 D4)
  PCICR |= (1 << PCIE2);     // 開啟 PORTD 的 PCINT (D0~D7)
  PCMSK2 |= (1 << PCINT20);  // 啟用 D4 對應的 PCINT

  // 設定 Timer 1：預分頻 8 → 每 tick = 0.5 µs
  TCCR1A = 0;  
  TCCR1B = 0;
  TCCR1B |= (1 << CS11);  // clk/8
  TCNT1 = 0;              // 重置計數器

  sei(); // 全域中斷開啟
  delay(1000);
  Serial.println("DONE");
}

void loop() {

  if (Serial.available() > 0) {
  String input = Serial.readStringUntil('\n');  // 讀整行
  input.trim();

  int commaIndex = input.indexOf(',');
  if (commaIndex > 0) {
    String angle_str = input.substring(commaIndex + 1);
    angle_read = angle_str.toFloat();  // 轉 float
  }

  done = false;
  reset = false;
  obstacle = false;

  // === 調整角度範圍 ===
  if (angle_read > 180) {
    angle_read -= 360;
  }
  angle_read = -angle_read; //輪子裝反
  if (abs(angle_read) < 10) { // 過小角度忽略
    angle_read = 0;
  }

  // === 初始化控制參數 ===
  executing = true;
  noInterrupts();
  encoderCount_L = 0;
  encoderCount_R = 0;
  interrupts();

  integral_L = 0;
  integral_R = 0;
  derivative_L = 0;
  derivative_R = 0;
  current_cnt_L = current_cnt_R = 0;
  lastError_L = lastError_R = 0;
  positionError_L = positionError_R = 0;
  // 將角度轉換成 encoder 目標 count
  // 0.4685 是轉向齒輪比常數 (1 degree -> 0.4685 encoder轉)
  target_count_L = ((angle_read * (PI / 180.0)) * PPR * 0.4685);
  target_count_R = ((angle_read * (PI / 180.0)) * PPR * 0.4685);
  pwmOutput_L = 0;
  pwmOutput_R = 0;
}

  unsigned long now = millis();
  if(now - lastMillis >= sampleTime){
    updateUltrasonic();
    if (motion_mode == 0 && executing) {

    noInterrupts();
    current_cnt_L = encoderCount_L;
    current_cnt_R = encoderCount_R;
    interrupts();

    // ======= 外圈 PID：角度控制（目標角度 → 期望角速度） =======
    positionError_L = target_count_L - current_cnt_L;
    positionError_R = target_count_R - current_cnt_R;

    // 當誤差大時給較高目標速度，接近目標自動減速
    float desired_speed_L = constrain(positionError_L , -40, 40);
    float desired_speed_R = constrain(positionError_R , -40, 40);

    // ======= 內圈 PID：速度控制（期望角速度 → PWM） =======
    static long last_cnt_L = 0;
    static long last_cnt_R = 0;

    real_speed_L = (current_cnt_L - last_cnt_L); // 每個 sampleTime 轉多少 pulse
    real_speed_R = (current_cnt_R - last_cnt_R);

    last_cnt_L = current_cnt_L;
    last_cnt_R = current_cnt_R;

    // PID 左馬達
    float error_L = desired_speed_L - real_speed_L;
    integral_L += error_L;
    derivative_L = error_L - lastError_L;
    lastError_L = error_L;
    pwmOutput_L = Kp_L * error_L + Ki_L * integral_L + Kd_L * derivative_L;
    pwmOutput_L = constrain(pwmOutput_L, -100, 100);

    if (pwmOutput_L > 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, pwmOutput_L);
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, -pwmOutput_L);
    }

    // PID 右馬達
    float error_R = desired_speed_R - real_speed_R;
    integral_R += error_R;
    derivative_R = error_R - lastError_R;
    lastError_R = error_R;
    pwmOutput_R = Kp_R * error_R + Ki_R * integral_R + Kd_R * derivative_R;
    pwmOutput_R = constrain(pwmOutput_R, -100, 100);

    if (pwmOutput_R > 0) {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, pwmOutput_R);
    } else {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, -pwmOutput_R);
    }

    // ======= 終止條件（角度到達） =======
    if (abs(positionError_L) <= accepted_tolerance && abs(positionError_R) <= accepted_tolerance) {
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
      executing = false;

      noInterrupts();
      encoderCount_L = 0;
      encoderCount_R = 0;
      interrupts();

      integral_L = integral_R = current_cnt_L = current_cnt_R = 0;
      derivative_L = derivative_R = 0;
      last_cnt_L = last_cnt_R = 0;
      positionError_L = positionError_R = 0;
      error_L = error_R = 0;
      lastError_L = lastError_R = 0;
      pwmOutput_L = pwmOutput_R = 0;
      motion_mode = 1;  
      delay(100);
      //Serial.println("Rotation Done.");
    }
    Serial.print("desired_speed_L: ");
    Serial.print(desired_speed_L);
    Serial.print(" real_speed_L: ");
    Serial.print(real_speed_L);
    Serial.print(" pwm_L: ");
    Serial.print(pwmOutput_L);
    Serial.print(" || desired_speed_R: ");
    Serial.print(desired_speed_R);
    Serial.print(" real_speed_R: ");
    Serial.print(real_speed_R);
    Serial.print(" pwm_R: ");
    Serial.println(pwmOutput_R);
  }
  else if (motion_mode == 1 && !executing) {
    static bool initialized = false;
    static unsigned long lastTime = 0;
    /*speed_L = 0, speed_R = 0;
    last_cnt_L = 0, last_cnt_R = 0;*/

    // 期望速度（可自行設定，單位：encoder count / 取樣時間）
    const float target_speed_L = -30.0;   // 左輪往前轉速度
    const float target_speed_R =  30.0;   // 右輪往前轉速度
    const unsigned long dt_ms = 1;       // 取樣週期 50 ms

    if (!initialized) {
        initialized = true;
        noInterrupts();
        encoderCount_L = 0;
        encoderCount_R = 0;
        interrupts();

        integral_L = 0;
        integral_R = 0;
        derivative_L = 0;
        derivative_R = 0;
        lastError_L = 0;
        lastError_R = 0;
        last_cnt_L = 0;
        last_cnt_R = 0;
        error_L = error_R = 0;
        lastTime = millis();
    }

    // --- 每 dt_ms 更新一次速度 ---
    if (millis() - lastTime >= dt_ms) {
        lastTime = millis();

        // 1. 計算目前編碼器速度 (Δcount / Δt)
        noInterrupts();
        long now_cnt_L = encoderCount_L;
        long now_cnt_R = encoderCount_R;
        interrupts();

        speed_L = (now_cnt_L - last_cnt_L);   // encoder count per dt_ms
        speed_R = (now_cnt_R - last_cnt_R);
        last_cnt_L = now_cnt_L;
        last_cnt_R = now_cnt_R;

        // 2. 計算 PID 誤差 (以速度為控制量)
        error_L = target_speed_L - speed_L;
        error_R = target_speed_R - speed_R;

        integral_L += error_L;
        integral_R += error_R;

        // 防止積分飽和
        integral_L = constrain(integral_L, -500, 500);
        integral_R = constrain(integral_R, -500, 500);
        error_L = constrain(error_L, -20, 20);
        error_R = constrain(error_R, -20, 20);

        derivative_L = error_L - lastError_L;
        derivative_R = error_R - lastError_R;

        lastError_L = error_L;
        lastError_R = error_R;

        // 3. 計算 PWM 輸出
        pwmOutput_L = Kp_L * error_L + Ki_L * integral_L + Kd_L * derivative_L;
        pwmOutput_R = Kp_R * error_R + Ki_R * integral_R + Kd_R * derivative_R;

        pwmOutput_L = constrain(pwmOutput_L, -200, 200);
        pwmOutput_R = constrain(pwmOutput_R, -200, 200);

        // 4. 實際輸出到馬達
        if (pwmOutput_L > 0) {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            analogWrite(ENA, pwmOutput_L);
        } else if (pwmOutput_L < 0) {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            analogWrite(ENA, -pwmOutput_L);
        } else {
            analogWrite(ENA, 0);
        }

        if (pwmOutput_R > 0) {
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            analogWrite(ENB, pwmOutput_R);
        } else if (pwmOutput_R < 0) {
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            analogWrite(ENB, -pwmOutput_R);
        } else {
            analogWrite(ENB, 0);
        }

        // 5. 印出除錯資訊
        Serial.print("target_speed_L: ");
        Serial.print(target_speed_L);
        Serial.print(" current_speed_L: ");
        Serial.print(speed_L);
        Serial.print(" pwm_L: ");
        Serial.print(pwmOutput_L);
        Serial.print(" || target_speed_R: ");
        Serial.print(target_speed_R);
        Serial.print(" current_speed_R: ");
        Serial.print(speed_R);
        Serial.print(" pwm_R: ");
        Serial.println(pwmOutput_R);
    }

    // --- 停止條件（例如行駛一定距離後停） ---
    if (abs(encoderCount_R) >= 800 && abs(encoderCount_L) >= 800) {
        analogWrite(ENA, 0);
        analogWrite(ENB, 0);
        delay(200);
        noInterrupts();
        encoderCount_L = 0;
        encoderCount_R = 0;
        interrupts();
        current_cnt_L = current_cnt_R = 0;
        integral_L = integral_R = 0;
        derivative_L = derivative_R = 0;
        executing = false;
        initialized = false;
        error_L = error_R = 0;
        last_cnt_L = last_cnt_R = 0;
        delay(2000);
        motion_mode = 0;
        Serial.println("DONE");
    }
}
else if(motion_mode == 3){
    static bool initialized = false;
    static unsigned long lastTime = 0;
    static float speed_L = 0, speed_R = 0;
    static float last_cnt_L = 0, last_cnt_R = 0;

    // 期望速度（可自行設定，單位：encoder count / 取樣時間）
    const float target_speed_L = 30.0;   // 左輪往前轉速度
    const float target_speed_R = -30.0;   // 右輪往前轉速度
    const unsigned long dt_ms = 1;       // 取樣週期 50 ms

    if (!initialized) {
        initialized = true;
        noInterrupts();
        encoderCount_L = 0;
        encoderCount_R = 0;
        interrupts();

        integral_L = 0;
        integral_R = 0;
        lastError_L = 0;
        lastError_R = 0;
        last_cnt_L = 0;
        last_cnt_R = 0;
        lastTime = millis();
    }

    // --- 每 dt_ms 更新一次速度 ---
    if (millis() - lastTime >= dt_ms) {
        lastTime = millis();

        // 1. 計算目前編碼器速度 (Δcount / Δt)
        noInterrupts();
        long now_cnt_L = encoderCount_L;
        long now_cnt_R = encoderCount_R;
        interrupts();

        speed_L = (now_cnt_L - last_cnt_L);   // encoder count per dt_ms
        speed_R = (now_cnt_R - last_cnt_R);
        last_cnt_L = now_cnt_L;
        last_cnt_R = now_cnt_R;

        // 2. 計算 PID 誤差 (以速度為控制量)
        error_L = target_speed_L - speed_L;
        error_R = target_speed_R - speed_R;

        integral_L += error_L;
        integral_R += error_R;

        // 防止積分飽和
        integral_L = constrain(integral_L, -500, 500);
        integral_R = constrain(integral_R, -500, 500);
        error_L = constrain(error_L, -20, 20);
        error_R = constrain(error_R, -20, 20);

        derivative_L = error_L - lastError_L;
        derivative_R = error_R - lastError_R;

        lastError_L = error_L;
        lastError_R = error_R;

        // 3. 計算 PWM 輸出
        pwmOutput_L = Kp_L * error_L + Ki_L * integral_L + Kd_L * derivative_L;
        pwmOutput_R = Kp_R * error_R + Ki_R * integral_R + Kd_R * derivative_R;

        pwmOutput_L = constrain(pwmOutput_L, -100, 100);
        pwmOutput_R = constrain(pwmOutput_R, -100, 100);

        // 4. 實際輸出到馬達
        if (pwmOutput_L > 0) {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            analogWrite(ENA, pwmOutput_L);
        } else if (pwmOutput_L < 0) {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            analogWrite(ENA, -pwmOutput_L);
        } else {
            analogWrite(ENA, 0);
        }

        if (pwmOutput_R > 0) {
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            analogWrite(ENB, pwmOutput_R);
        } else if (pwmOutput_R < 0) {
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            analogWrite(ENB, -pwmOutput_R);
        } else {
            analogWrite(ENB, 0);
        }
    }

    // --- 停止條件（例如行駛一定距離後停） ---
      if (abs(encoderCount_R) >= 80 && abs(encoderCount_L) >= 80) {
        analogWrite(ENA, 0);
        analogWrite(ENB, 0);
        pwmOutput_L = pwmOutput_R = 0;
        delay(2000);
        noInterrupts();
        encoderCount_L = 0;
        encoderCount_R = 0;
        interrupts();
        
        integral_L = integral_R = 0;
        derivative_L = derivative_R = 0;
        lastError_L = lastError_R = 0;
        last_cnt_L = last_cnt_R = 0;
        current_cnt_L = current_cnt_R = 0;
        real_speed_L = real_speed_R = 0;
        positionError_L = positionError_R = 0;
        speed_L = speed_R = 0;
        executing = true;
        initialized = false;
        
        target_count_L = ((60 * (PI / 180.0)) * PPR * 0.4685);
        target_count_R = ((60 * (PI / 180.0)) * PPR * 0.4685);
        motion_mode = 0;
        Serial.println("Back Done");
      }
    }
    lastMillis = now;
  }
}

void encoderISR_L() {
  bool A = digitalRead(ENCODER_L_A);
  bool B = digitalRead(ENCODER_L_B);

  if (A == B) {
    encoderCount_L++;  // 正轉
  } else {
    encoderCount_L--;  // 反轉
  }
}

void encoderISR_R(){
  bool C = digitalRead(ENCODER_R_A);
  bool D = digitalRead(ENCODER_R_B);

  if (C == D) {
    encoderCount_R++;  // 正轉
  } else {
    encoderCount_R--;  // 反轉
  }
}

void updateUltrasonic() {
  static unsigned long lastTrigTime = 0;
  unsigned long now = millis();

  // 每 60ms 發一次超音波
  if (now - lastTrigTime >= 60) {
    lastTrigTime = now;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  }

  if (dataReady) {
    dataReady = false;
    unsigned long duration = timerValue / 2;
    float distance = (duration * 0.0343) / 2;

    // ---------- 遇到障礙物 ----------
    if (distance < 17 && !obstacle) {
      obstacle = true;
      reset = false;
      Serial.println("Obstacle detected!");
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
      noInterrupts();
      encoderCount_L = 0;
      encoderCount_R = 0;
      interrupts();
      integral_L = 0;
      integral_R = 0;
      derivative_L = 0;
      derivative_R = 0;
      current_cnt_L = current_cnt_R = 0;
      positionError_L = positionError_R = 0;
      error_L = error_R = 0;
      lastError_L = lastError_R = 0;
      last_cnt_L = last_cnt_R = 0;
      // 進入旋轉模式（原地左轉 60 度）
      motion_mode = 3; // ->motion_mode = 3
      executing = false;
      pwmOutput_L = 0;
      pwmOutput_R = 0;
    }
    // ---------- 避障結束 (距離恢復) ----------
    else if(distance >= 17 && obstacle){
      obstacle = false;
    }
  } 
}


// Echo 腳位狀態改變 → 進入中斷
ISR(PCINT2_vect) {
  if (digitalRead(echoPin) == HIGH) {
    TCNT1 = 0;        // 重置 Timer1
    measuring = true; // 開始量測
  } else if (measuring) {
    timerValue = TCNT1; // 記錄時間
    measuring = false;
    dataReady = true;
  }
}

#include<Arduino.h>
#include<math.h>
#define header 0xAA
// Định nghĩa chân Encoder (Kênh A và B cho 3 động cơ)
#define enco_1  34
#define enco_2  35
#define enco_3  32
#define enco_4  33
#define enco_5  25
#define enco_6  26    
// Định nghĩa chân phát xung PWM để điều khiển tốc độ  
#define PWM_1  18
#define PWM_2  21
#define PWM_3  23
// Định nghĩa chân IN để điều khiển chiều quay (Bridge H)
#define DIR_1  5 
#define DIR_2  19
#define DIR_3  22   
// Cấu hình bộ điều khiển PWM của ESP32      
#define PWM_CHANNEL_1 0   
#define PWM_CHANNEL_2 1
#define PWM_CHANNEL_3 2        
#define PWM_FREQ 100000         // Tần số 100kHz
#define PWM_RESOLUTION 8        // Độ phân giải 8 bit (0-255)
byte dulieu ;                    // Lưu dữ liệu nhận từ Serial
bool flagTime = false;          // Cờ xử lý thời gian
bool flagRun = false;           // Cờ cho phép robot chạy
bool flagStop = false;          // Cờ dừng khẩn cấp
bool flagReset = false;         // Cờ reset vị trí
bool flag = false;              // Cờ trạng thái dừng/chạy chung
bool reset_goc = false;         // Cờ đưa góc về 0
volatile long posi_1=0;         // Vị trí thực tế (xung) motor 1
volatile long posi_2=0;         // Vị trí thực tế (xung) motor 2
volatile long posi_3=0;         // Vị trí thực tế (xung) motor 3
// Trạng thái encoder trước đó để so sánh chiều quay
int lastEncoded_1 = 0;          
int lastEncoded_2 = 0;   
int lastEncoded_3 = 0;  
// Góc đích khi chọn chế độ
double target_1 = 0;    
double target_2 = 0; 
double target_3 = 0; 
double target_chuyen_1 = 0;    
double target_chuyen_2 = 0; 
double target_chuyen_3 = 0; 
// Góc đích ban đầu
double target_goc_1 = 0;    
double target_goc_2 = 0; 
double target_goc_3 = 0;  
// Góc hiện tại
double goc_1 = 0;  
double goc_2 = 0;        
double goc_3 = 0;   
// Góc trung gian
float target_change_1 = 0;         
float target_change_2 = 0;  
float target_change_3 = 0;  
// Chiều quay
int dir_1 = 0;
int dir_2 = 0;
int dir_3 = 0;
// tọa độ x y
double toado_X = 0;    
double toado_Y = 0;    
double output_1 = 0;   
double output_2 = 0;   
double output_3 = 0;   
float error_1 = 0;
float error_2 = 0;
float error_3 = 0;
float P_1 = 0;
float I_1 = 0;
float D_1 = 0;
float P_2 = 0;
float I_2 = 0;
float D_2 = 0;
float P_3 = 0;
float I_3 = 0;
float D_3 = 0;
const float kp_1 = 3.5;
const float kd_1 = 0.035;
const float ki_1 = 0.2;
const float kp_2 = 3.5;
const float kd_2 = 0.035;
const float ki_2 = 0.2;
const float kp_3 = 3.5;
const float kd_3 = 0.035;
const float ki_3 = 0.2;
unsigned long last = 0;
float lastError_1 = 0;
float lastError_2 = 0;
float lastError_3 = 0;
const int CPR_1 = 8192;
const int CPR_2 = 8192;
const int CPR_3 = 8192;
// Thông số robot (cm)
const float H = 50.0;          // Cạnh tam giác cơ sở
const float h = H / 10.0;          // Cạnh tam giác platform
const float l = 20.0;          // Chiều dài mỗi khâu (la = lb = l)
const float r  = h / sqrt(3);
// Góc offset của từng điểm C_i trên platform (rad)
const float OFFSET[3] = {
  210.0 * PI / 180.0,   // C1
  330.0 * PI / 180.0,   // C2
   90.0 * PI / 180.0    // C3
};
float offset_1 = -19.49;
float offset_2 = 100.51;
float offset_3 = -139.49;
const float A[3][2] = {
  { 0.0,                   0.0              },  // A1
  { H,                     0.0              },  // A2
  { H / 2.0,  (sqrt(3.0) / 2.0) * H        }   // A3
};
// Biến lưu tọa độ C_i
float Cx[3], Cy[3];
bool kiem_tra_workspace(float xP, float yP) {
  for (int i = 0; i < 3; i++) {
    float dx = xP + r * cos(OFFSET[i]) - A[i][0];
    float dy = yP + r * sin(OFFSET[i]) - A[i][1];
    float d  = sqrt(dx * dx + dy * dy);
    if (d >= 2.0 * l || d <= 0.001) return false;
  }
  return true;
}
void toa_do_dinh(float xP, float yP) {
  for (int i = 0; i < 3; i++) {
    Cx[i] = xP + r * cos(OFFSET[i]);
    Cy[i] = yP + r * sin(OFFSET[i]);
  }
}
void chuyen_doi(float theta[3]) {
  target_goc_1 = theta[0] * 180.0 / PI - offset_1;
  target_goc_2 = theta[1] * 180.0 / PI - offset_2;
  target_goc_3 = theta[2] * 180.0 / PI - offset_3;
}
void forwardKinematics(float theta1, float theta2, float theta3){
  float th1 = (theta1 + offset_1) * PI / 180.0;
  float th2 = (theta2 + offset_2) * PI / 180.0;
  float th3 = (theta3 + offset_3) * PI / 180.0;
  float Px;
  float Py;
  Px = l * cos(th1) + l * cos(th1 + th2) + r * cos(th1 + th2 + th3 + PI / 6.0);
  Py = l * sin(th1) + l * sin(th1 + th2) + r * sin(th1 + th2 + th3 + PI / 6.0);
  toado_X = Px;
  toado_Y = Py;
}
void tinh_goc() {
  float theta[3];
  for (int i = 0; i < 3; i++) {
    float dx = Cx[i] - A[i][0];
    float dy = Cy[i] - A[i][1];
    float d  = sqrt(dx * dx + dy * dy); 
    float alpha = acos(constrain(d / (2.0 * l), -1.0, 1.0));
    theta[i]    = atan2(dy, dx) - alpha;
  }
  chuyen_doi(theta);
}
void duong_thang(){

}
void tam_giac(){

}
void hinh_vuong(){
  
}
void gui_target() {
  int16_t t1 = target_1;
  int16_t t2 = target_2;
  int16_t t3 = target_3;
  byte t[8];
  t[0] = 0xAA;
  t[1] = 'T';
  t[2] = (byte)(t1 >> 8);
  t[3] = (byte)(t1 & 0xFF);
  t[4] = (byte)(t2 >> 8);
  t[5] = (byte)(t2 & 0xFF);
  t[6] = (byte)(t3 >> 8);
  t[7] = (byte)(t3 & 0xFF);
  Serial.write(t, 8);
}
void gui_posi() {
  int16_t p1 = posi_1;
  int16_t p2 = posi_2;
  int16_t p3 = posi_3;
  byte p[8];
  p[0] = 0xAA;
  p[1] = 'P';
  p[2] = (byte)(p1 >> 8);
  p[3] = (byte)(p1 & 0xFF);
  p[4] = (byte)(p2 >> 8);
  p[5] = (byte)(p2 & 0xFF);
  p[6] = (byte)(p3 >> 8);
  p[7] = (byte)(p3 & 0xFF);
  Serial.write(p, 8);
}
void IRAM_ATTR readEncoder_1() {
  int MSB_1 = digitalRead(enco_1); // đọc kênh A
  int LSB_1 = digitalRead(enco_2); // đọc kênh B
int encoded_1 = (MSB_1 << 1) | LSB_1;        // gộp thành 2 bit (00,01,10,11)
  int sum_1 = (lastEncoded_1 << 2) | encoded_1; // ghép trạng thái trước + hiện tại (4 bit)
  // Xác định hướng quay
  if (sum_1 == 0b1101 || sum_1 == 0b0100 || sum_1 == 0b1011 || sum_1 == 0b0010) {
    posi_1++;  // tiến
  }
  if (sum_1 == 0b1110 || sum_1 == 0b0111 || sum_1 == 0b0001 || sum_1 == 0b1000) {
    posi_1--;  // lùi
  }
  lastEncoded_1 = encoded_1;  // lưu trạng thái hiện tại cho lần sau
}
void IRAM_ATTR readEncoder_2() {
  int MSB_2 = digitalRead(enco_3); // đọc kênh A
  int LSB_2 = digitalRead(enco_4); // đọc kênh B
  int encoded_2 = (MSB_2 << 1) | LSB_2;        // gộp thành 2 bit (00,01,10,11)
  int sum_2 = (lastEncoded_2 << 2) | encoded_2; // ghép trạng thái trước + hiện tại (4 bit)
  // Xác định hướng quay
  if (sum_2 == 0b1101 || sum_2 == 0b0100 || sum_2 == 0b1011 || sum_2 == 0b0010) {
    posi_2++;  // tiến
  }
  if (sum_2 == 0b1110 || sum_2 == 0b0111 || sum_2 == 0b0001 || sum_2 == 0b1000) {
    posi_2--;  // lùi
  }
  lastEncoded_2 = encoded_2;  // lưu trạng thái hiện tại cho lần sau
}
void IRAM_ATTR readEncoder_3() {
  int MSB_3 = digitalRead(enco_5); // đọc kênh A
int LSB_3 = digitalRead(enco_6); // đọc kênh B
  int encoded_3 = (MSB_3 << 1) | LSB_3;        // gộp thành 2 bit (00,01,10,11)
  int sum_3 = (lastEncoded_3 << 2) | encoded_3; // ghép trạng thái trước + hiện tại (4 bit)
  // Xác định hướng quay
  if (sum_3 == 0b1101 || sum_3 == 0b0100 || sum_3 == 0b1011 || sum_3 == 0b0010) {
    posi_3++;  // tiến
  }
  if (sum_3 == 0b1110 || sum_3 == 0b0111 || sum_3 == 0b0001 || sum_3 == 0b1000) {
    posi_3--;  // lùi
  }
  lastEncoded_3 = encoded_3;  // lưu trạng thái hiện tại cho lần sau
}
void Pwm_out_1(int dir, int pwm_val) {
  if (dir == 1) {
    digitalWrite(DIR_1, HIGH);
    ledcWrite(PWM_CHANNEL_1, pwm_val);
  } 
  else if (dir == -1) {
    digitalWrite(DIR_1, LOW);
    ledcWrite(PWM_CHANNEL_1, abs(pwm_val));  
  }
  else{
    digitalWrite(DIR_1, HIGH);
    ledcWrite(PWM_CHANNEL_1, 0); 
  }
}  
void Pwm_out_2(int dir, int pwm_val) {
  if (dir == 1) {
    digitalWrite(DIR_2, HIGH);
    ledcWrite(PWM_CHANNEL_2, pwm_val);
  } 
  else if (dir == -1) {
    digitalWrite(DIR_2, LOW);
    ledcWrite(PWM_CHANNEL_2, abs(pwm_val));  
  }
  else{
    digitalWrite(DIR_2, HIGH);
    ledcWrite(PWM_CHANNEL_2, 0); 
  }
}  
void Pwm_out_3(int dir, int pwm_val) {
  if (dir == 1) {
    digitalWrite(DIR_3, HIGH);
    ledcWrite(PWM_CHANNEL_3, pwm_val);
  } 
  else if (dir == -1) {
    digitalWrite(DIR_3, LOW);
    ledcWrite(PWM_CHANNEL_3, abs(pwm_val));  
  }
  else{
    digitalWrite(DIR_3, HIGH);
    ledcWrite(PWM_CHANNEL_3, 0); 
  }
}  
void xuly(){
  byte rxBuf[10];
  int rxIndex = 0;
  bool state = false;
  while (Serial.available()) {
    byte c = Serial.read();
    if (!state){
      if (c == header) {
        state = true;
        rxIndex = 0;
        rxBuf[rxIndex++] = c;
      }
    }
    else {
rxBuf[rxIndex++] = c;
      // Khi có dữ liệu
      if (rxIndex == 2) {
        dulieu = rxBuf[1];
        if (dulieu == 'T') flagTime = true;
        else if (dulieu == 'C') flagRun = true;
        else if (dulieu == 'D') flagStop = true;
        else if (dulieu == 'R') flagReset = true;
        else if (dulieu == 'G'){
          
        }
        // Nhận giá trị tọa độ X,Y
        else if (dulieu == 'S') {
          while (rxIndex < 6) {
            if (Serial.available()) {
              rxBuf[rxIndex++] = Serial.read();
            }
          }
          int16_t x = (rxBuf[2] << 8) | rxBuf[3];
          int16_t y = (rxBuf[4] << 8) | rxBuf[5];
          toado_X = x/100;
          toado_Y = y/100;
          if (kiem_tra_workspace(toado_X, toado_Y)) {
            toa_do_dinh(toado_X, toado_Y);
            tinh_goc();
            target_1 = (target_goc_1 / 360.0) * CPR_1;
            target_2 = (target_goc_2 / 360.0) * CPR_2;
            target_3 = (target_goc_3 / 360.0) * CPR_3;
            gui_target();
          }
          else {
            byte data[2] = {0xAA, 'W'};
            Serial.write(data, 2);
          }
        } 
        state = false;
      }  
    }  
  }
}
void trang_thai_goc(){
  if(!reset_goc){
    target_chuyen_1 = target_1;
    target_chuyen_2 = target_2;
    target_chuyen_3 = target_3;
  } else {
    target_chuyen_1 = 0;
    target_chuyen_2 = 0;
    target_chuyen_3 = 0;
  }
}
void dieukhien(){
  if(flag) return;
  int x=38000;
  for (int i = 0; i < x; i++) {  
    unsigned long now = micros();
    float dt = ((float)(now - last))  / 1.0e6; 
    last=now;
    goc_1 = ((double)posi_1 / CPR_1) * 360.0;
    goc_2 = ((double)posi_2 / CPR_2) * 360.0;
    goc_3 = ((double)posi_3 / CPR_3) * 360.0;
    trang_thai_goc();
    target_change_1 = (target_chuyen_1-posi_1)*i/x + posi_1; 
    target_change_2 = (target_chuyen_2-posi_2)*i/x + posi_2;     
    target_change_3 = (target_chuyen_3-posi_3)*i/x + posi_3;     
    error_1 = target_change_1-posi_1;
    error_2 = target_change_2-posi_2;
    error_3 = target_change_3-posi_3;
    P_1 = kp_1 * error_1;
    I_1 += ki_1 * error_1 * dt;
    D_1 = kd_1 * (error_1 - lastError_1) / dt;
    output_1 = P_1 + I_1 + D_1;
    lastError_1 = error_1;
    P_2 = kp_2 * error_2;
    I_2 += ki_2 * error_2 * dt;
    D_2 = kd_2 * (error_2 - lastError_2) / dt;
    output_2 = P_2 + I_2 + D_2;
    lastError_2 = error_2;
    P_3 = kp_3 * error_3;
    I_3 += ki_3 * error_3 * dt;
    D_3 = kd_3 * (error_3 - lastError_3) / dt;
    output_3 = P_3 + I_3 + D_3;
    lastError_3 = error_3;
    int pwm_1 = (int)fabs(output_1);
    pwm_1 = constrain(pwm_1, 0, 100);
    dir_1 = (output_1 < 0) ? -1 : 1;
    Pwm_out_1(dir_1, pwm_1);
    int pwm_2 = (int)fabs(output_2);
    pwm_2 = constrain(pwm_2, 0, 100); 
    dir_2 = (output_2 < 0) ? -1 : 1;
    Pwm_out_2(dir_2, pwm_2);
    int pwm_3 = (int)fabs(output_3);
    pwm_3 = constrain(pwm_3, 0, 100); 
    dir_3 = (output_3 < 0) ? -1 : 1;
Pwm_out_3(dir_3, pwm_3);
    if (i % 380 == 0) {  
      gui_posi();
    }
  }
}
void thoigian(){
  if (flagTime) {  
    byte data[2] = {0xAA, 'D'};
    Serial.write(data, 2);
    flagTime = false;    
  }
}

void reset(){
  if (flagReset) {
    flagReset = false; 
    reset_goc = true;
  }
}
void chay(){
  if (flagRun) {
      flagRun = false; 
      flag=false;
      reset_goc = false;
  }
}
void dung(){
  if (flagStop) {
    flagStop = false;  
    for (int p = 150; p >= 0; p -= 5) {
      Pwm_out_1(0, p);
      Pwm_out_2(0, p);
      Pwm_out_3(0, p);
      delay(10);
    }
    flag=true;
  }
}
void setup() {
  Serial.begin(115200);
  pinMode(enco_1,INPUT_PULLUP); 
  pinMode(enco_2,INPUT_PULLUP);
  pinMode(enco_3, INPUT_PULLUP);
  pinMode(enco_4, INPUT_PULLUP);
  pinMode(enco_5, INPUT_PULLUP);
  pinMode(enco_6, INPUT_PULLUP);
  pinMode(DIR_1,OUTPUT);
  pinMode(DIR_2,OUTPUT);
  pinMode(DIR_3, OUTPUT);
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_1, PWM_CHANNEL_1);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_2, PWM_CHANNEL_2);
  ledcSetup(PWM_CHANNEL_3, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_3, PWM_CHANNEL_3);
  attachInterrupt(digitalPinToInterrupt(enco_1),readEncoder_1,CHANGE);
  attachInterrupt(digitalPinToInterrupt(enco_2),readEncoder_1,CHANGE);
  attachInterrupt(digitalPinToInterrupt(enco_3),readEncoder_2,CHANGE);
  attachInterrupt(digitalPinToInterrupt(enco_4),readEncoder_2,CHANGE);
  attachInterrupt(digitalPinToInterrupt(enco_5),readEncoder_3,CHANGE);
  attachInterrupt(digitalPinToInterrupt(enco_6),readEncoder_3,CHANGE);
}
void loop() {
  xuly(); 
  chay();
  dung();
  reset();
  dieukhien();
  thoigian();
}

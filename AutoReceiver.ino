const int left_res = A1;
const int right_res = A7;


void setup() {
 Serial.begin(9600);
 pinMode(PIN_MOTOR_DIR, OUTPUT); //принимаем 
 pinMode(PIN_MOTOR_PWM, OUTPUT);
 analogReference(EXTERNAL); //ареф

}

void loop() {
  float a1 = analogRead(A1);
  float a7 = analogRead(A7);

  float raz = v1 - v7;  //разница напряжений
  

}



int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {    //подключение и вычисление пид регулятора
    float err = setpoint - input;
    static float integral = 0, prevErr = 0; 
    integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
    float D = (err - prevErr) / dt;
    prevErr = err;
    
    return constrain(err * kp + integral + D * kd, minOut, maxOut);
}

const int ENCODER_PIN = 2;   
const int MOTOR_PIN1 = 9;    
const int MOTOR_PIN2 = 8;    
const int PWM_PIN = 10;      

double dt, last_time;
double integral, previous, output = 0;
double kp, ki, kd;
double setpoint = 20.00;   

volatile long encoderCount = 0;  
long lastEncoderCount = 0;     
unsigned long lastEncoderTime = 0;  
float currentSpeed = 0; 

void setup() {
  kp = 0.8;
  ki = 0.20;
  kd = 0.001;
  last_time = millis();
  
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, CHANGE);

  Serial.begin(9600);
  analogWrite(PWM_PIN, 0);
}

void loop() {
  double now = millis();
  dt = (now - last_time) / 1000.0; 
  last_time = now;

 
  if (millis() - lastEncoderTime >= 100) {  
    currentSpeed = (encoderCount - lastEncoderCount) * 60000.0 / 900.0 / 100;  
    lastEncoderCount = encoderCount;
    lastEncoderTime = millis();
  }

  double error = setpoint - currentSpeed;
  output = pid(error);

  // Control motor with PID output
  controlMotor(output);

  // Print for debugging
  Serial.print("Setpoint: "); Serial.print(setpoint);
  Serial.print(", Actual Speed: "); Serial.print(currentSpeed);
  Serial.print(", Output: "); Serial.println(output);

  delay(100);  // Loop delay
}

void encoderISR() {
  encoderCount++;
}

double pid(double error) {
  double proportional = error;
  integral += error * dt;
  
  // Prevent integral windup
  integral = constrain(integral, -255, 255);
  
  double derivative = (error - previous) / dt;
  previous = error;
  
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  
  return constrain(output, -255, 255);  
}

void controlMotor(double speed) {
  if (speed > 0) {
    digitalWrite(MOTOR_PIN1, HIGH);
    digitalWrite(MOTOR_PIN2, LOW);
  } else if (speed < 0) {
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, HIGH);
    speed = -speed; 
  } else {
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, LOW);
  }
  
  analogWrite(PWM_PIN, constrain(speed, 0, 255)); 
}


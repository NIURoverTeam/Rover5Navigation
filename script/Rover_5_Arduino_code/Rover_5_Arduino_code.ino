#define right_pwr 10
#define left_pwr 11
#define right_dir 12
#define left_dir 13
#define REVERSE HIGH
#define FORWARD LOW

unsigned long tick;

void setup() {
  pinMode(left_pwr, OUTPUT);
  pinMode(right_pwr, OUTPUT);
  pinMode(left_dir, OUTPUT);
  pinMode(right_dir, OUTPUT);
  Serial.begin(9600);
  tick = millis();
}


void loop()
{
  if(Serial.available()) {
    uint32_t first = Serial.read();
    while(!Serial.available());
    uint32_t second = Serial.read();
    
    switch (first) {
      case 'L':
        forwardMove(left_pwr, left_dir, second);
        break;
      case 'R':
        forwardMove(right_pwr, right_dir, second);
        break;
      case 'l':
        reverseMove(left_pwr, left_dir, second);
        break;
      case 'r':
        reverseMove(right_pwr, right_dir, second);
        break;
      default:
        break;
    }
  }
  
  if (millis() - tick >= 1000) {
    stopMovement();
    tick = millis();
  }
}

void forwardMove(int motor_pin, int dir_pin, int pwm) {
  analogWrite(motor_pin, pwm);
  digitalWrite(dir_pin, FORWARD);
}

void reverseMove(int motor_pin, int dir_pin, int pwm) {
  analogWrite(motor_pin, pwm);
  digitalWrite(dir_pin, REVERSE);
}

void stopMovement () {
  digitalWrite(left_pwr, 0);
  analogWrite(left_dir, FORWARD);
  digitalWrite(right_pwr, 0);
  analogWrite(right_dir, FORWARD);
}

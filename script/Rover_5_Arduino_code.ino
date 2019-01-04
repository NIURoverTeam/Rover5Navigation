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
    if (first == 'l' || first == 'r') {
      uint32_t second = Serial.read();
      while(!Serial.available());
      uint32_t third = Serial.read();
      movement(first, (int) second, (char) third);
    }
  }
  
  if (millis() - tick >= 1000) {
    stopMovement();
    tick = millis();
  }
}

void movement (char side, int pwm, char dir) {
  analogWrite((side=='L'?left_pwr:right_pwr), pwm);
  digitalWrite((side=='L'?left_dir:right_dir), (dir=='B'?REVERSE:FORWARD));
}

void stopMovement () {
  digitalWrite(left_pwr, 0);
  analogWrite(left_dir, FORWARD);
  digitalWrite(right_pwr, 0);
  analogWrite(right_dir, FORWARD);
}


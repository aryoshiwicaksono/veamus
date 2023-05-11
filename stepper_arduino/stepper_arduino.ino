int dirPin = 5;
int stepPin = 6;

void setup() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
}

void loop() {
  int silang = analogRead(A0);
  int lingkaran = analogRead(A1);

  if(silang > 100){ 	
    digitalWrite(dirPin, HIGH);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }

  if(lingkaran > 100){
    digitalWrite(dirPin, LOW);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }

}

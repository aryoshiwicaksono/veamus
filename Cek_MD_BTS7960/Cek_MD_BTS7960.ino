#define RPWM 5
#define LPWM 6
#define REN 8
#define LEN 9


int pot;
int out1;
int out2;

void setup() {
  Serial.begin(9600);
  pinMode(RPWM,OUTPUT);
  pinMode(LPWM,OUTPUT);
  pinMode(LEN,OUTPUT);
  pinMode(REN,OUTPUT);
  digitalWrite(REN,HIGH);
  digitalWrite(LEN,HIGH);

}
 
 
void loop() {
  
    digitalWrite(RPWM,HIGH);
    digitalWrite(LPWM,LOW);
    delay(1000);
  
  
    digitalWrite(LPWM,HIGH);
    digitalWrite(RPWM,LOW);
        delay(1000);

    
  
}
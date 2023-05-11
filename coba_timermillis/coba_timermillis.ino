unsigned long oldtime = 0;
int c;

void setup() {
  Serial.begin(9600);

}

void loop() {
 if (millis() - oldtime >= 1000){
    Serial.print("Interrupt dengan Millis() :");
    c++;
    Serial.println(c);
    oldtime = millis();
  }
}

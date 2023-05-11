//ESP32
#include <HardwareSerial.h>
#include <Ps3Controller.h>

#define DATALEN 8
uint8_t data[8];

HardwareSerial SerialPort(2); // use UART2

void setup()  
{
  SerialPort.begin(115200, SERIAL_8N1, 16, 17);
  Serial.begin(9600);
  // Ps3.begin("00:02:03:04:05:09"); 
  Ps3.begin("00:1a:7d:da:71:15");

} 
void loop()  
{ 

  if (Ps3.isConnected()){
    data[0] = (Ps3.data.button.up << 5) | (Ps3.data.button.right << 4) | (Ps3.data.button.down << 3)| (Ps3.data.button.left << 2) | (Ps3.data.button.start << 1) | Ps3.data.button.select;
    data[1] = (Ps3.data.button.triangle << 5) | (Ps3.data.button.circle << 4) | (Ps3.data.button.cross << 3) | (Ps3.data.button.square << 2) | (Ps3.data.button.l1 << 1) | Ps3.data.button.r1;
    data[2] = Ps3.data.analog.stick.lx;
    data[3] = Ps3.data.analog.stick.ly;
    data[4] = Ps3.data.analog.stick.rx;
    data[5] = Ps3.data.analog.stick.ry;
    data[6] = Ps3.data.analog.button.l2;
    data[7] = Ps3.data.analog.button.r2;

    for(int i=2; i<6; i++){
      if(data[i] > 117 && data[i] < 137){
        data[i] = 127;
    } 
  }

  for(int i=6; i<8; i++){
    if(data[i] < 10){
      data[i] = 0;
    }
  }
  

  SerialPort.write(249);
  SerialPort.write(data, DATALEN);
  // data = random(10);
  // Serial.println(data);
  // SerialPort.print(data);
  // Serial.print(Ps3.data.button.up);
  // Serial.print(Ps3.data.button.right);
  // Serial.print(Ps3.data.button.down);
  // Serial.println(Ps3.data.button.left);
  Serial.print("lx = ");
  Serial.println(Ps3.data.analog.stick.lx);
    Serial.print("ly = ");
  Serial.println(Ps3.data.analog.stick.ly);
      Serial.print("rx = ");
  Serial.println(Ps3.data.analog.stick.rx);
   Serial.print("ry = ");
  Serial.println(Ps3.data.analog.stick.ry);
  
  delay(1000);
}
//  SerialPort.write(0);
}

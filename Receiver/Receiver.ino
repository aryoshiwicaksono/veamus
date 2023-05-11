//ESP32S3

#include <HardwareSerial.h>

HardwareSerial SerialPort(2); // use UART2

// Setting PWM properties
const int freq = 30000;
const int resolution = 8;

// Millis Properties
int c;
unsigned long oldtime[6];


// Variabel remote PS3
int datatest, dataarduino[8];
int datacounter, isdatareceived;

int segitiga, lingkaran, silang, kotak;
int up, right, down, left, start, selects;
int Lx, Ly, Rx, Ry;
int L1, R1, L2, R2;
uint8_t c;



/* Motor assignment

    FRONT  

  /r1    r2\
              RIGHT
  \r4    r3/

*/
float r1,r2,r3,r4;

int motor[4][3] = {
  //in1,in2, pwm
  {19, 20, 21}, //r1
  {45, 0, 35},  //r2
  {38, 39, 40}, //r3
  {14, 13, 12}  //r4
};

int pneumatic1 = 3;
int pneumatic2 = 8;
int stepperSilang = 9;
int stepperLingkaran = 46;


void setup()
{
  SerialPort.begin(115200, SERIAL_8N1, 18, 17);
  for(int i = 0; i<8 ; i++) dataarduino[i] = 0;
  
  pinMode(pneumatic1, OUTPUT);
  pinMode(pneumatic2, OUTPUT);

  pinMode(stepperSilang, OUTPUT);
  pinMode(stepperLingkaran, OUTPUT);

  motor_init(motor[0][0],motor[0][1],motor[0][2],0);
  motor_init(motor[1][0],motor[1][1],motor[1][2],1);
  motor_init(motor[2][0],motor[2][1],motor[2][2],2);
  motor_init(motor[3][0],motor[3][1],motor[3][2],4);

}

void loop()
{
  if (SerialPort.available())
  {
    c = SerialPort.read();
    receivehandler(c);
  }
  int speed = 50;

  r1 =  (up - down + right - left)*1.3  + (+ R1 - L1)*1.3;
  r2 =  (-up + down + right - left)*1.3 + (+ R1 - L1)*1.3;
  r3 =  (-up + down - right + left)*1.3 + (+ R1 - L1)*1.3;
  r4 =  (up - down - right + left)*1.3  + (+ R1 - L1)*1.3;

//Pneumatic
if(segitiga){
  oldtime[0] = millis();
  digitalWrite(pneumatic1, HIGH);
   if (millis() - oldtime[0] >= 1000){
    digitalWrite(pneumatic1, LOW);
  }
}
if(kotak){
  oldtime[1] = millis();
  digitalWrite(pneumatic2, HIGH);
   if (millis() - oldtime[1] >= 1000){
    digitalWrite(pneumatic2, LOW);
  }
}

//Stepper
  if(silang){ 	
    digitalWrite(stepperSilang, HIGH);
  }
  else{
    digitalWrite(stepperSilang, LOW);
  }
  if(lingkaran){ 	
    digitalWrite(stepperLingkaran, HIGH);
  }
  else{
    digitalWrite(stepperLingkaran, LOW);
  }


  motor_set_speed(motor[0][0],motor[0][1],motor[0][2], r1*speed, 0);
  motor_set_speed(motor[1][0],motor[1][1],motor[1][2], r2*speed, 1);
  motor_set_speed(motor[2][0],motor[2][1],motor[2][2], r3*speed, 2);
  motor_set_speed(motor[3][0],motor[3][1],motor[3][2], r4*speed, 4);
  
  // motor_set_speed(motor[0][0],motor[0][1],motor[0][2], 50, 0);
  // motor_set_speed(motor[1][0],motor[1][1],motor[1][2], 50, 1);
  // motor_set_speed(motor[2][0],motor[2][1],motor[2][2], 50, 2);
  // motor_set_speed(motor[3][0],motor[3][1],motor[3][2], 50, 4);
  // delay(2000);
  // motor_set_speed(motor[0][0],motor[0][1],motor[0][2], -50, 0);
  // motor_set_speed(motor[1][0],motor[1][1],motor[1][2], -50, 1);
  // motor_set_speed(motor[2][0],motor[2][1],motor[2][2], -50, 2);
  // motor_set_speed(motor[3][0],motor[3][1],motor[3][2], -50, 4);
  // delay(2000);

}

void receivehandler(uint8_t c){
	if(c == 249){
		isdatareceived = 1;
	}else if(isdatareceived){
		dataarduino[datacounter] = c;
		datacounter++;

		if(datacounter == 8){
			isdatareceived = 0;
			datacounter    = 0;
		}
	}

	up 	    = (dataarduino[0] >> 5) & 0x01;
	right   = (dataarduino[0] >> 4) & 0x01;
	down    = (dataarduino[0] >> 3) & 0x01;
	left    = (dataarduino[0] >> 2) & 0x01;
	start   = (dataarduino[0] >> 1) & 0x01;
	selects = (dataarduino[0] >> 0) & 0x01;

	segitiga  = (dataarduino[1] >> 5) & 0x01;
	lingkaran = (dataarduino[1] >> 4) & 0x01;
	silang    = (dataarduino[1] >> 3) & 0x01;
	kotak 	  = (dataarduino[1] >> 2) & 0x01;
	L1 	 	    = (dataarduino[1] >> 1) & 0x01;
	R1		    = (dataarduino[1] >> 0) & 0x01;

	Lx = dataarduino[2];
	Ly = dataarduino[3];
	Rx = dataarduino[4];
	Ry = dataarduino[5];
	L2 = dataarduino[6];
	R2 = dataarduino[7];
}

void motor_init(int in1, int in2, int pwm, int pwmChannel){
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  ledcAttachPin(pwm, pwmChannel);  
  ledcSetup(pwmChannel, freq, resolution);
}

void motor_set_speed( int in1, int in2, int pwm, float percentage, int pwmChannel){ //masukkan kecepatan dalam bentuk persen -100% sampai 100%
	// error handling supaya kecepatan yang dimasukkan tidak bisa kurang dari -100% atau lebih dari 100%
	if (percentage < -100){
		percentage = -100;
	}else if (percentage > 100){
		percentage = 100;
	}else{
		percentage = percentage;
	} 

	// fungsi untuk setting kecepatan setiap motor sesuai persentasi kecepatannya
	if (percentage<0){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
		percentage *= -1;
	}
	else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
	}
    percentage = map(percentage, 0,100,0,255);
    ledcWrite(pwmChannel, percentage);   
}




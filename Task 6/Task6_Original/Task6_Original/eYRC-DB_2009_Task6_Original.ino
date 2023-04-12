
//#include <ServoTimer2.h>
#include <Servo.h>
#include <Wire.h>
#define MPU6050       0x68         // MPU6050 I2C address
#define ACCEL_CONFIG  0x1C          //Talk to the ACCEL_CONFIG register
#define GYRO_CONFIG   0x1B          // Talk to the GYRO_CONFIG register
#define PWR_MGMT_1    0x6B           // Talk to the register 6B
#define PWR_MGMT_2    0x6C            // Talk to the register 6c
int buzzerPin = 30; // Set the pin for the buzzer  
bool buz=false; bool flag1=false; bool flag2=false; bool flag3=false; bool flag4=false; int pos=0;
#define BRAKE         24         //PA2
#define PWM           10        //PB5    //OC1A
#define DIRECTION     25           //PA3
long buztime=1000;
#define enA           5       //    // OC3A
#define in1           22          //PA0
#define in2           23           //PA1
#define steerpin      44       //O  
#define deliverpin     45 
int thresh=2; bool boost=false; int pcp; bool moveflag=true;
Servo steering;
Servo del; 
int da=0; int db=0; int dc=0; int dd=0; int de=0;  int minm=300; int maxm=950; long is; float pna=0; float pnb=0; float pnc=0; float pnd=0; float pne=0; float last_error=0;
int a=1; int b=1; int c=1; int d=1; int e=1; bool turn; int dl=0; float proportional; float integral=0; float last_proportional=0; float error=0; float derivative=0;
long atime=0;
int pa=1; int pb=1; int pc=1; int pd=1; int pe=1; int cp=0; long stime=0;
float k1 = 115.0;                                // k1 obtained from LQR stabilizer
float k2 = 15;                                // k2 obtained from LQR stabilizer
float k3 = 0.04;                                // k3 obtained from LQR stabilizer
float timeloop = 10;                            // timeloop for nidec motor
int startT=0;
int pwm = 0;                                    // the pwm which is given to nidec motor

int32_t motor_speed;                            // motor speed obtained from previous loop
long thisT; 
long lastT=0; 
long steerT = 0; 
int16_t AcX, AcY, AcZ, GyZ, gyroZ;               // angles obtained from  gyroscope

//Sensor output scaling
#define accSens 0             // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1            // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s
#define Gyro_amount 0.996     // percent of gyro in complementary filter

//IMU offset values
int16_t  AcX_offset = -750;
int16_t  AcY_offset = 360;
int16_t  AcZ_offset = 0;
int16_t  GyZ_offset = 0;
int32_t  GyZ_offset_sum = 0;
float alpha = 0.40;     
float zdot;
int steer;
float zangle;
float Acc_angle;
 bool flag=false;
bool vertical = false;  



void setup() {
  Serial.begin(115200);

 TCCR2B = 0b00000010;
 TCCR2A = 0b00000011; 
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  pinMode(PWM, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  digitalWrite(BRAKE, LOW);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);
  Serial.println("TRSensor example");
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  steering.attach(steerpin);
   del.attach(deliverpin);
  delay(1000);
  angle_setup();
  analogWrite(PWM, 0);
  digitalWrite(buzzerPin, HIGH);
  delay(1000);
  digitalWrite(buzzerPin, LOW);
}

void loop() {
  // This code balances the bike, the flywheel gives torque every 10ms 
  thisT = millis();
  
  if (thisT - lastT >= timeloop) {
    angle_get();
    if (vertical) {
      digitalWrite(BRAKE, HIGH);
      gyroZ = GyZ / 131.0;                      // Conversion to degree/second
      
      zdot = alpha * gyroZ + (1 - alpha) * zdot;    // deciding the angular velocity of bike by using gyroscope
      pwm = -constrain(k1 * zangle + k2 * zdot + k3 * -motor_speed, -255, 255); // we used k matrix obtained from lqr method now applying feedback control U=-kX logic using optimal matrix 

   Nidec_motor_control(pwm);
      motor_speed += pwm;
    } else {
    Nidec_motor_control(0);
      digitalWrite(BRAKE, LOW);
      motor_speed = 0;
     
    }
lastT= thisT;
}
// reading analog values of IR sensor to follow line and count checkpoints and delivery nodes
 a=analogRead(A0); b=analogRead(A1); c=analogRead(A2); d=analogRead(A3); e=analogRead(A4); float na=a; float nb=b; float nc=c; float nd=d; float ne=e;
pna=na; pnb=nb; pnc=nc; pnd=nd; pne=ne;
if (pna>=900){pna=1;}else{pna=0;} if (pnb>=900){pnb=1;}else{pnb=0;} if (pnc>=900){pnc=1;}else{pnc=0;} if (pnd>=900){pnd=1;}else{pnd=0;} if (pne>=900){pne=1;}else{pne=0;}

// normalising the IR values from range 1-1000 for line following using alalog values
na=(na-minm)/(maxm-minm);
nb=(nb-minm)/(maxm-minm);
nc=(nc-minm)/(maxm-minm);
nd=(nd-minm)/(maxm-minm);
ne=(ne-minm)/(maxm-minm);

// Counting the checkpoints, a maxima comes followed by minima of analog read values, we use this event for counting checkpoint
if((a+b+c+d+e)>=4750){
  flag1=true;
}
//if(cp==5 && dl!=3){
if(flag1 && (a+b+c+d+e)<=3200){ flag2=true;}
if(flag1 && flag2)
{cp=cp+1; flag1=false; flag2=false;}
Serial.print("checkpoints  is: ");
  Serial.print(cp);


//This is a conditional statement that checks if a delivery location  has come, every 130 milliseconds.
  // counting the delivery nodes, 130 ms a loop continously looks for change in IR values (which were converted into 0 an 1) that corresponds to a delivery node
  // also there is atleast difference of 2 in count of checkpoints(cp) and delivery nodes(dl) traversed  
if(thisT-atime>=130){
  da=a; db=b; dc=c; dd=d; de=e;

if (da>=900){da=1;}else{da=0;} if (db>=900){db=1;}else{db=0;} if (dc>=900){dc=1;}else{dc=0;} if (dd>=900){dd=1;}else{dd=0;} if (de>=900){de=1;}else{de=0;}
if(cp==4){thresh=3;} else{thresh=2;}

if((de-pa)==1 && dc==1 && de-pe==1 && dd==1 && pa!=1 && cp-dl>=thresh && pa==0 && da==0) // this is how a delivery location is counted
{dl=dl+1;  // Generate a tone at the specified frequency
}
atime=thisT;
}

Serial.print("count del :");
Serial.print(dl);

// slows down the bike at extreamities of white line so that bike does not leave track

  if((pna==1 || pne==1) && pnc==0 && (cp==5 || cp==6 || cp==4 || cp==2)){
error=whitelinefollow(na,nb,nc,nd,ne,2000, 1.7, 0.000865, 0.1, pcp,cp);}
else{ if(cp==5){ error=whitelinefollow(na,nb,nc,nd,ne,2000, 1.8, 0.0008, 0, pcp,cp); } 
else {error=whitelinefollow(na,nb,nc,nd,ne,2000, 1.33, 0.0008, 0.2, pcp,cp); }}

 

 


 

if(dl==1 && cp==2){deliver();}  /// first delivery at apartment
if(dl==2 && cp==4){deliver();}    // second deliver at hospital 
if(dl==4 && cp==5){deliver(); analogWrite(enA, 0); moveflag=false; }
//digitalWrite(buzzerPin, HIGH);
//  delay(1000);
//  digitalWrite(buzzerPin, LOW); }           // third deliver at old age home

if((cp==1 || cp==2) && dl==0){nodeliver();}  // no delivery when 
if((cp==3 || cp==4) && dl==1){nodeliver();} //no delivery when
if(cp==5 && dl==2){nodeliver();} 
if(dl<1){nodeliver();} 
if(dl==3 && cp==5){nodeliver();} //ghost
if(dl==5 && cp==5){nodeliver();} //ghost
if(dl==5 && cp==6){nodeliver();}
if(dl==5 && cp==7){nodeliver();}





// bike speed is adjusted here, at last delivery the bike stops and buzzes
digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  if(moveflag){
  if((pne==1 || pnd==1) && pnc==0) { 
 analogWrite(enA, 130);} else{  analogWrite(enA, 180);}
  }


pa=da; pb=db; pc=dc; pd=dd; pe=de;
pcp=cp;
}

#include <Wire.h>

#define MPU6050       0x68         // MPU6050 I2C address
#define ACCEL_CONFIG  0x1C          //Talk to the ACCEL_CONFIG register
#define GYRO_CONFIG   0x1B          // Talk to the GYRO_CONFIG register
#define PWR_MGMT_1    0x6B           // Talk to the register 6B
#define PWR_MGMT_2    0x6C            // Talk to the register 6c

#define BRAKE         6   //PH3//OC4A
#define PWM           11  //PB5//OC1A
#define DIRECTION     7   //PH4//0C4B

#define enA           5
#define in1           22
#define in2           23

const uint16_t frequency = 20000;                 // The motor driver can handle a PWM frequency up to 20kHz
const uint16_t PWMVALUE = F_CPU / frequency / 2;  // The frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, prescaling is used so the frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/frequency/2

float k1 = 75.0; 
float k2 = 5.25;   
float k3 = 0.04;  
float timeloop = 10;  

int pwm = 0;
byte dir;
int32_t motor_speed; 
uint32_t timer;
long thisT, lastT = 0; 
int16_t AcX, AcY, AcZ, GyZ, gyroZ;

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

float zangle;
float Acc_angle;

bool vertical = false;  

uint8_t i2cData[14]; // Buffer for I2C data

void setup() {
  Serial.begin(115200);

  
  TCCR1B = (1 << WGM13) | (1 << CS10);  // matching  PWM Phase and Frequency with NIDEC motor  (ICR1 as TOP count, no prescaling)
  ICR1 = PWMVALUE;                      // highest value is ICR1, matching so the frequency is equal to 20kHz

 
  TCCR1A = (1 << COM1A1) | (1 << COM1B1); // reset OC1A/OC1B when compare match while up-counting and set OC1A/OC1B when compare matches while downcounting
  
  setspeed(400); 
  pinMode(PWM, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  digitalWrite(BRAKE, HIGH);
  delay(1000);
  angle_setup();
}

void loop() {
  
   
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
   delay(10);
 
  
}

#include <Servo.h>
#include <FastTrig.h>

#define servoDepanKanan_0 PB10
#define servoDepanKanan_1 PB1
#define servoDepanKanan_2 PB0
#define servoTengahKanan_0 PA7
#define servoTengahKanan_1 PA6
#define servoTengahKanan_2 PA5
#define servoBelakangKanan_0 PA0
#define servoBelakangKanan_1 PA1
#define servoBelakangKanan_2 PB6
#define servoBelakangKiri_0 PB7
#define servoBelakangKiri_1 PB9
#define servoBelakangKiri_2 PB8
#define servoTengahKiri_0 PA8
#define servoTengahKiri_1 PA9
#define servoTengahKiri_2 PA10
#define servoDepanKiri_0 PB13
#define servoDepanKiri_1 PB14
#define servoDepanKiri_2 PB15

Servo servo0_0;
Servo servo0_1;
Servo servo0_2;
Servo servo1_0;
Servo servo1_1;
Servo servo1_2;
Servo servo2_0;
Servo servo2_1;
Servo servo2_2;
Servo servo3_0;
Servo servo3_1;
Servo servo3_2;
Servo servo4_0;
Servo servo4_1;
Servo servo4_2;
Servo servo5_0;
Servo servo5_1;
Servo servo5_2;

#define CoxaLength 77  //30 //waktu dibagi dengan
#define FemurLength 49
#define TibiaLength 103
#define rate 4  // ketelitian langkah  //harus ganjil
#define delayTanggaTama 10
#define lebarLangkah 39   // lebar langkah dibagi rate*2 harus bulat 40 4 32 //harus bulat agar stabil
#define tinggiLangkah 32  //tinggi langkah dibagi rate harus bulat
#define iterasi 0.01
#define Zoff 40
#define PosX 0
#define PosY 0
#define PosZ 0
#define RotX 0
#define RotY 0
#define RotZ 0
#define BodyCenterOffsetX_0 62.64
#define BodyCenterOffsetX_1 72
#define BodyCenterOffsetX_2 62.64
#define BodyCenterOffsetX_3 -62.64
#define BodyCenterOffsetX_4 -72
#define BodyCenterOffsetX_5 -62.64
#define BodyCenterOffsetY_0 70
#define BodyCenterOffsetY_1 0
#define BodyCenterOffsetY_2 -70
#define BodyCenterOffsetY_3 -70
#define BodyCenterOffsetY_4 0 
#define BodyCenterOFfsetY_5 70

int servoDepanKanan_0 ;
int servoDepanKanan_1 ;
int servoDepanKanan_2 ;
int servoTengahKanan_0 ;
int servoTengahKanan_1 ;
int servoTengahKanan_2 ;
int servoBelakangKanan_0; 
int servoBelakangKanan_1 ;
int servoBelakangKanan_2 ;
int servoBelakangKiri_0 ;
int servoBelakangKiri_1 ;
int servoBelakangKiri_2 ;
int servoTengahKiri_0 ;
int servoTengahKiri_1 ;
int servoTengahKiri_2 ;
int servoDepanKiri_0 ;
int servoDepanKiri_1 ;
int servoDepanKiri_2 ;

void InitialLeg () {
  Serial.begin(1000000) ;
  servoDepanKanan_0 = (cos(60/180*PI)*(CoxaLength+FemurLength)) ;
  servoDepanKanan_2 = TibiaLength ;
  servoDepanKanan_1 = (sin(60/180*PI)*(CoxaLength + FemurLength)) ;
  servoTengahKanan_0 = (CoxaLength + FemurLength) ;
  servoTengahKanan_2 = TibiaLength ;
  servoTengahKanan_1 = 0 ;
  servoBelakangKanan_0 = (cos(60/180*PI)*(CoxaLength + FemurLength)) ;
  servoBelakangKanan_2 = TibiaLength ;
  servoBelakangKanan_1 = (sin(-60/180*PI)*(CoxaLength + FemurLength)) ;
  servoBelakangKiri_0 = (-cos(60/180*PI)*(CoxaLength + FemurLength)) ;
  servoBelakangKiri_2 = TibiaLength ; 
  servoBelakangKiri_1 = (sin(-60/180*PI)*(CoxaLength + FemurLength)) ;
  servoTengahKiri_0 = -(CoxaLength + TibiaLength) ; 
  servoTengahKiri_2 = TibiaLength ;
  servoTengahKiri_1 = 0 ;
  servoDepanKiri_0 = (-cos(60/180*PI)*CoxaLength + FemurLength );
  servoDepanKiri_2 = TibiaLength ;
  servoDepanKiri_1 = sin(60/180*PI)* (CoxaLength + FemurLength);

}

void BodyIk () {
  TotalY_1 = servoDepanKanan_1 + BodyCenterOffsetY_1 + PosY ;
  TotalX_1 = servoDepanKanan_0 + BodyCenterOffsetX_1 + PosX ;
  DistBodyCenterFeet_1 = sqrt(TotalY_1^2 + TotalX_1^2) ;
  AngleBodyCenterX_1 = PI/2 - atan2(TotalY_1, TotalX_1);
  RollZ_1 = tan(RotZ * PI/180) * TotalX_1 ;
  PitchZ_1 = tan(RotX * PI/180) * TotalY_1 ; 
  BodyIKX_1 = cos(AngleBodyCenterX_1 + (RotY * PI/180)) * DistBodyCenterFeet_1 - TotalX_1 ;
  BodyIKY_1 = (sin(AngelBodyCenterX_1 + (RotY * PI/180)) * DistBodyCenterFeet_1) - TotalY_1 ; 
  BodyIKZ_1 = Rollz_1 + PitchZ_1 ;

  TotalY_2 = servoTengahKanan_1 + BodyCenterOffsetY_2 + PosY ; 
  TotalX_2 = servoTengahKanan_0 + PosX + BodyCenterOffsetX_2 ;
  DistBodyCenterFeet_2 = sqrt(TotalY_2^2 + TotalX_2^2) ; 
  AngleBodyCenterX_2 = PI/2 - atan2(TotalY_2,TotalY_2) ; 
  RollZ_2 = tan(RotZ * PI/180) * TotalX_2 ;
  PitchZ_2 = tan(RotX * PI/180) * TotalY_2 ; 
  BodyIKX_2 = cos(AngleBodyCenterX_2 + (RotY * PI/180)) * DistBodyCenterFeet_2 - TotalX_2 ;
  BodyIKY_2 = (sin(AngleBodyCenterX_2) + (RotY * PI/180)) * DistBodyCenterFeet_2 - TotalY_2 ;
  BodyIKZ_2 = RollZ_2 + PitchZ_2 ; 

  TotalY_3 = 






}
void setup() {
    servo0_0.attach(servoDepanKanan_0);
  servo0_1.attach(servoDepanKanan_1);
  servo0_2.attach(servoDepanKanan_2);
  servo1_0.attach(servoTengahKanan_0);
  servo1_1.attach(servoTengahKanan_1);
  servo1_2.attach(servoTengahKanan_2);
  servo2_0.attach(servoBelakangKanan_0);
  servo2_1.attach(servoBelakangKanan_1);
  servo2_2.attach(servoBelakangKanan_2);
  servo3_0.attach(servoBelakangKiri_0);
  servo3_1.attach(servoBelakangKiri_1);
  servo3_2.attach(servoBelakangKiri_2);
  servo4_0.attach(servoTengahKiri_0);
  servo4_1.attach(servoTengahKiri_1);
  servo4_2.attach(servoTengahKiri_2);
  servo5_0.attach(servoDepanKiri_0);
  servo5_1.attach(servoDepanKiri_1);
  servo5_2.attach(servoDepanKiri_2);
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

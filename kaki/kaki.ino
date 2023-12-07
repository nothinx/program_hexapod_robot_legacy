#include <SPI.h>
#include <Servo.h>
#include <SpeedTrig.h>

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

//#define Zoff 55  //45
#define coxa 77  //30 //waktu dibagi dengan
#define femur 49
#define tibia 103
#define rate 4  // ketelitian langkah  //harus ganjil
#define delayTanggaTama 10
#define lebarLangkah 36   // lebar langkah dibagi rate*2 harus bulat 40 4 32 //harus bulat agar stabil
#define tinggiLangkah 32  //tinggi langkah dibagi rate harus bulat

//int gantiKaki = 0;
//int gantiKakiDorong = 0;
//int gantiKakiBelakang = 0;

#define lebarLangkahPivot 12

//#define endCom() (Serial.end())                               // End Serial Comunication
//#define switchCom(DirPin, Mode) (digitalwrite(DirPin, Mode))  // Switch to TX/RX Mode
//#define pin 6
//#define Tx_MODE 1
//#define Rx_MODE 0

//#define delayJalanTempat 25

int input = 0, inputBefore = 0;
int delayKecepatan = 70;  //80
//int speeds = 400;         //600
//int inputTangga = 0;
//int inputPuing = 0;
//int inputKorban = 0;
//int posKaki;
//int inputSebelum;
void syncLeg();

float posisiAwal = ((coxa + femur) / 1.6);  //1.5
float langkahDatar[20];
float langkahNaik = (tinggiLangkah / rate);
int jumlahPosisiKaki = ((rate * 2) + 1);  //terdapat pada void motion
int rate1 = (rate + 1);
int rate2 = ((rate * 2) + 1);
int rate3 = ((rate * 3) + 1);
int rate4 = ((rate * 4) + 1);
int ubahGerak = 0;
//int prepareZ = 0;
int indexKanan = 0, indexKiri = 0;
//int dorong = 65;

int tinggiKakiKananDepan;
int tinggiKakiKananTengah;
int tinggiKakiKananBelakang;
int tinggiKakiKiriBelakang;
int tinggiKakiKiriTengah;
int tinggiKakiKiriDepan;

int bukaanKakiKananDepan;
int bukaanKakiKananTengah;
int bukaanKakiKananBelakang;
int bukaanKakiKiriBelakang;
int bukaanKakiKiriTengah;
int bukaanKakiKiriDepan;

int putarKakiKananDepan;
int putarKakiKananTengah;
int putarKakiKananBelakang;
int putarKakiKiriBelakang;
int putarKakiKiriTengah;
int putarKakiKiriDepan;

int head;
int angka = 0;
int arah = 0;
int angle[18];

int panjangData;
unsigned char checksum;

struct leg {
  int sudutDalam;
  int sudutTengah;
  int sudutLuar;
  int posisi;
  int motion;   // ada 4 motion
  int gerakan;  // 0 = gerak segitiga     1 = dorong
  float posisiX;
  float posisiY;
  float posisiZ;
} leg[6];

void inisialisasiAwal() {
  for (int a = 0; a < 6; a++) {
    leg[a].posisiX = posisiAwal;
    leg[a].posisiY = posisiAwal;
    leg[a].posisiZ = 0;
    leg[a].posisi = jumlahPosisiKaki;
    leg[a].gerakan = 1;
    inverse(a, leg[a].posisiX, leg[a].posisiY, leg[a].posisiZ);
  }
}

void setKaki();

void setup() {
  SPI.begin(); //untuk memulai komunikasi SPI
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

  setKaki();
  Serial.begin(9600);

  langkahDatar[0] = (lebarLangkah / (rate * 2));

  langkahDatar[1] = (lebarLangkah / (rate * (1.2)));  //ke arah 1 kanan
  langkahDatar[2] = (lebarLangkah / (rate * (1.2)));  //ke arah 1 kiri

  langkahDatar[5] = (lebarLangkah / (rate * (1.8)));  //ke arah 2 kanan
  langkahDatar[6] = (lebarLangkah / (rate * (1.2)));  //ke arah 2 kiri

  langkahDatar[7] = (lebarLangkah / (rate * (1.3)));  //ke arah 3 kanan
  langkahDatar[8] = (lebarLangkah / (rate * (1.5)));  //ke arah 3 kiri

  langkahDatar[3] = (lebarLangkah / (rate * (1.2)));  //ke arah 4 kanan
  langkahDatar[4] = (lebarLangkah / (rate * (1.5)));  //ke arah 4 kiri

  langkahDatar[17] = (lebarLangkahPivot / (rate * (2.8)));  //pivot Api
  inisialisasiAwal();
  syncLeg();
  Serial.println("CLEARDATA"); //untuk membersihkan sheet
  Serial.println("LABEL,Time,Started Time,Date,Potensio,Perubahan");
  Serial.println("RESETTIMER");
}

long rad2deg(float rad) {
  return ((int)(rad * 57.29577957855));
}

int convert(int deg) {
  int zz = map(deg, 0, 300, 0, 1024);  //((int)(deg*(1024/300)));
  return zz;
}

float gerakServo(float nilai) {
  return map(nilai, 0, 180, 700, 1950);
}

int float2int(float input) {
  int temp = (int)input;            // floor-rounded input
  float dif = input - (float)temp;  //
  if (dif < 0.5) return temp;
  else return temp + 1;
}

void syncWrite() {
  servo0_0.write(gerakServo(angle[0]));
  servo0_1.write(gerakServo(angle[1]));
  servo0_2.write(gerakServo(angle[2]));
  // servo1_0.write(gerakServo(angle[3]));
  // servo1_1.write(gerakServo(angle[4]));
  // servo1_2.write(gerakServo(angle[5]));
  // servo2_0.write(gerakServo(angle[6]));
  // servo2_1.write(gerakServo(angle[7]));
  // servo2_2.write(gerakServo(angle[8]));
  // servo3_0.write(gerakServo(angle[9]));
  // servo3_1.write(gerakServo(angle[10]));
  // servo3_2.write(gerakServo(angle[11]));
  // servo4_0.write(gerakServo(angle[12]));
  // servo4_1.write(gerakServo(angle[13]));
  // servo4_2.write(gerakServo(angle[14]));
  // servo5_0.write(gerakServo(angle[15]));
  // servo5_1.write(gerakServo(angle[16]));
  // servo5_2.write(gerakServo(angle[17]));
}

void setServo(int idLeg, int sudut1, int sudut2, int sudut3) {
  if (idLeg == 0) {
    angle[0] = putarKakiKananDepan + sudut1;
    angle[1] = tinggiKakiKananDepan + sudut2;  //240
    angle[2] = bukaanKakiKananDepan + sudut3;  //60
  } else if (idLeg == 1) {
    angle[3] = (90 - sudut1) + putarKakiKananTengah;
    angle[4] = tinggiKakiKananTengah + sudut2;  //240
    angle[5] = bukaanKakiKananTengah + sudut3;  //60
  } else if (idLeg == 2) {
    angle[6] = sudut1 + putarKakiKananBelakang;
    angle[7] = tinggiKakiKananBelakang + sudut2;  //60
    angle[8] = bukaanKakiKananBelakang + sudut3;  //240
  } else if (idLeg == 3) {
    angle[9] = sudut1 + putarKakiKiriBelakang;
    angle[10] = tinggiKakiKiriBelakang + sudut2;  //60
    angle[11] = bukaanKakiKiriBelakang + sudut3;  //240
  } else if (idLeg == 4) {
    angle[12] = sudut1 + putarKakiKiriTengah;
    angle[13] = tinggiKakiKiriTengah + sudut2;  //60
    angle[14] = bukaanKakiKiriTengah + sudut3;  //240
  } else if (idLeg == 5) {
    angle[15] = sudut1 + putarKakiKiriDepan;
    angle[16] = tinggiKakiKiriDepan + sudut2;  //60
    angle[17] = bukaanKakiKiriDepan + sudut3;  //240
  }
}

void syncLeg() {
  setServo(0, leg[0].sudutDalam, leg[0].sudutTengah, leg[0].sudutLuar);
  setServo(1, leg[1].sudutDalam, leg[1].sudutTengah, leg[1].sudutLuar);
  setServo(2, leg[2].sudutDalam, leg[2].sudutTengah, leg[2].sudutLuar);
  setServo(3, leg[3].sudutDalam, leg[3].sudutTengah, leg[3].sudutLuar);
  setServo(4, leg[4].sudutDalam, leg[4].sudutTengah, leg[4].sudutLuar);
  setServo(5, leg[5].sudutDalam, leg[5].sudutTengah, leg[5].sudutLuar);
  syncWrite();
}

void inverse(int idLeg, float x, float y, float z) {
  float degree1, degree2, degree3;
  float degree2_1, degree2_2;
  float L1, L, L_2, A, Lcox;
  float temp2, temp2_2, temp2_3, temp2_4;
  float temp3, temp3_2, temp3_3, temp3_4;
  float femur_2, tibia_2;
  int sudut1, sudut2, sudut3;

  // x = x + 10;
  // y = y + 50;
  z = 40 - z;
  degree1 = SpeedTrig.atan2(x, y);
  L1 = sqrt((x * x) + (y * y));  //akar x^2 + y^2

  Lcox = L1 - coxa;                   //coxa adalah 10
  L = sqrt((z * z) + (Lcox * Lcox));  //akar z^2 + Lcox^2
  femur_2 = femur * femur;
  tibia_2 = tibia * tibia;
  L_2 = L * L;

  degree2_1 = SpeedTrig.atan2(z, Lcox);
  temp2 = ((L_2 + femur_2 - tibia_2) / (2 * femur * L));
  degree2_2 = SpeedTrig.acos(temp2);
  degree2_1 = degree2_1 * -1;
  degree2 = degree2_2 + degree2_1;

  temp3 = ((femur_2 + tibia_2 - L_2) / (2 * femur * tibia));
  degree3 = SpeedTrig.acos(temp3);

  sudut1 = float2int(rad2deg(degree1));
  sudut2 = float2int(rad2deg(degree2));
  sudut3 = float2int(rad2deg(degree3));

  sudut2 = 90 - sudut2;
  sudut3 = 90 - sudut3;

  if (sudut1 < 0) {
    // sudut3 = 180 + sudut3;
    sudut1 = -1 * sudut1;
  }
  if (sudut3 < 0) {
    // sudut3 = 180 + sudut3;
    sudut3 = -1 * sudut3;
  }
  if (sudut2 < 0) {
    // sudut2 = 180 + sudut2;
    sudut2 = -1 * sudut2;
  }
  // Serial.println(sudut3);
  // sudut3 = 180 - sudut3;

  leg[idLeg].sudutDalam = sudut1;
  leg[idLeg].sudutTengah = sudut2;
  leg[idLeg].sudutLuar = sudut3;
}

void motion(int idLeg, int indexLebar)  //0,1
{
  // Serial.println(idLeg);
  if (leg[idLeg].motion == 0)  // ke Y positif
  {
    if (leg[idLeg].gerakan == 0)  //gerak segitiga
    {
      if (leg[idLeg].posisi < rate1)  //(rate+1))
      {
        leg[idLeg].posisiY += langkahDatar[indexLebar];
        leg[idLeg].posisiZ += langkahNaik;
        leg[idLeg].posisi++;
      } else if (leg[idLeg].posisi >= rate1 && leg[idLeg].posisi < rate2)  //leg[idLeg].posisi>=(rate+1) && leg[idLeg].posisi<((rate*2)+1)
      {
        leg[idLeg].posisiY += langkahDatar[indexLebar];
        leg[idLeg].posisiZ -= langkahNaik;
        leg[idLeg].posisi++;
      }
      if (leg[idLeg].posisi == rate2) {
        leg[idLeg].gerakan = 1;
      }

    } else if (leg[idLeg].gerakan == 1)  //dorong
    {
      if (leg[idLeg].posisi > 1) {
        leg[idLeg].posisiY -= langkahDatar[indexLebar];
        leg[idLeg].posisi--;
      }
      if (leg[idLeg].posisi == 1) {
        leg[idLeg].gerakan = 0;
      }
    }
  }

  else if (leg[idLeg].motion == 1)  // ke Y negatif
  {
    if (leg[idLeg].gerakan == 0)  //gerak segitiga
    {
      if (leg[idLeg].posisi > rate1)  //(rate+1))
      {
        leg[idLeg].posisiY -= langkahDatar[indexLebar];
        leg[idLeg].posisiZ += langkahNaik;
        leg[idLeg].posisi--;
      } else if (leg[idLeg].posisi <= rate1)  //(rate+1))
      {
        leg[idLeg].posisiY -= langkahDatar[indexLebar];
        leg[idLeg].posisiZ -= langkahNaik;
        leg[idLeg].posisi--;
      }
      if (leg[idLeg].posisi == 1) {
        leg[idLeg].gerakan = 1;
      }

    } else if (leg[idLeg].gerakan == 1)  //dorong
    {
      if (leg[idLeg].posisi < rate2)  //((rate*2)+1))
      {
        leg[idLeg].posisiY += langkahDatar[indexLebar];
        leg[idLeg].posisi++;
      }
      if (leg[idLeg].posisi == rate2)  //((rate*2)+1))
      {
        leg[idLeg].gerakan = 0;
      }
    }
  }

  else if (leg[idLeg].motion == 2)  // ke X negatif
  {
    if (leg[idLeg].gerakan == 0)  //gerak segitiga
    {
      if (leg[idLeg].posisi < rate3)  //((rate*3)+1))
      {
        leg[idLeg].posisiX -= langkahDatar[indexLebar];
        leg[idLeg].posisiZ += langkahNaik;
        leg[idLeg].posisi++;
      } else if (leg[idLeg].posisi >= rate3)  //((rate*3)+1))
      {
        leg[idLeg].posisiX -= langkahDatar[indexLebar];
        leg[idLeg].posisiZ -= langkahNaik;
        leg[idLeg].posisi++;
      }
      if (leg[idLeg].posisi == rate4)  //((rate*4)+1))
      {
        leg[idLeg].gerakan = 1;
      }
    } else if (leg[idLeg].gerakan == 1)  //dorong
    {
      if (leg[idLeg].posisi > rate2)  //((rate*2)+1))
      {
        leg[idLeg].posisiX -= langkahDatar[indexLebar];
        leg[idLeg].posisi--;
      }
      if (leg[idLeg].posisi == rate2)  //((rate*2)+1))
      {
        leg[idLeg].gerakan = 0;
      }
    }
  }

  else if (leg[idLeg].motion == 3)  // ke X positif
  {
    if (leg[idLeg].gerakan == 0)  //gerak segitiga
    {
      if (leg[idLeg].posisi > rate3)  //((rate*3)+1))
      {
        leg[idLeg].posisiX += langkahDatar[indexLebar];
        leg[idLeg].posisiZ += langkahNaik;
        leg[idLeg].posisi--;
      } else if (leg[idLeg].posisi <= rate3)  //((rate*3)+1))
      {
        leg[idLeg].posisiX += langkahDatar[indexLebar];
        leg[idLeg].posisiZ -= langkahNaik;
        leg[idLeg].posisi--;
      }
      if (leg[idLeg].posisi == rate2)  //((rate*2)+1))
      {
        leg[idLeg].gerakan = 1;
      }

    } else if (leg[idLeg].gerakan == 1)  //dorong
    {
      if (leg[idLeg].posisi < rate4)  //((rate*4)+1))
      {
        leg[idLeg].posisiX -= langkahDatar[indexLebar];
        leg[idLeg].posisi++;
      }
      if (leg[idLeg].posisi == rate4)  //((rate*4)+1))
      {
        leg[idLeg].gerakan = 0;
      }
    }
  }

  if (leg[idLeg].motion == 4)  // ke Y positif
  {
    if (leg[idLeg].gerakan == 0)  //gerak segitiga
    {
      if (leg[idLeg].posisi < rate1)  //(rate+1))
      {
        leg[idLeg].posisiX -= langkahDatar[indexLebar];
        leg[idLeg].posisiZ += langkahNaik;
        leg[idLeg].posisi++;
      } else if (leg[idLeg].posisi >= rate1 && leg[idLeg].posisi < rate2)  //leg[idLeg].posisi>=(rate+1) && leg[idLeg].posisi<((rate*2)+1)
      {
        leg[idLeg].posisiX -= langkahDatar[indexLebar];
        leg[idLeg].posisiZ -= langkahNaik;
        leg[idLeg].posisi++;
      }
      if (leg[idLeg].posisi == rate2) {
        leg[idLeg].gerakan = 1;
      }

    } else if (leg[idLeg].gerakan == 1)  //dorong
    {
      if (leg[idLeg].posisi > 1) {
        leg[idLeg].posisiY -= langkahDatar[indexLebar];
        leg[idLeg].posisi--;
      }
      if (leg[idLeg].posisi == 1) {
        leg[idLeg].gerakan = 0;
      }
    }
  }
  inverse(idLeg, leg[idLeg].posisiX, leg[idLeg].posisiY, leg[idLeg].posisiZ);
  delay(4);
}

void tuning() {
  setServo(0, 45, 90, 90);
  setServo(1, 45, 90, 90);
  setServo(2, 45, 90, 90);
  setServo(3, 45, 90, 90);
  syncWrite();
}

void directions(int kananD, int kananT, int kananB, int kiriB, int kiriT, int kiriD) {
  if (inputBefore == 1) {
    indexKanan = 1;
    indexKiri = 2;
  } else if (inputBefore == 2) {
    indexKanan = 3;
    indexKiri = 4;
  } else if (inputBefore == 3) {
    indexKanan = 5;
    indexKiri = 6;
  } else if (inputBefore == 4) {
    indexKanan = 7;
    indexKiri = 8;
  }

  if (ubahGerak == 0) {
    // setKaki();
    // syncLeg();

    while (leg[kananD].posisi != 1) {
      leg[kananD].motion = 1;
      motion(kananD, indexKanan);
      syncLeg();
    }
    while (leg[kananB].posisi != rate4)  //((rate*4)+1))
    {
      leg[kananB].motion = 2;
      motion(kananB, indexKanan);
      syncLeg();
    }
    ubahGerak = 1;
  }
  leg[kananD].motion = 0;
  leg[kananT].motion = 3;
  leg[kananB].motion = 0;
  leg[kiriB].motion = 3;
  leg[kiriT].motion = 4;
  leg[kiriD].motion = 3;
  motion(kananD, indexKanan);
  motion(kananT, indexKanan);
  motion(kananB, indexKanan);
  motion(kiriB, indexKiri);
  motion(kiriT, indexKiri);
  motion(kiriD, indexKiri);
  syncLeg();
}

void pivot(int arah) {
  if (arah == 0)  //putar kiri
  {
    if (ubahGerak == 0) {
      while (leg[0].posisi != rate4 && leg[2].posisi != rate4) {
        leg[0].motion = 2;
        motion(0, 0);
        leg[2].motion = 2;
        motion(2, 0);
        syncLeg();
      }
      ubahGerak = 1;
    }
    leg[0].motion = 2;
    leg[1].motion = 2;
    leg[2].motion = 2;
    leg[3].motion = 2;
    leg[4].motion = 2;
    leg[5].motion = 2;
    motion(0, 0);
    motion(1, 0);
    motion(2, 0);
    motion(3, 0);
    motion(4, 0);
    motion(5, 0);
    syncLeg();
  } else if (arah == 1)  //putar kanan
  {
    if (ubahGerak == 0) {
      while (leg[1].posisi != 1 && leg[3].posisi != 1) {
        leg[1].motion = 1;
        motion(1, 0);
        leg[3].motion = 1;
        motion(3, 0);
        syncLeg();
      }
      ubahGerak = 1;
    }
    leg[0].motion = 1;
    leg[1].motion = 1;
    leg[2].motion = 1;
    leg[3].motion = 1;
    leg[4].motion = 1;
    leg[5].motion = 1;
    motion(0, 0);
    motion(1, 0);
    motion(2, 0);
    motion(3, 0);
    motion(4, 0);
    motion(5, 0);
    syncLeg();
  }
}

void geser(int arah) {
  if (arah == 0)  //putar kiri
  {
    if (ubahGerak == 0) {
      while (leg[0].posisi != rate4 && leg[2].posisi != rate4) {
        leg[0].motion = 2;
        motion(0, 0);
        leg[2].motion = 2;
        motion(2, 0);
        syncLeg();
      }
      ubahGerak = 1;
    }
    leg[0].motion = 2;
    leg[1].motion = 2;
    leg[2].motion = 2;
    leg[3].motion = 2;
    motion(0, 0);
    motion(1, 0);
    motion(2, 0);
    motion(3, 0);
    syncLeg();
  } else if (arah == 1)  //putar kanan
  {
    if (ubahGerak == 0) {
      while (leg[1].posisi != 1 && leg[3].posisi != 1) {
        leg[1].motion = 1;
        motion(1, 0);
        leg[3].motion = 1;
        motion(3, 0);
        syncLeg();
      }
      ubahGerak = 1;
    }
    leg[0].motion = 1;
    leg[1].motion = 1;
    leg[2].motion = 1;
    leg[3].motion = 1;
    motion(0, 0);
    motion(1, 0);
    motion(2, 0);
    motion(3, 0);
    syncLeg();
  }
}

void tuningTangga() {
  delay(1000);
  setServo(0, 45, 100, 90);
  setServo(1, 45, 80, 110);
  setServo(2, 45, 80, 110);
  setServo(3, 45, 100, 90);
  syncWrite();
  delay(1000);
  int hitungTangga = 0;
  while (hitungTangga < 5) {
    setServo(0, 45, 120, 90);
    syncWrite();
    delay(delayTanggaTama);
    setServo(0, 15, 120, 90);
    syncWrite();
    delay(delayTanggaTama);
    setServo(0, 15, 100, 90);
    syncWrite();
    delay(delayTanggaTama);

    setServo(2, 45, 110, 100);
    syncWrite();
    delay(delayTanggaTama);
    setServo(2, 90, 110, 100);
    syncWrite();
    delay(delayTanggaTama);
    setServo(2, 90, 60, 100);
    syncWrite();
    delay(delayTanggaTama);

    setServo(0, 45, 100, 90);
    setServo(1, 45, 80, 110);
    setServo(2, 45, 80, 110);
    setServo(3, 45, 100, 90);
    syncWrite();
    delay(delayTanggaTama);

    setServo(3, 45, 120, 90);
    syncWrite();
    delay(delayTanggaTama);
    setServo(3, 75, 120, 90);
    syncWrite();
    delay(delayTanggaTama);
    setServo(3, 75, 100, 90);
    syncWrite();
    delay(delayTanggaTama);

    setServo(1, 45, 110, 100);
    syncWrite();
    delay(delayTanggaTama);
    setServo(1, 0, 110, 100);
    syncWrite();
    delay(delayTanggaTama);
    setServo(1, 0, 60, 110);
    syncWrite();
    delay(delayTanggaTama);

    setServo(0, 45, 100, 90);
    setServo(1, 45, 80, 110);
    setServo(2, 45, 80, 110);
    setServo(3, 45, 100, 90);
    syncWrite();
    delay(delayTanggaTama);

    hitungTangga++;
  }
}

void setKaki() {
  putarKakiKananDepan = 2;     //kurang ke kepan
  putarKakiKananTengah = 2;    // tambah ke depan
  putarKakiKananBelakang = 2;  //dikurang semakin kedepan
  putarKakiKiriBelakang = 2;
  putarKakiKiriTengah = 2;  // tambah ke depan
  putarKakiKiriDepan = 2;   // tambah ke depan

  tinggiKakiKananDepan = 0;  //tambah jadi naik
  tinggiKakiKananTengah = 0;
  tinggiKakiKananBelakang = 0;
  tinggiKakiKiriBelakang = 0;
  tinggiKakiKiriTengah = 0;
  tinggiKakiKiriDepan = 0;

  bukaanKakiKananDepan = 0;  //tambah jadi buka
  bukaanKakiKananTengah = 0;
  bukaanKakiKananBelakang = 0;
  bukaanKakiKiriBelakang = 0;  //tambah jadi tutup
  bukaanKakiKiriTengah = 0;    //tambah jadi tutup
  bukaanKakiKiriDepan = 0;
}

void cekPerintah() {
  if (leg[0].posisi == rate2 || leg[1].posisi == rate2 || leg[2].posisi == rate2 || leg[3].posisi == rate2 || leg[4].posisi == rate2 || leg[5].posisi == rate2) {
    if (input != inputBefore)  //&&input<10&&inputBefore<10)posisi
    {
      ubahGerak = 0;
      inisialisasiAwal();
      inputBefore = input;
    }
  }
  input = 1;

  if (inputBefore == 0) {
    setKaki();
    syncLeg();
  } else if (inputBefore == 1)  //maju
  {
    setKaki();
    directions(0, 1, 2, 3, 4, 5);
  } else if (inputBefore == 2)  //kiri
  {
    setKaki();
    geser(1);
  } else if (inputBefore == 3)  //mundur
  {
    setKaki();
    directions(3, 4, 5, 0, 1, 2);
  } else if (inputBefore == 4)  //kanan
  {
    setKaki();
    geser(0);
  } else if (inputBefore == 5)  // pivot KIRI
  {
    setKaki();
    pivot(1);
  } else if (inputBefore == 6)  // pivot KANAN
  {
    setKaki();
    pivot(0);
  } else {
    syncLeg();
  }
  delay(delayKecepatan);
}

void pasangKaki() {
  servo0_0.write(gerakServo(0));
  servo0_1.write(gerakServo(0));
  servo0_2.write(gerakServo(90 + 10));

  servo1_0.write(gerakServo(45));
  servo1_1.write(gerakServo(0));
  servo1_2.write(gerakServo(90));

  servo2_0.write(gerakServo(90));
  servo2_1.write(gerakServo(0));
  servo2_2.write(gerakServo(90));

  servo3_0.write(gerakServo(0));
  servo3_1.write(gerakServo(0));
  servo3_2.write(gerakServo(90));

  servo4_0.write(gerakServo(45 - 5));
  servo4_1.write(gerakServo(0));
  servo4_2.write(gerakServo(90));

  servo5_0.write(gerakServo(100 + 3));
  servo5_1.write(gerakServo(20));  //0
  servo5_2.write(gerakServo(90));
}

void diam() {
  int ata = 45;
  int ten = 45;
  int baw = 100;
  // servo0_0.write(gerakServo(ata));
  // servo0_1.write(gerakServo(ten));
  // servo0_2.write(gerakServo(baw + 10));
  servo1_0.write(gerakServo(ata));
  servo1_1.write(gerakServo(ten));
  servo1_2.write(gerakServo(baw));
  servo2_0.write(gerakServo(ata));
  servo2_1.write(gerakServo(ten));
  servo2_2.write(gerakServo(baw));
  servo3_0.write(gerakServo(ata));
  servo3_1.write(gerakServo(ten));
  servo3_2.write(gerakServo(baw));
  servo4_0.write(gerakServo(ata - 5));
  servo4_1.write(gerakServo(ten));
  servo4_2.write(gerakServo(baw));
  servo5_0.write(gerakServo(ata + 3));
  servo5_1.write(gerakServo(ten));
  servo5_2.write(gerakServo(baw));
}

void loop() {
  // inverse(3,,5,4,5);
  // pasangKaki();
  diam();
  cekPerintah();
  // Serial.print("DATA, TIME, TIMER, DATE,");
  // Serial.println("----------------------------------");
  // Serial.println("posisi :");
  // Serial.println(x);
  // Serial.println(y);
  // Serial.println(z);
  // Serial.println("sudut :");
  Serial.println(leg[2].sudutDalam);
  Serial.println(leg[2].sudutTengah);
  Serial.println(leg[2].sudutLuar);
  // Serial.println("----------------------------------");

  // delay(800);
  // Serial.println("halo");
  // diam();
  // for (int i = 30; i <= 120; i++) {
  //   inverse(2, i, 200, 200);
  //   setServo(2, leg[2].sudutDalam, leg[2].sudutTengah, leg[2].sudutLuar);
  //   syncWrite();
  //   delay(30);
  // }
}

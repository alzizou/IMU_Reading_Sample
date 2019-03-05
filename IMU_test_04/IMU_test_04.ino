#include <I2Cdev.h>
#include <HMC5883L.h>
#include <ITG3200.h>
#include <ADXL345.h>
#include <Wire.h> 

float NUM = 0;

ADXL345 acc;
int acc_x, acc_y, acc_z;
double acc_xyz[3];
double acc_xg, acc_yg, acc_zg; // in g
float acc_xg_SUM=0;
float acc_yg_SUM=0;
float acc_zg_SUM=0;
float acc_xg_AVG=0;
float acc_yg_AVG=0;
float acc_zg_AVG=0;

ITG3200 gyr;
double gyr_temp; // in C
int16_t gyr_x,gyr_y,gyr_z;
float gyr_dx,gyr_dy,gyr_dz; // in deg/sec
float gyr_dx_SUM=0;
float gyr_dy_SUM=0;
float gyr_dz_SUM=0;
float gyr_dx_AVG=0;
float gyr_dy_AVG=0;
float gyr_dz_AVG=0;

HMC5883L mag;
int16_t mx, my, mz;
float heading_xy, heading_xz;
float heading_xy_SUM=0;
float heading_xz_SUM=0;
float heading_xy_AVG=0;
float heading_xz_AVG=0;

void setup(){

  Serial.begin(115200);
  Wire.begin();
  
  acc.powerOn();

  gyr.init();
  gyr.zeroCalibrate(200,10);

  mag.initialize();
  
  delay(5);  
  
}

void loop(){
  NUM = NUM + 1;

// Acceleration readings  
  acc.readXYZ(&acc_x, &acc_y, &acc_z);
  acc.getAcceleration(acc_xyz);
  acc_xg = acc_xyz[0];
  acc_yg = acc_xyz[1];
  acc_zg = acc_xyz[2];
  acc_xg_SUM = acc_xg_SUM + acc_xg;
  acc_yg_SUM = acc_yg_SUM + acc_yg;
  acc_zg_SUM = acc_zg_SUM + acc_zg;
  acc_xg_AVG = acc_xg_SUM / NUM;
  acc_yg_AVG = acc_yg_SUM / NUM;
  acc_zg_AVG = acc_zg_SUM / NUM;;
  Serial.print("acc_x = ");
  Serial.println(acc_x);
  Serial.print("acc_y = ");
  Serial.println(acc_y);
  Serial.print("acc_z = ");
  Serial.println(acc_z);
  Serial.print("acc_xg = ");
  Serial.print(acc_xg);
  Serial.println("g");
  Serial.print("acc_yg = ");
  Serial.print(acc_yg);
  Serial.println("g");
  Serial.print("acc_zg = ");
  Serial.print(acc_zg);
  Serial.println("g");
  Serial.print("acc_xg_AVG = ");
  Serial.print(acc_xg_AVG);
  Serial.println("g");
  Serial.print("acc_yg_AVG = ");
  Serial.print(acc_yg_AVG);
  Serial.println("g");
  Serial.print("acc_zg_AVG = ");
  Serial.print(acc_zg_AVG);
  Serial.println("g");

// Gyro readings
  gyr_temp = gyr.getTemperature();
  gyr.getXYZ(&gyr_x,&gyr_y,&gyr_z);
  gyr.getAngularVelocity(&gyr_dx,&gyr_dy,&gyr_dz);
  gyr_dx_SUM = gyr_dx_SUM + gyr_dx;
  gyr_dy_SUM = gyr_dy_SUM + gyr_dy;
  gyr_dz_SUM = gyr_dz_SUM + gyr_dz;
  gyr_dx_AVG = gyr_dx_SUM / NUM;
  gyr_dy_AVG = gyr_dy_SUM / NUM;
  gyr_dz_AVG = gyr_dz_SUM / NUM;  
  Serial.print("Temperature = ");
  Serial.print(gyr_temp);
  Serial.println(" C ");
  Serial.print("gyr_x = ");
  Serial.println(gyr_x);
  Serial.print("gyr_y = ");
  Serial.println(gyr_y);
  Serial.print("gyr_z = ");
  Serial.println(gyr_z);
  Serial.print("gyr_dx = ");
  Serial.print(gyr_dx);
  Serial.println(" deg/sec ");
  Serial.print("gyr_dy = ");
  Serial.print(gyr_dy);
  Serial.println(" deg/sec ");
  Serial.print("gyr_dz = ");
  Serial.print(gyr_dz);
  Serial.println(" deg/sec ");
  Serial.print("gyr_dx_AVG = ");
  Serial.print(gyr_dx_AVG);
  Serial.println(" deg/sec ");
  Serial.print("gyr_dy_AVG = ");
  Serial.print(gyr_dy_AVG);
  Serial.println(" deg/sec ");
  Serial.print("gyr_dz_AVG = ");
  Serial.print(gyr_dz_AVG);
  Serial.println(" deg/sec ");

// Magnetometer Readings
  mag.getHeading(&mx, &my, &mz);
  Serial.print("mag_x = ");
  Serial.println(mx); 
  Serial.print("mag_y = ");
  Serial.println(my); 
  Serial.print("mag_z = ");
  Serial.println(mz); 
  heading_xy = atan2(my, mx);
  heading_xz = atan2(mz, mx);
  if(heading_xy < 0)
    heading_xy += 2 * M_PI;
  if(heading_xz < 0)
    heading_xz += 2* M_PI;

  heading_xy_SUM = heading_xy_SUM + heading_xy;
  heading_xz_SUM = heading_xz_SUM + heading_xz;
  heading_xy_AVG = heading_xy_SUM / NUM;
  heading_xz_AVG = heading_xz_SUM / NUM;
  Serial.print("heading_xy = ");
  Serial.println(heading_xy * 180/M_PI);
  Serial.print("heading_xz = ");
  Serial.println(heading_xz * 180/M_PI);
  Serial.print("heading_xy_AVG = ");
  Serial.println(heading_xy_AVG * 180/M_PI);
  Serial.print("heading_xz_AVG = ");
  Serial.println(heading_xz_AVG * 180/M_PI);
    
  //delay(100);
}


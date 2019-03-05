#include <I2Cdev.h>
#include <HMC5883L.h>
#include <ITG3200.h>
#include <ADXL345.h>
#include <Wire.h> 

unsigned long time1 = 0;
unsigned long time2 = 0;
float deltat = 0.1;

int NUM = 0;
int NUM_AVG = 1000;
const int Filter_Order = 4;
const float g = 9.81;
//const float deltat = 0.001f;

ADXL345 acc;
double acc_xyz[3];
float acc_x, acc_y, acc_z; // normalaized
float acc_xg, acc_yg, acc_zg; // m/s^2
int acc_xN, acc_yN, acc_zN; 
float acc_x_Filt, acc_y_Filt, acc_z_Filt; 
float acc_x_Filt_OK, acc_y_Filt_OK, acc_z_Filt_OK; 
float acc_x_SUM=0, acc_y_SUM=0, acc_z_SUM=0;
float acc_x_AVG=0, acc_y_AVG=0, acc_z_AVG=0;
float acc_x_HISTORY[5] = {0,0,0,0,0};
float acc_y_HISTORY[5] = {0,0,0,0,0};
float acc_z_HISTORY[5] = {0,0,0,0,0};
float acc_x_Filt_HISTORY[5] = {0,0,0,0,0};
float acc_y_Filt_HISTORY[5] = {0,0,0,0,0};
float acc_z_Filt_HISTORY[5] = {0,0,0,0,0};

ITG3200 gyr;
float gyr_dx, gyr_dy, gyr_dz; // in deg/sec
float gyr_x, gyr_y, gyr_z; // in rad/sec
int16_t gyr_xN, gyr_yN, gyr_zN; 
float gyr_x_Filt, gyr_y_Filt, gyr_z_Filt;
float gyr_x_Filt_OK, gyr_y_Filt_OK, gyr_z_Filt_OK;
float gyr_x_SUM=0, gyr_y_SUM=0, gyr_z_SUM=0;
float gyr_x_AVG=0, gyr_y_AVG=0, gyr_z_AVG=0;
float gyr_x_HISTORY[5] = {0,0,0,0,0};
float gyr_y_HISTORY[5] = {0,0,0,0,0};
float gyr_z_HISTORY[5] = {0,0,0,0,0};
float gyr_x_Filt_HISTORY[5] = {0,0,0,0,0};
float gyr_y_Filt_HISTORY[5] = {0,0,0,0,0};
float gyr_z_Filt_HISTORY[5] = {0,0,0,0,0};

HMC5883L mag;
int16_t mx, my, mz;
float mx_Filt, my_Filt, mz_Filt;
float mx_Filt_OK, my_Filt_OK, mz_Filt_OK;
float mx_SUM=0, my_SUM=0, mz_SUM=0;
float mx_AVG=0, my_AVG=0, mz_AVG=0;
float mx_HISTORY[5] = {0,0,0,0,0};
float my_HISTORY[5] = {0,0,0,0,0};
float mz_HISTORY[5] = {0,0,0,0,0};
float mx_Filt_HISTORY[5] = {0,0,0,0,0};
float my_Filt_HISTORY[5] = {0,0,0,0,0};
float mz_Filt_HISTORY[5] = {0,0,0,0,0};

// constants for Euler angles computation
float norm = 0;

float SEq_1 = 1.0f, SEq_2 = 0, SEq_3=0, SEq_4=0;
float b_x = 1.0f, b_z = 0;
float w_bx = 0, w_by = 0, w_bz = 0;

const float gyroMeasError = 3.1415926535 * (5.0f / 180.0f);
const float gyroMeasDrift = 3.1415926535 * (0.2f / 180.0f);
float beta = sqrt(3.0f/4.0f) * gyroMeasError;
float zeta = sqrt(3.0f/4.0f) * gyroMeasDrift;

float f_1, f_2, f_3, f_4, f_5, f_6;
float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, J_41, J_42, J_43, J_44,
  J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64;
float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;
float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4;
float w_err_x,w_err_y, w_err_z;
float h_x, h_y, h_z;
float a_x_Madg, a_y_Madg, a_z_Madg;
float w_x_Madg, w_y_Madg, w_z_Madg;
float m_x_Madg, m_y_Madg, m_z_Madg;

float halfSEq_1, halfSEq_2, halfSEq_3, halfSEq_4;
float twoSEq_1, twoSEq_2, twoSEq_3, twoSEq_4;
float twob_x, twob_z;
float twob_xSEq_1, twob_xSEq_2, twob_xSEq_3, twob_xSEq_4;
float twob_zSEq_1, twob_zSEq_2, twob_zSEq_3, twob_zSEq_4;
float SEq_1SEq_2, SEq_1SEq_3, SEq_1SEq_4, SEq_3SEq_4, SEq_2SEq_3, SEq_2SEq_4;
float twom_x, twom_y, twom_z;

float Phi, Theta, Psi;

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

//Generating the sampling time
  time2 = millis();
  deltat = (time2-time1)*1e-3;
  time1 = time2; 
  Serial.println(deltat,4);

  NUM = NUM + 1;    
  Serial.println(NUM); 
  
//Averaging
  if (NUM < NUM_AVG) {          
  // Acceleration readings 
    acc.readXYZ(&acc_xN, &acc_yN, &acc_zN);     
    acc.getAcceleration(acc_xyz);
    acc_x = acc_xyz[0];
    acc_y = acc_xyz[1];
    acc_z = acc_xyz[2];
    acc_x_SUM = acc_x_SUM + acc_x;
    acc_y_SUM = acc_y_SUM + acc_y;
    acc_z_SUM = acc_z_SUM + acc_z;
    acc_x_AVG = acc_x_SUM / (float)NUM;
    acc_y_AVG = acc_y_SUM / (float)NUM;
    acc_z_AVG = acc_z_SUM / (float)NUM;
    // Gyro readings    
    gyr.getXYZ(&gyr_xN,&gyr_yN,&gyr_zN);
    gyr.getAngularVelocity(&gyr_dx,&gyr_dy,&gyr_dz);
    gyr_x = gyr_dx * 3.1416 / 180.0f;
    gyr_y = gyr_dy * 3.1416 / 180.0f;
    gyr_z = gyr_dz * 3.1416 / 180.0f;
    gyr_x_SUM = gyr_x_SUM + gyr_x;
    gyr_y_SUM = gyr_y_SUM + gyr_y;
    gyr_z_SUM = gyr_z_SUM + gyr_z;
    gyr_x_AVG = gyr_x_SUM / (float)NUM;
    gyr_y_AVG = gyr_y_SUM / (float)NUM;
    gyr_z_AVG = gyr_z_SUM / (float)NUM;  
    // Magnetometer Readings
    mag.getHeading(&mx, &my, &mz);  
    mx_SUM = mx_SUM + mx;
    my_SUM = my_SUM + my;
    mz_SUM = mz_SUM + mz;
    mx_AVG = mx_SUM / (float)NUM;
    my_AVG = my_SUM / (float)NUM;
    mz_AVG = mz_SUM / (float)NUM;   
    //delay(100); 
       
  }else {    

    acc.readXYZ(&acc_xN, &acc_yN, &acc_zN);
    acc.getAcceleration(acc_xyz);
    acc_x = acc_xyz[0];
    acc_y = acc_xyz[1];
    acc_z = acc_xyz[2];
    gyr.getXYZ(&gyr_xN,&gyr_yN,&gyr_zN);
    gyr.getAngularVelocity(&gyr_dx,&gyr_dy,&gyr_dz);
    gyr_x = gyr_dx * 3.1416 / 180.0f;
    gyr_y = gyr_dy * 3.1416 / 180.0f;
    gyr_z = gyr_dz * 3.1416 / 180.0f;
    mag.getHeading(&mx, &my, &mz);  
    Serial.print("acc_x = ");
    Serial.println(acc_x,4);
    Serial.print("acc_y = ");
    Serial.println(acc_y,4);
    Serial.print("acc_z = ");
    Serial.println(acc_z,4);
    Serial.print("gyr_x = ");
    Serial.println(gyr_x,4);
    Serial.print("gyr_y = ");
    Serial.println(gyr_y,4);
    Serial.print("gyr_z = ");
    Serial.println(gyr_z,4);
    Serial.print("mag_x = ");
    Serial.println(mx);
    Serial.print("mag_y = ");
    Serial.println(my);
    Serial.print("mag_z = ");
    Serial.println(mz);



    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // Filtering
    
    acc_x_Filt = LowPass_Filter(acc_x,acc_x_HISTORY,acc_x_Filt_HISTORY);
    acc_y_Filt = LowPass_Filter(acc_y,acc_y_HISTORY,acc_y_Filt_HISTORY);
    acc_z_Filt = LowPass_Filter(acc_z,acc_z_HISTORY,acc_z_Filt_HISTORY);
    gyr_x_Filt = LowPass_Filter(gyr_x,gyr_x_HISTORY,gyr_x_Filt_HISTORY);
    gyr_y_Filt = LowPass_Filter(gyr_y,gyr_y_HISTORY,gyr_y_Filt_HISTORY);
    gyr_z_Filt = LowPass_Filter(gyr_z,gyr_z_HISTORY,gyr_z_Filt_HISTORY);
    mx_Filt = LowPass_Filter((float)mx,mx_HISTORY,mx_Filt_HISTORY);
    my_Filt = LowPass_Filter((float)my,my_HISTORY,my_Filt_HISTORY);
    mz_Filt = LowPass_Filter((float)mz,mz_HISTORY,mz_Filt_HISTORY);
    Serial.print("acc_x_Filt = ");
    Serial.println(acc_x_Filt,4);
    Serial.print("acc_y_Filt = ");
    Serial.println(acc_y_Filt,4);
    Serial.print("acc_z_Filt = ");
    Serial.println(acc_z_Filt,4);
    Serial.print("gyr_x_Filt = ");
    Serial.println(gyr_x_Filt,4);
    Serial.print("gyr_y_Filt = ");
    Serial.println(gyr_y_Filt,4);
    Serial.print("gyr_z_Filt = ");
    Serial.println(gyr_z_Filt,4);
    Serial.print("mag_x_Filt = ");
    Serial.println(mx_Filt,4);
    Serial.print("mag_y_Filt = ");
    Serial.println(my_Filt,4);
    Serial.print("mag_z_Filt = ");
    Serial.println(mz_Filt,4);

    // Generating History
    for(int i=(Filter_Order-1);i>0;i--){
      acc_x_HISTORY[i] = acc_x_HISTORY[i-1];
      acc_y_HISTORY[i] = acc_y_HISTORY[i-1];
      acc_z_HISTORY[i] = acc_z_HISTORY[i-1];
      acc_x_Filt_HISTORY[i] = acc_x_Filt_HISTORY[i-1];
      acc_y_Filt_HISTORY[i] = acc_y_Filt_HISTORY[i-1];
      acc_z_Filt_HISTORY[i] = acc_z_Filt_HISTORY[i-1];
      //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      gyr_x_HISTORY[i] = gyr_x_HISTORY[i-1];
      gyr_y_HISTORY[i] = gyr_y_HISTORY[i-1];
      gyr_z_HISTORY[i] = gyr_z_HISTORY[i-1];
      gyr_x_Filt_HISTORY[i] = gyr_x_Filt_HISTORY[i-1];
      gyr_y_Filt_HISTORY[i] = gyr_y_Filt_HISTORY[i-1];
      gyr_z_Filt_HISTORY[i] = gyr_z_Filt_HISTORY[i-1];
      //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      mx_HISTORY[i] = mx_HISTORY[i-1];
      my_HISTORY[i] = my_HISTORY[i-1];
      mz_HISTORY[i] = mz_HISTORY[i-1];
      mx_Filt_HISTORY[i] = mx_Filt_HISTORY[i-1];
      my_Filt_HISTORY[i] = my_Filt_HISTORY[i-1];
      mz_Filt_HISTORY[i] = mz_Filt_HISTORY[i-1];
    }
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    acc_x_HISTORY[0] = acc_x;
    acc_y_HISTORY[0] = acc_y;
    acc_z_HISTORY[0] = acc_z;
    acc_x_Filt_HISTORY[0] = acc_x_Filt;
    acc_y_Filt_HISTORY[0] = acc_y_Filt;
    acc_z_Filt_HISTORY[0] = acc_z_Filt;
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    gyr_x_HISTORY[0] = gyr_x;
    gyr_y_HISTORY[0] = gyr_y;
    gyr_z_HISTORY[0] = gyr_z;
    gyr_x_Filt_HISTORY[0] = gyr_x_Filt;
    gyr_y_Filt_HISTORY[0] = gyr_y_Filt;
    gyr_z_Filt_HISTORY[0] = gyr_z_Filt;
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    mx_HISTORY[0] = mx;
    my_HISTORY[0] = my;
    mz_HISTORY[0] = mz;
    mx_Filt_HISTORY[0] = mx_Filt;
    my_Filt_HISTORY[0] = my_Filt;
    mz_Filt_HISTORY[0] = mz_Filt;
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // Callibrating
    
    acc_x_Filt_OK = acc_x_Filt - acc_x_AVG;
    acc_y_Filt_OK = acc_y_Filt - acc_y_AVG;
    acc_z_Filt_OK = acc_z_Filt;// - acc_z_AVG;
    acc_xg = acc_x_Filt_OK * g;
    acc_yg = acc_y_Filt_OK * g;
    acc_zg = acc_z_Filt_OK * g;
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    gyr_x_Filt_OK = gyr_x_Filt - gyr_x_AVG;
    gyr_y_Filt_OK = gyr_y_Filt - gyr_y_AVG;
    gyr_z_Filt_OK = gyr_z_Filt - gyr_z_AVG;
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    mx_Filt_OK = mx_Filt;// - mx_AVG;
    my_Filt_OK = my_Filt;// - my_AVG;
    mz_Filt_OK = mz_Filt;// - mz_AVG;
    Serial.print("acc_x_OK = ");
    Serial.println(acc_xg,4);
    Serial.print("acc_y_OK = ");
    Serial.println(acc_yg,4);
    Serial.print("acc_z_OK = ");
    Serial.println(acc_zg,4);
    Serial.print("gyr_x_OK = ");
    Serial.println(gyr_x_Filt_OK,4);
    Serial.print("gyr_y_OK = ");
    Serial.println(gyr_y_Filt_OK,4);
    Serial.print("gyr_z_OK = ");
    Serial.println(gyr_z_Filt_OK,4);
    Serial.print("mag_x_OK = ");
    Serial.println(mx_Filt_OK,4);
    Serial.print("mag_y_OK = ");
    Serial.println(my_Filt_OK,4);
    Serial.print("mag_z_OK = ");
    Serial.println(mz_Filt_OK,4);



    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //Euler angles computation
    
    a_x_Madg = acc_x_Filt_OK; a_y_Madg = acc_y_Filt_OK; a_z_Madg = acc_z_Filt_OK;
    w_x_Madg = gyr_x_Filt_OK; w_y_Madg = gyr_y_Filt_OK; w_z_Madg = gyr_z_Filt_OK;
    m_x_Madg = mx_Filt_OK; m_y_Madg = my_Filt_OK; m_z_Madg = mz_Filt_OK;

    // Auxilary variables to avoid repeated calculation
    halfSEq_1 = 0.5f * SEq_1;
    halfSEq_2 = 0.5f * SEq_2;
    halfSEq_3 = 0.5f * SEq_3;
    halfSEq_4 = 0.5f * SEq_4;
    twoSEq_1 = 2.0f * SEq_1;
    twoSEq_2 = 2.0f * SEq_2;
    twoSEq_3 = 2.0f * SEq_3;
    twoSEq_4 = 2.0f * SEq_4;
    twob_x = 2.0f * b_x;
    twob_z = 2.0f * b_z;
    twob_xSEq_1 = 2.0f * b_x * SEq_1;
    twob_xSEq_2 = 2.0f * b_x * SEq_2;
    twob_xSEq_3 = 2.0f * b_x * SEq_3;
    twob_xSEq_4 = 2.0f * b_x * SEq_4;
    twob_zSEq_1 = 2.0f * b_z * SEq_1;
    twob_zSEq_2 = 2.0f * b_z * SEq_2;
    twob_zSEq_3 = 2.0f * b_z * SEq_3;
    twob_zSEq_4 = 2.0f * b_z * SEq_4;
    twom_x = 2.0f * m_x_Madg;
    twom_y = 2.0f * m_y_Madg;
    twom_z = 2.0f * m_z_Madg; 
  
    // normalize the accelerometer measurement
    norm = sqrt(a_x_Madg * a_x_Madg + a_y_Madg * a_y_Madg + a_z_Madg * a_z_Madg);
    a_x_Madg /= norm;
    a_y_Madg /= norm;
    a_z_Madg /= norm;
  
    // normalize the magnetometer measurement
    norm = sqrt(m_x_Madg * m_x_Madg + m_y_Madg * m_y_Madg + m_z_Madg * m_z_Madg);
    m_x_Madg /= norm;
    m_y_Madg /= norm;
    m_z_Madg /= norm;
  
    // compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x_Madg;
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y_Madg;
    f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z_Madg;
    f_4 = twob_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z *(SEq_2SEq_4 - SEq_1SEq_3) - m_x_Madg;
    f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y_Madg;
    f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z_Madg;
    J_11or24 = twoSEq_3;
    J_12or23 = 2.0f * SEq_4;
    J_13or22 = twoSEq_1;
    J_14or21 = twoSEq_2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;
    J_41 = twob_zSEq_3;
    J_42 = twob_zSEq_4;
    J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1;
    J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2;
    J_51 = twob_xSEq_4 - twob_zSEq_2;
    J_52 = twob_xSEq_3 + twob_zSEq_1;
    J_53 = twob_xSEq_2 + twob_zSEq_4;
    J_54 = twob_xSEq_1 - twob_zSEq_3;
    J_61 = twob_xSEq_3;
    J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
    J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
    J_64 = twob_xSEq_2;
  
    // Computethe gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
  
    // normalise the gradient to estimate direction of the gyroscope error
    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 = SEqHatDot_1 / norm;
    SEqHatDot_2 = SEqHatDot_2 / norm;
    SEqHatDot_3 = SEqHatDot_3 / norm;
    SEqHatDot_4 = SEqHatDot_4 / norm;
  
    // compute angular estimated direction of the gyroscope error
    w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
    w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
    w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
  
    // compute and remove the gyroscope biases
    w_bx += w_err_x * deltat * zeta;
    w_by += w_err_y * deltat * zeta;
    w_bz += w_err_z * deltat * zeta;
    w_x_Madg -= w_bx;
    w_y_Madg -= w_by;
    w_z_Madg -= w_bz;
    Serial.print("w_x_corr = ");
    Serial.println(w_x_Madg,4);
    Serial.print("w_y_corr = ");
    Serial.println(w_y_Madg,4);
    Serial.print("w_z_corr = ");
    Serial.println(w_z_Madg,4);
  
    // compute the quaternion rate measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * w_x_Madg - halfSEq_3 * w_y_Madg - halfSEq_4 * w_z_Madg;
    SEqDot_omega_2 = halfSEq_1 * w_x_Madg + halfSEq_3 * w_z_Madg - halfSEq_4 * w_y_Madg;
    SEqDot_omega_3 = halfSEq_1 * w_y_Madg - halfSEq_2 * w_z_Madg + halfSEq_4 * w_x_Madg;
    SEqDot_omega_4 = halfSEq_1 * w_z_Madg + halfSEq_2 * w_y_Madg - halfSEq_3 * w_x_Madg;
  
    // compute then integrate the estimated quaternion rate
    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
  
    // normalise quaternion
    norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 /= norm;
    SEq_2 /= norm;
    SEq_3 /= norm;
    SEq_4 /= norm;
    Serial.print("Quat_1 = ");
    Serial.println(SEq_1,4);
    Serial.print("Quat_2 = ");
    Serial.println(SEq_2,4);
    Serial.print("Quat_3 = ");
    Serial.println(SEq_3,4);
    Serial.print("Quat_4 = ");
    Serial.println(SEq_4,4);
  
    // compute magnetic flux in the earth frame
    SEq_1SEq_2 = SEq_1 * SEq_2;
    SEq_1SEq_3 = SEq_1 * SEq_3;
    SEq_1SEq_4 = SEq_1 * SEq_4;
    SEq_3SEq_4 = SEq_3 * SEq_4;
    SEq_2SEq_3 = SEq_2 * SEq_3;
    SEq_2SEq_4 = SEq_2 * SEq_4;
    h_x = twom_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
    h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - SEq_2 * SEq_2 - SEq_4 * SEq_4) + twom_z * (SEq_3SEq_4 * SEq_1SEq_2);
    h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3);
    
    // normalize the flux vector to have only components in the x and z
    b_x = sqrt(h_x * h_x + h_y * h_y);
    b_z = h_z; 


    // Coputation of Euler Angles
    Phi =atan2( (2.0f*SEq_3*SEq_4 - 2.0f*SEq_1*SEq_2),(2.0f*SEq_1*SEq_1 + 2.0f*SEq_4*SEq_4 - 1.0f) );
    Theta = -asin( (2.0f*SEq_2*SEq_4 + 2.0f*SEq_1*SEq_3) );
    Psi =atan2( (2.0f*SEq_2*SEq_3 - 2.0f*SEq_1*SEq_4),(2.0f*SEq_1*SEq_1 + 2.0f*SEq_2*SEq_2 - 1.0f) );
    Serial.print("Phi = ");
    Serial.println(Phi,4);
    Serial.print("Tht = ");
    Serial.println(Theta,4);
    Serial.print("Psi = ");
    Serial.println(Psi,4);

  }
  // delay(100);   
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
float LowPass_Filter(float inp, float *inp_HISTORY, float *out_HISTORY) {
  float out = 0; 
  float out_Num = 0;
  float out_Den = 0; 
  //float Numerator[Filter_Order+1] = {0.0628e-4, 0.1883e-4, 0.1883e-4, 0.0628e-4}; //Order=3
  //float Denominator[Filter_Order+1] = {1, -2.9266, 2.8559, -0.9293}; //Order=3
  float Numerator[Filter_Order+1] = {0.5459e-3, 2.1836e-3, 3.2754e-3, 2.1836e-3, 0.5459e-3}; //Order=4
  float Denominator[Filter_Order+1] = {1, -3.1681, 3.8582, -2.1293, 0.4481}; //Order=4
  //float Numerator[Filter_Order+1] = {0.0351e-0, 0.1755e-0, 0.3509e-0, 0.3509e-0, 0.1755e-0, 0.0351e-0}; //Order=5
  //float Denominator[Filter_Order+1] = {1, -0.5390, 0.8312, -0.2671, 0.1099, -0.0120}; //Order=5
  out_Num = inp * Numerator[0];
  for(int j=0;j<Filter_Order;j++){
    out_Num = out_Num + inp_HISTORY[j] * Numerator[j+1];
    out_Den = out_Den + out_HISTORY[j] * Denominator[j+1];
  }  
  out = out_Num - out_Den;
  return out;
}



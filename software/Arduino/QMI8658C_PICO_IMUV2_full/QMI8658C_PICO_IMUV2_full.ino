//TBSI-SSR LinZenan
//此固件适用于QMI8658C-低成本版本设计
//修改于2022年06月20日，此程序供潇洒学长SR磁控鱼项目使用-加入了freertos任务调度功能-加入了四元数求解功能(基于sensefusion)
//功能包括：上电WIFI-OTA检测升级，
//#include <Arduino.h>

//WIFI OTA相关
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>

//Bluetooth相关
#include "BluetoothSerial.h"

//传感器驱动相关
#include "QMI8658C.h"
#include "I2Cdev.h"
#include "JY901.h"

//四元数&欧拉角计算
#include "SensorFusion.h" //SF
SF fusion;

#define I2C_BUS          Wire                           // Define the I2C bus (Wire instance) you wish to use
I2Cdev                   i2c_0(&I2C_BUS);               // Instantiate the I2Cdev object and point to the desired I2C bus

// IIC端口号
#define SDA0 15
#define SCL0 4

#define SerialDebug false  // set to true to get Serial output for debugging
#define ad0 13 //set to high for QMI8658C
#define myLed_R 32

// // 模拟开关
// #define DC1 13
// #define DC2 15

// WIFI配置相关
const char* host = "esp32-dev";
const char* ssid = "TBSI-SSR-Zenan"; 
const char* password = "cn1234567890";
WebServer server(80);

// adc port*4
const int potPin_01 = 39; // TENG1检测
const int potPin_02 = 32; // TENG2检测
const int potPin_03 = 33; // res1检测
const int potPin_04 = 34; // res2检测
const int potPin_05 = 35; // 电池电压检测

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//cup为小端模式存储，也就是在存储的时候，低位被存在0字节，高位在1字节
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))     //取出int型变量的低字节
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))     //    取存储在此变量下一内存字节的内容，高字节
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

//角度到弧度转换
float pi = 3.141592653589793238462643383279502884f;

//bluetooth
BluetoothSerial SerialBT;

//led
static bool led_status = true;

// some parameters 
unsigned char DataToSend[30];//send to ANOV7

// 定义采集数据包-结构体
typedef struct {
  float data_imu_onboard[7];//on board imu raw data  [accx, accy, accz, gyrox, gyroy, gyroz, temp]
  float data_imu_exboard[10];//out of board imu raw data [accx, accy, accz, gyrox, gyroy, gyroz, magx, magy, magz, temp]
  int16_t data_adc[5];//raw data 0~4095
} Datapack_collect;

// 定义发送数据包-结构体
typedef struct {
  int16_t data_imu_onboard[7];//on board imu raw data
  int16_t data_imu_exboard[10];//out of board imu raw data
  int16_t data_adc[5];//raw data 0~4095
  int16_t onboard_euler_angle[3]; // roll pitch yaw * 100
  int16_t onboard_qu_value[4]; // q0~q3 * 10000
} Datapack_send;

Datapack_collect data_acquire_pack; // 采集的数据包
Datapack_send data_send_pack; // 发送的数据包

//mode define
int work_mode = 0; //状态机用于判断切换当前状态（0-当前状态为WIFI-OTA阶段；1-当前状态为Bluetooth阶段）
int count_break = 0; // 用于计数wifi连接失败次数，失败次数达到上限就退出wifi-OTA模式，进入Bluetooth模式
int send_mode = 0; //发送数据模式(0-走串口可以接入外部数传；1-走蓝牙虚拟串口)

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      * AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      * GFS_16DPS     , GFS_32DPS     , GFS_64DPS     , GFS_128DPS    , GFS_256DPS    , GFS_512DPS    , GFS_1024DPS   , GFS_2048DPS    
      * AODR_3Hz      , AODR_11Hz     , AODR_21Hz     , AODR_128Hz    , AODR_31_5Hz   , AODR_62_5Hz   , AODR_125Hz    , AODR_250Hz    , AODR_500Hz    , AODR_1000Hz   , AODR_2000Hz   , AODR_4000Hz   , AODR_8000Hz
      * GODR_31_5Hz   , GODR_62_5Hz   , GODR_125Hz    , GODR_250Hz    , GODR_500Hz    , GODR_1000Hz   , GODR_2000Hz   , GODR_4000Hz   , GODR_4000Hz, GODR_8000Hz   
*/ 
uint8_t Ascale = AFS_2G, Gscale = GFS_2048DPS, AODR = AODR_500Hz, GODR = GODR_500Hz;

float aRes, gRes;               // scale resolutions per LSB for the accel and gyro sensor2
float accelBias[3] = {0., 0., 0.}, gyroBias[3] = {0., 0., 0.}; // offset biases for the accel and gyro
int16_t QMI8658C_Data[7];        // Stores the 16-bit signed sensor output
float Gtemperature;           // Stores the real internal gyro temperature in degrees Celsius
float ax, ay, az, gx, gy, gz;  // variables to hold latest accel/gyro data values 

float e_Gtemperature;           // Stores the real internal gyro temperature in degrees Celsius
float e_ax, e_ay, e_az, e_gx, e_gy, e_gz, e_mx, e_my, e_mz;  // variables to hold latest accel/gyro/mag data values 
uint8_t QMI8658Cstatus;

bool newQMI8658C_Data = false;
bool newQMI8658CTap  = false;

QMI8658C QMI8658C(&i2c_0); // instantiate QMI8658C class

// freertos 任务中MUTEX信号锁定义
SemaphoreHandle_t xMutex_datapack = NULL; //创建信号量Handler
TickType_t timeOut = 5; //用于获取信号量的Timeout 5 ticks

// 欧拉角计算相关参数定义
float o_gx, o_gy, o_gz, o_ax, o_ay, o_az, o_mx, o_my, o_mz;
float pitch, roll, yaw;
float cal_q[4];
float deltat;

/*
 * Login page
 */
const char* loginIndex = 
 "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>SSR-Tactile sensing firmware update Login Page</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<td>Username:</td>"
        "<td><input type='text' size=25 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td>Password:</td>"
            "<td><input type='Password' size=25 name='pwd'><br></td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='zenan' && form.pwd.value=='zenan')"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
"</script>";
 
/*
 * Server Index Page
 */
 
const char* serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";
 

//电压采集
void ADC_read()
{
    data_acquire_pack.data_adc[0] = analogRead(potPin_01);  
    data_acquire_pack.data_adc[1] = analogRead(potPin_02);  
    data_acquire_pack.data_adc[2] = analogRead(potPin_03);  
    data_acquire_pack.data_adc[3] = analogRead(potPin_04);  
    data_acquire_pack.data_adc[4] = analogRead(potPin_05);  
}

//更新一次外置imu原始数据(9轴)
void read_raw_exboard_imu_data(void)
{
  // 读取imu原始数据
  JY901.GetAcc();
  JY901.GetGyro();  
  JY901.GetMag();
  JY901.GetTemp();
  
  // Now we'll calculate the accleration value into actual g's
  e_ax = (float)JY901.stcAcc.a[0]/32768*16;  // get actual g value, this depends on scale being set
  e_ay = (float)JY901.stcAcc.a[1]/32768*16;   
  e_az = (float)JY901.stcAcc.a[2]/32768*16;  
  // Calculate the gyro value into actual degrees per second
  e_gx = (float)JY901.stcGyro.w[0]/32768*2000/180*pi;  // get actual gyro value and transfer to rad/s, this depends on scale being set
  e_gy = (float)JY901.stcGyro.w[1]/32768*2000/180*pi;  
  e_gz = (float)JY901.stcGyro.w[2]/32768*2000/180*pi;
  // mag data
  e_mx = (float)JY901.stcMag.h[0]*1.0;  // get actual mag value, this depends on scale being set
  e_my = (float)JY901.stcMag.h[1]*1.0;  
  e_mz = (float)JY901.stcMag.h[2]*1.0; 
  // temp 
  e_Gtemperature = (float)JY901.Temp; // /100.0 Gyro chip temperature in degrees Centigrade
  
  //加速度计
  data_acquire_pack.data_imu_exboard[0] = e_ax;  // (int)1000* get actual g value, this depends on scale being set
  data_acquire_pack.data_imu_exboard[1] = e_ay;  // (int)1000*
  data_acquire_pack.data_imu_exboard[2] = e_az;  // (int)1000*

  //陀螺仪
  data_acquire_pack.data_imu_exboard[3] = e_gx;  // (int)get actual gyro value, this depends on scale being set
  data_acquire_pack.data_imu_exboard[4] = e_gy;  // (int)
  data_acquire_pack.data_imu_exboard[5] = e_gz;  // (int)

  //磁力计
  data_acquire_pack.data_imu_exboard[6] = e_mx;  // (int) get actual mag value, this depends on scale being set
  data_acquire_pack.data_imu_exboard[7] = e_my;  // (int) 
  data_acquire_pack.data_imu_exboard[8] = e_mz;  // (int)

  //温度
  data_acquire_pack.data_imu_exboard[9] = e_Gtemperature; // (int)
}

//更新一次imu原始数据
void read_raw_onboard_imu_data(void)
{
  // 读取imu原始数据
  QMI8658C.readData(QMI8658C_Data); 
   
  // Now we'll calculate the accleration value into actual g's
  // ax = (float)QMI8658C_Data[1]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
  // ay = (float)QMI8658C_Data[2]*aRes - accelBias[1];   
  // az = (float)QMI8658C_Data[3]*aRes - accelBias[2];  
  // Serial.print("aRes*10000 = ");
  // Serial.println(aRes*10000);
  // Serial.print("gRes*10000 = ");
  // Serial.println(gRes*10000);


  ax = (float)QMI8658C_Data[1]*aRes;  //  - accelBias[0] get actual g value, this depends on scale being set
  ay = (float)QMI8658C_Data[2]*aRes;  //  - accelBias[1]
  az = (float)QMI8658C_Data[3]*aRes;  //  - accelBias[2]
  // ax = (float)QMI8658C_Data[1]*aRes - accelBias[0];  //   get actual g value, this depends on scale being set
  // ay = (float)QMI8658C_Data[2]*aRes - accelBias[1];  //
  // az = (float)QMI8658C_Data[3]*aRes - accelBias[2];  // 

  // Calculate the gyro value into actual degrees per second
  gx = (float)QMI8658C_Data[4]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
  gy = (float)QMI8658C_Data[5]*gRes - gyroBias[1];  
  gz = (float)QMI8658C_Data[6]*gRes - gyroBias[2];
  // temp 
  Gtemperature = ((float) QMI8658C_Data[0]) / 256.0f * 100; // Gyro chip temperature in degrees Centigrade
  
  //加速度计
  data_acquire_pack.data_imu_onboard[0] = ax;  // (int)1000* get actual g value, this depends on scale being set
  data_acquire_pack.data_imu_onboard[1] = ay;  // (int)1000*
  data_acquire_pack.data_imu_onboard[2] = az;  // (int)1000*

  //陀螺仪 转为弧度制
  data_acquire_pack.data_imu_onboard[3] = gx/180*pi;  // (int) get actual gyro value, this depends on scale being set
  data_acquire_pack.data_imu_onboard[4] = gy/180*pi;  // (int)
  data_acquire_pack.data_imu_onboard[5] = gz/180*pi;  // (int)

  //温度
  data_acquire_pack.data_imu_onboard[6] = Gtemperature;
}


//ANOv7上位机波形观察 欧拉角
//每一个协议帧只能最多发送10个数据量(不是byte)
//协议：0xAA+0xFF+0x03+NUM(byte)+DATA(低位先发)+SC+AC 4个通道的值
void ANOv7_TENG_DATA_short_Send_03()
{
  unsigned char Max_data_num = 10;//每一帧最大发送数据量
  unsigned char _cnt_send = 0;//用于记录发送帧发送位数
  unsigned char _cnt_data = 0;//用于记录数据发送位数
  unsigned char SC = 0;//求和检验值
  unsigned char AC = 0;//累求和检验值
  unsigned long time; 
  /********************* 协议帧 发送过程（0x03）  *********************/
  //帧头
  DataToSend[_cnt_send++] = 0xAA;
  DataToSend[_cnt_send++] = 0xFF;
  DataToSend[_cnt_send++] = 0x03;
  DataToSend[_cnt_send++] = 3*2+1 ;  //short发送占两个字节

  // 欧拉角回传
  DataToSend[_cnt_send++] = BYTE0(data_send_pack.onboard_euler_angle[0]);
  DataToSend[_cnt_send++] = BYTE1(data_send_pack.onboard_euler_angle[0]);
  DataToSend[_cnt_send++] = BYTE0(data_send_pack.onboard_euler_angle[1]);
  DataToSend[_cnt_send++] = BYTE1(data_send_pack.onboard_euler_angle[1]);
  DataToSend[_cnt_send++] = BYTE0(data_send_pack.onboard_euler_angle[2]);
  DataToSend[_cnt_send++] = BYTE1(data_send_pack.onboard_euler_angle[2]);

  DataToSend[_cnt_send++] = 0; //fusion_sta

  //双校验
  for(unsigned char i = 0;i < (3*2+1+4);i++)
  {
    SC = DataToSend[i] + SC;
    AC = SC + AC;
  }

  DataToSend[_cnt_send++] = SC;
  DataToSend[_cnt_send++] = AC;

  //发送
  for(unsigned char i = 0;i < _cnt_send;i++)
  {
    if(send_mode == 0)
    {
      Serial.write(DataToSend[i]);
    }
    else
    {
      SerialBT.write(DataToSend[i]);
    }
  }
  /********************* 协议帧 发送过程（0x04）  *********************/
}


//ANOv7上位机波形观察 四元数
//每一个协议帧只能最多发送10个数据量(不是byte)
void ANOv7_TENG_DATA_short_Send_04(void)
{
  unsigned char Max_data_num = 10;//每一帧最大发送数据量
  unsigned char _cnt_send = 0;//用于记录发送帧发送位数
  unsigned char _cnt_data = 0;//用于记录数据发送位数
  unsigned char SC = 0;//求和检验值
  unsigned char AC = 0;//累求和检验值
  unsigned long time; 
  /********************* 协议帧 发送过程（0x04）  *********************/
  //帧头
  DataToSend[_cnt_send++] = 0xAA;
  DataToSend[_cnt_send++] = 0xFF;
  DataToSend[_cnt_send++] = 0x04;
  DataToSend[_cnt_send++] = 4*2+1;  //short发送占两个字节

  // 四元数回传
  DataToSend[_cnt_send++] = BYTE0(data_send_pack.onboard_qu_value[0]);
  DataToSend[_cnt_send++] = BYTE1(data_send_pack.onboard_qu_value[0]);
  DataToSend[_cnt_send++] = BYTE0(data_send_pack.onboard_qu_value[1]);
  DataToSend[_cnt_send++] = BYTE1(data_send_pack.onboard_qu_value[1]);
  DataToSend[_cnt_send++] = BYTE0(data_send_pack.onboard_qu_value[2]);
  DataToSend[_cnt_send++] = BYTE1(data_send_pack.onboard_qu_value[2]);
  DataToSend[_cnt_send++] = BYTE0(data_send_pack.onboard_qu_value[3]);
  DataToSend[_cnt_send++] = BYTE1(data_send_pack.onboard_qu_value[3]);

  DataToSend[_cnt_send++] = 0; //fusion_sta

  //双校验
  for(unsigned char i = 0;i < (4*2+1+4);i++)
  {
    SC = DataToSend[i] + SC;
    AC = SC + AC;
  }

  DataToSend[_cnt_send++] = SC;
  DataToSend[_cnt_send++] = AC;

  //发送
  for(unsigned char i = 0;i < _cnt_send;i++)
  {
    if(send_mode == 0)
    {
      Serial.write(DataToSend[i]);
    }
    else
    {
      SerialBT.write(DataToSend[i]);
    }
  }
  /********************* 协议帧 发送过程（0x04）  *********************/
}


//ANOv7上位机波形观察
//每一个协议帧只能最多发送10个数据量(不是byte)
//协议：0xAA+0xFF+0xF1+NUM(byte)+DATA(低位先发)+SC+AC 5个通道的ADC值
void ANOv7_TENG_DATA_short_Send1()
{
  unsigned char Max_data_num = 10;//每一帧最大发送数据量
  unsigned char _cnt_send = 0;//用于记录发送帧发送位数
  unsigned char _cnt_data = 0;//用于记录数据发送位数
  unsigned char SC = 0;//求和检验值
  unsigned char AC = 0;//累求和检验值

  /********************* 协议帧 发送过程（0xF1）  *********************/
  //帧头
  DataToSend[_cnt_send++] = 0xAA;
  DataToSend[_cnt_send++] = 0xFF;
  DataToSend[_cnt_send++] = 0xF1;
  DataToSend[_cnt_send++] = 5*2;//short发送占两个字节
  //数据
  for(_cnt_data = 0;_cnt_data < 5;_cnt_data++)
  {
    DataToSend[_cnt_send++] = BYTE0(data_send_pack.data_adc[_cnt_data]);
    DataToSend[_cnt_send++] = BYTE1(data_send_pack.data_adc[_cnt_data]);
  }
  //双校验
  for(unsigned char i = 0;i < (5*2+4);i++)
  {
    SC = DataToSend[i] + SC;
    AC = SC + AC;
  }

  DataToSend[_cnt_send++] = SC;
  DataToSend[_cnt_send++] = AC;

  //发送
  for(unsigned char i = 0;i < _cnt_send;i++)
  {
    if(send_mode == 0)
    {
      Serial.write(DataToSend[i]);
    }
    else
    {
      SerialBT.write(DataToSend[i]);
    }
  }
  /********************* 协议帧 发送过程（0xF1）  *********************/
}


//ANOv7上位机波形观察
//每一个协议帧只能最多发送10个数据量(不是byte)
//协议：0xAA+0xFF+0xF2+NUM(byte)+DATA(低位先发)+SC+AC 7个通道的值（accX,accY,accZ,gyroX,gyroY,gyroZ,temp,time0,time1）
void ANOv7_TENG_DATA_short_Send2()
{
  unsigned char Max_data_num = 10;//每一帧最大发送数据量
  unsigned char _cnt_send = 0;//用于记录发送帧发送位数
  unsigned char _cnt_data = 0;//用于记录数据发送位数
  unsigned char SC = 0;//求和检验值
  unsigned char AC = 0;//累求和检验值
  unsigned long time; 
  /********************* 协议帧 发送过程（0xF2）  *********************/
  //帧头
  DataToSend[_cnt_send++] = 0xAA;
  DataToSend[_cnt_send++] = 0xFF;
  DataToSend[_cnt_send++] = 0xF2;
  DataToSend[_cnt_send++] = 9*2;//short发送占两个字节
  //数据
  for(_cnt_data = 0;_cnt_data < 7;_cnt_data++)
  {
    DataToSend[_cnt_send++] = BYTE0(data_send_pack.data_imu_onboard[_cnt_data]);
    DataToSend[_cnt_send++] = BYTE1(data_send_pack.data_imu_onboard[_cnt_data]);
  }
  //加入时间刻度
  time = millis();
  DataToSend[_cnt_send++] = BYTE0(time);
  DataToSend[_cnt_send++] = BYTE1(time);
  DataToSend[_cnt_send++] = BYTE2(time);
  DataToSend[_cnt_send++] = BYTE3(time);
  
  //双校验
  for(unsigned char i = 0;i < (9*2+4);i++)
  {
    SC = DataToSend[i] + SC;
    AC = SC + AC;
  }

  DataToSend[_cnt_send++] = SC;
  DataToSend[_cnt_send++] = AC;

  //发送
  for(unsigned char i = 0;i < _cnt_send;i++)
  {
    if(send_mode == 0)
    {
      Serial.write(DataToSend[i]);
    }
    else
    {
      SerialBT.write(DataToSend[i]);
    }
  }
  /********************* 协议帧 发送过程（0xF2）  *********************/
}


//ANOv7上位机波形观察
//每一个协议帧只能最多发送10个数据量(不是byte)
//协议：0xAA+0xFF+0xF3+NUM(byte)+DATA(低位先发)+SC+AC 10个通道的值（e_accX,e_accY,e_accZ,e_gyroX,e_gyroY,e_gyroZ,e_magX,e_magY,e_magZ,e_temp）
void ANOv7_TENG_DATA_short_Send3()
{
  unsigned char Max_data_num = 10;//每一帧最大发送数据量
  unsigned char _cnt_send = 0;//用于记录发送帧发送位数
  unsigned char _cnt_data = 0;//用于记录数据发送位数
  unsigned char SC = 0;//求和检验值
  unsigned char AC = 0;//累求和检验值
  unsigned long time; 
  /********************* 协议帧 发送过程（0xF3）  *********************/
  //帧头
  DataToSend[_cnt_send++] = 0xAA;
  DataToSend[_cnt_send++] = 0xFF;
  DataToSend[_cnt_send++] = 0xF3;
  DataToSend[_cnt_send++] = 10*2;//short发送占两个字节
  //数据
  for(_cnt_data = 0;_cnt_data < 10;_cnt_data++)
  {
    DataToSend[_cnt_send++] = BYTE0(data_send_pack.data_imu_exboard[_cnt_data]);
    DataToSend[_cnt_send++] = BYTE1(data_send_pack.data_imu_exboard[_cnt_data]);
  }
  
  //双校验
  for(unsigned char i = 0;i < (10*2+4);i++)
  {
    SC = DataToSend[i] + SC;
    AC = SC + AC;
  }

  DataToSend[_cnt_send++] = SC;
  DataToSend[_cnt_send++] = AC;

  //发送
  for(unsigned char i = 0;i < _cnt_send;i++)
  {
    if(send_mode == 0)
    {
      Serial.write(DataToSend[i]);
    }
    else
    {
      SerialBT.write(DataToSend[i]);
    }
  }
  /********************* 协议帧 发送过程（0xF3）  *********************/
}


//LED_Task 任务主体(兼容debug功能)
void LED_Task(void *ptParam) {  
  // 初始化当前tick时刻
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // 定义单周期的tick数量
  const TickType_t xFrequency = 500; // 间隔 500 ticks = 0.5 seconds = 2Hz
  
  for(;;) //使用for更高效
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    // LED_Task 主任务
    led_status = !led_status;
    if(led_status==true)
    {
      digitalWrite(myLed_R, LOW); // start with led on
      // digitalWrite(myLed_G, HIGH); // start with led on
      // digitalWrite(myLed_B, LOW); // start with led on
    }
    else
    {
      digitalWrite(myLed_R, HIGH); // start with led on
      // digitalWrite(myLed_G, LOW); // start with led on
      // digitalWrite(myLed_B, HIGH); // start with led on
    }
    // 串口打印板载imu传感器信息-for debug
    if(SerialDebug == 1 && work_mode == 1) {
      Serial.println("On board imu data record:");
      Serial.print("ax = "); Serial.print((int)1000*ax);  
      Serial.print(" ay = "); Serial.print((int)1000*ay); 
      Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
      Serial.print("gx = "); Serial.print( gx, 2); 
      Serial.print(" gy = "); Serial.print( gy, 2); 
      Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
      Serial.print("Gyro temperature is ");  Serial.print(Gtemperature, 1);  Serial.println("° C"); // Print T values to tenths of s degree C

      Serial.println("Out of board imu data record:");
      Serial.print("e_ax = "); Serial.print((int)1000*e_ax);  
      Serial.print(" e_ay = "); Serial.print((int)1000*e_ay); 
      Serial.print(" e_az = "); Serial.print((int)1000*e_az); Serial.println(" mg");
      Serial.print("e_gx = "); Serial.print( e_gx, 2); 
      Serial.print(" e_gy = "); Serial.print( e_gy, 2); 
      Serial.print(" e_gz = "); Serial.print( e_gz, 2); Serial.println(" deg/s");
      Serial.print("e_mx = "); Serial.print( e_mx, 2); 
      Serial.print(" e_my = "); Serial.print( e_my, 2); 
      Serial.print(" e_mz = "); Serial.print( e_mz, 2); Serial.println("");
      Serial.print("e_temperature is ");  Serial.print(e_Gtemperature, 1);  Serial.println("° C"); // Print T values to tenths of s degree C
    }
  }
}


//Acquire_Task 任务主体
void Acquire_Task(void *ptParam) {  
  // 初始化当前tick时刻
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // 定义单周期的tick数量
  const TickType_t xFrequency = 8; // 间隔 8 ticks = 8 ms = 125Hz
  
  for(;;) //使用for更高效
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    // 首先查询datapack的Mutex锁是否被占用
    if (xSemaphoreTake(xMutex_datapack, timeOut) == pdPASS) { // 未被占用
      
      // Acquire_Task 主任务
      read_raw_onboard_imu_data(); // 板载imu数据获取
      // read_raw_exboard_imu_data(); // 外置imu数据获取
      ADC_read(); // adc数据获取
      
      xSemaphoreGive(xMutex_datapack); //释放钥匙
    }
    else{
      // 发现被占用 do nothing
    }
  }
}

//结构体深度拷贝(√)
void data_copy(Datapack_collect *data_ac, Datapack_send *data_se)
{
  for(int i = 0;i < 7;i++)
  {
    data_se->data_imu_onboard[i] = (int16_t)(100*data_ac->data_imu_onboard[i]);
  }
  for(int i = 0;i < 10;i++)
  {
    data_se->data_imu_exboard[i] = (int16_t)(100*data_ac->data_imu_exboard[i]);
  }
  for(int i = 0;i < 5;i++)
  {
    data_se->data_adc[i] = (int16_t)data_ac->data_adc[i];
  }
  // data_se->onboard_euler_angle[0] = roll*10;
  // data_se->onboard_euler_angle[1] = pitch*10;
  // data_se->onboard_euler_angle[2] = yaw*10;
  
}


//Cal_Euler_angle_Task 任务主体
void Cal_Euler_angle_Task(void *ptPram)
{
  // 初始化当前tick时刻
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // 定义单周期的tick数量
  const TickType_t xFrequency = 10; // 间隔 10 ticks = 10 ms = 100Hz
  
  for(;;) //使用for更高效
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    // 首先查询datapack的Mutex锁是否被占用
    if (xSemaphoreTake(xMutex_datapack, timeOut) == pdPASS) { // 未被占用
      
      // Cal_Euler_angle_Task 主任务 这里主任务必须尽量轻不要占用过多时间！！！(遵守先把数据提取出来，然后立刻释放，数据再处理)
      o_ax = data_acquire_pack.data_imu_onboard[0] * 9.8f;
      o_ay = data_acquire_pack.data_imu_onboard[1] * 9.8f;
      o_az = data_acquire_pack.data_imu_onboard[2] * 9.8f;
      o_gx = data_acquire_pack.data_imu_onboard[3];
      o_gy = data_acquire_pack.data_imu_onboard[4];
      o_gz = data_acquire_pack.data_imu_onboard[5];

      // o_ax = data_acquire_pack.data_imu_exboard[0];
      // o_ay = data_acquire_pack.data_imu_exboard[1];
      // o_az = data_acquire_pack.data_imu_exboard[2];
      // o_gx = data_acquire_pack.data_imu_exboard[3];
      // o_gx = data_acquire_pack.data_imu_exboard[4];
      // o_gx = data_acquire_pack.data_imu_exboard[5];
      // o_mx = data_acquire_pack.data_imu_exboard[6];
      // o_mx = data_acquire_pack.data_imu_exboard[7];
      // o_mx = data_acquire_pack.data_imu_exboard[8];      
      xSemaphoreGive(xMutex_datapack); //释放钥匙
    }
    else{
      // 发现被占用 do nothing
    }

    deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
    //choose only one of these two:
    fusion.MahonyUpdate(o_gx, o_gy, o_gz, o_ax, o_ay, o_az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
    // fusion.MadgwickUpdate(o_gx, o_gy, o_gz, o_ax, o_ay, o_az, o_mx, o_my, o_mz, deltat);  //else use the magwick, it is slower but more accurate
    pitch = fusion.getPitch();
    roll = fusion.getRoll();
    yaw = fusion.getYaw();

    cal_q[0] = fusion.getQuat()[0];
    cal_q[1] = fusion.getQuat()[1];
    cal_q[2] = fusion.getQuat()[2];
    cal_q[3] = fusion.getQuat()[3];

    data_send_pack.onboard_euler_angle[0] = int16_t(roll * 100.0f);
    data_send_pack.onboard_euler_angle[1] = int16_t(pitch * 100.0f);
    data_send_pack.onboard_euler_angle[2] = int16_t(yaw * 100.0f);
    data_send_pack.onboard_qu_value[0] = int16_t(cal_q[0] * 10000.0f);
    data_send_pack.onboard_qu_value[1] = int16_t(cal_q[1] * 10000.0f);
    data_send_pack.onboard_qu_value[2] = int16_t(cal_q[2] * 10000.0f);
    data_send_pack.onboard_qu_value[3] = int16_t(cal_q[3] * 10000.0f);

    pitch = fusion.getPitchRadians();
    roll = fusion.getRollRadians();
    yaw = fusion.getYawRadians();

    // Serial.print("o_ax:"); 
    // Serial.println(o_ax); 
    // Serial.print("o_ay:"); 
    // Serial.println(o_ay); 
    // Serial.print("o_az:"); 
    // Serial.println(o_az);
    // Serial.print("o_a** = ");
    // Serial.println(o_ax*o_ax+o_ay*o_ay+o_az*o_az);

    // Serial.print("o_gx:"); 
    // Serial.println(o_gx); 
    // Serial.print("o_gy:"); 
    // Serial.println(o_gy); 
    // Serial.print("o_gz:"); 
    // Serial.println(o_gz);

    // Serial.print("o_mx:"); 
    // Serial.println(o_mx); 
    // Serial.print("o_my:"); 
    // Serial.println(o_my); 
    // Serial.print("o_mz:"); 
    // Serial.println(o_mz);
    if(send_mode == 0) // 注意这里逻辑和send_task相反，send_task发蓝牙这边就发串口，send_task发串口这边就发蓝牙
    {
      SerialBT.print(pitch); 
      SerialBT.print(":");
      SerialBT.print(roll); 
      SerialBT.print(":");
      SerialBT.println(yaw);
    }
    else if(send_mode == 1)
    {
      Serial.print(pitch); 
      Serial.print(":");
      Serial.print(roll); 
      Serial.print(":");
      Serial.println(yaw);
    }
  }
}

//Send_Task 任务主体
void Send_Task(void *ptParam) {  
  // 初始化当前tick时刻
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // 定义单周期的tick数量
  const TickType_t xFrequency = 10; // 间隔 10 ticks = 10 ms = 100Hz
  
  for(;;) //使用for更高效
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    // 首先查询datapack的Mutex锁是否被占用
    if (xSemaphoreTake(xMutex_datapack, timeOut) == pdPASS) { // 未被占用
      
      // Send_Task 主任务 这里主任务必须尽量轻不要占用过多时间！！！(遵守先把数据提取出来，然后立刻释放，数据再发送出去)
      // struct_shallow_copy();// 浅拷贝数据会出问题，因为存在数组传递的只是地址不是具体的数值！！！
      data_copy(&data_acquire_pack, &data_send_pack);
      xSemaphoreGive(xMutex_datapack); //释放钥匙
    }
    else{
      // 发现被占用 do nothing
    }

    // 发送单独放出来，不过多占用MUTEX时间
    // ANOv7_TENG_DATA_short_Send1();
    ANOv7_TENG_DATA_short_Send2(); // on board imu data(6轴 acc_x/y/z+gyro_x/y/z+temp+time1+time2)
    ANOv7_TENG_DATA_short_Send_04();
    ANOv7_TENG_DATA_short_Send_03();
    // ANOv7_TENG_DATA_short_Send3(); // ex board imu data(9轴 acc_x/y/z+gyro_x/y/z+mag_x/y/z+temp)
  }
}


//WIFI_Update_Task 任务主体
void WIFI_Update_Task(void *ptParam) {  
  // 初始化当前tick时刻
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // 定义单周期的tick数量
  const TickType_t xFrequency = 8; // 间隔 8 ticks = 8 ms = 125Hz
  
  for(;;) //使用for更高效
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    // WIFI_Update_Task 主任务
    server.handleClient();
  }
}


// freertos任务创建
void freertos_task_create(int work_mode)
{
  if(work_mode == 0) //说明进入WIFI升级模式, 此时只有两个任务LED_Task, 
  { 
    xTaskCreatePinnedToCore(LED_Task, "LED_Blink", 1024 * 8, NULL, 1, NULL, 1);
    vTaskDelay(1000);
    xTaskCreatePinnedToCore(WIFI_Update_Task, "WIFI_Update", 1024 * 8, NULL, 2, NULL, 1);
    Serial.println("Working on mode_0, has created 2 tasks[LED_Blink-2Hz, WIFI_Update-125Hz]");
  }
  else if(work_mode == 1) //说明进入数据采集模式
  {
    xMutex_datapack = xSemaphoreCreateMutex(); //创建MUTEX
    xTaskCreatePinnedToCore(LED_Task, "LED_Blink", 1024 * 8, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(Acquire_Task, "Datapack_Acquisition", 1024 * 8, NULL, 2, NULL, 1);
    vTaskDelay(1000); //让数据采集程序提前先运行一秒获取第一笔数据
    xTaskCreatePinnedToCore(Send_Task, "Datapack_Send", 1024 * 8, NULL, 2, NULL, 1);
    vTaskDelay(1000);
    xTaskCreatePinnedToCore(Cal_Euler_angle_Task, "Cal_Euler_angle", 1024*8,  NULL, 2, NULL, 1);
    Serial.println("Working on mode_1, has created 4 tasks[LED_Blink-2Hz, Datapack_Acquisition-125Hz, Datapack_Send-100Hz, Cal_Euler_angle-10Hz]");
  }
}


void setup(void) {
  Serial.begin(115200);
  
  // Configure led
  pinMode(myLed_R, OUTPUT);
  digitalWrite(myLed_R, LOW); // start with led on
  pinMode(ad0, OUTPUT);
  digitalWrite(ad0, LOW); //  set QMI8658C mode
  delay(1000);
  
  // Connect to WiFi network
  Serial.println("WIFI start connecting...");
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  count_break = 0; // 用于计数wifi连接失败次数，失败次数达到上限就退出wifi-OTA模式，进入Bluetooth模式
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    count_break++;
    if(count_break == 5) //约为2.5s
    {
      //连接10s以上说明已经不存在default-wifi因此跳出尝试连接状态
      work_mode = 1; //状态机用于判断切换当前状态（0-当前状态为WIFI-OTA阶段；1-当前状态为Bluetooth阶段）
      Serial.println("");
      Serial.println("Fail to connect to WIFI...");
      break;
    }
    Serial.print(".");
  }

  if(work_mode == 0) // 已经连上wifi了进入ota升级模式
  {
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    /*use mdns for host name resolution*/
    if (!MDNS.begin(host)) { //http://esp32.local
      Serial.println(F("Error setting up MDNS responder!"));
      while (1) {
        delay(1000);
      }
    }
    Serial.println(F("mDNS responder started"));
    Serial.println("You can upgrade this device...");
    /*return index page which is stored in serverIndex */
    server.on("/", HTTP_GET, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/html", loginIndex);
    });
    
    server.on("/serverIndex", HTTP_GET, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/html", serverIndex);
    });
    /*handling uploading firmware file */
    server.on("/update", HTTP_POST, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
      ESP.restart();
    }, []() {
      HTTPUpload& upload = server.upload();
      if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("Update: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        /* flashing firmware to ESP*/
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) { //true to set the size to the current progress
          Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        } else {
          Update.printError(Serial);
        }
      }
    });
    server.begin();

    // 创建freertos任务
    freertos_task_create(0);
  }
  else if(work_mode == 1) // 未找到wifi所以进入蓝牙正常工作模式
  {
    //关闭wifi
    WiFi.disconnect();
    delay(2000);
    
    //打开蓝牙功能
    SerialBT.begin("TBSI-SSR-PICO-IMU_V2"); //Bluetooth device name
    Serial.println("The device started, now you can pair it with bluetooth!");
    delay(2000);

    // // Configure 模拟开关（低电平关闭）
    // pinMode(DC1, OUTPUT);
    // digitalWrite(DC1, LOW); // start with led on
    // pinMode(DC2, OUTPUT);
    // digitalWrite(DC2, LOW); // start with led on
    // delay(100);
   
    Wire.begin(SDA0, SCL0); // set master mode 
    Wire.setClock(400000); // I2C frequency at 400 kHz  
    delay(1000);
   
    i2c_0.I2Cscan();
    digitalWrite(myLed_R, HIGH);
    delay(1000);
    digitalWrite(myLed_R, LOW);
    delay(1000);
    digitalWrite(myLed_R, HIGH);
    delay(1000);
    digitalWrite(myLed_R, LOW);
    delay(1000);

    // Read the QMI8658C Chip ID register, this is a good test of communication
    Serial.println("QMI8658C accel/gyro...");
    byte c = QMI8658C.getChipID();  // Read CHIP_ID register for QMI8658C
    Serial.print("QMI8658C "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x05, HEX);
    Serial.println(" ");
    delay(1000); 
    if(c == 0x05) // check if all I2C sensors have acknowledged
    {
       Serial.println("QMI8658C are online..."); Serial.println(" ");
       
       digitalWrite(myLed_R, HIGH);
       delay(1000);
       digitalWrite(myLed_R, LOW);
    
       // get sensor resolutions, only need to do this once
       aRes = QMI8658C.getAres(Ascale);
       gRes = QMI8658C.getGres(Gscale);

       Serial.print("aRes*10000 = ");
       Serial.println(aRes*10000);
       Serial.print("gRes*10000 = ");
       Serial.println(gRes*10000);
    
       QMI8658C.init(Ascale, Gscale, AODR, GODR);
    
       QMI8658C.offsetBias(accelBias, gyroBias);
       Serial.println("accel biases (mg)"); Serial.println(1000.0f * accelBias[0]); Serial.println(1000.0f * accelBias[1]); Serial.println(1000.0f * accelBias[2]);
       Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
       delay(1000); 
    
       digitalWrite(myLed_R, HIGH);
    }
    else 
    {
      if(c != 0x05) Serial.println(" QMI8658C not functioning!");
      while(1){}; //不成功就阻塞
    }
    // JY901.StartIIC(2);// 最后开启外置imu

    // 创建freertos任务
    freertos_task_create(1);
  }
}
// End of setup //



void loop() {
  // all task has been transfered to freertos task !!! no need for loop();
}
// End of main loop //




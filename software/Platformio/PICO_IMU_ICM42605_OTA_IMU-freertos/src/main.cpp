//TBSI-SSR LinZenan
//板载ICM42605 6轴传感器
//修改于2022年06月13日，此程序供潇洒学长磁控鱼项目数据采集项目使用
#include <Arduino.h>

//WIFI OTA相关
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>

#include "BluetoothSerial.h"
#include "ICM42605.h"
#include "I2Cdev.h"
#include "JY901.h"

#define I2C_BUS          Wire                           // Define the I2C bus (Wire instance) you wish to use
I2Cdev                   i2c_0(&I2C_BUS);               // Instantiate the I2Cdev object and point to the desired I2C bus

// IIC端口号
#define SDA0 15
#define SCL0 4

#define SerialDebug false  // set to true to get Serial output for debugging
#define myLed 32

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

//定时器中断
#define SET_US 100 //设置定时器中断时间
hw_timer_t *timer=NULL;//创建一个定时器结构体
volatile SemaphoreHandle_t timerSemaphore;//创建一个定时器信号量
int time_count=0;//用于设置标志位
int time_status_0_5s = 0; //0.5s标志位
int time_status_0_2ms = 0; //0.1ms标志位
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//bluetooth
BluetoothSerial SerialBT;

//led
static bool led_status = true;

// some parameters 
unsigned char DataToSend[30];//send to ANOV7

// 定义采集数据包-结构体
typedef struct {
  int16_t data_imu_onboard[7];//on board imu raw data
  int16_t data_imu_exboard[10];//out of board imu raw data
  int16_t data_adc[5];//raw data 0~4095
} Datapack;

Datapack data_acquire_pack; // 采集的数据包
Datapack data_send_pack; // 发送的数据包

//mode define
int work_mode = 0; //状态机用于判断切换当前状态（0-当前状态为WIFI-OTA阶段；1-当前状态为Bluetooth阶段）
int count_break = 0; // 用于计数wifi连接失败次数，失败次数达到上限就退出wifi-OTA模式，进入Bluetooth模式
int send_mode = 1; //发送数据模式(0-走串口可以接入外部数传；1-走蓝牙虚拟串口)

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      GFS_15_125DPS, GFS_31_25DPS, GFS_62_5DPS, GFS_125DPS, GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS 
      AODR_1_5625Hz, AODR_3_125Hz, AODR_6_25Hz, AODR_12_5Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_500Hz, AODR_1000Hz, AODR_2000Hz, AODR_4000Hz, AODR_8000Hz
      GODR_12_5Hz, GODR_25Hz, GODR_50Hz, GODR_100Hz, GODR_200Hz, GODR_500Hz, GODR_1000Hz, GODR_2000Hz, GODR_4000Hz, GODR_8000Hz
*/ 
uint8_t Ascale = AFS_16G, Gscale = GFS_2000DPS, AODR = AODR_1000Hz, GODR = GODR_1000Hz; // 板载IMU量程速率配置，保持和外置IMU一致

float aRes, gRes;               // scale resolutions per LSB for the accel and gyro sensor2
float accelBias[3] = {0., 0., 0.}, gyroBias[3] = {0., 0., 0.}; // offset biases for the accel and gyro
int16_t ICM42605Data[7];        // Stores the 16-bit signed sensor output

float Gtemperature;           // Stores the real internal gyro temperature in degrees Celsius
float ax, ay, az, gx, gy, gz;  // variables to hold latest accel/gyro data values 

float e_Gtemperature;           // Stores the real internal gyro temperature in degrees Celsius
float e_ax, e_ay, e_az, e_gx, e_gy, e_gz, e_mx, e_my, e_mz;  // variables to hold latest accel/gyro/mag data values 

uint8_t ICM42605status;

bool newICM42605Data = false;
bool newICM42605Tap  = false;

ICM42605 ICM42605(&i2c_0); // instantiate ICM42605 class

// freertos 任务中MUTEX信号锁定义
SemaphoreHandle_t xMutex_datapack = NULL; //创建信号量Handler
TickType_t timeOut = 5; //用于获取信号量的Timeout 5 ticks

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


//更新一次板载imu原始数据（6轴）
void read_raw_onboard_imu_data(void)
{
  // 读取imu原始数据
  ICM42605.readData(ICM42605Data); 
   
  // Now we'll calculate the accleration value into actual g's
  ax = (float)ICM42605Data[1]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
  ay = (float)ICM42605Data[2]*aRes - accelBias[1];   
  az = (float)ICM42605Data[3]*aRes - accelBias[2];  
  // Calculate the gyro value into actual degrees per second
  gx = (float)ICM42605Data[4]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
  gy = (float)ICM42605Data[5]*gRes - gyroBias[1];  
  gz = (float)ICM42605Data[6]*gRes - gyroBias[2];
  // temp 
  Gtemperature = ((float) ICM42605Data[0]) / 132.48f + 25.0f * 100; // Gyro chip temperature in degrees Centigrade
  
  //加速度计
  data_acquire_pack.data_imu_onboard[0] = (int)1000*ax;  // get actual g value, this depends on scale being set
  data_acquire_pack.data_imu_onboard[1] = (int)1000*ay;   
  data_acquire_pack.data_imu_onboard[2] = (int)1000*az;

  //陀螺仪
  data_acquire_pack.data_imu_onboard[3] = (int)gx;  // get actual gyro value, this depends on scale being set
  data_acquire_pack.data_imu_onboard[4] = (int)gy;   
  data_acquire_pack.data_imu_onboard[5] = (int)gz;

  //温度
  data_acquire_pack.data_imu_onboard[6] = Gtemperature;
}


//电压采集
void ADC_read()
{
    data_acquire_pack.data_adc[0] = analogRead(potPin_01);  
    data_acquire_pack.data_adc[1] = analogRead(potPin_02);  
    data_acquire_pack.data_adc[2] = analogRead(potPin_03);  
    data_acquire_pack.data_adc[3] = analogRead(potPin_04);  
    data_acquire_pack.data_adc[4] = analogRead(potPin_05);  
}


//ANOv7上位机波形观察
//每一个协议帧只能最多发送10个数据量(不是byte)
//协议：0xAA+0xFF+0xF1+NUM(byte)+DATA(低位先发)+SC+AC 4个通道的ADC值
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
    DataToSend[_cnt_send++] = BYTE0(data_acquire_pack.data_adc[_cnt_data]);
    DataToSend[_cnt_send++] = BYTE1(data_acquire_pack.data_adc[_cnt_data]);
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
      digitalWrite(myLed, LOW); // start with led on
    }
    else
    {
      digitalWrite(myLed, HIGH); // start with led on
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
      ADC_read(); // adc数据获取
      
      xSemaphoreGive(xMutex_datapack); //释放钥匙
    }
    else{
      // 发现被占用 do nothing
    }
  }
}

//由于结构体中不存在指针可以直接使用浅拷贝复制获得的数据(×)
void struct_shallow_copy(void)
{
    data_send_pack = data_acquire_pack;       //structure directly assignment
}

//结构体深度拷贝(√)
void struct_deep_copy(Datapack *data_ac, Datapack *data_se)
{
  for(int i = 0;i < 7;i++)
  {
    data_se->data_imu_onboard[i] = data_ac->data_imu_onboard[i];
  }
  for(int i = 0;i < 5;i++)
  {
    data_se->data_adc[i] = data_ac->data_adc[i];
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
      struct_deep_copy(&data_acquire_pack, &data_send_pack);
      xSemaphoreGive(xMutex_datapack); //释放钥匙
    }
    else{
      // 发现被占用 do nothing
    }

    // 发送单独放出来，不过多占用MUTEX时间
    ANOv7_TENG_DATA_short_Send1();
    ANOv7_TENG_DATA_short_Send2(); // on board imu data(6轴 acc_x/y/z+gyro_x/y/z+temp+time1+time2)
    ANOv7_TENG_DATA_short_Send3(); // ex board imu data(9轴 acc_x/y/z+gyro_x/y/z+mag_x/y/z+temp)
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
    Serial.println("Working on mode_1, has created 3 tasks[LED_Blink-2Hz, Datapack_Acquisition-125Hz, Datapack_Send-100Hz]");
  }
}

void setup() {
  Serial.begin(115200);
  
  // Configure led
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW); // start with led on 
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
    SerialBT.begin("TBSI-SSR-PICO-IMU-V1"); //Bluetooth device name
    Serial.println("The device started, now you can pair it with bluetooth!");
    delay(2000);
  
    Wire.begin(SDA0, SCL0); // set master mode 
    Wire.setClock(400000); // I2C frequency at 400 kHz  
    delay(1000);
  
    i2c_0.I2Cscan();
    digitalWrite(myLed, HIGH);
    delay(1000);
    digitalWrite(myLed, LOW);
    delay(1000);
    digitalWrite(myLed, HIGH);
    delay(1000);
    digitalWrite(myLed, LOW);
    delay(1000);
    // Read the ICM42605 Chip ID register, this is a good test of communication
    Serial.println("On board IMU--ICM42605 accel/gyro...");
    byte c = ICM42605.getChipID();  // Read CHIP_ID register for ICM42605
    Serial.print("On board IMU--ICM42605 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x42, HEX);
    Serial.println(" ");
    delay(1000); 
    if(c == 0x42) // check if all I2C sensors have acknowledged
    {
    Serial.println("On board IMU--ICM42605 are online..."); Serial.println(" ");
    
    digitalWrite(myLed, HIGH);
    delay(1000);
    digitalWrite(myLed, LOW);
    ICM42605.reset();  // software reset ICM42605 to default registers

    // on board configuration get sensor resolutions, only need to do this once
    aRes = ICM42605.getAres(Ascale);
    gRes = ICM42605.getGres(Gscale);

    ICM42605.init(Ascale, Gscale, AODR, GODR);

    ICM42605.offsetBias(accelBias, gyroBias);
    Serial.println("On board IMU--accel biases (mg)"); Serial.println(1000.0f * accelBias[0]); Serial.println(1000.0f * accelBias[1]); Serial.println(1000.0f * accelBias[2]);
    Serial.println("On board IMU--gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
    delay(1000); 

    digitalWrite(myLed, HIGH);
    }
    else 
    {
      if(c != 0x6A) Serial.println("On board IMU--ICM42605 not functioning!");
      while(1){}; //不成功就阻塞
    }
      // 创建freertos任务
      freertos_task_create(1);
  }
}
// End of setup //


void loop() {
    // all task has been transfered to freertos task !!! no need for loop();
}
// End of main loop //


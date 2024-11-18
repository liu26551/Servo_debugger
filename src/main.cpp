/*
 * @Author: LIUXY 2816184983@qq.com
 * @Date: 2024-06-05 14:41:36
 * @LastEditors: LIUXY 2816184983@qq.com
 * @LastEditTime: 2024-11-10 12:50:10
 * @FilePath: \Esp32_air_filtere:\PlatformIO\Projects\Servo_debugger\src\main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: LIUXY 2816184983@qq.com
 * @Date: 2024-06-02 22:07:56
 * @LastEditors: LIUXY 2816184983@qq.com
 * @LastEditTime: 2024-09-30 14:58:43
 * @FilePath: \micro_ros2_espteste:\PlatformIO\Projects\Servo_controller\src\main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// #include <FastLED.h>
#include "LobotBusServo.hpp"
#include <ESP32Servo.h>
#include "badapple.h"

// How many leds in your strip?
#define NUM_LEDS 1
#define WS2812_PIN 21
#define LED_PIN 15
// Define the array of leds
// CRGB leds[NUM_LEDS];

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// 图片像素高度，宽度
#define LOGO_WIDTH 85
#define LOGO_HEIGHT 64
// 图片数量
#define LOGO_NUM 657

// 电池电压ADC检测引脚
// #define ADC_BAT 33
#define ADC_BAT 34
// 电位器检测引脚
// #define ADC_P 34
#define ADC_P 35

// 定义伺服电机1的引脚号
// #define SERVO1_PIN 16
#define SERVO1_PIN 25
// 定义伺服电机2的引脚号
// #define SERVO2_PIN 17
#define SERVO2_PIN 26

// 定义WS2812 LED灯的引脚号

/* 定义键盘按键的引脚编号 */
// #define KEY1_PIN 4  /* 键盘第一个按键的引脚 */
// #define KEY2_PIN 32 /* 键盘第二个按键的引脚 */
// #define KEY3_PIN 25 /* 键盘第三个按键的引脚 */
// #define KEY4_PIN 27 /* 键盘第四个按键的引脚 */
// #define KEY5_PIN 22 /* 键盘第五个按键的引脚 */

#define KEY1_PIN 18 /* 键盘第一个按键的引脚 UP */
#define KEY2_PIN 4  /* 键盘第二个按键的引脚 DOWN */
#define KEY3_PIN 16 /* 键盘第三个按键的引脚 OK */
#define KEY4_PIN 19 /* 键盘第四个按键的引脚  */
#define KEY5_PIN 17 /* 键盘第五个按键的引脚 */

// 定义RX引脚的编号
// #define RX_PIN 18
#define RX_PIN 27

// 定义TX引脚的编号
// #define TX_PIN 26
#define TX_PIN 14

// 定义SDA引脚编号
// #define SDA_PIN 23
#define SDA_PIN 22
// 定义SCL引脚编号
// #define SCL_PIN 19
#define SCL_PIN 23

// 菜单状态
#define MAIN_MENU 0
#define PWM_MENU 1
#define BUS_MENU 2
#define PWM_SET_CENTER 3
#define PWM_CONTROL_ANGLE 4
#define BUS_SET_ID 5
#define BUS_CONTROL_MOVE 6
#define BUS_SET_CENTER 7
#define BUS_SET_OFFSET 8

// 菜单状态

// 创建初始菜单状态
static int menuState = MAIN_MENU;
static int MidState = PWM_MENU;
static int ActionState = 0;
const float beta = 0.25;
static int angle = 0;
static int previous_angle = 0;
static int id = 1;
static uint8_t setID = 1;

// 电量显示区域长和宽，坐标位置定义
#define BAT_X 100
#define BAT_Y 2
#define BAT_WIDTH 28
#define BAT_HEIGHT 14

const int mainmenu[] = {PWM_MENU, BUS_MENU};
// PWM模式舵机菜单
const int pwmmenu[] = {
    PWM_CONTROL_ANGLE,
    PWM_SET_CENTER};
// 总线舵机模式菜单
const int busmenu[] = {
    BUS_CONTROL_MOVE,
    BUS_SET_CENTER,
    BUS_SET_OFFSET,
    BUS_SET_ID};

// 菜单栏列表
const char *mainMenuItems[] = {"1.PWM Mode", "2.Bus Servo Mode"};
const char *PWM_menuItems[] = {"1.Control Angle", "2.Set Center", "3.Back"};
const char *BUS_menuItems[] = {"1.Control Move", "2.Set Center", "3.Set Offset", "4.Set ID", "5.Back"};
// 每种菜单栏数量
const int mainMenuItemCount = sizeof(mainMenuItems) / sizeof(mainMenuItems[0]);
const int PWM_menuItemCount = sizeof(PWM_menuItems) / sizeof(PWM_menuItems[0]);
const int BUS_menuItemCount = sizeof(BUS_menuItems) / sizeof(BUS_menuItems[0]);

volatile int currentSelection = 0; // 当前选中的菜单项

// 初始化OLED的I2C地址
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// 总线舵机设置
LobotBusServo BusServo(Serial1);
Servo servo1; // create servo object to control a servo
Servo servo2; // create servo object to control a servo

void displayMenu();
void checkButtonPress();
// 功能函数，不同模式对应不同功能
void ServoContorl();
void setup()
{
  // 初始化硬件串口2
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN); // 使用GPIO9作为RX，GPIO10作为TX
                                                     // 假设按钮连接到Arduino的数字引脚5
                                                     // WS2812B初始化
  // FastLED.addLeds<WS2812B, WS2812_PIN, RGB>(leds, NUM_LEDS); // GRB ordering is typical

  pinMode(KEY1_PIN, INPUT_PULLUP); // 设置为上拉输入
                                   // Allow allocation of all timers
  pinMode(KEY2_PIN, INPUT_PULLUP);
  // 增加第三个菜单，用于确定对应的菜单功能
  pinMode(KEY3_PIN, INPUT_PULLUP);

  pinMode(KEY4_PIN, INPUT_PULLUP);
  // 增加第三个菜单，用于确定对应的菜单功能
  pinMode(KEY5_PIN, INPUT_PULLUP);

  // 启动所有定时器
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // 舵机频率和引脚配置
  servo1.setPeriodHertz(50);            // standard 50 hz servo
  servo1.attach(SERVO1_PIN, 500, 2500); // attaches the servo on pin 18 to the servo object
  servo2.setPeriodHertz(50);            // standard 50 hz servo
  servo2.attach(SERVO2_PIN, 500, 2500); // attaches the servo on pin 18 to the servo object

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  // 初始化OLED显示屏'
  Wire.begin(SDA_PIN, SCL_PIN, 4000000UL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
  // 清空屏幕以开始
  display.clearDisplay();
  display.setTextSize(1);              // 设置文本大小
  display.setTextColor(SSD1306_WHITE); // 设置文本颜色为白色
  display.setCursor(0, 0);             // 设置文本起始位置
  display.display();                   // 显示初始化屏幕
  // oled初始化结束
  // 显示图案
  // Clear the buffer
  // Clear the buffer
  for (int i = 100; i < (LOGO_NUM - 500); i++)
  {
    display.display();      // 显示当前的图像
    display.clearDisplay(); // 清空屏幕
    // testanimate(badapple[0], LOGO_WIDTH, LOGO_HEIGHT); // Animate bitmaps
    display.drawBitmap((SCREEN_WIDTH - LOGO_WIDTH) / 2, 0, badapple[i], LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);
    delay(50);
  }
  delay(500);
}

void loop()
{
  // Serial.println(String(3 * 3.3 * analogRead(ADC_BAT) / 4095.0 + 0.40f) + " " + String(analogRead(ADC_P)) + " " + String(digitalRead(buttonPin)));
  // Serial.println(analogRead(ADC_P));
  // Serial.println(digitalRead(buttonPin));
  // Serial.println();
  checkButtonPress(); // 检查按钮是否被按下
  displayMenu();      // 显示菜单
  ServoContorl();     // 模式动作控制

  // int angle = map(analogRead(ADC_P), 0, 4095, 500, 2500);
  // servo1.writeMicroseconds(angle);
  // servo2.writeMicroseconds(angle);
  // int pos = map(analogRead(ADC_P), 0, 4095, 0, 1000);
  // BusServo.Move(1, pos, 100);
  delay(30); // 简单的延时，减少CPU使用率
  // Serial.println(String(ActionState) + " " + String(menuState) + " " + String(MidState));
}

void displayMenu()
{
  display.clearDisplay();
  // 角度显示
  if (ActionState == PWM_CONTROL_ANGLE)
  {
    display.setCursor(2, 0); // 设置文本起始位置
    display.setTextSize(1);  // 设置文本大小
    display.println("Angle:" + String(angle));
  }
  // 总线模式下角度和ID显示
  else if (ActionState == BUS_CONTROL_MOVE)
  {
    display.setCursor(2, 0);
    display.setTextSize(1);
    display.println("Angle:" + String(angle));
    // 显示ID号
    display.setCursor(2, 8);
    // 如果读取失败，则显示读取失败
    if (BusServo.ReadID() == -2048)
    {
      display.print("ID:Fail");
    }
    else
    {
      display.print("ID:" + String(BusServo.ReadID()));
    }
  }

  // 总线模式下角度和设置ID显示
  else if (ActionState == BUS_SET_ID)
  {
    display.setCursor(2, 0);
    display.setTextSize(1);
    display.println("Angle:" + String(angle));
    // 显示ID号
    display.setCursor(2, 8);
    // 如果读取失败，则显示读取失败
    if (BusServo.ReadID() == -2048)
    {
      display.print("ID:Fail");
    }
    else
    {
      display.print("ID:" + String(BusServo.ReadID()));
    }
    display.setCursor(50, 8);
    display.print("SetID:" + String(setID));
  }
  else
  {
    display.setCursor(2, 0); // 设置文本起始位置
    display.setTextSize(1);  // 设置文本大小
    display.println("Angle:" + String(angle));
  }
  // 3 * 3.3 * analogRead(ADC_BAT) / 4095.0 + 0.40f
  // 电量显示
  float voltage = (11.0 * (3.3 * (analogRead(ADC_BAT) / 4095.0f) + 0.15) - 6.4f) / 2.0f;
  // Serial.println(3.3 * (analogRead(ADC_BAT) / 4095.0f) + 0.15);
  // Serial.println(voltage);
  voltage = voltage > 0 ? voltage : 0;
  display.drawRect(BAT_X, BAT_Y, BAT_WIDTH, BAT_HEIGHT, SSD1306_WHITE);
  display.fillRect(BAT_X, BAT_Y, int16_t(BAT_WIDTH * voltage) < BAT_WIDTH ? int16_t(BAT_WIDTH * voltage) : BAT_WIDTH, BAT_HEIGHT, SSD1306_WHITE);
  if (voltage == 0)
  {
    display.setCursor(BAT_X + 2, 5); // 设置文本起始位置
    display.setTextSize(1);          // 设置文本大小
    display.println("LOW");
  }

  // 分割线绘制
  // display.drawLine(0, 17, 127, 17, SSD1306_WHITE); // 画线，颜色为白色
  display.drawRect(0, 17, 128, 47, SSD1306_WHITE);

  display.setCursor(0, 19); // 设置文本起始位置
  display.setTextSize(1);   // 设置文本大小
                            /*
                              选择菜单栏内容显示
                            */

  switch (menuState)
  {
  case MAIN_MENU:
    for (int i = 0; i < mainMenuItemCount; ++i)
    {
      // 显示菜单项
      String line = (i == currentSelection) ? "*" : " ";
      line += mainMenuItems[i];
      if (i == currentSelection)
      {
      }
      display.println(line);
    }
    break;
  case PWM_MENU:
    for (int i = 0; i < PWM_menuItemCount; ++i)
    {
      // 显示菜单项
      String line = (i == currentSelection) ? "*" : " ";
      line += PWM_menuItems[i];

      display.println(line);
    }
    break;
  case BUS_MENU:
    for (int i = 0; i < BUS_menuItemCount; ++i)
    {
      // 显示菜单项
      String line = (i == currentSelection) ? "*" : " ";
      line += BUS_menuItems[i];
      display.println(line);
    }
    break;
  }

  // 画线框,显示接线顺序  S + -
  int boxX = 116;     // 线框左上角X坐标
  int boxY = 24;      // 线框左上角Y坐标
  int boxWidth = 10;  // 线框宽度
  int boxHeight = 34; // 线框高度
  display.drawRect(boxX, boxY, boxWidth, boxHeight, SSD1306_WHITE);
  display.setCursor(boxX + 2, boxY + 2);  // 设置文本开始位置
  display.println("S");                   // 打印文本
  display.setCursor(boxX + 2, boxY + 12); // 设置文本开始位置
  display.println("+");                   // 打印文本
  display.setCursor(boxX + 2, boxY + 22); // 设置文本开始位置
  display.println("-");                   // 打印文本

  display.display();
}

void checkButtonPress()
{
  // down 按键
  if (digitalRead(KEY2_PIN) == LOW)
  {
    // Serial.println("Button pressed");
    // 按钮被按下
    // delay(2); // 简单的消抖
    while (digitalRead(KEY2_PIN) == LOW)
    {
      // 等待按钮释放
    }
    // 处理按钮按下事件

    switch (menuState)
    {
    case MAIN_MENU:
      currentSelection = (currentSelection + 1) % mainMenuItemCount;
      MidState = mainmenu[currentSelection];
      break;
    case PWM_MENU:
      currentSelection = (currentSelection + 1) % PWM_menuItemCount;
      MidState = pwmmenu[currentSelection];
      previous_angle = previous_angle = map(analogRead(ADC_P), 800, 3600, 500, 2500);
      break;
    case BUS_MENU:
      currentSelection = (currentSelection + 1) % BUS_menuItemCount;
      MidState = busmenu[currentSelection];
      previous_angle = previous_angle = map(analogRead(ADC_P), 800, 3600, 500, 2500);
      break;
    default:
      break;
    }
  }
  // up button
  if (digitalRead(KEY1_PIN) == LOW)
  {
    // Serial.println("Button pressed");
    // 按钮被按下
    // delay(2); // 简单的消抖
    while (digitalRead(KEY1_PIN) == LOW)
    {
      // 等待按钮释放
    }
    // 处理按钮按下事件

    switch (menuState)
    {
    case MAIN_MENU:
      currentSelection = (currentSelection - 1 + mainMenuItemCount) % mainMenuItemCount;
      MidState = mainmenu[currentSelection];
      break;
    case PWM_MENU:
      currentSelection = (currentSelection - 1 + PWM_menuItemCount) % PWM_menuItemCount;
      MidState = pwmmenu[currentSelection];
      previous_angle = map(analogRead(ADC_P), 800, 3600, 500, 2500);
      break;
    case BUS_MENU:
      currentSelection = (currentSelection - 1 + BUS_menuItemCount) % BUS_menuItemCount;
      MidState = busmenu[currentSelection];
      previous_angle = map(analogRead(ADC_P), 800, 3600, 0, 1000);

    default:
      break;
    }
  }

  // 功能选择按键
  if (digitalRead(KEY3_PIN) == LOW)
  {
    while (digitalRead(KEY3_PIN) == LOW)
    {
      // 等待按钮释放
    }
    if (menuState == MAIN_MENU)
    {
      menuState = MidState;
      // 清空当前菜单编号
      currentSelection = 0;
      ActionState = 0;
    }
    // 当处于PWM菜单时,选择对应的工作模式
    else if (menuState == PWM_MENU)
    {
      // 退出PWM菜单
      if (currentSelection == (PWM_menuItemCount - 1))
      {
        menuState = MAIN_MENU;
        currentSelection = 0;
        // 清空动作模式
        ActionState = 0;
        MidState = PWM_MENU;
      }
      else
      {
        ActionState = pwmmenu[currentSelection];
      }
    }
    // 当处于总线舵机菜单时,选择对应的工作模式
    else if (menuState == BUS_MENU)
    {
      // 如果选择退出总线舵机菜单
      if (currentSelection == (BUS_menuItemCount - 1))
      {
        menuState = MAIN_MENU;
        currentSelection = 0;
        // 清空动作模式
        ActionState = 0;
        MidState = PWM_MENU;
      }
      else
      {
        ActionState = busmenu[currentSelection];
        if (ActionState == BUS_SET_ID)
        {
          BusServo.SetID(254, setID);
        }
      }
    }
  }

  // setID值改变按键 4 5 实现加减
  if (digitalRead(KEY4_PIN) == LOW)
  {
    while (digitalRead(KEY4_PIN) == LOW)
    {
      // 等待按钮释放
    }
    if (menuState == BUS_MENU)
    {

      setID = setID - 1;
      if (setID < 1)
      {
        setID = 1;
      }
    }
  }
  if (digitalRead(KEY5_PIN) == LOW)
  {
    while (digitalRead(KEY5_PIN) == LOW)
    {
      // 等待按钮释放
    }
    if (menuState == BUS_MENU)
    {

      setID = setID + 1;
      if (setID > 254)
      {
        setID = 254;
      }
    }
  }
}

void ServoContorl()
{
  switch (ActionState)
  {
    // PWM模式一键居中
  case PWM_SET_CENTER:
    angle = 1500;
    servo1.writeMicroseconds(angle);
    servo2.writeMicroseconds(angle);
    break;
    // PWM模式角度控制
  case PWM_CONTROL_ANGLE:

    angle = map(analogRead(ADC_P), 800, 3600, 500, 2500);
    angle = int(beta * angle + (1 - beta) * previous_angle);
    previous_angle = angle;
    // 限制angle范围
    angle = constrain(angle, 500, 2500);
    Serial.println(angle);
    servo1.writeMicroseconds(angle);
    servo2.writeMicroseconds(angle);
    break;
    // 总线模式 ID设置
  case BUS_SET_ID:
    // 总线舵机模式
    BusServo.SetID(254, setID);
    break;
    // 总线模式运动控制
  case BUS_CONTROL_MOVE:
    angle = map(analogRead(ADC_P), 800, 3600, 0, 1000);
    // 限定angle的值在0-1000之间
    angle = int(beta * angle + (1 - beta) * previous_angle);
    previous_angle = angle;
    angle = constrain(angle, 0, 1000);
    BusServo.Move(254, angle, 15);
    break;
  // 总线模式一键居中
  case BUS_SET_CENTER:
    BusServo.Move(254, 500, 15);
    break;
  // 总线模式偏差设置
  case BUS_SET_OFFSET:
    break;
    // 退出菜单
  }
}

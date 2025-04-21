#include <AccelStepper.h>
#include <TMCStepper.h>

#include <SPI.h>
#include <SD.h>
#define CS_PIN 17  // Пин Chip Select (CS)

#include <Servo.h>

#include <Adafruit_GFX.h>
#include <Adafruit_GrayOLED.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>

#include <Wire.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(128, 32, &Wire, -1);
#define ENC_SW 3
#define ENC_DT 15
#define ENC_CLK 14

// Конфигурация моторов
#define STEP_PIN_1 6
#define DIR_PIN_1 7
#define STEP_PIN_2 12
#define DIR_PIN_2 13
#define STEP_PIN_3 11
#define DIR_PIN_3 10
#define TMC_RX 5
#define TMC_TX 4
#define DRIVER_ADDRESS 0b00
#define R_SENSE 0.11f  
uint16_t speed = 5;
uint16_t MICROSTEPS = 16;
uint32_t ACCELERATION = 20000;
uint32_t MAXSPEED = 1000;
uint8_t TOFF = 5; // задерка включения 0-15
uint16_t RMC_DEFAULT = 800;// макс ток мА
float HOLD_I = 0.5; // ток удержания - доля RMC_DEFAULT 0.0-1.0 
uint8_t SEUP = 2; // шаг увеличения тока при обнаружении нагрузки 0-3
uint8_t SEDN = 1; // шаг уменьшения тока при обнаружении нагрузки 0-3
bool SPREAD_CYCLE = false;  // true - точный шумный(pread_cycle); false - тихий, ны высок скорости не стабилен(stealthchop)
uint8_t STALLGUARD = 150;  // 0-255 чувствительность, защита от пропуска шагов (работает в spead_cycle)
uint8_t PWM_FREQ = 0; // частота шим 0 - 35кГц; 1 - 2.5кГц; 2 - 39кГц; 3 - 100кГц
bool AUTO_PWN = true;  // плановсть шим
uint8_t PWM_GRAD = 10; // градиент шим влияет на плавность 0-255
bool PWM_AUTOGRAD = true; // автоматическая настройки градиента шим
uint8_t PWM_OFS = 30; // смещение шим 
uint16_t STEALTHCHOP_OFF = 200; // скорость выше которой отключается тихий режим шаг/с (если SPREAD_CYCLE true)
uint16_t COOLSTEP = 0; // скорость ниже которой активируется coolstep (автоматическое снижение тока)
uint8_t HIGH_TEMP = 100; // порог температуры для снижения тока 0-255
bool FILTER_STEP = false; // фильтрация степ сиганала true/false

// SerialUART tmcSerial(4, 5); 
TMC2209Stepper driver(&Serial2, R_SENSE, DRIVER_ADDRESS);
// TMC2209Stepper driver(TMC_RX, TMC_TX, R_SENSE, DRIVER_ADDRESS); // RX 5 TX 4
AccelStepper motor1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper motor2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);
AccelStepper motor3(AccelStepper::DRIVER, STEP_PIN_3, DIR_PIN_3);

void driverInit() {
  Serial2.setTX(4);
  Serial2.setRX(5);
  Serial2.begin(115200);
  while(!Serial2){}
  driver.begin();
  driver.toff(TOFF);
  driver.hold_multiplier(HOLD_I);
  driver.SGTHRS(STALLGUARD);
  driver.TCOOLTHRS(COOLSTEP);
  driver.TPWMTHRS(STEALTHCHOP_OFF);
  driver.pwm_freq(PWM_FREQ);
  driver.seup(SEUP);
  driver.sedn(SEDN);
  driver.pwm_grad(PWM_GRAD);
  driver.pwm_ofs(PWM_OFS);
  driver.pwm_autograd(PWM_AUTOGRAD);
  driver.rms_current(RMC_DEFAULT);
  driver.microsteps(MICROSTEPS);
  driver.pwm_autoscale(AUTO_PWN); 
  driver.en_spreadCycle(SPREAD_CYCLE); 

  Serial.println("tmc uart succses");
  Serial.print("GCONF: 0x"); Serial.println(driver.GCONF(), HEX);
  Serial.print("IOIN: 0x"); Serial.println(driver.IOIN(), HEX);
  Serial.print("CURRENT: "); Serial.println(driver.rms_current());
}

#include <math.h>

// Структура для хранения частот моторов (шагов/сек)
class MotorFreq {
public:
  float motor1;
  float motor2;
  float motor3;
  MotorFreq(){}
};

// Параметры робота
const float WHEEL_RADIUS = 0.029f;  // Радиус колеса (метры)
const float ROBOT_RADIUS = 0.1055f; // 84.5+8+13

const float COS_120 = -0.5f;           // cos(120°)
const float SIN_120 = 0.86602540378f;   // sin(120°)



float updateMotors(float direction_deg, float speed_mps = 0.5, float rotation_radps = 0) {
  float dir_rad = direction_deg * M_PI / 180.0f;
  float vx = speed_mps * cos(dir_rad);
  float vy = speed_mps * sin(dir_rad);
  float wz = rotation_radps;

  MotorFreq freq;
  freq.motor1 = ( -vx * 0.0f    + vy * 1.0f    + wz * ROBOT_RADIUS ) / WHEEL_RADIUS;  // Колесо 1 (90°)
  freq.motor2 = ( -vx * SIN_120 + vy * COS_120 + wz * ROBOT_RADIUS ) / WHEEL_RADIUS;  // Колесо 2 (210°)
  freq.motor3 = ( -vx * -SIN_120 + vy * COS_120 + wz * ROBOT_RADIUS ) / WHEEL_RADIUS;  // Колесо 3 (330°)

  motor1.setSpeed(freq.motor1*speed*MICROSTEPS);  
  motor2.setSpeed(freq.motor2*speed*MICROSTEPS);
  motor3.setSpeed(freq.motor3*speed*MICROSTEPS);
  return max(freq.motor1*speed, max(freq.motor2*speed, freq.motor3*speed));
  // Serial.print(freq.motor1);
  // Serial.print(" - ");
  // Serial.print(freq.motor2);
  // Serial.print(" - ");
  // Serial.println(freq.motor3);
}

void hardResetStepper(AccelStepper &stepper) {
  stepper.setCurrentPosition(0);
    
    // 4. Принудительная синхронизация
  stepper.moveTo(0);
  // stepper.run();
  stepper.runToPosition(); 
    // // stepper.stop();
    // stepper.setCurrentPosition(0);
    // // stepper.moveTo(0);  // Принудительно обнуляем цель
    // stepper.runToPosition();  // Дожимаем до нуля (если нужно)
    // // stepper.disableOutputs();  // Отключаем питание (если драйвер поддерживает)
    // delay(100);  // Даем время на "успокоение" драйвера
    // // stepper.enableOutputs();  // Включаем обратно
}

// int laststep1, laststep2, laststep3;

void driveto(float direction_deg, float distance, float speed_mps = 0.5, float rotation_radps = 0, bool delayneed = true, bool stopneed = true) {
  // uint64_t st = millis();
  // while (updateMotors(direction_deg, speed_mps, rotation_radps)*(millis()-st)/1000.0 < distance){
  //   motor1.runSpeed(); j
  //   motor2.runSpeed();
  //   motor3.runSpeed();
  // }
  // motor1.stop();
  // motor2.stop();
  // motor3.stop();
  float dir_rad = direction_deg * M_PI / 180.0f;
  float vx = speed_mps * cos(dir_rad);
  float vy = speed_mps * sin(dir_rad);
  float wz = rotation_radps;

  MotorFreq freq;
  MotorFreq dis;

  freq.motor1 = ( -vx * 0.0f    + vy * 1.0f    + wz * ROBOT_RADIUS ) / WHEEL_RADIUS;  // Колесо 1 (90°)
  freq.motor2 = ( -vx * SIN_120 + vy * COS_120 + wz * ROBOT_RADIUS ) / WHEEL_RADIUS;  // Колесо 2 (210°)
  freq.motor3 = ( -vx * -SIN_120 + vy * COS_120 + wz * ROBOT_RADIUS ) / WHEEL_RADIUS;  // Колесо 3 (330°)

  motor1.setMaxSpeed(freq.motor1*speed*MICROSTEPS);  
  motor2.setMaxSpeed(freq.motor2*speed*MICROSTEPS);
  motor3.setMaxSpeed(freq.motor3*speed*MICROSTEPS);

  motor1.setAcceleration(freq.motor1*30000*WHEEL_RADIUS);  
  motor2.setAcceleration(freq.motor2*30000*WHEEL_RADIUS);
  motor3.setAcceleration(freq.motor3*30000*WHEEL_RADIUS);

  dis.motor1 = ( -vx * 0.0f    + vy * 1.0f    + wz * ROBOT_RADIUS ) * distance;  // Колесо 1 (90°)
  dis.motor2 = ( -vx * SIN_120 + vy * COS_120 + wz * ROBOT_RADIUS ) * distance;  // Колесо 2 (210°)
  dis.motor3 = ( -vx * -SIN_120 + vy * COS_120 + wz * ROBOT_RADIUS ) * distance;  // Колесо 3 (330°)

  motor1.setCurrentPosition(0);
  motor2.setCurrentPosition(0);
  motor3.setCurrentPosition(0);

  motor1.moveTo(dis.motor1*speed*MICROSTEPS);  
  motor2.moveTo(dis.motor2*speed*MICROSTEPS);
  motor3.moveTo(dis.motor3*speed*MICROSTEPS);

  Serial.print(dis.motor1);
  Serial.print(" ");
  Serial.print(dis.motor2);
  Serial.print(" ");
  Serial.println(dis.motor3);
  // laststep1 = dis.motor1*speed*MICROSTEPS;
  // laststep2 = dis.motor2*speed*MICROSTEPS;
  // laststep3 = dis.motor3*speed*MICROSTEPS;
  while (!(motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0 && motor3.distanceToGo() == 0)) {
    motor1.run();
    motor2.run();
    motor3.run();
  }

  // motor1.runToPosition();
  // motor2.runToPosition();
  // motor3.runToPosition();
  if (stopneed){
    motor1.stop();
    motor2.stop();
    motor3.stop();
  }
  if (delayneed) delay(200);

  

  // motor1.runToPosition();
  // motor2.runToPosition();
  // motor3.runToPosition();

  // hardResetStepper(motor1);
  // hardResetStepper(motor2);
  // hardResetStepper(motor3);
  
}




// Функция чтения и выполнения G-кода
void readAndExecuteGCode(const char* filename) {
  File file = SD.open(filename);
  
  if (!file) {
    Serial.println("❌ Ошибка открытия файла!");
    return;
  }

  Serial.println("🔎 Чтение G-кода...");

  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();  // Удаляем лишние пробелы и \r\n
    
    if (line.startsWith("G1")) {  // Если это команда перемещения (G1 X Y)
      processG1Command(line);    // Обрабатываем её
    }
    // Можно добавить другие G-коды (G0, G28 и т.д.)
  }

  file.close();
  Serial.println("✅ G-код выполнен!");
}

// Разбор строки G1 X Y и вызов driveTo(угол, длина)
void processG1Command(String gcodeLine) {
  float x = 0, y = 0;
  
  // Ищем X и Y в строке (пример: "G1 X10.5 Y20.3")
  int xPos = gcodeLine.indexOf('X');
  int yPos = gcodeLine.indexOf('Y');
  
  if (xPos != -1) {
    x = gcodeLine.substring(xPos + 1).toFloat();
  }
  if (yPos != -1) {
    y = gcodeLine.substring(yPos + 1).toFloat();
  }

  Serial.print("🔄 G1 Команда: X=");
  Serial.print(x);
  Serial.print(", Y=");
  Serial.println(y);

  // Преобразуем X,Y в угол и длину
  float angle = atan2(y, x) * 180 / PI;  // Угол в градусах (0-360)
  float distance = sqrt(x * x + y * y);  // Расстояние (гипотенуза)

  Serial.print("➡️ driveTo: Угол=");
  Serial.print(angle);
  Serial.print("°, Длина=");
  Serial.println(distance);

  driveTo(angle, distance);  // Вызываем функцию движения
}




Servo serv;

void setup() {
  Serial.begin(115200);
  // while(!Serial){}
  pinMode(ENC_SW, INPUT);
  pinMode(ENC_DT, INPUT);
  pinMode(ENC_CLK, INPUT);
  Wire.setSDA(8);  // GP2 на Pico
  Wire.setSCL(9);  // GP3 на Pico
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED не найден!");
    while (1){}
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  // display.setCursor(0, 0);
  // display.println("OLED");
  // display.display();
  // Serial.println("print");
  // delay(1000);
  // Serial.println("print2");
  // display.clearDisplay();
  // display.display();

  display.setCursor(20, 20);
  display.println("program1");
  display.drawFastHLine(10, 30, 108, 1);
  display.display();
  Serial.println("print3");
  

  
  delay(500);
  if (!SD.begin(CS_PIN)) {
    Serial.println("❌ SD-карта НЕ найдена!");
    // return;
  } else {
    Serial.println("✅ SD-карта найдена!");
  }
    // Проверяем, есть ли файлы на карте
  File root = SD.open("/");
  if (!root) {
    Serial.println("❌ Ошибка открытия корневой папки!");
    // return;
    while (1){}
  }
  // delay(2000); 
  // while (!Serial){}
  // Serial.println("serial monitor succses");
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(STEP_PIN_3, OUTPUT);
  pinMode(DIR_PIN_3, OUTPUT);
  pinMode(TMC_TX, OUTPUT);
  pinMode(TMC_RX, INPUT);
  pinMode(20, OUTPUT);
  serv.attach(20);
  serv.write(90);
  driverInit();

  delay(500);
  
  motor1.setMaxSpeed(MAXSPEED*MICROSTEPS);
  motor1.setAcceleration(ACCELERATION*MICROSTEPS);
  // motor1.setSpeed(200*MICROSTEPS);
  motor2.setMaxSpeed(MAXSPEED*MICROSTEPS);
  motor2.setAcceleration(ACCELERATION*MICROSTEPS);
  // motor2.setSpeed(200*MICROSTEPS);
  motor3.setMaxSpeed(MAXSPEED*MICROSTEPS);
  motor3.setAcceleration(ACCELERATION*MICROSTEPS);
  // motor3.setSpeed(200*MICROSTEPS);

  readAndExecuteGCode("gcode.txt");
}
int num = 0;
void enc() {
    if (digitalRead(ENC_CLK) == 0 && digitalRead(ENC_DT) == 1){
      num++;
      while (!(digitalRead(ENC_CLK) == 1 && digitalRead(ENC_DT) == 1)){}
    } else if (digitalRead(ENC_CLK) == 1 && digitalRead(ENC_DT) == 0){
      num--;
      while (!(digitalRead(ENC_CLK) == 1 && digitalRead(ENC_DT) == 1)){}
    } 
}

bool working = false;
int tasknum = 0;

void drawCircle(float radius, int num_segments = 50) {
    const float angle_step = 2 * M_PI / num_segments;  // Угловой шаг в радианах
    
    for (int i = 0; i < num_segments; i++) {
        float angle = i * angle_step;  // Текущий угол в радианах
        float step_length = 2 * radius * sin(angle_step / 2);  // Длина шага
        
        // Переводим угол в градусы и отправляем команду
        float angle_deg = angle * 180.0 / M_PI;
        driveto(angle_deg, step_length, 0.5f, 0, false, false);
        
        // Небольшая задержка (если нужно)
        delay(4);
    }
}

void drawSpiral(float outer_radius, float inner_radius, int num_loops = 5, int steps_per_loop = 50) {
    float total_steps = num_loops * steps_per_loop;
    float radius_step = (outer_radius - inner_radius) / total_steps;
    float angle_step = 2 * M_PI / steps_per_loop;  // Угловой шаг за один виток
    
    float current_radius = outer_radius;
    
    for (int i = 0; i < total_steps; i++) {
        float theta = i * angle_step;  // Текущий угол
        float step_length = 2 * current_radius * sin(angle_step / 2);  // Длина шага
        
        // Переводим угол в градусы и отправляем команду
        float angle_deg = theta * 180.0 / M_PI;
        driveto(angle_deg, step_length, 0.5f, 0, false, false);
        
        // Уменьшаем радиус
        current_radius -= radius_step;
        
        // Небольшая задержка (опционально)
        delay(4);
    }
}

void drawEllipse(float a, float b, int segments = 400) {
    float angle_step = 2 * M_PI / segments;
    
    for (int i = 0; i <= segments; i++) {
        float theta = i * angle_step;
        
        // Координаты точки на эллипсе
        float x = a * cos(theta);
        float y = b * sin(theta);
        
        // Координаты следующей точки
        float next_theta = (i + 1) * angle_step;
        float next_x = a * cos(next_theta);
        float next_y = b * sin(next_theta);
        
        // Вычисляем вектор движения
        float dx = next_x - x;
        float dy = next_y - y;
        
        // Длина шага
        float step_length = sqrt(dx*dx + dy*dy);
        
        // Угол направления (в градусах)
        float angle_deg = atan2(dy, dx) * 180.0 / M_PI;
        if (angle_deg < 0) angle_deg += 360;
        
        // Двигаемся
        driveto(angle_deg, step_length, 0.5f, 0, false, false);
        
        // Небольшая задержка
        delay(10);
    }
}

void drawHumanLikeStar(float size) {
    // Углы для 5-конечной звезды (шаг 144°)
    float star_angles[] = {0, 144, 288, 72, 216}; 
    
    for (int i = 0; i < 5; i++) {  // 6 шагов (5 линий + замыкание)
        float angle_deg = star_angles[i];
        driveto(angle_deg, size);
        delay(100);  // Пауза между линиями (опционально)
    }
}

void loop() {
  
}
char ver[ ] = "147pp04";

//#define SI_OVERCLOCK 750000000L
#define ENCODER_OPTIMIZE_INTERRUPTS

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define OLED_I2C_ADRESS 0x3C // Display I2c adress

#define max_number_of_bands	30 // Максимальное оличество диапазонов.
#define max_bpf_set 7 // 0-7 каналы управления BPF 74hc595.
#define Si_Xtall_Freq 27000000UL // Частота кварца si5351, Гц.
#define si_cload SI5351_CRYSTAL_LOAD_8PF// 
#define lo_max_freq 9900000UL // Максимальная частота опоры, Гц.
#define lo_min_freq 1000000UL // Минимальная частота опоры, Гц.
#define min_hardware_freq 10 // *100KHz Минимальный железный предел частоты диапазона VFO
#define max_hardware_freq 300 // *100KHz Максимальный железный предел частоты диапазона VFO
#define ONE_WIRE_BUS 14 // Порт датчика температуры
#define myEncBtn 4 // Порт нажатия кноба.
#define mypowerpin 15 // Порт показометра мощности. А0
#define mybattpin 21 // Порт датчика АКБ А1
#define txpin 17 //Порт датчика ТХ.

#define dataPin 11 // DS 74hc595 controller pin
#define latchPin 12 // ST_CP 74hc595 controller pin 
#define clockPin 13 //  SH_CP 74hc595 controller pin


#include "Adafruit_SSD1306.h" // Use version 1.2.7!!!
#include "si5351a.h"
#include "Wire.h"
#include "Encoder.h"
#include "Eeprom24C32_64.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "DS1307RTC.h"

//Общие настройки
struct general_set {
  byte stp_set = 3; //Начальный шаг настройки.
  byte band_set = 0; // Стартовый диапазон.
  byte number_of_bands_set = 0; // Количество диапазонов.
  unsigned long lo_freq_set = 8867270UL; // Начальная частота опоры USB при первом включении.
  int Si_Xtall_calFreq_set = 5850; // Начальная частота калибровки кварца, Гц.
  byte batt_cal_set = 208; // Начальная калибровка вольтметра.
  boolean reverse_encoder_set = false; //Реверс энкодера.
  byte mem_enc_div = 4; // Делитель импульсов энкодера
  int temp_cal = -2; //калибровка датчика температуры
} general_setting;


#define stp general_setting.stp_set
#define band general_setting.band_set
#define number_of_bands general_setting.number_of_bands_set
#define lo_freq general_setting.lo_freq_set
#define Si_Xtall_calFreq general_setting.Si_Xtall_calFreq_set
#define batt_cal general_setting.batt_cal_set
#define reverse_encoder general_setting.reverse_encoder_set
#define mem_enc_div general_setting.mem_enc_div
#define temp_cal general_setting.temp_cal

// Диапазонные настройки
struct band_set {
  unsigned long vfo_freq_set = 7100000UL; // Начальная частота VFO при первом включении.
  unsigned int min_freq_set = 15; // *100KHz Минимальный предел частоты диапазона VFO.
  unsigned int max_freq_set = 80; // *100KHz Максимальный предел частоты диапазона VFO.
  byte bpf_set = 0; // Канал BPF
} band_setting;
#define vfo_freq band_setting.vfo_freq_set
#define min_freq band_setting.min_freq_set
#define max_freq band_setting.max_freq_set
#define bpf_set band_setting.bpf_set
//
//
//

byte menu = 0; //Начальное положение меню.
unsigned int arraystp[] = {1, 10, 50, 100, 1000, 10000}; //шаги настройки * 10 герц.

byte enc_div;
byte mypower;
byte mybatt;
int8_t temperature;
int screenstep = 1000;

long oldPosition  = 0;
boolean actencf = false;
boolean txen = false;
int RXifshift = 0;
boolean knobup = true;
boolean exitmenu = false;
boolean reqtemp = false;
boolean timesetup = false;
boolean actfmenuf = false;
boolean bpfUpdate = false;

unsigned long previousdsp = 0;
unsigned long previoustemp = 0;
unsigned long previoustime = 0;
unsigned long knobMillis = 0;
unsigned long actenc = 0;


static Eeprom24C32_64 AT24C32(0x50);
Si5351 si;
Encoder myEnc(2, 3); //порты подключения енкодера.
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
tmElements_t tm;


void setup() {
  Serial.begin(57600);
  pinMode(myEncBtn, INPUT);
  pinMode(mypowerpin, INPUT);
  digitalWrite(myEncBtn, HIGH);

  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  digitalWrite(latchPin, LOW);
  digitalWrite(clockPin, LOW);
  digitalWrite(dataPin, LOW);

  analogReference(INTERNAL);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADRESS);
  display.clearDisplay();
  display.display();
  sensors.begin();
  memread();
  enc_div = mem_enc_div;
  si5351init();
  si5351correction();
  vfosetup();
  battmeter();
  powermeter();
  tempsensor ();
  timenow ();
  versionprint ();
  mainscreen();
}

void loop() { // Главный цикл
  pushknob();
  readencoder();
  txsensor();
  bpfset();
  if (!menu) {
    if (txen) screenstep = 100;
    else screenstep = 1000;
    if (millis() - previousdsp > screenstep) {
      storetomem();
      battmeter();
      powermeter();
      tempsensor();
      timenow();
      mainscreen();
      previousdsp = millis();
    }
  }
}

void timenow () {
  if (timesetup) {
    tm.Second = 0;
    RTC.write(tm);
    timesetup = false;
  }
  else {
    if (millis() - previoustime > 1000 || !previoustime) {
      previoustime = millis();
      if (!RTC.read(tm)) {
        if (RTC.chipPresent()) {
          tm.Hour = 0;  tm.Minute = 0; tm.Second = 0;
          RTC.write(tm);
        }
      }
    }
  }
}

void tempsensor () {
  if (millis() - previoustemp > 5000 && !reqtemp) {
    sensors.setWaitForConversion(false);
    sensors.requestTemperatures();
    sensors.setWaitForConversion(true);
    reqtemp = true;
  }
  if (millis() - previoustemp > 8000 && reqtemp) {
    temperature = temp_cal + (int8_t)(0.5 + sensors.getTempCByIndex(0));
    previoustemp = millis();
    reqtemp = false;
  }
}

void txsensor () {
  boolean txsens = digitalRead(txpin);
  //Если радио на приеме и нажали ТХ
  if (txsens && !txen) {
    txen = true;
    vfosetup();
  }
  //Если радио на передаче и отпустили ТХ
  if (!txsens && txen) {
    txen = false;
    vfosetup();
    mainscreen();
  }
}

void pushknob () {  // Обработка нажатия на кноб

  boolean knobdown = digitalRead(myEncBtn);   //Читаем состояние кноба
  if (!knobdown && knobup) {   //Если кноб был отпущен, но нажат сейчас
    knobup = false;   // отмечаем флаг что нажат
    knobMillis = millis();  // запускаем таймер для антидребезга
  }

  if (knobdown && !knobup) { //Если кноб нажат
    knobup = true; // отмечаем флаг что кноб отпущен
    long knobupmillis = millis();
    if (knobupmillis - knobMillis >= 1000) { //Если длительное нажатие
      if (menu == 0) menu = 3; //Перейти в инжменю
      else if (menu != 0) menu = 0;
    }

    if (knobupmillis - knobMillis < 1000 && knobupmillis - knobMillis > 100) { //Если кноб отпущен и был нажат и времени от таймера прошло 100Мс
      if (menu < 3 && menu > 0 && actfmenuf) { //Если 0<меню<3 и крутили енкодер в быстром меню, то выйти на главный экран
        actfmenuf = false;
        menu = 0;
      }
      else {
        menu ++; //Переходим на меню дальше
        if (menu == 3) menu = 0; //Если меню 3 выйти на главный экран
        if (menu > 14) menu = 3; //Если меню больше 13 перейти на меню 3
      }
      if (!number_of_bands && menu == 1) menu++; //Если диапазоны выключены - пропустить меню 1
    }

    mainscreen();
  }
}

void storetomem() { // Если крутили енкодер, то через 10 секунд все сохранить

  if ((millis() - actenc > 10000UL) && actencf) {
    actenc = millis();
    actencf = false;
    memwrite ();
  }
}

void readencoder() { // работа с енкодером
  long newPosition;
  newPosition = myEnc.read() / enc_div;
  if (reverse_encoder) newPosition *= (-1);
  if (newPosition != oldPosition && digitalRead(myEncBtn)) { // ЕСЛИ КРУТИЛИ НЕ нажатый энкодер
    if (menu > 0 && menu < 3) actfmenuf = true; // Если крутили энкодер в быстром меню - флаг вверх!
    switch (menu) {

      case 0: //Основная настройка частоты
        if (newPosition > oldPosition && vfo_freq <= max_freq * 100000UL) {
          if (vfo_freq % (arraystp[stp] * 10UL)) {
            vfo_freq = vfo_freq + (arraystp[stp] * 10UL) - (vfo_freq % (arraystp[stp] * 10UL));
          }
          else {
            vfo_freq = vfo_freq + (arraystp[stp] * 10UL);
          }
        }
        if (newPosition < oldPosition && vfo_freq >= min_freq * 100000UL) {
          if (vfo_freq % (arraystp[stp] * 10UL)) {
            vfo_freq = vfo_freq - (vfo_freq % (arraystp[stp] * 10UL));
          }
          else {
            vfo_freq = vfo_freq - (arraystp[stp] * 10UL);
          }
        }
        if (vfo_freq < min_freq * 100000UL) vfo_freq = min_freq * 100000UL;
        if (vfo_freq > max_freq * 100000UL) vfo_freq = max_freq * 100000UL;
        vfosetup();
        break;

      case 1: //Переключение диапазонов
        if (newPosition > oldPosition && band < number_of_bands) band++;
        if (newPosition < oldPosition && band > 0) band--;
        if (band > number_of_bands) band = number_of_bands;
        band_memread();
        bpfUpdate = false;
        vfosetup();
        break;

      case 2: //Настройка ШАГА настройки
        if (newPosition > oldPosition && stp < (sizeof(arraystp) / sizeof(arraystp[0]) - 1)) stp++;
        if (newPosition < oldPosition && stp > 0) stp--;
        //if (stp < 10) stp = 10;
        if (stp > (sizeof(arraystp) / sizeof(arraystp[0]) - 1)) stp = (sizeof(arraystp) / sizeof(arraystp[0]) - 1);
        break;

      case 3: //Настройка min_freq
        if (newPosition > oldPosition && min_freq <= max_hardware_freq) min_freq++;
        if (newPosition < oldPosition && min_freq >= min_hardware_freq) min_freq--;
        min_freq = constrain(min_freq, min_hardware_freq, max_freq - 1);
        break;

      case 4: //Настройка maxfreq
        if (newPosition > oldPosition && max_freq <= max_hardware_freq) max_freq++;
        if (newPosition < oldPosition && max_freq >= min_hardware_freq) max_freq--;
        max_freq = constrain(max_freq, min_freq + 1, max_hardware_freq);
        break;

      case 5: //Настройка bpf_set
        if (newPosition > oldPosition && bpf_set < max_bpf_set) bpf_set++;
        if (newPosition < oldPosition && bpf_set > 0) bpf_set--;
        bpf_set = constrain(bpf_set, 0, max_bpf_set);
        bpfUpdate = false;
        break;

      case 6: //Настройка LO гетеродина
        if (newPosition > oldPosition && lo_freq <= lo_max_freq) lo_freq += arraystp[stp];
        if (newPosition < oldPosition && lo_freq >= lo_min_freq) lo_freq -= arraystp[stp];
        lo_freq = constrain(lo_freq, lo_min_freq, lo_max_freq);
        vfosetup();
        break;

      case 7: //Настройка калибровки кварца PLL
        if (newPosition > oldPosition && Si_Xtall_calFreq <= 30000) Si_Xtall_calFreq += arraystp[stp];
        if (newPosition < oldPosition && Si_Xtall_calFreq >= - 30000) Si_Xtall_calFreq -= arraystp[stp];
        Si_Xtall_calFreq = constrain(Si_Xtall_calFreq, -30000, 30000);
        si5351correction();
        vfosetup();
        break;

      case 8: //Настройка калибровки по питанию
        if (newPosition > oldPosition && batt_cal <= 254) batt_cal++;
        if (newPosition < oldPosition && batt_cal >= 100) batt_cal--;
        if (batt_cal > 254) batt_cal = 254;
        if (batt_cal < 100) batt_cal = 100;
        break;

      case 9: // Настройка количества диапазонов
        if (newPosition > oldPosition && number_of_bands < max_number_of_bands) number_of_bands++;
        if (newPosition < oldPosition && number_of_bands > 0) number_of_bands--;
        if (number_of_bands > max_number_of_bands) number_of_bands = max_number_of_bands;
        break;

      case 10: //Настройка Часов
        if (newPosition > oldPosition && tm.Hour < 24) tm.Hour++;
        if (newPosition < oldPosition && tm.Hour > 0) tm.Hour--;
        if (tm.Hour > 23) tm.Hour = 23;
        timesetup = true;
        break;

      case 11: //Настройка Минут
        if (newPosition > oldPosition && tm.Minute < 60) tm.Minute++;
        if (newPosition < oldPosition && tm.Minute > 0) tm.Minute--;
        if (tm.Minute > 59) tm.Minute = 0;
        timesetup = true;
        break;

      case 12: //Инверсия энкодера.
        reverse_encoder = !reverse_encoder;
        newPosition *= (-1);
        break;

      case 13: // Настройка делителя импульсов энкодера
        if (newPosition > oldPosition && mem_enc_div < 255) mem_enc_div++;
        if (newPosition < oldPosition && mem_enc_div > 1) mem_enc_div--;
        mem_enc_div = constrain(mem_enc_div, 1, 255);
        break;

      case 14: //Калибровка термодатчика
        if (newPosition > oldPosition && temp_cal <= 30) temp_cal++;
        if (newPosition < oldPosition && temp_cal >= - 30) temp_cal--;
        temp_cal = constrain(temp_cal, -30, 30);
        break;

    }
    actenc = millis();
    actencf = true;
    mainscreen();
  }
  oldPosition = newPosition;
}

void powermeter () { // Измеритель уровня выхода
  analogRead(mypowerpin);
  int rawpower = analogRead(mypowerpin);
  mypower = map(rawpower, 0, 1023, 0, 100);
}

void battmeter () { // Измеритель напряжения питания
  analogRead(mybattpin);
  int rawbatt = analogRead(mybattpin);
  mybatt = map(rawbatt, 0, 1023, 0, batt_cal);
}

void mainscreen() { //Процедура рисования главного экрана
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(WHITE);
  display.setTextSize(3);
  switch (menu) {

    case 0: //Главный экран
      //Вывод частоты на дисплей
      if ((vfo_freq / 1000000) < 10) display.print(" ");
      display.print(vfo_freq / 1000000);//Вывод МГц
      display.setCursor(display.getCursorX() + 5, display.getCursorY()); //Переводим курсор чуть правее текущего положения
      if ((vfo_freq % 1000000) / 1000 < 100) display.print("0");
      if ((vfo_freq % 1000000) / 1000 < 10) display.print("0");
      display.print((vfo_freq % 1000000) / 1000); //Выводим КГц
      display.setTextSize(2); // Для сотен и десятков герц делаем шрифт поменьше
      display.setCursor(display.getCursorX() + 5, 7); //Переводим курсор чуть ниже текущего положения
      if ((vfo_freq % 1000) / 10 < 10) display.print("0"); //Если герц <10 то выводим "0" перед ними.
      display.println((vfo_freq % 1000) / 10);

      //Выводим вторую строку на дисплей
      display.setTextSize(1);// Ставим маленький шрифт
      if (mybatt - 100 < 0) display.print("0");
      display.print(mybatt / 10);
      display.print(".");
      display.print(mybatt % 10);
      display.print("v");

      if (txen) {//Если передача, то вывод показометра мощности
        display.print("PWR ");
        display.fillRect(64, 23, mypower, 9, WHITE);
      }
      else {// Если прием, то рисовать температуру часы, полосу и диапазон
        //char ddot
        if (temperature >= -50 && temperature <= 50) {
          if (temperature >= 0) display.print(" ");
          display.print(temperature);
          display.print((char)247);
          display.print("C");
        }
        else {
          display.print(" err ");
        }
        if (actencf) {
          display.print(" ");
        }
        else {
          display.print(".");
        }
        if (tm.Hour < 10) display.print(" ");
        display.print(tm.Hour);
        if (tm.Second % 2) {
          display.print(":");
        }
        else {
          display.print(" ");
        }
        if (tm.Minute < 10) display.print("0");
        display.print(tm.Minute);
        if (vfo_freq > 10000000UL) {
          display.print(" U");
        }
        else {
          display.print(" L");
        }
        if (band < 10) display.print(" ");
        display.print(band);
      }
      break;

    case 1: //Меню 1 - диапазон
      display.println(band);
      display.setTextSize(1);
      display.print(menu);
      display.print("  BAND from 0 to ");
      display.print(number_of_bands);
      break;

    case 2: //Меню 2 - шаг настройки
      display.println(arraystp[stp] * 10UL);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Step Hz");
      break;

    // Инжменю!!!

    case 3: //Меню 5 - Настройка min_freq
      display.println(min_freq * 100);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Min Freq ");
      display.print((char)240);
      display.print("kHz");
      break;

    case 4: //Меню 6 - Настройка maxfreq
      display.println(max_freq * 100);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Max Freq ");
      display.print((char)240);
      display.print("kHz");
      break;

    case 5: //Меню 6 - Канал ДПФ
      display.println(bpf_set);
      display.setTextSize(1);
      display.print(menu);
      display.print("  BPF Chanel 0-7");
      break;

    case 6: //Меню 7 - Настройка LO гетеродина
      display.setTextSize(2);
      display.println(lo_freq);
      display.setTextSize(1);
      display.print(menu);
      display.print("  LO ");
      display.print((char)240);
      display.print("Hz");
      break;

    case 7: //Меню 8 - Настройка калибровки кварца
      display.println(Si_Xtall_calFreq);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Xtal Cal ");
      display.print((char)240);
      display.print("Hz");
      break;

    case 8: //Меню 9 - Настройка калибровки по питанию
      display.println(batt_cal);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Batt Cal");
      break;

    case 9: //Меню 10 - Количество диапазонов
      display.println(number_of_bands);
      display.setTextSize(1);
      display.print(menu);
      display.print("  MAX Num Bands");
      break;

    case 10: //Меню 11 - Настройка Часов
      if (tm.Hour < 10) display.print("0");
      display.println(tm.Hour);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Hour");
      break;

    case 11: //Меню 12 - Настройка Минут
      if (tm.Minute < 10) display.print("0");
      display.println(tm.Minute);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Minute");
      break;

    case 12: //Меню 12 - Reverse Encoder
      if (reverse_encoder) {
        display.println("Yes");
      }
      else
      {
        display.println("NO");
      }
      display.setTextSize(1);
      display.print(menu);
      display.println("  Reverse Encoder");
      break;

    case 13: //Меню 13 - Reverse Encoder
      display.println(mem_enc_div);
      display.setTextSize(1);
      display.print(menu);
      display.print("  Encoder Divider");
      break;


    case 14: //Меню 13 - Калибровка термодатчика
      display.println(temp_cal);
      display.setTextSize(1);
      display.print(menu);
      display.print(" Temp CAL ");
      display.print((char)240);
      display.print((char)247);
      display.print("C");
      break;

  }
  display.display();
  //debug();

}


void vfosetup() {
  if (vfo_freq > 10000000UL) {
    si.set_freq((vfo_freq - lo_freq), 0, (lo_freq));
  }
  else {
    si.set_freq((vfo_freq + lo_freq), 0, (lo_freq));
  }
}

void si5351init() {
  si.setup(0, 0, 0);
  si.cload(si_cload);
}

void si5351correction() {
  si.set_xtal_freq(Si_Xtall_Freq + Si_Xtall_calFreq);
  si.update_freq(0);
  si.update_freq(2);
}

void memwrite () { //Запись general_setting
  int crc = 0;
  byte i = 0;
  byte * adr;
  adr =  (byte*)(& general_setting);
  while (i < (sizeof(general_setting)))
  {
    crc += *(adr + i);
    i++;
  }
  AT24C32.writeEE(2, general_setting);
  AT24C32.writeEE(0, (crc + 1));

  // Запись band_setting
  crc = 0;
  i = 0;
  adr =  (byte*)(& band_setting);
  while (i < (sizeof(band_setting)))
  {
    crc += *(adr + i);
    i++;
  }
  AT24C32.writeEE(sizeof(general_setting) + 2 + ((sizeof(band_setting) + 2)*band) + 2, band_setting);
  AT24C32.writeEE(sizeof(general_setting) + 2 + ((sizeof(band_setting) + 2)*band), (crc + 1));
}

void memread() {
  int crc = 0;
  int crcrom = 0;
  byte i = 0;

  // Чтение general_setting
  AT24C32.readEE (0, crc);
  while (i < (sizeof(general_setting)))
  {
    crcrom += AT24C32.readByte ((i + 2));
    i++;
  }
  if (crc == (crcrom + 1)) {
    AT24C32.readEE (2, general_setting);
  }
  else {
    memwrite ();
  }
  band_memread();
}

void band_memread() {

  int crc = 0;
  int crcrom = 0;
  byte i = 0;
  AT24C32.readEE (sizeof(general_setting) + 2 + ((sizeof(band_setting) + 2) * band), crc);
  while (i < (sizeof(band_setting)))
  {
    crcrom += AT24C32.readByte ((i + sizeof(general_setting) + 2 + ((sizeof(band_setting) + 2) * band)) + 2);
    i++;
  }
  if (crc == (crcrom + 1)) {
    AT24C32.readEE (sizeof(general_setting) + 2 + ((sizeof(band_setting) + 2)*band) + 2, band_setting);
  }
  else {
    memwrite ();
  }
}

void versionprint() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(WHITE);
  display.setTextSize(3);
  display.println(ver);
  display.display();
  delay(1000);
}


void bpfset() {
  if (!bpfUpdate) {
    byte byteToSend = 0;
    bitSet(byteToSend, bpf_set);
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, byteToSend);
    digitalWrite(latchPin, HIGH);
    bpfUpdate = true;
  }
}

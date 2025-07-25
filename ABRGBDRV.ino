#include <FastLED.h>
#include <at24c02.h>

void (*resetFunc)(void) = 0;

#define MAX_NUM_LEDS 30
#define DATA_CH1 2
#define DATA_CH2 3
#define DATA_CH3 4
#define DATA_CH4 5
int numCH1;
int numCH2;
int numCH3;
int numCH4;
CRGB ledCH1[MAX_NUM_LEDS];
CRGB ledCH2[MAX_NUM_LEDS];
CRGB ledCH3[MAX_NUM_LEDS];
CRGB ledCH4[MAX_NUM_LEDS];

struct funcSin {  //y=a*sin[b*(x+c)]+d
  float a, b, c, d;
  float getValue(int x) {
    return a * sin(b * (x + c)) + d;
  }
  funcSin(float aa, float bb, float cc, float dd) {
    a = aa;
    b = bb;
    c = cc;
    d = dd;
  }
};

struct funcCH {
  funcSin R, G, B;
  int rollSpeed;  //Unit--5ms
  funcCH()
    : R(20.0, 0.25, 0.0, 50.0),
      G(20.0, 0.25, 0.0, 50.0),
      B(20.0, 0.25, 0.0, 50.0),
      rollSpeed(2) {}
} funcCH1, funcCH2, funcCH3, funcCH4;

int roll, phaseCH1, phaseCH2, phaseCH3, phaseCH4;

void rollLights() {
  if (roll % funcCH1.rollSpeed == 0) {
    phaseCH1++;
    if (phaseCH1 >= numCH1)
      phaseCH1 = 0;
    int i, j;
    for (i = 0, j = phaseCH1; j < numCH1; i++, j++) {
      ledCH1[j].r = funcCH1.R.getValue(i);
      ledCH1[j].g = funcCH1.G.getValue(i);
      ledCH1[j].b = funcCH1.B.getValue(i);
    }
    for (j = 0; j < phaseCH1; i++, j++) {
      ledCH1[j].r = funcCH1.R.getValue(i);
      ledCH1[j].g = funcCH1.G.getValue(i);
      ledCH1[j].b = funcCH1.B.getValue(i);
    }
  }
  if (roll % funcCH2.rollSpeed == 0) {
    phaseCH2++;
    if (phaseCH2 >= numCH2)
      phaseCH2 = 0;
    int i, j;
    for (i = 0, j = phaseCH2; j < numCH2; i++, j++) {
      ledCH2[j].r = funcCH2.R.getValue(i);
      ledCH2[j].g = funcCH2.G.getValue(i);
      ledCH2[j].b = funcCH2.B.getValue(i);
    }
    for (j = 0; j < phaseCH2; i++, j++) {
      ledCH2[j].r = funcCH2.R.getValue(i);
      ledCH2[j].g = funcCH2.G.getValue(i);
      ledCH2[j].b = funcCH2.B.getValue(i);
    }
  }
  if (roll % funcCH3.rollSpeed == 0) {
    phaseCH3++;
    if (phaseCH3 >= numCH3)
      phaseCH3 = 0;
    int i, j;
    for (i = 0, j = phaseCH3; j < numCH3; i++, j++) {
      ledCH3[j].r = funcCH3.R.getValue(i);
      ledCH3[j].g = funcCH3.G.getValue(i);
      ledCH3[j].b = funcCH3.B.getValue(i);
    }
    for (j = 0; j < phaseCH3; i++, j++) {
      ledCH3[j].r = funcCH3.R.getValue(i);
      ledCH3[j].g = funcCH3.G.getValue(i);
      ledCH3[j].b = funcCH3.B.getValue(i);
    }
  }
  if (roll % funcCH4.rollSpeed == 0) {
    phaseCH4++;
    if (phaseCH4 >= numCH4)
      phaseCH4 = 0;
    int i, j;
    for (i = 0, j = phaseCH4; j < numCH4; i++, j++) {
      ledCH4[j].r = funcCH4.R.getValue(i);
      ledCH4[j].g = funcCH4.G.getValue(i);
      ledCH4[j].b = funcCH4.B.getValue(i);
    }
    for (j = 0; j < phaseCH4; i++, j++) {
      ledCH4[j].r = funcCH4.R.getValue(i);
      ledCH4[j].g = funcCH4.G.getValue(i);
      ledCH4[j].b = funcCH4.B.getValue(i);
    }
  }
  FastLED.show();
  roll++;
}

AT24C02 ee(AT24C_ADDRESS_0);
#define EE24LC02MAXBYTES 2048 / 8
#define FLOAT_BYTE_NUM 4  //float类型占用字节数
#define INT_BYTE_NUM 4
//浮点数存储共用体
union storFloatData {
  float value;
  uint8_t byte[FLOAT_BYTE_NUM];
};

union storIntData {
  int value;
  uint8_t byte[INT_BYTE_NUM];
};

void Storage_WriteFloatNum(uint16_t addr, union storFloatData data) {
  uint8_t i = 0;
  uint8_t a = 0;

  for (i = 0; i <= FLOAT_BYTE_NUM - 1; i++) {
    ee.write(addr + i, data.byte[i]);
    delay(10);
  }
  return;
}
union storFloatData Storage_ReadFloatNum(uint16_t addr) {
  uint8_t i = 0;
  storFloatData read_data;

  for (i = 0; i <= FLOAT_BYTE_NUM - 1; i++) {
    read_data.byte[i] = ee.read(addr + i);
  }

  return read_data;
}

void Storage_WriteIntNum(uint16_t addr, union storIntData data) {
  uint8_t i = 0;
  uint8_t a = 0;

  for (i = 0; i <= INT_BYTE_NUM - 1; i++) {
    ee.write(addr + i, data.byte[i]);
    delay(10);
  }
  return;
}
union storIntData Storage_ReadIntNum(uint16_t addr) {
  uint8_t i = 0;
  storIntData read_data;

  for (i = 0; i <= INT_BYTE_NUM - 1; i++) {
    read_data.byte[i] = ee.read(addr + i);
  }

  return read_data;
}
/*————————————————
版权声明：本文为CSDN博主「你行你上天」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/hnxyxiaomeng/article/details/53525306*/

#define SERIAL_OUT Serial

const float Rsense = 0.01;
const float Gain = 60.0;
const float Vref = 5.0;  // 内部参考电压

int testAvailableLEDS(int ch) {
  CRGB *leds;
  int adcPin;
  switch (ch) {
    case 1:
      leds = ledCH1;
      adcPin = A0;
      break;
    case 2:
      leds = ledCH2;
      adcPin = A1;
      break;
    case 3:
      leds = ledCH3;
      adcPin = A2;
      break;
    case 4:
      leds = ledCH4;
      adcPin = A3;
  }
  FastLED.clear();
  FastLED.show();
  delay(500);

  int count = 2;
  float prev = 0;
  for (int i = 0; i < MAX_NUM_LEDS; i++) {
    leds[i] = CRGB::White;
    FastLED.show();
    delay(100);
    int adcValue = analogRead(adcPin);
    float voltage = adcValue * Vref / 1023.0;
    float current = voltage / (Rsense * Gain);

    // Serial.print("数值: ");
    // Serial.println(adcValue);
    // Serial.print("电压: ");
    // Serial.println(voltage, 2);
    // Serial.print("电流: ");
    // Serial.println(current, 2);
    // Serial.print("电流差: ");
    // Serial.println(current - prev, 3);
    if (i >= 2 && current - prev > 0.08)
      count++;
    prev = current;
  }

  FastLED.clear();
  FastLED.show();

  Serial.print("电流: ");
  Serial.print(prev, 2);
  Serial.print("A | 数量: ");
  Serial.println(count);

  return count;
}

void writeCHinfo(int ch, int num, funcCH led) {
  uint16_t offset;
  switch (ch) {
    case 1:
      offset = 0x0;
      break;
    case 2:
      offset = 0x40;
      break;
    case 3:
      offset = 0x80;
      break;
    case 4:
      offset = 0xC0;
  }
  storIntData n;
  storFloatData f;
  n.value = num;
  Storage_WriteIntNum(offset + 0x4, n);
  n.value = led.rollSpeed;
  Storage_WriteIntNum(offset + 0x8, n);

  f.value = led.R.a;
  Storage_WriteFloatNum(offset + 0xC, f);
  f.value = led.R.b;
  Storage_WriteFloatNum(offset + 0x10, f);
  f.value = led.R.c;
  Storage_WriteFloatNum(offset + 0x14, f);
  f.value = led.R.d;
  Storage_WriteFloatNum(offset + 0x18, f);

  f.value = led.G.a;
  Storage_WriteFloatNum(offset + 0x1C, f);
  f.value = led.G.b;
  Storage_WriteFloatNum(offset + 0x20, f);
  f.value = led.G.c;
  Storage_WriteFloatNum(offset + 0x24, f);
  f.value = led.G.d;
  Storage_WriteFloatNum(offset + 0x28, f);

  f.value = led.B.a;
  Storage_WriteFloatNum(offset + 0x2C, f);
  f.value = led.B.b;
  Storage_WriteFloatNum(offset + 0x30, f);
  f.value = led.B.c;
  Storage_WriteFloatNum(offset + 0x34, f);
  f.value = led.B.d;
  Storage_WriteFloatNum(offset + 0x38, f);
}

void readCHinfo(int ch, int &num, funcCH &led) {
  uint16_t offset;
  switch (ch) {
    case 1:
      offset = 0x0;
      break;
    case 2:
      offset = 0x40;
      break;
    case 3:
      offset = 0x80;
      break;
    case 4:
      offset = 0xC0;
  }
  storIntData n;
  storFloatData f;
  n = Storage_ReadIntNum(offset + 0x4);
  num = n.value;
  n = Storage_ReadIntNum(offset + 0x8);
  led.rollSpeed = n.value;

  f = Storage_ReadFloatNum(offset + 0xC);
  led.R.a = f.value;
  f = Storage_ReadFloatNum(offset + 0x10);
  led.R.b = f.value;
  f = Storage_ReadFloatNum(offset + 0x14);
  led.R.c = f.value;
  f = Storage_ReadFloatNum(offset + 0x18);
  led.R.d = f.value;

  f = Storage_ReadFloatNum(offset + 0x1C);
  led.G.a = f.value;
  f = Storage_ReadFloatNum(offset + 0x20);
  led.G.b = f.value;
  f = Storage_ReadFloatNum(offset + 0x24);
  led.G.c = f.value;
  f = Storage_ReadFloatNum(offset + 0x28);
  led.G.d = f.value;

  f = Storage_ReadFloatNum(offset + 0x2C);
  led.B.a = f.value;
  f = Storage_ReadFloatNum(offset + 0x30);
  led.B.b = f.value;
  f = Storage_ReadFloatNum(offset + 0x34);
  led.B.c = f.value;
  f = Storage_ReadFloatNum(offset + 0x38);
  led.B.d = f.value;
}

void readAndPrintAll() {
  for (int i = 0; i < 0x100; i++) {
    byte by = ee.read(i);
    SERIAL_OUT.print(by, HEX);
    SERIAL_OUT.print('\t');
    if (i % 4 == 3)
      SERIAL_OUT.print('\n');
  }
}

void setup() {
  analogReference(INTERNAL);  // 提高精度
  Serial.begin(9600);
  Wire.begin();

  float readValue;

  FastLED.addLeds<NEOPIXEL, DATA_CH1>(ledCH1, MAX_NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, DATA_CH2>(ledCH2, MAX_NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, DATA_CH3>(ledCH3, MAX_NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, DATA_CH4>(ledCH4, MAX_NUM_LEDS);

  funcCH1.rollSpeed = 5;
  funcCH1.R = funcSin(20.0, 0.25, 0.0, 50.0);
  funcCH1.G = funcSin(20.0, 0.25, 8.0, 50.0);
  funcCH1.B = funcSin(20.0, 0.25, -8.0, 50.0);

  funcCH2.rollSpeed = 5;
  funcCH2.R = funcSin(80.0, 0.25, 0.0, 80.0);
  funcCH2.G = funcSin(80.0, 0.25, 6.0, 80.0);
  funcCH2.B = funcSin(80.0, 0.25, -6.0, 80.0);

  funcCH3.rollSpeed = 2;
  funcCH3.R = funcSin(-80.0, 0.25, 0.0, 80.0);
  funcCH3.G = funcSin(-80.0, 0.25, 6.0, 80.0);
  funcCH3.B = funcSin(-80.0, 0.25, -6.0, 80.0);

  funcCH4.rollSpeed = 10;
  funcCH4.R = funcSin(-80.0, 0.25, 0.0, 80.0);
  funcCH4.G = funcSin(-80.0, 0.25, 6.0, 80.0);
  funcCH4.B = funcSin(-80.0, 0.25, -6.0, 80.0);

  // ee.write(0x0, 0xFF);
  // delay(10);
  //readAndPrintAll();


  byte by = ee.read(0x0);
  if (by != 0xFF) {
    SERIAL_OUT.println("Available data found.");
    readCHinfo(1, numCH1, funcCH1);
    readCHinfo(2, numCH2, funcCH2);
    readCHinfo(3, numCH3, funcCH3);
    readCHinfo(4, numCH4, funcCH4);
  } else {
    SERIAL_OUT.println("No available data.Filling the default data.");
    numCH1 = testAvailableLEDS(1);
    numCH2 = testAvailableLEDS(2);
    numCH3 = testAvailableLEDS(3);
    numCH4 = testAvailableLEDS(4);
    writeCHinfo(1, numCH1, funcCH1);
    writeCHinfo(2, numCH2, funcCH2);
    writeCHinfo(3, numCH3, funcCH3);
    writeCHinfo(4, numCH4, funcCH4);
    ee.write(0x0, 0xAA);
  }

  // pinMode(13, OUTPUT);
  // Timer1.initialize(5000);  // initialize timer1, and set a 1/2 second period
  // Timer1.pwm(9, 512);       // setup pwm on pin 9, 50% duty cycle
  // Timer1.attachInterrupt(rollLights);  // attaches callback() as a timer overflow interrupt
  //                                      // ————————————————
  //                                      // 版权声明：本文为CSDN博主「Zeeland」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
  //                                      // 原文链接：https://blog.csdn.net/linZinan_/article/details/127832771
}

void loop() {
  unsigned long oldTime = micros(), lastTime;
  if (SERIAL_OUT.available()) {
    String s = SERIAL_OUT.readStringUntil('\n');
    if (s[0] == 'R' && s[1] != 'S' && s[1] != 'E') {
      int ch = s[1] - '0';
      switch (ch) {
        case 1:
          SERIAL_OUT.println("#NN" + String(numCH1));
          SERIAL_OUT.println("#TT" + String(funcCH1.rollSpeed));

          SERIAL_OUT.println("#RA" + String(funcCH1.R.a));
          SERIAL_OUT.println("#RB" + String(funcCH1.R.b));
          SERIAL_OUT.println("#RC" + String(funcCH1.R.c));
          SERIAL_OUT.println("#RD" + String(funcCH1.R.d));

          SERIAL_OUT.println("#GA" + String(funcCH1.G.a));
          SERIAL_OUT.println("#GB" + String(funcCH1.G.b));
          SERIAL_OUT.println("#GC" + String(funcCH1.G.c));
          SERIAL_OUT.println("#GD" + String(funcCH1.G.d));

          SERIAL_OUT.println("#BA" + String(funcCH1.B.a));
          SERIAL_OUT.println("#BB" + String(funcCH1.B.b));
          SERIAL_OUT.println("#BC" + String(funcCH1.B.c));
          SERIAL_OUT.println("#BD" + String(funcCH1.B.d));
          break;
        case 2:
          SERIAL_OUT.println("#NN" + String(numCH2));
          SERIAL_OUT.println("#TT" + String(funcCH2.rollSpeed));

          SERIAL_OUT.println("#RA" + String(funcCH2.R.a));
          SERIAL_OUT.println("#RB" + String(funcCH2.R.b));
          SERIAL_OUT.println("#RC" + String(funcCH2.R.c));
          SERIAL_OUT.println("#RD" + String(funcCH2.R.d));

          SERIAL_OUT.println("#GA" + String(funcCH2.G.a));
          SERIAL_OUT.println("#GB" + String(funcCH2.G.b));
          SERIAL_OUT.println("#GC" + String(funcCH2.G.c));
          SERIAL_OUT.println("#GD" + String(funcCH2.G.d));

          SERIAL_OUT.println("#BA" + String(funcCH2.B.a));
          SERIAL_OUT.println("#BB" + String(funcCH2.B.b));
          SERIAL_OUT.println("#BC" + String(funcCH2.B.c));
          SERIAL_OUT.println("#BD" + String(funcCH2.B.d));
          break;
        case 3:
          SERIAL_OUT.println("#NN" + String(numCH3));
          SERIAL_OUT.println("#TT" + String(funcCH3.rollSpeed));

          SERIAL_OUT.println("#RA" + String(funcCH3.R.a));
          SERIAL_OUT.println("#RB" + String(funcCH3.R.b));
          SERIAL_OUT.println("#RC" + String(funcCH3.R.c));
          SERIAL_OUT.println("#RD" + String(funcCH3.R.d));

          SERIAL_OUT.println("#GA" + String(funcCH3.G.a));
          SERIAL_OUT.println("#GB" + String(funcCH3.G.b));
          SERIAL_OUT.println("#GC" + String(funcCH3.G.c));
          SERIAL_OUT.println("#GD" + String(funcCH3.G.d));

          SERIAL_OUT.println("#BA" + String(funcCH3.B.a));
          SERIAL_OUT.println("#BB" + String(funcCH3.B.b));
          SERIAL_OUT.println("#BC" + String(funcCH3.B.c));
          SERIAL_OUT.println("#BD" + String(funcCH3.B.d));
          break;
        case 4:
          SERIAL_OUT.println("#NN" + String(numCH4));
          SERIAL_OUT.println("#TT" + String(funcCH4.rollSpeed));

          SERIAL_OUT.println("#RA" + String(funcCH4.R.a));
          SERIAL_OUT.println("#RB" + String(funcCH4.R.b));
          SERIAL_OUT.println("#RC" + String(funcCH4.R.c));
          SERIAL_OUT.println("#RD" + String(funcCH4.R.d));

          SERIAL_OUT.println("#GA" + String(funcCH4.G.a));
          SERIAL_OUT.println("#GB" + String(funcCH4.G.b));
          SERIAL_OUT.println("#GC" + String(funcCH4.G.c));
          SERIAL_OUT.println("#GD" + String(funcCH4.G.d));

          SERIAL_OUT.println("#BA" + String(funcCH4.B.a));
          SERIAL_OUT.println("#BB" + String(funcCH4.B.b));
          SERIAL_OUT.println("#BC" + String(funcCH4.B.c));
          SERIAL_OUT.println("#BD" + String(funcCH4.B.d));
      }
    } else if (s == "REC") {
      SERIAL_OUT.println("Resetting everything to default.");
      ee.write(0x0, 0xFF);
      delay(300);
      resetFunc();
    } else if (s == "RST") {
      SERIAL_OUT.println("Resetting...");
      delay(300);
      resetFunc();
    } else if (s[0] == 'S') {  //Set Command Format : S(CH)*(NUM)*(RS)*(RA)*(RB)*(RC)*(RD)*(GA)*(GB)*(GC)*(GD)*(BA)*(BB)*(BC)*(BD)\n
      int ch = s[1] - '0';
      String ss[14];
      for (int i = 3, j = 0; i < s.length(); i++) {
        if (s[i] == '*') {
          j++;
          continue;
        }
        ss[j] += s[i];
      }
      switch (ch) {
        case 1:
          numCH1 = ss[0].toInt();
          funcCH1.rollSpeed = ss[1].toInt();

          funcCH1.R.a = ss[2].toFloat();
          funcCH1.R.b = ss[3].toFloat();
          funcCH1.R.c = ss[4].toFloat();
          funcCH1.R.d = ss[5].toFloat();

          funcCH1.G.a = ss[6].toFloat();
          funcCH1.G.b = ss[7].toFloat();
          funcCH1.G.c = ss[8].toFloat();
          funcCH1.G.d = ss[9].toFloat();

          funcCH1.B.a = ss[10].toFloat();
          funcCH1.B.b = ss[11].toFloat();
          funcCH1.B.c = ss[12].toFloat();
          funcCH1.B.d = ss[13].toFloat();
          break;
        case 2:
          numCH2 = ss[0].toInt();
          funcCH2.rollSpeed = ss[1].toInt();

          funcCH2.R.a = ss[2].toFloat();
          funcCH2.R.b = ss[3].toFloat();
          funcCH2.R.c = ss[4].toFloat();
          funcCH2.R.d = ss[5].toFloat();

          funcCH2.G.a = ss[6].toFloat();
          funcCH2.G.b = ss[7].toFloat();
          funcCH2.G.c = ss[8].toFloat();
          funcCH2.G.d = ss[9].toFloat();

          funcCH2.B.a = ss[10].toFloat();
          funcCH2.B.b = ss[11].toFloat();
          funcCH2.B.c = ss[12].toFloat();
          funcCH2.B.d = ss[13].toFloat();
          break;
        case 3:
          numCH3 = ss[0].toInt();
          funcCH3.rollSpeed = ss[1].toInt();

          funcCH3.R.a = ss[2].toFloat();
          funcCH3.R.b = ss[3].toFloat();
          funcCH3.R.c = ss[4].toFloat();
          funcCH3.R.d = ss[5].toFloat();

          funcCH3.G.a = ss[6].toFloat();
          funcCH3.G.b = ss[7].toFloat();
          funcCH3.G.c = ss[8].toFloat();
          funcCH3.G.d = ss[9].toFloat();

          funcCH3.B.a = ss[10].toFloat();
          funcCH3.B.b = ss[11].toFloat();
          funcCH3.B.c = ss[12].toFloat();
          funcCH3.B.d = ss[13].toFloat();
          break;
        case 4:
          numCH4 = ss[0].toInt();
          funcCH4.rollSpeed = ss[1].toInt();

          funcCH4.R.a = ss[2].toFloat();
          funcCH4.R.b = ss[3].toFloat();
          funcCH4.R.c = ss[4].toFloat();
          funcCH4.R.d = ss[5].toFloat();

          funcCH4.G.a = ss[6].toFloat();
          funcCH4.G.b = ss[7].toFloat();
          funcCH4.G.c = ss[8].toFloat();
          funcCH4.G.d = ss[9].toFloat();

          funcCH4.B.a = ss[10].toFloat();
          funcCH4.B.b = ss[11].toFloat();
          funcCH4.B.c = ss[12].toFloat();
          funcCH4.B.d = ss[13].toFloat();
      }
    } else if (s[0] == 'W') {
      SERIAL_OUT.println("Writing to EEPROM...");
      writeCHinfo(1, numCH1, funcCH1);
      writeCHinfo(2, numCH2, funcCH2);
      writeCHinfo(3, numCH3, funcCH3);
      writeCHinfo(4, numCH4, funcCH4);
    } else if (s[0] == 'T') {
      int ch = s[1] - '0', num;
      num = testAvailableLEDS(ch);
      SERIAL_OUT.println("#NN" + String(num));
    }
  }
  rollLights();
  lastTime = micros() - oldTime;
  if (lastTime < 5000) {
    //SERIAL_OUT.println(lastTime);
    delay((5000 - lastTime) / 1000);
  }
  // delay(5);
}

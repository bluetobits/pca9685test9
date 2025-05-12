//pca9685test9 Steve Lomax April 2025
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Encoder.h>
#include <EEPROM.h>
#include <PCF8575.h>            //https://github.com/RobTillaart/PCF8575
#include <FastLED.h>            //https://github.com/FastLED/FastLED
#include <LiquidCrystal_I2C.h>  //https://github.com/johnrickman/LiquidCrystal_I2C


#define DEBUG 1
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define debugln2(x, y) Serial.println(x, y)
#else
#define debug(x)
#define debugln(x)
#define debugln2(x, y)
#endif

#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates
constexpr int NO_OF_SERVOS = 32;
const int TOP_PULSE_LEN = 2400;    // setting the maximum cw servo position(actual = 2500 but not all servos are the same)
const int BOTTOM_PULSE_LEN = 600;  //setting the minimum ccw servo position

constexpr int8_t POINT_PAIRS[] = { 0, 1, 2, 3, 3, 3, 3, -3,
                                   8, 9, 10, 11, 3, 13, 14, 15,
                                   16, -1, 18, 19, 20, 21, 22, 23,
                                   24, 25, 26, 27, 28, 29, 30, 31 };


constexpr int pushSw[] = { 5, 4, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
                           7, 6, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9 };

constexpr int EEPROM_ADDRESS = 0;  // Define EEPROM base address
constexpr int ENCODER_PUSH = 8;
constexpr int LONG_PUSH = 2000;
// unsigned long pollTimeOut;
//constexpr unsigned int pollFreq = 2;
const uint8_t ENCA_PIN = 2;  // encoder pin A
const uint8_t ENCB_PIN = 3;  // encoder pin B
int cal = 0;
int pointPairing = 1;// flag if selected. int because memstruct must be the same tyoes
bool centreServo = 0;
bool changeMoveSpeed = 0;
int lastPointMoved = 0;
int oldLastPointMoved = 0;
int moveSpeed = 60;
int encoderPos = 0;

const uint8_t LEDS_MIMIC[] = {
  0, 1, 2, 3, 4, 5, 6, 7,
  8, 9, 10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 21, 22, 23,
  24, 25, 26, 27, 28, 29, 30, 31
};
const uint8_t NO_OF_LEDS = 32;
const uint8_t DATA_PIN = 11;  // neopixels data connect from here via 50 ohm resistor
CRGB leds[NO_OF_LEDS];        //LED neopixel strip
//colour Hues:
const uint8_t Hred = 0;
const uint8_t Hora = 32;
const uint8_t Hyel = 64;
const uint8_t Hgre = 96;
const uint8_t Hcya = 128;
const uint8_t Hblu = 160;
const uint8_t Hpur = 192;
const uint8_t Hpin = 224;
bool moving = 0;



unsigned long timeNow;
unsigned long flashTimeNow;
unsigned long flashTimeout = 0;        //Global.  Neopixel flash timer
bool flash = 0;                        //Global.  Neopixel flash status
const unsigned long flashSpeed = 400;  // flash rate in ms
uint8_t onSat = 255;                   // Global. Neopixel saturation level (0-255)
uint8_t onLev = 80;                    // Global. Neopixel brightness level (0-255)
uint8_t onHue = 255;

Adafruit_PWMServoDriver PCA1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver PCA2 = Adafruit_PWMServoDriver(0x43);
PCF8575 PCF1(0x20);  // Set the PCF1 I2C address (default is 0x20)
PCF8575 PCF2(0x22);  // Set the PCF2 I2C address (default is 0x20)
Encoder encoder(ENCA_PIN, ENCB_PIN);
LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 20 chars and 4 line display


struct MemStruct {
  int mclosedPos[NO_OF_SERVOS];
  int mthrownPos[NO_OF_SERVOS];
  int mmoveSpeedMem;
  int mpointPairing = 1;  // save point pairing enabled status

  void readEEPROM() {
    EEPROM.get(EEPROM_ADDRESS, *this);
  }
  void writeEEPROM() {
    EEPROM.put(EEPROM_ADDRESS, *this);
  }
};

MemStruct memData;


struct Points {
  int closedPos = 700;
  int thrownPos = 2300;
  int curPos = 700;
  bool target = 0;
  void MovePoint(int index) {
    // Determine desired position
    int targetPos = target ? thrownPos : closedPos;
    int delta = targetPos - curPos;//DTG from current pos.
    if (delta != 0) {
      // Move by at most moveSpeed each call
      int step;// distance to move in this call
      if (delta > 0) { //clockwise movement
        if (delta < moveSpeed) step = delta; //if the DTG is less than movespeed set the step to the DTG
        else step = moveSpeed;// otherwise set step to the movespeed
      } else {
        if (delta > -moveSpeed) step = delta; // same for anticlockwise.
        else step = -moveSpeed;
      }
      curPos += step; 
      moving = true;
      
      // Write pulse to the correct driver
      if (index < 16) {
        PCA1.writeMicroseconds(index, curPos);
      } else {
        PCA2.writeMicroseconds(index - 16, curPos);
      }
    }
  }
};

Points point[NO_OF_SERVOS];


void savePointValues() {

  for (int i = 0; i < NO_OF_SERVOS; i++) {
    memData.mthrownPos[i] = point[i].thrownPos;
    memData.mclosedPos[i] = point[i].closedPos;
  }
  memData.mmoveSpeedMem = moveSpeed;
  memData.mpointPairing = pointPairing;
  memData.writeEEPROM();
  debug("written pointPairing = ");
  debug(pointPairing);
  debug(" Written moveSpeed = ");
  debugln(moveSpeed);
  loadPointValues();
}
void loadPointValues() {

  memData.readEEPROM();
  for (int i = 0; i < NO_OF_SERVOS; i++) {  // transfer memory values to point values
    point[i].thrownPos = memData.mthrownPos[i];
    point[i].closedPos = memData.mclosedPos[i];
    if (point[i].thrownPos < BOTTOM_PULSE_LEN || point[i].thrownPos > TOP_PULSE_LEN) {
      point[i].thrownPos = 1700;
    }
    if (point[i].closedPos < BOTTOM_PULSE_LEN || point[i].closedPos > TOP_PULSE_LEN) {
      point[i].closedPos = 1300;
    }
  }
  moveSpeed = memData.mmoveSpeedMem;
  pointPairing = memData.mpointPairing;
  if (pointPairing > 1) pointPairing = 1;
  if (moveSpeed == 0) (moveSpeed = 10);
  debug("loaded pointPairing = ");
  debug(pointPairing);
  debug("  loaded moveSpeed = ");
  debugln(moveSpeed);
}

void startPos() {
  int tempspeed = moveSpeed;
  moveSpeed = 1500;
  for (int i = 0; i < NO_OF_SERVOS; i++) {
    point[i].target = 0;
    point[i].MovePoint(i);
  }
  moveSpeed = tempspeed;
}
void scanButtons() {
  // unsigned long nowMillis = millis();
  // if (nowMillis - pollTimeOut >= pollFreq) {
  //   pollTimeOut = millis();
  for (int i = 0; i < NO_OF_SERVOS; i++) {
    if (i < 16) {

      if (!PCF1.read(i)) {
        while (!(PCF1.read(i))) {}
        delay(10);
        // debug("PCF1 pressed for servo ");
        // debug(i);

        point[i].target = !point[i].target;//flip the target 
        point[i].MovePoint(i);
        pointPairs(i);
        lastPointMoved = i;
        lcdPos();//writes the position character
      }
    } else {
      if (!PCF2.read(i - 16)) {
        while (!PCF2.read(i - 16)) {}
        delay(10);
        // debug("PCF2 pressed for servo ");
        // debug(i);

        point[i].target = !point[i].target;
        point[i].MovePoint(i);
        pointPairs(i);
        lastPointMoved = i;
        lcdPos();//writes the position character
      }
    }
  }
}
void pointPairs(int idx) {  //idx = the current switched point
  if (pointPairing) {
    //
    for (int k = idx; k < NO_OF_SERVOS; k++) {  // from current position through to all higher
      if (k != POINT_PAIRS[idx]) {
        if (abs(POINT_PAIRS[k]) == idx) {  // if the value of point Paris  == the current position
          POINT_PAIRS[k] < 0 ? (point[k].target = !point[idx].target) : (point[k].target = point[idx].target);
          point[k].MovePoint(k);
        }
      }
    }
  }
}
void scanPushButtons() {
  for (int i = 0; i < NO_OF_SERVOS; i++) {
    if (digitalRead(pushSw[i]) == 0) {
      // debug("Manual Button pressed for servo ");
      // debug(i);
      point[i].target = !point[i].target;
      point[i].MovePoint(i);
      pointPairs(i);
      delay(10);
      while (digitalRead(pushSw[i]) == 0) {}
      delay(10);
      lastPointMoved = i;
      lcdPos();
    }
  }
}

void calibrate() {
  static int encoderPos = 0;
  bool longPress = 0;
  int pressCount = 0;

  if (digitalRead(ENCODER_PUSH) == 0) {  // it's a press

    lcd.clear();
    lcd.setCursor(4, 0);
    lcd.print(F("Short Press"));
    lcd.setCursor(3, 2);
    lcd.print(F("release to exit"));
    lcd.setCursor(2, 3);
    (cal) ? (lcd.print(F("without saving"))) : (lcd.print(F("hold to calibrate")));
    lcd.setCursor(1, 0);

    unsigned long timepressed = millis();
    while (digitalRead(ENCODER_PUSH) == 0) {
      if (millis() - timepressed > LONG_PUSH) {  // long press
        longPress = 1;
        lcd.clear();
        lcd.setCursor(1, 0);
        if (cal || changeMoveSpeed) {
          (lcd.print(F("Saving...")));
          delay (2000);
          lcdGrid();
          lcdPos();
        } else {
          (lcd.print(F("Calibrating Points")));
          lcd.setCursor(0, 3);
          lcd.print(F("Long=Save Short=Undo"));
        }
      }
    }
    delay(20);                      // released for 1st time
    if (!cal || changeMoveSpeed) {  // IF NOT CALIBRATING and encoder was pushed & released  at least once

      // while (digitalRead(ENCODER_PUSH) == 0) {}  //wait for release
      // delay(100);                                //debounce

      if (longPress) {
        if (changeMoveSpeed) {  // Save speed and reset flag
          savePointValues();
          changeMoveSpeed = false;
          debug("Saved moveSpeed");
        } else {             //start calibration
          cal = 1;           //set cal flag
          encoderPos = 0;    // reset encoder position
          encoder.write(0);  // reset zero encoder value
          debug("Longpress");
        }
      } else {
        if (changeMoveSpeed) {  //reload old values
          loadPointValues();
          changeMoveSpeed = 0;
        }
        pressCount = 1;
        lcd.setCursor(0, 0);
        lcd.print(pressCount);
        lcd.setCursor(0, 1);
        lcd.print(F("Cancel Can pt speed "));           //its a short press so count presses
        while (millis() - timepressed < (LONG_PUSH)) {  //if still within LONG_PUSH
          if (digitalRead(ENCODER_PUSH) == 0) {         //keeps looping until time expires
            while (digitalRead(ENCODER_PUSH) == 0) {}
            timepressed = millis();
            delay(20);  //debouince
            pressCount++;
            lcd.setCursor(0, 0);
            lcd.print(pressCount);
            lcd.setCursor(0, 1);
            switch (pressCount) {
              case 5:
                pressCount = 1;

              case 1:
                lcd.print(F("Cancel Can pt speed "));
                break;
              case 2:
                lcd.print(F("Point Pairing   "));
                lcd.setCursor(16, 1);
                pointPairing ? (lcd.print(F("OFF?"))) : (lcd.print(F("ON? ")));
                // pointPairing ? (lcd.print(F("ON "))) : (lcd.print(F("OFF")));
                break;
              case 3:
                lcd.print(F("Centre servo   "));
                lcd.setCursor(16, 1);
                centreServo ? (lcd.print(F("OFF?"))) : (lcd.print(F("ON? ")));
                break;
              case 4:
                lcd.print(F("Set changeMoveSpeed"));
                debugln("");
                break;
              default:
                pressCount = 1;
                break;
            }
          }
        }


        lcd.setCursor(17, 0);

        switch (pressCount) {
          case 1:
            changeMoveSpeed = false;
            debugln("Ignore or Reset changeMoveSpeed");
            lcd.clear();
            lcdGrid();
            lcdPos();
            break;
          case 2:
            debug(pointPairing);
            debug(" Toggled pointPairing = ");
            pointPairing = !pointPairing;
            debugln(pointPairing);
            savePointValues();
            lcd.clear();
            lcdGrid();
            lcdPos();
            break;
          case 3:
            debug(centreServo);
            centreServo = !centreServo;
            debug(" Toggled centreServo = ");
            debugln(centreServo);
            lcd.clear();
            lcdGrid();
            lcdPos();
            break;
          case 4:
            changeMoveSpeed = true;
            debugln("changeMoveSpeed");
            lcd.clear();
            lcd.setCursor(2, 0);
            lcd.print(F("Point Move Speed"));
            lcd.setCursor(0, 3);
            lcd.print(F("Long Save/Short Undo"));
            break;
          default:
            //lcd.print(F("Cancel"));
            debugln("Unhandled press count");
            lcd.clear();
            lcdGrid();
            lcdPos();
            delay(2000);
            break;
        }
      }
    } else {    //IF are CALIBRATING when encoder pushed and not chaning move speed
      cal = 0;  // stop calibrating whether long or short press
      lcdGrid();
      lcdPos();
      if (longPress) {
        savePointValues();
        debugln("Saving new settings");
      } else {
        loadPointValues();
        debugln("restoring old settings");
        for (int i = 0; i < NO_OF_SERVOS; i++) {
          point[i].MovePoint(i);
        }
      }
    }
  }
  if (cal) {  // at any time
    //move points to new positions
    //static uint8_t pointNum;
    int32_t move = (encoder.read() - encoderPos);            //current value - old value
    if (move != 0 || oldLastPointMoved != lastPointMoved) {  //there has been an encoder move since last pass (could be -ve) or new point switched
      move *= 4;
      debugln(move);
      debug(" moving point ");

      encoder.write(0);  // reset encoder

      if (point[lastPointMoved].target) {// recalibrating thrown position
        debug(" Thrown  ");
        point[lastPointMoved].thrownPos += move;// changing the thrown position by encoder turn  (+ or -)
        if (point[lastPointMoved].thrownPos < BOTTOM_PULSE_LEN) point[lastPointMoved].thrownPos = BOTTOM_PULSE_LEN;
        if (point[lastPointMoved].thrownPos > TOP_PULSE_LEN) point[lastPointMoved].thrownPos = TOP_PULSE_LEN;
        if (lastPointMoved < 16) {
          PCA1.writeMicroseconds(lastPointMoved, point[lastPointMoved].thrownPos);
        } else {
          PCA2.writeMicroseconds(lastPointMoved - 16, point[lastPointMoved].thrownPos);
        }
        point[lastPointMoved].curPos = point[lastPointMoved].thrownPos;

      } else {
        debug(" Closed ");
        point[lastPointMoved].closedPos += move;
        if (point[lastPointMoved].closedPos < BOTTOM_PULSE_LEN) point[lastPointMoved].closedPos = BOTTOM_PULSE_LEN;
        if (point[lastPointMoved].closedPos > TOP_PULSE_LEN) point[lastPointMoved].closedPos = TOP_PULSE_LEN;
        if (lastPointMoved < 16) {
          PCA1.writeMicroseconds(lastPointMoved, point[lastPointMoved].closedPos);
        } else {
          PCA2.writeMicroseconds(lastPointMoved - 16, point[lastPointMoved].closedPos);
        }
        point[lastPointMoved].curPos = point[lastPointMoved].closedPos;
      }
      Serial.println(lastPointMoved);
      oldLastPointMoved = lastPointMoved;
    }
  }
}
void testLeds() {
  for (int i = 0; i < NO_OF_SERVOS; i++) {
    leds[LEDS_MIMIC[i]] = CHSV(Hred, onSat, onLev);  // set all to red
    FastLED.show();
  }
}
void offLeds() {
  for (int i = 0; i < NO_OF_SERVOS; i++) {
    leds[LEDS_MIMIC[i]] = CHSV(Hred, onSat, 0);  // set all to red off
    FastLED.show();
  }
}
void setLeds() {
  flashTimeNow = millis();
  if (flashTimeNow > (flashTimeout)) {         // every 400ms change flash on to flash off or vice versa
    flash = !flash;                            // whatever flash state is (on or off), make it the opposite
    flashTimeout = flashTimeNow + flashSpeed;  //reset the timer
  }
  for (int i = 0; i < NO_OF_SERVOS; i++) {
    // loop through all points
    leds[LEDS_MIMIC[i]] = CHSV(Hred, onSat, onLev);  // set all to red
    if (!point[i].target) {
      leds[LEDS_MIMIC[i]] = CHSV(Hgre, onSat, onLev);  //set closed points to green
    }
    if (pointPairing) {
      if (POINT_PAIRS[i] != i) {
        leds[LEDS_MIMIC[i]] = CHSV(Hora, onSat, onLev);  // set all paired to orange
        if (!point[i].target) {
          leds[LEDS_MIMIC[i]] = CHSV(Hcya, onSat, onLev);  //set closed paired points to cyan
        }
      }
    }

    if (moving) {
      if (!point[i].target) {
        if (point[i].closedPos != point[i].curPos) {
          leds[LEDS_MIMIC[i]] = CHSV(Hblu, onSat, onLev);  //set points not in their correct position to blue (moving points)
        }
      } else {
        if (point[i].thrownPos != point[i].curPos) {
          leds[LEDS_MIMIC[i]] = CHSV(Hblu, onSat, onLev);  //set points not in their correct position to blue (moving points)
        }
      }
    }


    if (flash) {
      if (cal && i == lastPointMoved) {
        leds[LEDS_MIMIC[i]] = CHSV(Hpur, onSat, onLev);  //set points being calibrated to purple
      }
    }
  }
  FastLED.show();
}
void lcdGrid() {
  lcd.setCursor(0, 0);
  lcd.print(F("Pnt 0123456789012345"));
  lcd.setCursor(0, 1);
  lcd.print(F("Pos "));
  lcd.setCursor(0, 2);
  lcd.print(F("Pnt 6789012345678901"));
  lcd.setCursor(0, 3);
  lcd.print(F("Pos "));
}

void lcdPos() {
  lcd.setCursor(4, 1);
  for (int i = 0; i < 16; i++) {
    lcd.print(
      (pointPairing && POINT_PAIRS[i] != i) ?
        (point[i].target ? F("I") : F("\"")) :
        (point[i].target ? F("|") : F("-"))
    );
  }

  lcd.setCursor(4, 3);
  for (int i = 0; i < 16; i++) {
    lcd.print(
      (pointPairing && POINT_PAIRS[i + 16] != i+16) ?
        (point[i + 16].target ? F("I") : F("\"")) :
        (point[i + 16].target ? F("|") : F("-"))
    );
  }
}
void lcdPrint() {

  if (cal) {

    lcd.setCursor(0, 1);
    lcd.print("Point ");
    lcd.print(lastPointMoved);
    lcd.print("  ");
    lcd.setCursor(9, 1);

    if (point[lastPointMoved].target) {
      lcd.print(F("THROWN "));
    } else {
      lcd.print(F("CLOSED "));
    }
    // lcd.setCursor(14, 1);
    lcd.print(point[lastPointMoved].curPos);
    lcd.print(" ");
    // lcd.setCursor(1, 2);
    // lcd.print("Long Push to save");
  }
}
void pointMoveSpeed() {
  encoderPos = encoder.read();

  if (encoderPos < 0) {
    moveSpeed++;
  } else if (encoderPos > 0) {
    if (moveSpeed > 1) moveSpeed--;
  }
  encoder.write(0);
  lcd.setCursor(8, 2);
  lcd.print(moveSpeed);
  lcd.print(F("  "));
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  PCF1.begin();
  PCF2.begin();
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NO_OF_LEDS);

  Serial.println("Dual PCA9685 pcf8575 Servo test8 speedAdj");
  int nDevices = 0;
  offLeds();
  Serial.println("Scanning I2C ");

  for (byte address = 1; address < 127; ++address) {
    // The i2c_scanner uses the return value of
    // the Wire.endTransmission to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device at 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println("  !");

      ++nDevices;
    } else if (error == 4) {
      Serial.print("error at 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  lcd.init();  // initialize the lcd
  lcd.backlight();
  lcd.setCursor(2, 0);
  lcd.print("Point control v9");
  lcd.setCursor(2, 1);
  lcd.print("Steve Lomax 2025");
  delay(1000);


  for (int i = 0; i < 3; i++) {
    pinMode(pushSw[i], INPUT_PULLUP);
  }
  for (int i = 16; i < 18; i++) {
    pinMode(pushSw[i], INPUT_PULLUP);
  }

  pinMode(ENCODER_PUSH, INPUT_PULLUP);
  loadPointValues();

  PCA1.begin();
  PCA1.setPWMFreq(50);
  PCA1.setOscillatorFrequency(25000000);
  PCA2.begin();
  PCA2.setPWMFreq(50);
  PCA2.setOscillatorFrequency(25000000);



  PCF1.write16(0Xffff);
  debug("PCF1 = ");
  debugln2(PCF1.read16(), BIN);

  PCF2.write16(0Xffff);
  debug("PCF2 = ");
  debugln2(PCF2.read16(), BIN);

  lcd.setCursor(4, 3);
  lcd.print(F("Loading..."));
  startPos();
  delay(2000);

  lcd.clear();
  lcdGrid();
  lcdPos();
}


void loop() {
  scanButtons();
  scanPushButtons();
  lcdPrint();
  if (moving) {
    moving = 0;
    for (int i = 0; i < NO_OF_SERVOS; i++) {
      point[i].MovePoint(i);
    }
  } else {
    calibrate();
    if (changeMoveSpeed) pointMoveSpeed();
  }
  setLeds();
}

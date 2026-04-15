#include <Adafruit_MCP23X17.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include <EEPROM.h>
#include <Wire.h>
#include "RTClib.h"

RTC_DS3231 rtc;
bool rtc_ok = false;
Adafruit_MCP23X17 mcp;

/* ================= HARDWARE ================= */
LiquidCrystal lcd(6, 7, 2, 3, 4, 5);
Servo lockServo;
const int servoPin        = 8;
const int buzzerPin       = 13;
const int insideButtonPin = 10;
const int joystickX       = A2;
const int joystickBtn     = 9;
const int reedSwitchPin   = 11;  // LOW = magnet present (door closed)

/* ================= EEPROM MAP ================= */
#define EEPROM_USER_PIN     0
#define EEPROM_LOCK_FLAG   10
#define EEPROM_ATTEMPTS    11
#define EEPROM_ADMIN_PIN   20
#define EEPROM_MAGIC       50
#define EEPROM_LOG_INDEX   55
#define EEPROM_LOG_START   60
#define EEPROM_MAGIC_VAL   0xA5

/* ================= LOGGING ================= */
#define LOG_SIZE        8
#define MAX_LOGS        25
#define EVT_ADMIN_FAIL  1
#define EVT_LOCKOUT     2

/* ================= KEYPAD ================= */
char keys[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
int rowPins[4] = {0, 1, 2, 3}; // GPA0-GPA3
int colPins[4] = {4, 5, 6, 7}; // GPA4-GPA7

/* ================= ADMIN MENU ================= */
const char* menuItems[] = {
  "Clear Logs",
  "Export Logs",
  "Set RTC",
  "Change User PIN",
  "Exit Menu"
};
const byte MENU_ITEM_COUNT = 5;
byte menuIndex = 0;

// Joystick state
bool joystickMoved               = false;
bool joystickBtnLast             = HIGH;
unsigned long joystickBtnTime    = 0;
unsigned long joystickHoldStart  = 0;
unsigned long joystickScrollTime = 0;
const unsigned long BTN_DEBOUNCE          = 50;
const unsigned long SCROLL_INITIAL_DELAY  = 500;
const unsigned long SCROLL_REPEAT_INTERVAL = 150;

/* ================= LONG PRESS (A key) ================= */
unsigned long aPressStart       = 0;
bool          aHeld             = false;
const unsigned long A_LONG_PRESS = 1500;

/* ================= MODES ================= */
enum Mode {
  MODE_USER,
  MODE_CHANGE_PIN,
  MODE_ADMIN_CHANGE_PIN,
  MODE_ADMIN_UNLOCK,
  MODE_ADMIN_MENU,
  MODE_SET_RTC,
  MODE_CONFIRM_CLEAR
};
Mode mode = MODE_USER;

/* ================= PIN BUFFERS ================= */
char userPIN[5];
char adminPIN[5];
char input[5];      byte inputLen = 0;
char tempNewPIN[5]; byte tempNewLen = 0;

/* ================= LOCKOUT ================= */
byte attempts = 0;
const byte MAX_ATTEMPTS = 3;
bool lockedOut = false;

/* ================= MESSAGE ================= */
bool showMessage = false;
unsigned long msgStart = 0;
const unsigned long MSG_TIME = 2000;

/* ================= SERVO ================= */
bool servoActive = false;
bool doorOpened  = false;  // must go HIGH (open) before we watch for close

/* ================= SHOW PASSWORD ================= */
bool showPassword = false;

/* ================= CHANGE PIN STATE ================= */
enum ChangePINStep {
  STEP_NONE,
  STEP_OLD,
  STEP_NEW,
  STEP_CONFIRM
};
ChangePINStep changeStep = STEP_NONE;

/* ================= RTC SET STATE ================= */
enum RTCStep {
  RTC_NONE,
  RTC_YEAR,
  RTC_MONTH,
  RTC_DAY,
  RTC_HOUR,
  RTC_MIN,
  RTC_SEC,
  RTC_CONFIRM
};
RTCStep rtcStep = RTC_NONE;
int rtcYear, rtcMonth, rtcDay, rtcHour, rtcMin, rtcSec;

/* ================= SPAM PROTECTION ================= */
const byte SPAM_WARNING_LIMIT = 8;
const byte SPAM_LIMIT = 12;
unsigned long lastKeyTime = 0;
const unsigned long SPAM_TIME_WINDOW = 400;
byte spamCount = 0;
bool spamWarned = false;

/* ================= SERIAL ADMIN CHANGE ================= */
#define ADMIN_CHANGE_CMD 'A'

/* ================= FORWARD DECLARATIONS ================= */
void logEvent(byte type);
void exportLogs();
void clearLogs();
void drawAdminMenu();

/* ================= HELPERS ================= */
void beepClick(int times, int duration){
  if(!buzzerAllowed()) return;
  for(int i=0;i<times;i++){
    digitalWrite(buzzerPin,HIGH);
    delay(duration);
    digitalWrite(buzzerPin,LOW);
    delay(50);
  }
}
void beepSuccess(){ beepClick(3,100); }
void beepFail(){    beepClick(1,400); }

void keyClick(){
  if(!buzzerAllowed()) return;
  digitalWrite(buzzerPin,HIGH);
  delay(6);
  digitalWrite(buzzerPin,LOW);
}

void showStarsOrPassword(char *buf, byte len){
  lcd.setCursor(0,1);
  lcd.print("                ");
  lcd.setCursor(0,1);
  for(byte i=0;i<len;i++){
    lcd.print(showPassword ? buf[i] : '*');
  }
}

void clearBuf(char *buf, byte &len){
  len = 0;
  buf[0] = '\0';
}

void saveAttempts(){ EEPROM.update(EEPROM_ATTEMPTS, attempts); }
void saveLockout(bool s){ EEPROM.update(EEPROM_LOCK_FLAG, s); }

void clearLockout(){
  lockedOut  = false;
  attempts   = 0;
  spamCount  = 0;
  spamWarned = false;
  saveAttempts();
  saveLockout(false);
}

void saveUserPIN(){
  for(byte i=0;i<4;i++) EEPROM.update(EEPROM_USER_PIN+i, userPIN[i]);
}

void saveAdminPIN(){
  for(byte i=0;i<4;i++) EEPROM.update(EEPROM_ADMIN_PIN+i, adminPIN[i]);
}

bool buzzerAllowed(){
  if(!rtc_ok) return true;  // no RTC — always beep
  DateTime now = rtc.now();
  int h = now.hour();
  return (h >= 7 && h < 21);  // 7:00 AM to 8:59 PM
}

void startServo(){
  lockServo.write(90);
  servoActive = true;
  doorOpened  = false;
}

/* ================= RTC CHECK ================= */
bool checkRTC(){
  Wire.beginTransmission(0x68);
  return (Wire.endTransmission() == 0);
}

/* ================= EEPROM TIMESTAMPED LOG ================= */
void logEvent(byte type){
  byte idx = EEPROM.read(EEPROM_LOG_INDEX);
  if(idx >= MAX_LOGS) idx = 0;
  int addr = EEPROM_LOG_START + idx * LOG_SIZE;

  if(rtc_ok){
    DateTime now = rtc.now();
    EEPROM.update(addr+0, type);
    EEPROM.update(addr+1, now.year()-2000);
    EEPROM.update(addr+2, now.month());
    EEPROM.update(addr+3, now.day());
    EEPROM.update(addr+4, now.hour());
    EEPROM.update(addr+5, now.minute());
    EEPROM.update(addr+6, now.second());
    EEPROM.update(addr+7, 0);
  } else {
    EEPROM.update(addr+0, type);
    for(int i=1;i<8;i++) EEPROM.update(addr+i, 0);
  }

  idx++;
  if(idx >= MAX_LOGS) idx = 0;
  EEPROM.update(EEPROM_LOG_INDEX, idx);
}

void clearLogs(){
  EEPROM.update(EEPROM_LOG_INDEX, 0);
  for(int i=0;i<MAX_LOGS*LOG_SIZE;i++)
    EEPROM.update(EEPROM_LOG_START+i, 0xFF);
}

void exportLogs(){
  Serial.println(F("=== LOG EXPORT ==="));
  bool found = false;

  for(byte i=0;i<MAX_LOGS;i++){
    int addr = EEPROM_LOG_START + i*LOG_SIZE;
    byte t = EEPROM.read(addr);
    if(t==0 || t==0xFF) continue;
    found = true;

    byte y  = EEPROM.read(addr+1);
    byte m  = EEPROM.read(addr+2);
    byte d  = EEPROM.read(addr+3);
    byte h  = EEPROM.read(addr+4);
    byte mi = EEPROM.read(addr+5);
    byte s  = EEPROM.read(addr+6);

    String typeStr = (t==EVT_ADMIN_FAIL) ? "ADMIN_FAIL" : "LOCKOUT";
    Serial.print(F("LOG #")); Serial.print(i+1);
    Serial.print(F("  [")); Serial.print(typeStr); Serial.print(F("]  "));
    Serial.print(2000+y); Serial.print(F("-"));
    if(m<10) Serial.print("0"); Serial.print(m); Serial.print(F("-"));
    if(d<10) Serial.print("0"); Serial.print(d); Serial.print(F(" "));
    if(h<10) Serial.print("0"); Serial.print(h); Serial.print(F(":"));
    if(mi<10) Serial.print("0"); Serial.print(mi); Serial.print(F(":"));
    if(s<10) Serial.print("0"); Serial.println(s);
  }

  if(!found) Serial.println(F("NO LOGS TO EXPORT"));
  Serial.println(F("=== END EXPORT ==="));
}

/* ================= ADMIN MENU DISPLAY ================= */
// Row 0: "> Current Item"
// Row 1: "  Next Item" (context / peek ahead)
void drawAdminMenu(){
  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print("> ");
  lcd.print(menuItems[menuIndex]);

  // Find next valid item for context row
  byte next  = (menuIndex + 1) % MENU_ITEM_COUNT;
  byte tries = 0;
  while(tries < MENU_ITEM_COUNT){
    if(next==2 && !rtc_ok){ next=(next+1)%MENU_ITEM_COUNT; tries++; continue; }
    break;
  }
  if(next != menuIndex){
    lcd.setCursor(0,1);
    lcd.print("  ");
    lcd.print(menuItems[next]);
  }
}

/* ================= INPUT HANDLER ================= */
void handleInputKey(char key, char *buf, byte &len){
  if(key>='0' && key<='9' && len<4){
    buf[len++] = key;
    buf[len]   = '\0';
  }
  else if(key=='C'){
    clearBuf(buf, len);
  }
  else if(key=='*'){
    showPassword = !showPassword;
  }
  showStarsOrPassword(buf, len);
}

/* ================= RTC DISPLAY HELP ================= */
void showRTCStep(RTCStep s){
  lcd.clear();
  switch(s){
    case RTC_YEAR:    lcd.print("Set Year (YY):"); break;
    case RTC_MONTH:   lcd.print("Set Month:");     break;
    case RTC_DAY:     lcd.print("Set Day:");       break;
    case RTC_HOUR:    lcd.print("Set Hour:");      break;
    case RTC_MIN:     lcd.print("Set Min:");       break;
    case RTC_SEC:     lcd.print("Set Sec:");       break;
    case RTC_CONFIRM: lcd.print("Press # to Save");break;
    default:          lcd.print("RTC SET");        break;
  }
}

/* ================= VALID PIN CHECK ================= */
bool isValidPIN(const char* pin){
  for(int i=0;i<4;i++){
    if(pin[i]<'0' || pin[i]>'9') return false;
  }
  return true;
}

/* ================= SETUP ================= */
void setup(){
  Serial.begin(9600);
  while(!Serial){ }
  delay(100);

  Wire.begin();
  mcp.begin_I2C(0x20);

  for(int r=0;r<4;r++){
    mcp.pinMode(rowPins[r], OUTPUT);
    mcp.digitalWrite(rowPins[r], HIGH);
  }
  for(int c=0;c<4;c++){
    mcp.pinMode(colPins[c], INPUT_PULLUP);
  }

  rtc_ok = checkRTC();
  if(rtc_ok){
    rtc.begin();
    if(rtc.lostPower())
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  lcd.begin(16,2);
  lockServo.attach(servoPin);
  pinMode(buzzerPin,       OUTPUT);
  pinMode(insideButtonPin, INPUT_PULLUP);
  pinMode(joystickBtn,     INPUT_PULLUP);
  pinMode(reedSwitchPin,   INPUT_PULLUP);
  // joystickX (A2) is analog — no pinMode needed

  if(EEPROM.read(EEPROM_MAGIC) != EEPROM_MAGIC_VAL){
    EEPROM.update(EEPROM_MAGIC,      EEPROM_MAGIC_VAL);
    EEPROM.update(EEPROM_LOCK_FLAG,  0);
    EEPROM.update(EEPROM_ATTEMPTS,   0);
    EEPROM.update(EEPROM_LOG_INDEX,  0);
    for(byte i=0;i<4;i++){
      EEPROM.update(EEPROM_USER_PIN+i,  '1');
      EEPROM.update(EEPROM_ADMIN_PIN+i, '0');
    }
    for(int i=0;i<MAX_LOGS*LOG_SIZE;i++)
      EEPROM.update(EEPROM_LOG_START+i, 0xFF);
  }

  for(byte i=0;i<4;i++){
    char c = EEPROM.read(EEPROM_USER_PIN+i);
    userPIN[i]  = (c>='0'&&c<='9') ? c : '1';
    c = EEPROM.read(EEPROM_ADMIN_PIN+i);
    adminPIN[i] = (c>='0'&&c<='9') ? c : '0';
  }
  userPIN[4] = adminPIN[4] = '\0';

  lockedOut = EEPROM.read(EEPROM_LOCK_FLAG);
  attempts  = EEPROM.read(EEPROM_ATTEMPTS);

  lockServo.write(0);
  lcd.print(lockedOut ? "LOCKOUT" : "Enter Password:");
}

/* ================= LOOP ================= */
void loop(){
  unsigned long nowMs = millis();

  // SERVO — wait for door to open, then lock when it closes
  if(servoActive){
    bool magnet = digitalRead(reedSwitchPin)==HIGH;  // HIGH = magnet present = closed
    if(!magnet) doorOpened = true;          // door has opened (magnet gone)
    if(doorOpened && magnet){               // door opened then closed again
      lockServo.write(0);
      servoActive = false;
      doorOpened  = false;
    }
  }

  // MESSAGE TIMER
  if(showMessage && nowMs-msgStart>=MSG_TIME){
    showMessage = false;
    lcd.clear();
    if(mode==MODE_ADMIN_MENU){
      drawAdminMenu();
    } else {
      lcd.print(lockedOut ? "LOCKOUT" : "Enter Password:");
    }
  }

  // INSIDE BUTTON UNLOCK
  if(digitalRead(insideButtonPin) == LOW){
    startServo();
    beepSuccess();
    delay(300);
  }

  // ===== SERIAL ADMIN PIN CHANGE =====
  if(Serial.available()){
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if(cmd.length()==10 && cmd[0]==ADMIN_CHANGE_CMD && cmd[5]==':'){
      String oldPin = cmd.substring(1,5);
      String newPin = cmd.substring(6,10);
      if(oldPin==String(adminPIN) && isValidPIN(newPin.c_str())){
        for(int i=0;i<4;i++){
          adminPIN[i] = newPin[i];
          EEPROM.update(EEPROM_ADMIN_PIN+i, adminPIN[i]);
        }
        Serial.println(F("Admin PIN updated successfully."));
      } else {
        Serial.println(F("Admin PIN change failed."));
      }
    } else if(cmd.length()>0){
      Serial.println(F("Invalid command."));
    }
  }

  // ===== JOYSTICK (admin menu only) =====
  if(mode==MODE_ADMIN_MENU){
    int xVal = analogRead(joystickX);

    if(xVal > 700 || xVal < 300){
      bool doScroll = false;
      if(!joystickMoved){
        joystickMoved      = true;
        joystickHoldStart  = nowMs;
        joystickScrollTime = nowMs;
        doScroll = true;
      } else if(nowMs - joystickHoldStart  >= SCROLL_INITIAL_DELAY &&
                nowMs - joystickScrollTime >= SCROLL_REPEAT_INTERVAL){
        joystickScrollTime = nowMs;
        doScroll = true;
      }

      if(doScroll){
        if(xVal > 700){
          byte next  = (menuIndex+1) % MENU_ITEM_COUNT;
          byte tries = 0;
          while(tries < MENU_ITEM_COUNT){
            if(next==2 && !rtc_ok){ next=(next+1)%MENU_ITEM_COUNT; tries++; continue; }
            break;
          }
          menuIndex = next;
        } else {
          byte prev  = (menuIndex+MENU_ITEM_COUNT-1) % MENU_ITEM_COUNT;
          byte tries = 0;
          while(tries < MENU_ITEM_COUNT){
            if(prev==2 && !rtc_ok){ prev=(prev+MENU_ITEM_COUNT-1)%MENU_ITEM_COUNT; tries++; continue; }
            break;
          }
          menuIndex = prev;
        }
        drawAdminMenu();
        beepClick(1,20);
      }
    }
    // Stick returned to centre — allow next move
    else {
      joystickMoved = false;
    }

    // Joystick button select
    bool btnNow = digitalRead(joystickBtn);
    if(btnNow==LOW && joystickBtnLast==HIGH && nowMs-joystickBtnTime>BTN_DEBOUNCE){
      joystickBtnTime = nowMs;
      switch(menuIndex){
        case 0: // Clear Logs
          mode = MODE_CONFIRM_CLEAR;
          lcd.clear();
          lcd.print("Confirm Clear?");
          lcd.setCursor(0,1);
          lcd.print("A=YES  C=NO");
          break;
        case 1: // Export Logs
          lcd.clear();
          lcd.print("EXPORTING...");
          exportLogs();
          lcd.setCursor(0,1);
          lcd.print("DONE");
          showMessage = true;
          msgStart    = nowMs;
          mode        = MODE_USER;
          break;
        case 2: // Set RTC
          mode    = MODE_SET_RTC;
          rtcStep = RTC_YEAR;
          showRTCStep(rtcStep);
          clearBuf(input, inputLen);
          break;
        case 3: // Change User PIN (admin — no old PIN required)
          mode       = MODE_ADMIN_CHANGE_PIN;
          changeStep = STEP_NEW;
          clearBuf(input, inputLen);
          lcd.clear();
          lcd.print("New User PIN:");
          break;
        case 4: // Exit Menu
          mode = MODE_USER;
          lcd.clear();
          lcd.print("Enter Password:");
          break;
      }
    }
    joystickBtnLast = btnNow;
  }

  // ===== KEYPAD SCAN =====
  char key = 0;
  for(int r=0;r<4;r++){
    for(int i=0;i<4;i++) mcp.digitalWrite(rowPins[i], HIGH);
    mcp.digitalWrite(rowPins[r], LOW);
    for(int c=0;c<4;c++){
      if(mcp.digitalRead(colPins[c])==LOW){
        key = keys[r][c];
        delay(200);
      }
    }
  }

  // ===== LONG PRESS TRACKING FOR A =====
  if(key=='A' && mode!=MODE_ADMIN_MENU && mode!=MODE_ADMIN_UNLOCK){
    if(!aHeld){
      aHeld      = true;
      aPressStart = nowMs;
    } else if(nowMs - aPressStart >= A_LONG_PRESS){
      // Long press confirmed — enter admin unlock
      aHeld = false;
      if(lockedOut || mode==MODE_USER){
        mode = MODE_ADMIN_UNLOCK;
        clearBuf(input, inputLen);
        lcd.clear();
        lcd.print("Admin PIN:");
        beepClick(2, 50);
      }
    }
    return;  // don't process A as a normal key
  } else if(key!='A'){
    aHeld = false;  // released or different key — reset
  } else {
    return;  // key=='A' in admin menu/unlock — swallow it
  }

  if(!key) return;
  keyClick();

  // D always cancels back to user screen
  if(key=='D'){
    mode       = MODE_USER;
    changeStep = STEP_NONE;
    rtcStep    = RTC_NONE;
    clearBuf(input, inputLen);
    clearBuf(tempNewPIN, tempNewLen);
    lcd.clear();
    lcd.print(lockedOut ? "LOCKOUT" : "Enter Password:");
    return;
  }

  // SPAM PROTECTION
  if(mode!=MODE_SET_RTC && !lockedOut && key>='0' && key<='9'){
    unsigned long now = millis();
    if(now-lastKeyTime < SPAM_TIME_WINDOW) spamCount++;
    else spamCount = 0;
    lastKeyTime = now;

    if(!spamWarned && spamCount>=SPAM_WARNING_LIMIT){
      spamWarned = true;
      lcd.clear();
      lcd.print("STOP SPAMMING");
      lcd.setCursor(0,1);
      lcd.print("Lockout soon");
      beepFail();
      showMessage = true;
      msgStart    = nowMs;
    }
    if(spamCount>=SPAM_LIMIT){
      lockedOut = true;
      saveLockout(true);
      logEvent(EVT_LOCKOUT);
      lcd.clear();
      lcd.print("LOCKOUT");
      beepFail();
      return;
    }
  }

  // LOCKOUT gate — blocks all keypad input when locked out
  if(lockedOut && mode!=MODE_ADMIN_UNLOCK) return;

  // ADMIN UNLOCK MODE
  if(mode==MODE_ADMIN_UNLOCK){
    if(key=='#'){
      lcd.clear();
      if(strcmp(input,adminPIN)==0){
        clearLockout();
        startServo();
        beepSuccess();
        mode = MODE_USER;
        lcd.print("Enter Password:");
      } else {
        lcd.print("BAD ADMIN PIN");
        beepFail();
        logEvent(EVT_ADMIN_FAIL);
        showMessage = true;
        msgStart    = nowMs;
        mode        = MODE_USER;
      }
      clearBuf(input, inputLen);
      return;
    }
    handleInputKey(key, input, inputLen);
    return;
  }

  // Keypad has no role in the admin menu — joystick handles it
  if(mode==MODE_ADMIN_MENU) return;

  // CONFIRM CLEAR MODE
  if(mode==MODE_CONFIRM_CLEAR){
    if(key=='A'){
      clearLogs();
      lcd.clear();
      lcd.print("Logs Cleared");
      beepSuccess();
      showMessage = true;
      msgStart    = nowMs;
      mode        = MODE_ADMIN_MENU;
      menuIndex   = 0;
      return;
    }
    if(key=='C'){
      lcd.clear();
      lcd.print("Cancelled");
      showMessage = true;
      msgStart    = nowMs;
      mode        = MODE_ADMIN_MENU;
      return;
    }
    return;
  }

  // RTC SET MODE
  if(mode==MODE_SET_RTC){
    if(key=='#'){
      if(rtcStep==RTC_CONFIRM){
        rtc.adjust(DateTime(rtcYear,rtcMonth,rtcDay,rtcHour,rtcMin,rtcSec));
        lcd.clear();
        lcd.print("RTC SET OK");
        beepSuccess();
        showMessage = true;
        msgStart    = nowMs;
        mode        = MODE_ADMIN_MENU;
        rtcStep     = RTC_NONE;
        return;
      }

      int val = atoi(input);
      bool valid = (inputLen > 0);
      if(valid){
        if     (rtcStep==RTC_YEAR)  valid = (val >= 0 && val <= 99);
        else if(rtcStep==RTC_MONTH) valid = (val >= 1 && val <= 12);
        else if(rtcStep==RTC_DAY)   valid = (val >= 1 && val <= 31);
        else if(rtcStep==RTC_HOUR)  valid = (val >= 0 && val <= 23);
        else if(rtcStep==RTC_MIN)   valid = (val >= 0 && val <= 59);
        else if(rtcStep==RTC_SEC)   valid = (val >= 0 && val <= 59);
      }

      if(!valid){
        lcd.clear();
        lcd.print("Invalid value");
        beepFail();
        delay(800);
        showRTCStep(rtcStep);
        clearBuf(input, inputLen);
        return;
      }

      if     (rtcStep==RTC_YEAR)  { rtcYear  = val+2000; rtcStep=RTC_MONTH;   }
      else if(rtcStep==RTC_MONTH) { rtcMonth = val;      rtcStep=RTC_DAY;     }
      else if(rtcStep==RTC_DAY)   { rtcDay   = val;      rtcStep=RTC_HOUR;    }
      else if(rtcStep==RTC_HOUR)  { rtcHour  = val;      rtcStep=RTC_MIN;     }
      else if(rtcStep==RTC_MIN)   { rtcMin   = val;      rtcStep=RTC_SEC;     }
      else if(rtcStep==RTC_SEC)   { rtcSec   = val;      rtcStep=RTC_CONFIRM; }

      showRTCStep(rtcStep);
      clearBuf(input, inputLen);
      return;
    }
    handleInputKey(key, input, inputLen);
    return;
  }

  // ADMIN CHANGE PIN MODE (no old-PIN check)
  if(mode==MODE_ADMIN_CHANGE_PIN){
    if(key=='#'){
      if(changeStep==STEP_NEW){
        if(inputLen==4){
          strcpy(tempNewPIN, input);
          tempNewLen = inputLen;
          clearBuf(input, inputLen);
          changeStep = STEP_CONFIRM;
          lcd.clear();
          lcd.print("Confirm PIN:");
        } else {
          lcd.clear();
          lcd.print("Need 4 digits");
          beepFail();
          clearBuf(input, inputLen);
          showMessage = true;
          msgStart    = nowMs;
        }
      }
      else if(changeStep==STEP_CONFIRM){
        if(strcmp(input,tempNewPIN)==0){
          strcpy(userPIN, tempNewPIN);
          saveUserPIN();
          lcd.clear();
          lcd.print("PIN Saved");
          beepSuccess();
        } else {
          lcd.clear();
          lcd.print("PIN Mismatch");
          beepFail();
        }
        clearBuf(input, inputLen);
        clearBuf(tempNewPIN, tempNewLen);
        changeStep  = STEP_NONE;
        showMessage = true;
        msgStart    = nowMs;
        mode        = MODE_ADMIN_MENU;
      }
      return;
    }
    handleInputKey(key, input, inputLen);
    return;
  }

  // USER CHANGE PIN MODE (requires old PIN)
  if(mode==MODE_CHANGE_PIN){
    if(key=='#'){
      if(changeStep==STEP_OLD){
        if(strcmp(input,userPIN)==0){
          changeStep = STEP_NEW;
          clearBuf(input, inputLen);
          lcd.clear();
          lcd.print("New PIN:");
        } else {
          lcd.clear();
          lcd.print("Bad Old PIN");
          beepFail();
          clearBuf(input, inputLen);
          showMessage = true;
          msgStart    = nowMs;
          mode        = MODE_USER;
        }
      }
      else if(changeStep==STEP_NEW){
        strcpy(tempNewPIN, input);
        tempNewLen = inputLen;
        clearBuf(input, inputLen);
        changeStep = STEP_CONFIRM;
        lcd.clear();
        lcd.print("Confirm PIN:");
      }
      else if(changeStep==STEP_CONFIRM){
        if(strcmp(input,tempNewPIN)==0){
          strcpy(userPIN, tempNewPIN);
          saveUserPIN();
          lcd.clear();
          lcd.print("PIN Saved");
          beepSuccess();
        } else {
          lcd.clear();
          lcd.print("PIN Mismatch");
          beepFail();
        }
        clearBuf(input, inputLen);
        clearBuf(tempNewPIN, tempNewLen);
        changeStep  = STEP_NONE;
        showMessage = true;
        msgStart    = nowMs;
        mode        = MODE_USER;
      }
      return;
    }
    handleInputKey(key, input, inputLen);
    return;
  }

  // LOCKED OUT
  if(lockedOut) return;

  // USER MODE PIN ENTRY
  if(key=='#'){
    lcd.clear();
    if(strcmp(input,userPIN)==0){
      lcd.print("Access Granted");
      startServo();
      attempts = 0;
      saveAttempts();
      beepSuccess();
    } else {
      attempts++;
      saveAttempts();
      if(attempts>=MAX_ATTEMPTS){
        lockedOut = true;
        saveLockout(true);
        logEvent(EVT_LOCKOUT);
        lcd.print("LOCKOUT");
        beepFail();
      } else {
        lcd.print("Access Denied");
        beepFail();
      }
    }
    clearBuf(input, inputLen);
    showMessage = !lockedOut;
    msgStart    = nowMs;
    return;
  }

  handleInputKey(key, input, inputLen);
}

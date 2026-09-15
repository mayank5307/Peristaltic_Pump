// include the library code:
#include <LiquidCrystal.h> // Standard parallel LCD library
#include <EEPROM.h> // Write and read EEPROM (to save and load settings)

// LCD PARALLEL SETUP -------------------------------------------------------------------
#define LCD_COLUMNS 16
#define LCD_ROWS 2

// Pin layout: LiquidCrystal lcd(RS, E, D4, D5, D6, D7);
const int rs = 13, en = 12, d4 = 8, d5 = 9, d6 = 10, d7 = 11;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// JOYSTICK PINS & HARDWARE SETUP --------------------------------------------------------
#define JOYSTICK_NAV_PIN A1    // Axis mapped to Menu Navigation (Up / Down through list)
#define JOYSTICK_VALUE_PIN A0  // Axis mapped to Parameter Values (Increase / Decrease)
#define JOYSTICK_SW_PIN 2      // Select / Enter button

// EXPANDED THRESHOLDS (Center ~340, Min ~217, Max ~682)
#define JOYSTICK_LOW_THRESH 240
#define JOYSTICK_HIGH_THRESH 460

// Navigation timing & state tracking
boolean nav_axis_centered = true;
boolean value_axis_centered = true;
unsigned long last_val_hold_ms = 0;
unsigned long val_hold_start_ms = 0; // Tracks when joystick push started for acceleration
const unsigned long HOLD_REPEAT_DELAY = 150; // Repeat rate in ms

// Release settling timer to ignore mechanical bounce upon releasing the stick
unsigned long last_val_release_ms = 0;
unsigned long last_nav_release_ms = 0;
const unsigned long JOYSTICK_RELEASE_SETTLE_MS = 250; // Ignore inputs for 250ms after centering

// Button debouncing state
boolean last_button_state = HIGH;
unsigned long last_button_press_ms = 0;
const unsigned long BUTTON_DEBOUNCE_DELAY = 200; // ms debounce window

// STEP MOTOR -----------------------------------------------------------------------------
#define MOTOR_STEP_PIN 7
#define MOTOR_DIR_PIN 6
#define STEP_MODE 4 // (1: Full Step, 2: Half Step, 4: Quarter Step, ...)
#define STEPS_PER_FULL_ROT 200 // @ full steps
long delay_us;
long steps;
long step_counter = 0;

// CONTINUOUS BOTTLE FILLING STATE --------------------------------------------------------
unsigned long fill_state_ms = 0;
boolean is_filling_phase = true; // true = rotating (filling), false = resting (bottle change)
int current_bottle_count = 0;
int last_printed_count = -1;    // Tracks LCD counter updates

// CALIBRATION -----------------------------------------------------------------------------
#define CALIBR_ROTATIONS 30
#define CALIBR_DURATION 30 // seconds
#define CALIBR_DECIMALS 3
const int CALIBR_DECIMAL_CORR = pow(10,CALIBR_DECIMALS);

// SERIAL COMMUNICATION ---------------------------------------------------------------------
#define BAUD 9600
String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete
long vol_uL=0;
long rate_uL_min =0;
int cal=0;
boolean usb_start=0;
char inChar;

// STATE ------------------------------------------------------------------------------------
boolean in_menu=0;
volatile boolean in_action=0;
boolean menu_entered=0;
boolean menu_left=0;

// GENERAL -----------------------------------------------------------------------------------
#define MICROSEC_PER_SEC 1000000

const unsigned int CALIBR_STEPS = CALIBR_ROTATIONS * STEPS_PER_FULL_ROT * STEP_MODE;
const unsigned int CALIBR_DELAY_US = (CALIBR_DURATION * MICROSEC_PER_SEC)/(CALIBR_STEPS*2);

// MENU ---------------------------------------------------------------------------------------
#define NUM_OF_MENU_ITEMS 11
#define VALUE_MAX_DIGITS 4
int menu_number_1=0;
int menu_number_2=1;
boolean val_change =0;
double value_dbl;
char value_str[VALUE_MAX_DIGITS+1];

enum menu_type {
  VALUE,
  OPTION,
  ACTION
};

typedef struct 
{
  const char* name_;
  menu_type type; //0: value type, 1:option type, 2:action type
  int value;
  int decimals;
  int lim;
  const char* options[4];
  const char* suffix;
} menu_item;

int menu_items_limit = NUM_OF_MENU_ITEMS - 1;
menu_item menu[NUM_OF_MENU_ITEMS];

// SETUP ----------------------------------------------------------------------------------
void setup(){
  
  pinMode(MOTOR_STEP_PIN, OUTPUT); 
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, LOW);
  digitalWrite(MOTOR_STEP_PIN, LOW);

  // Configure Joystick SW Pin with Internal Pull-up
  pinMode(JOYSTICK_SW_PIN, INPUT_PULLUP);

  menu[0].name_ = "Start";
  menu[0].type = ACTION;
  menu[0].value = 0;
  menu[0].lim = 0;
  menu[0].suffix = "ON!";

  menu[1].name_ = "Volume";
  menu[1].type = VALUE;
  menu[1].value = 100; // Default: 10.0 mL
  menu[1].decimals = 1;
  menu[1].lim = 9999;
  menu[1].suffix = "mL";

  menu[2].name_ = "Speed";
  menu[2].type = VALUE;
  menu[2].value = 200; // Default: 200 rpm
  menu[2].decimals = 0; // RPM displays as integer
  menu[2].lim = 400;   // Max: 400 rpm
  menu[2].suffix = "rpm";

  // Menu item 3: F. Time (Filling Time)
  menu[3].name_ = "F.Time";
  menu[3].type = VALUE;
  menu[3].value = 20; // Default: 2.0s
  menu[3].decimals = 1;
  menu[3].lim = 999;
  menu[3].suffix = "s";

  // Menu item 4: S. Time (Stop Time)
  menu[4].name_ = "S.Time";
  menu[4].type = VALUE;
  menu[4].value = 30; // Default: 3.0s
  menu[4].decimals = 1;
  menu[4].lim = 999;
  menu[4].suffix = "s";

  // Menu item 5: Count (Number of bottles)
  menu[5].name_ = "Count";
  menu[5].type = VALUE;
  menu[5].value = 5; // Default: 5
  menu[5].decimals = 0;
  menu[5].lim = 999;
  menu[5].suffix = "pcs";

  // Menu item 6: Direction
  menu[6].name_ = "Direction:";
  menu[6].type = OPTION;
  menu[6].value = 0;
  menu[6].lim = 2-1;
  menu[6].options[0] = "CW";
  menu[6].options[1] = "CCW";

  // Menu item 7: Mode
  menu[7].name_ = "Mode:";
  menu[7].type = OPTION;
  menu[7].value = 0;
  menu[7].lim = 4-1;
  menu[7].options[0] = "Dose";
  menu[7].options[1] = "Pump";
  menu[7].options[2] = "Cal.";
  menu[7].options[3] = "Cont";

  // Menu item 8: Cal.
  menu[8].name_ = "Cal.";
  menu[8].type = VALUE;
  menu[8].value = 10000; // Fixed factor (10.000 mL)
  menu[8].decimals = CALIBR_DECIMALS;
  menu[8].lim = 20000;
  menu[8].suffix = "mL";

  // Menu item 9: Save Settings
  menu[9].name_ = "Save Sett.";
  menu[9].type = ACTION;
  menu[9].value = 0;
  menu[9].lim = 0;
  menu[9].suffix = "OK!";

  // Menu item 10: USB Control
  menu[10].name_ = "USB Ctrl";
  menu[10].type = ACTION;
  menu[10].value = 0;
  menu[10].lim = 0;
  menu[10].suffix = "ON!";

  // Read saved settings from EEPROM
  for (int i=0; i <= menu_items_limit; i++){
      menu[i].value = eepromReadInt(i*2);
  }
  
  // Hardcoded default overrides if EEPROM is uninitialized
  if (menu[1].value == 0) menu[1].value = 100; // 10.0 mL
  if (menu[2].value == 0 || menu[2].value > 400) menu[2].value = 200; // 200 rpm
  if (menu[3].value == 0) menu[3].value = 20;  // 2.0s F. Time
  if (menu[4].value == 0) menu[4].value = 30;  // 3.0s S. Time
  if (menu[5].value == 0) menu[5].value = 5;   // 5 Count
  if (menu[8].value == 0) menu[8].value = 10000;
  
  eepromWriteInt(1 * 2, menu[1].value);
  eepromWriteInt(2 * 2, menu[2].value);
  eepromWriteInt(3 * 2, menu[3].value);
  eepromWriteInt(4 * 2, menu[4].value);
  eepromWriteInt(5 * 2, menu[5].value);
  eepromWriteInt(8 * 2, menu[8].value);

  // Initialize physical motor direction pin immediately
  digitalWrite(MOTOR_DIR_PIN, (menu[6].value == 0) ? LOW : HIGH);

  Serial.begin(BAUD);
  inputString.reserve(200);

  // Initialize standard LCD
  lcd.begin(LCD_COLUMNS, LCD_ROWS);

  update_lcd();
  steps = steps_calc(menu[1].value, 0, menu[8].value, menu[1].decimals);
  delay_us = delay_us_calc(menu[2].value, 2, menu[8].value, menu[2].decimals);
}

// LOOP -----------------------------------------------------------------------------------
void loop() {
  
  // JOYSTICK PUSH-BUTTON HANDLING
  boolean current_button_state = digitalRead(JOYSTICK_SW_PIN);
  if (last_button_state == HIGH && current_button_state == LOW) {
    if (millis() - last_button_press_ms > BUTTON_DEBOUNCE_DELAY) {
      last_button_press_ms = millis();
      
      if(menu[menu_number_1].type == VALUE || menu[menu_number_1].type == OPTION){
        in_menu = !in_menu;
      }
      if(menu[menu_number_1].type == ACTION){
        in_action = !in_action;
        step_counter = 0;
        fill_state_ms = millis();
        is_filling_phase = true;
        current_bottle_count = 0;
        last_printed_count = -1;

        // Force-update motor direction pin before executing motor pulses
        digitalWrite(MOTOR_DIR_PIN, (menu[6].value == 0) ? LOW : HIGH);

        steps = steps_calc(menu[1].value, 0, menu[8].value, menu[1].decimals);
        delay_us = delay_us_calc(menu[2].value, 2, menu[8].value, menu[2].decimals);
      }
      
      if (in_action == true || in_menu == true){
        menu_entered = true;
      }
      
      if (in_action == false && in_menu == false){
        menu_left = true;
      }
    }
  }
  last_button_state = current_button_state;

  // SETUP ACTION/VALUE ENTRY
  if (menu_entered){
    lcd.blink();
    if (menu[menu_number_1].type == ACTION){
      lcd.setCursor(11, 0);
      lcd.print(menu[menu_number_1].suffix);
    }
    menu_entered = false;
  }

  // ACTIONS
  if (in_action){
    switch (menu_number_1){
    case 0: // Start
      if (menu[7].value == 0){ // Dose
        if (dose(steps, delay_us, step_counter)){
          exit_action_menu();
        }
      } else if (menu[7].value == 1){ // Pump
        unsigned long target_ms = (unsigned long)(menu[3].value) * 100;
        if (millis() - fill_state_ms >= target_ms) {
          exit_action_menu();
        } else {
          pump(delay_us);
        }
      } else if (menu[7].value == 2){ // Cal.
        if (dose(CALIBR_STEPS, CALIBR_DELAY_US, step_counter)){
          exit_action_menu();
        }
      } else if (menu[7].value == 3){ // Cont (Continuous bottle filling mode)
        unsigned long fill_ms = (unsigned long)(menu[3].value) * 100;
        unsigned long stop_ms = (unsigned long)(menu[4].value) * 100;
        int total_target_count = menu[5].value;

        if (current_bottle_count != last_printed_count) {
          lcd.setCursor(14, 0);
          lcd.print(current_bottle_count);
          lcd.print(" ");
          last_printed_count = current_bottle_count;
        }

        if (is_filling_phase) {
          if (millis() - fill_state_ms >= fill_ms) {
            current_bottle_count++;
            if (current_bottle_count >= total_target_count) {
              lcd.setCursor(14, 0);
              lcd.print(current_bottle_count);
              delay(200);
              exit_action_menu();
            } else {
              is_filling_phase = false;
              fill_state_ms = millis();
            }
          } else {
            pump(delay_us);
          }
        } else {
          if (millis() - fill_state_ms >= stop_ms) {
            is_filling_phase = true;
            fill_state_ms = millis();
          }
        }
      }
      break;

    case 9: // Save Settings
      for (int i=0; i <= menu_items_limit; i++){
        eepromWriteInt(i*2, menu[i].value);
      }
      delay(700);
      menu_left = true;
      break;
    
    case 10: // USB Control
      while (Serial.available()) {
        inChar = (char)Serial.read();
        step_counter = 0;
        if (inChar == 'p'){
          rate_uL_min = Serial.parseInt();
          cal = Serial.parseInt();
          if(cal == 0){
            cal = menu[8].value;
          }
          delay_us = delay_us_calc(rate_uL_min, 1, cal, 0);
          usb_start = true;
        } else if (inChar == 'd'){
          vol_uL = Serial.parseInt();
          rate_uL_min = Serial.parseInt();
          cal = Serial.parseInt();
          if(cal == 0){
            cal = menu[8].value;
          }
          steps = steps_calc(vol_uL, 1, cal, 0);
          delay_us = delay_us_calc(rate_uL_min, 1, cal, 0);
          usb_start = true;
        } else if (inChar == 'c'){
          usb_start = true;
        } else if (inChar == 'w'){
          cal = Serial.parseInt();
          menu[8].value = cal;
          for (int i=0; i <= menu_items_limit; i++){
            eepromWriteInt(i*2, menu[i].value);
          }
          usb_start = false;
        } else if (inChar == 'x'){
          usb_start = false;
        }
      }
      
      if (usb_start) {
        if(inChar == 'p'){
          pump(delay_us);
        } else if (inChar == 'd') {
          if (dose(steps, delay_us, step_counter)){
            usb_start = false;
          }
        } else if (inChar == 'c'){
          if (dose(CALIBR_STEPS, CALIBR_DELAY_US, step_counter)){
            usb_start = false;
          }
        }
      }
      break;
    }

  // MENU NAVIGATION (NO ACTION RUNNING)
  } else if (!in_action){

    int raw_nav = analogRead(JOYSTICK_NAV_PIN);     // Reading pin A1
    int raw_val = analogRead(JOYSTICK_VALUE_PIN);   // Reading pin A0

    // 1. MENU NAVIGATION (Up/Down)
    if (!in_menu) {
      if ((raw_nav > JOYSTICK_HIGH_THRESH || raw_nav < JOYSTICK_LOW_THRESH) && nav_axis_centered && (millis() - last_nav_release_ms > JOYSTICK_RELEASE_SETTLE_MS)) {
        
        if (raw_nav > JOYSTICK_HIGH_THRESH) {
          menu_number_1++;
          if (menu_number_1 > menu_items_limit) menu_number_1 = 0;
        } else if (raw_nav < JOYSTICK_LOW_THRESH) {
          menu_number_1--;
          if (menu_number_1 < 0) menu_number_1 = menu_items_limit;
        }
        
        menu_number_2 = menu_number_1 + 1;
        if (menu_number_2 > menu_items_limit) menu_number_2 = 0;

        nav_axis_centered = false; 
        val_change = true;
      } else if (raw_nav >= JOYSTICK_LOW_THRESH && raw_nav <= JOYSTICK_HIGH_THRESH) {
        if (!nav_axis_centered) {
          last_nav_release_ms = millis();
        }
        nav_axis_centered = true;
      }
    }

    // 2. PARAMETER EDITING (Increase/Decrease)
    if (in_menu) {
      if ((raw_val > JOYSTICK_HIGH_THRESH || raw_val < JOYSTICK_LOW_THRESH) && (millis() - last_val_release_ms > JOYSTICK_RELEASE_SETTLE_MS)) {
        
        // A) VALUE TYPE (NUMERIC): Continuous holding with time-based acceleration
        if (menu[menu_number_1].type == VALUE) {
          if (value_axis_centered) {
            val_hold_start_ms = millis();
          }

          if (value_axis_centered || (millis() - last_val_hold_ms > HOLD_REPEAT_DELAY)) {
            last_val_hold_ms = millis();
            
            unsigned long hold_duration_sec = (millis() - val_hold_start_ms) / 1000;
            int step_increment = 1 << hold_duration_sec; // Doubles every 1s
            if (step_increment > 50) step_increment = 50;

            if (raw_val > JOYSTICK_HIGH_THRESH) {
              menu[menu_number_1].value += step_increment;
              if (menu[menu_number_1].value > menu[menu_number_1].lim) menu[menu_number_1].value = 0;
            } else if (raw_val < JOYSTICK_LOW_THRESH) {
              menu[menu_number_1].value -= step_increment;
              if (menu[menu_number_1].value < 0) menu[menu_number_1].value = menu[menu_number_1].lim;
            }

            value_axis_centered = false;
            val_change = true;
          }

        // B) OPTION TYPE (TEXT OPTIONS): Single step per joystick movement
        } else if (menu[menu_number_1].type == OPTION) {
          if (value_axis_centered) {
            if (raw_val > JOYSTICK_HIGH_THRESH) {
              menu[menu_number_1].value++;
              if (menu[menu_number_1].value > menu[menu_number_1].lim) menu[menu_number_1].value = 0;
            } else if (raw_val < JOYSTICK_LOW_THRESH) {
              menu[menu_number_1].value--;
              if (menu[menu_number_1].value < 0) menu[menu_number_1].value = menu[menu_number_1].lim;
            }

            // Immediately update the physical DIR pin state if Direction item was modified
            if (menu_number_1 == 6) {
              digitalWrite(MOTOR_DIR_PIN, (menu[6].value == 0) ? LOW : HIGH);
            }

            value_axis_centered = false;
            val_change = true;
          }
        }

      } else if (raw_val >= JOYSTICK_LOW_THRESH && raw_val <= JOYSTICK_HIGH_THRESH) {
        if (!value_axis_centered) {
          last_val_release_ms = millis();
        }
        value_axis_centered = true;
      }
    }

    if (val_change == true){
      update_lcd();
      val_change = false;
    }
  }

  // CLOSE
  if (menu_left){
    lcd.noBlink();
    if (menu[menu_number_1].type == ACTION){
      exit_action_menu();
    }
    
    // Ensure direction pin state is synchronized when leaving edit mode
    digitalWrite(MOTOR_DIR_PIN, (menu[6].value == 0) ? LOW : HIGH);

    steps = steps_calc(menu[1].value, 0, menu[8].value, menu[1].decimals);
    delay_us = delay_us_calc(menu[2].value, 2, menu[8].value, menu[2].decimals);
    
    menu_left = false;
  }
} 

// HELPER FUNCTIONS ------------------------------------------------------------------------

boolean dose(long _steps, int _delay_us, long & inc) {
  if(inc < _steps){
    digitalWrite(MOTOR_STEP_PIN, HIGH); 
    delayMicroseconds(_delay_us);
    digitalWrite(MOTOR_STEP_PIN, LOW); 
    delayMicroseconds(_delay_us);
    inc++;
    return false;
  } else {
    inc = 0;
    return true;
  }
}

void pump(int _delay_us) {
  digitalWrite(MOTOR_STEP_PIN, HIGH); 
  delayMicroseconds(_delay_us); 
  digitalWrite(MOTOR_STEP_PIN, LOW); 
  delayMicroseconds(_delay_us);
}

void exit_action_menu(){
  in_action = false;
  lcd.setCursor(11, 0);
  lcd.print("     ");
  lcd.noBlink();
}

long steps_calc(long volume, int unit_mode, int calibr, int decimals){ 
  long _steps;
  int decimal_corr;
  double conv;
  double cal;

  decimal_corr = pow(10, decimals);
  cal = calibr;
  cal = (cal / CALIBR_ROTATIONS) / CALIBR_DECIMAL_CORR;

  if (cal <= 0) cal = 1.0;

  if(unit_mode == 2){
    conv = 1.0;
  } else if (unit_mode == 1){
    conv = 1.0 / cal / 1000;
  } else if (unit_mode == 0){
    conv = 1.0 / cal;
  }

  _steps = STEPS_PER_FULL_ROT * STEP_MODE * conv * volume / decimal_corr;
  return _steps;
}

long delay_us_calc(long vol_per_min, int unit_mode, int calibr, int decimals){
  double d_delay_us;
  long _delay_us;
  int decimal_corr;
  double conv;
  double cal;  

  decimal_corr = pow(10, decimals);

  cal = calibr;
  cal = (cal / CALIBR_ROTATIONS) / CALIBR_DECIMAL_CORR;

  if (cal <= 0) cal = 1.0;

  if(unit_mode == 2){
    conv = 1.0;
  } else if (unit_mode == 1){
    conv = 1.0 / cal / 1000;
  } else if (unit_mode == 0){
    conv = 1.0 / cal;
  }
  
  d_delay_us = (1 / (STEPS_PER_FULL_ROT * STEP_MODE * conv * vol_per_min / decimal_corr)) * 60 * MICROSEC_PER_SEC / 2;
  _delay_us = d_delay_us;
  return _delay_us;
}

void update_lcd(){
  lcd.clear();

  // First line
  lcd.setCursor(0, 0);
  lcd.print(menu_number_1);
  lcd.print("|");
  lcd.print(menu[menu_number_1].name_);
  if (menu[menu_number_1].type == 0){
    value_dbl = menu[menu_number_1].value;
    value_dbl = value_dbl / pow(10, menu[menu_number_1].decimals);
    dtostrf(value_dbl, VALUE_MAX_DIGITS, menu[menu_number_1].decimals, value_str);
    lcd.print(" ");
    lcd.print(value_str);
    lcd.print(menu[menu_number_1].suffix);
  } else if(menu[menu_number_1].type == 1){
    lcd.print(" ");
    lcd.print(menu[menu_number_1].options[menu[menu_number_1].value]);
  }
  
  // Second line
  lcd.setCursor(0, 1);
  lcd.print(menu_number_2);
  lcd.print("|");
  lcd.print(menu[menu_number_2].name_);
  if (menu[menu_number_2].type == 0){
    value_dbl = menu[menu_number_2].value;
    value_dbl = value_dbl / pow(10, menu[menu_number_2].decimals);
    dtostrf(value_dbl, VALUE_MAX_DIGITS, menu[menu_number_2].decimals, value_str);
    lcd.print(" ");
    lcd.print(value_str);
    lcd.print(menu[menu_number_2].suffix);
  } else if(menu[menu_number_2].type == 1){
    lcd.print(" ");
    lcd.print(menu[menu_number_2].options[menu[menu_number_2].value]);
  }
  
  lcd.setCursor(1, 0);
}

void eepromWriteInt(int adr, int wert) { 
  byte low, high;
  low = wert & 0xFF;
  high = (wert >> 8) & 0xFF;
  EEPROM.write(adr, low);
  EEPROM.write(adr + 1, high);
}

int eepromReadInt(int adr) { 
  byte low, high;
  low = EEPROM.read(adr);
  high = EEPROM.read(adr + 1);
  return low + ((high << 8) & 0xFF00);
}
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "SparkFun_Qwiic_Twist_Arduino_Library.h" 
//nav sensors
#include "Adafruit_VL53L1X.h"

const float version = 1.0001;
/*
 * Electric Mayhem, 1.0.001 -  Robomagellan
 * by Chas Ihler
 * https://iradan.com
 * 
 * Target SAMD51 (Sparkfun MicroMod)
 * Controller 1 - UI, Arbitator
 *
 */
 

/*
 *  ---------- Definitions & Hardware  -------------
 *
 */

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
//vl53
#define IRQ_PIN 2
#define XSHUT_PIN 3


const bool NAV_LOGGING = true;
const bool PWR_LOGGING = false;
const bool CAL_LOGGING = false;
const bool VBS_LOGGING = true;
const bool CAN_LOGGING = false;
const bool COM_LOGGING = false;
const bool STATE_LOGGING = false;
const bool TSKMGR_LOGGING = false;
const bool TRACE_LOGGER = true;


/*
 *  ---------- Instances  -------------
 *
 */

TWIST twist;                              
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//sensors
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

/*
 *  ---------- Variables  -------------
 *
 */
 
  //x, y
  byte waypoint [20] [2] {
    {25, 25},
    {25, 50},
    {25, 75},
    {5, 80},
    {25, 80}
  };
  const int valid_path = 0x7F;

//devices
bool DEV_ROTENC = true;
bool DEV_TOF1 = true;

//state
 enum Missions {HOLD, TEST, RC, SRSC1, SRSC2, PCC1, PCC2};   //The states
 Missions RobotMissions = HOLD;
 enum Modes {MENU, AUTO, MANU};   //The states
 Modes RobotMode = MENU;


 //menus
 #define MENUARRAYSIZE 35
 String menutxt[MENUARRAYSIZE] = { "HALT", "SBC1", "SBC2", "RM 1", "RM 2", "TEST", "RC  "};
 #define OPTARRAYSIZE 10
 String opttxt[OPTARRAYSIZE] = { "NO", "YES"};
 bool menu_displayed = false; 

int menu = 0;
bool mission_button_pushed = false;
bool mission_selected = false;
int mission_is = 0;
bool mission_go = false;
bool mission_initialize = false;
bool mission_complete = false;
int mission_task = 0; //mission sequence number. 

//sensors
float tof_distance = 0;

//integration
  unsigned int dT = 50;
  unsigned long previousIntMillis = 0;
  float previous_error;
  float last_integration_error;
  bool updated = false;

// task manager variables
  unsigned long tasktime_start = 0;
  unsigned long tasktime_1 = 0;
  unsigned long tasktime_2 = 0;
  unsigned long tasktime_3 = 0;
  unsigned long tasktime_4 = 0;
  unsigned long tasktime_5 = 0;
  unsigned long tasktime_6 = 0;
  unsigned long tasktime_7 = 0;
  unsigned long tasktime_8 = 0;
  unsigned long tasktime_end = 0;
  
// non blocking wait
  unsigned long previousMillis = 0;    
  unsigned long previousTXMillis = 0;  
  unsigned long previousTX01Millis = 0;  
  unsigned long previousSensorMillis = 0;
  unsigned long previoustaskMillis = 0;
  unsigned long previousWFPIDMillis = 0;
  unsigned int last_transition = 0; //for led blink
  
  const int wfpid_interval = 100;
  const int wait_interval = 500;   //used for polling delays
  const int sensor_poll_delay = 250; 
  
  volatile bool flagRecv = false; 
  int can_rx;

  unsigned char len = 0;
  unsigned char buf[4];
  char str[20];

  //Trace Logger
  bool trace_trigger = false;

  
  bool b_run_once = false;

  //task flags
  bool b_tsm_wait_request = false;
  bool b_tsm_wait_surpress = false;

/*
 *  ---------- Setup  -------------
 *
 */


void setup() {
  delay(10);
  Wire.begin();
  Serial.begin(115200);

  Serial.println("EMOS 1.0.001");

  if (twist.begin() == false) {
    Serial.println("Twist does not appear to be connected. Please check wiring. Disabled...");
    DEV_ROTENC = false;
  }
  Wire.setClock(400000); //Optional: After Twist begin(), increase I2C speed to max, 400kHz
  delay(10);
  if (! vl53.begin(0x29, &Wire)) {
    DEV_TOF1 = false;
  }
  if (! vl53.startRanging()) {
    DEV_TOF1 = false;
  }
  vl53.setTimingBudget(50);

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.display();
  delay(2000); // Pause for 2 seconds
  // Clear the buffer
  display.clearDisplay();
  //display.invertDisplay(true);

  if (DEV_ROTENC) twist.setColor(0, 40, 100); //Set knob color
  if (DEV_ROTENC) twist.setLimit(6); //

  delay(100);
}


/*
 * Utilities
 */


//returns true if bit set in integer
bool unpack_bit (int frame, int bit) {
  if ((frame >> bit) & 0x01) {
    return true;  
  } else {
    return false;
  }
}
//((a >> 3)  & 0x01)


 void print_bits(byte byVal) {
      for (int i = 0; i < 8; i++)
    {
        bool b = byVal & 0x80;
        Serial.print(b);
        byVal = byVal << 1;
    }
    Serial.println(" ");
}


int returnMSB(int testInt) { 
    if (testInt == 0) 
        return 0; 
  
    int msb = 0; 
    testInt = testInt / 2; 
    while (testInt != 0) { 
        testInt = testInt / 2; 
        msb++; 
    } 
  
    return (1 << msb); 
}  


boolean isBitSet (byte myVar, byte bitNumber) {
  bool bitvalue;
  bitvalue = myVar & (1 << bitNumber);
  return bitvalue;
}  


 bool non_blocking_wait (void) {
  //call in loop
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= wait_interval) {
    // time passed, save new time.
    previousMillis = currentMillis;
    return true;
  }
  return false;
}


bool integration_delay (int ki) {
  unsigned long currentIntMillis = millis();
  if (currentIntMillis - previousIntMillis >= ki) {
    previousIntMillis = currentIntMillis;
    return true;
  }
  return false;
}


bool sensor_delay (void) {
  unsigned long sensorDelayMillis = millis();
  if (sensorDelayMillis - previousSensorMillis >= sensor_poll_delay) {
    // time passed, save new time.
    previousSensorMillis = sensorDelayMillis;
    return true;
  }
  return false;
}

bool wfpid_delay (void) {
  unsigned long sensor2DelayMillis = millis();

  if ((sensor2DelayMillis - previousWFPIDMillis) >= wfpid_interval) {
    // time passed, save new time.
    previousWFPIDMillis = sensor2DelayMillis;
    return true;
  }
  return false;
}


/* delay to prevent CAN message flooding */

 bool transmit_delay_wait (void) {

  unsigned long txDelayMillis = millis();
  if (txDelayMillis - previousTXMillis >= 250) {
    // time passed, save new time.
    previousTXMillis = txDelayMillis;
    return true;
  }
  return false;
}

 bool taskmanager_wait (void) {
  unsigned long taskDelayMillis = millis();
  if (taskDelayMillis - previoustaskMillis >= 500) {
    previoustaskMillis = taskDelayMillis;
    return true;
  }
  return false;
}


void tsm_wait(void) {
  //if (VBS_LOGGING) Serial.println("TSM_WAIT");
/*
  if (!b_tsm_wait_surpress) {
    b_bsm_stop_surpress = false;
    b_bsm_wander_surpress = true;
    b_bsm_goto_surpress = true;
    b_bsm_backup_surpress = true;
    b_bsm_crib_surpress = true;
    b_bsm_task_surpress  = true;
    b_bsm_sweep_surpress  = true;
    b_bsm_evade_surpress  = true;
    
    static bool led_state;
    
    if (hb_delay(1000) && led_state) {
      digitalWrite(READY_LED, LOW);
      led_state = false; 
      last_transition = millis();
      if (VBS_LOGGING) {
        Serial.println("Heatbeat Low");
      }
    }
    
    if (hb_delay(1000)  && !led_state) {
      digitalWrite(READY_LED, HIGH);
      led_state = true;
      last_transition = millis();
      if (VBS_LOGGING) {
        Serial.println("Heartbeat High");
      }
    }  
  
    pbBehaviorState = STOP;
  }
  */
}


/******************* functions ************************/

// Dispplay


void display_main_menu (int enc_cnt) {
  
  if (menu_displayed==false) {
    display.clearDisplay();   
    display.setTextSize(1);             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    display.setCursor(0,0);             // Start at top-left corner
    display.println(F("EMOS 1.0.001"));
    display.display();
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.setCursor(0,20);             // Start at top-left corner
    display.println(F("  SELECT MODE     "));
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  menu_displayed = true;
  }
  display.setCursor(0,32);           
  display.println(F("  "));
  display.setCursor(0,30); 
  display.print(menutxt[enc_cnt]);     
  display.display();
}

void display_confirm_menu (int enc_cnt) {
  display.setCursor(0,45);           
  display.print("                  ");
  display.setCursor(0,45);           
  display.print("Confirm? Y/N: ");
  display.print(opttxt[enc_cnt]);
  display.display();
}

void display_manu () {
  
  if (menu_displayed==false) {
    display.clearDisplay();   
    display.setTextSize(1);             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    display.setCursor(0,0);             // Start at top-left corner
    display.println(F("EMOS 1.0.001"));
    display.display();
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.setCursor(0,11);             // Start at top-left corner
    display.println(F("       MANUAL       "));
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  menu_displayed = true;
  }
  display.setCursor(0,22);           
  display.println(F("  "));
  display.setCursor(0,32); 
  display.print("mode: "); 
  display.print(mission_is);    
  display.display();
}

void display_tof () {  
  
  display.setCursor(0,32);           
  display.println(F("              "));
  display.setCursor(0,32); 
  display.print("tof: "); 
  display.print(tof_distance);
  display.println(" mm");    
  display.display();
}



// Hardware

int check_enc (void) {
  int i_cnt = 0;
  if (DEV_ROTENC) {
      Serial.print("Count: ");
      Serial.println(twist.getCount());
      return twist.getCount();
  }
  return 0;
}

void poll_sensors(void) {
  float old_tofd = tof_distance;
  if (DEV_TOF1) {
    if (vl53.dataReady()) {
      // new measurement for the taking!
      tof_distance = vl53.distance();
      if (tof_distance == -1) {
        tof_distance = old_tofd;
        return;
      }
      // data is read out, time for another reading!
      vl53.clearInterrupt();
    }
  }  
}

/*
 * Top Level Main Loop Calls
 * Sense
 * Think
 * Act
 */

//WAIT, GOTO_HALL, CHECK_POS, GOTO_KITCHEN, GOTO_FRIG, OPEN_FRIG, GET_CAN, EXIT_KITCH, GOTO_TABLE, DROP_CAN, GO_HOME
void tsm() {
/*
 switch (pbTaskState) {
 case WAIT:       { tsm_wait(); break; }  
 case GOTO_HALL:  { tsm_wait(); break; }
 case CHECK_POS:  { tsm_wait(); break; }
 case WALL_FOLLOW:{ wall_follow(); break;}
 case GOTO_FRIG:  { tsm_wait(); break; }
 case OPEN_FRIG:  { tsm_wait(); break; }
 case GET_CAN:    { tsm_wait(); break; }
 //case CLOSE_FRIG: { tsm_wait(); break; }
 case EXIT_KITCH: { tsm_wait(); break; }
 case GOTO_TABLE: { tsm_wait(); break; }
 case DROP_CAN:   { tsm_wait(); break; } 
 case GO_HOME:    { tsm_wait(); break; } 
 default:
  break;
 }
 */
}

void bsm() {
  /*
  b_bsm_stop_flag = false;
  b_bsm_wander_flag = false;
  b_bsm_goto_flag = false;
  b_bsm_backup_flag = false;
  b_bsm_crib_flag = false;
  b_bsm_task_flag  = false;
  b_bsm_sweep_flag  = false;
  b_bsm_evade_flag  = false;
       
 switch (pbBehaviorState) {
 case STOP:       { b_bsm_stop_flag = bsm_stop(); break; }  
 case WANDER:     { tsm_wait(); break; }
 case GOTO:       { tsm_wait(); break; }
 case BACKUP:     { tsm_wait(); break; }
 case CRIB:       { tsm_wait(); break; }
 case ROTATE:     { tsm_wait(); break; }
 case TASK:       { tsm_wait(); break; }
 case SWEEP:      { tsm_wait(); break; }
 case EVADE:      { b_bsm_evade_flag = bsm_evade(); break; }
 case FUTURE:     { tsm_wait(); break; }
 }
 */
}

void arbitrate(void) {
    /*
     * 
     *  set active goal
     *  supress flags based on goal
     *  determine priority function with control
     *
     *  Determine tasks
     *  Run Task Station Machine
     *  Modify Supression Flags if needed. 
     *  Run Behavior 
     *  Run any special tasking (elevators/grabbers/etc.)
     *  
     * Prioity 
     * Lowest to Highest
     *      Stop
     *      Navigate to waypoint[x] 
     *      Switch Seq @ waypoint
     *      Special Function x (expecting sensor data)
     *      Avoid based on Lidar (unless expecting an approach)
     *      Check and Escape Bumpers (if not expecting a bump) 
     */
   
     //Determine Active Goal / Task State

     //if (b_start_button && pbTaskState == WAIT) {
        /* -- TESTING
        enableStepper();
        int myStepperPos = stepper.currentPosition();
        if (VBS_LOGGING) {
          Serial.print("Current Position"); Serial.println(myStepperPos);
        }
        delay(1000);
        stepper.moveTo(myStepperPos-1000);
        */

      //  pbTaskState = GOTO_HALL;
     //}

     //OVERRIDE Behavior State Machine
    /*
    if (poll_bumpers() && !b_flag_bumpers_supressed) {
      b_movement_repeat = false;
      pbBehaviorState = EVADE;
      b_bsm_stop_surpress = true;
      b_bsm_wander_surpress = true;
      b_bsm_goto_surpress = true;
      b_bsm_backup_surpress = true;
      b_bsm_crib_surpress = true;
      b_bsm_task_surpress  = true;
      b_bsm_sweep_surpress  = true;
      b_bsm_evade_surpress  = false;
    }
    */
     //surpress flags using task states / goal
     //if task x.. & task flag y 
      //b_bsm_stop_surpress = false;
      //b_bsm_wander_surpress = true;
      //b_bsm_goto_surpress = true;
      //b_bsm_backup_surpress = true;
      //b_bsm_crib_surpress = true;
      //b_bsm_task_surpress  = true;
      //b_bsm_sweep_surpress  = true;
      //b_bsm_evade_surpress  = true;

     //act
     //determing highest priority
      //b_bsm_stop_flag = false;
      //b_bsm_wander_flag = false;
      //b_bsm_goto_flag = false;
      //b_bsm_backup_flag = false;
      //b_bsm_crib_flag = false;
      //b_bsm_task_flag  = false;
      //b_bsm_sweep_flag  = false;
     
}


/*
 * MAIN LOOP
 */
 
void loop() {
  unsigned long totaltasktime = 0;
  unsigned long te = 0;
  float tp, ttf = 0.0;

  trace_trigger = false;

  tasktime_start = millis();
  tasktime_1 = millis();

  if (b_run_once==false) {    //run once start-up bits.
    b_run_once = true;
      
  } 
  
  tasktime_2 = millis();

    //determine the super-mode
    if (RobotMode==MENU) {
      if (menu == 0) {
        display_main_menu(check_enc());
        if (twist.isPressed()) {
          menu = 1;
          delay(200);
        }
        
      }
      if (menu == 1) {
          display_confirm_menu(check_enc());
          if (twist.isPressed()) {
            delay(200);
            if (check_enc() == 0) {
              menu = 0;
              menu_displayed=false;
            } else {
              mission_selected = true;
              mission_is = check_enc();
              menu = 0;
              menu_displayed=false;
            }
          }
        }
    }
    //HOLD, SRSC1, SRSC2, PCC1, PCC2, TEST, RC}
    if (mission_selected == true && mission_is < 4 ) {
      RobotMode=MANU;
      if (mission_initialize == false) {
        //init
        menu = 0;
        mission_initialize = true;
      } else {
        //do stuff
      }
    }
    if (mission_selected == true && mission_is > 3 ) {
      RobotMode=AUTO;
      if (mission_initialize == false) {
        //init
      } else {
        //do stuff
      }
    }

    if (RobotMode==MANU) {
      if (menu==0) {
        display_manu();
        menu = 99;
      }
      if (menu == 99) {
        display_tof();
      }
    }

    //menu-wait?
    //run autonomous
    //run remote
  
  tasktime_3 = millis();

  poll_sensors();

  tasktime_4 = millis();

  tsm(); tasktime_5 = millis();

  arbitrate(); tasktime_6 = millis();

  bsm(); tasktime_7 = millis();

  tasktime_8 = millis();

   //publish
  if (transmit_delay_wait()){ 
    //publish
    
  }

  tasktime_end = millis();
  if (taskmanager_wait()){
     if (TSKMGR_LOGGING || trace_trigger) {
        totaltasktime = tasktime_end - tasktime_start; 
        ttf = totaltasktime;
        ttf = ttf * 1.0;
        Serial.println("-----------------------------------");
        Serial.print("* Task Manager - Loop Time(ms): ");  Serial.println(ttf);
        Serial.println("* Task ID    Time/%");

          Serial.print("  1  START      ");
          te = tasktime_2 - tasktime_1;

          tp = te / ttf;
          tp = tp * 100.0;
        Serial.print(te); Serial.print(" / ");
        Serial.print(tp); Serial.println("%");

          Serial.print("  2 TASK 2     ");
          te = tasktime_3 - tasktime_2;

          tp = te / ttf;
          tp = tp * 100.0;
        Serial.print(te); Serial.print(" / ");
        Serial.print(tp); Serial.println("%");

        Serial.print("  3 STASK 3       ");
          te = tasktime_4 - tasktime_3;

          tp = te / ttf;
          tp = tp * 100.0;
        Serial.print(te); Serial.print(" / ");
        Serial.print(tp); Serial.println("%");

        Serial.print("  4 TASK 4       ");
          te = tasktime_5 - tasktime_4;

          tp = te / ttf;
          tp = tp * 100.0;
        Serial.print(te); Serial.print(" / ");
        Serial.print(tp); Serial.println("%");

        Serial.print("  5 TASK 5       ");
          te = tasktime_6 - tasktime_5;

          tp = te / ttf;
          tp = tp * 100.0;
        Serial.print(te); Serial.print(" / ");
        Serial.print(tp); Serial.println("%");

        Serial.print("  6 TASK 6      ");
          te = tasktime_7 - tasktime_6;

          tp = te / ttf;
          tp = tp * 100.0;
        Serial.print(te); Serial.print(" / ");
        Serial.print(tp); Serial.println("%");   

        Serial.print("  7 - TASK 7     ");
          te = tasktime_8 - tasktime_7;

          tp = te / ttf;
          tp = tp * 100.0;
        Serial.print(te); Serial.print(" / ");
        Serial.print(tp); Serial.println("%");  

        Serial.print("  8 - TASK 8       ");
          te = tasktime_end - tasktime_8;
 
          tp = te / ttf;
          tp = tp * 100.0;
        Serial.print(te); Serial.print(" / ");
        Serial.print(tp); Serial.println("%");       

        Serial.println("-----------------------------------");

     }
  }
}

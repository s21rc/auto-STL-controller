/*
  S21RC STL (Magnetic Loop) auto controller

  Hardware used:
  Stepper Motor: Bipoler 4 wire NEMA 17, 1.3A
  Stepper Driver module: DRV8825
  Display: 1.3" OLED SH1106 I2C
  Rotary Encoder: 5 pin simple rotary
  Microcontroller: Arduino NANO (ATMEGA 328P) 16MHz
  Motor Control Cable: CAT6 Unshielded Twisted Pair (used two pair for signal,
    one pair motor body ground)
   VSWR circuit : contains 3 comparator, SWR<3, SWR<2 and SWR<1.5. 
    Also one pre-scaller devide by 64 for the frequency reading.
  
  Vacuum Capacitor: 5-100pf
  Antenna: 1m dia - 0.625" copper tube. Gamma Match
  
  Operation:
  When power on, read last tuned frequency and position from EEPROM and Display
  When TUNE pressed, it reads the frequency and compare with band table. if matches with Band table,
  go directly there and then fine tune for best SWR. If not found then motor moves each step and check SWR.

  
  This code is in the public domain. You are free to copy/modify and use for yourself.

  https://www.s21rc.net
*/



#include <FreqCount.h>// Frequency counter
#include <AccelStepper.h> // Stepper motor
#include <EEPROM.h> //EEPROM 
#include <U8g2lib.h> //Display 
#include <Wire.h> //I2C
#include <SimpleRotary.h> //Rotary Encoder

//Initialize OLED display
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

//Motor driver initialization
int motorSpeed = 400; //maximum steps per second (about 5rps / at 8 microsteps)
int motorAccel = 3600; //steps/second/second to accelerate
int motor_speed = motorSpeed; 

//Motor Driver PIN setup, any 3 digital IO is ok.
int motorStepPin = 12; 
int motorDirPin = 11; 
const int Enable_PIN = 13;


AccelStepper stepper(1, motorStepPin, motorDirPin);
const int MAX_limit = 3500; // (200* MAX Rotation of Vacuum Variable Cap)- 100 " change as per your Capacitor
const int MIN_limit = 10; //
int motor_position;

// Rotary Encoder setup "Any 3 IO is ok to use"
#define DT A1
#define CLK A0
#define SW A2
SimpleRotary rotary(CLK,DT,SW); // Pin A, Pin B, Switch Pin
int ctr = 0;
byte lastDir = 0;

//VSWR Bridge pin setup
const byte TUNE_N = 3; // Label:TUNE
const byte TLATCH = 4; // Label:LATCH
const byte IND_N = 10; //Label: KEY
const byte VSWR3 = 8; //Label:VSWR3
const byte INT_N = 2; // Label:INT, Must be Digital Pin D2 or D3 for interrupt to work
const byte VSWR2 = 7; // Label:VSWR2
const byte VSWR15 = 6; //Label:VSWR1.5
const byte RF_N = 9;  // Label:RF
const byte FREQ_in = 5; //Label:FRQ, must be Digital PIN 5 for timer


int R, swr3, swr2, swr15;


//Reference for fast tune

//unsigned long reference_frequency_12m = 347250;
int reference_position_12m;

//unsigned long reference_frequency_15m = 303218;
int reference_position_15m;

//unsigned long reference_frequency_17m = 270031;
int reference_position_17m;

//unsigned long reference_frequency_20m = 224000;
int reference_position_20m;

//unsigned long reference_frequency_30m = 179843;
int reference_position_30m;

int band_base_position=0;

unsigned long last_tuned_freq;
int last_tuned_position;

//last tuned
unsigned long current_frequency;
unsigned long frq;
const float prescaller = 0.064075; // Idealy 0.064, adjust to match frequency reading with radio TX

//EEPROM Address for last tuned location and frequency
int const EEPROM_location = 0;
int const EEPROM_frequency = 24;

//12m reference
int const EEPROM_12m_location = 100;

//15m reference
int const EEPROM_15m_location = 200;

//17m reference
int const EEPROM_17m_location = 300;

//20m reference
int const EEPROM_20m_location = 400;

//30m reference
int const EEPROM_30m_location = 500;


volatile int tune_pressed = LOW;
//int latch_enable = LOW; 
int VSWR_result = 99;
int TUNE_abort = 1;

//software MC reset
//void(* resetFunc) (void) = 0;


void setup(){

/*  Initialization ** ONLY RUN ONCE
 *  Manually put the motor to minimum capacitance and few turns as margin. 
 *  You need to tune first without [NOTE2] code - write down the position for each band start, and put the result in NOTE2
 *  
 *  
 *  // [NOTE1] Following data to initialize the motor position for first time
    unsigned long last_tuned_freq= 468750;
    int last_tuned_position=1;
    EEPROM.put(EEPROM_location, last_tuned_position);
    EEPROM.put(EEPROM_frequency, last_tuned_freq);

    // [NOTE2] Following data for fast tune using pre defined location
    int reference_position_12m=634; //Position for 24890KHz
    int reference_position_15m=1151; //Position for 21000KHz
    int reference_position_17m=1430; //Position for 18068KHz
    int reference_position_20m=2033; //Position for 14000KHz
    int reference_position_30m=3343; // //Position for 10100KHz

    
    EEPROM.put(EEPROM_12m_location, reference_position_12m);
    EEPROM.put(EEPROM_15m_location, reference_position_15m);
    EEPROM.put(EEPROM_17m_location, reference_position_17m);
    EEPROM.put(EEPROM_20m_location, reference_position_20m);
    EEPROM.put(EEPROM_30m_location, reference_position_30m);
    */

  
  u8g2.begin();  
  u8g2.enableUTF8Print();
  
  FreqCount.begin(1000);

  //Stepper motor driver
  stepper.setMaxSpeed(motorSpeed);
  stepper.setSpeed(motorSpeed);
  stepper.setAcceleration(motorAccel);
  pinMode(Enable_PIN, OUTPUT);
  digitalWrite(Enable_PIN, HIGH); //Disabling Motor Driver    
   
  // VSWR board
  pinMode(FREQ_in, INPUT_PULLUP);
  pinMode(VSWR15, INPUT);
  pinMode(VSWR2, INPUT);
  pinMode(VSWR3, INPUT); 
  pinMode(RF_N, INPUT); 
  pinMode(TLATCH, OUTPUT);
  pinMode(IND_N, OUTPUT);
  pinMode(INT_N, INPUT_PULLUP);
  pinMode(TUNE_N, INPUT_PULLUP);
  
  Serial.begin(9600);
  
  // Interrupt to start tuning
  attachInterrupt(digitalPinToInterrupt(INT_N), tune_latch, FALLING);


  /* Only uncomment for debug purpose, do not leave as it will slow down the system motor. */
  //Serial.println ("===================");
  //Serial.println ("== Initial state ==");
  //Serial.println ("===================");
  //digitalWrite(TLATCH, latch_enable);
  //Serial.print ("TUNE LATCH:");
  //Serial.println (digitalRead(TLATCH));
  //Serial.print ("TUNE State:");
  //Serial.println (digitalRead(TUNE_N));
  //Serial.print ("IND State:");
  //Serial.println (digitalRead(IND_N));
  // Read last tuned frequency from EEPROM
  EEPROM.get(EEPROM_frequency, last_tuned_freq);
  //Serial.print("Last tuned frequency:");
  //Serial.print(last_tuned_freq*0.064);
  //Serial.println("  KHz");
  // Read last tuned position from EEPROM
  
  EEPROM.get(EEPROM_location, last_tuned_position);
  //Serial.print("Last tuned position:");
  //Serial.println(last_tuned_position);
   
  stepper.setCurrentPosition(last_tuned_position);
  motor_position = stepper.currentPosition();
  //Serial.print("Setting Current Motor position to:");
  //Serial.println(motor_position);
  //Serial.println ("##############");
  draw(last_tuned_freq, motor_position, 1); // int freq, int steps, status code 1:Ready 2:TUNE STARt 3:FAST TUNE 4:FINE TUNE 5:TUNED 6:SLOW SCAN 7:TUNE FAIL 
}

void loop(){
//Serial.println (F_CPU);
if (tune_pressed==HIGH ){
  delay(500);
  if (digitalRead(TUNE_N)==LOW)
       Tune_start();     
  }

   rotary_encoder();
}



// TUNE Function
void Tune_start(){
  
  digitalWrite(TLATCH, 1);
  delay(100);
  digitalWrite (IND_N, 1); // instruct radio to Transmit


  EEPROM.get(EEPROM_frequency, last_tuned_freq);
  EEPROM.get(EEPROM_location, last_tuned_position);
  draw(last_tuned_freq, last_tuned_position, 2); // int freq, int steps, status code 1:Ready 2:TUNE STARt 3:FAST TUNE 4:FINE TUNE 5:TUNED 6:SLOW SCAN 7:TUNE FAIL 

  digitalWrite(Enable_PIN, LOW); //enabling Motor Driver
  //Serial.println ("****** Motor Enable ******");
  
  TUNE_abort=0;
  delay(1000);

  current_frequency =0;
  //    //Serial.println ("Reading frequency...");
      current_frequency=read_freq();
      if (current_frequency!=0) TUNE_abort = 1;
      else return;
      
  //Serial.print("Current frequency:");
  //Serial.print(current_frequency*0.064);
  //Serial.println("   KHz");
  draw(current_frequency, motor_position, 2); // int freq, int steps, status code 1:Ready 2:TUNE STARt 3:FAST TUNE 4:FINE TUNE 5:TUNED 6:SLOW SCAN 7:TUNE FAIL 

  band_base_position=reference_point_check(current_frequency);
  //Serial.print("Band base position");
  //Serial.print(band_base_position);

  // Read last tuned frequency from EEPROM
  EEPROM.get(EEPROM_frequency, last_tuned_freq);
  //Serial.print("Last tuned frequency:");
  //Serial.print(last_tuned_freq*0.064);
  //Serial.println(" KHz");

  // Read last tuned position from EEPROM
  EEPROM.get(EEPROM_location, last_tuned_position);
  //Serial.print("Last tuned position:");
  //Serial.println(last_tuned_position);

  int swr_low=0;
  int swr_low1=0;
  int swr_low2=0;
  int downwards=1;
  
  int current_position = stepper.currentPosition();
  int optimum_position = last_tuned_position;
    
  //deside direction
  int motor_direction = 0;
  int fast_tune = 0;
  if (band_base_position!=0)
    fast_tune=goto_base_position(band_base_position);

  if (fast_tune==1){
      motor_direction = 2;
      draw(current_frequency, band_base_position, 4); // int freq, int steps, status code 1:Ready 2:TUNE START 3:FAST TUNE 4:FINE TUNE 5:TUNED 6:SLOW SCAN 7:TUNE FAIL 
    //  //Serial.print ("Fast tune complete> Starting Fine tune");
    //  //Serial.println ("Decrement/Motor CW");
  }
  
 
    VSWR_result=VSWR_check();
  
    
    /*
     //VSWR result goes here
    //Serial.print ("### (CP 101) VSWR Result code: ");
    //Serial.println (VSWR_result);
   */
  
  if (VSWR_result==0 && current_position == last_tuned_position ) tune_success();
    
  else if (fast_tune == 0 && VSWR_result!=0 && TUNE_abort == 1 && current_frequency<last_tuned_freq ){  
    motor_direction = 1;
    //Serial.print ("Motor Direction required:");
    //Serial.println ("Increment/Motor CCW");
  }
  
    else if (fast_tune == 0 && VSWR_result!=0 && TUNE_abort==1 && current_frequency>last_tuned_freq ){
      motor_direction = 2;
      //Serial.print ("Motor Direction required:");
      //Serial.println ("Decrement/Motor CW");
    }
         
      current_position = stepper.currentPosition();
      while(VSWR_result!=0 && motor_direction ==1 && current_position < MAX_limit){
      current_position++;
      stepper.moveTo(current_position);
      stepper.runToPosition();
     
       VSWR_result=VSWR_check();
       
       if(VSWR_result == 3 && downwards == 1) swr_low1=current_position;
           else if(VSWR_result == 2 && downwards == 1) downwards = 0;
              

        if(VSWR_result == 1 ) swr_low = 1;
             
        if(VSWR_result == 3 && swr_low == 1){ 
          swr_low2=current_position;
          int position_difference = swr_low2-swr_low1; 
          position_difference=position_difference/2;
          optimum_position = current_position - position_difference;
          stepper.moveTo(optimum_position);
          stepper.runToPosition();
          motor_direction = 0;
          }
       }

     current_position = stepper.currentPosition();
      while(VSWR_result!=0 && motor_direction ==2 && current_position > MIN_limit){
      current_position--;
      stepper.moveTo(current_position);
      stepper.runToPosition();

      VSWR_result=VSWR_check();
        
      if(VSWR_result == 3 && downwards == 1) swr_low1=current_position;
          else if(VSWR_result == 2 && downwards == 1) downwards = 0;

        if(VSWR_result == 1 ) swr_low = 1;
      
        if(VSWR_result == 3 && swr_low == 1){ 
          swr_low2=current_position;
          int position_difference = swr_low2-swr_low1; 
          position_difference=position_difference/2;
          optimum_position = current_position - position_difference;
          stepper.moveTo(optimum_position);
          stepper.runToPosition();
          motor_direction = 0;
         }
     }
     
    int new_position = stepper.currentPosition();
   
    //Serial.print("### (CP 103) Motor Position:");
    //Serial.println (new_position);
    
    VSWR_result=VSWR_check();
    //Serial.print ("VSWR Result code: ");
    //Serial.println (VSWR_result);
    
    
    if (VSWR_result!=0)
    Tune_state (VSWR_result, new_position, current_frequency, last_tuned_position);
    else tune_fail();
   //Serial.println ("### (CP 102)");
}



// TUNE status check function
void Tune_state (int swr_status, int current_position, long current_frequency, int last_position){
  //Serial.println ("Entering TUNE_STATE function");
  
  if (TUNE_abort == 1 && swr_status == 1 && current_position != last_position){
    //Write current position to last tuned position EEPROM
    EEPROM.put(EEPROM_location, current_position);
    //Write current frequency to last tuned freq table EEPROM
    EEPROM.put(EEPROM_frequency, current_frequency);

    //Serial.println ("TUNE PASS: New Frequency and Position saved");
    
    //Send Tune pass command to Radio.
    tune_success();
    return;
    }
   
      else if(current_position == last_position && current_frequency==last_tuned_freq){
      
      //Serial.println ("TUNE PASS: Already TUNED");
      //Send Tune pass command to Radio.
      tune_success();
      return;
      }
      else if(TUNE_abort == 1 && swr_status == 4 && current_position != last_position){
      
        //Serial.println ("NOT FOUND: Returning to last tuned position");
        //motor return to old position 
        stepper.moveTo(last_position);
        stepper.runToPosition();
        //Serial.println ("returned to last position");    
        
        //Send Tune fail command to Radio.
        tune_fail();
        return;
        }

        else if(TUNE_abort == 0 && swr_status == 0){
          //no RF
          //Serial.println ("TUNE FAIL: Not enough RF");
          tune_fail();
          return;
          }

          else tune_fail();
  
  
 // //Serial.println ("Exiting TUNE_STATE function");
  return;
  
  }

int VSWR_check(){
  //Serial.println ("Entering VSWR_CHECK function");
  
  
  R = digitalRead(RF_N); 
  swr3 = digitalRead(VSWR3); 
  swr2 = digitalRead(VSWR2); 
  swr15 = digitalRead(VSWR15);
  
    if (R==0 && swr3!=0 && swr2!=0 && swr15!=0) return 4;
    else if (R==0 && swr3==0 && swr2!=0 && swr15!=0) return 3;
      else if (R==0 && swr3==0 && swr2==0 && swr15!=0) return 2; 
        else if (R==0 && swr3==0 && swr2==0 && swr15==0) return 1;
          else if (R!=0 && swr3!=0 && swr2!=0 && swr15!=0) return 0;
            else return 0;
  }


//Tune latch function,
void tune_latch() {
   tune_pressed = HIGH;
  }

// TUNE success function. Save data and instruct radio tune ok
void tune_success(){
          int display_position;
          display_position=stepper.currentPosition();
          digitalWrite (TLATCH, 0);
          digitalWrite (IND_N, 0);
          
          digitalWrite(Enable_PIN, HIGH); // Disabling Motor Driver
          draw(current_frequency, display_position, 5); // int freq, int steps, status code 1:Ready 2:TUNE STARt 3:FAST TUNE 4:FINE TUNE 5:TUNED 6:SLOW SCAN 7:TUNE FAIL 

          
          //Serial.println ("****** TUNE PASS ******");
          //Serial.println ("****** Motor Disable ******");
          //Serial.println ("RESTARTING");
          tune_pressed==LOW;
          delay(3000);
          
          }

// TUNE failure function. instruct radio tune fail and go back to last tuned position
void tune_fail(){

          //Send fail command to radio
          digitalWrite (TLATCH, 0);
          //Serial.println("FAIL");
          //Serial.println("Tune LATCH Release..");
          //delay(50); 
          digitalWrite (IND_N, 0);
          delay(20);
          digitalWrite (IND_N, 1);
          delay(200);
          digitalWrite (IND_N, 0);
          
          //Tuner go back to previous state
          //Serial.println ("Returning to last tuned position");
         //motor return to old position 
          stepper.moveTo(last_tuned_position);
          stepper.runToPosition();
          //Serial.println ("returned to last tuned position");    
          
          digitalWrite(Enable_PIN, HIGH); // Disabling Motor Driver
          draw(last_tuned_freq, last_tuned_position, 7); // int freq, int steps, status code 1:Ready 2:TUNE STARt 3:FAST TUNE 4:FINE TUNE 5:TUNED 6:SLOW SCAN 7:TUNE FAIL 

          //Serial.println ("****** TUNE FAIL ******");
          //Serial.println ("****** Motor Disable ******");
          //Serial.println ("RESTARTING");
          tune_pressed==LOW;
          delay(3000);
          }


 //Frequency counter
unsigned long read_freq(){
    delay (1100);
    if (FreqCount.available()) 
    frq= FreqCount.read();

//Serial.println(frq);
  return frq;
}

// Band position check from table
int reference_point_check(unsigned long band_check_freq){
int position_result;

band_check_freq = band_check_freq * prescaller;
  
  if (band_check_freq>24790 && band_check_freq< 25090 ){
    EEPROM.get(EEPROM_12m_location, position_result);
    return position_result;
  }
  else if (band_check_freq> 20900 && band_check_freq< 21550 ){
    EEPROM.get(EEPROM_15m_location, position_result);
    return position_result;
  }
  else if (band_check_freq> 17968 && band_check_freq< 18268 ){
    EEPROM.get(EEPROM_17m_location, position_result);
    return position_result;
  }
  else if (band_check_freq> 13900 && band_check_freq<14450 ){
    EEPROM.get(EEPROM_20m_location, position_result);
    return position_result;
  }
  else if (band_check_freq> 10000 && band_check_freq<10250 ){
    EEPROM.get(EEPROM_30m_location, position_result);
    return position_result;
  }
  else return 0;
  }

  int goto_base_position(int band){
    if (band>MAX_limit) return 0;
    else {
    draw(current_frequency, band, 3); // int freq, int steps, status code 1:Ready 2:TUNE STARt 3:FAST TUNE 4:FINE TUNE 5:TUNED 6:SLOW SCAN 7:TUNE FAIL   
    int band_L1=band + 10;
    int band_L2=band - 10;
    int temp_location= stepper.currentPosition();
    stepper.moveTo(band);
    stepper.runToPosition();
    int new_band_position = stepper.currentPosition();
    //Serial.print("New BAND Position:");
    //Serial.println(new_band_position);
    //Serial.print("Band Limit L1");
    //Serial.println(band_L1);
    
    //Serial.print("Band Limit L2");
    //Serial.println(band_L2);
    
    if (new_band_position < band_L1 && new_band_position > band_L2 ) return 1;
    else {
      draw(current_frequency, temp_location, 6); // int freq, int steps, status code 1:Ready 2:TUNE STARt 3:FAST TUNE 4:FINE TUNE 5:TUNED 6:SLOW SCAN 7:TUNE FAIL 
      //Serial.println("Going back to slow scan"); 
      stepper.moveTo(temp_location);
      stepper.runToPosition();
      return 0;
      }
  }
  }

  
//Display routine
void drawDisplay(unsigned long int display_FRQ, int display_STP, uint8_t display_STAT)
{
  
  //Signal symbol
  u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2.drawGlyph(0, 16, 247);  
  //gear symbol
  u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2.drawGlyph(0, 40, 129);  
  u8g2.setFont(u8g2_font_VCR_OSD_tn);
  u8g2.setCursor(30, 16);
  u8g2.print(display_FRQ);
  u8g2.setFont(u8g2_font_smart_patrol_nbp_tr);
  u8g2.print(" KHz");    // requires enableUTF8Print()
  u8g2.setFont(u8g2_font_VCR_OSD_tn);
  u8g2.setCursor(30, 40);
  u8g2.print(display_STP);
  u8g2.setFont(u8g2_font_smart_patrol_nbp_tr);
  u8g2.print(" STEP");    // requires enableUTF8Print()
  if (display_STAT==1){
  u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2.drawGlyph(0, 63, 84);
  u8g2.setCursor(30, 63);
  u8g2.setFont(u8g2_font_smart_patrol_nbp_tr);
  u8g2.print("READY");    // requires enableUTF8Print()
  }
  else if (display_STAT==2){
  u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2.drawGlyph(0, 63, 154); 
  u8g2.setCursor(30, 63);
  u8g2.setFont(u8g2_font_smart_patrol_nbp_tr);
  u8g2.print("TUNE START");    // requires enableUTF8Print()
 }
 else if (display_STAT==3){
  u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2.drawGlyph(0, 63, 214);  
  u8g2.setCursor(30, 63);
  u8g2.setFont(u8g2_font_smart_patrol_nbp_tr);
  u8g2.print("FAST TUNE");    // requires enableUTF8Print()
 }
 else if (display_STAT==4){
  u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2.drawGlyph(0, 63, 118); 
  u8g2.setCursor(30, 63);
  u8g2.setFont(u8g2_font_smart_patrol_nbp_tr);
  u8g2.print("FINE TUNE");    // requires enableUTF8Print()
 }
 else if (display_STAT==5){
  u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2.drawGlyph(0, 63, 120);  
  u8g2.setCursor(30, 63);
  u8g2.setFont(u8g2_font_smart_patrol_nbp_tr);
  u8g2.print("TUNED");    // requires enableUTF8Print()
 }
 else if (display_STAT==6){
  u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2.drawGlyph(0, 63, 211); 
  u8g2.setCursor(30, 63);
  u8g2.setFont(u8g2_font_smart_patrol_nbp_tr);
  u8g2.print("SLOW SCAN");    // requires enableUTF8Print()
 }
 else if (display_STAT==7){
  u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2.drawGlyph(0, 63, 121); 
  u8g2.setCursor(30, 63);
  u8g2.setFont(u8g2_font_smart_patrol_nbp_tr);
  u8g2.print("TUNE FAIL");    // requires enableUTF8Print()
 }
  else if (display_STAT==8){
  u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2.drawGlyph(0, 63, 154); 
  u8g2.setCursor(30, 63);
  u8g2.setFont(u8g2_font_smart_patrol_nbp_tr);
  u8g2.print("MANUAL");    // requires enableUTF8Print()
 }
 else if (display_STAT==9){
  u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2.drawGlyph(0, 63, 229); 
  u8g2.setCursor(30, 63);
  u8g2.setFont(u8g2_font_smart_patrol_nbp_tr);
  u8g2.print("MANUAL");    // requires enableUTF8Print()
 }
 
}

void draw(unsigned long int display_FRQ, int display_STP, uint8_t display_STAT)
{
  
  display_FRQ=display_FRQ * prescaller;
    u8g2.firstPage();
    do {
      drawDisplay(display_FRQ, display_STP, display_STAT);
      } while ( u8g2.nextPage() );

  }


void rotary_encoder(){
 
  int rBtn = rotary.push();
  int rLBtn = rotary.pushLong(1000);

  if ( rBtn == 1 ) {
  int manual_location = stepper.currentPosition();
  draw(last_tuned_freq, manual_location, 8); // int freq, int steps, status code 1:Ready 2:TUNE STARt 3:FAST TUNE 4:FINE TUNE 5:TUNED 6:SLOW SCAN 7:TUNE FAIL 8: MANUAL 
  Serial.println("Manual Tune");
  digitalWrite(Enable_PIN, LOW); //Enabling Motor Driver   

  byte btn_stat = 1;
      // Check direction
      unsigned long encoder_time = millis();
      
      EEPROM.get(EEPROM_frequency, last_tuned_freq);

      while (btn_stat == 1){
        int rDir = rotary.rotate();
        int rBtn = rotary.push();
        if ( rDir == 1  ) {
          // CW
          encoder_time=millis();
          manual_location = stepper.currentPosition();
          manual_location = manual_location +1;
          stepper.moveTo(manual_location);
          stepper.runToPosition();
          draw(last_tuned_freq, manual_location, 8); // int freq, int steps, status code 1:Ready 2:TUNE STARt 3:FAST TUNE 4:FINE TUNE 5:TUNED 6:SLOW SCAN 7:TUNE FAIL 8: MANUAL 
        
          ctr++;
       
          Serial.println(ctr);
          lastDir = rDir;
        }
        if ( rDir == 2 ) {
          // CCW
          encoder_time=millis();
          manual_location = stepper.currentPosition();
          manual_location = manual_location -1;
          stepper.moveTo(manual_location);
          stepper.runToPosition();
          draw(last_tuned_freq, manual_location, 8); // int freq, int steps, status code 1:Ready 2:TUNE STARt 3:FAST TUNE 4:FINE TUNE 5:TUNED 6:SLOW SCAN 7:TUNE FAIL 8: MANUAL 
          ctr--;
          Serial.println(ctr);
          lastDir = rDir;
        }
        if ( rBtn == 1){
          btn_stat = 0;
          //exit loop
          //exit and save position to eeprom  
          //Write current position to last tuned position EEPROM
          manual_location = stepper.currentPosition();
          EEPROM.put(EEPROM_location, manual_location);
          //Write current frequency to last tuned freq table EEPROM
          //EEPROM.put(EEPROM_frequency, last_tuned_freq);
           draw(last_tuned_freq, manual_location, 9); // int freq, int steps, status code 1:Ready 2:TUNE STARt 3:FAST TUNE 4:FINE TUNE 5:TUNED 6:SLOW SCAN 7:TUNE FAIL 8: MANUAL        
           Serial.println("EXIT Manual Tune");
           
           //rBtn =0;
           //delay(500);
           ctr=0;
         } 
        //timer check
        if (millis() - encoder_time > 10000){
            //Write current position to last tuned position EEPROM
            manual_location = stepper.currentPosition();
            EEPROM.put(EEPROM_location, manual_location);
            //Write current frequency to last tuned freq table EEPROM
            //EEPROM.put(EEPROM_frequency, last_tuned_freq);
                        
            Serial.println("5 seconds no change, save to eeprom");
            encoder_time=millis();
        }
      }
      digitalWrite(Enable_PIN, HIGH); //Disabling Motor Driver   
  }

/*
  // For future additional function by long press
  if ( rLBtn == 1 ) {
  
  Serial.println("Long Button pressed");

  byte btn_stat = 1;
      // Check direction
      
      while (btn_stat == 1){
        int rDir = rotary.rotate();
        int rBtn = rotary.push();
        if ( rDir == 1  ) {
          // CW
          
          ctr++;
          Serial.println(ctr);
          lastDir = rDir;
          }
        if ( rDir == 2 ) {
          // CCW
          
          ctr--;
          Serial.println("do nothing");
          lastDir = rDir;
          }
        if (ctr>=2){
          Serial.println("run save position function");
          ctr=0;
          
             
          
          break;
          }
        
        if (rBtn == 1){
          btn_stat = 0;
          //exit loop
          //exit without anything          
           Serial.println("EXIT Long press functions");
           
           //rBtn =0;
           //delay(500);
         }
      } 
        
        
      }*/
}

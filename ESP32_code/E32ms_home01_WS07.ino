// ESP32 update for the NodeMCU8266 with motor shield robot - using the ESP8226 - ESP32 adaptor v3.0
// E32ms_home01_WS05.ino - web server version 04 for the 38 pin ESP32 module connected to the ESP8266 motor shield
//   - builds upon NMms_home01_web_server_20.ino but completely rewritten still using the normal WebServer
//  code developed and compiled using the Arduino IDE v1.8.5
//  ESP32 Dev Module set as the Board in the IDE - using board version 1.0.4
//   even though the board is probably the NodeMCU-32S i.e. 38 pin ESP32
//  Flash Size: 4MB
//  Partition Scheme set to: Default 4MB with spiffs (1.2 MB App / 1.5MB SPIFFS)
// Core Debug Level set to: "Error"
//   lots of other Board settings! but none experimented with

// SPIFF memory initially uploaded using the separate file 'data' upload method

// different versions of this code are used for different robots where the following hard coded parameters are changed:
// - line 61:    String version = "04";
// - line 64:    int IPnum = 10;
// - line 70:    String namehost = "NodeMCU01";
// - line 73:    String softAPref = "home01";

// code uses the slide switch settings to determine what action is taken
// ** opmodes for 3 switches, 1 of which one is the GO/STOP **
// opmode = 4 means AD  softAP web server mode
// opmode = 5 means AC  local WiFi web server mode
// opmode = 6 means BC  autonomous run around mode
// opmode = 8 means BD  demo mode
// opmode = 9 means an undefined operational mode

// both options 4 and 5 use a web server to provide a web interface to control the motors, update files, etc: 
//  now with:
//  - auto stop added to the run-around mode and extensive use of default parameters stored in SPIFFS
//  - demo mode run from a 'action' list file that can be edited from a web page
//  - auto run  and demo run added to web options

#include "WiFi.h"
#include "WebServer.h"
#include <WiFiClient.h>
#include <ESPmDNS.h>         // Include the mDNS library
#include "SPIFFS.h"

#include <StringArray.h>
#include "Arduino.h"
#include "FS.h"
#include "SPIFFS.h"
#include <ESP32Servo.h>  // used for the servo but need to avoid PWM channel conflict
                         //  - so motor PWM channels are set t0 14 & 15

#include "SSD1306Wire.h"        // used for the 128x64 OLED
#include "Open_Sans_Regular_12.h"
#include "Open_Sans_Regular_16.h"
#include "Open_Sans_Regular_18.h"
#include "Open_Sans_Regular_24.h"
#include <LiquidCrystal_I2C.h>  // used for the 16x2 LCD

// -------------------------------------------------------
//  define all the main variables here to make them global
//  some of these variables need ONLY be defined here as 
//  their values are 'read' later from SPIFFS files
// -------------------------------------------------------

// some WiFi variables set here to be global but the WiFi config is done in 
//   the void loop so that it can be either SoftAP or connection to a local WiFi

String version = "05";

// IP addresses for the **soft AP** operation option 4 (AD) switch setting
int IPnum = 10;
IPAddress local_IP(10,0,5,IPnum);     // using a 10.x.x.x IP address space
IPAddress gateway(10,0,5,1);
IPAddress subnet(255,255,255,0);

// hostname as a char variable
const char* namehost = "NodeMCU01";

// softAP device ref
String softAPref = "home01";

WebServer server(80);  //instantiate the web server at port 80 (http port#)

int loopout = 0; // flag for print out at the start of the 'loop' cycle

// buzzer tunes parameters
String notes[110] = {};  // initialise the notes string array
int num_notes;
String str_notes;
String twinkle_melody[42] = {}; // initialise the twinkle_melody string array
String twinkle_tempo[42] = {};  // initialise the twinkle_tempo string array
int num_twinkle = 42;
String adventure_melody[44] = {}; // initialise the adventure_melody string array
String adventure_tempo[44] = {};  // initialise the adventure_tempo string array
int num_adventure = 44;
int buzzpin = 13;    // buzzer GPIO pin

// network credentials to connect to local WiFi networks for the option 5 (AC) switch setting
//  these variables are simply initialised here since it is assumed that all the values have
//  already been 'written' to the appropraite SPIFFS files or will be populated via the web 
//  interface using the softAP option
// N.B. variables are all strings BUT thse need to be converted to a char for use with the WiFi
String ssid_selected ="";
int NSSIDs = 5;   // number of local WiFi credentials that can be set up: the first one found is used
String ssid1 = " ";
String ssid2 = " ";
String ssid3 = " ";
String ssid4 = " ";
String ssid5 = " ";
String password1 = " ";
String password2 = " ";
String password3 = " ";
String password4 = " ";
String password5 = " ";

// demo action variables: option 8 - slide switch BD, runs these actions on a continuous basis when in GO mode
int num_demo = 40;              // maximum number of demo actions allowed
String demo_controls[40] ={};   // pairs of strings in an array that define the demo actions
String demo_actions[9] ={};     // array of all the potential demo action command strings

// variable to indicate whether either of the two WiFi options have been set up
// this is needed since the setup is different per switch option, so the set up
//   must be done within the "void loop()" and reset if the switch settings change
const char* WiFiup = "no";

String page_content = "";   // initialised global variable for web page content
String web_server = "off";  // 'off' just means the robot is in logical STOP mode so a limited web page is shown

String server_started = "no";  // logic flag so the server is not started until a WiFi connection has been set up

// ************************************************
// define all the motor operation variables
//  again just initialised here as the values are
//  assumed to be 'read' from SPIFFS files
// ************************************************
int turntime = 1200;     // time in ms to do a 90-deg turn (read later from SPIFFS)
int spintime = 1200;     // time in ms to do a 90-deg spin (read later from SPIFFS)
int turnspeed = 90;      // turning speed as a % (read later from SPIFFS)
int spinspeed = 90;      // spinning speed as a % (read later from SPIFFS)

// power adjustment for left/right motors (read later from SPIFFS) i.e.
// set motorL to <1 if turning to the right or 
//     motorR to <1 if turning to the left
float motorL = 1.0;   
float motorR = 1.0;

int r_speed = 90;         // default speed % as an integer value  0-100%

// ** motor drive **
// pins used when the ESP32 + adaptor are inserted into the motor shield
// pin 19 -  motor A: HIGH for 100% forward or PWM duty cycle 0-255 for PWM%
// pin 18 -  motor B: HIGH for 100% forward or PWM duty cycle 0-255 for PWM%
// pin 5  -  motor A direction: LOW = FWD - HIGH = BACK
// pin 17 -  motor B direction: LOW = FWD - HIGH = BACK

int PWMpinA = 19;
int PWMpinB = 18;
int dirpinA = 5;
int dirpinB = 17;

// motor A wired as the left motor when facing forwards
// motor B wired as the right motor when facing forwards

// ***************************************************************************
// set the PWM parameters for use with the LED PWM function which is used
// for motorcontrol - ledc must be used as analogWrite is not on the ESP32
// 16 channels can be used and resolution=8 gives PWM range 0-255 i.e. 2**8-1
//  - but need to avoid channels 0 & 1 to avoid servo conflict
// ***************************************************************************
const int freq = 300;
const int pwmChannelA = 14;  // using a channel at the end of the allowed number
const int pwmChannelB = 15;  //  which seems to avoid a conflict with the servo function
const int resolution = 8;
int dutyCycle = 0;
int dutythreshold = 110;   // minimum duty cycle needed to turn the motors @10%

// ***************************************************************
// set the robot state variable used in the HTML page generation
//  for various logic checks e.g.  to colour the buttons
// 0 means: robot STOP
// 1 means: robot FORWARDS
// 2 means: robot BACKWARDS
// 3 means: robot EMERGENCY STOP
// ***************************************************************
int robot_state = 0;

// ***************************************************************
// set the web state variable used in various places
//  for various logic checks e.g.  in bump detection logic
// 0 means: undefined stste
// 1 means: 'run about' selected
// 2 means: 'auto run' selected
// 3 means: 'demo' selected
// 4 means: 'component testing' selected
// 5 means: 'update parameters' selected
// 6 means: 'system info' selected
// 7 means: '??' selected
// ***************************************************************
int web_state = 0;

// *************************************
// set all the slide switch variables
// *************************************
int s_debug = 1;       // 1 = debug on  0 = debug off
int num_switches = 3;  // this variable is hard-coded as either 3 or 4 so that the same 
                       //  logic/function can be used with different numbers of switches
                       //  the current ESP32 robot build only uses 3 switches 
int opmode = 0;        // set to an initial IDLE state
int opmode_last = 10;  // set to an initial impossible value
int swmode = 0;        // set to an initial OFF state
int swmode_last = 10;  // set to an initial impossible value

// set ESP32 slide switch pin variables
//  these are fully set initially here otherwise nothing can be operated!!
//   even though all the GPIO pins can (exceptionally!) be updated
//   via the web interface if for some reason the robot is rewired
int s_AB  = 16;
int s_CD  = 4;
int s_EF  = 99;  // not used in early NodeMCU builds so num_switches above should be set to 3
int onoff = 23;

// set initial switch states to something undefined so that the program checks can run from the start
int state_onoff = 2;
int state_AB = 2;
int state_CD = 2;
int state_EF = 2;

// set which switch setting modes are active, i.e. programmed (1) or inactive (0)
//  will need to updated in the code if more switch setting options are coded
int mode_1 = 0;
int mode_2 = 0;
int mode_3 = 0;
int mode_4 = 1;  // softAP web interface
int mode_5 = 1;  // local WiFi web interface
int mode_6 = 1;  // autonomous run around
int mode_7 = 0; 
int mode_8 = 1;  // demo mode

// ***************************
// ultrasonic sensor variables
// ***************************
// uses an HC-SR04P (3V3 version) Ultrasonic Sensor connected to the #2 (trig) and #15 (echo) pins
int trigPin = 2;    // pin #2 used for the sensor Ping Pin (Trig)
int echoPin = 15;   // pin #15 used for the sensor Echo Pin (Echo)
unsigned long duration, inches;
int indec, cmdec;  // not needed??
int inchconv = 147; // ratio between pulse width and inches
int cmconv = 59;    // ratio between pulse width and cm
int avoid_distance = 20;   // set a defined distance to take avoidance action
int avoid_limit = 2;       // number of times distance limit is reached before action taken - avoids spurious readings
int avoid_count = 0;       // number of times that distance limit is reached
int cm, lcm, rcm;          // calculated U/S distance variables for static or servo mount options

const char* leftright = "right";
int avoidspintime = 600;   // time in ms to spin in the avoidance routine
int ReverseTime = 350;     // time in ms to reverse in the avoidance routine
String stopauto = "yes";   // set whether autostop should be used
String cm_str = "";

// *******************************************
// set the various servo variables
// *******************************************
// uses a SG90 servo connected to the #27 pin via the servo1 pins on the the adaptor PCB
//  recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
int swivelPin = 27;     // this is servo1 on the adaptor PCB
int swivelCentre = 100; // set this 'offset' to be dead ahead setting (or adjust servo horn mounting)
                        // for servo installed with wires exiting from the front: -'ve = right & +'ve = left
Servo swivelServo;      // N.B. this seems to always take channels 0 & 1 so need to use others for motor PWM

// *******************************************
// set the various RGB LED variables
// *******************************************
const int redPin = 33;
const int greenPin = 25;
const int bluePin = 26;

// ******************************
// set the bump switch variables
// ******************************
int bumpPin = 14;
bool bumpsensed = false;

// ********************************************************
// String versions of the various integer variables
// - easier to deal with in SPIFFS and web pages! even 
//   though this creates overhead with various conversions
// ********************************************************
String str_NSSIDs = String(NSSIDs);
String str_mode_1 = String(mode_1);
String str_mode_2 = String(mode_2);
String str_mode_3 = String(mode_3);
String str_mode_4 = String(mode_4);
String str_mode_5 = String(mode_5);
String str_mode_6 = String(mode_6);
String str_mode_7 = String(mode_7);
String str_mode_8 = String(mode_8);

String str_num_switches = "3";
String str_s_AB ="16";
String str_s_CD = "4";
String str_s_EF = "99";
String str_onoff = "23";

String str_trigPin = "2";
String str_echoPin = "15";

String str_inchconv;
String str_cmconv;
String str_avoid_distance;
String str_stop_distance;
String str_avoid_limit;
String str_avoidspintime;
String str_turntime;
String str_spintime;
String str_turnspeed;
String str_spinspeed;
String str_r_speed;
String str_ReverseTime;

String str_bumpPin = "14";

String str_motorL;   
String str_motorR;

// ******************************
// installed components variables
// ******************************
String str_LCD16x2 = "no";     // yes/no flag for whether a 16x2 LCD is installed
String str_LCD16x2_last = "no";
String str_OLED64x128 ="no";   // yes/no flag for whether a 64x128 OLED is installed
String str_OLED64x128_last = "no";
String str_bump = "no";        // yes/no flag for whether a bump switch is installed
String str_USservo = "yes";    // yes/no flag for whether a servo for the ultrasonic sensor is installed

// *********************
// web page variables
// *********************
String header_content;
int stop_distance;    // emergency stop distance for web use

// *****************************
// ** initial set up function **
// *****************************
void setup() {

    // ********************
    // prepare the RGB LED 
    // ********************
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);
    RGB_color("red");  // set to red to signify the start of the program

    // ******************************************************
    // start serial monitor for all the 'printed' debug info
    // ******************************************************
    Serial.begin(115200);
    delay(2000);              // short pause for the Serial Monitor to be set up before printing
    Serial.println("    ");   // do a few prints to the Serial Monitor as the first few are sometimes missed
    Serial.println("    ");
    Serial.println("    ");
    Serial.print("WS program version ");
    Serial.print(version);
    Serial.println(" starting ...........");

    Serial.print("setup() running on core ");
    Serial.println(xPortGetCoreID());
    
    // ********************************
    // start the SPI Flash File System
    // ********************************
    if(!SPIFFS.begin(true)){
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    } else {
        Serial.println("SPIFFS mounted OK");
    }

    // *****************************************
    // List all the SPIFFS system info (FSinfo)
    // *****************************************
    Serial.print("SPIFFS total available bytes: ");
    Serial.println(SPIFFS.totalBytes());
    Serial.print("SPIFFS used bytes: ");
    Serial.println(SPIFFS.usedBytes());

    // *******************************************
    // prepare the buzzer and the tune parameters
    // *******************************************
    pinMode(buzzpin, OUTPUT);
    str_notes = read_text("/numnotes.txt");
    num_notes = str_notes.toInt();

    // read notes from file
    File notesfile = SPIFFS.open("/notes.txt");
    if(!notesfile){
        Serial.println("Failed to open notesfile for reading");
        return;
    }
    read_strings("/notes.txt", notes, num_notes);

    // load song melodies and tempos from SPIFFS
    read_strings("/twinkle_twinkle_melody.txt", twinkle_melody, num_twinkle);
    read_strings("/twinkle_twinkle_tempo.txt", twinkle_tempo, num_twinkle);
    read_strings("/adventure_time_melody.txt", adventure_melody, num_adventure);
    read_strings("/adventure_time_tempo.txt", adventure_tempo, num_adventure);
    Serial.println ("Song melodies and tempos loaded from SPIFFS");

    // **************************************************************
    // read all the default setting data from the SPIFFS text files
    //  with the exception of the slide switch GPIO pins all these 
    //  files are assumed to already exist - further code versions 
    //  could make this more robust !    
    // **************************************************************
    //Serial.println("-------------------------------");
    read_strings("/demo_controls.txt", demo_controls, num_demo);
   
    demo_actions[0] = "FWD";
    demo_actions[1] = "BACK";
    demo_actions[2] = "DELAY";
    demo_actions[3] = "STOP";
    demo_actions[4] = "SPINL";
    demo_actions[5] = "SPINR";
    demo_actions[6] = "TURNL";
    demo_actions[7] = "TURNR";
    demo_actions[8] = "PLAY";

    // check demo_controls has sensible demo_actions
    Serial.println("checking demo_controls");
    for (int i=0; i<=num_demo-1; i=i+2){
        for (int j=0; j<=8; j=j+1) {
            if (demo_controls[i] == demo_actions[j]) {
                Serial.print(demo_actions[j]); 
                Serial.println(" action found");
                break;
            }
        }
    }
    Serial.println("demo_controls check finished");    

    Serial.println("-------------------------------"); 
    str_NSSIDs = read_text("/str_NSSIDs.txt");
    Serial.println("-------------------------------");
    ssid1 = read_text("/ssid1.txt");
    ssid2 = read_text("/ssid2.txt");
    ssid3 = read_text("/ssid3.txt");
    ssid4 = read_text("/ssid4.txt");
    ssid5 = read_text("/ssid5.txt");
    Serial.println("-------------------------------");
    password1 = read_text("/password1.txt");
    password2 = read_text("/password2.txt");
    password3 = read_text("/password3.txt");
    password4 = read_text("/password4.txt");
    password5 = read_text("/password5.txt");
    Serial.println("-------------------------------");
    str_r_speed = read_text("/str_r_speed.txt");
    Serial.println("-------------------------------");
    str_mode_1 = read_text("/str_mode_1.txt");
    str_mode_2 = read_text("/str_mode_2.txt");
    str_mode_3 = read_text("/str_mode_3.txt");
    str_mode_4 = read_text("/str_mode_4.txt");
    str_mode_5 = read_text("/str_mode_5.txt");
    str_mode_6 = read_text("/str_mode_6.txt");
    str_mode_7 = read_text("/str_mode_7.txt");
    str_mode_8 = read_text("/str_mode_8.txt");
    Serial.println("-------------------------------");

    if (SPIFFS.exists("/str_num_switches.txt")) {
        str_num_switches = read_text("/str_num_switches.txt");
    } else {
        str_num_switches = String(num_switches);
        write_text("/str_num_switches.txt", str_num_switches);
    }

    if (SPIFFS.exists("/str_s_AB.txt")) {
        str_s_AB = read_text("/str_s_AB.txt");
    } else {
        str_s_AB = String(s_AB);
        write_text("/str_s_AB.txt", str_s_AB);
    }

    if (SPIFFS.exists("/str_s_CD.txt")) {
        str_s_CD = read_text("/str_s_CD.txt");
    } else {
        str_s_CD = String(s_CD);
        write_text("/str_s_CD.txt", str_s_CD);
    }

    if (SPIFFS.exists("/str_s_EF.txt")) {
        str_s_EF = read_text("/str_s_EF.txt");
    } else {
        str_s_EF = String(s_EF);
        write_text("/str_s_EF.txt", str_s_EF);
    }

    if (SPIFFS.exists("/str_onoff.txt")) {
        str_onoff = read_text("/str_onoff.txt");
    } else {
        str_onoff = String(onoff);
        write_text("/str_onoff.txt", str_onoff);
    }
    Serial.println("-------------------------------");

    // read the display installed files and set up displays that are installed
    str_LCD16x2 = read_text("/str_LCD16x2.txt");
    if (str_LCD16x2 == "yes" and str_LCD16x2_last == "no") {
        lcdtext(0, 0, "Hello, World!", "LCD initialised");
        str_LCD16x2_last = "yes";
        // print message
        Serial.println("\n16x2 LCD initialised");
    }

    str_OLED64x128 = read_text("/str_OLED64x128.txt");
    if (str_OLED64x128 == "yes" and str_OLED64x128_last == "no") {
        oledclear();
        oledtext(3, 0, 0, 0, 0, 16, "Hello, world!", "Hello, world!", "Hello, world!", "");
        Serial.println("\n128x64 OLED initialised");
        str_OLED64x128_last = "yes";
    }

    str_bumpPin = read_text("/str_bumpPin.txt");
    str_bump = read_text("/str_bump.txt");
    str_USservo = read_text("/str_USservo.txt");
    Serial.println("-------------------------------");


    str_trigPin = read_text("/str_trigPin.txt");
    str_echoPin = read_text("/str_echoPin.txt");
    Serial.println("-------------------------------");

    str_inchconv = read_text("/str_inchconv.txt");
    str_cmconv = read_text("/str_cmconv.txt");
    str_avoid_distance = read_text("/str_avoid_distance.txt");
    str_stop_distance = read_text("/str_stop_distance.txt");
    str_avoid_limit = read_text("/str_avoid_limit.txt");
    str_avoidspintime = read_text("/str_avoidspintime.txt");
    str_turntime = read_text("/str_turntime.txt");
    str_spintime = read_text("/str_spintime.txt");
    str_turnspeed = read_text("/str_turnspeed.txt");
    str_spinspeed = read_text("/str_spinspeed.txt");
    str_r_speed = read_text("/str_r_speed.txt");
    str_motorL = read_text("/str_motorL.txt");
    str_motorR = read_text("/str_motorR.txt");
    str_ReverseTime = read_text("/str_ReverseTime.txt");
    stopauto = read_text("/str_stopauto.txt");
    Serial.println("-------------------------------");

    // ********************************************************************
    // convert the integer and float strings back into integers and floats
    // ********************************************************************
    mode_1 = str_mode_1.toInt();
    mode_2 = str_mode_2.toInt();
    mode_3 = str_mode_3.toInt();
    mode_4 = str_mode_4.toInt();
    mode_5 = str_mode_5.toInt();
    mode_6 = str_mode_6.toInt();
    mode_7 = str_mode_7.toInt();
    mode_8 = str_mode_8.toInt();

    num_switches = str_num_switches.toInt();
    s_AB = str_s_AB.toInt();
    s_CD = str_s_CD.toInt();
    s_EF = str_s_EF.toInt();
    onoff = str_onoff.toInt();

    trigPin = str_trigPin.toInt();
    echoPin = str_echoPin.toInt();

    inchconv = str_inchconv.toInt();
    cmconv = str_cmconv.toInt();
    avoid_distance = str_avoid_distance.toInt();
    stop_distance = str_stop_distance.toInt();
    avoid_limit = str_avoid_limit.toInt();
    avoidspintime = str_avoidspintime.toInt();
    turntime = str_turntime.toInt();
    spintime = str_spintime.toInt();
    turnspeed = str_turnspeed.toInt();
    spinspeed = str_spinspeed.toInt();
    r_speed = str_r_speed.toInt();
    motorL = str_motorL.toFloat();
    motorR = str_motorR.toFloat();
    ReverseTime = str_ReverseTime.toInt();

    bumpPin = str_bumpPin.toInt();

    // *****************************************************
    // prepare Motor control pins with both motors 'stopped'
    // *****************************************************

    pinMode(PWMpinA, OUTPUT);
    digitalWrite(PWMpinA, LOW);
    pinMode(PWMpinB, OUTPUT);
    digitalWrite(PWMpinB, LOW);
    pinMode(dirpinA, OUTPUT);
    digitalWrite(dirpinA, LOW);
    pinMode(dirpinB, OUTPUT);
    digitalWrite(dirpinB, LOW);

    // configure LED-like PWM functionalities for the motors
    ledcSetup(pwmChannelA, freq, resolution);
    ledcSetup(pwmChannelB, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(PWMpinA, pwmChannelA);
    ledcAttachPin(PWMpinB, pwmChannelB);

    // ***************************************
    // prepare ultrasonic sensor control pins 
    // ***************************************
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // *******************
    // prepare the servo
    // *******************
    swivelServo.attach(swivelPin, 600, 2200);  // configures pin, min, max
                                               // min = pulse width for the min (0-degree) angle (default is 544)
                                               // max = pulse width for the max (180-degree) angle (default is 2400)
    swivel(0);

    Serial.println();
    Serial.println("setting up interrupt handlers...");

    // *****************************************************
    // prepare the bump switch (even if it is not installed)
    // *****************************************************
    pinMode(14, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(14), detectbump, FALLING);  // calls detectbump on detection of a HIGH to LOW fall event

    // ************************************************************************
    // interrupt 'handlers' triggered by various Web server requests:
    //  each specific URL that is detected by the web server invokes a 
    //  designated function that does something and then refreshes the web page
    // ************************************************************************
    // ** do this if web root i.e. / is requested
    server.on("/", handle_root);

    // ****** main selection actions ******
    // ** do this if /run_about is requested
    server.on("/run_about", handle_run_about);
    // ** do this if /auto_run is requested
    server.on("/auto_run", handle_auto_run);
    // ** do this if /demo_run is requested
    server.on("/demo_run", handle_demo_run);
    // ** do this if /run_tests is requested
    server.on("/run_tests", handle_run_tests);
    // ** do this if /parameters is requested
    server.on("/parameters", handle_parameters);
    // ** do this if /sysinfo is requested
    server.on("/sysinfo", handle_sysinfo);

    // ****** sub selection parameter update actions ******
    // ** do this if /WiFi_params is requested
    server.on("/WiFi_params", handle_WiFi_params);
    // ** do this if /robot_updates is requested
    server.on("/robot_updates", handle_robot_updates);
    // ** do this if /GPIO_updates is requested
    server.on("/GPIO_updates", handle_GPIO_updates);
    // ** do this if /demo_updates is requested
    server.on("/demo_updates", handle_demo_updates);

    // ****** detailed parameter update submission actions ******
    // ** do this if /WiFi_updates1 is requested
    server.on("/WiFi_updates1", handle_WiFi_updates1);
    // ** do this if /WiFi_updates2 is requested
    server.on("/WiFi_updates2", handle_WiFi_updates2);
    // ** do this if /WiFi_updates3 is requested
    server.on("/WiFi_updates3", handle_WiFi_updates3);
    // ** do this if /WiFi_updates4 is requested
    server.on("/WiFi_updates4", handle_WiFi_updates4);
    // ** do this if /WiFi_updates5 is requested
    server.on("/WiFi_updates5", handle_WiFi_updates5);

    // ** do this if /robot_params is requested
    server.on("/robot_params", handle_robot_params);
    // ** do this if /GPIO_pins is requested
    server.on("/GPIO_pins", handle_GPIO_pins);
    // ** do this if /demo_file is requested
    server.on("/demo_file", handle_demo_file);

    // ****** robot auto run control actions ******
    // ** do this if /run_auto is requested
    server.on("/run_auto", handle_run_auto);
    // do this if /run_stop is requested
    server.on("/run_stop", handle_run_stop);
    // ****** web input actions ******
    // do this if /autospeed is requested as part of an input form
    server.on("/autospeed", handle_autospeed);

    // ****** robot demo run control actions ******
    // ** do this if /run_demo is requested
    server.on("/run_demo", handle_run_demo);
    // do this if /stop_demo is requested
    server.on("/stop_demo", handle_stop_demo);

    // ****** robot run about control actions ******
    // ** do this if /forward is requested
    server.on("/forward", handle_forward);
    // do this if /backward is requested
    server.on("/backward", handle_backward);
    // do this if /halt is requested
    server.on("/halt", handle_stop);
    // do this if /turnleft is requested
    server.on("/turnleft", handle_turnleft);
    // do this if /turnright is requested
    server.on("/turnright", handle_turnright);
    // do this if /spinleft is requested
    server.on("/spinleft", handle_spinleft);
    // do this if /spinright is requested
    server.on("/spinright", handle_spinright);
    // ****** web input actions ******
    // do this if /postspeed is requested as part of an input form
    server.on("/postspeed", handle_postspeed);

    // ****** component testing control actions ******
    // ** do this if /testspeed is requested
    server.on("/testspeed", handle_testspeed);
    // ** do this if /leftfwd is requested
    server.on("/leftfwd", handle_leftfwd);
    // ** do this if /leftback is requested
    server.on("/leftback", handle_leftback);
    // ** do this if /rightfwd is requested
    server.on("/rightfwd", handle_rightfwd);
    // ** do this if /rightback is requested
    server.on("/rightback", handle_rightback);
    // ** do this if /teststop is requested
    server.on("/teststop", handle_teststop);
    // ** do this if /USread is requested
    server.on("/USread", handle_USread);
    // ** do this if /servo_left is requested (left = +'ve angles when servo wires exit from the front)
    server.on("/servo_left", handle_servo_left);
    // ** do this if /servo_right is requested (right = -'ve angles when servo wires exit from the front)
    server.on("/servo_right", handle_servo_right);
    // ** do this if /RGB_red is requested 
    server.on("/RGB_red", handle_RGB_red);
    // ** do this if /RGB_green is requested 
    server.on("/RGB_green", handle_RGB_green);
    // ** do this if /RGB_blue is requested 
    server.on("/RGB_blue", handle_RGB_blue);
    // ** do this if /buzzbeep is requested 
    server.on("/buzzbeep", handle_buzzbeep);
    // ** do this if /twinkle is requested 
    server.on("/twinkle", handle_twinkle);
    // ** do this if /testLCD is requested 
    server.on("/testLCD", handle_testLCD);
    // ** do this if /testOLED is requested 
    server.on("/testOLED", handle_testOLED);

    Serial.println();
    Serial.println("all interrupt handlers set up");

    // web server not started here as its needs the WiFi to be 'up'
    //  and this is only done when a specific slide switch is set 
 
    // ** do a one-time population of the common header HTML used in all web pages
    header_content = HTMLheader();

    RGB_color("blue");  // set to blue to signify completion of the initial set up

}


// ***********************************
// ** continuous operation function **
// ***********************************
void loop() {
    // the code here runs repeatedly in a loop

    if (loopout == 0) {
        Serial.print("loop() running on core ");
        Serial.println(xPortGetCoreID());
        loopout = 1;  // set the loopout parameter so that the loop is only executed once
    }
    
    // get the latest swmode and opmodes from the switch settings
    swmode = check_onoff(swmode_last, s_debug, onoff);
    opmode = check_slideswitch(num_switches, opmode_last, s_debug, s_AB, s_CD, s_EF);
    if (opmode != opmode_last) {  // print out the opmode if it has changed but don't reset so it can be done later
        Serial.print (" new switch settings and therefore new opmode: ");
        Serial.println(opmode);
        // but change WiFi status since this may have to be reset
        WiFiup = "no";
    }

    // ** only opmodes 4, 5, 6 & 8 are possible with 3 switches so only these are currently checked in this code **

    // ##############################################################
    // ## opmode 4 inactive: run web server with the softAP option ##
    // ##############################################################
    if (opmode == 4 and swmode == 0) {
        stop();     // stop the motors just in case something has been left running
        swivel(0);  // ... and centre the servo
        if (mode_4 == 0) {
            if (opmode_last != 4 or swmode_last !=0) {
                Serial.println ("opmode 4 - (AD) not set up yet - CODE INACTIVE ");
                opmode_last = 4;
                swmode_last = 0;
                RGB_color("red");  // set to red to signify an unallowed operation
                if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "softAP mode", "code not", "set up", "");}
                if (str_LCD16x2 == "yes") {lcdtext(0, 0, "softAP mode", "code not set up");}
            }
        } else {           
            if  (opmode_last != 4 or swmode_last !=0 ) {           
                Serial.println ("opmode 4 - 3 switches: (AD) web server operation - in logical STOP mode");
                opmode_last = 4;
                swmode_last = 0;
                web_server = "off";  // set flag to show minimal web content
                RGB_color("blue");  // set to blue to signify an allowed operation but in STOP mode
                if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "softAP mode", "STOP set", "", "");}
                if (str_LCD16x2 == "yes") {lcdtext(0, 0, "softAP mode", "STOP set");}
            }
            // run limited opmode4 actions for STOP condition

            if (WiFiup == "no") {
                setupSoftAP();
            }

            server.handleClient();  // look for an HTTP request from a browser
        }

    }

    // ##############################################################
    // ##  opmode 4 active: run web server with the softAP option  ##
    // ##############################################################
    else if (opmode == 4 and swmode == 1) {
        if (mode_4 == 0) {
            stop();     // stop the motors just in case something has been left running
            swivel(0);  // ... and centre the servo
            if (opmode_last != 4 or swmode_last !=1) {
                Serial.println ("opmode 4 - (AD) not set up yet - CODE INACTIVE ");
                opmode_last = 4;
                swmode_last = 1;
                RGB_color("red");  // set to red to signify an unallowed operation
                if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "softAP mode", "code not", "set up", "");}
                if (str_LCD16x2 == "yes") {lcdtext(0, 0, "softAP mode", "code not set up");}
            }
        } else {           
            if  (opmode_last != 4 or swmode_last !=1 ) {           
                Serial.println ("opmode 4 - 3 switches: (AD) web server operation - in logical GO mode");
                stop();             // stop the motors just as a start condition
                web_server = "on";  // set flag to show full web content
                opmode_last = 4;
                swmode_last = 1;
                RGB_color("green");  // set to green to signify an allowed operation in GO mode
                if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "softAP mode", "GO set", "", "");}
                if (str_LCD16x2 == "yes") {lcdtext(0, 0, "softAP mode", "GO set");}
            }

            // run opmode4 actions for GO condition
            if (WiFiup == "no") {
                setupSoftAP();
            }

            int obdist = getDistance();     // check distance for potential need for an autostop
            // keep checking the distance from the ultrasonic sensor but only when going forward
            if (robot_state == 1 and obdist < stop_distance and stopauto == "yes") 
            {
               handle_diststop();      // stops robot 'outside' of the web interface
               Serial.print ("emergency stop distance: ");
               Serial.println (obdist);
            }

            server.handleClient();  // look for an HTTP request from a browser
        }

    }

    // #########################################################################
    // ## opmode 5 inactive: run web server with local WiFi connection option ##
    // #########################################################################
    else if (opmode == 5 and swmode == 0) {
        stop();     // stop the motors just in case something has been left running
        swivel(0);  // ... and centre the servo
        if (mode_5 == 0) {
            if (opmode_last != 5 or swmode_last !=0) {
                Serial.println ("opmode 5 - (AD) not set up yet - CODE INACTIVE ");
                opmode_last = 5;
                swmode_last = 0;
                RGB_color("red");  // set to red to signify an unallowed operation
                if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "local WiFi mode", "code not", "set up", "");}
                if (str_LCD16x2 == "yes") {lcdtext(0, 0, "local WiFi mode", "code not set up");}
            }
        } else {           
            if  (opmode_last != 5 or swmode_last !=0 ) {           
                Serial.println ("opmode 4 - 3 switches: (AD) web server operation - in logical STOP mode");
                opmode_last = 5;
                swmode_last = 0;
                web_server = "off";  // set flag to show minimal web content
                RGB_color("blue");  // set to blue to signify an allowed operation but in STOP mode
                if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "local WiFi mode", "STOP set", "", "");}
                if (str_LCD16x2 == "yes") {lcdtext(0, 0, "local WiFi mode", "STOP set");}
            }
            // run limited opmode5 actions for STOP condition

            if (WiFiup == "no") {
                setuplocalWiFi();
            }

        }
    }

    // ########################################################################
    // ##  opmode 5 active: run web server with local WiFi connection option ##
    // ########################################################################
    else if (opmode == 5 and swmode == 1) {
        if (mode_5 == 0) {
            if (opmode_last != 5 or swmode_last !=1) {
                Serial.println ("opmode 5 - 3 switches: (AC) not set up yet - CODE INACTIVE ");
                opmode_last = 5;
                swmode_last = 1;
                RGB_color("red");  // set to red to signify an unallowed operation
                if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "local WiFi mode", "code not", "set up", "");}
                if (str_LCD16x2 == "yes") {lcdtext(0, 0, "local WiFi mode", "code not set up");}
            }
        } else {           
            if  (opmode_last != 5 or swmode_last !=1 ) {           
                Serial.println ("opmode 5 - 3 switches: (AC) in logical GO mode");
                opmode_last = 5;
                swmode_last = 1;
                web_server = "on";  // set flag to show full web content
                RGB_color("green");  // set to green to signify an allowed operation in GO mode
                if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "local WiFi mode", "GO set", "", "");}
                if (str_LCD16x2 == "yes") {lcdtext(0, 0, "local WiFi mode", "GO set");
            }
            // run opmode5 actions for GO condition

            if (WiFiup == "no") {
                setuplocalWiFi();
            }

            int obdist = getDistance();     // check distance for potential need for an autostop
            // keep checking the distance from the ultrasonic sensor but only when going forward
            if (robot_state == 1 and obdist < stop_distance and stopauto == "yes")
            {
               handle_diststop();    // stops robot 'outside' of the web interface
               Serial.print ("emergency stop distance: ");
               Serial.println (obdist);
            }

            server.handleClient();  // look for an HTTP request from a browser to provide limited content
        }

    }

    // #####################################################
    // ## opmode 6 inactive: autonomous run around mode   ##
    // #####################################################
    else if (opmode == 6 and swmode == 0) {
	      stop();     // stop the motors just in case something has been left running
          swivel(0);  // ... and centre the servo
        // if here then opmode 6 selected but it is inactive                                       
        if (opmode_last != 6 or swmode_last !=0) { 
            Serial.println ("opmode 6 - 3 switches: (BC) autonomous run around mode - in logical STOP mode");
            opmode_last = 6;
            swmode_last = 0;
            RGB_color("blue");  // set to blue to signify an allowed operation but in STOP mode
            if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "auto run mode", "STOP set", "", "");}
            if (str_LCD16x2 == "yes") {lcdtext(0, 0, "auto run mode", "STOP set");
        }
    }

    // ####################################################
    // ##  opmode 6 active: autonomous run around mode   ##
    // ####################################################
    else if (opmode == 6 and swmode == 1) {

        if (mode_6 == 0) {
            if (opmode_last != 6 or swmode_last !=1) {
                Serial.println ("opmode 6 - 3 switches: (BC) autonomous run around mode - CODE INACTIVE ");
                opmode_last = 6;
                swmode_last = 1;
                RGB_color("red");  // set to red to signify an unallowed operation
                if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "auto run mode", "code not", "set up", "");}
                if (str_LCD16x2 == "yes") {lcdtext(0, 0, "auto run mode", "code not set up");}
            }
        } else {           
            if  (opmode_last != 6 or swmode_last !=1 ) {           
                Serial.println ("opmode 6 - 3 switches: (BC) autonomous run around mode - in logical GO mode");
                opmode_last = 6;
                swmode_last = 1;
                RGB_color("green");  // set to green to signify an allowed operation in GO mode
                if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "auto run mode", "GO set", "", "");}
                if (str_LCD16x2 == "yes") {lcdtext(0, 0, "auto run mode", "GO set");
            }
            // run opmode6 actions

            forwards(80);
            delay(100);       // keep going forward and check the distance ahead every 100 ms
            cm = getDistance();

            if (cm < avoid_distance) {  // if an object is less than the avoid distance then 'count' the # of times this has happened
                avoid_count = avoid_count + 1;
                Serial.print(" distance: ");
                Serial.println(cm);
                Serial.print(" avoid count: ");
                Serial.println(avoid_count);
            }
  
            if(avoid_count >= avoid_limit) {  // if avoid count limit is reached then take avoidance action
                avoid_count = 0;  // reset the count limit first
                stop();
                if (str_USservo == "yes") {
                    auto_avoid();       // this routine moves servo to check left/right distances before altering direction
                } else {
                    auto_avoid_static();   // this routine just does a simple backup and an alternating change of direction
                }
            }

        }

    }

    // ################################################################
    // ## opmode 8 inactive: demo mode using the demo_controls file  ##
    // ################################################################
    else if (opmode == 8 and swmode == 0) {
        stop();     // stop the motors just in case something has been left running
        swivel(0);  // ... and centre the servo
        // if here then opmode 8 selected but it is inactive                                       
        if (opmode_last != 8 or swmode_last !=0) { 
            Serial.println ("opmode 8 - 3 switches: (BD) simple demo mode - in logical STOP mode");
            opmode_last = 8;
            swmode_last = 0;
            RGB_color("blue");  // set to blue to signify an allowed operation but in STOP mode
        }
    }

    // ################################################################
    // ##  opmode 8 active: demo mode using the demo_controls file   ##
    // ################################################################
    else if (opmode == 8 and swmode == 1) {
        if (mode_8 == 0) {
            if (opmode_last != 8 or swmode_last !=1) {
                Serial.println ("opmode 8 - 3 switches: (BD) simple demo mode - CODE INACTIVE ");
                opmode_last = 8;
                swmode_last = 1;
                RGB_color("red");  // set to red to signify an unallowed operation
                if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "demo mode", "code not", "set up", "");}
                if (str_LCD16x2 == "yes") {lcdtext(0, 0, "demo mode", "code not set up");}
            }
        } else {           
            if  (opmode_last != 8 or swmode_last !=1 ) {           
                Serial.println ("opmode 8 - 3 switches: (BD) simple demo mode - in logical GO mode");
                opmode_last = 8;
                swmode_last = 1;
                RGB_color("green");  // set to green to signify an allowed operation in GO mode
                if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "demo mode", "GO set", "", "");}
                if (str_LCD16x2 == "yes") {lcdtext(0, 0, "demo mode", "GO set");
            }
            // run opmode8 actions
            for (int i=0; i<=num_demo-1; i=i+2){
                //action = demo_controls[i];
                //act_int = demo_controls[i+1].toInt();
                // decode the action and execute it
                Serial.print ("demo action:");
                //Serial.print (action);
                Serial.print (demo_controls[i]);
                Serial.print ("-");
                Serial.print (demo_controls[i].length());
                Serial.print ("-");
                //Serial.println (act_int);
                Serial.println (demo_controls[i+1].toInt());
                demo_act(demo_controls[i], demo_controls[i+1].toInt());
            }
        }
    }
}

// ********************************************************************
//  this routine is automatically called when a bump event is detected
// ********************************************************************
void detectbump() {
    bumpsensed = true;
    Serial.println("bump detected");


}

// *****************************************
// *******  RGB LED colour set *************
// *****************************************
void RGB_color(String colour)
{
    if (colour == "red") {
        digitalWrite(redPin, HIGH);
        digitalWrite(greenPin, LOW);
        digitalWrite(bluePin, LOW);
    } else if (colour == "green") {
        digitalWrite(redPin, LOW);
        digitalWrite(greenPin, HIGH);
        digitalWrite(bluePin, LOW);
    } else if (colour == "blue") {
        digitalWrite(redPin, LOW);
        digitalWrite(greenPin, LOW);
        digitalWrite(bluePin, HIGH);
    } else if (colour == "off") {
        digitalWrite(redPin, LOW);
        digitalWrite(greenPin, LOW);
        digitalWrite(bluePin, LOW);
    } else {
        Serial.println("unrecognised RGB setting");
    }

}


// *****************************************************************************************
// simple routine to set the servo to an angle from its centred position
// **************************************************************************************
void swivel(int angle)
{
    if (angle < -80) {
        angle = -80;          // set a minimum relative angle that the servo can 'swivel'
    } else if (angle > +80) { 
        angle = +80;          // set a maximum relative angle that the servo can 'swivel'
    }     
    swivelServo.write(swivelCentre + angle);  // set the servo angle relative to the 'swivelCentre' offset
    delay(600);                               // wait for servo to get there
}


//
// *****************************************************************************************
// simple routine to just check the ON/OFF slide switch state and return the swmode variable
// type int as it returns the swmode as a simple integer
// debug set to 1 gives additional outputs - but the routine is otherwise 'silent'
// **************************************************************************************
//
int check_onoff(int swmode_last, int s_debug, int onoff)
{
    // check for just the ON/OFF slide switch setting
    // swmode_last is a simple passed parameter of the current/last swmode setting
    // swmode = 0 means 'OFF' and swmode = 1 means 'ON'

  	int swmode=10;  // initially set to a non-normal value

    int state_onoff = digitalRead(onoff);
    // if the on/off switch is low (=0), it's OFF
    if (state_onoff == 0)
    {
        if (swmode_last!=0 && s_debug==1)
        {
            Serial.println("** The on/off switch is now OFF \n");
            Serial.println("** slide switches set to IDLE mode \n");

        }
		    swmode=0;    //  switch mode is OFF
    }
	
    else if (state_onoff == 1)
    {
        if (swmode_last!=1 && s_debug==1)
        {
            Serial.println("** The on/off switch is now ON \n");
            Serial.println("** slide switches set to an active mode for whatever opmode is set \n");

        }
		    swmode=1;    //  switch mode is ON
    }

    return swmode; 

}


//
// *****************************************************************************************
// motor control functions using PWM speed control - battery power could provide upto 8V (6xAA) 
// but a motor will generally not run with less than 4V so only accept values >100 and 
// take the 'percent' % input value passed as a parameter as the 100-255 range
// **************************************************************************************
//

void forwards(int percent){
    // set the green LED on and blue + red off
    RGB_color("green");

    // pwr is a duty cycle derived from the percent value in the range from dutythreshold to 255
    int pwr;
    if (percent == 0) { 
        pwr = 0;
    } else {    
        pwr = dutythreshold + percent*(255-dutythreshold)/100;
    }
    // adjust left/right power
    int pwrL = pwr*motorL;
    int pwrR = pwr*motorR;
    Serial.print("left motor power (dutythreshold to 255): ");
    Serial.println(pwrL);    
    Serial.print("right motor power (dutythreshold to 255): ");
    Serial.println(pwrR);
    ledcWrite(pwmChannelA, pwrL); // motor A (left) PWM speed
    ledcWrite(pwmChannelB, pwrR); // motor B (right) PWM speed
    digitalWrite(dirpinA, LOW);  // motor A forward
    digitalWrite(dirpinB, LOW);  // motor B forward
}

void leftforwards(int percent){
    // set the green LED on and blue + red off
    RGB_color("green");
    
    // pwr is a duty cycle derived from the percent value in the range from dutythreshold to 255
    int pwr;
    if (percent == 0) { 
        pwr = 0;
    } else {    
        pwr = dutythreshold + percent*(255-dutythreshold)/100;
    }
    // adjust left power
    int pwrL = pwr*motorL;
    Serial.print("left motor power (100 to 255): ");
    Serial.println(pwrL);    
    ledcWrite(pwmChannelA, pwrL); // motor A (left) PWM speed
    digitalWrite(dirpinA, LOW);   // motor A forward
}

void rightforwards(int percent){
    // set the green LED on and blue + red off
    RGB_color("green");
    
    // pwr is a duty cycle derived from the percent value in the range from dutythreshold to 255
    int pwr;
    if (percent == 0) { 
        pwr = 0;
    } else {    
        pwr = dutythreshold + percent*(255-dutythreshold)/100;
    }
    // adjust right power
    int pwrR = pwr*motorR;
    Serial.print("right motor power (100 to 255): ");
    Serial.println(pwrR); 
    ledcWrite(pwmChannelB, pwrR); // motor B (right) PWM speed
    digitalWrite(dirpinB, LOW);   // motor B forward
}

void stop(){

    // set the red LED on and blue + green off
    RGB_color("red");

    digitalWrite(PWMpinA, LOW);
    ledcWrite(pwmChannelA, 0);
    digitalWrite(PWMpinB, LOW);
    ledcWrite(pwmChannelB, 0);
    digitalWrite(dirpinA, LOW);
    digitalWrite(dirpinB, LOW);
}


void backwards(int percent){

    // set the green LED on and blue + red off
    RGB_color("green");

    // pwr is a duty cycle derived from the percent value in the range from dutythreshold to 255
    int pwr;
    if (percent == 0) { 
        pwr = 0;
    } else {    
        pwr = dutythreshold + percent*(255-dutythreshold)/100;
    }
    int pwrL = pwr*motorL;
    int pwrR = pwr*motorR;
    Serial.print("left motor power (100 to 255): ");
    Serial.println(pwrL);    
    Serial.print("right motor power (100 to 255): ");
    Serial.println(pwrR);
    ledcWrite(pwmChannelA, pwrL); // motor A (left) PWM speed
    ledcWrite(pwmChannelB, pwrR); // motor B (right) PWM speed
    digitalWrite(dirpinA, HIGH);  // motor A backward
    digitalWrite(dirpinB, HIGH);  // motor B backward
}

void leftbackwards(int percent){
    // set the green LED on and blue + red off
    RGB_color("green");
    
    // pwr is a duty cycle derived from the percent value in the range from dutythreshold to 2553
    int pwr;
    if (percent == 0) { 
        pwr = 0;
    } else {    
        pwr = dutythreshold + percent*(255-dutythreshold)/100;
    }
    int pwrL = pwr*motorL;
    Serial.print("left motor power (100 to 255): ");
    Serial.println(pwrL);    
    ledcWrite(pwmChannelA, pwrL); // motor A (left) PWM speed
    digitalWrite(dirpinA, HIGH);  // motor A backward
}

void rightbackwards(int percent){
    // set the green LED on and blue + red off
    RGB_color("green");;
    
    // pwr is a duty cycle derived from the percent value in the range from dutythreshold to 255
    int pwr;
    if (percent == 0) { 
        pwr = 0;
    } else {    
        pwr = dutythreshold + percent*(255-dutythreshold)/100;
    }
    int pwrR = pwr*motorR;
    Serial.print("right motor power (100 to 255): ");
    Serial.println(pwrR);
    ledcWrite(pwmChannelB, pwrR); // motor B (right) PWM speed
    digitalWrite(dirpinB, HIGH);  // motor B backward
}


void turn_left(int t_time, int percent){

    // set the blue LED on and green + red off
    RGB_color("blue");

    int pwr;
    if (percent == 0) { 
        pwr = 0;
    } else {    
        pwr = dutythreshold + percent*(255-dutythreshold)/100;
    }
    // adjust left/right power
    int pwrR = pwr*motorR;
    digitalWrite(PWMpinA, LOW);   // turn motor A (left) off
    ledcWrite(pwmChannelA, 0);
    ledcWrite(pwmChannelB, pwrR); // set motor B (right) to the passed turning speed
    digitalWrite(dirpinA, LOW);   // set motor A 'nominally' to a forward direction
    digitalWrite(dirpinB, LOW);   // run motor B (right) forwards
    delay(t_time);                // amount of time (ms) the motor is run to turn 90-deg
    stop();
}


void turn_right(int t_time, int percent){

    // set the blue LED on and green + red off
    RGB_color("blue");

    int pwr;
    if (percent == 0) { 
        pwr = 0;
    } else {    
        pwr = dutythreshold + percent*(255-dutythreshold)/100;
    }
    // adjust left/right power
    int pwrL = pwr*motorL;
    ledcWrite(pwmChannelA, pwrL); // set motor A (left) to the passed turning speed
    digitalWrite(PWMpinB, LOW);   // turn motor B (right) off
    ledcWrite(pwmChannelB, 0);
    digitalWrite(dirpinA, LOW);   // run motor A (left) forwards
    digitalWrite(dirpinB, LOW);   // set motor B 'nominally' to a forward direction
    delay(t_time);                // amount of time (ms) the motor is run to turn 90-deg
    stop();
}  


void spin_left(int s_time, int percent){

    // set the blue LED on and green + red off
    RGB_color("blue");

    int pwr;
    if (percent == 0) { 
        pwr = 0;
    } else {    
        pwr = pwr = dutythreshold + percent*(255-dutythreshold)/100;
    }
    // adjust left/right power
    int pwrL = pwr*motorL;
    int pwrR = pwr*motorR;
    ledcWrite(pwmChannelA, pwrL);  // set motor A (left) to the passed turning speed
    ledcWrite(pwmChannelB, pwrR);  // set motor B (right) to the passed turning speed
    digitalWrite(dirpinA, HIGH);   // run motor A (left) backwards
    digitalWrite(dirpinB, LOW);    // run motor B (right) forwards
  	delay(s_time);                 // amount of time (ms) the motor is run to spin 90-deg
    stop();
}


void spin_right(int s_time, int percent){

    // set the blue LED on and green + red off
    RGB_color("blue");

    int pwr;
    if (percent == 0) { 
        pwr = 0;
    } else {    
        pwr = pwr = dutythreshold + percent*(255-dutythreshold)/100;
    }
    // adjust left/right power
    int pwrL = pwr*motorL;
    int pwrR = pwr*motorR;
    ledcWrite(pwmChannelA, 200);   // set motor A (left) to the passed turning speed
    ledcWrite(pwmChannelB, 200);   // set motor B (right) to the passed turning speed
    digitalWrite(dirpinA, LOW);    // run motor A (left) forwards
    digitalWrite(dirpinB, HIGH);   // run motor B (right) backwards
    delay(s_time);                 // amount of time (ms) the motor is run to spin 90-deg
    stop();
}


//
// **************************************************************************************
// routine to check slide switch state and change the opmode setting accordingly
// type int as it returns the opmode as a simple integer where the options are set out below
//  n_switch: provides the number of switches in use (usually just 3 for early NodeMCU builds
//  opmode_last: provides the last value of opmode for various logic checks
//  s_debug: if set to 1 gives additional outputs - but the routine is otherwise 'silent'
//  s_AB, s_CD and s_EF: provide the GPIO# for the switches, 2 or 3 of which set activity options
//
//  switch state calculation:  state_AB   state_CD   state_EF are either 0 or 1
// 
// **************************************************************************************
//
int check_slideswitch(int n_switch, int opmode_last, int s_debug, int s_AB, int s_CD, int s_EF)
{
    // check for each slide switch setting
    // opmode_last is a simple passed parameter of the current/last opmode setting
    // opmode = 0 means 'idle' with the ON/OFF switch in the OFF position - but not checked here
    // ** opmodes for 4 switches 1 of which is the GO/STOP **
    // opmode = 1 means ACE
    // opmode = 2 means BCE
    // opmode = 3 means ADE
    // opmode = 4 means ADF
    // opmode = 5 means ACF
    // opmode = 6 means BCF
    // opmode = 7 means BDE
    // opmode = 8 means BDF
    // opmode = 9 means an undefined operational mode

    // ** opmodes for 3 switches 1 of which one is the GO/STOP **
    // opmode = 4 means AD  softAP web server mode
    // opmode = 5 means AC  local WiFi web server mode
    // opmode = 6 means BC  autonomous run around mode
    // opmode = 8 means BD  demo mode
    // opmode = 9 means an undefined operational mode

    opmode=10;  // initially set to a non-normal value

    // read the current switch states
    state_AB = digitalRead(s_AB);
    state_CD = digitalRead(s_CD);
    if (n_switch == 4) {
        state_EF = digitalRead(s_EF);
    } else {
        state_EF = 0;	
    }

    // Now check all the various switch state permutations
	
    // *** ON/OFF slide switch *** not checked here - now in a separate routine (see above)
    //     -------------------
       

    // *** ACE combination - opmode 1 ***
    //     --------------------------
    if (state_AB == 1 && state_CD == 1 && state_EF == 1)
  	{
        if (s_debug==1 && opmode_last != 1)
        {
            Serial.println("** 4 slide switches set to 'ACE'- opmode 1");
        }
        opmode=1;
  	}

    // *** BCE combination - opmode 2 ***
    //     --------------------------
    else if (state_AB == 0 && state_CD == 1 && state_EF == 1)
    {
        if (s_debug==1 && opmode_last != 2)
        {
            Serial.println("** 4 slide switches set to 'BCE'- opmode 2");
        }
        opmode=2;
  	}

    // *** ADE combination - opmode 3 ***
    //     --------------------------
    else if (state_AB == 1 && state_CD == 0 && state_EF == 1)
  	{
        if (s_debug==1 && opmode_last != 3)
        {
            Serial.println("** 4 slide switches set to 'ADE'- opmode 3");
        }
        opmode=3;
  	}

    // *** ADF combination - opmode 4 ***
    //     --------------------------
  	else if (state_AB == 1 && state_CD == 0 && state_EF == 0)
  	{
        if (s_debug==1 && opmode_last != 4 && n_switch == 4)
        {
            Serial.println("** 4 slide switches set to 'ADF'- opmode 4");
        } 
        else if (s_debug==1 && opmode_last != 4 && n_switch == 3)
        {
            Serial.println("** 3 slide switches set to 'AD'- opmode 4");
        }

        opmode=4;
	  }

    // *** ACF combination - opmode 5 ***
    //     --------------------------
  	else if (state_AB == 1 && state_CD == 1 && state_EF == 0)
    {
        if (s_debug==1 && opmode_last != 5 && n_switch == 4)
        {
            Serial.println("** 4 slide switches set to 'ACF'- opmode 5");
        } 
        else if (s_debug==1 && opmode_last != 5 && n_switch == 3)
        {
            Serial.println("** 3 slide switches set to 'AC'- opmode 5");
        }
        opmode=5;
  	}

    // *** BCF combination - opmode 6 ***
    //     --------------------------
	  else if (state_AB == 0 && state_CD == 1 && state_EF == 0)
  	{
        if (s_debug==1 && opmode_last != 6 && n_switch == 4)
        {
            Serial.println("** 4 slide switches set to 'BCF'- opmode 6");
        } 
        else if (s_debug==1 && opmode_last != 6 && n_switch == 3)
        {
            Serial.println("** 3 slide switches set to 'BC'- opmode 6");
        }
        opmode=6;
	  }

    // *** BDE combination - opmode 7 ***
    //     --------------------------
  	else if (state_AB == 0 && state_CD == 0 && state_EF == 1)
    {
        if (s_debug==1 && opmode_last != 7)
        {
            Serial.println("** 4slide switches set to 'BDE'- opmode 7");
        }
        opmode=7;
  	}

    // *** BDF combination - opmode 8 ***
    //     --------------------------
  	else if (state_AB == 0 && state_CD == 0 && state_EF == 0)
  	{
        if (s_debug==1 && opmode_last != 8 && n_switch == 4)
        {
            Serial.println("** 4 slide switches set to 'BDF'- opmode 8");
        } 
        else if (s_debug==1 && opmode_last != 8 && n_switch == 3)
        {
            Serial.println("** 3 slide switches set to 'BD'- opmode 8");
        }
        opmode=8;
	  }

    // *** opmode 99 ***
    //     --------------------------
    else  // should never be here!!
  	{
        if (s_debug==1 && opmode_last != 99 && n_switch == 4)
        {
            Serial.println("** 4 slide switches set to an impossible combination! ");
        } 
        else if (s_debug==1 && opmode_last != 99 && n_switch == 3)
        {
            Serial.println("** 3 slide switches set to an impossible combination! ");
        }
        opmode=99;
  	}

    return opmode;

}

// *******************************************************
// function to sense the distance to an obstacle
// uses the time taken for pulse to be sensed coming back
// and the speed of sound in air as 34326cm/s
// returns a value in cm
// *******************************************************
int getDistance()
{
  int rval;
  // send trigger pulse and time how long it takes to 'hear' it come back
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);   // set the lLOW for a 'stabilising' period so that the HIGH signal is 'clean'
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // if pulseIn times out it probably means the sensor is too close
  //  for the electronics to 'catch' the return signal in time or 
  //  maybe the outbound pulse just got lost i.e. an object is too 
  //  far away so reset the U/S with a LOW and return  a silly distance
  //  that could if necessary be 'checked for'
  duration = pulseIn(echoPin, HIGH, 38000L);  // Set timeout to 38mS
                                              // which sets duration to zero
  if (duration == 0)
  {
    return 999; 
  }
  rval = microsecondsToCentimeters(duration);
//  Serial.println(rval);
  return rval;
}

// **************************************
// ultrasonic sensor conversion function
// **************************************
long microsecondsToCentimeters(long microseconds)
{
  // total distance there and back is usecs*34326/1000000 
  // so half distance is usecs*34326/(1000000*2) = usecs/58.26
  return microseconds / cmconv;
}


// ****************************************
// autonomous obstacle avoidance function
//  for a static mounted ultrasonic sensor
// ****************************************
void auto_avoid_static()
{
    // Move back a little, then turn right or left on alternate tries

    // Back off a little 
    Serial.println(" moving backwards") ;
    backwards(70); 
    delay(ReverseTime);
    stop();

   // Turn right or left
    if (leftright == "right") {
        Serial.println("** spin Right to avoid obstacle **");
        spin_right(avoidspintime, spinspeed);
        leftright = "left";
    } else {
        Serial.println("** spin Left to avoid obstacle **");
        spin_left(avoidspintime, spinspeed);
        leftright = "right";
    }

}

// **************************************
// autonomous obstacle avoidance function
//  for a servo mounted ultrasonic sensor
// **************************************
void auto_avoid()
{
    stop();
    swivel(-55);
    lcm = getDistance();    // measure the distance ahead to the left
    swivel(55);
    rcm = getDistance();    // measure the distance ahead to the right

    // back up and then spin to either left or right to try and avoid object
    backwards(150);
    delay(700);
    if (lcm < rcm) {
        spin_right(avoidspintime, spinspeed);
    } else {
        spin_left(avoidspintime, spinspeed);
    }
    swivel(0);  // reset the servo and return
}


// ******************************************************
// ****          set up 'soft AP' WiFi               ****
// ******************************************************
void setupSoftAP()
{
    Serial.print("Setting soft-AP IP configuration ... ");
    Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");

    Serial.print("Setting soft-AP ssid and password ... ");
    String softAP_str = "ESPsoftAP" + softAPref;
    Serial.println(WiFi.softAP(softAP_str.c_str(), "pswd12345") ? "Ready" : "Failed!");

    Serial.print("Soft-AP IP address = ");
    Serial.println(WiFi.softAPIP());     // IP address shouldn't change so could be used in a browser

    Serial.printf("Soft-AP MAC address = %s\n", WiFi.softAPmacAddress().c_str());

    if (!MDNS.begin(namehost)) {        // Start the mDNS responder for namehost ....local
        Serial.println("Error setting up MDNS responder!");
    }
    Serial.print("mDNS responder started for domain: ");
    Serial.print(namehost);
    Serial.println(".local");    
    delay(2000);
    WiFiup = "yes";

    if (server_started == "no" ) {
        Serial.println();
        Serial.println("starting web server .........");
        // ** start web server
        server.begin();
        Serial.println("Web server started!");
        server_started = "yes";
    }


}


// ********************************************
// **** set up WiFi with host name as well ****
// ********************************************
void setuplocalWiFi()
{
    Serial.print("trying to connect to WiFi with hostname: ");

    Serial.println(namehost);
    //WiFi.hostname(namehost);

    // scan to find all the 'live' broadcast SSID's ....
    int n = WiFi.scanNetworks();
    Serial.print("Number of WiFi networks found: ");
    Serial.println(n);
    ssid_selected ="";

    Serial.print("trying to connect to: ");
    Serial.println(ssid1);
    // try to use ssid1 first
    for (int i = 0; i < n; ++i) {
        Serial.print("Trying SSID: ");
        Serial.println(WiFi.SSID(i));
        if (WiFi.SSID(i)== ssid1 ) {
            ssid_selected = ssid1;
            break;
        }
    }
    if (ssid_selected != "" ) {
        // make connection to selected WiFi
        connectWiFi();
        return;
    }

    // --------------------------------------------
    // now try ssid2 if ssid1 not already selected
    Serial.print("trying to connect to: ");
    Serial.println(ssid2);
    for (int i = 0; i < n; ++i) {
        Serial.print("Trying SSID: ");
        Serial.println(WiFi.SSID(i));
        if (WiFi.SSID(i)== ssid2 ) {
            ssid_selected = ssid2;
            break;
        }       
    }
    if (ssid_selected != "" ) {
        // make connection to selected WiFi
        connectWiFi();
        return;
    }

    // --------------------------------------------
    // now try ssid3 
    Serial.print("trying to connect to: ");
    Serial.println(ssid3);
    for (int i = 0; i < n; ++i) {
        Serial.print("Trying SSID: ");
        Serial.println(WiFi.SSID(i));
        if (WiFi.SSID(i)== ssid3 ) {
            ssid_selected = ssid3;
            break;
        }       
    }
    if (ssid_selected != "" ) {
        // make connection to selected WiFi
        connectWiFi();
        return;
    }

    // --------------------------------------------
    // now try ssid4
    Serial.print("trying to connect to: ");
    Serial.println(ssid4);
    for (int i = 0; i < n; ++i) {
        Serial.print("Trying SSID: ");
        Serial.println(WiFi.SSID(i));
        if (WiFi.SSID(i)== ssid4 ) {
            ssid_selected = ssid4;
            break;
        }       
    }
    if (ssid_selected != "" ) {
        // make connection to selected WiFi
        connectWiFi();
        return;
    }

    // --------------------------------------------
    // now try ssid5
    Serial.print("trying to connect to: ");
    Serial.println(ssid5);
    for (int i = 0; i < n; ++i) {
        Serial.print("Trying SSID: ");
        Serial.println(WiFi.SSID(i));
        if (WiFi.SSID(i)== ssid5 ) {
            ssid_selected = ssid5;
            break;
        }       
    }
    if (ssid_selected != "" ) {
        // make connection to selected WiFi
        connectWiFi();
        return;
    }

    // if here then no allowed local WiFi found 
    Serial.println(" No allowed WiFi found");


}

// ******************************
// **** make WiFi connection ****
// ******************************
void connectWiFi()
{

    Serial.print(ssid_selected);
    Serial.println(" selected - now trying to connect");
	
	  if (ssid_selected == ssid1 ) {
            // convert the selected ssid and password to the char variables
            WiFi.begin(ssid1.c_str(), password1.c_str());    //initiate connection to ssid1
            Serial.print("SSID: ");
            Serial.println(ssid1);

    } else if (ssid_selected == ssid2 ) {
           // convert the selected ssid and password to the char variables
            WiFi.begin(ssid2.c_str(), password2.c_str());    //initiate connection to ssid2
            Serial.print("SSID: ");
            Serial.println(ssid2);

    } else if (ssid_selected == ssid3 ) {
            // convert the selected ssid and password to the char variables
            WiFi.begin(ssid3.c_str(), password3.c_str());    //initiate connection to ssid3
            Serial.print("SSID: ");
            Serial.println(ssid3);

    } else if (ssid_selected == ssid4 ) {
            // convert the selected ssid and password to the char variables
            WiFi.begin(ssid4.c_str(), password4.c_str());    //initiate connection to ssid4
            Serial.print("SSID: ");
            Serial.println(ssid4);

    } else if (ssid_selected == ssid5 ) {
            // convert the selected ssid and password to the char variables
            WiFi.begin(ssid5.c_str(), password5.c_str());    //initiate connection to ssid5
            Serial.print("SSID: ");
            Serial.println(ssid5);

    }
    Serial.println("");
    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    // Connected to the first available/defined WiFi Access Point
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid_selected);
    Serial.print("IP address: ");
    delay(500);
    Serial.println(WiFi.localIP());
    Serial.print("MAC address: ");
    delay(500);
    Serial.println(WiFi.macAddress());
    delay(1000);
    WiFi.setHostname(namehost);
    delay(1500);
    Serial.print("updated hostname: ");
    Serial.println(WiFi.getHostname());
    WiFiup = "yes";

    if (server_started == "no" ) {
        Serial.println();
        Serial.println("starting web server .........");
        // ** start web server
        server.begin();
        Serial.println("Web server started!");
        server_started = "yes";
    }

}


// **********************************************************************
// Function to create/open an existing file and write a single string to 
//  it - the file name and text are passed strings to the function 
// **********************************************************************
void write_text(String wfile, String wtext) {
    //w=Write Open file for writing
    File SPIfile = SPIFFS.open(wfile, "w");
  
    if (!SPIfile) {
        Serial.println("file open failed");
    }
    else
    {
        //Write data to file
        Serial.print("Writing Data to File: ");
        Serial.println(wtext);
        SPIfile.print(wtext);
        SPIfile.close();  //Close file

    }
}


// **********************************************************
// Function to read a single string from a written text file
// **********************************************************
String read_text(String rfile) {
    int i;
    String str_read;
    //open the file for reading
    File f = SPIFFS.open(rfile, "r");
  
    if (!f) {
        str_read = "file open failed";
        Serial.println(str_read);
    }
    else
    {
        Serial.print("Reading Text Data from File ");
        Serial.print(rfile);
        //read string from file
        str_read = f.readStringUntil('\n');
        if (rfile.substring(1,5) == "pass") {
            Serial.println("password not shown");
        } else {
            Serial.print(": ");
            Serial.println(str_read);
        }
        f.close();  //Close file
        Serial.println("File Closed");
    }
    return str_read;

}


// ************************************************************************************
// Function to create/open an existing multiple string file and write data to it
// the local variables: 'wfile' is passed the file name, 'strarray' the string array
//  to be written to the file and int num is the number of array elements to be written
// ************************************************************************************
void write_strings(String wfile, String strarray[], int num) {
    //w=Write: open file for writing from the beginning overwriting whatever is already there
    File SPIfileStr = SPIFFS.open(wfile, "w");
  
    if (!SPIfileStr) {
        Serial.println("file open failed");
    }
    else
    {
        //Write string array data to file
        Serial.println("  ");
        Serial.println("Writing String array data to the file");
        for (int i=0; i<=num-1; i++){
            SPIfileStr.println(strarray[i]);    // writes an individual string with a LF at the end
        }
        SPIfileStr.close();  //Close file
        Serial.println("String array writing complete ");
    }
}

// *****************************************************************************************
// Function to read an existing multiple string file and extract the strings from it
// the local variables: 'rfile' is passed the file name, 'strarray' the string array
//  'pointed' to by reference and int num is the number of string array elements to be read
// *****************************************************************************************
void read_strings(String rfile, String strarray[], int num) {
    String tempstr;
    //r=Read Open file for reading
    File SPIfileStr = SPIFFS.open(rfile, "r");
  
    if (!SPIfileStr) {
        Serial.println("multiple string file open failed");
        strarray[0] = "multiple string file open failed";

    }
    else
    {
        //Read string data from file looking for the LF's that separate each string
        Serial.println("  ");
        Serial.println("Function read_strings: reading 'num' strings from file");
        for (int i=0; i<=num-1; i++){                     // loop thru the 'num' strings in the file
            tempstr=SPIfileStr.readStringUntil('\n'); // [i] string read upto the \n terminator
                               //  the terminator is discarded from the stream but adds a space
            //if (tempstr.length() > 2) {
            //    tempstr.remove(tempstr.length()-1,1);
            //}
            strarray[i] = tempstr;
            //Serial.print(i);
            //Serial.print(": ");
            //Serial.println(strarray[i]);
        }
        SPIfileStr.close();  //Close file
        Serial.println("Function read_strings: string reading complete ");

    }
}


// **********************************************************************
// ***  function to decode the demo_control 'pairs' and execute them  ***
// **********************************************************************
void demo_act(String act, int a_int) {
    Serial.print ("action requested:");
    Serial.print (act);
    Serial.print ("-");
    Serial.print (act.length());
    Serial.print ("-");
    Serial.println (a_int);
    
    if (act == demo_actions[0]) {
        forwards(a_int);
    } else if (act == demo_actions[1]) {
        backwards(a_int);
    } else if (act == demo_actions[2]) {
        delay(a_int);
    } else if (act == demo_actions[3]) {
        stop();
    } else if (act == demo_actions[4]) {
        spin_left(a_int, turnspeed);
    } else if (act == demo_actions[5]) {
        spin_right(a_int, turnspeed);
    } else if (act == demo_actions[6]) {
        turn_left(a_int, turnspeed);
    } else if (act == demo_actions[7]) {
        turn_right(a_int, turnspeed);
    } else if (act == demo_actions[8]) {
        playtune(a_int);
    } else {
        Serial.println("No recognised action found ");
    }
}



// *************************************************************************
// Function to 'sound' the passive buzzer with a specific frequency/duration
// *************************************************************************
int buzz(int frequency, int duration) {   
    // create the function "buzz" and feed it the note (e.g. DS6=1245Hz) and duration (length in ms))
    //Serial.print("Buzzing: pin ");
    //Serial.println(buzzpin);
    //Serial.print("Buzzing: frequency ");   // pitch/frequency of the note
    //Serial.println(frequency);
    //Serial.print("Buzzing: length (ms) "); // length/duration of the note in ms
    //Serial.println(duration);
    if (frequency == 0) {
        delay(duration);
        int buzzstat = 0;
        return buzzstat;
    }
    // from the frequency calculate the time between buzzer pin HIGH-LOW setting in microseconds
    //float period = 1.0 / frequency;       // in physics, the period (sec/cyc) is the inverse of the frequency (cyc/sec)
    int delayValue = (1000*1000/frequency)/2;  // calculate the time in microseconds for half of the wave
    int numCycles = int((duration * frequency)/1000);   // the number of waves to produce is the duration times the frequency
    //Serial.print("Number of cycles: ");
    //Serial.println(numCycles);
    //Serial.print("Hi-Low delay (microsecs): ");
    //Serial.println(delayValue);
    for (int i=0; i<=numCycles-1; i++) {  // start a loop from 0 to the variable "cycles" calculated above
        digitalWrite(buzzpin, HIGH);      // set buzzer pin to high
        delayMicroseconds(delayValue);    // wait with buzzer pin high
        digitalWrite(buzzpin, LOW);       // set buzzer pin to low
        delayMicroseconds(delayValue);    // wait with buzzer pin low
    }

}

// **********************
// simple beep function
// **********************
void beep(int beeptime) {
    // beeptime in seconds
    Serial.println("beeping buzzer at 900Hz for beeptime seconds");
    for (int j=0; j<=beeptime-1; j++) {
        // total duration of all the steps below to add up to 1 second
        buzz(900, 300);
        delay(200);
        buzz(900, 300);
        delay(200);
    }
}

// **********************************************************************************
// Function to play a specific tune/sound depending upon the passed reference number
// **********************************************************************************
void playtune(int tune) {
    // use the tune number to select what is played
    if (tune == 1) {
        Serial.println("beeping buzzer at 900Hz for 5 seconds");
        for (int j=0; j<=4; j++) {
            buzz(900, 500);
            delay(500);
        }
     } else if (tune == 2) {
        Serial.println("playing twinkle twinkle little star");
        play(notes, num_twinkle, num_notes, twinkle_melody, twinkle_tempo, 0.50, 1.000);
     } else if (tune == 3) {
        Serial.println("playing adventure time");
        play(notes, num_adventure, num_notes, adventure_melody, adventure_tempo, 1.3, 1.500);
     } else {
        Serial.println("tune number not valid");
     }
}

// ***********************************************************
// Function to 'play' a defined melody at its associated tempo
// ***********************************************************
void play(String allnotes[], int songnotes, int numnotes, String strmelody[], String strtempo[], float pause, float pace) { 
    // allnotes[]  - array of all the standard notes and their frequency
    // songnotes   - number of notes in the song
    // numnotes    - number of possible standard notes
    // strmelody[] - array of notes in the song
    // strtempo[]  - array of note lengths i.e. 1, 2, 4, 8 etc., where 8=quaver 4=semiquaver etc.
    // pause       - multiplier of the note duration as a pause between notes
    // pace        - used to calculate the note duration = pace/tempo
    float selfreq;
    Serial.print("Playing ");
    Serial.print(songnotes);
    Serial.println(" song notes");
    for (int i=0; i<=songnotes-1; i++) {        // Play strmelody[] song notes
      //esp_task_wdt_reset();  // reset the watchdog timer just in case
      // loop thru all the possible notes to look for the associated frequency for the note to be played
      Serial.print("Playing song note: ");
      Serial.println(strmelody[i]);
      for (int j=0; j<=numnotes-1; j++) {
        String selectnote;
        if (strmelody[i].length() == 2) {
            selectnote = allnotes[j].substring(0,2);
        } else if (strmelody[i].length() == 3) {
            selectnote = allnotes[j].substring(0,3);
        } else {
            Serial.print(" funny number of characters in melody note"); 
        }
        //Serial.print("Is it this note: ");
        //Serial.println(selectnote);
        if (strmelody[i] == selectnote or strmelody[i] == "0") {
          // if here playnote found or freq = 0 - so use its frequency and play the note
          String tempo = strtempo[i];
          int noteDuration = 1000*pace/tempo.toInt();  // length or duration of the note in milliseconds
          if ( strmelody[i] == "0" ) {
            selfreq = 0.0;
            Serial.println("frequency set to zero");
          } else {
            String selfreqstr = allnotes[j].substring(5);
            //Serial.print("associated frequency found: ");
            //Serial.println(selfreqstr);
            selfreq = selfreqstr.toFloat();
            //Serial.print("Playing frequency: ");
            //Serial.println(selfreqstr);
          }
          int buzzstat = buzz(selfreq, noteDuration);      // Change the frequency along the song note  
          float pauseBetweenNotes = noteDuration * pause;
          //Serial.print("pause betwen notes (ms): ");
          //Serial.println(pauseBetweenNotes);
          delay(pauseBetweenNotes);          
          // now break out of the 'note finding' j for loop
          break;
        }
      }

      // now continue to loop through all the song notes
    }
}

void lcdtext(int col1, int col2, String text1, String text2) {
    LiquidCrystal_I2C lcd(0x27, 16, 2);
    lcd.init();
    // turn on LCD backlight                      
    lcd.backlight(); // make sure the backlight is on
    // set cursor to the passed columns for each row
    //   and display text
    lcd.setCursor(col1, 0);
    lcd.print(text1);
    lcd.setCursor(col2, 1);
    lcd.print(text2);
}

void lcdclear(String blonoff) {
    LiquidCrystal_I2C lcd(0x27, 16, 2);
    lcd.init();
    // LCD backlight on or off                   
    if (blonoff == "on") {
        lcd.backlight(); // make sure the backlight is on
    } else if (blonoff == "off") {
        lcd.noBacklight();
    }
    // clear screen
    lcd.clear();
}

void oledtext(int rows, int col1, int col2, int col3, int col4, int fontsize, String text1, String text2, String text3, String text4) {
    SSD1306Wire display(0x3c, 21, 22);
    display.init();  // redeclare and init again??
    display.flipScreenVertically();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    if (fontsize == 12) {
        display.setFont(Open_Sans_Regular_12);
    } else if (fontsize == 16) {
        display.setFont(Open_Sans_Regular_16);
    } else if (fontsize == 18) {
        display.setFont(Open_Sans_Regular_18);
    } else if (fontsize == 24) {
        display.setFont(Open_Sans_Regular_18);
    } else {
        display.setFont(Open_Sans_Regular_12);  // set a default if an unknown font size is set
        Serial.println("Default font size set to 12");
    }
    if (rows == 1) {
        display.drawString(col1, 24, text1);
    } else if (rows == 2) {
        display.drawString(col1, 0, text1);
        display.drawString(col2, 31, text2);
    } else if (rows == 3) {   // normally fontsize should =16
        display.drawString(col1, 0, text1);
        display.drawString(col2, 21, text2);
        display.drawString(col3, 42, text3);
    } else if (rows == 4) {
        display.drawString(col1, 0, text1);
        display.drawString(col2, 16, text2);
        display.drawString(col3, 32, text3);
        display.drawString(col3, 48, text4);
    }
    display.display();
}

void oledclear() {
    SSD1306Wire display(0x3c, 21, 22);
    display.init();  // redeclare and init again??
    display.clear();
    display.displayOff();
    display.end();
}


void printBootReason()
{
    esp_reset_reason_t reset_reason = esp_reset_reason();

    switch (reset_reason)
    {
    case ESP_RST_UNKNOWN:    Serial.println("Reset reason can not be determined"); break;
    case ESP_RST_POWERON:    Serial.println("Reset due to power-on event"); break;
    case ESP_RST_EXT:        Serial.println("Reset by external pin (not applicable for ESP32)"); break;
    case ESP_RST_SW:         Serial.println("Software reset via esp_restart"); break;
    case ESP_RST_PANIC:      Serial.println("Software reset due to exception/panic"); break;
    case ESP_RST_INT_WDT:    Serial.println("Reset (software or hardware) due to interrupt watchdog"); break;
    case ESP_RST_TASK_WDT:   Serial.println("Reset due to task watchdog"); break;
    case ESP_RST_WDT:        Serial.println("Reset due to other watchdogs"); break;
    case ESP_RST_DEEPSLEEP:  Serial.println("Reset after exiting deep sleep mode"); break;
    case ESP_RST_BROWNOUT:   Serial.println("Brownout reset (software or hardware)"); break;
    case ESP_RST_SDIO:       Serial.println("Reset over SDIO"); break;
    }

    if (reset_reason == ESP_RST_DEEPSLEEP)
    {
        esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

        switch(wakeup_reason)
        { 
            case ESP_SLEEP_WAKEUP_UNDEFINED:    Serial.println("In case of deep sleep: reset was not caused by exit from deep sleep"); break;
            case ESP_SLEEP_WAKEUP_ALL:          Serial.println("Not a wakeup cause: used to disable all wakeup sources with esp_sleep_disable_wakeup_source"); break;
            case ESP_SLEEP_WAKEUP_EXT0:         Serial.println("Wakeup caused by external signal using RTC_IO"); break;
            case ESP_SLEEP_WAKEUP_EXT1:         Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
            case ESP_SLEEP_WAKEUP_TIMER:        Serial.println("Wakeup caused by timer"); break;
            case ESP_SLEEP_WAKEUP_TOUCHPAD:     Serial.println("Wakeup caused by touchpad"); break;
            case ESP_SLEEP_WAKEUP_ULP:          Serial.println("Wakeup caused by ULP program"); break;
            case ESP_SLEEP_WAKEUP_GPIO:         Serial.println("Wakeup caused by GPIO (light sleep only)"); break;
            case ESP_SLEEP_WAKEUP_UART:         Serial.println("Wakeup caused by UART (light sleep only)"); break;
        }
    }
}


// *****************************************************************
// ***  this section is for all the browser response 'handlers'  ***
// *****************************************************************

void handle_root() {
    // ** do this if web root i.e. / is requested
    stop();  // stop robot just in case
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "main menu", "selected", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "main menu", "selected");}
    robot_state = 0;
    Serial.println("web root - robot stopped");
    server.send(200, "text/html", HTMLmain()); 
}

// **********************************
// *** main selection 'handlers' ****
// **********************************
// ***** main selection actions *****
void handle_run_about() {
    // ** do this if /run_about is requested
    stop();  // stop robot just in case
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "robot", "run-around", "selected", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "robot run-around", "selected");}
    robot_state = 0;
    web_state = 1;
    Serial.println("selecting robot run-about control");
    server.send(200, "text/html", HTMLrobot_run_about()); 
}
void handle_auto_run() {
    // ** do this if /auto_run is requested
    stop();  // stop robot just in case
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "robot", "auto run", "selected", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "robot auto run", "selected");}
    robot_state = 0;
    web_state = 2;
    Serial.println("selecting robot autonomous operation control");
    server.send(200, "text/html", HTMLrobot_auto_run()); 
}
void handle_demo_run() {
    // ** do this if /demo_run is requested
    stop();  // stop robot just in case
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "robot demo", "selected", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "robot demo", "selected");}
    robot_state = 0;
    web_state = 3;
    Serial.println("selecting robot demo operation control");
    server.send(200, "text/html", HTMLrobot_demo_run()); 
}
void handle_run_tests() {
    // ** do this if /run_tests is requested
    stop();  // stop robot just in case
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "component", "tests", "selected", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "component tests", "selected");}
    robot_state = 0;
    web_state = 4;
    Serial.println("selecting component testing");
    server.send(200, "text/html", HTMLrun_tests()); 
}
void handle_parameters() {
    // ** do this if /parameters is requested
    stop();  // stop robot just in case
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "robot", "details", "selected", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "robot details", "selected");}
    robot_state = 0;
    web_state = 5;
    Serial.println("selecting parameter update");
    server.send(200, "text/html", HTMLparameter_selection()); 
}
void handle_sysinfo() {
    // ** do this if /sysinfo is requested
    stop();  // stop robot just in case
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "system info", "selected", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "system info", "selected");}
    robot_state = 0;
    web_state = 6;
    Serial.println("selecting system information display");
    server.send(200, "text/html", HTMLsysinfo()); 
}

// ******************************************************
// *** sub selection  demo operation 'handlers' ***
// ******************************************************
// ****** robot demo run control actions ******
void handle_run_demo() {
    // ** do this if /run_demo is requested
    stop();  // stop robot just for now
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "robot demo", "running", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "robot demo", "running");}
    swivel(0); // centre the servo
    robot_state = 1;
    web_state = 3;
    Serial.println("putting robot into demo operation");
    while (robot_state == 1) {
        for (int i=0; i<=num_demo-1; i=i+2){
            //action = demo_controls[i];
            //act_int = demo_controls[i+1].toInt();
            // decode the action and execute it
            Serial.print ("demo action:");
            //Serial.print (action);
            Serial.print (demo_controls[i]);
            Serial.print ("-");
            Serial.print (demo_controls[i].length());
            Serial.print ("-");
            //Serial.println (act_int);
            Serial.println (demo_controls[i+1].toInt());
            demo_act(demo_controls[i], demo_controls[i+1].toInt());
            server.send(200, "text/html", HTMLrobot_demo_run()); 
            server.handleClient();  // look for an HTTP request from a browser within the 'while' 
            //  and the 'for' loops so that the demo mode can be interupted after each demo command
            //  e.g. clicking the STOP button stops the robot and sets robot_state to 0 to stop the 
            //  'while' loop - but the check below is additionally needed to immediately break the 'for' loop
            if (robot_state != 1) {
                break;
            }
        }
    }
}
void handle_stop_demo() {
    // do this if /stop_demo is requested
    stop();  // stop robot 
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "robot demo", "stopped", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "robot demo", "stopped");}
    swivel(0); // centre the servo
    web_state = 3;
    robot_state = 0;
    Serial.println("stopping robot's demo operation");
    server.send(200, "text/html", HTMLrobot_demo_run()); 
}

// ******************************************************
// *** sub selection  autonomous operation 'handlers' ***
// ******************************************************
// ****** robot auto run control actions ******
void handle_run_auto() {
    // ** do this if /run_auto is requested
    stop();  // stop robot just for now
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "auto robot", "running", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "auto robot", "running");}
    robot_state = 1;
    web_state = 2;
    Serial.println("putting robot into autonomous operation");
    while (robot_state == 1) {
        forwards(80);
        delay(100);       // keep going forward and check the distance ahead every 100 ms
        cm = getDistance();

        if (cm < avoid_distance) {  // if an object is less than the avoid distance then 'count' the # of times this has happened
            avoid_count = avoid_count + 1;
            Serial.print(" distance: ");
            Serial.println(cm);
            Serial.print(" avoid count: ");
            Serial.println(avoid_count);
        }
  
        if(avoid_count >= avoid_limit) {  // if avoid count limit is reached then take avoidance action
            avoid_count = 0;  // reset the count limit first
            stop();
            if (str_USservo == "yes") {
                auto_avoid();       // this routine moves servo to check left/right distances before altering direction
            } else {
                auto_avoid_static();   // this routine just does a simple backup and an alternating change of direction
            }
        }
        server.send(200, "text/html", HTMLrobot_auto_run()); 
        server.handleClient();  // look for an HTTP request from a browser within the while loop
                                //  so that the auto-run mode can be interupted e.g. clicking the
                                //  web STOP button causes the robot to stop and robot_state set 
                                //  to 0 so the while loop stops as well
    }
}
void handle_run_stop() {
    // do this if /run_stop is requested
    stop();  // stop robot 
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "auto robot", "stopped", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "auto robot", "stopped");}
    robot_state = 0;
    web_state = 2;
    Serial.println("stopping robot's autonomous operation");
    server.send(200, "text/html", HTMLrobot_auto_run()); 
}
// *** web browser robot auto-run input 'handlers' ***
// ****** web input actions ******
void handle_autospeed() {
    // do this if /autospeed is requested as part of an input form
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "robot speed", "updated", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "robot speed", "updated");}
    str_r_speed = server.arg("r_speed_str");  // get string from browser response
    r_speed = str_r_speed.toInt();            // convert string to the integer version
    Serial.print("robot speed input integer: ");
    Serial.println(r_speed);
    robot_state = 0;
    server.send(200, "text/html", HTMLrobot_auto_run());
}

// **************************************************
// *** sub selection  parameter update 'handlers' ***
// **************************************************
void handle_WiFi_params() {
    // ** do this if /WiFi_params is requested
    stop();  // stop robot just in case
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "WiFi updates", "selected", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "WiFi updates", "selected");}
    robot_state = 0;
    web_state = 51;
    Serial.println("selecting WiFi updates");
    server.send(200, "text/html", HTMLWiFi_params()); 
}
void handle_robot_updates() {
    // ** do this if /robot_updates is requested
    stop();  // stop robot just in case
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "robot details", "selected", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "robot details", "selected");}
    robot_state = 0;
    web_state = 52;
    Serial.println("selecting robot parameter updates");
    server.send(200, "text/html", HTMLrobot_updates()); 
}
void handle_GPIO_updates() {
    // ** do this if /GPIO_updates is requested
    stop();  // stop robot just in case
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "GPIO updates", "selected", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "GPIO updates", "selected");}
    robot_state = 0;
    web_state = 53;
    Serial.println("selecting GPIO pin updates");
    server.send(200, "text/html", HTMLGPIO_updates()); 
}
void handle_demo_updates() {
    // ** do this if /demo_updates is requested
    stop();  // stop robot just in case
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "demo commands", "selected", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "demo commands", "selected");}
    robot_state = 0;
    web_state = 54;
    Serial.println("selecting demo file updates");
    server.send(200, "text/html", HTMLdemo_updates()); 
}


// *******************************************************
// *** parameter update submission detailed 'handlers' ***
// *******************************************************
// ****** detailed demo file update submission actions ******
void handle_demo_file() {
    // ** do this if /demo_file is requested
    stop();  // stop robot just in case
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "demo commands", "updated", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "demo commands", "updated");}
    robot_state = 0;
    web_state = 54;
    Serial.println("demo file update");
     // get updated demo action details from browser response 
    int numdemo;
    numdemo = server.arg("demo_num_str").toInt(); 
    // check demo_act_str is an allowable command
    Serial.println("checking demo_controls");
    for (int j=0; j<=8; j=j+1) {
        if (server.arg("demo_act_str") == demo_actions[j]) {
            Serial.print(demo_actions[j]); 
            Serial.println(" action found");
            demo_controls[numdemo*2] = server.arg("demo_act_str");
            break;
        } else {
            Serial.println(" no allowable action found");
            demo_controls[numdemo*2] = "ERROR";
        }
    }
    // to be added later: a value check on 'demo_val_str' as well    
    demo_controls[numdemo*2+1] = server.arg("demo_val_str").toInt();
    // write the updated 'demo_controls' back out to the SPIFFS file
    write_strings("/demo_controls.txt", demo_controls, num_demo);
    server.send(200, "text/html", HTMLdemo_updates());
}


// ****** detailed robot parameter update submission actions ******
void handle_robot_params() {
    // ** do this if /robot_params is requested
    stop();  // stop robot just in case
    robot_state = 0;
    web_state = 52;
    Serial.println("robot parameter update");

    // get robot parameter strings from the browser response
    str_avoid_distance = server.arg("avoid_distance_str"); 
    str_stop_distance = server.arg("stop_distance_str"); 
    str_avoid_limit = server.arg("avoid_limit_str");  
    str_avoidspintime = server.arg("avoidspintime_str"); 
    str_turntime = server.arg("turntime_str"); 
    str_spintime = server.arg("spintime_str");
    str_turnspeed = server.arg("turnspeed_str"); 
    str_spinspeed = server.arg("spinspeed_str");
    str_r_speed = server.arg("r_speed_str"); 
    str_ReverseTime = server.arg("ReverseTime_str"); 
    stopauto = server.arg("stopauto_str");
    str_motorL = server.arg("motorL_str"); 
    str_motorR = server.arg("motorR_str"); 
    str_LCD16x2 = server.arg("LCD16x2_str"); 
    str_OLED64x128 = server.arg("OLED64x128_str"); 
    str_bump = server.arg("bump_str"); 
    str_USservo = server.arg("USservo_str"); 

    // initialise displays if they are newly 'installed' or clear them if now not installed
    if (str_LCD16x2 == "yes" and str_LCD16x2_last == "no") {
        lcdtext(0, 0, "Hello, World!", " ");
        str_LCD16x2_last = "yes";
        Serial.println("\n now installed so 16x2 LCD initialised");
        // print message
        Serial.println("\n16x2 LCD initialised");
    } else if (str_LCD16x2 == "no" and str_LCD16x2_last == "yes") {
        lcdclear("off");  // clear screen and put lightlight off
        Serial.println("\n now uninstalled so 16x2 LCD initialised before display cleared");
        str_LCD16x2_last = "no";
    }

    if (str_OLED64x128 == "yes" and str_OLED64x128_last == "no") {
        oledtext(3, 0, 0, 0, 0, 16, " ", " ", "Hello, world!", " ");
        str_OLED64x128_last = "yes";
        Serial.println("\n now installed so 128x64 OLED initialised");
    } else if (str_OLED64x128 == "no" and str_OLED64x128_last == "yes") {
        oledclear();
        Serial.println("\n now uninstalled so 128x64 OLED cleared");
        str_OLED64x128_last = "no";
    }

    // convert strings back to integers
    avoid_distance = str_avoid_distance.toInt();
    stop_distance = str_stop_distance.toInt();
    avoid_limit = str_avoid_limit.toInt();
    avoidspintime = str_avoidspintime.toInt();
    turntime = str_turntime.toInt();
    spintime = str_spintime.toInt();
    turnspeed = str_turnspeed.toInt();
    spinspeed = str_spinspeed.toInt();
    r_speed = str_r_speed.toInt();
    motorL = str_motorL.toFloat();
    motorR = str_motorR.toFloat();
    ReverseTime = str_ReverseTime.toInt();

    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "robot details", "updated", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "robot details", "updated");}
    // resave the robot parameter strings
    Serial.println("-------------------------------");
    write_text("/str_avoid_distance.txt", str_avoid_distance);
    write_text("/str_stop_distance.txt", str_stop_distance);
    write_text("/str_avoid_limit.txt", str_avoid_limit);
    write_text("/str_avoidspintime.txt", str_avoidspintime);
    write_text("/str_turntime.txt", str_turntime);
    write_text("/str_spintime.txt", str_spintime);
    write_text("/str_turnspeed.txt", str_turnspeed);
    write_text("/str_spinspeed.txt", str_spinspeed);
    write_text("/str_r_speed.txt", str_r_speed);
    write_text("/str_ReverseTime.txt", str_ReverseTime);
    write_text("/str_stopauto.txt", stopauto);
    Serial.println("-------------------------------");
    write_text("/str_motorL.txt", str_motorL);
    write_text("/str_motorR.txt", str_motorR);
    write_text("/str_LCD16x2.txt", str_LCD16x2);
    write_text("/str_OLED64x128.txt", str_OLED64x128);
    write_text("/str_bump.txt", str_bump);
    write_text("/str_USservo.txt", str_USservo);
    Serial.println("-------------------------------");
    Serial.println("robot parameter data resaved ");
    server.send(200, "text/html", HTMLrobot_updates()); 
}

// ****** detailed WiFi parameter update submission actions ******
// *** WiFi update input 'handlers' ***
// -----------------------------------
void handle_WiFi_updates1() {
    // ** do this if /WiFi_updates1 is requested
    stop();  // stop robot just in case
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "WiFi details", "updated", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "WiFi details", "updated");}
    robot_state = 0;
    web_state = 51;
    Serial.println("WiFi SSID1 settings update");
    ssid1 = server.arg("ssid_1");          // get string from browser response
    password1 = server.arg("password_1");  // get string from browser response
    // resave the WiFi SSID and its password
    write_text("/ssid1.txt", ssid1);
    write_text("/password1.txt", password1);
    server.send(200, "text/html", HTMLWiFi_params());
}

void handle_WiFi_updates2() {
    // ** do this if /WiFi_updates2 is requested
    stop();  // stop robot just in case
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "WiFi details", "updated", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "WiFi details", "updated");}
    robot_state = 0;
    web_state = 51;
    Serial.println("WiFi SSID2 settings update");
    ssid2 = server.arg("ssid_2");          // get string from browser response
    password2 = server.arg("password_2");  // get string from browser response
    // resave the WiFi SSID and its password
    write_text("/ssid2.txt", ssid2);
    write_text("/password2.txt", password2);
    server.send(200, "text/html", HTMLWiFi_params());
}

void handle_WiFi_updates3() {
    // ** do this if /WiFi_updates3 is requested
    stop();  // stop robot just in case
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "WiFi details", "updated", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "WiFi details", "updated");}
    robot_state = 0;
    web_state = 51;
    Serial.println("WiFi SSID3 settings update");
    ssid3 = server.arg("ssid_3");          // get string from browser response
    password3 = server.arg("password_3");  // get string from browser response
    // resave the WiFi SSID and its password
    write_text("/ssid3.txt", ssid3);
    write_text("/password3.txt", password3);
    server.send(200, "text/html", HTMLWiFi_params());
}

void handle_WiFi_updates4() {
    // ** do this if /WiFi_updates4 is requested
    stop();  // stop robot just in case
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "WiFi details", "updated", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "WiFi details", "updated");}
    robot_state = 0;
    web_state = 51;
    Serial.println("WiFi SSID4 settings update");
    ssid4 = server.arg("ssid_4");          // get string from browser response
    password4 = server.arg("password_4");  // get string from browser response
    // resave the WiFi SSID and its password
    write_text("/ssid4.txt", ssid4);
    write_text("/password4.txt", password4);
    server.send(200, "text/html", HTMLWiFi_params());
}

void handle_WiFi_updates5() {
    // ** do this if /WiFi_updates5 is requested
    stop();  // stop robot just in case
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "WiFi details", "updated", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "WiFi details", "updated");}
    robot_state = 0;
    web_state = 51;
    Serial.println("WiFi SSID5 settings update");
    ssid5 = server.arg("ssid_5");          // get string from browser response
    password5 = server.arg("password_5");  // get string from browser response
    // resave the WiFi SSID and its password
    write_text("/ssid5.txt", ssid5);
    write_text("/password5.txt", password5);
    server.send(200, "text/html", HTMLWiFi_params());
}

// *** GPIO pin# update input 'handler' ***
// ----------------------------------------
// ****** detailed GPIO pin update submission actions ******
void handle_GPIO_pins() {
    // ** do this if /GPIO_pins is requested
    stop();  // stop robot just in case
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "GPIO pin#", "updated", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "GPIO pin#", "updated");}  
    robot_state = 0;
    web_state = 53;
    str_s_AB = server.arg("s_AB_str");        // get string from browser response
    str_s_CD = server.arg("s_CD_str");        // get string from browser response
    str_onoff = server.arg("onoff_str");      // get string from browser response
    str_trigPin = server.arg("trigPin_str");  // get string from browser response
    str_echoPin = server.arg("echoPin_str");  // get string from browser response

    // convert strings back to integers
    s_AB = str_s_AB.toInt();
    s_CD = str_s_CD.toInt();
    s_EF = str_s_EF.toInt();
    onoff = str_onoff.toInt();
    trigPin = str_trigPin.toInt();
    echoPin = str_echoPin.toInt();

    // resave the GPIO pin# settings as strings
    Serial.println("-------------------------------");
    write_text("/str_s_AB.txt", str_s_AB);
    write_text("/str_s_CD.txt", str_s_CD);
    write_text("/str_onoff.txt", str_onoff);
    write_text("/str_trigPin.txt", str_trigPin);
    write_text("/str_echoPin.txt", str_echoPin);;
    Serial.println("-------------------------------");
    Serial.println("GPIO pin# data resaved ");
    server.send(200, "text/html", HTMLGPIO_updates());
}

// ********************************
// *** robot control 'handlers' ***
// ********************************
// ****** robot run about control actions ******
void handle_stop() {
    // do this if /halt is requested
    stop();  // stop robot
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "robot", "stopped", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "robot", "stopped");}   
    swivel(0); // centre the servo
    robot_state = 0;
    web_state = 11;
    Serial.println("robot stopped");
    server.send(200, "text/html", HTMLrobot_run_about()); 
}
void handle_diststop() {  // special version used for auto stop with object detection
    stop();  // stop robot
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "emergency", "stop", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "emergency", "stop");}
    robot_state = 3;
    web_state = 12;
    Serial.println("robot emergency stopped");
    server.send(200, "text/html", HTMLrobot_run_about()); 
}
void handle_forward() {
    // ** do this if /forward is requested
    forwards(r_speed);   // run robot forwards at r_speed% speed
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "running", "forwards", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "running", "forwards");}
    robot_state = 1;
    web_state = 13;
    Serial.println("robot going forwards");
    server.send(200, "text/html", HTMLrobot_run_about()); 
}
void handle_backward() {
    // do this if /backward is requested
    backwards(r_speed);   // run robot backwards at r_speed% speed
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "running", "backwards", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "running", "backwards");}
    robot_state = 2;
    web_state = 18;
    Serial.println("robot going backwards");
    server.send(200, "text/html", HTMLrobot_run_about()); 
}
void handle_turnleft() {
    // do this if /turnleft is requested
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "turning ", "left ", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "turning left", " ");}
    turn_left(turntime, turnspeed);  // turn the robot left 90-deg
    robot_state = 0;
    web_state = 14;
    Serial.println("robot turned left");
    server.send(200, "text/html", HTMLrobot_run_about()); 
}
void handle_turnright() {
    // do this if /turnright is requested
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "turning ", "right ", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "turning right", " ");}
    turn_right(turntime, turnspeed);  // turn the robot right 90-deg
    robot_state = 0;
    web_state = 15;
    Serial.println("robot turned right");
    server.send(200, "text/html", HTMLrobot_run_about()); 
}
void handle_spinleft() {
    // do this if /spinleft is requested
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "spinning ", "left ", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "spinning left", " ");}
    spin_left(spintime, turnspeed);  // spin the robot left 90-deg
    robot_state = 0;
    web_state = 16;
    Serial.println("robot spun left");
    server.send(200, "text/html", HTMLrobot_run_about());
}
void handle_spinright() {
    // do this if /spinright is requested
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "spinning ", "right ", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "spinning right", " ");}
    spin_right(spintime, turnspeed);  // spin the robot left 90-deg
    robot_state = 0;
    web_state = 17;
    Serial.println("robot spun right");
    server.send(200, "text/html", HTMLrobot_run_about()); 
}
// ****** web input actions ******
// *** web browser robot control input 'handlers' ***
void handle_postspeed() {
    // do this if /postspeed is requested as part of an input form
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "default speed", "updated", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "default speed", "updated");}
    str_r_speed = server.arg("r_speed_str");  // get string from browser response
    r_speed = str_r_speed.toInt();           // convert string to the integer version
    Serial.print("robot speed input integer: ");
    Serial.println(r_speed);
    robot_state = 0;
    web_state = 19;
    server.send(200, "text/html", HTMLrobot_run_about());
}

// ************************************
// *** component testing 'handlers' ***
// ************************************
// ****** component testing control actions ******
void handle_testspeed() {
    // ** do this if /testspeed is requested
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "default speed", "updated", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "default speed", "updated");}
    str_r_speed = server.arg("r_speed_str");  // get string from browser response
    r_speed = str_r_speed.toInt();           // convert string to the integer version
    Serial.print("component testing speed input integer: ");
    Serial.println(r_speed);
    robot_state = 0;
    web_state = 41;
    server.send(200, "text/html", HTMLrun_tests());
}
void handle_USread() {
    // ** do this if /USread is requested
    stop();  // stop both motors just in case

    robot_state = 0;
    web_state = 42;
    RGB_color("blue");
    Serial.println("reading U/S sensor");
    cm = getDistance();
    cm_str = String(cm);
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "ultrasonic", "sensor read", cm_str + "cm", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "ultrasonic", "sensor read");}
    server.send(200, "text/html", HTMLrun_tests()); 
}
void handle_teststop() {
    // ** do this if /teststop is requested
    stop();  // stop both motors
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "everything", "stopped", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "everything", "stopped");}
    robot_state = 0;
    web_state = 43;
    swivel(0); // centre the servo
    RGB_color("off");
    Serial.println("motors stopped, servo centred and LEDs all off");
    delay(2000);
    if (str_LCD16x2 == "yes") {lcdclear("off");}
    if (str_OLED64x128 == "yes") {oledclear();}
    server.send(200, "text/html", HTMLrun_tests()); 
}
void handle_leftfwd() {
    // ** do this if /leftfwd is requested
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "left motor", "forwards", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "left motor", "forwards");}
    leftforwards(r_speed);   // run left motor forwards at r_speed% speed
    robot_state = 4;
    web_state = 44;
    swivel(0); // centre the servo
    Serial.println("left motor forwards");
    server.send(200, "text/html", HTMLrun_tests()); 
}
void handle_rightfwd() {
    // ** do this if /rightfwd is requested
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "right motor", "forwards", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "right motor", "forwards");}
    rightforwards(r_speed);   // run left motor forwards at r_speed% speed
    robot_state = 4;
    web_state = 44;
    swivel(0); // centre the servo
    Serial.println("right motor forwards");
    server.send(200, "text/html", HTMLrun_tests()); 
}
void handle_leftback() {
    // ** do this if /leftback is requested
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "left motor", "backwards", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "left motor", "backwards");}
    leftbackwards(r_speed);   // run left motor forwards at r_speed% speed
    robot_state = 5;
    web_state = 45;
    swivel(0); // centre the servo
    Serial.println("left motor backwards");
    server.send(200, "text/html", HTMLrun_tests()); 
}
void handle_rightback() {
    // ** do this if /rightback is requested
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "right motor", "backwards", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "right motor", "backwards");}
    rightbackwards(r_speed);   // run left motor forwards at r_speed% speed
    robot_state = 5;
    web_state = 45;
    swivel(0); // centre the servo
    Serial.println("right motor backwards");
    server.send(200, "text/html", HTMLrun_tests()); 
}
void handle_servo_left() {
    // ** do this if /servo_left is requested (left = +'ve angles when servo wires exit from the front)
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "left servo", "swivelled", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "right servo", "swivelled");}
    stop();  // stop both motors just in case
    robot_state = 0;
    web_state = 46;
    Serial.println("servo swivelled left");
    swivel(55);
    server.send(200, "text/html", HTMLrun_tests()); 
}
void handle_servo_right() {
    // ** do this if /servo_right is requested (right = -'ve angles when servo wires exit from the front)
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "right servo", "swivelled", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "right servo", "swivelled");}
    stop();  // stop both motors just in case
    robot_state = 0;
    web_state = 46;
    Serial.println("servo swivelled right");
    swivel(-55);
    server.send(200, "text/html", HTMLrun_tests()); 
}
void handle_RGB_red() {
    // ** do this if /RGB_red is requested (right = -'ve angles when servo wires exit from the front)
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "RGB LEDs", "Red LED on", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "RGB LEDs", "Red LED on");}
    stop();  // stop both motors just in case
    robot_state = 0;
    web_state = 47;
    Serial.println("RGB red on");
    RGB_color("red");
    server.send(200, "text/html", HTMLrun_tests()); 
}
void handle_RGB_green() {
    // ** do this if /RGB_green is requested (right = -'ve angles when servo wires exit from the front)
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "RGB LEDs", "Green LED on", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "RGB LEDs", "Green LED on");}
    stop();  // stop both motors just in case
    robot_state = 0;
    web_state = 47;
    Serial.println("RGB green on");
    RGB_color("green");
    server.send(200, "text/html", HTMLrun_tests()); 
}
void handle_RGB_blue() {
    // ** do this if /RGB_blue is requested (right = -'ve angles when servo wires exit from the front)

    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "RGB LEDs", "Blue LED on", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "RGB LEDs", "Blue LED on");}    stop();  // stop both motors just in case
    robot_state = 0;
    web_state = 47;
    Serial.println("RGB blue on");
    RGB_color("blue");
    server.send(200, "text/html", HTMLrun_tests()); 
}
void handle_buzzbeep() {
    // ** do this if /buzzbeep is requested (right = -'ve angles when servo wires exit from the front)
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "buzzing", "for 5 seconds", "", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "buzzing", "for 5 seconds");}
    stop();  // stop both motors just in case
    robot_state = 0;
    web_state = 48;
    Serial.println("beeping buzzer at 900Hz for 5 seconds");
    playtune(1);
    server.send(200, "text/html", HTMLrun_tests()); 
}
void handle_twinkle() {
    // ** do this if /twinkle is requested (right = -'ve angles when servo wires exit from the front)
    // update the display with some status text if they are installed
    if (str_OLED64x128 == "yes") {oledtext(3, 0, 0, 0, 0, 16, "playing", "twinkle", "twinkle", "");}
    if (str_LCD16x2 == "yes") {lcdtext(0, 0, "playing", "twinkle twinkle");}
    stop();  // stop both motors just in case
    robot_state = 0;
    web_state = 48;
    Serial.println("playing twinkle twinkle little star");
    playtune(2);
    server.send(200, "text/html", HTMLrun_tests()); 
}

void handle_testLCD() {
    // ** do this if /testLCD is requested 
    stop();  // stop both motors just in case
    robot_state = 0;
    web_state = 49;
    Serial.println("displaying text on LCD");
    lcdtext(0, 0, "Hello, World!", "LCD test");
    server.send(200, "text/html", HTMLrun_tests()); 
}

void handle_testOLED() {
    // ** do this if /testOLED is requested 
    stop();  // stop both motors just in case
    robot_state = 0;
    web_state = 49;
    Serial.println("displaying text on OLED");
    oledtext(3, 0, 0, 0, 0, 16, "Hello, world!", " ", "OLED test", "");
    server.send(200, "text/html", HTMLrun_tests()); 
}


// ****************************************************************
// ******  create the various web pages that are being used  ******
// ****************************************************************

// --------------------------------------------------------------------------------
// create the header area used in all the web pages - done once in the setup stage
// --------------------------------------------------------------------------------
String HTMLheader() {
    String h_content = "<!DOCTYPE html> <html>\n";
    h_content +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=yes\">\n";
    h_content +="<title>Robot Control</title>\n";
	
    // style content in the page header ( to be replaced by a css file eventually )
    h_content +="<style>html { font-family: Verdana; display: inline-block; margin: 0px auto; text-align: center; font-size: 15px;}\n";
    h_content +="body{margin-top: 50px;} h1 {color: #444444; margin: 10px auto 10px; font-size: 32px;} h3 {color: #444444; margin: 10px auto 10px; font-size: 24px;} h4 {color: #444444; margin: 10px auto 10px; font-size: 18px;}\n";
    h_content +=".button {display: block; width: 80px; background-color: #1abc9c;border: none;color: white; padding: 0px 20px 10px 20px; text-decoration: none; font-size: 32px; margin: 5px auto 5px; cursor: pointer; border-radius: 4px;}\n";
    h_content +=".btninline {display: inline-block; }\n";
    h_content +=".button-on {background-color: #1abc9c;}\n";
    h_content +=".button-on:active {background-color: #16a085;}\n";
    h_content +=".button-off {background-color: #34495e;}\n";
    h_content +=".button-off:active {background-color: #2c3e50;}\n";
    h_content +=".button-red {background-color: #f51031;}\n";
    h_content +=".button-red:active {background-color: #d20e2a;}\n";
    h_content +="p {font-size: 18px;color: #888; margin: 5px;}\n";
    h_content +="</style>\n";
    h_content +="</head>\n";
    return h_content;
}

// -----------------------------------------
// create the main selection web page
// -----------------------------------------
String HTMLmain(){
    String page_content = header_content;
    // start of main selection web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP32 Robot Web Server</h1>\n";
    page_content +="<h3>main selections</h1>\n";
    page_content +="<p><a class=\"button btninline button-off\" href=\"run_about\"><button>Run-about controls</button></a>&nbsp; &nbsp; &nbsp;";
    page_content +="<a class=\"button btninline button-off\" href=\"auto_run\"><button>Auto-run controls</button></a></p>\n";
    page_content +="<p><a class=\"button btninline button-off\" href=\"demo_run\"><button>Demo-run controls</button></a>&nbsp; &nbsp; &nbsp;";
    page_content +="<a class=\"button btninline button-off\" href=\"run_tests\"><button>Component testing</button></a></p>\n";
    page_content +="<p><a class=\"button btninline button-off\" href=\"parameters\"><button>Update parameters</button></a>&nbsp; &nbsp; &nbsp;";
    page_content +="<a class=\"button btninline button-off\" href=\"sysinfo\"><button>System information</button></a></p>\n";

    page_content +="<p>&nbsp;</p>\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;

}


// ---------------------------------------------
// create the system information display web page
// ---------------------------------------------
String HTMLsysinfo(){
    String page_content = header_content;

    // start of main selection web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP32 Robot Web Server</h1>\n";
    page_content +="<h3>System Information - software v";
    page_content +=version;
    page_content +="</h3>\n";

    // **** networking information ****
    page_content +="<div style=\" font-size: 18px; margin-bottom: 5px; margin-top: 10px;\"><b>Networking:</b></div>\n";
    page_content +="<table border=\"1\" style=\"width: 450px; text-align: left;\" align=\"center\" cellpadding=\"3\" cellspacing=\"0\">\n";
    page_content +="<tr><td style=\"width: 225px; \">connected to WiFi SSID:</td><td>" ;
    if (opmode == 5) {
        page_content +=ssid_selected;
    } else {
        page_content +="ESPsoftAP";
    }
    page_content +="</td></tr>\n";
    page_content +="<tr><td>host name:</td><td>" ;
    if (opmode == 5) {
        page_content +=WiFi.getHostname();
    } else {
        page_content +="ESP32";
    }
    page_content +="</td></tr>\n";
    page_content +="<tr><td>assigned IP address:</td><td>" ;
    if (opmode == 5) {
        page_content +=WiFi.localIP().toString();
    } else {
        page_content +=WiFi.softAPIP().toString();
    }
    page_content +="</td></tr>\n";
    page_content +="<tr><td>WiFi MAC address:</td><td>" ;
    if (opmode == 5) {
        page_content +=WiFi.macAddress().c_str();
    } else {
        page_content +=WiFi.softAPmacAddress().c_str();
    }
    page_content +="</td></tr>\n";
    page_content +="</table>\n";

    // **** file system (SPIFFS) ****
    page_content +="<div style=\" font-size: 18px; margin-bottom: 5px; margin-top: 10px;\"><b>File System (SPI Flash File System):</b></div>\n";
    page_content +="<table border=\"1\" style=\"width: 450px; text-align: left;\" align=\"center\" cellpadding=\"3\" cellspacing=\"0\">\n";
    page_content +="<tr><td style=\"width: 225px; \">Total KB:</td><td>" ;
    page_content +=String((float)SPIFFS.totalBytes() / 1024.0);
    page_content +="</td></tr>\n";
    page_content +="<tr><td style=\"width: 225px; \">Used KB:</td><td>" ;
    page_content +=String((float)SPIFFS.usedBytes() / 1024.0);
    page_content +="</td></tr>\n";
    page_content +="</table>\n";

    // **** memory information ****
    page_content +="<div style=\" font-size: 18px; margin-bottom: 5px; margin-top: 10px;\"><b>Memory information: Internal RAM</b></div>\n";
    page_content +="<table border=\"1\" style=\"width: 450px; text-align: left;\" align=\"center\" cellpadding=\"3\" cellspacing=\"0\">\n";

    //page_content +="<tr><td style=\"width: 225px; \">Total heap size:</td><td>" ;
    //page_content +=String(ESP.String(ESP.getHeapSize());
    //page_content +="</td></tr>\n";

    page_content +="<tr><td style=\"width: 225px; \">Available heap:</td><td>" ;
    page_content +=String(ESP.getFreeHeap());
    page_content +="</td></tr>\n";

    page_content +="<tr><td style=\"width: 225px; \">lowest level of free heap since boot:</td><td>" ;
    page_content +=String(ESP.getMinFreeHeap());
    page_content +="</td></tr>\n";

    page_content +="<tr><td style=\"width: 225px; \">largest block of heap that can be allocated at once:</td><td>" ;
    page_content +=String(ESP.getMaxAllocHeap());
    page_content +="</td></tr>\n";

    page_content +="</table>\n";

    page_content +="<div style=\" font-size: 18px; margin-bottom: 5px; margin-top: 10px;\"><b>Memory information: SPI RAM</b></div>\n";
    page_content +="<table border=\"1\" style=\"width: 450px; text-align: left;\" align=\"center\" cellpadding=\"3\" cellspacing=\"0\">\n";

    page_content +="<tr><td style=\"width: 225px; \">Total RAM size:</td><td>" ;
    page_content +=String(ESP.getPsramSize());
    page_content +="</td></tr>\n";

    page_content +="<tr><td style=\"width: 225px; \">Free RAM:</td><td>" ;
    page_content +=String(ESP.getFreePsram());
    page_content +="</td></tr>\n";

    page_content +="<tr><td style=\"width: 225px; \">Minimum free RAM:</td><td>" ;
    page_content +=String(ESP.getMinFreePsram());
    page_content +="</td></tr>\n";

    page_content +="<tr><td style=\"width: 225px; \">Maximum allocatable RAM:</td><td>" ;
    page_content +=String(ESP.getMaxAllocPsram());
    page_content +="</td></tr>\n";

    page_content +="</table>\n";

    // **** chip and firmware information ****
    page_content +="<div style=\" font-size: 18px; margin-bottom: 5px; margin-top: 10px;\"><b>Chip and Firmware information:</b></div>\n";
    page_content +="<table border=\"1\" style=\"width: 450px; text-align: left;\" align=\"center\" cellpadding=\"3\" cellspacing=\"0\">\n";

    page_content +="<tr><td style=\"width: 225px; \">chip revision::</td><td>" ;
    page_content +=String(ESP.getChipRevision());
    page_content +="</td></tr>\n";

    page_content +="<tr><td>Flash chip size:</td><td>" ;
    page_content +=ESP.getFlashChipSize();
    page_content +="</td></tr>\n";

    page_content +="<tr><td>SDK version::</td><td>" ;
    page_content +=String(ESP.getSdkVersion());
    page_content +="</td></tr>\n";

    page_content +="</table>\n";

    page_content +="<p>&nbsp;</p>\n";

    page_content +="<p><a class=\"button button-off\" href=\"/\"><button>back to main selection</button></a></p>\n";
    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;
}

// ---------------------------------------------
// create the parameter type selection web page
// ---------------------------------------------
String HTMLparameter_selection(){
    String page_content = header_content;
    // start of main selection web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP32 Robot Web Server</h1>\n";
    page_content +="<h3>parameter type selection</h3>\n";
    page_content +="<p><a class=\"button button-off\" href=\"WiFi_params\"><button>WiFi parameters</button></a></p>\n";
    page_content +="<p><a class=\"button button-off\" href=\"robot_updates\"><button>Robot parameters</button></a></p>\n";
    page_content +="<p><a class=\"button button-off\" href=\"demo_updates\"><button>Demo action list</button></a></p>\n";
    page_content +="<p><a class=\"button button-off\" href=\"GPIO_updates\"><button>GPIO pin numbers</button></a></p>\n";

    page_content +="<p><a class=\"button button-off\" href=\"/\"><button>back to main selection</button></a></p>\n";


    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;
}

// ---------------------------------------------
// create the demo action list update web page
// ---------------------------------------------
String HTMLdemo_updates(){
    String page_content = header_content;
    // start of main selection web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP32 Robot Web Server</h1>\n";
    page_content +="<h3>Demo action list update</h3>\n";

    // list in a table all the current 20 demo actions that are defined 
    page_content +="<table align=\"center\">\n";
    for (int i=0; i<=19; i++){
        page_content +="<tr><td width=\"100px\">Action [";
        page_content +=i;
        page_content +="]</td><td width=\"100px\">";
        page_content +=demo_controls[i*2];
        page_content +="</td><td width=\"60px\">";
        page_content +=demo_controls[i*2+1];
        page_content +="</td></tr>\n";
    }
    page_content +="</table>\n";
    page_content +="<p>&nbsp;</p>\n";

    // input section of the web page
    page_content +="<form method=\"post\" action=\"/demo_file\"> \n";
    page_content +="Action number: \n";
    page_content +="<input type=\"text\" name=\"demo_num_str\" size=\"6\"> &nbsp; &nbsp;";
    page_content +="<input type=\"text\" name=\"demo_act_str\" size=\"12\"> &nbsp; &nbsp;";
    page_content +="<input type=\"text\" name=\"demo_val_str\" size=\"6\"> \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="<input type=\"submit\" value=\"Submit\">\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</form>\n";

    page_content +="<p><a class=\"button button-off\" href=\"/\"><button>back to main selection</button></a></p>\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;
}


// ---------------------------------------------
// create the WiFi parameter update web page
// ---------------------------------------------
String HTMLWiFi_params(){
    String page_content = header_content;
    // start of main selection web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP32 Robot Web Server</h1>\n";
    page_content +="<h3>WiFi parameter update</h3>\n";
    page_content +="<h3>input/update SSID name and WEP key</h3>\n";

    // input sections of the web page

    page_content +="<form method=\"post\" action=\"/WiFi_updates1\"> \n";
    page_content +="SSID1: \n";
    page_content +="<input type=\"text\" name=\"ssid_1\" size=\"12\" value= ";
    page_content +=ssid1;
    page_content += "> &nbsp; &nbsp; <input type=\"text\" name=\"password_1\"  ";
    page_content += " size=\"12\" placeholder=\"********\" > ";
    page_content +=" &nbsp; &nbsp; <input type=\"submit\" value=\"Submit\">\n";
    page_content +="</form>\n";

    page_content +="<p>&nbsp;</p>\n";

    page_content +="<form method=\"post\" action=\"/WiFi_updates2\"> \n";
    page_content +="SSID2: \n";
    page_content +="<input type=\"text\" name=\"ssid_2\" size=\"12\" value= ";
    page_content +=ssid2;
    page_content += "> &nbsp; &nbsp; <input type=\"text\" name=\"password_2\"  ";
    page_content += " size=\"12\" placeholder=\"********\" > ";
    page_content +=" &nbsp; &nbsp; <input type=\"submit\" value=\"Submit\">\n";
    page_content +="</form>\n";

    page_content +="<p>&nbsp;</p>\n";

    page_content +="<form method=\"post\" action=\"/WiFi_updates3\"> \n";
    page_content +="SSID3: \n";
    page_content +="<input type=\"text\" name=\"ssid_3\" size=\"12\" value= ";
    page_content +=ssid3;
    page_content += "> &nbsp; &nbsp; <input type=\"text\" name=\"password_3\"  ";
    page_content += " size=\"12\" placeholder=\"********\" > ";
    page_content +=" &nbsp; &nbsp; <input type=\"submit\" value=\"Submit\">\n";
    page_content +="</form>\n";

    page_content +="<p>&nbsp;</p>\n";

    page_content +="<form method=\"post\" action=\"/WiFi_updates4\"> \n";
    page_content +="SSID4: \n";
    page_content +="<input type=\"text\" name=\"ssid_4\" size=\"12\" value= ";
    page_content +=ssid4;
    page_content += "> &nbsp; &nbsp; <input type=\"text\" name=\"password_4\"  ";
    page_content += " size=\"12\" placeholder=\"********\" > ";
    page_content +=" &nbsp; &nbsp; <input type=\"submit\" value=\"Submit\">\n";
    page_content +="</form>\n";

    page_content +="<p>&nbsp;</p>\n";

    page_content +="<form method=\"post\" action=\"/WiFi_updates5\"> \n";
    page_content +="SSID5: \n";
    page_content +="<input type=\"text\" name=\"ssid_5\" size=\"12\" value= ";
    page_content +=ssid5;
    page_content += "> &nbsp; &nbsp; <input type=\"text\" name=\"password_5\"  ";
    page_content += " size=\"12\" placeholder=\"********\" > ";
    page_content +=" &nbsp; &nbsp; <input type=\"submit\" value=\"Submit\">\n";
    page_content +="</form>\n";

    page_content +="<p>&nbsp;</p>\n";

    page_content +="<p><a class=\"button button-off\" href=\"/\"><button>back to main selection</button></a></p>\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;
}


// ---------------------------------------------
// create the robot demo-run control web page
// ---------------------------------------------
String HTMLrobot_demo_run(){
    String page_content = header_content;
    // start of robot demo control web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP32 Robot Web Server</h1>\n";
    page_content +="<h3>demo run controls</h3>\n";
    if (web_server == "off") {
        page_content +="<h3>Robot in STOP switch mode</h3>\n";
        page_content +="<h3>Set robot slide switch to GO to show demo-run web controls</h3>\n";
    } else {
        if(robot_state == 1) {
            page_content +="<p><a class=\"button button-on\" href=\"run_demo\"><button>DEMO RUN</button></a></p>\n";
        } else {
            page_content +="<p><a class=\"button button-off\" href=\"run_demo\"><button>DEMO RUN</button></a></p>\n";
        }

        if(robot_state == 0) {
            page_content +="<p><a class=\"button button-on\" href=\"stop_demo\"><button>DEMO STOP</button></a></p>\n";
        } else {
            page_content +="<p><a class=\"button button-off\" href=\"stop_demo\"><button>DEMO STOP</button></a></p>\n";
        }
    }

    page_content +="<p>&nbsp;</p>\n";
    page_content +="<p><a class=\"button button-off\" href=\"/\"><button>back to main selection</button></a></p>\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;

}


// ---------------------------------------------
// create the robot auto-run control web page
// ---------------------------------------------
String HTMLrobot_auto_run(){
    String page_content = header_content;
    // start of robot auto-run control web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP32 Robot Web Server</h1>\n";
    page_content +="<h3>auto-run controls</h3>\n";
    // input sections of the web page
    // input the robot speed to be used as a % from 0-100
    page_content +="<form method=\"post\" action=\"/autospeed\"> \n";
    page_content +="Robot speed: \n";
    page_content +="<input type=\"text\" name=\"r_speed_str\" size=\"3\" value= ";
    page_content +=str_r_speed;
    page_content += " >\n";
    page_content +="<input type=\"submit\" value=\"Submit\">\n";
    page_content +="</form>\n";

    if (web_server == "off") {
        page_content +="<h3>Robot in STOP switch mode</h3>\n";
        page_content +="<h3>Set robot slide switch to GO to show auto-run web controls</h3>\n";
    } else {
        if(robot_state == 1) {
            page_content +="<p><a class=\"button button-on\" href=\"run_auto\"><button>AUTO RUN</button></a></p>\n";
        } else {
            page_content +="<p><a class=\"button button-off\" href=\"run_auto\"><button>AUTO RUN</button></a></p>\n";
        }

        if(robot_state == 0) {
            page_content +="<p><a class=\"button button-on\" href=\"run_stop\"><button>STOP</button></a></p>\n";
        } else {
            page_content +="<p><a class=\"button button-off\" href=\"run_stop\"><button>STOP</button></a></p>\n";
        }
    }

    page_content +="<p>&nbsp;</p>\n";
    page_content +="<p><a class=\"button button-off\" href=\"/\"><button>back to main selection</button></a></p>\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;

}


// ---------------------------------------------
// create the robot run-about control web page
// ---------------------------------------------
String HTMLrobot_run_about(){
    String page_content = header_content;
    // start of robot run-about control web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP32 Robot Web Server</h1>\n";
    page_content +="<h3>run about controls - autostop: ";
    page_content +=stopauto;
    page_content +="</h3>\n";
    // input sections of the web page
    // input the robot speed to be used as a % from 0-100
    page_content +="<form method=\"post\" action=\"/postspeed\"> \n";
    page_content +="Robot speed: \n";
    page_content +="<input type=\"text\" name=\"r_speed_str\" size=\"3\" value= ";
    page_content +=str_r_speed;
    page_content += " >\n";
    page_content +="<input type=\"submit\" value=\"Submit\">\n";
    page_content +="</form>\n";

    if (web_server == "off") {
        page_content +="<h3>Robot in STOP switch mode</h3>\n";
        page_content +="<h3>Set robot slide switch to GO to show run-about web controls</h3>\n";
    } else {
        if(robot_state == 1) {
            page_content +="<p><a class=\"button button-on\" href=\"forward\"><button>FWD</button></a></p>\n";
        } else {
            page_content +="<p><a class=\"button button-off\" href=\"forward\"><button>FWD</button></a></p>\n";
        }
        page_content +="<p><a class=\"button btninline button-off\" href=\"turnleft\"><button>TURN<br/>LEFT</button></a>&nbsp; &nbsp; &nbsp; ";
        page_content +="<a class=\"button btninline button-off\" href=\"turnright\"><button>TURN<br/>RIGHT</button></a></p>\n";
        page_content +="<p><a class=\"button button-on\" href=\"halt\"><button>STOP</button></a></p>\n";
        page_content +="<p><a class=\"button btninline button-off\" href=\"spinleft\"><button>SPIN<br/>LEFT</button></a>&nbsp; &nbsp; &nbsp; ";
        page_content +="<a class=\"button btninline button-off\" href=\"spinright\"><button>SPIN<br/>RIGHT</button></a></p>\n";
        if(robot_state == 2) {
            page_content +="<p><a class=\"button button-on\" href=\"backward\"><button>BACK</button></a></p>\n";
        } else {
            page_content +="<p><a class=\"button button-off\" href=\"backward\"><button>BACK</button></a></p>\n";
        }
    }

    page_content +="<p>&nbsp;</p>\n";
    page_content +="<p><a class=\"button button-off\" href=\"/\"><button>back to main selection</button></a></p>\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;

}


// ---------------------------------------------
// create the component testing web page
// ---------------------------------------------
String HTMLrun_tests(){
    String page_content = header_content;
    // start of component testing web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP32 Robot Web Server</h1>\n";
    page_content +="<h3>component testing</h3>\n";
    // input sections of the web page
    // input the motor speed to be used as a % from 0-100
    page_content +="<form method=\"post\" action=\"/testspeed\"> \n";
    page_content +="Motor test speed: \n";
    page_content +="<input type=\"text\" name=\"r_speed_str\" size=\"3\" value= ";
    page_content +=str_r_speed;
    page_content += " >\n";
    page_content +="<input type=\"submit\" value=\"Submit\">\n";
    page_content +="</form>\n";
    // 3 lines below have the id for the AJAX update carried out by the javascript inthe header section
    page_content +="<p>Bump detection status: <span id=/"bumpid/">";
    page_content +=bumptext;
    page_content +="</span></p>\n";
    if (web_server == "off") {
        page_content +="<h3>Robot in STOP switch mode</h3>\n";
        page_content +="<h3>Set robot slide switch to GO to show component testing controls</h3>\n";
    } else {
        page_content +="<p><a class=\"button btninline button-off\" href=\"leftfwd\"><button>LEFT<br/>FWD</button></a>&nbsp; &nbsp; &nbsp; ";
        page_content +="<a class=\"button btninline button-off\" href=\"leftback\"><button>LEFT<br/>BACK</button></a></p>\n";
        page_content +="<p><a class=\"button btninline button-off\" href=\"rightfwd\"><button>RIGHT<br/>FWD</button></a>&nbsp; &nbsp; &nbsp; ";
        page_content +="<a class=\"button btninline button-off\" href=\"rightback\"><button>RIGHT<br/>BACK</button></a></p>\n";
        page_content +="<p><a class=\"button button-on\" href=\"teststop\"><button>RESET</button></a></p>\n";
        page_content +="<p><a class=\"button btninline button-off\" href=\"servo_left\"><button>Servo<br/>Left</button></a>&nbsp; ";
        page_content +="<a class=\"button btninline button-on\" href=\"USread\"><button>U-SONIC<br/>SENSOR<br/>";
        page_content +=cm_str;
        page_content +=" cm</button></a>&nbsp; ";
        page_content +="<a class=\"button btninline button-off\" href=\"servo_right\"><button>Servo<br/>Right</button></a></p>\n";
        page_content +="<p><a class=\"button btninline button-off\" href=\"RGB_red\"><button>RGB<br/>RED</button></a>&nbsp; ";
        page_content +="<a class=\"button btninline button-off\" href=\"RGB_green\"><button>RGB<br/>GREEN</button></a>&nbsp; ";
        page_content +="<a class=\"button btninline button-off\" href=\"RGB_blue\"><button>RGB<br/>BLUE</button></a></p>\n";
        page_content +="<p><a class=\"button btninline button-off\" href=\"buzzbeep\"><button>beep<br/>buzzer</button></a>&nbsp; &nbsp; &nbsp; ";
        page_content +="<a class=\"button btninline button-off\" href=\"twinkle\"><button>play<br/>twinkle</button></a></p>\n";
        if (str_LCD16x2 == "yes") {page_content +="<p><a class=\"button btninline button-off\" href=\"testLCD\"><button>test<br/>LCD</button></a>";}
        if (str_LCD16x2 == "yes" and str_OLED64x128 == "yes") {page_content +="&nbsp; &nbsp; &nbsp;";}
        if (str_LCD16x2 == "no" and str_OLED64x128 == "yes") {page_content +="<p>";}
        if (str_OLED64x128 == "yes") {page_content +="<a class=\"button btninline button-off\" href=\"testOLED\"><button>test<br/>OLED</button></a>";}
        page_content +="</p>\n";

    }

    page_content +="<p>&nbsp;</p>\n";
    page_content +="<p><a class=\"button button-off\" href=\"/\"><button>back to main selection</button></a></p>\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;

}


// ---------------------------------------------
// create the GPIO pin parameter update web page
// ---------------------------------------------
String HTMLGPIO_updates(){
    String page_content = header_content;
    // start of main selection web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP32 Robot Web Server</h1>\n";
    page_content +="<h3>GPIO pin usage update</h3>\n";
    page_content +="<h3>** these pin# settings should rarely be changed **</h3>\n";
    page_content +="<h3>input/update GPIO pin number</h3>\n";

    // input sections of the web page
    page_content +="<form method=\"post\" action=\"/GPIO_pins\"> \n";

    page_content +="slide switch AB: \n";
    page_content +="<input type=\"text\" name=\"s_AB_str\" size=\"6\" value= ";
    page_content +=str_s_AB;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="slide switch CD: \n";
    page_content +="<input type=\"text\" name=\"s_CD_str\" size=\"6\" value= ";
    page_content +=str_s_CD;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="slide switch GO-STOP: \n";
    page_content +="<input type=\"text\" name=\"onoff_str\" size=\"6\" value= ";
    page_content +=str_onoff;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="ultrasonic trig: \n";
    page_content +="<input type=\"text\" name=\"trigPin_str\" size=\"6\" value= ";
    page_content +=str_trigPin;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="ultrasonic echo: \n";
    page_content +="<input type=\"text\" name=\"echoPin_str\" size=\"12\" value= ";
    page_content +=str_echoPin;
    page_content +=" > \n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="<input type=\"submit\" value=\"Submit\">\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</form>\n";

    page_content +="<p><a class=\"button button-off\" href=\"/\"><button>back to main selection</button></a></p>\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;
}


// -----------------------------------------------------
// create the robot operation parameter update web page
// -----------------------------------------------------
String HTMLrobot_updates(){
    String page_content = header_content;
    // start of main selection web page content
    page_content +="<body>\n";
    page_content +="<h1>";
    if(opmode == 5) {
        page_content += namehost;
    } else {
        page_content += "10.0.5.";
        page_content += IPnum;
    }
    page_content +="</h1>\n";
    page_content +="<h1>ESP32 Robot Web Server</h1>\n";
    page_content +="<h3>robot operation parameters</h3>\n";
    page_content +="<h4>input/update the individual operational parameters</h4>\n";

    // input sections of the web page
    page_content +="<form method=\"post\" action=\"/robot_params\"> \n";

    page_content +="<table border=\"0\" style=\"width: 450px; text-align: left;\" align=\"center\" cellpadding=\"3\" cellspacing=\"0\">\n";

    page_content +="<tr><td style=\"width: 375px; text-align: right;\">take avoidance action distance (cm):</td><td>" ;
    page_content +="<input type=\"text\" name=\"avoid_distance_str\" size=\"8\" value= ";
    page_content +=str_avoid_distance;
    page_content +=" ></td></tr>\n";

    page_content +="<tr><td style=\"width: 375px; text-align: right;\">avoidance sensor repeats:</td><td>" ;
    page_content +="<input type=\"text\" name=\"avoid_limit_str\" size=\"8\" value= ";
    page_content +=str_avoid_limit;
    page_content +=" ></td></tr>\n";

    page_content +="<tr><td style=\"width: 375px; text-align: right;\">avoidance action spin time (ms):</td><td>" ;
    page_content +="<input type=\"text\" name=\"avoidspintime_str\" size=\"8\" value= ";
    page_content +=str_avoidspintime;
    page_content +=" ></td></tr>\n";

    page_content +="<tr><td style=\"width: 375px; text-align: right;\">avoidance action reverse time (ms):</td><td>" ;
    page_content +="<input type=\"text\" name=\"ReverseTime_str\" size=\"8\" value= ";
    page_content +=str_ReverseTime;
    page_content +=" ></td></tr>\n";

    page_content +="<tr><td style=\"width: 375px; text-align: right;\">activate autostop (yes/no):</td><td>" ;
    page_content +="<input type=\"text\" name=\"stopauto_str\" size=\"8\" value= ";
    page_content +=stopauto;
    page_content +=" ></td></tr>\n";

    page_content +="<tr><td style=\"width: 375px; text-align: right;\">autostop distance (cm):</td><td>" ;
    page_content +="<input type=\"text\" name=\"stop_distance_str\" size=\"8\" value= ";
    page_content +=str_stop_distance;
    page_content +=" ></td></tr>\n";

    page_content +="<tr><td style=\"width: 375px; text-align: right;\">robot turn time (ms):</td><td>" ;
    page_content +="<input type=\"text\" name=\"turntime_str\" size=\"8\" value= ";
    page_content +=str_turntime;
    page_content +=" ></td></tr>\n";

    page_content +="<tr><td style=\"width: 375px; text-align: right;\">robot turn speed (%):</td><td>" ;
    page_content +="<input type=\"text\" name=\"turnspeed_str\" size=\"8\" value= ";
    page_content +=str_turnspeed;
    page_content +=" ></td></tr>\n";

    page_content +="<tr><td style=\"width: 375px; text-align: right;\">robot spin time (ms):</td><td>" ;
    page_content +="<input type=\"text\" name=\"spintime_str\" size=\"8\" value= ";
    page_content +=str_spintime;
    page_content +=" ></td></tr>\n";

    page_content +="<tr><td style=\"width: 375px; text-align: right;\">robot spin speed (%):</td><td>" ;
    page_content +="<input type=\"text\" name=\"spinspeed_str\" size=\"8\" value= ";
    page_content +=str_spinspeed;
    page_content +=" ></td></tr>\n";

    page_content +="<tr><td style=\"width: 375px; text-align: right;\">default robot speed (%):</td><td>" ;
    page_content +="<input type=\"text\" name=\"r_speed_str\" size=\"8\" value= ";
    page_content +=str_r_speed;
    page_content +=" ></td></tr>\n";

    page_content +="<tr><td style=\"width: 375px; text-align: right;\">left motor balance ratio:</td><td>" ;
    page_content +="<input type=\"text\" name=\"motorL_str\" size=\"8\" value= ";
    page_content +=str_motorL;
    page_content +=" ></td></tr>\n";

    page_content +="<tr><td style=\"width: 375px; text-align: right;\">right motor balance ratio:</td><td>" ;
    page_content +="<input type=\"text\" name=\"motorR_str\" size=\"8\" value= ";
    page_content +=str_motorR;
    page_content +=" ></td></tr>\n";

    page_content +="<tr><td style=\"width: 375px; text-align: right;\">16x2 LCD installed (yes/no)</td><td>" ;
    page_content +="<input type=\"text\" name=\"LCD16x2_str\" size=\"8\" value= ";
    page_content +=str_LCD16x2;
    page_content +=" ></td></tr>\n";

    page_content +="<tr><td style=\"width: 375px; text-align: right;\">64x128 OLED installed (yes/no)</td><td>" ;
    page_content +="<input type=\"text\" name=\"OLED64x128_str\" size=\"8\" value= ";
    page_content +=str_OLED64x128;
    page_content +=" ></td></tr>\n";

    page_content +="<tr><td style=\"width: 375px; text-align: right;\">bump switch sensor installed (yes/no)</td><td>" ;
    page_content +="<input type=\"text\" name=\"bump_str\" size=\"8\" value= ";
    page_content +=str_bump;
    page_content +=" ></td></tr>\n";

    page_content +="<tr><td style=\"width: 375px; text-align: right;\">ultrasonic sensor servo installed (yes/no)</td><td>" ;
    page_content +="<input type=\"text\" name=\"USservo_str\" size=\"8\" value= ";
    page_content +=str_USservo;
    page_content +=" ></td></tr>\n";

    page_content +="</table>\n";
    page_content +="<p>&nbsp;</p>\n";
    page_content +="<input type=\"submit\" value=\"Submit\">\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</form>\n";

    page_content +="<p><a class=\"button button-off\" href=\"/\"><button>back to main selection</button></a></p>\n";
    page_content +="<p>&nbsp;</p>\n";

    page_content +="</body>\n";
    page_content +="</html>\n";
    return page_content;
}

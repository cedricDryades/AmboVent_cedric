/*
THIS CODE WAS WRITTEN FOR TESTING AND RUNNING A HOME-MADE VENTILATION DEVICE.
IT IS NOT TESTED FOR SAFETY AND NOT APPROVED FOR USE IN ANY CLINICAL, MEDICAL OR COMERCIAL DEVICE.
IT IS NOT APPROVED BY ANY REGULATORY AUTHORITY.
USE ONLY AT YOUR OWN RISK.

To start calibrations:
First enter the maintenance setup menu by pressing the TEST button for 3 seconds.
Using the RATE potentiometer select the calibration required and press TEST to select.
Follow instructions on the screen.

For the Arm range calibration:
Use the Rate potentiometer to move the arm up/down.
*/

#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <SparkFun_MS5803_I2C.h>
#include <Wire.h>
#include <ams_as5048b.h>

// system configuration
#define full_configuration \
    1  // 1 is the default - full system.   0 is for partial system - potentiometer installed on
       // pulley, no potentiometers, ...
#define pressure_sensor_available 1  // 1 - you have installed an I2C pressure sensor
#define central_monitor_system 0     // 1 - send unique ID for 10 seconds upon startup, 0 - dont

#if central_monitor_system
#include "ArduinoUniqueID.h"
#endif

/**** options for serial com ***/
// both options should not be on at the same time!
#define send_to_monitor 0
    // 0 -> No serial output for monitor
    // 1 -> Send data to monitor, see send_data_to_monitor() for more info
#define data_frequency 1000 // data refresh rate
				
#define telemetry 0
    // 0 -> Does not send any telemetry over serial 
    // 1 -> Send telemetry... see print_tele() for optional telemetry data to send
#define tele_frequency 1000 // Telemetry refresh rate

/*** Debugging modes (for programmers only!) ***/
// Sends relevant debugging information over serial. You should turn telemetry && send_to_monitor off
// Debug adds delay to the execution, they should not be activated in "production"!
#define debug 1
#define debug_screen 1
#define debug_IO 1

/*** USER INTERFACE ***/
#define display_refresh_rate 3000 // min refresh rate for the screen
		
#define deltaUD \
    5  // define the value chnage per each button press for the non-potentiometer version only
#define pot_alpha 0.85  // filter the pot values

// clinical
#define perc_of_lower_volume 50.0  // % of max press - defines lower volume
#define perc_of_lower_vol_display \
    33.0  // % of max press - defines lower volume to display when reaching the real lower volume
#define wait_time_after_resistance \
    3  // seconds to wait before re-attempt to push air after max pressure was achieved
#define max_pres_disconnected \
    10  // if the max pressure during breathing cycle does not reach this value - pipe is
        // disconnected
#define insp_pressure_default \
    40  // defualt value - hold this pressure while breathing - the value is changed if
        // INSP_Pressure potentiometer is inatalled
#define safety_pres_above_insp 10  // defines safety pressure as the inspirium pressure + this one
#define safety_pressure 70         // quickly pullback arm when reaching this pressure in cm H2O
#define speed_multiplier_reverse \
    2  // factor of speeed for releasing the pressure (runs motion in reverse at X this speed
#define motion_time_default 35  // motion time in 100 mSec 35 = 3500 mSec
#define patient_triggered_breath_def \
    1  // 1 = trigger new breath in case of patient inhale during the PEEP plateu
#define delta_pres_patient_inhale 5  // in cmH2O
#define alpha_pres 0.98              // used to average the pressure during the PEEP plateu

#if (full_configuration == 0)  // no pot for UI, feedback pot on pulley
#    define LCD_available 0
#    define pres_pot_available \
        0              // 1 if the system has 3 potentiometer and can control the inspirium pressure
#    define SW2_pin 7  // breath - On / Off / cal
#    define TST_pin 2  // test mode - not in use
#    define pin_LED_AMP 11           // amplitude LED
#    define pin_LED_FREQ 9           // frequency LED
#    define pin_LED_Fail 10          // FAIL and calib blue LED
#    define pin_USR 12               // User LED
#    define FD_pin 4                 // freq Down
#    define FU_pin 5                 // freq Up
#    define AD_pin 8                 // Amp Down
#    define AU_pin 6                 // Amp Up
#    define curr_sense 1             // 1- there is a curent sensor
#    define control_with_pot 0       // 1 = control with potentiometers  0 = with push buttons
#    define FF 0.6                   // motion control feed forward
#    define KP 0.2                   // motion control propportional gain
#    define KI 2                     // motion control integral gain
#    define integral_limit 6         // limits the integral of error
#    define f_reduction_up_val 0.65  // reduce feedforward by this factor when moving up
#endif

#if (full_configuration == 1)  // feedback pot on arm, potentiometers for UI
#    define LCD_available 1
#    define pres_pot_available \
        1              // 1 if the system has 3 potentiometer and can control the inspirium pressure
#    define SW2_pin 4  // breath - On / Off / cal
#    define TST_pin 2  // test mode - not in use
#    define RST_pin 5  // reset alarm - not in use
#    define pin_LED_AMP 13           // amplitude LED
#    define pin_LED_FREQ 13          // frequency LED
#    define pin_LED_Fail 10          // FAIL and calib blue LED
#    define pin_USR 9                // User LED
#    define FD_pin 13                // freq Down - not used when you have potentiometers
#    define FU_pin 13                // freq Up - not used when you have potentiometers
#    define AD_pin 13                // Amp Down - not used when you have potentiometers
#    define AU_pin 13                // Amp Up - not used when you have potentiometers
#    define curr_sense 0             // o no current sensor
#    define control_with_pot 1       // 1 = control with potentiometers  0 = with push buttons
#    define FF 4.5                   // motion control feed forward
#    define KP 1.2                   // motion control propportional gain
#    define KI 7                     // motion control integral gain
#    define integral_limit 5         // limits the integral of error
#    define f_reduction_up_val 0.85  // reduce feedforward by this factor when moving up
#endif

// other Arduino pins alocation
#define pin_PWM 3  // digital pin that sends the PWM to the motor
#define pin_POT 0  // analog pin of motion feedback potentiometer
#define pin_CUR 1  // analog pin of current sense
#define pin_AMP 2  // analog pin of amplitude potentiometer control
#define pin_FRQ 3  // analog pin of rate potentiometer control
#define pin_PRE 6  // analog pin of pressure potentiometer control

// Talon SR or SPARK controller PWM settings ("angle" for Servo library)
#define PWM_mid 93  // was 93 -   mid value for PWM 0 motion - higher pushes up
#define PWM_max 85
#define PWM_min -85
#define max_allowed_current 100  // 100=10 Amps

// motion control parameters
#define cycleTime 10        // milisec
#define alpha 0.95          // filter for current apatation - higher = stronger low pass filter
#define profile_length 250  // motion control profile length
#define motion_control_allowed_error 30  // % of range

// motor and sensor definitions
#define invert_mot 1
#define invert_pot 0
#define magnetic_encoder 1

// I2C sensors adresses
#define I2C_adr_lcd 0x27
#define I2C_adr_press 0x76
#define I2C_adr_arm 0x40
			
Servo motor;
// LCDi2cNHD lcd(2, 16, 0x28, 0);  // Set the LCD address to 0x28 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(I2C_adr_lcd, 16,2);  // Set the LCD address to 0x27 for a 16 chars and 2 line display
#if (pressure_sensor_available == 1)
MS5803 sparkfunPress(I2C_adr_press);
#endif

#if (magnetic_encoder)
AMS_AS5048B armSensor;
#endif

// Motion profile parameters
// pos byte 0...255  units: promiles of full range
// vel int 0...255  ZERO is at 128 , units: pos change per 0.2 sec
// profile data:  press 125 points (50%) relase 125

const PROGMEM byte pos[profile_length] = {
    0,   0,   1,   2,   4,   6,   8,   10,  13,  15,  18,  21,  25,  28,  31,  35,  38,  42,
    46,  50,  54,  57,  61,  66,  70,  74,  78,  82,  86,  91,  95,  99,  104, 108, 112, 117,
    121, 125, 130, 134, 138, 143, 147, 151, 156, 160, 164, 169, 173, 177, 181, 185, 189, 194,
    198, 201, 205, 209, 213, 217, 220, 224, 227, 230, 234, 237, 240, 242, 245, 247, 249, 251,
    253, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 254, 253, 252, 250, 248, 246, 244, 241, 238, 235, 232, 229, 225, 222, 218, 214, 210,
    206, 202, 198, 193, 189, 184, 180, 175, 171, 166, 162, 157, 152, 148, 143, 138, 134, 129,
    124, 120, 115, 111, 106, 102, 97,  93,  89,  84,  80,  76,  72,  68,  64,  61,  57,  54,
    50,  47,  44,  41,  38,  36,  33,  31,  29,  27,  25,  23,  22,  20,  19,  17,  16,  15,
    13,  12,  11,  10,  9,   8,   7,   6,   6,   5,   4,   3,   3,   2,   2,   1,   1,   1,
    0,   0,   0,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,   2,   2,
    1,   1,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0};
const PROGMEM byte vel[profile_length] = {
    129, 132, 134, 136, 137, 139, 140, 141, 142, 143, 143, 144, 144, 145, 146, 146, 146, 147,
    147, 147, 148, 148, 148, 148, 149, 149, 149, 149, 149, 149, 150, 150, 150, 150, 150, 150,
    150, 150, 150, 150, 150, 150, 150, 150, 150, 149, 149, 149, 149, 149, 149, 148, 148, 148,
    148, 147, 147, 147, 146, 146, 146, 145, 144, 144, 143, 143, 142, 141, 140, 139, 137, 136,
    134, 132, 129, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128,
    128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128,
    128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 127,
    125, 123, 121, 120, 119, 117, 116, 115, 114, 113, 112, 111, 111, 110, 109, 109, 108, 108,
    107, 107, 106, 106, 106, 106, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105,
    105, 105, 105, 105, 106, 106, 106, 107, 107, 107, 108, 108, 109, 109, 110, 110, 111, 111,
    112, 113, 113, 114, 115, 116, 117, 118, 118, 119, 119, 120, 120, 120, 121, 121, 121, 122,
    122, 122, 123, 123, 123, 124, 124, 124, 124, 125, 125, 125, 125, 125, 126, 126, 126, 126,
    126, 127, 127, 127, 127, 127, 127, 127, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128,
    128, 128, 129, 129, 129, 129, 129, 129, 129, 129, 129, 128, 128, 128, 128, 128};

byte FD_cur, FU_cur, AD_cur, AU_cur, FD_prev, FU_prev, AD_prev, AU_prev, SW2_cur, SW2_prev, SW2_pressed, SW2_debounced, TST_cur, TST_prev, TST_pressed, TST_debounced, RST_cur, RST_prev, RST_pressed = 0, RST_debounced, LED_status,
    USR_status, blueOn, calibrated = 0, calibON, numBlinkFreq, menu_pos;
byte monitor_index = 0, BPM = 14, prev_BPM, in_wait, failure, send_beep, wanted_cycle_time,
     disconnected = 0, high_pressure_detected = 0, motion_failure = 0, hold_breath,
     safety_pressure_detected;
byte counter_ON, counter_OFF, insp_pressure, prev_insp_pressure, safety_pressure_counter,
    no_fail_counter, TST, counter_TST_OFF, counter_TST_ON, TSTtemp;
byte patient_triggered_breath, motion_time, progress;
int A_sensed_pos, prev_A_sensed_pos, A_current, Compression_perc = 80, prev_Compression_perc, A_rate, A_comp,
                                  A_pres;
int motorPWM, index = 0, prev_index, i, wait_cycles, cycle_number, cycles_lost, index_last_motion;
int pressure_abs, breath_cycle_time, max_pressure = 0, prev_max_pressure = 0, min_pressure = 100,
                                     prev_min_pressure = 0, index_to_hold_breath, pressure_baseline;
int comp_pot_low = 0, comp_pot_high = 1023, rate_pot_low = 0, rate_pot_high = 1023,
    pres_pot_low = 0, pres_pot_high = 1023;
unsigned int max_arm_pos, min_arm_pos;
unsigned long lastSent, lastIndex, lastUSRblink, lastBlue, start_wait,
    last_sent_data, last_sent_tele, last_read_pres, start_disp_pres, SW2_timer, TST_timer, RST_timer;
float pot_rate, pot_pres, pot_comp, avg_pres;
float wanted_pos, wanted_vel_PWM, range, range_factor, profile_planned_vel, planned_vel, integral,
    error, prev_error, f_reduction_up;

String display_line1, display_line2, display_prev_line1, display_prev_line2;
int	display_updates, display_last;

enum screens : int
{
	SCR_STBY,
	SCR_BREATH,
	SCR_CFG,
	SCR_CFG_PRES_SAVED,
	SCR_CFG_POTL,
	SCR_CFG_POTR,
	SCR_CFG_POTS,
	SCR_CFG_ARM_UP,
	SCR_CFG_ARM_DOWN,
	SCR_CFG_ARM_SAFE,
	SCR_CFG_ARM_SAVE,
	SCR_CFG_MOTION,
	SCR_CFG_MOTION_SAVE
};
enum screens screen;

enum main_states : byte
{
    STBY_STATE,
    BREATH_STATE,
    CFG_STATE
};
enum main_states state;

void setup()
{
    pinMode(pin_PWM, OUTPUT);
    pinMode(FD_pin, INPUT_PULLUP);
    pinMode(FU_pin, INPUT_PULLUP);
    pinMode(AD_pin, INPUT_PULLUP);
    pinMode(AU_pin, INPUT_PULLUP);
    pinMode(SW2_pin, INPUT_PULLUP);
    pinMode(TST_pin, INPUT_PULLUP);
    pinMode(RST_pin, INPUT_PULLUP);
    pinMode(pin_LED_AMP, OUTPUT);
    pinMode(pin_LED_FREQ, OUTPUT);
    pinMode(pin_LED_Fail, OUTPUT);
    pinMode(pin_USR, OUTPUT);
    motor.attach(pin_PWM);
    Serial.begin(115200);
    Wire.begin();
	Wire.setClock(100000UL);

#if (pressure_sensor_available == 1)
    {
		// Checking if sensor present
		// BAZINGA add non resetable error if missing
		Wire.beginTransmission(I2C_adr_press);
		byte i2c_error = Wire.endTransmission();
		if(i2c_error){ Serial.println("ERROR: Pressure sensor not present @ address " + String(I2C_adr_press, HEX)); }

        sparkfunPress.reset();
        sparkfunPress.begin();
        pressure_baseline = int(sparkfunPress.getPressure(ADC_4096));
    }
#endif

    if (LCD_available)
    {
		// Checking if LCD present
		// BAZINGA add non resetable error if missing
		Wire.beginTransmission(I2C_adr_lcd);
		byte i2c_error = Wire.endTransmission();
		if(i2c_error){ Serial.println("ERROR: LCD not present @ address " + String(I2C_adr_lcd, HEX)); }
		
        // lcd.init();      // initialize the LCD
		lcd.init();      // initialize the LCD
		lcd.backlight();  // Turn on the blacklight and print a message.
        lcd.setCursor(0, 0);
        lcd.print("AmboVent");
        lcd.setCursor(0, 1);
        lcd.print("MAKE HAVEN");
        delay(1000);
    }
	
	Serial.println("Ambovent - Make Haven");

#if central_monitor_system
    for (i = 0; i < 100; i++)
    {
        UniqueIDdump(Serial);
    }  // for IAI monitor run for 100 cycles
#endif

#if (magnetic_encoder)
    {
		// Checking if magnetic encoder present
		// BAZINGA add non resetable error if missing
		Wire.beginTransmission(I2C_adr_arm);
		byte i2c_error = Wire.endTransmission();
		if(i2c_error){ Serial.println("ERROR: arm encoder not present @ address " + String(I2C_adr_arm, HEX)); }

        armSensor.begin();
    }
#endif

    state = STBY_STATE; // BAZINGA state not implemented - might not be necessary
	screen = SCR_STBY;
    EEPROM.get(4, min_arm_pos);
    delay(20);
    EEPROM.get(8, max_arm_pos);
    delay(20);
    EEPROM.get(12, comp_pot_low);
    delay(20);
    EEPROM.get(16, comp_pot_high);
    delay(20);
    EEPROM.get(20, rate_pot_low);
    delay(20);
    EEPROM.get(24, rate_pot_high);
    delay(20);
    EEPROM.get(28, pres_pot_low);
    delay(20);
    EEPROM.get(32, pres_pot_high);
    delay(20);
    if (min_arm_pos >= 0 && min_arm_pos < 1024 && max_arm_pos >= 0 && max_arm_pos < 1024)
        calibrated = 1;
    insp_pressure = insp_pressure_default;
    patient_triggered_breath = patient_triggered_breath_def;
    motion_time = motion_time_default;
}

void loop()
{	
	// Reads all the sensor values and stores them
    read_IO();

	// Background operations - () BAZINGA to be done by transfering the bottom part of read_IO
	// Checks if failures are still active
	operations();
	
	// UI / actions (depending on the "screen" the machine operates whatever it needs to)
	user_interface();

	// Telemetry (see function)
    print_tele();
	
	// Monitor data display (see function)
    send_data_to_monitor();
	
	// update the display (once per loop max)
	display_refresh();
}

// User interface 
// Action can define the state of the machine as well as the diferrent menus/interactions
void user_interface()
{
	/* Actions based on the state of the machine
    switch (state)
    {
    case STBY_STATE:  // standby
        standby_func();
        if (SW2_pressed)  // start breathing motion
        {
			if(!calibrated){
				display_to_LCD("ERROR: Machine", "not calibrated"); // BAZINGA
				delay(500);
			}else{
	            state = BREATH_STATE;
	            initialize_breath();
			}			
        }
		// BAZINGA retire car pas sur du truc
        // if (TST_cur == 0)
        //     last_TST_not_pressed = millis();
        // if (millis() - last_TST_not_pressed > 3000)
        // {
        //     LED_USR(1);
        //     while (TST == 1 || TST_pressed)
        //     {
        //         read_IO();
        //     }  // wait for button release
        //     progress = 0;
        //     state = CFG_STATE;
        // }

        break;

    case BREATH_STATE:  // run profile
        run_profile_func();
		display_breathing();
        if (SW2_pressed)
            state = STBY_STATE;  // stop breathing motion
        break;

    case CFG_STATE:  // maintanance menu
        display_menu();
        break;
    }
	*/

	// Screen to be displayed + actions per screen
	switch (screen)
	{
		// Startup "screen" - SCR_STBY
		case SCR_STBY:
			calibrated = 1; //BAZINGA
			display_breathing();

			if(RST_pressed == 1){            
				RST_pressed = 0;
				progress = 0;
				screen = SCR_CFG;
			}
			if(SW2_pressed == 1){            
				SW2_pressed = 0;
				LED_USR(1);
				initialize_breath();
				screen = SCR_BREATH;
			}
			if(TST_pressed == 1){            
				TST_pressed = 0;
				calibrated = 1;
				reset_failures();
			}
			break;

		case SCR_BREATH:
			if(SW2_pressed == 1){            
				SW2_pressed = 0;
				LED_USR(0);
				motion_stop();
				screen = SCR_STBY;
			}
			calibrated = 1; //BAZINGA
			run_profile_func();
			display_breathing();
			// reset_failures();
			break;
		
			// Configuration menu + submenus
		case SCR_CFG:
		
			LED_USR(1); // BAZINGA - better way to do this?
			calibON = 1; // BAZINGA - check if still usefull after all edits

			// Using pot to navigate in the menu
		    menu_pos = map(pot_rate, 0, 1023, 0, 8);
		    menu_pos = constrain(menu_pos, 0, 8);

		    switch (menu_pos)
		    {					
			    case 1:  // calib pot
			        display_to_LCD("Calibrate Pots", "TEST to start");
			        if (RST_pressed == 1)
			        {
						RST_pressed = 0;
						screen = SCR_CFG_POTL;						
			        }
			        break;

			    case 2:  // calib pressure sensor
			        display_to_LCD("Calib pressure", "TEST to start");
			        if (RST_pressed == 1)
			        {
			            pressure_baseline = int(sparkfunPress.getPressure(ADC_4096));
						RST_pressed = 0;
						screen = SCR_CFG_PRES_SAVED;
			        }
			        break;

			    case 3:  // move arm down once
			        if (progress == 0)
			        {
			            display_to_LCD("Press TEST to", "run one breath  ");
			            if (RST_pressed == 1)
			            {
							RST_pressed = 0;
			                initialize_breath();
			                progress = 1;
			            }
			        } else if (progress == 1) {
			            run_profile_func();
			            // if (cycle_number > 0)
			            //     exit_menu();
			        }
			        break;

					
			    case 4:  // calib arm range of movement
			        display_to_LCD("Calibrate Arm", "TEST to start");
			        if (RST_pressed == 1)
			        {
						RST_pressed = 0;
						screen = SCR_CFG_ARM_UP;
			        }
			        break;

			    case 5:  // set motion profile total time
			        display_to_LCD("Set Motion Time", "TEST to start ");
			        if (RST_pressed == 1)
			        {
						RST_pressed = 0;
						screen = SCR_CFG_MOTION;
					}
			        break;

			    case 6:  // toggle sync to patient
			        if (patient_triggered_breath == 1){ display_to_LCD("Sync to patient", "ON  ");}
			        if (patient_triggered_breath == 0){ display_to_LCD("Sync to patient", "OFF ");}
			        if (RST_pressed == 1){
						RST_pressed = 0;
			            patient_triggered_breath = 1 - patient_triggered_breath;
			        }
			        break;

			    default:
			        display_to_LCD("Exit Menu", "Press MENU"+String(menu_pos));
			        if (RST_pressed == 1){
						RST_pressed = 0;
					    // state = STBY_STATE;
						screen = SCR_STBY;
					    calibON = 0;
					    LED_USR(0); // BAZINGA
			        }
				break;
				
			} // end switch menu_pos
			break;
			
		case SCR_CFG_PRES_SAVED:
			display_to_LCD("Pressure setting", "saved");
			screen = SCR_CFG;
			break;
		
		case SCR_CFG_POTL:
        	display_to_LCD("Turn all pots LEFT", "TEST to continue");
			// calibrate_pot_range();
			if(RST_pressed == 1){            
				RST_pressed = 0;
			    comp_pot_low = analogRead(pin_AMP);
			    rate_pot_low = analogRead(pin_FRQ);
			    pres_pot_low = analogRead(pin_PRE);
				screen = SCR_CFG_POTR;
			}
			break;

		case SCR_CFG_POTR:
        	display_to_LCD("Turn all pots RIGHT", "TEST to continue");
			// calibrate_pot_range();
			if(RST_pressed == 1){            
				RST_pressed = 0;
			    comp_pot_high = analogRead(pin_AMP);
			    rate_pot_high = analogRead(pin_FRQ);
			    pres_pot_high = analogRead(pin_PRE);
				screen = SCR_CFG_POTS;
			}
			break;

		case SCR_CFG_POTS:
        	display_to_LCD("Saving pots calibration ", "");
		    EEPROM.put(12, comp_pot_low);
		    delay(100);
		    EEPROM.put(16, comp_pot_high);
		    delay(100);
		    EEPROM.put(20, rate_pot_low);
		    delay(100);
		    EEPROM.put(24, rate_pot_high);
		    delay(100);
		    EEPROM.put(28, pres_pot_low);
		    delay(100);
		    EEPROM.put(32, pres_pot_high);
		    delay(100);
			screen = SCR_CFG;
			break;

		case SCR_CFG_ARM_UP:
			display_to_LCD("Move arm upper", "position: " + String(A_sensed_pos));
			delay(100);
			if(RST_pressed == 1){            
				RST_pressed = 0;
			    min_arm_pos = A_sensed_pos;
				screen = SCR_CFG_ARM_DOWN;
			}
			break;
	
		case SCR_CFG_ARM_DOWN:
			display_to_LCD("Move arm lower", "position: " + String(A_sensed_pos));
			delay(100);
			if(RST_pressed == 1){            
				RST_pressed = 0;
			    max_arm_pos = A_sensed_pos;
				screen = SCR_CFG_ARM_SAFE;
			}
			break;

		case SCR_CFG_ARM_SAFE:
			delay(100);
			display_to_LCD("Release arm", "Gently");
			if(RST_pressed == 1){            
				RST_pressed = 0;
				screen = SCR_CFG_ARM_SAVE;
			}
			break;

		case SCR_CFG_ARM_SAVE:
			display_to_LCD("Saving Arm calibration", "");
			if(debug){Serial.println("Saving Arm Calibration. Lower:" + String(min_arm_pos) + " Upper:" + String(max_arm_pos));}
		    EEPROM.put(4, min_arm_pos);
		    delay(200);
		    EEPROM.put(8, max_arm_pos);
		    delay(200);
			screen = SCR_CFG;
			break;


		// case SCR_CFG_MOTION:
		// 	        motion_time = map(pot_rate, 0, 1023, 25, 50);
		// 	        motion_time = constrain(motion_time, 25, 50);
		//
		// 	String line2;
		// 	line2 += 100 * motion_time; //BAZINGA
		// 	line2 += " mSec";
		// 	display_to_LCD("Set Motion Time",line2);
		// 	if(RST_pressed == 1){
		// 		RST_pressed = 0;
		// 		screen = SCR_CFG_MOTION_SAVE;
		// 	}
		// 	break;

		case SCR_CFG_MOTION_SAVE:
			display_to_LCD("Motion setting", "saved");
			screen = SCR_CFG;
			break;


	
	}	
}











// Updates the display with whatever information was collected this loop
// Will freeze the Arduino for the lenght of the display_pause if any
void display_refresh()
{			
	// Screen has a max refresh rate to avoid flickering and unwated behaviours
	if((millis() - display_last) < display_refresh_rate){
		return;
	}else{
		display_last = millis();
	}

    // Not refreshing the screen if the content is the same (BAZINGA redundant with display_to_LCD)
    if( display_prev_line1 == display_line1 && display_prev_line2 == display_line2 ){ return; }
	display_prev_line1 = display_line1;
	display_prev_line2 = display_line2;
	
	// debug
	if(debug_screen){Serial.println("Screen: " + display_line1 + " || " + display_line2); delay(250);}
	else{display_updates=0;} // used in display_to_LCD to display any "lost" screen update

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(display_line1);
    lcd.setCursor(0, 1); // BAZINGA - Our New Haven display library is "backwards"...
    lcd.print(display_line2);

	// BAZINGA - is it needed? What does it do exactly?
    lastUSRblink = millis();
}

// Updates the display with whatever information was collected this loop
// Will freeze the Arduino for the lenght of the display_pause if any
void display_to_LCD(String line1, String line2)
{
	// Not updating if information has not changed
	if(display_line1==line1 && display_line2==line2){return;}

	display_line1 = line1;
	display_line2 = line2;

	// keeps track of the number of updates to catch if we are trying to update the screen more than once (see debug)
	display_updates += 1 ;
	
	//debug
	if(display_to_LCD>1){Serial.println("Debug screen: more than one update: " + display_line1 + " --- " + display_line2);}
}


// General background operations:
// - Resets the failure if they are not relevant anymore
// BAZINGA move all the operations in read_IO here
void operations()
{
	// BAZINGA reset failures temporarily
	reset_failures();
	
}

void run_profile_func()
{
    if (millis() - lastIndex >= wanted_cycle_time)  // do when cycle time was reached
    {
        cycles_lost = (millis() - lastIndex) / wanted_cycle_time - 1;
        cycles_lost = constrain(cycles_lost, 0, 15);
        lastIndex = millis();  // last start of cycle time
        calculate_wanted_pos_vel();

		// If the motion of the arm is too far away from expected
        if (100 * abs(error) / (max_arm_pos - min_arm_pos) > motion_control_allowed_error
            && cycle_number > 1)
            motion_failure = 1;

        if (safety_pressure_detected)
            index -= speed_multiplier_reverse
                     * (1 + cycles_lost);  // run in reverse if high pressure was detected
        if (index < 0)
        {
            if (safety_pressure_detected == 1)
                safety_pressure_counter += 1;  // count the number of cases reaching safety pressure
            safety_pressure_detected = 0;
            wait_cycles = 100 * wait_time_after_resistance;
            index = profile_length - 2;  // set index to the point of waiting
        }                                // stop the reverse when reching the cycle start point

        if (in_wait == 0)
            index += (1 + cycles_lost);  //  advance index while not waiting at the end of cycle
        if (patient_triggered_breath
            == 1)  // detect drop in presure during the PEEP plateu and trigger breath based on this
        {
            if (in_wait == 1 || (index > profile_length / 2 && (A_sensed_pos < min_arm_pos + range / 18)))
            {
                if (avg_pres - pressure_abs > delta_pres_patient_inhale)
                    start_new_cycle();  // start new breath cycle if patient tries to inhale durint
                                        // the PEEP plateu
                avg_pres =
                    avg_pres * alpha_pres
                    + (1 - alpha_pres) * float(pressure_abs);  // calculate the filtered pressure
            }
            else
            {
                avg_pres = pressure_abs;
            }  // initialize the filtered pressure
        }

        if (index >= (profile_length - 2))  // wait for the next cycle to begin in this point -> 2
                                            // points befoe the last cycle index
        {
            if (millis() - start_wait < breath_cycle_time)
            {
                index = profile_length - 2;
                in_wait = 1;  // still need to wait ...
            }
            else
                start_new_cycle();  // time has come ... start from index = 0
        }
        blink_user_led();
    }
    calc_failure();
    set_motor_PWM(wanted_vel_PWM);
    find_min_max_pressure();
}

void calculate_wanted_pos_vel()
{
    byte pos_from_profile, vel_from_profile;
    pos_from_profile = pgm_read_byte_near(pos + index);
    vel_from_profile = pgm_read_byte_near(vel + index + 1);

    range = range_factor * (max_arm_pos - min_arm_pos);  // range of movement in pot' readings
    wanted_pos = float(pos_from_profile) * range / 255 + min_arm_pos;  // wanted pos in pot clicks
    profile_planned_vel =
        (float(vel_from_profile) - 128.01) * range / 255;  // in clicks per 0.2 second

    planned_vel = profile_planned_vel;
    if (hold_breath == 1 && safety_pressure_detected == 0)
    {
        if (wanted_pos <= float(A_sensed_pos) || index == 0)
            hold_breath = 0;
        planned_vel = 0;
        integral = 0;
        wanted_pos = float(A_sensed_pos);  // hold current position
    }
    if (safety_pressure_detected)
        planned_vel = -speed_multiplier_reverse
                      * planned_vel;  // to do the revese in case high pressure detected
    prev_error = error;
    error = wanted_pos - float(A_sensed_pos);

    integral += error * float(wanted_cycle_time) / 1000;
    if (integral > integral_limit)
        integral = integral_limit;
    if (integral < -integral_limit)
        integral = -integral_limit;
    if (index < 2 || prev_error * error < 0)
        integral = 0;  // zero the integral accumulator at the beginning of cycle and movement up
    if (planned_vel < 0)
        f_reduction_up = f_reduction_up_val;
    else
        f_reduction_up = 1;  // reduce f for the movement up

    wanted_vel_PWM =
        FF * planned_vel * f_reduction_up + KP * error + KI * integral;  // PID correction
    wanted_vel_PWM = wanted_vel_PWM * float(cycleTime)
                     / float(wanted_cycle_time);  // reduce speed for longer cycles
}

void standby_func()  // not running profile
{
    if (USR_status)
    {
        if (millis() - lastUSRblink > 10)
        {
            USR_status = 0;
            lastUSRblink = millis();
            LED_USR(0);
        }
    }
    else
    {
        if (millis() - lastUSRblink > 490)
        {
            USR_status = 1;
            lastUSRblink = millis();
            LED_USR(1);
        }
    }

    if (TST_pressed)
    {
        initialize_breath();
        progress = 1;
    }
    if (progress == 1)
    {
        run_profile_func();
        if (cycle_number > 0)
            progress = 0;
    }
    else
    {
        wanted_vel_PWM = 0;  // dont move
        set_motor_PWM(wanted_vel_PWM);
    }
    delay(1);
}

void initialize_breath()
{
    cycle_number = 0;
    start_wait = millis();
    integral = 0;
    reset_failures();
    index = 0;
    in_wait = 0;
    high_pressure_detected = 0;
}

void start_new_cycle()
{
    index = 0;
    cycle_number += 1;
    start_wait = millis();
    in_wait = 0;
    send_beep = 1;
    high_pressure_detected = 0;
}

int range_pot(int val, int low, int high)
{
    int new_val;
    new_val = int(long(val - low) * long(1023) / (high - low));
    new_val = constrain(new_val, 0, 1023);
    return (new_val);
}

void find_min_max_pressure()
{
    if (max_pressure < pressure_abs)
        max_pressure = pressure_abs;  // find the max pressure in cycle
    if (min_pressure > pressure_abs)
        min_pressure = pressure_abs;  // find the min pressure in cycle
    if (index > profile_length - 10 && index < profile_length - 5)
    {
        prev_min_pressure = min_pressure;
        prev_max_pressure = max_pressure;
    }
    if (index >= profile_length - 5)
    {
        max_pressure = 0;
        min_pressure = 999;
    }
}

void blink_user_led()
{
    if (high_pressure_detected || safety_pressure_detected)  // blink LED fast
    {
        if (USR_status)
        {
            if (millis() - lastUSRblink > 20)
            {
                USR_status = 0;
                lastUSRblink = millis();
                LED_USR(0);
            }
        }
        else
        {
            if (millis() - lastUSRblink > 80)
            {
                USR_status = 1;
                lastUSRblink = millis();
                LED_USR(1);
            }
        }
    }
    else  //  not in failure - blink LED once per cycle
    {
        if (index > 0.1 * profile_length)
            LED_USR(0);
        else
            LED_USR(1);
    }
}

void calc_failure()
{
    if (prev_max_pressure < max_pres_disconnected && cycle_number > 2)
        disconnected = 1;
    else
        disconnected = 0;  // tube was disconnected
    if (pressure_abs > insp_pressure && hold_breath == 0 && profile_planned_vel > 0)
    {
        high_pressure_detected = 1;
        hold_breath = 1;
        index_to_hold_breath = index;
    }  // high pressure detected
    if (pressure_abs > safety_pressure && profile_planned_vel > 0)
        safety_pressure_detected = 1;
    if (pressure_abs > insp_pressure + safety_pres_above_insp && profile_planned_vel > 0)
        safety_pressure_detected = 1;
    if (index == 0 && prev_index != 0 && failure == 0 && safety_pressure_detected == 0)
        no_fail_counter += 1;
    if (index == 0)
        failure = 0;
    if (disconnected)
        failure = 1;
    if (safety_pressure_detected && safety_pressure_counter >= 1)
    {
        failure = 2;
        safety_pressure_counter = 1;
    }
    if (motion_failure)
        failure = 3;
    if (disconnected == 1 || motion_failure == 1 || safety_pressure_detected == 1)
    {
        no_fail_counter = 0;
    }
    else
    {
        LED_FAIL(0);
    }
    if (no_fail_counter >= 3)
        safety_pressure_counter = 0;
    if (no_fail_counter >= 100)
        no_fail_counter = 100;
    prev_index = index;
}

// In breathing & Standby mode, displays BPM, DEP and pressure
// displays possible failures on the second line
void display_breathing()  
{
    if (LCD_available)
    {
		// Building the two lines to be sent to the display
		String temp_line1 = "BPM:" + String(byte(BPM)) + "  Dep:" + String(byte(Compression_perc)) + "%";
		String temp_line2 = "";
		
        switch (failure)
		{
			case 0:
                if (millis() - start_disp_pres < 2000)
                {
					temp_line2 += "Insp. Press. :" + String(byte(insp_pressure));
                }
                else
                {
					temp_line2 += "Pmin:" + String(byte(prev_min_pressure)) + String(byte(prev_max_pressure)) + "  Pmax:" + String(byte(prev_max_pressure));
                }
				break;
			
			case 1:
				temp_line2 += "Pipe Disconnect";				
				break;
			
			case 2:
				temp_line2 += "High Pressure";
				break;
			
			case 3:
				temp_line2 += "Motion Fail";
				break;
		}	

		// Sending the info to the display buffer (see display_to_LCD())
		display_to_LCD(temp_line1, temp_line2);
    }
}

void reset_failures()
{
    motion_failure = 0;
    index_last_motion = index;
    failure = 0;
}

void set_motor_PWM(float wanted_vel_PWM)
{
    if (abs(A_sensed_pos - prev_A_sensed_pos) > 0 || abs(wanted_vel_PWM) < 15)
        index_last_motion = index;
    if (calibON == 1)
        wanted_vel_PWM = read_motion_for_calib();  // allows manual motion during calibration
    if (invert_mot)
        wanted_vel_PWM = -wanted_vel_PWM;
    if (curr_sense)
    {
        if (A_current > max_allowed_current)
            wanted_vel_PWM = 0;
    }
    if (motion_failure == 1 && calibON == 0)
        // wanted_vel_PWM = 0;
    if (wanted_vel_PWM > 0)
        wanted_vel_PWM += 3;  // undo controller dead band
    if (wanted_vel_PWM < 0)
        wanted_vel_PWM -= 3;  // undo controller dead band
    if (wanted_vel_PWM > PWM_max)
        wanted_vel_PWM = PWM_max;  // limit PWM
    if (wanted_vel_PWM < PWM_min)
        wanted_vel_PWM = PWM_min;  // limit PWM
    motorPWM = PWM_mid + int(wanted_vel_PWM);
    motor.write(motorPWM);
}

// Stop the arm movement
void motion_stop()
{
    wanted_vel_PWM = 0;  // dont move
    set_motor_PWM(wanted_vel_PWM);
}

int read_motion_for_calib()
{
    int wanted_cal_PWM;
    if (control_with_pot)
    {
        if (pot_rate > 750)
            wanted_cal_PWM = (pot_rate - 750) / 15;
        if (pot_rate < 250)
            wanted_cal_PWM = (pot_rate - 250) / 15;
        if (pot_rate >= 250 && pot_rate <= 750)
            wanted_cal_PWM = 0;
        if (SW2_cur == 1)
            wanted_cal_PWM = -12;
        // if (RST_cur_cur==1) wanted_cal_PWM= 12;
    }
    else
    {
        wanted_cal_PWM = 0;
        if (FD_cur == 1)
            wanted_cal_PWM = 8;
        if (FD_cur == 1)
            wanted_cal_PWM = -8;
        if (AD_cur == 1)
            wanted_cal_PWM = 16;
        if (AU_cur == 1)
            wanted_cal_PWM = -16;
    }
    return (wanted_cal_PWM);
}

// Read the current states of the user IO
// compares to previous state, if there has been an action saves that "state"
void read_IO()
{
	/* BAZINGA - previous shit - whyyyy?
    FD_prev = FD_cur;
    FU_prev = FU_cur;
    AD_prev = AD_cur;
    AU_prev = AU_cur;
    SW2_prev = SW2;
    prev_TST = TST;
    prev_BPM = BPM;
    prev_A_sensed_pos = A_sensed_pos;
    prev_Compression_perc = Compression_perc;
	*/	
	
	// RST Button - Used for the menu
	//RST_pressed = false; // Resetting any previous state
    RST_cur = (1 - digitalRead(RST_pin));

	if(RST_cur != RST_prev){ 
		RST_timer = millis();
	}

	// ignoring interactions that are too short (debounce)
	if((millis() - RST_timer) > 50){ 
		if(RST_cur != RST_debounced){
			RST_debounced = RST_cur;

			if(RST_debounced == 0){
				RST_pressed = 1;
				if(debug_IO){Serial.println("DEBUG IO: RST pressed!");}
			}
		}
	}
	RST_prev = RST_cur;


	// Start Button - 
    SW2_cur = (1 - digitalRead(SW2_pin));

	if(SW2_cur != SW2_prev){ 
		SW2_timer = millis();
	}

	// ignoring interactions that are too short (debounce)
	if((millis() - SW2_timer) > 50){ 
		if(SW2_cur != SW2_debounced){
			SW2_debounced = SW2_cur;

			if(SW2_debounced == 0){
				SW2_pressed = true;
				if(debug_IO){Serial.println("DEBUG IO: SW2 pressed!");}
			}
		}
	}
	SW2_prev = SW2_cur;
		
	// Test Button - 
    TST_cur = (1 - digitalRead(TST_pin));

	if(TST_cur != TST_prev){ 
		TST_timer = millis();
	}

	// ignoring interactions that are too short (debounce)
	if((millis() - TST_timer) > 50){ 
		if(TST_cur != TST_debounced){
			TST_debounced = TST_cur;

			if(TST_debounced == 0){
				TST_pressed = true;
				if(debug_IO){Serial.println("DEBUG IO: TST pressed!");}
			}
		}
	}
	TST_prev = TST_cur;
	
	

#if (magnetic_encoder)
	A_sensed_pos = 1024 * armSensor.angleR(U_TRN, true);
#else
	A_sensed_pos = analogRead(pin_POT);
#endif

    if (invert_pot)
        A_sensed_pos = 1023 - A_sensed_pos;
    A_current = analogRead(pin_CUR) / 8;  // in tenth Amps
    if (control_with_pot)
    {
        A_rate = analogRead(pin_FRQ);
        A_comp = analogRead(pin_AMP);
        A_pres = analogRead(pin_PRE);
        if (abs(pot_rate - A_rate) < 5)
            pot_rate = pot_alpha * pot_rate + (1 - pot_alpha) * A_rate;
        else
            pot_rate = A_rate;
        if (abs(pot_comp - A_comp) < 5)
            pot_comp = pot_alpha * pot_comp + (1 - pot_alpha) * A_comp;
        else
            pot_comp = A_comp;
        if (abs(pot_pres - A_pres) < 5)
            pot_pres = pot_alpha * pot_pres + (1 - pot_alpha) * A_pres;
        else
            pot_pres = A_pres;
        A_comp = range_pot(int(pot_comp), comp_pot_low, comp_pot_high);
        A_rate = range_pot(int(pot_rate), rate_pot_low, rate_pot_high);
        A_pres = range_pot(int(pot_pres), pres_pot_low, pres_pot_high);

        Compression_perc = perc_of_lower_vol_display
                           + int(float(A_comp) * (100 - perc_of_lower_vol_display) / 1023);
        Compression_perc = constrain(Compression_perc, perc_of_lower_vol_display, 100);

        BPM = 6 + (A_rate - 23) / 55;           // 0 is 6 breaths per minute, 1023 is 24 BPM
        breath_cycle_time = 60000 / BPM + 100;  // in milisec

        insp_pressure = 30 + A_pres / 25;  // 0 is 30 mBar, 1023 is 70 mBar
        insp_pressure = constrain(insp_pressure, 30, 70);

        if (abs(insp_pressure - prev_insp_pressure) > 1)
        {
            prev_insp_pressure = insp_pressure;
            start_disp_pres = millis();
        }
    }
    else
    {
        FD_cur = (1 - digitalRead(FD_pin));
        FD_cur = (1 - digitalRead(FU_pin));
        AD_cur = (1 - digitalRead(AD_pin));
        AU_cur = (1 - digitalRead(AU_pin));
        if (TST_pressed == 0)
        {
            if (FD_cur == 0 && FD_prev == 1)
            {
                BPM -= 2;
                if (BPM < 6)
                    BPM = 6;
                cycle_number = 0;
            }
            if (FD_cur == 0 && FU_prev == 1)
            {
                BPM += 2;
                if (BPM > 24)
                    BPM = 24;
                cycle_number = 0;
            }
            breath_cycle_time = 60000 / BPM + 100;
            if (AD_cur == 0 && AD_prev == 1)
            {
                Compression_perc -= deltaUD;
                if (Compression_perc < perc_of_lower_vol_display)
                    Compression_perc = perc_of_lower_vol_display;
            }
            if (AU_cur == 0 && AU_prev == 1)
            {
                Compression_perc += deltaUD;
                if (Compression_perc > 100)
                    Compression_perc = 100;
            }
        }
        if (TST_pressed == 1)
        {
            if (FD_cur == 0 && FD_prev == 1)
            {
                insp_pressure -= 5;
                if (insp_pressure < 30)
                    insp_pressure = 30;
            }
            if (FD_cur == 0 && FU_prev == 1)
            {
                insp_pressure += 5;
                if (insp_pressure > 70)
                    insp_pressure = 70;
            }
            if (AD_cur == 0 && AD_prev == 1)
            {
                insp_pressure -= 5;
                if (insp_pressure < 30)
                    insp_pressure = 30;
            }
            if (AU_cur == 0 && AU_prev == 1)
            {
                insp_pressure += 5;
                if (insp_pressure > 70)
                    insp_pressure = 70;
            }
        }
    }
	
    if (is_starting_respiration())
    {
        range_factor = perc_of_lower_volume
                       + (Compression_perc - perc_of_lower_vol_display)
                             * (100 - perc_of_lower_volume) / (100 - perc_of_lower_vol_display);
        range_factor = range_factor / 100;
        if (range_factor > 1)
            range_factor = 1;
        if (range_factor < 0)
            range_factor = 0;
    }

#if (pressure_sensor_available == 1)
    {
        if (millis() - last_read_pres > 100)
        {
            last_read_pres = millis();
            pressure_abs = int(sparkfunPress.getPressure(ADC_4096) - pressure_baseline);  // mbar
            if (pressure_abs < 0)
                pressure_abs = 0;
        }
    }
#endif

	// BAZINGA - should not be necessary anymore
    // if (prev_BPM != BPM || prev_Compression_perc != Compression_perc)
    //     display_breathing();
    wanted_cycle_time = int(100) * int(motion_time) / profile_length;
    if (wanted_cycle_time > breath_cycle_time / profile_length)
        wanted_cycle_time = breath_cycle_time / profile_length;
    if (wanted_cycle_time < cycleTime)
        wanted_cycle_time = cycleTime;
}

bool is_starting_respiration()
{
    return index == 0;
}

// Sends data over serial to be display by a monitor through an interface (UNDOCUMENTED)
// controlled by send_to_monitor
void send_data_to_monitor()
{
	if(send_to_monitor){
		// only refresh every data_frequency
	    if (millis() - last_sent_data < data_frequency) {return;}
		else{last_sent_data = millis();}
	
	    if (monitor_index == 0)
	        Serial.println("A");
	    if (monitor_index == 1)
	        Serial.println(byte(BPM));
	    if (monitor_index == 2)
	        Serial.println(byte(Compression_perc));
	    if (monitor_index == 3)
	        Serial.println(byte(pressure_abs));
	    if (monitor_index == 4)
	        Serial.println(byte(failure));
	    if (monitor_index == 5)
	    {
	        if (send_beep)
	        {
	            Serial.println(byte(1));
	            send_beep = 0;
	        }
	        else
	            Serial.println(byte(0));
	    }
	    if (monitor_index == 6)
	        Serial.println(byte(insp_pressure));
	    monitor_index += 1;
	    if (monitor_index == 7)
	        monitor_index = 0;
	}
}

void LED_FREQ(byte val)
{
    digitalWrite(pin_LED_FREQ, val);
}

void LED_AMP(byte val)
{
    digitalWrite(pin_LED_AMP, val);
}

void LED_FAIL(byte val)
{
    digitalWrite(pin_LED_Fail, val);
}

void LED_USR(byte val)
{
    digitalWrite(pin_USR, val);
}


// Serial debugging of the machine
// You can comment/uncomment any line necessary as needed
// controlled by the variable telemetry
void print_tele()
{
	if(telemetry){
		// only refresh every serial_frequency
	    if (millis() - last_sent_tele < tele_frequency) {return;}
		else{last_sent_tele = millis();}
	
	    Serial.print(" Current Failure (disc,motion,hiPres):"); Serial.print(disconnected); Serial.print(",");
	    Serial.print(motion_failure); Serial.print(","); Serial.print(high_pressure_detected);
	    Serial.print(" CL:");  Serial.print(cycles_lost);
	    Serial.print(" min,max:");  Serial.print(min_arm_pos); Serial.print(",");
	    Serial.print(max_arm_pos); Serial.print(" WPWM :");  Serial.print(motorPWM); Serial.print("integral:   ");  Serial.print(int(integral));
	    Serial.print(" Wa:");
	    Serial.print(int(wanted_pos));
	    Serial.print(" Ac:");
	    Serial.print(A_sensed_pos);
	    Serial.print(" cur:");  Serial.print(A_current);
	    Serial.print(" amp:");  Serial.print(Compression_perc);
	    Serial.print(" freq:");  Serial.print(A_rate);
	    Serial.print(" w cyc t:"); Serial.print(wanted_cycle_time);
	    Serial.print(" P :"); Serial.print(pressure_abs);
	    Serial.print(" AvgP :"); Serial.print(int(avg_pres));
	    Serial.print(" RF:");  Serial.print(range_factor);
	    Serial.println("");
	}
}

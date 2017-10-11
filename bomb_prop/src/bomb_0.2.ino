
#include <Arduino.h>
#include <Wire.h>
#include <Keypad.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define DEBUG
//#define FASTFORWARD
//#define SKIP_USB
//#define SKIP_WIRES
#define SKIP_BUTTON
#define NUM_BUTTON_PRESSES 1
#define SKIP_CODES
#define NUM_CODES 1
#define SKIP_SWITCHES

#define DELAY_MS 100
#define ADC_DELTA 50

#define A0_IN A0
#define A1_IN A1
#define A2_IN A2
#define A3_IN A3
#define A4_IN A4
#define B0_IN A5
#define B1_IN A6
#define B2_IN A7
#define B3_IN A8
#define B4_IN A9
#define NUM_ADC_CH 10

#define USB_DP A10
#define USB_DM A11
#define USB_TARGET 250
#define USB_TARGET_MARGIN 250

#define A0_RED_LED 42
#define A1_RED_LED 41
#define A2_RED_LED 40
#define A3_RED_LED 39
#define A4_RED_LED 38
#define B0_RED_LED 47
#define B1_RED_LED 46
#define B2_RED_LED 45
#define B3_RED_LED 44
#define B4_RED_LED 43
#define NUM_LEDS 5

#define SW0_BLUE 29
#define SW0_BLUE_INDEX 0
#define SW1_YELLOW 28
#define SW1_YELLOW_INDEX 1
#define SW2_GREEN 27
#define SW2_GREEN_INDEX 2
#define SW3_WHITE 26
#define SW3_WHITE_INDEX 3

#define NUM_SWITCHES 6
#define INPUT_SWITCH_VALUE 0
#define INPUT_SWITCH_DEBOUNCE 1
#define DEBOUNCE_TIME 50  //ms

#define RED_BUTTON_LED 13
#define RED_BUTTON_SW 12
#define RED_BUTTON_INDEX 4
#define BIG_RED_BUTTON_LED 11
#define BIG_RED_BUTTON_SW 10
#define BIG_RED_BUTTON_INDEX 5

#define ALPHANUM_SDA SDA
#define ALPHANUM_SCL SCL

#define NEOPIXEL_DATA 2
#define NUM_PIXELS 5
#define BIG_LED_RED 3
#define BIG_LED_GREEN 4
#define BIG_LED_BLUE 5

#define PIEZO_SPKR 6

#define UV_LED1 36
#define UV_LED2 37

#define BRIGHTNESS 127  // defines max brightness level for neopixels (max = 255)

#define CONTROL_IDLE 0
#define CONTROL_INIT 1
#define CONTROL_FLASH_CHECK_LED 2
#define CONTROL_SAY_HI 3
#define CONTROL_WAIT_FOR_USB_KEY 4
#define CONTROL_DEFUSE_INIT 5
#define CONTROL_DEFUSE 6
#define CONTROL_WIN 7
#define CONTROL_DONE 8

#define DEFUSE_IDLE 0
#define DEFUSE_INIT 1
#define DEFUSE_WIRES 2
#define DEFUSE_SWITCHES 3
#define DEFUSE_CODES 4
#define DEFUSE_BUTTON 5
#define DEFUSE_CHECK 6

#define WIRE_IDLE 0
#define WIRE_INIT 1
#define WIRE_SETUP 2
#define WIRE_CHECK 3

#define SWITCH_IDLE 0
#define SWITCH_INIT 1
#define SWITCH_SETUP 2
#define SWITCH_CHECK 3

#define CODE_IDLE 0
#define CODE_INIT 1
#define CODE_SETUP 2
#define CODE_CHECK 3

#define BUTTON_IDLE 0
#define BUTTON_INIT 1
#define BUTTON_SETUP 2
#define BUTTON_CHECK 3

#define OFF 0
#define ON 1
#define SLOW_BLINKING 2
#define FAST_BLINKING 3

#define WIRE_SEQ_NUM 0
#define COLOR_NUM 1
#define ADC_CH 2
#define LED_A 3
#define LEDA_MODE 4
#define LED_B 5
#define LEDB_MODE 6

Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();  //4-character alphanumeric display object

const byte ROWS = 4; // Four rows
const byte COLS = 4; // Three columns
// Define the Keymap
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'#','0','*','D'}
};
// Connect keypad ROW0, ROW1, ROW2 and ROW3 to these Arduino pins.
byte rowPins[ROWS] = { 21, 20, 19, 18 };
// Connect keypad COL0, COL1 and COL2 to these Arduino pins.
byte colPins[COLS] = { 17, 16, 15, 14 }; 

// Create the Keypad
Keypad kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );


// Neopixel definition
// Parameter 1 = number of pixels in strip  --> we have 5
// Parameter 2 = Arduino pin number (most are valid)  -->
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)  --> this is the one we are using
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, NEOPIXEL_DATA, NEO_RGB + NEO_KHZ800);

int adcPins[] = {A0_IN, A1_IN, A2_IN, A3_IN, A4_IN, B0_IN, B1_IN, B2_IN, B3_IN, B4_IN};
int adcValues[] = {0,0,0,0,0,0,0,0,0,0};
int adcValuesOld[] = {0,0,0,0,0,0,0,0,0,0};
int adcValueUSB_DP = 0;
int adcValueUSB_DM = 0;
byte LEDApins[] = {A0_RED_LED, A1_RED_LED, A2_RED_LED, A3_RED_LED, A4_RED_LED};
byte LEDBpins[] = {B0_RED_LED, B1_RED_LED, B2_RED_LED, B3_RED_LED, B4_RED_LED};
byte LEDAvalues[] = {0,0,0,0,0};
byte LEDBvalues[] = {0,0,0,0,0};
unsigned long ul_PreviousMillis = 0UL;
unsigned long ul_PreviousPixelMillis = 0UL;
unsigned long ul_PreviousRedLEDMillis = 0UL;
unsigned long ul_PreviousWireMillis = 0UL;
unsigned long ul_PreviousTimerMillis = 0UL;
unsigned long ul_PixelIntervalMillis = 50UL;
unsigned long ul_MsgIntervalMillis = 200UL;
unsigned long ul_PreviousMsgMillis = 0UL;
unsigned long ul_100msMillis = 100UL;
uint8_t pixelSequence = 0;
uint8_t currentPixel = 0;
uint8_t currentLED = 0;
uint16_t rainbowIndex = 0;
boolean direction = false;
boolean pressed = false;
boolean redButtonState = false;
uint8_t switches[] = {SW0_BLUE, SW1_YELLOW, SW2_GREEN, SW3_WHITE, RED_BUTTON_SW, BIG_RED_BUTTON_SW}; 
uint16_t switchValues[] = {0,0,0,0,1,1};
uint16_t switchValuesOld[] = {0,0,0,0,1,1}; 
uint8_t SWChannel = 0;
boolean inputFlag = false;
uint8_t input_switch_state = INPUT_SWITCH_VALUE;
boolean USBkeyInserted = false;
boolean redButtonPressed = false;
boolean flashRedLED = false;
uint8_t control_state = CONTROL_INIT;
boolean updateFlag = false;
uint8_t defuse_state = DEFUSE_IDLE;
uint8_t wire_state = WIRE_IDLE;
uint8_t switch_state = SWITCH_IDLE;
uint8_t code_state = CODE_IDLE;
uint8_t button_state = BUTTON_IDLE;
uint8_t nDefuseIter = 0;
uint8_t nDefuseIterReq = 1;
char *wireColorStr[6] = {0, "BLACK", "RED", "GREEN", "YELLOW", "BLUE"};  //start enumeration at 1
char *ordinalStr[6] = {"ZEROETH", "FIRST", "SECOND", "THIRD", "FOURTH", "FIFTH"};
int wireADCTarget[6] = {0, 0, 327, 704, 512, 254};
char msgStr[50];
uint8_t wireIter = 0;
uint8_t wireIterMax = 5;
boolean everyOther = false;
uint8_t wireOrdinal = 0;
uint8_t codeOrdinal = 0;
uint8_t buttonOrdinal = 0;
uint8_t swOrdinal = 0;
uint8_t msgIter = 0;
int countDownTimer = 0;
char charToSend[4];
boolean bigRedButtonPressed = false;

							// WIRE_SEQ_NUM, COLOR_NUM, ADC_CH, LED_A, LEDA_MODE, LED_B, LEDB_MODE --> indices is #defined
uint8_t wireSequenceArray[25][7] = {{ 1, 1, 3, 3, SLOW_BLINKING, 1, ON },
									{ 2, 2, 4, 4, ON, 0, OFF },
									{ 3, 3, 1, 1, OFF, 4, FAST_BLINKING },
									{ 4, 4, 0, 0, ON, 3, ON },
									{ 5, 5, 6, 1, OFF, 1, OFF },
									{ 1, 3, 1, 1, FAST_BLINKING, 4, FAST_BLINKING },
									{ 2, 5, 5, 1, SLOW_BLINKING, 0, SLOW_BLINKING },
									{ 3, 4, 4, 4, OFF, 0, OFF },
									{ 4, 2, 6, 3, ON, 1, SLOW_BLINKING },
									{ 5, 1, 9, 4, ON, 4, ON },
									{ 1, 5, 2, 2, ON, 1, OFF },
									{ 2, 3, 3, 3, OFF, 0, ON },
									{ 3, 2, 8, 0, OFF, 3, OFF },
									{ 4, 1, 9, 3, FAST_BLINKING, 4, SLOW_BLINKING },
									{ 5, 4, 7, 2, SLOW_BLINKING, 2, FAST_BLINKING },
									{ 1, 4, 9, 4, OFF, 4, OFF },
									{ 2, 1, 4, 4, OFF, 0, ON },
									{ 3, 5, 1, 1, OFF, 4, SLOW_BLINKING },
									{ 4, 3, 7, 2, OFF, 2, FAST_BLINKING },
									{ 5, 2, 0, 0, ON, 1, ON },
									{ 1, 2, 1, 1, FAST_BLINKING, 0, SLOW_BLINKING },
									{ 2, 4, 3, 3, SLOW_BLINKING, 3, FAST_BLINKING },
									{ 3, 1, 2, 2, FAST_BLINKING, 4, FAST_BLINKING },
									{ 4, 5, 4, 4, SLOW_BLINKING, 1, SLOW_BLINKING },
									{ 5, 3, 0, 0, FAST_BLINKING, 2, ON }};

uint8_t LEDmodeLUT[4][6] = {{ 0, 1, 2, 3, 2, 2},
							{ 0, 3, 0, 2, 3, 0},
							{ 0, 0, 3, 0, 1, 3},
							{ 0, 2, 1, 1, 0, 1}};
							
char *buttonWords[10][4] = {{      0,      0,      0,      0 },
							{ "WHEN", "METE", "REAL", "PREY" },  // '1'
							{ "FOUR", "MEET", "REEL", "BASS" },  // '2'
							{ "HAIR", "MEAT", "FLUE", "BASE" },  // '3'
							{ "PRAY", "POOR", "FLEW", "AYES" },  // '4'
							{ "LONE", "POUR", "WISE", "EYES" },  // '5'
							{ "LOAN", "PORE", "WHYS", "SAYS" },  // '6'
							{ "FARE", "LIEN", "ROES", "WHAT" },  // '7'
							{ "FAIR", "LEAN", "ROWS", "STOP" },  // '8'
							{ "FORE", "HARE", "ROSE", "WAIT" }}; // '9'

char codes[]   = {'M', 'X', 'T', 'S', 'W', 'F', 'U', 'K', 'R', 'Q', 'P', 'Y', 'G', 'H', 'J', 'Z', '9', '8', '7', '6', '5', '4', '3', '2', '1', '0' };
char decodes[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9', '0', 'A', 'B', 'C', 'D', '*', '#', 'D', 'C', 'B', 'A', '9', '8', '7', '6', '5', '4' };
uint8_t codeIndex[] = {0,0,0,0};
uint8_t switchLatest = 0;
uint8_t switched = 0;
uint8_t switchOrder[] = {0, 2, 0, 3, 1};
char *thisWord;
uint8_t thisWordNum = 0;
uint8_t switchTarget[] = {0,0,0,0,0};
boolean thrown = false;

void setup() {

	for (byte i=0; i<NUM_LEDS; i++)
	{
		pinMode(LEDApins[i], OUTPUT);
		pinMode(LEDBpins[i], OUTPUT);
		digitalWrite(LEDApins[i],LEDAvalues[i]);
		digitalWrite(LEDBpins[i],LEDBvalues[i]);
	}
	
	for (byte i=0; i<NUM_ADC_CH; i++)
	{
		pinMode(adcPins[i], INPUT);
	}
	pinMode(USB_DP, INPUT_PULLUP);
	pinMode(USB_DM, INPUT_PULLUP);
	
	for (byte i=0; i<NUM_SWITCHES; i++)
	{
		pinMode(switches[i], INPUT);
	}
	
	pinMode(RED_BUTTON_LED, OUTPUT);
	digitalWrite(RED_BUTTON_LED, LOW);  //start with red button LED off
	pinMode(RED_BUTTON_SW, INPUT_PULLUP);
	
	pinMode(BIG_RED_BUTTON_LED, OUTPUT);
	digitalWrite(BIG_RED_BUTTON_LED, LOW);  //start with red button LED off
	pinMode(BIG_RED_BUTTON_SW, INPUT_PULLUP);
	
	pinMode(BIG_LED_RED, OUTPUT);
	digitalWrite(BIG_LED_RED, HIGH);
	pinMode(BIG_LED_GREEN, OUTPUT);
	digitalWrite(BIG_LED_GREEN, HIGH);
	pinMode(BIG_LED_BLUE, OUTPUT);
	digitalWrite(BIG_LED_BLUE, HIGH);
	
	pinMode(PIEZO_SPKR, OUTPUT);
	digitalWrite(PIEZO_SPKR, LOW);
	delay(200);
	
	pinMode(UV_LED1, OUTPUT);
	digitalWrite(UV_LED1, HIGH);
	pinMode(UV_LED2, OUTPUT);
	digitalWrite(UV_LED2, HIGH);
	
	strip.begin();
	strip.show(); // Initialize all neopixels to 'off'
	strip.setBrightness(BRIGHTNESS);
	
	#ifdef DEBUG
		Serial.begin(115200);
		Serial.println("Starting serial console...");
	#endif
	
	alphaSetup();

	// int testtime = 5959;
	// char charToSend[4];
	
	// for (int i=0; i<60; i++)
	// {
	// sprintf(charToSend,"%d",testtime);
	// sendWord(charToSend);
	// delay(1000);
	// testtime--;
	//}

	sendWord("    ");
	//waitForUSBkey();
}

char displaybuffer[4] = {' ', ' ', ' ', ' '};

void loop()
{
	 // #ifdef DEBUG
		// Serial.print("state = ");
		// Serial.println(control_state);
	 // #endif
//	displayTime();
	controlStateMachine();
//	if (flashRedLED) {updateRedLED(200);}
	if (updateFlag==true) {getKeypad();}
	readSwitches();
	if ((inputFlag==true) && (updateFlag==true))
	{
		updateButtons();
	}
}


void controlStateMachine()
{
	boolean defuseComplete = false;
	switch(control_state)
    {
    case CONTROL_IDLE:
        // nothing
        break;
	case CONTROL_INIT:
		redButtonPressed = false;
		digitalWrite(RED_BUTTON_LED, HIGH);
		digitalWrite(BIG_RED_BUTTON_LED, HIGH);
		updateFlag = true;
		delay(200);
		control_state = CONTROL_FLASH_CHECK_LED;
		break;
	case CONTROL_FLASH_CHECK_LED:
		//flashRedLED = true;
		updateRedLED(200);
		if (redButtonPressed == true)
		{
			flashRedLED = false;
			redButtonPressed = false;
			control_state = CONTROL_SAY_HI;
		}
		break;
	case CONTROL_SAY_HI:
		sendMessage("    Hello there...     ");
		delay(500);
	#ifndef FASTFORWARD	
		sendMessage("    How are you today?     ");
		delay(500);
		sendMessage("    Would you like to play a game?     ");
		delay(1000);
		sendMessage("    first you will need to find a key     ");
		delay(500);
		sendMessage("    well?     ");
		delay(500);
		sendMessage("    tick tock, tick tock     ");
		delay(500);
		sendMessage("    hahahaha     ");
		delay(500);
		sendMessage("    a little bomb humor there     ");
		delay(500);
	#endif
		control_state = CONTROL_WAIT_FOR_USB_KEY;
		break;
	case CONTROL_WAIT_FOR_USB_KEY:
		#ifdef SKIP_USB
			USBkeyInserted = true;
		#endif
		if (USBkeyInserted==false)
		{
			adcValueUSB_DP = analogRead(USB_DP);
			adcValueUSB_DM = analogRead(USB_DM);
			#ifdef DEBUG
				Serial.println(adcValueUSB_DP);
				Serial.println(adcValueUSB_DM);
			#endif
				
			if ((adcValueUSB_DP > USB_TARGET - USB_TARGET_MARGIN) && (adcValueUSB_DP < USB_TARGET + USB_TARGET_MARGIN) && (adcValueUSB_DM > USB_TARGET - USB_TARGET_MARGIN) && (adcValueUSB_DM < USB_TARGET + USB_TARGET_MARGIN))
			{
				USBkeyInserted = true;
			}
		}
		else
		{
			control_state = CONTROL_DEFUSE_INIT;
		}
		break;
	case CONTROL_DEFUSE_INIT:
		for (int i=0; i<4; i++)
		{
			displaybuffer[i] = ' ';
		}
		sendMessage("    Defuse mode activated     ");
		defuse_state = DEFUSE_INIT;
		control_state = CONTROL_DEFUSE;
		break;
	case CONTROL_DEFUSE:
		updateNeoPixels(30, 6);
		defuseComplete = defuseStateMachine();
		if (defuseComplete)
		{
			control_state = CONTROL_WIN;
		}
		break;
	case CONTROL_WIN:
		//yay!
		//sendMessage("    Defusal successful     ");
		sendMessage("    Bomb Defused     ");
		msgIter = 0;
		control_state = CONTROL_DONE;
		break;
	case CONTROL_DONE:
		sendMessageNoDelay("     Door code 8535     ");
		updateNeoPixels(50, 1);
		break;
	default:
		control_state = CONTROL_IDLE;
		break;
	}
}
		
boolean defuseStateMachine()
{
	boolean wiresComplete = false;
	boolean switchesComplete = false;
	boolean codeComplete = false;
	boolean buttonComplete = false;
	
	switch(defuse_state)
	{
	case DEFUSE_IDLE:
		break;
	case DEFUSE_INIT:
		defuse_state = DEFUSE_WIRES;
		wire_state = WIRE_INIT;
		break;
	case DEFUSE_WIRES:
		wiresComplete = wireStateMachine();
		if (wiresComplete)
		{
			defuse_state = DEFUSE_SWITCHES;
			switch_state = SWITCH_INIT;
		}
		break;
	case DEFUSE_SWITCHES:
		switchesComplete = switchStateMachine();
		if (switchesComplete)
		{
			defuse_state = DEFUSE_CODES;
			code_state = CODE_INIT;
		}
		break;
	case DEFUSE_CODES:
		codeComplete = codeStateMachine();
		if (codeComplete)
		{
			defuse_state = DEFUSE_BUTTON;
			button_state = BUTTON_INIT;
		}
		break;
	case DEFUSE_BUTTON:
		buttonComplete = buttonStateMachine();
		if (buttonComplete)
		{
			defuse_state = DEFUSE_CHECK;
		}
		break;
	case DEFUSE_CHECK:
		nDefuseIter++;
		if (nDefuseIter >= nDefuseIterReq)
		{
			defuse_state = DEFUSE_IDLE;
			return true;
		}
		else
		{
			defuse_state = DEFUSE_INIT;
		}
		break;
	}
	return false;
}

boolean wireStateMachine()
{
	switch(wire_state)
	{
	case WIRE_IDLE:
		break;
	case WIRE_INIT:
		#ifdef SKIP_WIRES
			wire_state = WIRE_IDLE;
			return true;
		#endif
		sendMessage("    Wire connection sequence     ");
		wire_state = WIRE_SETUP;
		wireOrdinal = 1;
		redButtonPressed = false;
		break;
	case WIRE_SETUP:   //first wire, iteration wireIter
		
		sprintf(msgStr, "    Connect %s wire (%s)     ",ordinalStr[wireOrdinal],wireColorStr[wireSequenceArray[wireIter*5+wireOrdinal-1][COLOR_NUM]]);
		Serial.println(msgStr);
		msgIter = 0;
		sendMessageNoDelay(msgStr);
		updateWireLEDValues(wireIter*5+wireOrdinal-1);
		wire_state = WIRE_CHECK;
		break;
	case WIRE_CHECK:
		sendMessageNoDelay(msgStr);
		updateWireLEDs(250);
		if (redButtonPressed == true)
		{
			redButtonPressed = false;
			readADC();
			#ifdef DEBUG
				Serial.print("ADC channel = ");
				Serial.println(wireSequenceArray[wireIter*5+wireOrdinal-1][ADC_CH]);
				Serial.print("ADC value = ");
				Serial.println(adcValues[wireSequenceArray[wireIter*5+wireOrdinal-1][ADC_CH]]);
				Serial.print("Target value = ");
				Serial.println(wireADCTarget[wireOrdinal]);
			#endif
			//check if wire is connected correctly
			if ((adcValues[wireSequenceArray[wireIter*5+wireOrdinal-1][ADC_CH]] > wireADCTarget[wireSequenceArray[wireIter*5+wireOrdinal-1][COLOR_NUM]] - ADC_DELTA) && (adcValues[wireSequenceArray[wireIter*5+wireOrdinal-1][ADC_CH]] < wireADCTarget[wireSequenceArray[wireIter*5+wireOrdinal-1][COLOR_NUM]] + ADC_DELTA))   // if correct, move on to the next step
			{
				sendMessage("    GOOD     ");
				if (wireOrdinal==5)  //final wire is correct.  Yay!
				{
					sendMessage("    Wire connections complete     ");
					wire_state = WIRE_IDLE;
					return true;
				}
				wireOrdinal++;
				wire_state = WIRE_SETUP;
			}
			else     //if not, reset this step (but continue with next wire sequence)
			{
				//tone(PIEZO_SPKR, 1200, 1000);
				sendMessage("    INCORRECT     ");
				//sendMessage("    Restarting wire connection sequence      ");
				sendMessage("    Remove all wires     ");
				wireIter = (wireIter + 1) % wireIterMax;
				wireOrdinal = 1;
				wire_state = WIRE_SETUP;
			}
		}
		break;
		
	}
	return false;
}

boolean switchStateMachine()
{
	switch(switch_state)
	{
	case SWITCH_IDLE:
		break;
	case SWITCH_INIT:
		#ifdef SKIP_SWITCHES
			switch_state = SWITCH_IDLE;
			return true;
		#endif
		sendMessageNoDelay("    Turn all switches off (down) to begin switch sequence    ");
		swOrdinal = 1;
		for (int i=0; i<4; i++)
		{
			switchTarget[i] = 0;
		}
		if((switchValues[0]==0)&&(switchValues[1]==0)&&(switchValues[2]==0)&&(switchValues[3]==0))
		{
			//sendMessage("    Enter switch sequence     ");
			switch_state = SWITCH_SETUP;
		}
		break;
	case SWITCH_SETUP:
		switchTarget[switchOrder[swOrdinal]] = 1;
		switch_state = SWITCH_CHECK;
		break;
	case SWITCH_CHECK:
		sendMessageNoDelay("    Enter switch sequence     ");
		#ifdef DEBUG
			Serial.print("switched = ");
			Serial.println(switched);
			Serial.print("switchOrder[");
			Serial.print(swOrdinal);
			Serial.print("] = ");
			Serial.println(switchOrder[swOrdinal]);
		#endif
		for (int i=0; i<4; i++)
		{
			if ((switchValues[i]!=switchTarget[i])&&(i!=switchOrder[swOrdinal]))  // incorrect switch setting detected
			{
				sendMessage("    INCORRECT    ");
				switch_state = SWITCH_INIT;
				return false;
			}
			else if ((switchValues[i]!=switchTarget[i])&&(i==switchOrder[swOrdinal]))  // correct switch not yet thrown
			{
				thrown = false;   // set flag stating the next switch has not yet been thrown
			}
			else
			{
				thrown = true;	// set flag stating the next switch in sequence has been thrown
			}
		}
		if (thrown)
		{
			if (swOrdinal==4)  //sequence is complete
			{
				switch_state = SWITCH_IDLE;
				return true;
			}
			swOrdinal++;		//go to next switch in sequence
			switch_state = SWITCH_SETUP;
		}
		break;
	}
	return false;
}
	
boolean codeStateMachine()
{
	boolean fail = false;
	switch(code_state)
	{
	case CODE_IDLE:
		break;
	case CODE_INIT:
		#ifdef SKIP_CODES
			code_state = CODE_IDLE;
			return true;
		#endif
		sendMessage("    Start decoding process    ");
		codeOrdinal = 1;
		code_state = CODE_SETUP;
		break;
	case CODE_SETUP:
		for (int i=0; i<4; i++)
		{
			codeIndex[i] = random(26);
			msgStr[i] = codes[codeIndex[i]];
		}
		#ifdef DEBUG
			Serial.print("Code = ");
			Serial.print(codes[codeIndex[0]]);
			Serial.print(codes[codeIndex[1]]);
			Serial.print(codes[codeIndex[2]]);
			Serial.println(codes[codeIndex[3]]);
			Serial.print("Decode = ");
			Serial.print(decodes[codeIndex[0]]);
			Serial.print(decodes[codeIndex[1]]);
			Serial.print(decodes[codeIndex[2]]);
			Serial.println(decodes[codeIndex[3]]);
		#endif
		sendMessage("    Decode    ");
		sendWord(msgStr);
		for (int i=0; i<4; i++) {displaybuffer[i] = ' ';}
		code_state = CODE_CHECK;
		break;
	case CODE_CHECK:
		if (redButtonPressed == true)
		{
			fail = false;
			redButtonPressed = false;
			for (int i=0; i<4; i++)
			{
				if (displaybuffer[i] != decodes[codeIndex[i]])
				{
					//tone(PIEZO_SPKR, 1200, 1000);
					sendMessage("    INCORRECT     ");
					code_state = CODE_INIT;
					fail = true;
					break; // from for loop
				}
			}
			if (fail) {break;} //from switch statement
			sendMessage("    GOOD     ");
			if (codeOrdinal==NUM_CODES)  //final code is correct.  Yay!
			{
				sendMessage("    Decoding complete     ");
				code_state = CODE_IDLE;
				return true;
			}
			codeOrdinal++;
			code_state = CODE_SETUP;
		}
		break;
	}
	return false;
}
	
boolean buttonStateMachine()
{
	
	switch(button_state)
	{
	case BUTTON_IDLE:
		break;
	case BUTTON_INIT:
		#ifdef SKIP_BUTTON
			button_state = BUTTON_IDLE;
			return true;
		#endif
		sendMessage("    Start button sequence     ");
		buttonOrdinal = 1;
		button_state = BUTTON_SETUP;
		break;
	case BUTTON_SETUP:
		sendMessageNoDelay("    Press large button     ");
		if (bigRedButtonPressed)
		{
			thisWordNum = random(9)+1;
			thisWord = buttonWords[thisWordNum][random(4)];
			sendWord(thisWord);
			delay(500);
			countDownTimer = random(300)+300;  //tenths of seconds, set countdown timer to something between 30-60 sec
			button_state = BUTTON_CHECK;
			//sprintf(charToSend,"%d",countDownTimer);
			//sendWord(charToSend);
		}
		break;
	case BUTTON_CHECK:
		countDown();
		sprintf(charToSend,"0%d",countDownTimer);
		sendWord(charToSend);
		if (!bigRedButtonPressed)
		{
			if (countCheck(countDownTimer,thisWordNum))  // was button released at correct time?
			{
				sendMessage("    GOOD     ");
				if (buttonOrdinal == NUM_BUTTON_PRESSES)  //button section completed
				{
					button_state = BUTTON_IDLE;
					return true;
				}
				else
				{
					buttonOrdinal++;
					button_state = BUTTON_SETUP;
				}
			}
			else    // button released at incorrect time
			{
				sendMessage("    INCORRECT     ");
				button_state = BUTTON_INIT;
			}
		}
		break;
	}
	return false;
}

void getKeypad()
{
	char key = kpd.getKey();
	if(key)  // Check for a valid key.
	{
		displaybuffer[0] = displaybuffer[1];
		displaybuffer[1] = displaybuffer[2];
		displaybuffer[2] = displaybuffer[3];
		displaybuffer[3] = key;
		sendWord(displaybuffer);
		#ifdef DEBUG
			Serial.print("keypress = ");
			Serial.println(key);
		#endif
	}
}
	  
void displayTime()
{
	// int i;
	// unsigned long ul_CurrentMillis = millis();
	// if (ul_CurrentMillis - ul_PreviousMillis > ul_1secIntervalMillis) //if true, time to update timer
	//if ((timer_update == true) & (oneHzUpdate == true))
	// {
		// ul_PreviousMillis = ul_CurrentMillis;  //get the time (in milliseconds) for the next timeslot
	//	oneHzUpdate = false;
		// #ifdef DEBUG
			// Serial.print("Updating timer.  Time (millis) =  ");
			// Serial.println(millis());
			// Serial.print("timer_min = ");
			// Serial.println(timer_min);
			// Serial.print("timer_sec = ");
			// Serial.println(timer_sec);
		// #endif
		// if (timer_sec==0)
		// {
			// timer_sec = 59;
			// timer_min--;
		// }
		// else
		// {
			// timer_sec--;
		// }
		// if ((timer_min == 1) & (timer_sec == 0))  //For the last minute, go to ss.00 format
		// {
			// fastTime = true;
			// wTrig.trackPlayPoly(WARNING);
			// wTrig.trackLoop(WARNING, true);  // plays track WARNING continuously (loop enabled)
			// timer_min = 60;
		// }
		// int timestamp = 6000;
		// sendWord((char)timestamp);

	// }
}

void updateButtons()
{

	if (switchValues[RED_BUTTON_INDEX] == pressed)
	{
		#ifdef DEBUG
			Serial.println("red button pressed");
		#endif
		redButtonPressed = true;
		digitalWrite(RED_BUTTON_LED, HIGH);  //turn red button LED on
		direction = !direction;
	}
	else
	{
		digitalWrite(RED_BUTTON_LED, LOW);  //turn red button LED off
	}
	if (switchValues[BIG_RED_BUTTON_INDEX] == pressed)
	{
		#ifdef DEBUG
			Serial.println("big red button pressed");
		#endif
		bigRedButtonPressed = true;
		// digitalWrite(BIG_RED_BUTTON_LED, HIGH);
		// tone(PIEZO_SPKR, 1200);
	}
	else
	{
		digitalWrite(BIG_RED_BUTTON_LED, LOW);
		bigRedButtonPressed = false;
		//noTone(PIEZO_SPKR);
	}
	
	inputFlag = false;
}

void readSwitches()
{
	static unsigned long ts;
		
	switch(input_switch_state)
	{
	case INPUT_SWITCH_VALUE:
		switchValues[SWChannel] = digitalRead(switches[SWChannel]);
		if (switchValues[SWChannel] != switchValuesOld[SWChannel])   //check to see if switch changed value
		{
			ts = millis();
			input_switch_state = INPUT_SWITCH_DEBOUNCE;
		}
		else
		{
			SWChannel = (SWChannel + 1) % NUM_SWITCHES;
		}
	break;
	
	case INPUT_SWITCH_DEBOUNCE:
		switchValues[SWChannel] = digitalRead(switches[SWChannel]);
		if (switchValues[SWChannel] == switchValuesOld[SWChannel])   //check to see if switch value changed back
		{
			input_switch_state = INPUT_SWITCH_VALUE;
		}
		else if (millis() > ts + DEBOUNCE_TIME)
		{
			switchValuesOld[SWChannel] = switchValues[SWChannel];
			#ifdef DEBUG
				Serial.print("Switch");
				Serial.print(SWChannel);
				Serial.print("=");
				Serial.println(switchValues[SWChannel]);
			#endif
			//SWChannelOld = SWChannel;
			switchLatest = SWChannel;
			SWChannel = (SWChannel + 1) % NUM_SWITCHES;
			inputFlag = true;
		}
	break;
	}
}


void updateLEDs(byte *LEDs,byte *LEDvals, byte length)
{
	for (byte i=0; i<length; i++)
	{
		digitalWrite(LEDs[i],LEDvals[i]);
	}
}

void readADC()
{
	for (int i=0; i<NUM_ADC_CH; i++)
	{
		
		adcValuesOld[i] = adcValues[i];
		adcValues[i] = analogRead(adcPins[i]);
		#ifdef DEBUG
			Serial.print("ADC");
			Serial.print(i);
			Serial.print(" = ");
			Serial.println(adcValues[i]);
		#endif
		if (((adcValues[i] - adcValuesOld[i]) > ADC_DELTA) | ((adcValues[i] - adcValuesOld[i]) > ADC_DELTA))
		{
			;//do something if difference between reads is big enough
		}
	}
    
  
}

void alphaSetup()
{
	alpha4.begin(0x70);  // pass in the address

	alpha4.writeDigitRaw(3, 0x0);
	alpha4.writeDigitRaw(0, 0xFFFF);
	alpha4.writeDisplay();
	delay(100);
	alpha4.writeDigitRaw(0, 0x0);
	alpha4.writeDigitRaw(1, 0xFFFF);
	alpha4.writeDisplay();
	delay(100);
	alpha4.writeDigitRaw(1, 0x0);
	alpha4.writeDigitRaw(2, 0xFFFF);
	alpha4.writeDisplay();
	delay(100);
	alpha4.writeDigitRaw(2, 0x0);
	alpha4.writeDigitRaw(3, 0xFFFF);
	alpha4.writeDisplay();
	delay(100);
	  
	alpha4.clear();
	alpha4.writeDisplay();

	// display every character, 
	// for (uint8_t i='!'; i<='z'; i++)
	// {
		// alpha4.writeDigitAscii(0, i);
		// alpha4.writeDigitAscii(1, i+1);
		// alpha4.writeDigitAscii(2, i+2);
		// alpha4.writeDigitAscii(3, i+3);
		// alpha4.writeDisplay();
			
		// delay(100);
	// }
	
	// alpha4.clear();
	// alpha4.writeDisplay();
	//Serial.println("Start typing to display!");
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color((255 - WheelPos * 3), 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, (255 - WheelPos * 3));
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, (255 - WheelPos * 3), 0);
}

void updateNeoPixels(unsigned long ul_PixelIntervalMillis, byte pattern)
{
	uint8_t pixelSequenceLocal;
	unsigned long ul_CurrentMillis = millis();
	if ((ul_CurrentMillis - ul_PreviousPixelMillis > ul_PixelIntervalMillis))
	{
		ul_PreviousPixelMillis = ul_CurrentMillis;
		if (pattern==6)
		{
			pixelSequenceLocal = pattern;
		}
		else
		{
			pixelSequenceLocal = pixelSequence;
		}
		switch(pixelSequenceLocal)
		{
		case 0:
			colorWipe2(strip.Color(0,0,0), direction);  //off
			updateLED2(0, direction);
			break;
		case 1:
			colorWipe2(strip.Color(255,0,0), direction); // Red
			updateLED2(255, direction);
			break;
		case 2:
			colorWipe2(strip.Color(0,0,0), direction);  //off
			updateLED2(0, direction);
			break;
		case 3:
			colorWipe2(strip.Color(0,255,0), direction); // Green
			updateLED2(255, direction);
			break;
		case 4:
			colorWipe2(strip.Color(0,0,0), direction);  //off
			updateLED2(0, direction);
			break;
		case 5:
			colorWipe2(strip.Color(0,0,255), direction); // Blue
			updateLED2(255, direction);
			break;
		case 6:
			rainbowCycle2();
			break;
		}
	}
}

// Fill the dots one after the other with a color, doesn't use delay()
void colorWipe2(uint32_t c, boolean direction)
{
	// #ifdef DEBUG
		// Serial.print("pixelSequence = ");
		// Serial.println(pixelSequence);
		// Serial.print("currentPixel = ");
		// Serial.println(currentPixel);
	// #endif

	if (direction == true) //up=true, down=false
	{
		strip.setPixelColor(currentPixel, c);
	}
	else
	{
		strip.setPixelColor(NUM_PIXELS - 1 - currentPixel, c);
	}
	strip.show();
	currentPixel = (currentPixel+1) % NUM_PIXELS;
	if (currentPixel == 0)
	{
		pixelSequence = (pixelSequence+1) % 6;
	}
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle2()
{
	uint16_t i;
    for(i=0; i< NUM_PIXELS; i++)
	{
      strip.setPixelColor(i, Wheel(((i * 256 / NUM_PIXELS) + rainbowIndex) & 255));
    }
    strip.show();
    rainbowIndex = (rainbowIndex+1) % (256*5);
}

void updateLED2(uint8_t c, boolean direction)
{
	// #ifdef DEBUG
		// Serial.print("currentLED = ");
		// Serial.println(currentLED);
	// #endif

	if (direction == false) //up=false, down=true
	{
		LEDAvalues[currentLED] = c;
		LEDBvalues[currentLED] = c;
		digitalWrite(LEDApins[currentLED],c);
		digitalWrite(LEDBpins[currentLED],c);
	}
	else
	{
		LEDAvalues[NUM_LEDS - 1 - currentLED] = c;
		LEDBvalues[NUM_LEDS - 1 - currentLED] = c;
		digitalWrite(LEDApins[NUM_LEDS - 1 - currentLED],c);
		digitalWrite(LEDBpins[NUM_LEDS - 1 - currentLED],c);
	}
	currentLED = (currentLED+1) % NUM_LEDS;
}

void updateRedLED(unsigned long ul_LEDIntervalMillis)
{
	unsigned long ul_CurrentMillis = millis();
	
	if ((ul_CurrentMillis - ul_PreviousRedLEDMillis > ul_LEDIntervalMillis))
	{
		ul_PreviousRedLEDMillis = ul_CurrentMillis;
		digitalWrite(RED_BUTTON_LED, !digitalRead(RED_BUTTON_LED));  //toggle red button LED
	}
}

void updateWireLEDValues(int iter)
{
	uint8_t wireColorNum = wireSequenceArray[iter][COLOR_NUM];
	uint8_t mainLEDaMode = wireSequenceArray[iter][LEDA_MODE];
	uint8_t altLEDaMode = LEDmodeLUT[mainLEDaMode][wireColorNum];
	uint8_t mainLEDbMode = wireSequenceArray[iter][LEDB_MODE];
	uint8_t altLEDbMode = LEDmodeLUT[mainLEDbMode][wireColorNum];
	uint8_t randomNum;
	
	#ifdef DEBUG
		Serial.print("wireColorNum = ");
		Serial.println(wireColorNum);
		Serial.print("mainLEDaMode = ");
		Serial.println(mainLEDaMode);
		Serial.print("altLEDaMode = ");
		Serial.println(altLEDaMode);
		Serial.print("mainLEDbMode = ");
		Serial.println(mainLEDbMode);
		Serial.print("altLEDbMode = ");
		Serial.println(altLEDbMode);
	#endif
	for (int i=0; i<NUM_LEDS; i++)
	{
		if (i==wireSequenceArray[iter][LED_A])   // If this is the LED of interest, then set the mode appropriately
		{
			LEDAvalues[i] = mainLEDaMode;
		}
		else  // otherwire it's a distractor LED.  Set this LED mode to anything other than the main LED's mode or its alternate mode
		{
			do
			{
				randomNum = random(4);
			} while ((randomNum == mainLEDaMode) || (randomNum == altLEDaMode));
			LEDAvalues[i] = randomNum;
		}
		if (i==wireSequenceArray[iter][LED_B])   // If this is the LED of interest, then set the mode appropriately
		{
			LEDBvalues[i] = mainLEDbMode;
		}
		else  // otherwire it's a distractor LED.  Set this LED mode to anything other than the main LED's mode or its alternate mode
		{
			do
			{
				randomNum = random(4);
			} while ((randomNum == mainLEDbMode) || (randomNum == altLEDbMode));
			LEDBvalues[i] = randomNum;
		}
	}

}


void updateWireLEDs(unsigned long ul_LEDIntervalMillis)
{
	unsigned long ul_CurrentMillis = millis();

	if ((ul_CurrentMillis - ul_PreviousWireMillis > ul_LEDIntervalMillis))
	{
		ul_PreviousWireMillis = ul_CurrentMillis;
		everyOther = !everyOther;
		for (int i=0; i<NUM_LEDS; i++)
		{
			if (LEDAvalues[i] == OFF)
			{
				digitalWrite(LEDApins[i], LOW);  //off
			}
			else if (LEDAvalues[i] == ON)
			{
				digitalWrite(LEDApins[i], HIGH);  //on
			}
			else if (LEDAvalues[i] == SLOW_BLINKING)
			{
				if (everyOther)
				{
					digitalWrite(LEDApins[i], !digitalRead(LEDApins[i]));  //SLOW BLINK
				}
			}
			else
			{
				digitalWrite(LEDApins[i], !digitalRead(LEDApins[i]));  //FAST BLINK
			}
			if (LEDBvalues[i] == OFF)
			{
				digitalWrite(LEDBpins[i], LOW);  //off
			}
			else if (LEDBvalues[i] == ON)
			{
				digitalWrite(LEDBpins[i], HIGH);  //on
			}
			else if (LEDBvalues[i] == SLOW_BLINKING)
			{
				if (everyOther)
				{
					digitalWrite(LEDBpins[i], !digitalRead(LEDBpins[i]));  //SLOW BLINK
				}
			}
			else
			{
				digitalWrite(LEDBpins[i], !digitalRead(LEDBpins[i]));  //FAST BLINK
			}
		}
		
	}
}

void countDown()
{
	unsigned long ul_CurrentMillis = millis();
	if ((ul_CurrentMillis - ul_PreviousTimerMillis > ul_100msMillis))
	{
		ul_PreviousTimerMillis = ul_CurrentMillis;
		countDownTimer--;
	}
}

boolean countCheck(int num,int testDigit)	//compare each digit in num to testDigit
{											// if any of the digits in num = testDigit, return true
	int testnum;							// otherwise return false
	do
	{
		testnum = num % 10;
		#ifdef DEBUG
			Serial.print("num = ");
			Serial.println(num);
			Serial.print("testnum = ");
			Serial.println(testnum);
			Serial.print("testDigit = ");
			Serial.println(testDigit);
		#endif
		if (testnum == testDigit) {return true;}
		num = num/10;
	}
	while (num != 0);
	return false;
}


void waitForUSBkey()
{
	boolean USBkeyInserted = false;
	#ifdef SKIP_USB
		USBkeyInserted = true;
	#endif
	while (USBkeyInserted==false)
	{
		adcValueUSB_DP = analogRead(USB_DP);
		adcValueUSB_DM = analogRead(USB_DM);
		#ifdef DEBUG
			Serial.println(adcValueUSB_DP);
			Serial.println(adcValueUSB_DM);
		#endif
			
		if ((adcValueUSB_DP > USB_TARGET - USB_TARGET_MARGIN) && (adcValueUSB_DP < USB_TARGET + USB_TARGET_MARGIN) && (adcValueUSB_DM > USB_TARGET - USB_TARGET_MARGIN) && (adcValueUSB_DM < USB_TARGET + USB_TARGET_MARGIN))
		{
			USBkeyInserted = true;
		}
	}
}

void sendMessage(char *message)  //scrolls message to alphanumberic 4-character display
{
	for (uint8_t i=0; i<strlen(message)-4; i++)
	{
		alpha4.writeDigitAscii(0, message[i]);
		alpha4.writeDigitAscii(1, message[i+1]);
		alpha4.writeDigitAscii(2, message[i+2]);
		alpha4.writeDigitAscii(3, message[i+3]);
		alpha4.writeDisplay();
    
		delay(200);
	}
}

void sendMessageNoDelay(char *message)  //scrolls message to alphanumberic 4-character display
{
	unsigned long ul_CurrentMillis = millis();

	if ((ul_CurrentMillis - ul_PreviousMsgMillis > ul_MsgIntervalMillis))
	{
		ul_PreviousMsgMillis = ul_CurrentMillis;
		alpha4.writeDigitAscii(0, message[msgIter]);
		alpha4.writeDigitAscii(1, message[msgIter+1]);
		alpha4.writeDigitAscii(2, message[msgIter+2]);
		alpha4.writeDigitAscii(3, message[msgIter+3]);
		alpha4.writeDisplay();	
		
		msgIter = (msgIter+1) % (strlen(message)-4);
		
	}	

}

void sendWord(char *word)  //sends 4 characters to alphanumeric display
{
	alpha4.writeDigitAscii(0, word[0]);
	alpha4.writeDigitAscii(1, word[1]);
	alpha4.writeDigitAscii(2, word[2]);
	alpha4.writeDigitAscii(3, word[3]);
	alpha4.writeDisplay();
}

void sendChar(char c)  //sends 1 character to alphanumeric display (repeats 4X)
{
	alpha4.writeDigitAscii(0, c);
	alpha4.writeDigitAscii(1, c);
	alpha4.writeDigitAscii(2, c);
	alpha4.writeDigitAscii(3, c);
	alpha4.writeDisplay();
}
	
	
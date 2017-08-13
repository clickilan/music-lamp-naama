#define FASTLED_INTERRUPT_RETRY_COUNT 0
#include <Arduino.h>
#include <FastLED.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <string.h>
#include <CapacitiveSensor.h>
#include <EEPROM.h>
// #include <iostream.h>

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

/** BASIC CONFIGURATION  **/

//The amount of LEDs in the setup
#define NUM_LEDS 60
#define LED_TYPE WS2812B
//The pin that controls the LEDs
#define LED_PIN 6
//The pin that we read sensor values form
#define ANALOG_READ A0
//The input pin from the buttonmusic_light_naama
// #define BUTTON_PIN D2
#define RECIVER_PIN D1
#define SENDER_PIN D4
//Confirmed microphone low value, and max value
#define MIC_LOW 400.0
#define MIC_HIGH 450.0
/** Other macros */
//How many previous sensor values effects the operating average?
#define AVGLEN 5
//How many previous sensor values decides if we are on a peak/HIGH (e.g. in a song)
#define LONG_SECTOR 50

//Mneumonics
#define PEAK 3
#define NORMAL 2

//How long do we keep the "current average" sound, before restarting the measuring
#define MSECS 30 * 1000
#define CYCLES MSECS / DELAY

/*Sometimes readings are wrong or strange. How much is a reading allowed
to deviate from the average to not be discarded? **/
#define DEV_THRESH 0.8

// IMPORTANT: set here the max milli-Amps of your power supply 5V 2A = 2000
#define MILLI_AMPERE	2000

//Arduino loop delay
#define DELAY 1

//WiFi & MQTT details
const char* ssid = "winstonia";
const char* password = "kkkkhksjh";
const char* mqtt_server = "192.168.1.112";
int mqttState = 0;

//variables
int buttonPushCounter = 0;			// counter for the number of button presses
int buttonState = 0;						// current state of the button
int lastButtonState = 0;				// previous state of the button
int buttonTimer = millis();			// button press timer
int buttonLastTimer = millis();	// lsat button press
int MAX_READ = 400;
int MIN_READ = 350;
int addr = 0; // where to store data in EEPROM

// stating functions used later on

float fscale( float originalMin, float originalMax, float newBegin, float newEnd, float inputValue, float curve);
void insert(int val, int *avgs, int len);
int compute_average(int *avgs, int len);
void visualize_music();

// stating fastled effect fucntions
void rainbow();
void addGlitter();
void rainbowWithGlitter();
void juggle();
void juggle2();
void bpm();
uint8_t gHue = 0; // rotating "base colomoder" used by many of the patterns

//How many LEDs to we display
int curshow = NUM_LEDS;

/*Not really used yet. Thought to be able to switch between sound reactive
mode, and general gradient pulsing/static color*/
int mode;


//Showing different colors based on the mode.
int songmode = NORMAL;

//Average sound measurement the last CYCLES
unsigned long song_avg;

//The amount of iterations since the song_avg was reset
int iter = 0;

//The speed the LEDs fade to black if not relit
float fade_scale = 3; //1.2;

//Led array
CRGB leds[NUM_LEDS];

//Object for CapacitiveSensor
CapacitiveSensor cs_1_4 = CapacitiveSensor(RECIVER_PIN,SENDER_PIN); // 1M resistor between pins 2 & 4, pin 4 is sensor pin, add a wire and or foil

//Objects for Wifi and MQQT
WiFiClient espClient;
PubSubClient client(espClient);

/*Short sound avg used to "normalize" the input values.
We use the short average instead of using the sensor input directly */
int avgs[AVGLEN] = {-1};

//Longer sound avg
int long_avg[LONG_SECTOR] = {-1};

//Keeping track how often, and how long times we hit a certain mode
struct time_keeping {
	unsigned long times_start;
	short times;
};

//How much to increment or decrement each color every cycle
struct color {
	int r;
	int g;
	int b;
};

struct time_keeping high;
struct color Color;
struct color fadeColor;
struct color oldColor = { 0, 0, 0};

void callback(char* topic, byte* payload, unsigned int length) { //MQTT callback
  Serial.print("Message arrived [");
  Serial.print(topic);
  payload[length] = '\0';
  Serial.print("] ");
  char* message = (char*)payload;
  int msg = atoi(message);
  Serial.println();
  Serial.print(msg);
  // check payload and open/close relay
  if (strcmp(topic, "music_light_naama/mode") == 0) {
    if (payload[0] == 48) {
      mode = 1;
      Serial.println("Music mode!");
    }
    else if (payload[0] == 49) {
      mode = 0;
      Serial.println("some other mode");
    }
    else {
      Serial.println("unknown value");
    }
  }
	if (strcmp(topic, "music_light_naama/hue") == 0) {
		//make payload a char *
		char* hex = (char*)payload;
		// Get rid of '#' and convert it to integer
		int number = (int) strtol( &hex[1], NULL, 16);

		Color.r = number >> 16;
		Color.g = number >> 8 & 0xFF;
		Color.b = number & 0xFF;
		// Serial.println(Color.r);
		// Serial.println(Color.g);
		// Serial.println(Color.b);
	}
}

void setup() {
	Serial.begin(115200);
	Serial.println();
	Serial.println("Booting Sketch...");
	// pinMode(BUTTON_PIN, INPUT);
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	WiFi.setSleepMode(WIFI_NONE_SLEEP);
	while (WiFi.waitForConnectResult() != WL_CONNECTED) {
		Serial.println("Connection Failed! Retrying...");
		delay(5000);
		ESP.restart();
	}
	if (WiFi.status() == WL_CONNECTED) {
	WiFi.hostname("naama_light");
	}
	// WiFi.setSleepMode(WIFI_NONE_SLEEP);
	// setup OTA
	ArduinoOTA.onStart ([]() {
		Serial.println("Start");
	});
	ArduinoOTA.onEnd([]() {
		Serial.println("\nEnd");
	});
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
	});
		ArduinoOTA.onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
		else if (error == OTA_END_ERROR) Serial.println("End Failed");
	});
	ArduinoOTA.begin();
	Serial.println("Ready");
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());
	client.setServer(mqtt_server, 1883);
	client.setCallback(callback);
	//Set all lights to make sure all are working as expected

	//Capacitive sensor setup
	cs_1_4.set_CS_AutocaL_Millis(0xFFFFFFFF);// turn off autocalibrate on channel 1 - just as an example

	FastLED.addLeds<LED_TYPE, LED_PIN>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
	for (int i = 0; i < NUM_LEDS; i++)
		leds[i] = CRGB(0, 0, 255);
	FastLED.show();
	delay(1000);

	//bootstrap average with some low values
	for (int i = 0; i < AVGLEN; i++) {
		insert(250, avgs, AVGLEN);

	// EEPROM.begin(512);
	mode = (int) EEPROM.read(addr);
	Serial.println("mode is:");
	Serial.println(EEPROM.read(addr));
	}

	//Initial values
	high.times = 0;
	high.times_start = millis();
 	Color.r = 0;
	Color.g = 0;
	Color.b = 1;

	// importent maximum power for project!!!
	FastLED.setMaxPowerInVoltsAndMilliamps(5,MILLI_AMPERE);

}

void reconnect() {
  // loop untill connected
  while (!client.connected()) {
    Serial.print("Attampting MQTT connection...");
    // attempt to connect
    if (client.connect("music_light_naama", "ilangut", "powertool")) {
      Serial.println("connected");
      // once connected subscribe
      client.subscribe("wemos_mini/test");
      client.subscribe("wemos_mini/text");
    } else {
      Serial.print("failed to connect to MQTT, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

/*With this we can change the mode if we want to implement a general
lamp feature, with for instance general pulsing. Maybe if the
sound is low for a while? */
void loop() {
  ArduinoOTA.handle(); //Arduino OTA
	if (!client.connected()) {
    reconnect();
  }
  client.loop();
	long sensor1 = cs_1_4.capacitiveSensor(50);
	// Serial.println(sensor1);
	if(sensor1 >= 50) { // if capacitance is higher : 50 for 192.168.1.114
		buttonTimer = millis(); // time event
		if (buttonTimer - buttonLastTimer > 100) { // timer
			buttonState = 1;
			buttonLastTimer = buttonTimer; // set last touch time
		}
  }
  else {
    buttonState = 0;
	}
	// buttonState = digitalRead(BUTTON_PIN); // read the pushbutton input pin
  if (buttonState != lastButtonState) { // compare the buttonState to its previous state

    // if the state has changed, increment the counter
		if (buttonState == 1) {
      // if the current state is HIGH then the button
      // wend from off to on:
      mode++;
			if (mode == 3) {
				mode = 0;
			}
			EEPROM.write(addr, mode);
			Serial.println(EEPROM.read(addr));
		}
	}
	switch(mode) {
		case 0:
			visualize_music();
			break;
		default:
			break;
		case 1:
			juggle();
			FastLED.show();
			break;
		// case 2:
		// 	rainbowWithGlitter();
		// 	FastLED.show();
		// 	break;
		case 2:
			juggle2();
			FastLED.show();
			break;
		case 3: // blacken all leds
			for( int i = 0; i < NUM_LEDS; i++) {
				leds[i] = CRGB::Black;
			}
			fill_solid( leds, NUM_LEDS, CRGB::Black);
			FastLED.show();
			break;
		// case 5:
		// int step = 30;
		// color fadeColor = Color;
		// 	for( int j = 0; j < step; j++) {
		// 		fadeColor.r = (int) (oldColor.r + j*((Color.r - oldColor.r)/step));
		// 		fadeColor.g = (int) (oldColor.g + j*((Color.g - oldColor.g)/step));
		// 		fadeColor.b = (int) (oldColor.b + j*((Color.b - oldColor.b)/step));
		// 		for( int i = 0; i < NUM_LEDS; i++) {
		// 			// leds[i] = CRGB(r,g,b);
		// 			leds[i].setRGB(fadeColor.r, fadeColor.g, fadeColor.b);
		// 		}
		//
		// 	// fill_solid( leds, NUM_LEDS, CRGB(r,g,b));
		// 	// delay(25);
		// 	FastLED.delay(25);
		// 	FastLED.show();
		// }
		// 	oldColor = Color;
		// 	break;
	}
		delay(DELAY);       // delay in between reads for stability
	  lastButtonState = buttonState;
}

/**Funtion to check if the lamp should either enter a HIGH mode,
or revert to NORMAL if already in HIGH. If the sensors report values
that are higher than 1.1 times the average values, and this has happened
more than 30 times the last few milliseconds, it will enter HIGH mode.
TODO: Not very well written, remove hardcoded values, and make it more
reusable and configurable.  */
void check_high(int avg) {
	if (avg > (song_avg/iter * 1.1))  {
		if (high.times != 0) {
			if (millis() - high.times_start > 200.0) {
				high.times = 0;
				songmode = NORMAL;
			} else {
				high.times_start = millis();
				high.times++;
			}
		} else {
			high.times++;
			high.times_start = millis();

		}
	}
	if (high.times > 30 && millis() - high.times_start < 50.0)
		songmode = PEAK;
	else if (millis() - high.times_start > 200) {
		high.times = 0;
		songmode = NORMAL;
	}
}

//Main function for visualizing the sounds in the lamp
void visualize_music() {
	int sensor_value, mapped, avg, longavg;

	//Actual sensor value
	sensor_value = analogRead(ANALOG_READ);
	// if (sensor_value > MAX_READ) {
	// 	MAX_READ = sensor_value;
	// }
	// if (sensor_value < MIN_READ) {
	// 	MIN_READ = sensor_value;
	// }
	// Serial.println(MIN_READ);
	// Serial.println("current");
	// Serial.println(sensor_value);
	//If 0, discard immediately. Probably not right and save CPU.
	if (sensor_value == 0)
		return;

	//Discard readings that deviates too much from the past avg.
	mapped = (float)fscale(MIC_LOW, MIC_HIGH, MIC_LOW, (float)MIC_HIGH, (float)sensor_value, 2.0);
	avg = compute_average(avgs, AVGLEN);

	if (((avg - mapped) > avg*DEV_THRESH)) //|| ((avg - mapped) < -avg*DEV_THRESH))
		return;

	//Insert new avg. values
	insert(mapped, avgs, AVGLEN);
	insert(avg, long_avg, LONG_SECTOR);

	//Compute the "song average" sensor value
	song_avg += avg;
	iter++;
	if (iter > CYCLES) {
		song_avg = song_avg / iter;
		iter = 1;
	}

	longavg = compute_average(long_avg, LONG_SECTOR);

	//Check if we enter HIGH mode
	check_high(longavg);

	if (songmode == PEAK) {
		fade_scale = 3;
		Color.r = -5;
		Color.b = -5;
		Color.g = 5;
		// Color.r = 3;
		// Color.g = -1;
		// Color.b = 5;
	}
	else if (songmode == NORMAL) {
		fade_scale = 2;
		Color.r = 1;
		Color.b = 3;
		Color.g = 1;
		// Color.r = -5;
		// Color.b = -5;
		// 	Color.g = 5;
	}

	//Decides how many of the LEDs will be lit
	curshow = fscale(MIC_LOW, MIC_HIGH, 0.0, (float)NUM_LEDS, (float)avg, -1);
	//  Serial.println(curshow);
	/*Set the different leds. Control for too high and too low values.
          Fun thing to try: Dont account for overflow in one direction,
	  some interesting light effects appear! */
	for (int i = 0; i < NUM_LEDS; i++)
		//The leds we want to show
		if (i < curshow) {
			if (leds[i].r + Color.r > 255)
				leds[i].r = 255;
			else if (leds[i].r + Color.r < 0)
				leds[i].r = 0;
			else
				leds[i].r = leds[i].r + Color.r + i +10;

			if (leds[i].g + Color.g > 255)
				leds[i].g = 255;
			else if (leds[i].g + Color.g < 0)
				leds[i].g = 0;
			else
				leds[i].g = leds[i].g + Color.g - i;

			if (leds[i].b + Color.b > 255)
				leds[i].b = 255;
			else if (leds[i].b + Color.b < 0)
				leds[i].b = 0;
			else
				leds[i].b = leds[i].b + Color.b;

		//All the other LEDs begin their fading journey to eventual total darkness
		} else {
			leds[i] = CRGB(leds[i].r/fade_scale, leds[i].g/fade_scale, leds[i].b/fade_scale);
		}
	FastLED.show();
}
//Compute average of a int array, given the starting pointer and the length
int compute_average(int *avgs, int len) {
	int sum = 0;
	for (int i = 0; i < len; i++)
		sum += avgs[i];

	return (int)(sum / len);

}

//Insert a value into an array, and shift it down removing
//the first value if array already full
void insert(int val, int *avgs, int len) {
	for (int i = 0; i < len; i++) {
		if (avgs[i] == -1) {
			avgs[i] = val;
			return;
		}
	}

	for (int i = 1; i < len; i++) {
		avgs[i - 1] = avgs[i];
	}
	avgs[len - 1] = val;
}

void rainbow() {
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void addGlitter( fract8 chanceOfGlitter) {
  if( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void rainbowWithGlitter() { // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void juggle() { // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16(i+7,0,NUM_LEDS)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

void juggle2() { // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16(i+15,0,NUM_LEDS)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

void bpm() { // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

//Function imported from the arduino website.
//Basically map, but with a curve on the scale (can be non-uniform).
float fscale( float originalMin, float originalMax, float newBegin, float
		newEnd, float inputValue, float curve){

	float OriginalRange = 0;
	float NewRange = 0;
	float zeroRefCurVal = 0;
	float normalizedCurVal = 0;
	float rangedValue = 0;
	boolean invFlag = 0;


	// condition curve parameter
	// limit range

	if (curve > 10) curve = 10;
	if (curve < -10) curve = -10;

	curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
	curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

	// Check for out of range inputValues
	if (inputValue < originalMin) {
		inputValue = originalMin;
	}
	if (inputValue > originalMax) {
		inputValue = originalMax;
	}

	// Zero Refference the values
	OriginalRange = originalMax - originalMin;

	if (newEnd > newBegin){
		NewRange = newEnd - newBegin;
	}
	else
	{
		NewRange = newBegin - newEnd;
		invFlag = 1;
	}

	zeroRefCurVal = inputValue - originalMin;
	normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

	// Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
	if (originalMin > originalMax ) {
		return 0;
	}

	if (invFlag == 0){
		rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;

	}
	else     // invert the ranges
	{
		rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);
	}

	return rangedValue;
}

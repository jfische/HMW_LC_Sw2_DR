//*******************************************************************
//
// HMW_LC_Sw1_DR.cpp
//
// Homematic Wired Hombrew Hardware
// Arduino Uno als Homematic-Device
// Thorsten Pferdekaemper (thorsten@pferdekaemper.com)
// nach einer Vorlage von
// Dirk Hoffmann (hoffmann@vmd-jena.de)
//
//-------------------------------------------------------------------
//Hardwarebeschreibung:
// =====================
//
// Pinsettings for Arduino Uno
//
// D0: RXD, normaler Serieller Port fuer Debug-Zwecke
// D1: TXD, normaler Serieller Port fuer Debug-Zwecke
// D5: RXD, RO des RS485-Treiber
// D6: TXD, DI des RS485-Treiber
// D2: Direction (DE/-RE) Driver/Receiver Enable vom RS485-Treiber
//
// A0: Taster 1
// A1: Taster 2
// A2: Schaltausgang 1
// A3: Schaltausgang 2
// D13: oder D13 bei DEBUG_UNO Status-LED

// Die Firmware funktioniert mit dem Standard-Uno Bootloader, im
// Gegensatz zur Homematic-Firmware

//*******************************************************************

// Die "DEBUG_UNO"-Version ist fuer einen normalen Arduino Uno (oder so)
// gedacht. Dadurch wird RS485 ueber SoftwareSerial angesteuert
// und der Hardware-Serial-Port wird fuer Debug-Ausgaben benutzt
// Dadurch kann man die normale USB-Verbindung zum Programmieren und
// debuggen benutzen

#define DEBUG_NONE 0   // Kein Debug-Ausgang, RS485 auf Hardware-Serial
#define DEBUG_UNO 1    // Hardware-Serial ist Debug-Ausgang, RS485 per Soft auf pins 2/3
#define DEBUG_UNIV 2   // Hardware-Serial ist RS485, Debug per Soft auf pins 2/3

#define DEBUG_VERSION DEBUG_UNO

// Do not remove the include below
#include "HMW_LC_Sw2_DR.h"

#if DEBUG_VERSION == DEBUG_UNO || DEBUG_VERSION == DEBUG_UNIV
// TODO: Eigene SoftwareSerial
#include <SoftwareSerial.h>
#endif

// debug-Funktion
#include "HMWDebug.h"

// EEPROM
#include <EEPROM.h>

// HM Wired Protokoll
#include "HMWRS485.h"

// default module methods
#include "HMWModule.h"
#include "HMWRegister.h"


/********************************/
/* Pinbelegung: 				*/
/********************************/
#define BUTTON 8            // Das Bedienelement // Belegung wie "Universalsensor"
#define RS485_TXEN 2
#define I01 A0
#define I02 A1
#define O03 A2
#define O04 A3

#define IDENTIFY_LED 12         // Identify LED --> zusätzliche LED für Identifizierung in Verteilung
#define IDENTIFY_EEPROM 0xFF    // Adresse EEPROM Identify 

// Debugging...
#if DEBUG_VERSION == DEBUG_UNO
  #define RS485_RXD 5
  #define RS485_TXD 6
  #define LED 13        // Signal-LED
#elif DEBUG_VERSION == DEBUG_UNIV
  #define DEBUG_RXD 5
  #define DEBUG_TXD 6
  #define LED 4
#else
  #define LED 4         // Signal-LED
#endif

#define CHANNEL_IO_COUNT 4
#define CHANNEL_IO_BYTES 1  //CHANNEL_IO_COUNT / 8

// Das folgende Define kann benutzt werden, wenn ueber die
// Kanaele "geloopt" werden soll
// als Define, damit es zentral definiert werden kann, aber keinen (globalen) Speicherplatz braucht
#define CHANNEL_PORTS byte channelPorts[CHANNEL_IO_COUNT] = {I01, I02, O03, O04};

// Port Status, d.h. Port ist auf 0 oder 1
byte portStatus[CHANNEL_IO_BYTES];

// TODO: wird wirklich int gebraucht oder tut's auch byte?
unsigned int keyLongPressTime[2];
byte loggingTime;

#if DEBUG_VERSION == DEBUG_UNO
	SoftwareSerial rs485(RS485_RXD, RS485_TXD); // RX, TX
	HMWRS485 hmwrs485(&rs485, RS485_TXEN);
#elif DEBUG_VERSION == DEBUG_UNIV
	HMWRS485 hmwrs485(&Serial, RS485_TXEN);
#else
	HMWRS485 hmwrs485(&Serial, RS485_TXEN);  // keine Debug-Ausgaben
#endif

#if DEBUG_VERSION != DEBUG_NONE
void printChannelConf(){
  for(byte channel = 0; channel < CHANNEL_IO_COUNT; channel++) {
	  hmwdebug("Channel     : "); hmwdebug(channel); hmwdebug("\r\n");
	  hmwdebug("\r\n");
  }
}
#endif

HMWModule* hmwmodule;   // wird in setup initialisiert

// Config
hmw_config config;

// write config to EEPROM in a hopefully smart way
void writeConfig(){
//    byte* ptr;
//    byte data;
	// EEPROM lesen und schreiben
//	ptr = (byte*)(sensors);
//	for(int address = 0; address < sizeof(sensors[0]) * CHANNEL_IO_COUNT; address++){
//	  hmwmodule->writeEEPROM(address + 0x10, *ptr);
//	  ptr++;
//  };
};


// Read all inputs/outputs
// setzt Bits in portStatus[]
void readPins() {
  CHANNEL_PORTS
  for(byte i = 0; i < CHANNEL_IO_COUNT; i++){
	  // Pin lesen und Bit in portStatus setzen
	  // TODO: Check if this really works
	  bitWrite(portStatus[i/8],i%8,digitalRead(channelPorts[i]));
  }
}
// Klasse fuer Callbacks vom Protokoll
class HMWDevice : public HMWDeviceBase {
  public:
	void setLevel(byte channel,unsigned int level) {
	  // everything in the right limits?
	  if(channel != 2 && channel != 3) return;
      if(level > 255) return;
      // now set pin
      CHANNEL_PORTS
      if(level == 0xFF) {   // toggle
    	digitalWrite(channelPorts[channel], !digitalRead(channelPorts[channel]));
      }else if(level) // on
    	digitalWrite(channelPorts[channel], HIGH);
      else
    	digitalWrite(channelPorts[channel], LOW);
	}

	unsigned int getLevel(byte channel) {
      // everything in the right limits?
	  if(channel >= CHANNEL_IO_COUNT) return 0;
	  // read
	  CHANNEL_PORTS
	  if(digitalRead(channelPorts[channel])) return 0xC800;
	  else return 0;
	};

	void readConfig(){
	   byte* ptr;
	 // EEPROM lesen
	   ptr = (byte*)(&config);
	   for(unsigned long address = 0; address < sizeof(config); address++){
		 *ptr = EEPROM.read(address + 0x01);
		 ptr++;
	   };
	// defaults setzen, falls nicht sowieso klar
	   if(config.logging_time == 0xFF) config.logging_time = 20;
	   if(config.central_address == 0xFFFFFFFF) config.central_address = 0x00000001;
	   for(byte channel = 0; channel < HMW_CONFIG_NUM_KEYS; channel++){
		   if(config.keys[channel].long_press_time == 0xFF) config.keys[channel].long_press_time = 10;
	   };
	};

};

HMWDevice hmwdevice;


//The setup function is called once at startup of the sketch
void setup()
{
	#if DEBUG_VERSION == DEBUG_UNO
		pinMode(RS485_RXD, INPUT);
		pinMode(RS485_TXD, OUTPUT);
	#elif DEBUG_VERSION == DEBUG_UNIV
		pinMode(DEBUG_RXD, INPUT);
		pinMode(DEBUG_TXD, OUTPUT);
	#endif
		pinMode(RS485_TXEN, OUTPUT);
		digitalWrite(RS485_TXEN, LOW);


	#if DEBUG_VERSION == DEBUG_UNO
		hmwdebugstream = &Serial;
		Serial.begin(19200);
		rs485.begin(19200);    // RS485 via SoftwareSerial
	#elif DEBUG_VERSION == DEBUG_UNIV
		SoftwareSerial* debugSerial = new SoftwareSerial(DEBUG_RXD, DEBUG_TXD);
		debugSerial->begin(19200);
		hmwdebugstream = debugSerial;
		Serial.begin(19200, SERIAL_8E1);
	#else
		Serial.begin(19200, SERIAL_8E1);
	#endif

	// Input Pins arbeiten mit PULLUP, d.h. muessen per Taster
	// auf Masse gezogen werden
		pinMode(I01,INPUT_PULLUP);
		pinMode(I02,INPUT_PULLUP);
		pinMode(O03,OUTPUT);
		digitalWrite(O03,LOW);
		pinMode(O04,OUTPUT);
		digitalWrite(O04,LOW);

    pinMode(BUTTON, INPUT_PULLUP);
    pinMode(LED, OUTPUT);

  //-------------------------------------------------------------

  pinMode(IDENTIFY_LED, OUTPUT);      // Identify LED

  static long last_IDENT_LED_time;
  long now = millis();
  last_IDENT_LED_time = now;
  // Identity Status und EEPROM lesen 
  static byte IDENTY;
  IDENTY = EEPROM.read(IDENTIFY_EEPROM);

  if (IDENTY == 0x00) {
     digitalWrite(IDENTIFY_LED, LOW);
     last_IDENT_LED_time = now;
     } 
  else {
     if(now - last_IDENT_LED_time > 600) {        // schnelles Blinken
      digitalWrite(IDENTIFY_LED,!digitalRead(IDENTIFY_LED));  //Toggel LED
      last_IDENT_LED_time = now;
      }
   }
  //-------------------------------------------------------------

	// config aus EEPROM lesen
	hmwdevice.readConfig();
	// device type: 0x11 = HMW-LC-Sw2-DR
	// serial number
	// address
	// Modultyp irgendwo als define
	hmwmodule = new HMWModule(&hmwdevice, &hmwrs485, 0x11);
// "HHB2703111", 0x42380123);

	hmwmodule->broadcastAnnounce(0);

   hmwdebug("huhu\n");
	#if DEBUG_VERSION != DEBUG_NONE
		printChannelConf();
	#endif
}

//------------------------------------------------------------
// Hinzugefügt für initialisierung

void factoryReset() {
  // writes FF into config
//  memset(sensors, 0xFF, sizeof(sensors[0]) * CHANNEL_IO_COUNT);
  // set defaults
//  setDefaults();
}


void handleButton() {
  // langer Tastendruck (5s) -> LED blinkt hektisch
  // dann innerhalb 10s langer Tastendruck (3s) -> LED geht aus, EEPROM-Reset

//-----------------------------------------------------------------------
  // Erweiterung für Identity LED
  // Identity Status und EEPROM Adresse 255 lesen 
  static byte IDENTY;
  static long last_IDENT_LED_time;
  long now = millis();

  IDENTY = EEPROM.read(IDENTIFY_EEPROM);      //EEPROM Adresse 255 lesen 

  if (IDENTY == 0x00) {
     digitalWrite(IDENTIFY_LED, LOW);
     last_IDENT_LED_time = now;
     } 
  else {
     if(now - last_IDENT_LED_time > 600) { 
      digitalWrite(IDENTIFY_LED,!digitalRead(IDENTIFY_LED));  //Toggel LED
      last_IDENT_LED_time = now;                              // Zähler zurücksetzen
      }
   }

//-----------------------------------------------------------------------


  static long lastTime = 0;
  static byte status = 0;  // 0: normal, 1: Taste erstes mal gedrückt, 2: erster langer Druck erkannt
                           // 3: Warte auf zweiten Tastendruck, 4: Taste zweites Mal gedrückt
                           // 5: zweiter langer Druck erkannt

//  long now = millis();
  boolean buttonState = !digitalRead(BUTTON);


  switch(status) {
    case 0:
      if(buttonState) {status = 1;  hmwdebug(status);}
      lastTime = now;
      break;
    case 1:
      if(buttonState) {   // immer noch gedrueckt
        if(now - lastTime > 5000) {status = 2;   hmwdebug(status);}
      }else{              // nicht mehr gedrückt
      if(now - lastTime > 100)   // determine sensors and send announce on short press
        hmwmodule->broadcastAnnounce(0);
        status = 0;
        hmwdebug(status);
      };
      break;
    case 2:
      if(!buttonState) {  // losgelassen
      status = 3;
      hmwdebug(status);
      lastTime = now;
      };
      break;
    case 3:
      // wait at least 100ms
      if(now - lastTime < 100)
      break;
      if(buttonState) {   // zweiter Tastendruck
      status = 4;
      hmwdebug(status);
      lastTime = now;
      }else{              // noch nicht gedrueckt
      if(now - lastTime > 10000) {status = 0;   hmwdebug(status);}   // give up
      };
      break;
    case 4:
      if(now - lastTime < 100) // entprellen
            break;
      if(buttonState) {   // immer noch gedrueckt
        if(now - lastTime > 3000) {status = 5;  hmwdebug(status);}
      }else{              // nicht mehr gedrückt
        status = 0;
        hmwdebug(status);
      };
      break;
    case 5:   // zweiter Druck erkannt
      if(!buttonState) {    //erst wenn losgelassen
      // Factory-Reset          !!!!!!  TODO: Gehoert das ins Modul?
      factoryReset();
      hmwmodule->setNewId();
      status = 0;
      hmwdebug(status);
      }
      break;
  }

  // control LED
  static long lastLEDtime = 0;
  switch(status) {
    case 0:
      digitalWrite(LED, LOW);
      break;
    case 1:
      digitalWrite(LED, HIGH);
      break;
    case 2:
    case 3:
    case 4:
      if(now - lastLEDtime > 100) {  // schnelles Blinken
      digitalWrite(LED,!digitalRead(LED));
      lastLEDtime = now;
      };
      break;
    case 5:
      if(now - lastLEDtime > 750) {  // langsames Blinken
        digitalWrite(LED,!digitalRead(LED));
        lastLEDtime = now;
      };
  }
};

//--------------------------------------------------------

// Tasten
void handleKeys() {
// TODO: Vielleicht besser eine Klasse HMWKey oder so anlegen
  // millis() zum Zeitpunkt eines Tastendrucks
  // verwendet zum Entprellen, lange Tastendruecke und wiederholtes Senden langer Tastendruecke
  static unsigned long keyPressedMillis[2] = {0,0};   // Wir haben zwei Inputs
  static byte keyPressNum[2] = {0,0};
  // wann wurde das letzte mal "short" gesendet?
  static unsigned long lastSentLong[2] = {0,0};

  long now = millis();

  for(byte i = 0; i < 2; i++){
// INPUT_LOCKED?
   if(!config.keys[i].input_locked) continue;   // inverted logic, locked = 0
// Taste nicht gedrueckt (negative Logik wegen INPUT_PULLUP)
   if(bitRead(portStatus[i/8],i%8)){
	 // Taste war auch vorher nicht gedrueckt kann ignoriert werden
     // Taste war vorher gedrueckt?
	 if(keyPressedMillis[i]){
	   // entprellen, nur senden, wenn laenger als 50ms gedrueckt
	   // aber noch kein "long" gesendet
	   if(now - keyPressedMillis[i] >= 50 && !lastSentLong[i]){
	     keyPressNum[i]++;
// TODO: muss das eigentlich an die Zentrale gehen?
	     hmwmodule->broadcastKeyEvent(i,keyPressNum[i]);
	     // gleich ein Announce hinterher
	     // TODO: Vielleicht gehoert das in den allgemeinen Teil
	     hmwmodule->broadcastAnnounce(i);
	   };
	   keyPressedMillis[i] = 0;
	 };
   }else{
// Taste gedrueckt
	 // Taste war vorher schon gedrueckt
	 if(keyPressedMillis[i]){
       // muessen wir ein "long" senden?
	   if(lastSentLong[i]) {   // schon ein LONG gesendet
		  if(now - lastSentLong[i] >= 300){  // alle 300ms wiederholen
			// keyPressNum nicht erhoehen
			lastSentLong[i] = now ? now : 1; // der Teufel ist ein Eichhoernchen
			// TODO: muss das eigentlich an die Zentrale gehen?
			hmwmodule->broadcastKeyEvent(i,keyPressNum[i], true);
		  };
	   }else if(millis() - keyPressedMillis[i] >= long(config.keys[i].long_press_time) * 100) {
		  // erstes LONG
		  keyPressNum[i]++;
	      lastSentLong[i] = millis();
		  // TODO: muss das eigentlich an die Zentrale gehen?
		  hmwmodule->broadcastKeyEvent(i,keyPressNum[i], true);
	      // gleich ein Announce hinterher
		  // TODO: Vielleicht gehoert das in den allgemeinen Teil
		  hmwmodule->broadcastAnnounce(i);
	   };
	 }else{
	   // Taste war vorher nicht gedrueckt
	   keyPressedMillis[i] = now ? now : 1; // der Teufel ist ein Eichhoernchen
	   lastSentLong[i] = 0;
	 }
   }
  }
}


// The loop function is called in an endless loop
void loop()
{
// Daten empfangen (tut nichts, wenn keine Daten vorhanden)
	hmwrs485.loop();

// Check Keys
// Hier werden alle Ein-/Ausgaenge gelesen
// Pins lesen und nach portStatus[] schreiben
  readPins();

// Tasten abfragen, entprellen etc.
  handleKeys();

  // Bedienung ueber Button für Anlernen und Factory Reset
   handleButton();


  // hmwTxTargetAdress(4)                   the target adress
  // hmwTxFrameControllByte                 the controll byte
  // hmwTxSenderAdress(4)                   the sender adress
  // hmwTxFrameDataLength                   the length of data to send
  // hmwTxFrameData(MAX_RX_FRAME_LENGTH)    the data array to send
}







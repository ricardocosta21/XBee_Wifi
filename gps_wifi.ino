#include <avr\pgmspace.h>
#include <avr/eeprom.h>

//Begin Adafruit_GPS Stuff
    #include <Adafruit_GPS.h>
    #include <SoftwareSerial.h>
    
    SoftwareSerial mySerial(A4,A5);
    Adafruit_GPS GPS(&mySerial);
    
    #define GPSECHO true
    
    boolean usingInterrupt = true;
    void useInterrupt(boolean);
    uint32_t timer = millis();
//End Adafruit_GPS Stuff

//E-PAPER setup
   #include <ePaper.h>  // This file includes defines for each displayable character
   
   const int EIOpin = 2;     // Input/output pin for chip selection
   const int XCKpin = 3;     // Clock input pin for taking display data
   const int LATCHpin = 4;   // Latch pulse input pin for display data
   const int SLEEPBpin = 5;  // Sleep Pin for the display
   const int DI0pin = 6;     // Input pin for display data

   //setup display with pin definitions
   ePaper epaper = ePaper(EIOpin, XCKpin, LATCHpin, SLEEPBpin, DI0pin);
//end E-PAPER  */

//XBEE Wi-Fi set-up and global variables
   #include <XbeeWifi.h>

   // These are the pins that we are using to connect to the Xbee  
   #define XBEE_RESET 7
   #define XBEE_DOUT 8
   #define XBEE_ATN 9
   #define XBEE_SELECT SS

   // These are the network configuration parameters we're going to use
   #define CONFIG_ENCMODE XBEE_SEC_ENCTYPE_WPA2     // Network type is WPA2 encrypted
   #define CONFIG_SSID "Gum-DropB(EARS)"                   // SSID
   #define CONFIG_KEY "Candycane"                  // Password
   #define TxIP {192,168,0,101}
   #define Gateway "192.168.0.1"
   #define MyIP "192.168.0.222"

   // Create an xbee object to handle things for us
   XbeeWifi xbee;
//end XBEE


// Allow us to embed some PROGMEM strings inline so we don't run out of memory
   class __FlashStringHelper;
   #define F(str) reinterpret_cast<__FlashStringHelper *>(PSTR(str))

void setup(){
  // Serial at 57600
  Serial.begin(57600);
  GPS.begin(9600);
[b]//epaper.writeTop("Start Boot");   //   <--This line of code when un commented kills all functionality on the 'duino[/b]
epaper.writeDisplay();
  useInterrupt(false);
  pinMode(A0, INPUT);
  unsigned long age = 0;
  Serial.println("Booting Start:");
  
// Initialize the xbee
  bool result = xbee.init(XBEE_SELECT, XBEE_ATN, XBEE_RESET, XBEE_DOUT);
  if(!result){Serial.println("Failed @ 0");}
  if (result) {
    // Initialization okay so far, send setup parameters - if anything fails, result goes false
    result &= xbee.at_cmd_byte(XBEE_AT_NET_TYPE, XBEE_NET_TYPE_IBSS_INFRASTRUCTURE);
  if(!result){Serial.println("Failed @ 1");}
    result &= xbee.at_cmd_str(XBEE_AT_NET_SSID, CONFIG_SSID);
  if(!result){Serial.println("Failed @ 2");}
    result &= xbee.at_cmd_byte(XBEE_AT_NET_ADDRMODE, XBEE_NET_ADDRMODE_STATIC);
  if(!result){Serial.println("Failed @ 3");}
    result &= xbee.at_cmd_str(XBEE_AT_ADDR_IPADDR, MyIP);
  if(!result){Serial.println("Failed @ 4");}
    result &= xbee.at_cmd_str(XBEE_AT_ADDR_GATEWAY, Gateway);
  if(!result){Serial.println("Failed @ 5");}
    result &= xbee.at_cmd_byte(XBEE_AT_SEC_ENCTYPE, CONFIG_ENCMODE);
  if(!result){Serial.println("Failed @ 6");}
    if (CONFIG_ENCMODE != XBEE_SEC_ENCTYPE_NONE) {
      result &= xbee.at_cmd_str(XBEE_AT_SEC_KEY, CONFIG_KEY);
  if(!result){Serial.println("Failed @ 7");}
    }
  }
  if (!result) {
    // Something failed
    Serial.println(F("XBee Init Failed"));
 //   epaper.writeTop("XB Failed ");
    epaper.writeDisplay();
    while (true) { /* Loop forever - game over */}
  } else {    
    Serial.println("XB Success");
 //   epaper.writeTop("XB Init   ");
    epaper.writeDisplay();
  }  
//end XBEE init 

}//end setup

int Record = 0;

void loop(){

  // Just keep calling the process method on the xbee object
  xbee.process();

//Included code for Adafruit GPS example
  unsigned long age = 0;
  bool newdata = false;
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
 if(GPS.newNMEAreceived()){
    if(!GPS.parse(GPS.lastNMEA()))
      return;
//end adafruit example code
     
//wait 5 seconds
    if(millis() - timer > 5000)
    {
        timer = millis();
//Print data to Serial to computer
      Serial.print(GPS.hour, DEC); Serial.print(':');
      Serial.print(GPS.minute, DEC); Serial.print(':');
      Serial.print(GPS.seconds, DEC); Serial.print('.');
      Serial.println(GPS.milliseconds);
      Serial.print("Fix: "); Serial.println((int)GPS.fix);
      if (GPS.fix) {
        Serial.print("Location: ");
        Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
        Serial.print(", "); 
        Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
        
        Serial.print("Speed (knots): "); Serial.println(GPS.speed);
        Serial.print("Altitude: "); Serial.println(GPS.altitude);
        if(Record < 512){
//record data to EEPROM
          float time = 10000 * GPS.hour + 100 * GPS.minute + GPS.seconds;
          eeprom_write_block((void*)&time, (void*)((Record*20)), sizeof(time));
          eeprom_write_block((void*)&GPS.latitude, (void*)((Record*20)-12), sizeof(GPS.latitude));
          eeprom_write_block((void*)&GPS.longitude, (void*)((Record*20)-8), sizeof(GPS.longitude));
          eeprom_write_block((void*)&GPS.altitude, (void*)((Record*20)-4), sizeof(GPS.altitude));
          Serial.print("Records Saved: ");
          Serial.println(Record + 1);
          Record++;
        }
      }
    }
  }

//external trigger (just using a pin to connect to Vcc and GND as needed to test this part)
  if(digitalRead(A0))
  {
    //Join network
    if (xbee.last_status != XBEE_MODEM_STATUS_JOINED) {
      Serial.println(F("Not yet up and running"));
 //     epaper.writeTop("Connecting");
 //     epaper.writeBottom("To Network");
      epaper.writeDisplay();
    } else {
      Serial.println(F("Transmitting now"));
 //     epaper.writeTop(" Transmit ");
 //     epaper.writeBottom("          ");
      epaper.writeDisplay();
      
      // Create an s_txoptions object to describe the port, protocol and behaviors
      s_txoptions txopts;
      txopts.dest_port=12345;
      txopts.source_port=12345;
      txopts.protocol = XBEE_NET_IPPROTO_TCP;

      // Create a binary IP address representation
      char ip[] = TxIP;

      // Transmit the frame on  network with xbee
      if (!xbee.transmit((uint8_t *)ip, &txopts, (uint8_t *)"HelloWorld!\n", 12)) {
        Serial.println(F("Transmit failed"));
 //       epaper.writeTop(" Transmit ");
 //       epaper.writeBottom("  Failed  ");
        epaper.writeDisplay();
      } else {
        Serial.println(F("Transmit OK"));    
//        epaper.writeTop(" Transmit ");
 //       epaper.writeBottom(" Success  ");
        epaper.writeDisplay();
      }
    }
  }
}  

//****Included with the Adafruit GPS example
      // Interrupt is called once a millisecond, looks for any new GPS data, and stores it
      SIGNAL(TIMER0_COMPA_vect) {
        char c = GPS.read();
        // if you want to debug, this is a good time to do it!
      #ifdef UDR0
        if (GPSECHO)
         if (c) UDR0 = c;  
         // writing direct to UDR0 is much much faster than Serial.print 
         // but only one character can be written at a time. 
      #endif
      }

      void useInterrupt(boolean v) {
        if (v) {
         // Timer0 is already used for millis() - we'll just interrupt somewhere
         // in the middle and call the "Compare A" function above
         OCR0A = 0xAF;
         TIMSK0 |= _BV(OCIE0A);
         usingInterrupt = true;
        } else {
         // do not call the interrupt function COMPA anymore
         TIMSK0 &= ~_BV(OCIE0A);
         usingInterrupt = false;
        }
      }

//============================================================================
// Copyright (c) 2021-2022, Airborne Innovations, LLC.  All rights reserved
//============================================================================

// Relays data from serial to ethernet:
//
// Takes serial data and sends it over ethernet:
//     - UDP Multicast
//     or
//     - Standard UDP

// Notes on multicast:  The SEND_MULTICAST_UDP method does work. However
// Microhard p(x)DDL's aren't happy with it.  It turns out the best thing for
// multicast is just to send a standard UDP packet but to a multicast address.
// So that's what we're doing here.

//Enable one of the following:
//#define SEND_MULTICAST_UDP
// Note one acceptable way to send multicast is to send standard udp using the multicast IP address.
#define SEND_STANDARD_UDP

#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SERCOM.h>
#include "wiring_private.h" // pinPeripheral()
#include "SPI.h"            // SPIClass

#define AIDBG3 ()


#ifdef UDP_TX_PACKET_MAX_SIZE
#undef UDP_TX_PACKET_MAX_SIZE // Originally defined in arduino-1.8.9/hardware/teensy/avr/libraries/Ethernet/src/Ethernet.h:150
#define UDP_TX_PACKET_MAX_SIZE 860 //increase UDP size
#endif

// Timeout after which to send partial UDP if serial data stops coming
#define UDP_SEND_TIMEOUT_MILLISEC 50

//Enable to print incoming packet info
//#define DEBUG_UDPPACKET
//Enable to print incoming packet info byte by byte
//#define DEBUG_UDP_PKT
//Enable to print short packet info
#define DEBUG_UDP_SHORT

// Enable to enable LED pin. Note this conflicts with SCK for the ethernet chip at the moment.
//#define LED_ENABLED

//------------------------------------------------
//                   GLOBALS
//------------------------------------------------
// Teensy 3.x / Teensy LC have the LED on pin 13
//const int ledPin = 13;
// SAME51 LED on pin D11
const int ledPin = 11;
int led = 1;
int tlast = 0;

#define PIN_SERIAL_TCOM1_RX A3
#define PAD_SERIAL_TCOM1_RX (SERCOM_RX_PAD_1)
#define PIN_SERIAL_TCOM1_TX A2          // Equivalent to Pin A2
#define PAD_SERIAL_TCOM1_TX (UART_TX_PAD_0)

#define PIN_SERIAL_TCOM2_RX 22
#define PAD_SERIAL_TCOM2_RX (SERCOM_RX_PAD_1)
#define PIN_SERIAL_TCOM2_TX 21          // Equivalent to Pin A12
#define PAD_SERIAL_TCOM2_TX (UART_TX_PAD_0)

Uart SerialTCOM1( &sercom4, PIN_SERIAL_TCOM1_RX, PIN_SERIAL_TCOM1_TX, PAD_SERIAL_TCOM1_RX, PAD_SERIAL_TCOM1_TX );
// From ~/.arduino15/packages/adafruit/hardware/samd/1.6.3/cores/arduino/Uart.h
//     Uart(SERCOM *_s, uint8_t _pinRX, uint8_t _pinTX, SercomRXPad _padRX, SercomUartTXPad _padTX);
//Uart SerialTCOM2(&sercom2, 22, 21, SERCOM_RX_PAD_1, UART_TX_PAD_0);
Uart SerialTCOM2(&sercom2, PIN_SERIAL_TCOM2_RX, PIN_SERIAL_TCOM2_TX, PAD_SERIAL_TCOM2_RX, PAD_SERIAL_TCOM2_TX);


// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

// Device IP addresses
IPAddress ip(192, 168, 168, 177);
IPAddress myDns(192, 168, 168, 1);
IPAddress gateway(192, 168, 168, 1);
IPAddress subnet(255, 255, 255, 0);

// Destination Multicast IP address
IPAddress multicastip(224,1,1,7);
unsigned int localPort       = 20001; // local port to listen on
// Destination standard UDP address
IPAddress destip=(192,168,168,222);
unsigned int destinationPort = 20002; // send port for standard UDP


// buffers for receiving and sending data
uint8_t packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
//char packetBuffer[255];  // buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged-Multicast";        // a string to send back
char DebugString[] = "2022.08.09_1234";

char SerialBuffer[UDP_TX_PACKET_MAX_SIZE];

char RelayBuffer[UDP_TX_PACKET_MAX_SIZE];

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;


// From ~/.arduino15/packages/arduino/hardware/samd/1.8.9/libraries/SPI/SPI.h
//     SPIClass(SERCOM *p_sercom, uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI, SercomSpiTXPad, SercomRXPad);
//SPIClass SPI2(&sercom0, 11, 10, 9, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_2);

// Board-specific changes needed in:
// ~/.arduino15/packages/adafruit/hardware/samd/1.7.2/variants/xxxx/variant.h

// Ethernet debugging can be enabled here:
// arduino-1.8.9/libraries/Ethernet/src/utility/w5100.cpp

//------------------------------------------------
//               end GLOBALS
//------------------------------------------------


//------------------------------------------------
//            FORWARD DECLARATIONS
//------------------------------------------------
void ProcInitialLEDState();
void SetLED( int val);
void ToggleLED();
//static void initSerial2(void);
static void rainbow(int wait);

int tlastserial;

void setup() {

  // WIZNET_RESET  (pin 38, PB23)
  // Is active low, it has a 10k pullup to 3.3V
  pinMode(23, OUTPUT);
  digitalWrite(23, LOW);  // Active low, so low is in reset
  delay(500);
  digitalWrite(23, HIGH); // Active low, so low is in reset
  delay(500);
  pinMode(23, INPUT);
  delay(500);
 
  // Configure the W5500's Chip Select (CS) pin:
  //Ethernet.init(15);  // Airborne Gimbal Connector Board
  Ethernet.init(3);     // Multicomm board with SAMD51/SAME51

  Serial1.begin(115200);
//  initSerial2(); // Specific to SAMD51/SAME51

  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  delay(500);

  ai_SAMx51_init_SerialTCOM1();
  ai_SAMx51_init_SerialTCOM2();

  Serial.printf("Airborne Innovations\n");
  Serial.printf("UDP Serial Ethernet Relay v1.0\n");
  delay(500);

  // start the Ethernet
  Ethernet.begin(mac, ip);
//  Ethernet.begin(mac, ip, myDns, gateway, subnet);

  delay(200);

  Serial.printf("%s\n", DebugString);
  pinMode(ledPin, OUTPUT);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.printf("Ethernet interface was not found.  Sorry, can't run without hardware. :(");

      digitalWrite(ledPin, HIGH);   // set the LED on
      while (true) {
          delay(500); // do nothing, no point running without Ethernet hardware
          ToggleLED();
      }
  }

  while (Ethernet.linkStatus() == LinkOFF) {
      Serial.printf("Ethernet cable is not connected.\n");
      ToggleLED();
      delay(200);
  }

  #ifdef SEND_MULTICAST_UDP
    Serial.println("Begin multicast UDP session\n");
    Udp.beginMulticast(multicastip, destinationPort);
  #endif

  #ifdef SEND_STANDARD_UDP
    Serial.println("Begin UDP session\n");
    Udp.begin(localPort);
  #endif
  
} //setup()


// Globals
int cnt = 0;
int i,j = 0;
int pktReady  = 0;
int LEDState  = 0;
int NoDataYet = 1;
int counttarget=500;
uint8_t curState = 0;
int     curIdx   = 0;

int pktlen = 0;

int lastms=0;

void loop() 
{


    if (millis()>lastms+3000)
    {
      Serial.println("This is a test of USB Serial");
      Serial1.println("This is a test of ExtSerial");
      SerialTCOM1.println("This is a test of SerialTCOM1");
      SerialTCOM2.println("This is a test of SerialTCOM2");
      lastms = millis();

      #ifdef DEBUG_UDP_SHORT
         Serial.printf("Send udp/mcast dest port=%d\n",destinationPort);
      #endif

      #ifdef SEND_MULTICAST_UDP
        Udp.beginPacket(multicastip, destinationPort);
        Udp.write(RelayBuffer, pktlen);
        Udp.endPacket();
      #endif
      
      #ifdef SEND_STANDARD_UDP
//      Udp.begin(localPort);
        pktlen = 1;
        Udp.beginPacket(destip, destinationPort);
        RelayBuffer[0]=0x55;
//        Udp.write(RelayBuffer,pktlen);
        Udp.write(ReplyBuffer);
        Udp.endPacket();
      #endif 
      
    }
    
    if (Serial1.available()) 
    {
        char inputSerial = Serial1.read();
        tlastserial =  millis();
        NoDataYet = 0;
        if (pktlen < 30)
        {
          SerialBuffer[pktlen] = inputSerial;
          pktlen++;
        }
          else
            pktReady = 1;

        Serial.printf("[0x%x]\n",inputSerial);
        if (pktReady == 1) 
          {
            // Send a UDP packet

            // prepare UDP packet
            // Write size bytes from buffer into the packet
            for (j=0; j<pktlen; ++j) 
            {
                RelayBuffer[j] = SerialBuffer[j];  // In another code we are intercepting packets in one form and transmitting different UDP packets. Otherwise this copy would be unnecessary.
            }

            #ifdef DEBUG_UDP_PKT
              for (j=0; j<pktlen; ++j) 
              {
                  Serial.printf("byte %d [0x%x]\n",j,RelayBuffer[j]);
              }
            #endif

            #ifdef SEND_MULTICAST_UDP
              Udp.beginPacket(multicastip, destinationPort);
              Udp.write(RelayBuffer, pktlen);
              Udp.endPacket();
              Serial.printf("Send multicast UDP");
            #endif //SEND_MULTICAST_UDP

            #ifdef SEND_STANDARD_UDP
               #ifdef DEBUG_UDP_SHORT
                  Serial.printf("Send udp/mcast dest port=%d\n",destinationPort);
               #endif
              Udp.begin(localPort);
              Udp.beginPacket(destip, destinationPort);
              Udp.write(RelayBuffer,pktlen);
              Udp.endPacket();
            #endif //SEND_STANDARD_UDP

         pktlen = 0;
         pktReady = 0;
       } // if pktReady

    } else if (NoDataYet == 1) 
    {
           ProcInitialLEDState();
    }
    else if (NoDataYet == 0 && (millis()-tlastserial > UDP_SEND_TIMEOUT_MILLISEC))
    {
      if (pktlen > 0)
      // Timeout to send in-progress serial data
      {
        pktReady = 1;
        tlastserial = millis();
      }
    }
    
} // loop()


// Flash duration in milliseconds
#define SHORT_FLASH 100
#define LONG_FLASH 1000


void ProcInitialLEDState()
// For airborne side, three short flashes and a pause.
{
    int t;
    t = millis();

    if ( (t - tlast) > counttarget)
    {

      switch (LEDState)
      {
        case 0:
          SetLED(1);  // LED on
          counttarget = SHORT_FLASH;
          tlast = millis();
          LEDState++;
        break;

        case 1:  //wait
          counttarget = SHORT_FLASH;
          tlast = millis();
          LEDState++;
        break;

        case 2: // LED off
          SetLED(0);
          counttarget = SHORT_FLASH;
          tlast = millis();
          LEDState++;
        break;

        case 3: // wait
          counttarget = SHORT_FLASH;
          tlast = millis();
          LEDState++;
        break;

        case 4:
          SetLED(1);  // LED on
          counttarget = SHORT_FLASH;
          tlast = millis();
          LEDState++;
        break;

        case 5:  //wait
          counttarget = SHORT_FLASH;
          tlast = millis();
          LEDState++;
        break;

        case 6:
          SetLED(0);  // LED off
          counttarget = SHORT_FLASH;
          tlast = millis();
          LEDState++;
        break;

        case 7:  //wait
          counttarget = SHORT_FLASH;
          tlast = millis();
          LEDState++;
        break;

        case 8: // LED on
          SetLED(1);
          counttarget = SHORT_FLASH;
          tlast = millis();
          LEDState++;
        break;

        case 9:  //wait
          counttarget = SHORT_FLASH;
          tlast = millis();
          LEDState++;
        break;

        case 10: // LED off
          SetLED(0);
          counttarget = SHORT_FLASH;
          tlast = millis();
          LEDState++;
        break;

        case 11:  //wait
          counttarget = SHORT_FLASH;
          tlast = millis();
          LEDState++;
        break;

        case 12: // LED on
          SetLED(1);
          counttarget = SHORT_FLASH;
          tlast = millis();
          LEDState++;
        break;

        case 13:  //wait
          counttarget = SHORT_FLASH;
          tlast = millis();
          LEDState++;
        break;

        case 14: // LED off
          SetLED(0);
          counttarget = SHORT_FLASH;
          tlast = millis();
          LEDState++;
        break;

        case 15: // wait
          counttarget = LONG_FLASH;
          tlast = millis();
          LEDState = 0;
        break;

        default:
          counttarget = SHORT_FLASH;
          tlast = millis();
          LEDState = 0;
        break;

      } // switch
    } // if count > counttarget
}


void SetLED( int val)
{
    if (val == 1) {
#ifdef LED_ENABLED
        digitalWrite(ledPin, HIGH);   // set the LED on
#endif
        led = 1;
    } else {
#ifdef LED_ENABLED
        digitalWrite(ledPin, LOW);    // set the LED off
#endif
        led = 0;
    }
}

void ToggleLED()
{
    // Put this in setup() when we want to use it
    // --------------------
    // Initialize the digital pin as an output.
    //pinMode(ledPin, OUTPUT);
    //digitalWrite(ledPin, HIGH);   // set the LED on
    // --------------------

    if (led == 0) {
#ifdef LED_ENABLED
        digitalWrite(ledPin, HIGH);   // set the LED on
#endif
        led = 1;
    } else {
#ifdef LED_ENABLED
        digitalWrite(ledPin, LOW);    // set the LED off
#endif
        led = 0;
    }
}





//------------------------------------------------
//                   SERIAL2
//------------------------------------------------
static void ai_SAMx51_init_SerialTCOM2(void)
{
    SerialTCOM2.begin(115200);              // Open the Serial2 port at 115200 baud
    pinPeripheral(A2, PIO_SERCOM_ALT);  // Assign SERCOM functionality to A2
    pinPeripheral(A3, PIO_SERCOM_ALT);  // Assign SERCOM functionality to A3
}

// Interrupt handler functions
void SERCOM2_0_Handler()
{
    SerialTCOM2.IrqHandler();
}
void SERCOM2_1_Handler()
{
    SerialTCOM2.IrqHandler();
}
void SERCOM2_2_Handler()
{
    SerialTCOM2.IrqHandler();
}
void SERCOM2_3_Handler()
{
    SerialTCOM2.IrqHandler();
}
//------------------------------------------------
//               end SERIAL2
//------------------------------------------------



void ai_SAMx51_init_SerialTCOM1(void)
{
    SerialTCOM1.begin(115200, SERIAL_8N1);  
    // Assign pins SERCOM functionality
    pinPeripheral(PIN_SERIAL_TCOM1_TX, PIO_SERCOM_ALT);
    pinPeripheral(PIN_SERIAL_TCOM1_RX, PIO_SERCOM_ALT);
    //SerialTCOM1.print("This is a test of SerialTCOM1");

}


void SERCOM4_0_Handler()
{
  SerialTCOM1.IrqHandler();
}
void SERCOM4_1_Handler()
{
  SerialTCOM1.IrqHandler();
}
void SERCOM4_2_Handler()
{
  SerialTCOM1.IrqHandler();
}
void SERCOM4_3_Handler()
{
  SerialTCOM1.IrqHandler();
}

#include <Adafruit_NeoPixel.h>
#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            6

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      5

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(10, 100, 48, 110);

unsigned int localPort = 56700;      // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,
char  ReplyBuffer[] = "acknowledged";       // a string to send back
String packetString;

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

char redChar[4];
char greenChar[4];
char blueChar[4];

int red = 0;
int green = 0;
int blue = 0;

int controlMode = 0;

void setup() {
  // start the Ethernet and UDP:
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);

  // Initialize NeoPixel library
  pixels.begin();

  // Start serial port
  Serial.begin(9600);

  // Initialize DIO pins
  pinMode(3, INPUT_PULLUP); // For reading mode switch
  pinMode(2, INPUT);
  pinMode(1, INPUT);

  // Determine whether we're in Ethernet or DIO mode
  controlMode = digitalRead(3);

}

void loop() {
  if (controlMode == 1) {

    int packetSize = Udp.parsePacket();  // if there's data available, read a packet

    if (packetSize) {
      Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
      Serial.println(packetBuffer);

      char redChar[4] = "000";
      char greenChar[4] = "000";
      char blueChar[4] = "000";

      for (int n = 0; n < 3; n++) {
        redChar[0 + n] = packetBuffer[n];
        greenChar[0 + n] = packetBuffer[n + 4];
        blueChar[0 + n] = packetBuffer[n + 8];
        /*
          Serial.print(n);
          Serial.print(';');
          Serial.print(redChar);
          Serial.print(';');
          Serial.print(greenChar);
          Serial.print(';');
          Serial.println(blueChar);
        */
      }

      /*
        Serial.print("Finished char: ");
        Serial.print(redChar);
        Serial.print(';');
        Serial.print(greenChar);
        Serial.print(';');
        Serial.println(blueChar);
      */

      red = atoi(redChar);
      green = atoi(greenChar);
      blue = atoi(blueChar);

      /*
        Serial.print("Finished int: ");
        Serial.print(red);
        Serial.print(';');
        Serial.print(green);
        Serial.print(';');
        Serial.println(blue);
      */

      red = constrain(red, 0, 255);
      green = constrain(green, 0, 255);
      blue = constrain(blue, 0, 255);
    }

  } else {
    int lightState = digitalRead(2);
    red = lightState * 255;
    green = lightState * 255;
    blue = lightState * 255;
  }


  Serial.print("Finished int: ");
  Serial.print(red);
  Serial.print(';');
  Serial.print(green);
  Serial.print(';');
  Serial.println(blue);
  
  for (int i = NUMPIXELS-1; i < NUMPIXELS; i++) {
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(red, green, blue));
    pixels.show(); // This sends the updated pixel color to the hardware.
  }
}

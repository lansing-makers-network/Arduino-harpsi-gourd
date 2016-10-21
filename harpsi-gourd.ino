/*******************************************************************************

 Lansing Makers Network
 ------------------------------

 harpsi-gourd.ino - Touch Gourd Piano
 Author: Michael P. Flaga

 Arduino based capacitive touch sensitive midi note player. aka piano or etc... using the MPR121 and VS1053


 This work is licensed under a Creative Commons Attribution-ShareAlike 3.0
 Unported License (CC BY-SA 3.0) http://creativecommons.org/licenses/by-sa/3.0/

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.

*******************************************************************************/

// Setup of Touch Sensor
#include <MPR121.h> // https://github.com/BareConductive/mpr121
#include <Wire.h>
#define LENGTH_OF_ARRAY(x) ((sizeof(x)/sizeof(x[0])))
#define numElectrodes 12
#define IDLE_TIMEOUT 30000
#define IDLE_RANDOM_LOW_LIMIT 20000
#define IDLE_RANDOM_HIGH_LIMIT 120000
#define INSTRUMENT_CHANGE_TIMEOUT 4000

#include <SPI.h>
#include "PitchToNote.h"

typedef struct // defining each Chip and its associated Touch pins, Note to play, and NeoPixel position
  {
    MPR121_t device;
    uint8_t address;
    unsigned char tthresh[12];
    unsigned char rthresh[12];
    uint32_t timeout[12];
    bool noteState[12];
    uint8_t key[12];
    uint8_t ledPos[12];
  } mprs;

// reserve space for each chip that could be used.
MPR121_t MPR121A;
MPR121_t MPR121B;  // can comment out unused to save dynamic memory.
MPR121_t MPR121C;

uint32_t lastAnyTouchedTimeOut;
uint32_t lastAnyTouchedTimeOutInstrument;
bool idle;
bool idleInstrument;

mprs chips[] = {
  (mprs) {
    MPR121A, // pointer to above reserved memory structure
    0x5A,    // individual address of MPR121 on I2C bus, as defined by pull ups
    {30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30}, //tthresh
    {10,   10,   10,   10,   10,   10,   10,   10,   10,   10,   10,   10}, //rthresh
    {00,   00,   00,   00,   00,   00,   00,   00,   00,   00,   00,   00}, //timeout
    { 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0}, //noteState
    {D5,  D5b,   C5,  A4b,   G4,  G4b,   D4,  D4b,   C4,   D7,  D7b,   C7}, //key
    { 1,    2,    3,    7,    8,    9,   13,   14,   15,   19,   20,   21}  //ledPos
  },

  (mprs) { // comment out if not present. Below will auto size array.
    MPR121B, // pointer to above reserved memory structure
    0x5B,    // individual address of MPR121 on I2C bus, as defined by pull ups
    {30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30}, //tthresh
    {10,   10,   10,   10,   10,   10,   10,   10,   10,   10,   10,   10}, //rthresh
    {00,   00,   00,   00,   00,   00,   00,   00,   00,   00,   00,   00}, //timeout
    { 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0}, //noteState
    {B4,  B4b,   A4,   F4,   E4,  E4b,   F6,   E6,  E6b,   D6,  D6b,   C6}, //key
    { 4,    5,    6,   10,   11,   12,   16,   17,   18,   22,   23,   24}  //ledPos
  }/*,
  (mprs) { // comment out if not present. Below will auto size array.
    MPR121C, // pointer to above reserved memory structure
    0x5C,    // individual address of MPR121 on I2C bus, as defined by pull ups
    {30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30}, //tthresh
    {10,   10,   10,   10,   10,   10,   10,   10,   10,   10,   10,   10}, //rthresh
    {00,   00,   00,   00,   00,   00,   00,   00,   00,   00,   00,   00}, //timeout
    { 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0}, //noteState
    {B7,  B7b,   A7,  A7b,   G7,  G7b,   F7,   E7,  E7b,   D7,  D7b,   C7}, //key
    { 9,    6,    3,    8,    5,    2,    7,    4,    1,   10,   11,   12}  //ledPos
  }*/
  };

// Setup of NeoPixel Array
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define NEOPIXEL_PIN 5
#define LED_TIMEOUT 1000
Adafruit_NeoPixel strip = Adafruit_NeoPixel((LENGTH_OF_ARRAY(chips) * numElectrodes), NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

uint32_t rainbow[] = { // arbitrary number of colors
  strip.Color(255, 128,   0),
  strip.Color(255, 255,   0),
  strip.Color(128, 255,   0),
  strip.Color(  0, 255,   0),
  strip.Color(  0, 255, 128),
  strip.Color(  0, 255, 255),
  strip.Color(  0, 128, 255),
  strip.Color(  0,   0, 255),
  strip.Color(128,   0, 255),
  strip.Color(255,   0, 255),
  strip.Color(255,   0, 128),
  strip.Color(255,   0,   0)
};

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

// Setup of VS1053 Audio DSP
#ifdef SFE
  #define VS_XCS    6 // Control Chip Select Pin (for accessing SPI Control/Status registers)
  #define VS_XDCS   7 // Data Chip Select / BSYNC Pin
  #define VS_DREQ   2 // Data Request Pin: Player asks for more data
  #define VS_RESET  8 //Reset is active low
  #define VS_IRQ    3
#else // AdaFruit
  #define VS_XCS    7 // Control Chip Select Pin (for accessing SPI Control/Status registers)
  #define VS_XDCS   6 // Data Chip Select / BSYNC Pin
  #define VS_DREQ   3 // Data Request Pin: Player asks for more data
  #define VS_RESET  8 //Reset is active low
  #define VS_IRQ    2
#endif

int instrument = 1; //47;
uint8_t volume = 127;
uint8_t idleVolume = volume/3;

void VSWriteRegister(unsigned char addressbyte, unsigned char highbyte, unsigned char lowbyte){
  while(!digitalRead(VS_DREQ)) ; //Wait for DREQ to go high indicating IC is available
  digitalWrite(VS_XCS, LOW); //Select control

  //SCI consists of instruction byte, address byte, and 16-bit data word.
  SPI.transfer(0x02); //Write instruction
  SPI.transfer(addressbyte);
  SPI.transfer(highbyte);
  SPI.transfer(lowbyte);
  while(!digitalRead(VS_DREQ)) ; //Wait for DREQ to go high indicating command is complete
  digitalWrite(VS_XCS, HIGH); //Deselect Control
}

// Plugin to put VS10XX into realtime MIDI mode
const unsigned short sVS1053b_Realtime_MIDI_Plugin[28] = { /* Compressed plugin */
  0x0007, 0x0001, 0x8050, 0x0006, 0x0014, 0x0030, 0x0715, 0xb080, /*    0 */
  0x3400, 0x0007, 0x9255, 0x3d00, 0x0024, 0x0030, 0x0295, 0x6890, /*    8 */
  0x3400, 0x0030, 0x0495, 0x3d00, 0x0024, 0x2908, 0x4d40, 0x0030, /*   10 */
  0x0200, 0x000a, 0x0001, 0x0050,
};

void VSLoadUserCode(void) {
  int i = 0;

  while (i<sizeof(sVS1053b_Realtime_MIDI_Plugin)/sizeof(sVS1053b_Realtime_MIDI_Plugin[0])) {
    unsigned short addr, n, val;
    addr = sVS1053b_Realtime_MIDI_Plugin[i++];
    n = sVS1053b_Realtime_MIDI_Plugin[i++];
    while (n--) {
      val = sVS1053b_Realtime_MIDI_Plugin[i++];
      VSWriteRegister(addr, val >> 8, val & 0xFF);
    }
  }
}

void setup()
{

  // configure interface pins to VS1053
  pinMode(VS_DREQ, INPUT);
  pinMode(VS_XCS, OUTPUT);
  pinMode(VS_XDCS, OUTPUT);
  digitalWrite(VS_XCS, HIGH); //Deselect Control
  digitalWrite(VS_XDCS, HIGH); //Deselect Data
  pinMode(VS_RESET, OUTPUT);

  Serial.begin(115200);
  //while(!Serial);  // only needed if you want serial feedback with the
         // Arduino Leonardo or Bare Touch Board
  Serial.println(F("started Harpsi-Gourd"));

  // start the I2C bus to the MRP121s
  Wire.begin();

  // initialize each available MPR121 and detect failures.
  for(int deviceID=0; deviceID < LENGTH_OF_ARRAY(chips); deviceID++){
    Serial.print(F("Initializing MPR #"));
    Serial.print(deviceID);
    Serial.println();

    if(!chips[deviceID].device.begin(chips[deviceID].address)){
      Serial.println(F("error setting up MPR121"));
      switch(chips[deviceID].device.getError()){
        case NO_ERROR:
          Serial.println(F("no error"));
          break;
        case ADDRESS_UNKNOWN:
          Serial.println(F("incorrect address"));
          break;
        case READBACK_FAIL:
          Serial.println(F("readback failure"));
          break;
        case OVERCURRENT_FLAG:
          Serial.println(F("overcurrent on REXT pin"));
          break;
        case OUT_OF_RANGE:
          Serial.println(F("electrode out of range"));
          break;
        case NOT_INITED:
          Serial.println(F("not initialised"));
          break;
        default:
          Serial.println(F("unknown error"));
          break;
      }
      while(1);
    }

    // Assign the common interrupt used by all MPR121s
    chips[deviceID].device.setInterruptPin(VS_IRQ);

    // Assign the individual sensitivity of each touch pin
    for(unsigned char channel=0; channel < numElectrodes; channel++){

      // this is the touch threshold - setting it low makes it more like a proximity trigger
      // default value is 40 for touch
      chips[deviceID].device.setTouchThreshold(channel, chips[deviceID].tthresh[channel]);

      // this is the release threshold - must ALWAYS be smaller than the touch threshold
      // default value is 20 for touch
      chips[deviceID].device.setReleaseThreshold(channel, chips[deviceID].rthresh[channel]);
    }

    // initial data update
    chips[deviceID].device.updateTouchData();
  }

  //Initialize VS1053 chip
  digitalWrite(VS_RESET, LOW); //Put VS1053 into hardware reset

  //Setup SPI for VS1053
  pinMode(10, OUTPUT); //Pin 10 must be set as an output for the SPI communication to work
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16); //Set SPI bus speed to 1MHz (16MHz / 16 = 1MHz)
  SPI.transfer(0xFF); //Throw a dummy byte at the bus

  delayMicroseconds(1);
  digitalWrite(VS_RESET, HIGH); //Bring up VS1053

  VSLoadUserCode();

  talkMIDI(0xB0, 0, 0);
  Serial.print(F("Bank: "));
  Serial.print(0, DEC);

  talkMIDI(0xB0, 0x07, volume); //0xB0 is channel message, set channel volume to near max (127)
  Serial.print(F(", Set Volume: "));
  Serial.print(volume, DEC);



  talkMIDI(0xC0, instrument, 0);
  Serial.print(F(", Instrument: "));
  Serial.println((instrument + 1), DEC);

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  colorWipe(strip.Color(255, 0, 0), 50); // Red
  colorWipe(strip.Color(0, 255, 0), 50); // Green
  colorWipe(strip.Color(0, 0, 255), 50); // Blue
  colorWipe(strip.Color(0, 0, 0), 0); // OFF
  randomSeed(analogRead(0));

  lastAnyTouchedTimeOut = millis() + (uint32_t) IDLE_TIMEOUT;
  idle = 0;
  lastAnyTouchedTimeOutInstrument = lastAnyTouchedTimeOut;
  idleInstrument = 0;

  noteOff(0, 0, 127); // first one is ignored

  // play an initial note
  noteOn(0, B7, 127);
  delay(500);
  noteOff(0, B7, 127);
  

  Serial.println(F("end setup"));
} // setup()

void sendMIDI(byte data)
{
  SPI.transfer(0);
  SPI.transfer(data);
}

//Plays a MIDI note. Doesn't check to see that cmd is greater than 127, or that data values are less than 127
void talkMIDI(byte cmd, byte data1, byte data2) {
  while (!digitalRead(VS_DREQ))
    ;
  digitalWrite(VS_XDCS, LOW);
  sendMIDI(cmd);
  //Some commands only have one data byte. All cmds less than 0xBn have 2 data bytes
  //(sort of: http://253.ccarh.org/handout/midiprotocol/)
  if( (cmd & 0xF0) <= 0xB0 || (cmd & 0xF0) >= 0xE0) {
    sendMIDI(data1);
    sendMIDI(data2);
  } else {
    sendMIDI(data1);
  }

  digitalWrite(VS_XDCS, HIGH);
} // talkMIDI()

//Send a MIDI note-on message.  Like pressing a piano key
//channel ranges from 0-15
void noteOn(byte channel, byte note, byte attack_velocity) {
  talkMIDI( (0x90 | channel), note, attack_velocity);
} // noteOn()

//Send a MIDI note-off message.  Like releasing a piano key
void noteOff(byte channel, byte note, byte release_velocity) {
  talkMIDI( (0x80 | channel), note, release_velocity);
} // noteOff()

void loop()
{
  uint32_t currentMillis = millis();

  // Capacitive Touch Sensor Keyboard emulator
  for(int deviceID = 0; deviceID < LENGTH_OF_ARRAY(chips); deviceID++){
    if(chips[deviceID].device.touchStatusChanged()){
      chips[deviceID].device.updateTouchData();
      for(uint8_t channel=0; channel < numElectrodes; channel++){
        if(chips[deviceID].device.isNewTouch(channel)){
          Serial.print(F("device=")); Serial.print(deviceID, DEC);
          Serial.print(F(", electrode=")); Serial.print(channel, DEC);
          Serial.print(F(", key=")); Serial.print(chips[deviceID].key[channel], DEC);
          Serial.print(F(", led=")); Serial.print(chips[deviceID].ledPos[channel], DEC);
          Serial.print(F(", rgb=0x")); Serial.print(rainbow[(chips[deviceID].ledPos[channel] - 1)], HEX);
          Serial.println(F(" was just touched by me!"));
          noteOn(0, chips[deviceID].key[channel], 127);
          strip.setPixelColor((chips[deviceID].ledPos[channel] - 1), rainbow[(chips[deviceID].ledPos[channel] - 1)]);
          strip.show();
          chips[deviceID].timeout[channel] = currentMillis + LED_TIMEOUT;
          chips[deviceID].noteState[channel] = 0; // ignore this during idle leave clean up. - cheat
        }
        else if(chips[deviceID].device.isNewRelease(channel)){
          Serial.print(F("device=")); Serial.print(deviceID, DEC);
          Serial.print(F(", electrode=")); Serial.print(channel, DEC);
          Serial.print(F(", key=")); Serial.print(chips[deviceID].key[channel], DEC);
          Serial.print(F(", led=")); Serial.print(chips[deviceID].ledPos[channel], DEC);
          Serial.println(F(" was just released."));
          noteOff(0, chips[deviceID].key[channel], 127);
          strip.setPixelColor((chips[deviceID].ledPos[channel] - 1), strip.Color(0, 0, 0));
          strip.show();
          chips[deviceID].timeout[channel] = 0x0000;
          chips[deviceID].noteState[channel] = 0;
        }
      }

      lastAnyTouchedTimeOut = millis() + (uint32_t) IDLE_TIMEOUT;
      lastAnyTouchedTimeOutInstrument = millis() + INSTRUMENT_CHANGE_TIMEOUT;
      if(idle == 1) { // check if leaving idle
        idle = 0;
        colorWipe(strip.Color(0, 0, 0), 0); // OFF
        Serial.println(F("Leaving Idle Mode."));

        // turn off any notes still on
        for(int deviceID2 = 0; deviceID2 < LENGTH_OF_ARRAY(chips); deviceID2++){
          for(uint8_t channel = 0; channel < numElectrodes; channel++){
            if (chips[deviceID2].noteState[channel] == 1) {
              noteOff(0, chips[deviceID2].key[channel], 127);
              chips[deviceID2].noteState[channel] = 0;
              Serial.print(F(", NoteOFF=")); Serial.print(chips[deviceID2].key[channel], DEC);
            }
          }
        }
        talkMIDI(0xB0, 0x07, volume); //0xB0 is channel message, set channel volume to near max (127)
        Serial.print(F(", Set Volume: "));
        Serial.println(volume, DEC);
      }
      idleInstrument = 0;
    }

    // check if individual neoPixels need updating
    for(uint8_t channel = 0; channel < numElectrodes; channel++){
      if(strip.getPixelColor((chips[deviceID].ledPos[channel] - 1))) { // if not OFF
        if(chips[deviceID].timeout[channel] < currentMillis) { // check if it needs changing
          Serial.print(F("led=")); Serial.print((chips[deviceID].ledPos[channel]), HEX);
          uint32_t randomSeconds = random(IDLE_RANDOM_LOW_LIMIT, IDLE_RANDOM_HIGH_LIMIT);
          Serial.print(F(", randomSeconds=")); Serial.print(randomSeconds, DEC);
          uint8_t ran = random(0, LENGTH_OF_ARRAY(rainbow));
          Serial.print(F(", ranRainbow=")); Serial.print(ran, DEC);
          strip.setPixelColor((chips[deviceID].ledPos[channel] - 1), rainbow[ran]);
          strip.show();
          if (chips[deviceID].noteState[channel] == 0) {
            noteOn(0, chips[deviceID].key[channel], 127);
            chips[deviceID].noteState[channel] = 1;
            Serial.print(F(", NoteON=")); Serial.print(chips[deviceID].key[channel], DEC);
          }
          else {
            noteOff(0, chips[deviceID].key[channel], 127);
            chips[deviceID].noteState[channel] = 0;
            Serial.print(F(", NoteOFF=")); Serial.print(chips[deviceID].key[channel], DEC);
          }
          Serial.println(F(" idle change timeout."));
          chips[deviceID].timeout[channel] = currentMillis + randomSeconds;
        }
      }
    }
  }

  if((lastAnyTouchedTimeOut < currentMillis) && (idle == 0)) { // check if should be idle
    idle = 1;
    Serial.println(F("Gone to Idle Mode."));
    talkMIDI(0xB0, 0x07, idleVolume); //0xB0 is channel message, set channel volume to near max (127)
    Serial.print(F(", Set Volume: "));
    Serial.println(idleVolume, DEC);
  }

  if(lastAnyTouchedTimeOutInstrument < currentMillis) { // check if should be idle
    lastAnyTouchedTimeOutInstrument = millis() + INSTRUMENT_CHANGE_TIMEOUT;
    instrument = random(0, 127);
    talkMIDI(0xC0, instrument, 0);
    Serial.println(F("Instrument Idle Change"));
    Serial.print(F(" Instrument: "));
    Serial.println((instrument + 1), DEC);
  }

  if(idle == 1) { // if in idle
    // do something?

    for(int deviceID = 0; deviceID < LENGTH_OF_ARRAY(chips); deviceID++){
      for(uint8_t channel = 0; channel < numElectrodes; channel++){
        if(chips[deviceID].timeout[channel] < currentMillis) { // check if it needs changing
          Serial.print(F("led=")); Serial.print((chips[deviceID].ledPos[channel]), HEX);
          uint32_t randomSeconds = random(IDLE_RANDOM_LOW_LIMIT, IDLE_RANDOM_HIGH_LIMIT);
          Serial.print(F(", randomSeconds=")); Serial.print(randomSeconds, DEC);
          uint8_t ran = random(0, LENGTH_OF_ARRAY(rainbow));
          Serial.print(F(", ranRainbow=")); Serial.print(ran, DEC);
          strip.setPixelColor((chips[deviceID].ledPos[channel] - 1), rainbow[ran]);
          strip.show();
          Serial.println(F(" Idle just timed out."));
          chips[deviceID].timeout[channel] = currentMillis + randomSeconds;
        }
      }
    }

  }

  if(Serial.available()) {
    parse_menu(Serial.read()); // get command from serial input
  }

} // loop()

void parse_menu(byte key_command) {

  uint8_t result; // result code from some function as to be tested at later time.

  if ((0x20 <= key_command) && (key_command <= 0x7E)) { // ignore Non keyboard characters.
    Serial.print(F("Received command: "));
    Serial.write(key_command);
    Serial.println(F(" "));

    if((key_command == '-') || (key_command == '+')) {
      if(key_command == '+') {
        Serial.println(F("Increamenting"));
        if (instrument < 127) {
          instrument++;
        }
      } else if(key_command == '-') {
        Serial.println(F("Decreamenting"));
        if (instrument > 0) {
          instrument--;
        }
      }
      talkMIDI(0xC0, instrument, 0);
      Serial.print(F(" Instrument: "));
      Serial.println((instrument + 1), DEC);
    } else {
      Serial.println(F("Unknown Command!"));
    }
  }
}

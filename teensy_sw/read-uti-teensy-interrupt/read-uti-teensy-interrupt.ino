// Teensy 4.1 UTI Shield Readout
// S. Luitz 2022
// based on ARDUINO UTI Shield Readout (S. Luitz)
//
// improved version - record pin states (potentially) in parallel using pin change interrupts
//
// Needs QNEthernet, ArduinoModbus, and ArduinoRS485 libraries
//
// Board Notes:
//
// sel1 and sel4 are permanently hardwired to "0" - this allows us to program the following modes:
// sel2 = 0, sel3 = 0 : 5 caps 0-2pF
// sel2 = 0, sel3 = 1 : 5 caps 0-12pF
// sel2 = 1, sel3 = 0 : 3 caps, variable range 0-300pF
// sel2 = 1, sel3 = 1 : unused (Thermistor 1-25kOhm, 4-wire)

// The mode preferred by the board is 0-300pF (0-2pF and 0-12pF require connecting the caps to the pin header meant for the resistors in mode 0-300pF )


// #define DEBUG
// #define DEBUG0
// #define DEBUG1

extern unsigned long _heap_start;
extern unsigned long _heap_end;
extern char *__brkval;

#include <SPI.h>
//#include <NativeEthernet.h>


#include <QNEthernet.h>
using namespace qindesign::network;

#include <EEPROM.h>

#include <ArduinoRS485.h>  // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>

#define EEPROM_BASE_CONF 40
#define CKSUM_INIT 0xcc

// Ethernet settings
byte mac[6];
char macstr[18];
IPAddress ip(192, 168, 1, 77);
IPAddress gateway(192, 168, 1, 254);
IPAddress subnet(255, 255, 255, 0);
IPAddress nameserver(192, 168, 1, 254);


// Modbus Server on port 502
EthernetServer ethServer(502);
ModbusTCPServer modbusTCPServer;

// EthernetClient client;

// board connections

const unsigned int MAXCHIPS = 4;  // maximum number of chips on the board
const unsigned int NCHIPS = 4;    // number of chips on the board 

// chips are numbered:
// +-------+
// ! T 0 1 !
// ! T 3 2 |
// +-------+

const uint8_t out[MAXCHIPS] = { 21, 18, 15, 40 };
const uint8_t sel2[MAXCHIPS] = { 22, 19, 16, 41 };
const uint8_t sel3[MAXCHIPS] = { 23, 20, 17, 14 };

const uint8_t sf_all = 39;  // slow/fast pin

// UTI readout modes

const uint16_t MODE_off = 0;
const uint16_t MODE_2pf = 1;
const uint16_t MODE_12pf = 2;
const uint16_t MODE_300pf = 3;

const uint16_t MODE_SLOW = 0;
const uint16_t MODE_FAST = 1;

// other constants

const uint8_t MAXPHASES = 5;  // maximum number of phases in a cycle
const uint8_t NCYCLES = 11;   // number of cycles to record - must be at least 11 to record one complete sequence
const uint8_t IDX_STOP = 2 * NCYCLES + 1;

const unsigned int OFFSET_MAXVAR = 20000;  // maximum length difference for the 2 Offset cycles
const unsigned int MAXWAIT = 300;          // number of microseconds to wait for acquisition to finish
const unsigned int LOOPDELAY = 300;
// 
// clock cycle counter speed of board
const float CLOCKSPEED = 600.0; // MHz

// Default Mode 
#define DEFMODE MODE_300pf
const uint16_t utispeed = MODE_SLOW;  // SLOW mode is less noisy


// pulse detection settings

const float tol = 1.3; // pulse length detection tolerance




// Modbus configuration:
// Holding Registers: H[ChipNO] : mode (0: off, 1: 2pf, 2: 12pf, 3: 300pf) -- default is 2
// Input Registers: I[2*ChipNO], I[2*ChipNO+1]: 32-bit float capacity ratio
//                  I[2*MAXCHIPS + ChipNo]: 16-bit status


// Status codes

const uint16_t STATUS_OK = 0x00;           // we have a value
const uint16_t STATUS_CLIPPED_LOW = 0x02;  // result was clipped to zero
const uint16_t STATUS_DIVZERO = 0x04;      // division by zero in the ratio calculation
const uint16_t STATUS_BADREAD = 0x08;      // chip not detected, etc.
const uint16_t STATUS_BADPHASES = 0x10;    // wrong number of phases detected



// Various global variables

// pulse train recording
volatile unsigned int iwidth[NCHIPS][NCYCLES];
volatile unsigned int prevtime[NCHIPS];
volatile bool oldstate[NCHIPS];
volatile uint8_t idx[NCHIPS];
volatile unsigned int ISRtime;


uint16_t inputregs[3 * MAXCHIPS];  // buffer to assemble measurement data before handing over to modbus
uint8_t ledblink = 0; // LED status for blinking
uint16_t oldmode[MAXCHIPS];




void setup() {


  // Serial 
  Serial.begin(115200);
  Serial.println("Serial initialized");
  stdPrint = &Serial;

  // initialize input and output pins and mode buffer
  for (unsigned int chip = 0; chip < NCHIPS; chip++) {
    if (sel2[chip]) pinMode(sel2[chip], OUTPUT);
    if (sel3[chip]) pinMode(sel3[chip], OUTPUT);
    if (out[chip]) pinMode(out[chip], INPUT);
    oldmode[chip] = 65535;
  }

  pinMode(sf_all, OUTPUT);  // set the slow/fast control to output

  // set pin 13 to output so that we can blink the LED
  pinMode(13, OUTPUT);

  // set up network
  teensyMAC(mac);
  Serial.print("MAC address: ");
  snprintf(macstr, 18, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println(macstr);


  // Try DHCP
  // Serial.println("Attempting to get IP configuration through DHCP");
  // if (Ethernet.begin()) {
  //   // Success - this is our new config
  //   Serial.println("DHCP Success!");
  // } else {
  //   Serial.println("DHCP Failure! Using default IP address");
  //   Ethernet.begin( ip, subnet, gateway);  // start ethernet interface
  // }

  Ethernet.begin(ip, subnet, gateway);
  Serial.println("Ethernet interface started");
  Serial.print("IP Address: ");
  Serial.println(Ethernet.localIP());
  Serial.print("Netmask: ");
  Serial.println(Ethernet.subnetMask());
  Serial.print("Gateway: ");
  Serial.println(Ethernet.gatewayIP());

  // start the Modbus server
  ethServer.begin();
  if (!modbusTCPServer.begin()) {
    Serial.println("Failed to start Modbus TCP Server!");
    while (1)
      ;
  }

  // set up modbus registers
  modbusTCPServer.configureHoldingRegisters(0, MAXCHIPS);
  modbusTCPServer.configureInputRegisters(0, 2 * MAXCHIPS + MAXCHIPS);

  // initalize mode registers from EEPROM if possible
  if (EEPROM_read_conf() == 0) {
// use default if we can't read from EEPROM
#ifdef DEBUG
    Serial.println("Initializing EEPROM with default settings");
#endif
    for (unsigned int chip = 0; chip < NCHIPS; chip++) {
      modbusTCPServer.holdingRegisterWrite(chip, DEFMODE);
    }
  }

  // enable pin interrrupts
  for (unsigned int chip = 0; chip < NCHIPS; chip++)
    attachInterrupt(digitalPinToInterrupt(out[chip]), pin_isr, CHANGE);
}


void pin_isr() {

  unsigned int now = ARM_DWT_CYCCNT;  // get cycle counter
  for (uint8_t chip = 0; chip < NCHIPS; chip++) {
    if (idx[chip] >= IDX_STOP) continue;  // skip chip if buffer is full - so it is safe to read out
    bool pin = digitalRead(out[chip]);
    if (pin == oldstate[chip]) continue;  // pin hasn't changed, no action needed
    if (!idx[chip]) {                     // index is 0: start new measurement
      if (pin) {                          // wait for the pin to become low
        oldstate[chip] = pin;
        continue;
      } else {  // pin has become low, initialize prevtime
        prevtime[chip] = now;
        idx[chip]++;
        oldstate[chip] = pin;
        continue;
      }
    }

    if (pin == 0) iwidth[chip][(idx[chip] / 2) - 1] = now - prevtime[chip];  // doing the math in unsigned int takes care of counter wrap
    idx[chip]++;
    prevtime[chip] = now;
    oldstate[chip] = pin;
    LEDBlink();
  }
  ISRtime = ARM_DWT_CYCCNT - now;
}

void loop() {

  unsigned long counts[MAXPHASES];
  unsigned long baseline;           // baseline measurement
  unsigned long ref;                // reference measurement
  unsigned long val;                // sensor measurement
  uint8_t phases;                   // phases in chip cycle
  unsigned int status = STATUS_OK;  // result status

  float ratio;  // D/C

#ifdef DEBUG
  Serial.println("+++++++++++++++++++++++++++++++++++");
#endif

  // loop over all chips
  for (uint8_t chip = 0; chip < NCHIPS; chip++) {
    ratio = 0;

    uint16_t mode = modbusTCPServer.holdingRegisterRead(chip);
    if (mode != oldmode[chip]) {
      setUTIMode(chip, mode & 0x03, sf_all);
      oldmode[chip] = mode;
      delay(300);
    }
    if (mode > 0) {
      ReadUTI(chip, &phases, counts);

#ifdef DEBUG
      Serial.print("Chip: ");
      Serial.print(chip);
      Serial.print(", Mode: ");
      Serial.println(mode);
#endif
#ifdef DEBUG0
      Serial.print("Phases detected: ");
      Serial.println(phases);
      Serial.println("Counts:");
      for (int i = 0; i < 3; i++) Serial.println(counts[i]);
#endif

      if (!phases) {
#ifdef DEBUG0
        Serial.println("**** Chip read error or not detected");
#endif
        status |= STATUS_BADREAD;

      };  // readout error ... will try next chip

      // check that we got the right number of phases corresponding to the mode
      if (mode == 3) {
        // this is mode 3 - we expect a 3-phase cycle
        if (phases != 3) {
#ifdef DEBUG
          Serial.println("**** Wrong number of phases in 3-phase cycle");
#endif
          status |= STATUS_BADPHASES;
        }

      } else {
        // modes 1 or 2 are 5-phase cycles
        if (phases != 5) {
#ifdef DEBUG
          Serial.println("**** Wrong number of phases in 5-phase cycle");
#endif
          status |= STATUS_BADPHASES;
        }
      }

      if (status == STATUS_OK) {
        baseline = counts[0];  // chip baseline (open input B)
        ref = counts[1];       // reference capacitor (input C)
        val = counts[2];       // sensor value (input D)

        if (ref != baseline) {  // make sure we don't divide by 0
          ratio = (float)(val - baseline) / (float)(ref - baseline);
        } else {
          ratio = 0;
          status |= STATUS_DIVZERO;
        }

        if (ratio < 0) {  // clamp lower limit of ratio to 0
          ratio = 0;
          status |= STATUS_CLIPPED_LOW;
        }
      }


#ifdef DEBUG
      Serial.print("Ratio ");
      Serial.print(" = ");
      Serial.println(ratio, 5);
#endif

      // update modbus buffer
      inputregs[2 * MAXCHIPS + chip] = status;
      enc_float(&inputregs[2 * chip], ratio);
      modbusTCPServer.writeInputRegisters(0, inputregs, 3 * MAXCHIPS);

#ifdef DEBUG0
      for (unsigned i = 0; i < 3 * MAXCHIPS; i++) {
        Serial.print(inputregs[i]);
        Serial.print(" ");
      }
      Serial.println();
#endif
      serveNetwork();

    }  // if chip enabled


    // blink LED
    // ledblink = (ledblink ? 0 : 1);
    // if (ledblink) digitalWrite(13, 1);
    // else digitalWrite(13, 0)
  }

  // printISRbufs();
#ifdef DEBUG0
  Serial.print("ISRtime: ");
  Serial.println((float)(1 / CLOCKSPEED * ISRtime));
#endif

  EEPROM_write_conf();
  serveNetwork();

  Serial.println(freeram());

}

void serveNetwork() {

  elapsedMillis sinceDelayStart = 0;
  while (sinceDelayStart < LOOPDELAY) {
  EthernetClient client = ethServer.available();
    if (client) {
#ifdef DEBUG0
      Serial.println("new client");
      Serial.println()
#endif
        modbusTCPServer.accept(client);
      if (client.connected()) {
        modbusTCPServer.poll();
      }
    }
  }
}



void setUTIMode(uint8_t chip, uint8_t mode, uint8_t sf_all) {

  // sel2 = 0, sel3 = 0 : 5 caps 0-2pF
  // sel2 = 0, sel3 = 1 : 5 caps 0-12pF
  // sel2 = 1, sel3 = 0 : 3 caps, variable range 0-300pf

  switch (mode) {
    case MODE_2pf:
      digitalWrite(sel2[chip], 0);
      digitalWrite(sel3[chip], 0);
      break;

    case MODE_12pf:
      digitalWrite(sel2[chip], 0);
      digitalWrite(sel3[chip], 1);
      break;

    case MODE_300pf:
      digitalWrite(sel2[chip], 1);
      digitalWrite(sel3[chip], 0);
      break;
  }

  digitalWrite(sf_all, utispeed);  // set slow/fast mode bit
}

int ReadUTI(uint8_t chip, uint8_t* phases, unsigned long counts[]) {

  int startindex = -1;
  unsigned int ph;
  unsigned int width[NCYCLES];
  ph = 3;
  elapsedMillis sinceAcqStart;

  

  idx[chip] = 0;  // start acquisition

  sinceAcqStart = 0;
  while ((idx[chip] < IDX_STOP) && (sinceAcqStart < MAXWAIT)) yield();  // wait for acquisition to complete

  if (idx[chip] < IDX_STOP) {  // acquisition timed out
#ifdef DEBUG
    Serial.println("Acquisition timeout");
#endif
    *phases = 0;
    return -1;
  }

  // copy data from interrupt handler memory to local array
  for (unsigned i = 0; i < NCYCLES; i++) width[i] = iwidth[chip][i];

#ifdef DEBUG0
  Serial.println("--- Data from ISR -----------------------");
  for (unsigned int j = 0; j < NCYCLES; j++) {
    // Serial.print(iwidth[i][j]/CLOCKSPEED);
    Serial.print(width[j]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println("-----------------------------------------");
#endif

  // find start of sequence

  for (unsigned int i = 0; i < NCYCLES - 5; i++) {
    if (abs((long)width[i] - (long)width[i + 1]) < OFFSET_MAXVAR && width[i] * tol < width[i + 2] && width[i] * tol < width[i + 3]
        && width[i + 1] * tol < width[i + 2] && width[i + 1] * tol < width[i + 3]) {
      startindex = i;
      break;
    }
  }

  if (startindex < 0) {
    // we didn't find a start index. Set phases to 0 and return error.

#ifdef DEBUG
    Serial.println("Error: Could not find start of sequence");
#endif
    *phases = 0;
    return -1;
  }

  // check for 4 and 5 phases
  if (width[startindex + 4] > tol * width[startindex]) ph = 4;
  if (width[startindex + 5] > tol * width[startindex]) ph = 5;

#ifdef DEBUG1
  Serial.println("****************************");
  Serial.print("Chip: ");
  Serial.println(chip);
  Serial.print("Startindex: ");
  Serial.println(startindex);
  Serial.print("Phases: ");
  Serial.println(ph);
  Serial.println("-----");

  for (unsigned int i = startindex; i <= startindex + ph; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.println(width[i]);
  }
#endif

  // copy counts & # of phases
  counts[0] = width[startindex] + width[startindex + 1];  // offset count is sum of first two widths
  for (unsigned int i = 1; i < ph; i++) counts[i] = width[startindex + i + 1];
  *phases = ph;
  return 0;
}


void EEPROM_cond_write(int addr, byte val)

// only write values that are different

{
  if ((EEPROM.read((addr)) != (val))) {
    EEPROM.write(addr, val);
#ifdef DEBUG
    Serial.print("WRITE_COND");
    Serial.print(addr);
    Serial.print(":");
    Serial.println(val);
#endif
  }
}

void EEPROM_write_conf() {

  int addr = EEPROM_BASE_CONF;
  uint8_t cksum = CKSUM_INIT;
  uint8_t val;

#ifdef DEBUG0
  Serial.println("Updating EEPROM configuration");
#endif

  for (unsigned int chip = 0; chip < NCHIPS; chip++) {
    val = modbusTCPServer.holdingRegisterRead(chip) & 0xff;
    EEPROM_cond_write(addr + chip, val);
    cksum ^= val;
  }
  // write checksum
  EEPROM_cond_write(addr + NCHIPS, cksum);
}

int EEPROM_read_conf() {

  uint8_t cksum = CKSUM_INIT;
  int addr = EEPROM_BASE_CONF;

#ifdef DEBUG0
  Serial.println("Reading EEPROM configuration");
#endif

  // verify checksum
  for (unsigned int chip = 0; chip < NCHIPS; chip++)
    cksum ^= EEPROM.read(addr + chip);

  if (cksum == EEPROM.read(addr + NCHIPS)) {
// checksum OK, read values
#ifdef DEBUG
    Serial.println("EEPROM: Checksum OK");
#endif
    for (unsigned int chip = 0; chip < NCHIPS; chip++) {
      uint16_t mode = EEPROM.read(addr + chip);
#ifdef DEBUG
      Serial.print("EEPROM read chip, mode:");
      Serial.print(chip);
      Serial.print(", ");
      Serial.println(mode);
#endif

      modbusTCPServer.holdingRegisterWrite(chip, mode);
    }
    return 1;
  } else {
// checksum not OK
#ifdef DEBUG
    Serial.println("EEPROM: Checksum NOT OK");
#endif
    return 0;
  }
}


void enc_float(uint16_t* t, float x) {

  union U {
    float f;
    uint32_t u;
  };
  union U u;
  u.f = x;
  t[0] = (u.u >> 16) & 0xffffU;
  t[1] = (u.u & 0xffffU);
}

void teensyMAC(uint8_t* mac) {
  for (uint8_t by = 0; by < 2; by++) mac[by] = (HW_OCOTP_MAC1 >> ((1 - by) * 8)) & 0xFF;
  for (uint8_t by = 0; by < 4; by++) mac[by + 2] = (HW_OCOTP_MAC0 >> ((3 - by) * 8)) & 0xFF;
}


void printISRbufs() {
  for (unsigned int i = 0; i < NCHIPS; i++) {
    Serial.print("Index ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.println(idx[i]);
    for (unsigned int j = 0; j < NCYCLES; j++) {
      Serial.print(iwidth[i][j] / CLOCKSPEED);
      // Serial.print(iwidth[i][j]);
      Serial.print(" ");
    };
    Serial.println();
    // idx[i] = 0;
  }
  Serial.print("ISRtime: ");
  Serial.println((float)(1 / CLOCKSPEED * ISRtime));
}

void LEDBlink() {

  ledblink = (ledblink ? 0 : 1);
  if (ledblink) digitalWrite(13, 1);
  else digitalWrite(13, 0);
}

int freeram() {
  return (char *)&_heap_end - __brkval;
}

// Teensy 4.1 UTI Shield Readout
// S. Luitz @ SLAC 2022
// Based on ARDUINO UTI Shield Readout S. Luitz @ SLAC 2016
// Modified by J. Nellakra @ Carleton 2022

// Improved version: record pin states in parallel using pin change interrupts

// Modifications done by Jyothi:
// - Output more values on Modbus.
// - Remove hardcoded constants.

extern unsigned long _heap_start;
extern unsigned long _heap_end;
extern char *__brkval;

#include <EEPROM.h>

#define CLK_TIME (1000 / 600.0) // Clocked at 600 MHz ~ 1.6 ns Cycles
#define EEPROM_BASE_CONF 40
#define CKSUM_INIT 0xcc

#include <SPI.h>
#include <QNEthernet.h>

using namespace qindesign::network;

#include <ArduinoRS485.h>  // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>

// Ethernet settings
byte mac[6];
char macstr[18];
IPAddress ip(172, 21, 103, 30);
IPAddress gateway(172, 21, 102, 1);
IPAddress subnet(255, 255, 254, 0);
IPAddress nameserver(1, 1, 1, 1);

// Modbus Server
EthernetServer ethServer(502);
ModbusTCPServer modbusTCPServer;

// Board Connections
#define LED_PIN 13

#define MAXCHIPS 4  // Maximum number of chips on the board
#define NCHIPS 1    // Number of chips on the board

// Pins sel1 and sel4 are permanently hardwired to "0". This allows us to program the following modes:
// - sel2 = 0, sel3 = 0 : 5 caps 0-2pF
// - sel2 = 0, sel3 = 1 : 5 caps 0-12pF
// - sel2 = 1, sel3 = 0 : 3 caps, variable range 0-300pf
// - sel2 = 1, sel3 = 1 : unused (Thermistor 1-25kOhm, 4-wire)

// Chips are numbered as such, depending on the board you have:
// -----------    ----------
// | 0 1 2 3 |    | 2    3 |
// | Teensy  | OR | 1    4 |
// -----------    | Teensy |
//                ----------

// Pins for Chip Connections
const uint8_t out[MAXCHIPS] = { 21, 18, 15, 40 };
const uint8_t sel2[MAXCHIPS] = { 22, 19, 16, 41 };
const uint8_t sel3[MAXCHIPS] = { 23, 20, 17, 14 };
const uint8_t sf_all = 39;  // Slow / fast pin for all chips.

// UTI Readout Modes
#define MODE_OFF   0
#define MODE_2PF   1
#define MODE_12PF  2
#define MODE_300PF 3

#define MODE_SLOW 0
#define MODE_FAST 1

// Other Constants
#define MAXPHASES 5  // Maximum number of phases in a cycle
#define NCYCLES 11   // Number of cycles to record - must be at least 11 to record one complete sequence
#define IDX_STOP (2 * NCYCLES + 1)

#define OFFSET_MAXVAR 20000  // Maximum length difference for the 2 Offset cycles
#define MAXWAIT 300          // Number of microseconds to wait for acquisition to finish
#define LOOPDELAY 300

// Default Mode to Initialise on Startup
#define DEFMODE MODE_12PF
#define UTISPEED MODE_SLOW  // SLOW mode is less noisy

#define tol 1.3 // Magic constant?? TODO: Describe it better.

// Global variables to record pulse trains
volatile unsigned int iwidth[NCHIPS][NCYCLES];
volatile unsigned int prevtime[NCHIPS];
volatile bool oldstate[NCHIPS];
volatile uint8_t idx[NCHIPS];
volatile unsigned int ISRtime;

// Modbus configuration:
// Holding Registers: H[ChipNO]: mode (0: off, 1: 2pf, 2: 12pf, 3: 300pf)
// Input Registers:
// - I[2*ChipNO], I[2*ChipNO + 1]: 32-bit float capacity ratio
// - I[2*NCHIPS + 2*ChipNO], I[+1]: 32-bit float baseline capacitance length
// - I[4*NCHIPS + 2*ChipNO], I[+1]: 32-bit float reference capacitance length
// - I[6*NCHIPS + 2*ChipNO], I[+1]: 32-bit float capacitance length value
// - I[8*NCHIPS + ChipNo]: 16-bit status

#define STATUS_OK 0x00           // We have a value
#define STATUS_CLIPPED_LOW 0x02  // Result was clipped to zero
#define STATUS_DIVZERO 0x04      // Division by zero in the ratio calculation
#define STATUS_BADREAD 0x08      // Chip not detected, etc.
#define STATUS_BADPHASES 0x10    // Wrong number of phases detected

uint16_t inputregs[9 * NCHIPS];  // Buffer to assemble measurement data before handing over to modbus
uint16_t chip_modes[NCHIPS]; // Stores the current chip modes to compare with settings received over modbus

// Debugging Information Controls
#define DEBUG  1 // Show Modbus data on serial output
#define DEBUG0 1 // Show chip output data
#define DEBUG1 1 // Show internal arduino data

void teensyMAC(uint8_t* mac) {
  for (uint8_t by = 0; by < 2; by++) {
    mac[by] = (HW_OCOTP_MAC1 >> ((1 - by) * 8)) & 0xFF;
  }

  for (uint8_t by = 0; by < 4; by++) {
    mac[by + 2] = (HW_OCOTP_MAC0 >> ((3 - by) * 8)) & 0xFF;
  }
}

int EEPROM_read_conf() {
  uint8_t cksum = CKSUM_INIT;
  int addr = EEPROM_BASE_CONF;

#ifdef DEBUG1
  Serial.println("Reading EEPROM configuration");
#endif

  // Verify checksum
  for (unsigned int chip = 0; chip < NCHIPS; chip++)
    cksum ^= EEPROM.read(addr + chip);

  if (cksum == EEPROM.read(addr + NCHIPS)) {
    // Checksum OK, read values

#ifdef DEBUG1
    Serial.println("Checksum OK");
#endif

    for (unsigned int chip = 0; chip < NCHIPS; chip++) {
      uint16_t mode = EEPROM.read(addr + chip);

#ifdef DEBUG1
      Serial.print("EEPROM read: chip ");
      Serial.print(chip);
      Serial.print(", mode ");
      Serial.println(mode);
#endif

      modbusTCPServer.holdingRegisterWrite(chip, mode);
    }

    return 1;
  } else {
    // Checksum not OK

#ifdef DEBUG1
    Serial.println("Error: Checksum NOT OK");
#endif

    return 0;
  }
}

// Function called on interrupts
void pin_isr() {
  unsigned int now = ARM_DWT_CYCCNT;  // Get cycle counter

  for (uint8_t chip = 0; chip < NCHIPS; chip++) {
    if (idx[chip] >= IDX_STOP) continue;  // Skip chip if buffer is full - so it is safe to read out

    bool pin = digitalRead(out[chip]);
    if (pin == oldstate[chip]) continue;  // Pin hasn't changed, no action needed

    /// If the index is 0, start a new measurement.
    if (!idx[chip]) {
      if (pin) {
        // Wait for the pin to become low
        oldstate[chip] = pin;
        continue;
      }
      
      else {
        // Pin has become low, initialize prevtime
        prevtime[chip] = now;
        oldstate[chip] = pin;
        idx[chip]++;
        continue;
      }
    }

    if (pin == 0) iwidth[chip][(idx[chip] / 2) - 1] = now - prevtime[chip];
    // Doing the math in unsigned int takes care of counter wrap

    // Update the prevtime.
    prevtime[chip] = now;
    oldstate[chip] = pin;
    idx[chip]++;

    LEDBlink();
  }

  // Update the time sent in the interrupt service routine.
  ISRtime = ARM_DWT_CYCCNT - now;
}

void setup() {
  // Initialise the serial
  Serial.begin(115200);
  Serial.println("Serial initialized");
  stdPrint = &Serial;

  // Initialize input and output pins and mode buffer
  for (unsigned int chip = 0; chip < NCHIPS; chip++) {
    if (sel2[chip]) pinMode(sel2[chip], OUTPUT);
    if (sel3[chip]) pinMode(sel3[chip], OUTPUT);
    if (out[chip])  pinMode( out[chip], INPUT );

    chip_modes[chip] = 65535; // Set garbage values to change later
  }

  pinMode(sf_all,  OUTPUT);  // Set the slow/fast control to output
  pinMode(LED_PIN, OUTPUT); // set the LED pin to output so that we can blink the LED

  // Set up networking
  teensyMAC(mac);
  Ethernet.begin(ip, subnet, gateway);

  snprintf(macstr, 18, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Serial.print("MAC address: ");
  Serial.println(macstr);

  Serial.print("IP Address: ");
  Serial.println(Ethernet.localIP());
  
  Serial.print("Netmask: ");
  Serial.println(Ethernet.subnetMask());

  Serial.print("Gateway: ");
  Serial.println(Ethernet.gatewayIP());

  // Start the Modbus server
  ethServer.begin();

  if (!modbusTCPServer.begin()) {
    Serial.println("ERROR: Failed to start Modbus TCP Server!");
    while(1); // Hang on failure
  }

  // Set up modbus registers
  modbusTCPServer.configureHoldingRegisters(0, NCHIPS);
  modbusTCPServer.configureInputRegisters(0, 9 * NCHIPS);

  // Initalize mode registers from EEPROM if possible
  if (EEPROM_read_conf() == 0) {

#ifdef DEBUG
    Serial.println("Initializing EEPROM with default settings");
#endif

    for (unsigned int chip = 0; chip < NCHIPS; chip++) {
      modbusTCPServer.holdingRegisterWrite(chip, DEFMODE);
    }
  }

  // Enable pin interrrupts
  for (unsigned int chip = 0; chip < NCHIPS; chip++) {
    attachInterrupt(digitalPinToInterrupt(out[chip]), pin_isr, CHANGE);
  }
}

void setUTIMode(uint8_t chip, uint8_t mode, uint8_t sf_all) {
  // Board Settings:
  // - sel2 = 0, sel3 = 0 : 5 caps 0-2pF
  // - sel2 = 0, sel3 = 1 : 5 caps 0-12pF
  // - sel2 = 1, sel3 = 0 : 3 caps, variable range 0-300pf

  switch (mode) {
    case MODE_2PF:
      digitalWrite(sel2[chip], 0);
      digitalWrite(sel3[chip], 0);
      break;

    case MODE_12PF:
      digitalWrite(sel2[chip], 0);
      digitalWrite(sel3[chip], 1);
      break;

    case MODE_300PF:
      digitalWrite(sel2[chip], 1);
      digitalWrite(sel3[chip], 0);
      break;
  }

  digitalWrite(sf_all, UTISPEED);  // Set slow / fast mode bit
}

int ReadUTI(uint8_t chip, uint8_t* phases, unsigned long counts[]) {

  int startindex = -1;
  unsigned int ph = 3;
  unsigned int width[NCYCLES];
  elapsedMillis sinceAcqStart;

  idx[chip] = 0;  // Start acquisition
  sinceAcqStart = 0;

  while ((idx[chip] < IDX_STOP) && (sinceAcqStart < MAXWAIT)) {
    yield(); // Wait for acquisition to complete
  }

  if (idx[chip] < IDX_STOP) {  // Acquisition timed out
#ifdef DEBUG1
    Serial.println("ERROR: Acquisition timeout");
#endif

    *phases = 0;
    return -1;
  }

  // Copy data from interrupt handler memory to local array
  for(unsigned i = 0; i < NCYCLES; i++) {
    width[i] = iwidth[chip][i];
  }

#ifdef DEBUG0
  Serial.println("------------ Data from ISR -------------");
  for (unsigned int j = 0; j < NCYCLES; j++) {
    Serial.print(width[j] * CLK_TIME, 1);
    Serial.print(" ns ");
  }
  Serial.println();
  Serial.println("----------------------------------------");
#endif

  // Find start of sequence
  for (unsigned int i = 0; i < NCYCLES - 5; i++) {
    if (abs((long)width[i] - (long)width[i + 1]) < OFFSET_MAXVAR
      && width[i] * tol < width[i + 2] && width[i] * tol < width[i + 3]
      && width[i + 1] * tol < width[i + 2] && width[i + 1] * tol < width[i + 3]
    ){
      startindex = i;
      break;
    }
  }

  if (startindex < 0) {
    // We didn't find a start index. Set phases to 0 and return error.
    
#ifdef DEBUG1
    Serial.println("ERROR: Could not find start of sequence");
#endif

    *phases = 0;
    return -1;
  }

  // Check for 4 and 5 phases
  if (width[startindex + 4] > tol * width[startindex]) ph = 4;
  if (width[startindex + 5] > tol * width[startindex]) ph = 5;

#ifdef DEBUG1
  Serial.print("Startindex: ");
  Serial.println(startindex);
#endif

  // Copy counts & # of phases
  counts[0] = width[startindex] + width[startindex + 1];  // Offset count is sum of first two widths
  for (unsigned int i = 1; i < ph; i++) {
    counts[i] = width[startindex + i + 1];
  }

  *phases = ph;
  return 0;
}

void EEPROM_cond_write(int addr, byte val) {
  // Only write values that are different
  if ((EEPROM.read((addr)) != (val))) {
    EEPROM.write(addr, val);

#ifdef DEBUG1
    Serial.print("Writing ");
    Serial.print(val);
    Serial.print(" to ");
    Serial.println(addr);
#endif
  }
}

void EEPROM_write_conf() {
  int addr = EEPROM_BASE_CONF;
  uint8_t cksum = CKSUM_INIT;
  uint8_t val;

#ifdef DEBUG1
  Serial.println("Updating EEPROM configuration");
#endif

  for (unsigned int chip = 0; chip < NCHIPS; chip++) {
    val = modbusTCPServer.holdingRegisterRead(chip) & 0xff;
    EEPROM_cond_write(addr + chip, val);
    cksum ^= val;
  }

  // Write checksum
  EEPROM_cond_write(addr + NCHIPS, cksum);
}

void enc_float(uint16_t* t, float x) {
  union U {
    float f;
    uint32_t u;
  } u;

  u.f = x;
  t[0] = (u.u >> 16) & 0xffffU;
  t[1] = (u.u & 0xffffU);
}

void printISRbufs() {
  for (unsigned int i = 0; i < NCHIPS; i++) {
    Serial.print("Index ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(idx[i]);

    for (unsigned int j = 0; j < NCYCLES; j++) {
      Serial.print(iwidth[i][j] * CLK_TIME, 1);
      Serial.print(" ns ");
    };
    Serial.println("");
  }

  Serial.print("ISRtime: ");
  Serial.print(CLK_TIME * ISRtime, 1);
  Serial.println(" ns");
}

void serveNetwork() {
  elapsedMillis sinceDelayStart = 0;

  while (sinceDelayStart < LOOPDELAY) {
    EthernetClient client = ethServer.available();

    if (client) {
#ifdef DEBUG1
      Serial.println("New client");
      Serial.println();
#endif

      modbusTCPServer.accept(client);
      if (client.connected()) {
        modbusTCPServer.poll();
      }
    }
  }
}

static void LEDBlink() {
  uint8_t ledblink = 0;

  ledblink = (ledblink ? 0 : 1);
  if (ledblink) digitalWrite(13, 1);
  else digitalWrite(13, 0);
}

int freeram() {
  return (char *)&_heap_end - __brkval;
}

void loop() {
  static unsigned long counts[MAXPHASES];

  unsigned long baseline = 0; // Baseline measurement
  unsigned long ref = 0;      // Reference measurement
  unsigned long val = 0;      // Sensor measurement
  uint8_t phases = 0;         // Phases in chip cycle

  unsigned int status = STATUS_OK; // Result status
  float ratio = 0.0; // Calculated as (value - baseline) / (reference - baseline)

#ifdef DEBUG
  Serial.println("++++++++++++++++++++++++++++++++++++++++");
#endif

  // Loop over all chips
  for (uint8_t chip = 0; chip < NCHIPS; chip++) {
    ratio = 0;

    // Get Modbus value and and it by 0x03 to get rid of higher bytes and ensure it's <= 3.
    uint16_t mode = modbusTCPServer.holdingRegisterRead(chip) & 0x03;

    if (mode != chip_modes[chip]) {
      setUTIMode(chip, mode, sf_all);
      chip_modes[chip] = mode;
      delay(300);
    }

    if (mode > 0) { // Chip Enabled
      // Get phases and lengths for the chip
      ReadUTI(chip, &phases, counts);

#ifdef DEBUG
      Serial.print("Chip: ");
      Serial.print(chip);
      Serial.print(", Mode: ");
      Serial.println(mode);
#endif

#ifdef DEBUG
      Serial.print("Phases detected: ");
      Serial.println(phases);

      Serial.print("Counts: ");
      for(int i = 0; i < phases; i++) {
        Serial.print(counts[i] * CLK_TIME, 1);
        Serial.print(" ns, ");
      }

      Serial.println("");
#endif

      if (!phases) {
#ifdef DEBUG0
        Serial.println("ERROR: Chip read error or not detected");
#endif
        status |= STATUS_BADREAD;

      };  // Readout error. We'll try the next chip

      // Check that we got the right number of phases corresponding to the mode
      if (mode == 3) {
        // This is mode 3 - we expect a 3-phase cycle
        if (phases != 3) {
#ifdef DEBUG0
          Serial.println("ERROR: Wrong number of phases for 3-phase cycle");
#endif
          status |= STATUS_BADPHASES;
        }

      } else {
        // Modes 1 or 2 are 5-phase cycles
        if (phases != 5) {
#ifdef DEBUG0
          Serial.println("ERROR: Wrong number of phases for 5-phase cycle");
#endif
          status |= STATUS_BADPHASES;
        }
      }

      if (status == STATUS_OK) {
        baseline = counts[0]; // Chip baseline (open input B)
        ref = counts[1];      // Reference capacitor (input C)
        val = counts[2];      // Sensor value (input D)

        if (ref != baseline) {  // Make sure we don't divide by 0
          ratio = (float)(val - baseline) / (float)(ref - baseline);          
        } else {
          ratio = 0;
          status |= STATUS_DIVZERO;
        }

        if (ratio < 0) {  // Clamp lower limit of ratio to 0
          ratio = 0;
          status |= STATUS_CLIPPED_LOW;
        }
      }

#ifdef DEBUG
      Serial.print("Ratio: ");
      Serial.println(ratio, 5);
#endif

      // Update Modbus buffer
      enc_float(&inputregs[0 * NCHIPS + 2 * chip], ratio);
      enc_float(&inputregs[2 * NCHIPS + 2 * chip], baseline * CLK_TIME);
      enc_float(&inputregs[4 * NCHIPS + 2 * chip], ref * CLK_TIME);
      enc_float(&inputregs[6 * NCHIPS + 2 * chip], val * CLK_TIME);
      
      inputregs[8 * NCHIPS + chip] = status;
    }
  }

  // Output to Modbus
  modbusTCPServer.writeInputRegisters(0, inputregs, 9 * NCHIPS);
  serveNetwork();

#ifdef DEBUG1
  Serial.print("Modbus Input Registers: ");
  for(unsigned i = 0; i < 9 * NCHIPS; i++) {
    Serial.print(inputregs[i]);
    Serial.print(" ");
  }

  Serial.println();

  Serial.print("Available RAM: ");
  Serial.print(freeram());
  Serial.println(" bytes");

  printISRbufs();
#endif

  // Save config to EEPROM and output to network
  EEPROM_write_conf();
  serveNetwork();
}

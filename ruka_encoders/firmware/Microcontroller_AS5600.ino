/*
 * AS5600 Magnetic Position Sensor with TCA9548A Multiplexer
 * CALIBRATION FIRMWARE — augments the streaming firmware with a command interface.
 *
 * Target:  ESP32 QT Py (Adafruit ESP32-PICO-D4)
 * I2C bus: Wire1 (STEMMA QT connector)
 *
 * Serial commands (send a single ASCII character, 115200 baud):
 *   'r' / 'R'  — one-shot read of all active sensors (one line)
 *   's' / 'S'  — toggle continuous streaming ON / OFF (starts ON)
 *   'h' / 'H'  — print help text
 *
 * Output format (one sensor per field, fields tab-separated, line ends with '\n'):
 *   S<N>:<deg>:<raw>  where deg is 0.00–360.00 and raw is 0–4095
 *   Example: S0:123.45:1405   S1:200.10:2275   S2:300.88:3424
 *
 * Required libraries (install via Arduino Library Manager):
 *   - Adafruit AS5600
 *   - Wire (built-in)
 */

#include <Wire.h>
#include <Adafruit_AS5600.h>

// ── Configuration ────────────────────────────────────────────────────────────

#define TCAADDR          0x70   // TCA9548A I2C address
#define NUMBER_OF_SENSORS 7     // how many AS5600 channels are wired up (0 … N-1)
#define AS5600_ADDR      0x36   // fixed AS5600 I2C address
#define SERIAL_BAUD      115200
#define STREAM_DELAY_MS  100    // milliseconds between streaming lines

// ── Globals ──────────────────────────────────────────────────────────────────

Adafruit_AS5600 as5600;
bool sensorIsInitialized[NUMBER_OF_SENSORS];
bool streamingEnabled = true;   // start in streaming mode

// ── Helpers ──────────────────────────────────────────────────────────────────

/** Select a TCA9548A channel (0-7). */
void tcaSelect(uint8_t ch) {
  if (ch > 7) return;
  Wire1.beginTransmission(TCAADDR);
  Wire1.write(1 << ch);
  Wire1.endTransmission();
  delay(5); // brief settle after mux switch
}

/** Read all active sensors and emit one tab-separated line to Serial. */
void readAndPrint() {
  for (uint8_t s = 0; s < NUMBER_OF_SENSORS; s++) {
    tcaSelect(s);

    float  deg = 0.0f;
    uint16_t raw = 0;

    if (sensorIsInitialized[s]) {
      raw = as5600.getRawAngle();           // 0 – 4095
      deg = (raw * 360.0f) / 4096.0f;      // convert to degrees
    }

    // Format: S<idx>:<degrees>:<raw>
    Serial.print("S");
    Serial.print(s);
    Serial.print(":");
    Serial.print(deg, 2);
    Serial.print(":");
    Serial.print(raw);

    if (s < NUMBER_OF_SENSORS - 1) {
      Serial.print("\t");
    }
  }
  Serial.println(); // end of record
}

/** Print a short usage reminder. */
void printHelp() {
  Serial.println("=== AS5600 Calibration Firmware ===");
  Serial.println("Commands (single ASCII char):");
  Serial.println("  r / R  : one-shot read (all sensors, one line)");
  Serial.println("  s / S  : toggle continuous streaming ON/OFF");
  Serial.println("  h / H  : print this help");
  Serial.print  ("Streaming is currently: ");
  Serial.println(streamingEnabled ? "ON" : "OFF");
  Serial.println("Output: S<N>:<deg>:<raw>  (tab-separated per sensor)");
  Serial.println("===================================");
}

// ── Setup ────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) { /* wait for native USB */ }
  delay(500);

  Serial.println("[BOOT] AS5600 Calibration Firmware");
  printHelp();

  // Initialise Wire1 (STEMMA QT / Qwiic)
  Wire1.begin();
  Wire1.setClock(400000);
  delay(20);

  // Probe each AS5600 through the mux
  for (uint8_t s = 0; s < NUMBER_OF_SENSORS; s++) {
    tcaSelect(s);
    delay(50);
    if (!as5600.begin(AS5600_ADDR, &Wire1)) {
      Serial.print("[INIT] Sensor ");
      Serial.print(s);
      Serial.println(" FAILED — check wiring");
      sensorIsInitialized[s] = false;
    } else {
      Serial.print("[INIT] Sensor ");
      Serial.print(s);
      Serial.println(" OK");
      sensorIsInitialized[s] = true;
    }
    delay(10);
  }

  Serial.println("[BOOT] Starting streaming (send 'h' for help)...");
}

// ── Loop ─────────────────────────────────────────────────────────────────────

void loop() {
  // Handle incoming commands (non-blocking)
  while (Serial.available() > 0) {
    char cmd = (char)Serial.read();
    switch (cmd) {
      case 'r': case 'R':
        readAndPrint();
        break;
      case 's': case 'S':
        streamingEnabled = !streamingEnabled;
        Serial.print("[CMD] Streaming: ");
        Serial.println(streamingEnabled ? "ON" : "OFF");
        break;
      case 'h': case 'H':
        printHelp();
        break;
      default:
        break; // ignore unknown characters
    }
  }

  // Continuous streaming
  if (streamingEnabled) {
    readAndPrint();
    delay(STREAM_DELAY_MS);
  }
}

#include <Arduino.h>

// Set USE_NEOPIXEL_OUTPUT to 1 to drive the signals with a NeoPixel-compatible
// LED strip or matrix instead of discrete outputs. With NeoPixels you can map
// each aspect (red / amber / green / walk / don't walk) to any contiguous run
// of pixels, making it easy to adapt the sketch to rings, bars or other
// form-factors. When disabled the sketch behaves like the original discrete LED
// version.
#define USE_NEOPIXEL_OUTPUT 0

#if USE_NEOPIXEL_OUTPUT
#include <Adafruit_NeoPixel.h>

// ESP32 friendly defaults for a NeoPixel strip. Adjust NEOPIXEL_LED_COUNT and
// the ranges below to suit your hardware. The defaults assume a 16 pixel bar
// where each aspect is represented by a single pixel, matching the Australian
// standard of two-aspect pedestrian heads (red/green only) and three-aspect
// vehicle heads (red/amber/green).
const uint8_t NEOPIXEL_DATA_PIN = 15;
const uint16_t NEOPIXEL_LED_COUNT = 16;
const uint8_t NEOPIXEL_BRIGHTNESS = 80;

Adafruit_NeoPixel gTrafficStrip(NEOPIXEL_LED_COUNT, NEOPIXEL_DATA_PIN,
                                NEO_GRB + NEO_KHZ800);
#endif

struct LightChannel {
  uint8_t pin;
  uint16_t pixelStart;
  uint16_t pixelLength;
};

struct SignalGroup {
  LightChannel red;
  LightChannel amber;
  LightChannel green;
};

struct PedestrianSignal {
  LightChannel dontWalk;
  LightChannel walk;
};

struct LightColor {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

constexpr uint8_t UNUSED_PIN = 0xFF;

constexpr LightChannel makeDigitalChannel(uint8_t pin) {
  return {pin, 0u, 0u};
}

constexpr LightChannel makeNeoPixelChannel(uint16_t start, uint16_t length) {
  return {UNUSED_PIN, start, length};
}

// Use makeCombinedChannel if you need to mirror an aspect on both a discrete
// output pin and a NeoPixel segment at the same time.
constexpr LightChannel makeCombinedChannel(uint8_t pin, uint16_t start,
                                           uint16_t length) {
  return {pin, start, length};
}

// Colours used when NeoPixels are enabled. Adjust to match your preferred look.
const LightColor VEHICLE_RED_COLOR = {255, 0, 0};
const LightColor VEHICLE_AMBER_COLOR = {255, 120, 0};
const LightColor VEHICLE_GREEN_COLOR = {0, 255, 0};
const LightColor PEDESTRIAN_DONT_WALK_COLOR = {255, 0, 0};
const LightColor PEDESTRIAN_WALK_COLOR = {0, 255, 0};

enum VehicleAspect {
  VEHICLE_OFF,
  VEHICLE_RED,
  VEHICLE_RED_AMBER,
  VEHICLE_GREEN,
  VEHICLE_AMBER
};

enum PedestrianAspect {
  PED_OFF,
  PED_DONT_WALK,
  PED_WALK  // Australian standard pedestrian lanterns are two-aspect (red/green)
};

#if USE_NEOPIXEL_OUTPUT
constexpr SignalGroup MAIN_THROUGH_SIGNAL = {makeNeoPixelChannel(0, 1),
                                             makeNeoPixelChannel(1, 1),
                                             makeNeoPixelChannel(2, 1)};
constexpr SignalGroup MAIN_TURN_SIGNAL = {makeNeoPixelChannel(3, 1),
                                          makeNeoPixelChannel(4, 1),
                                          makeNeoPixelChannel(5, 1)};
constexpr SignalGroup SIDE_THROUGH_SIGNAL = {makeNeoPixelChannel(6, 1),
                                             makeNeoPixelChannel(7, 1),
                                             makeNeoPixelChannel(8, 1)};
constexpr SignalGroup SIDE_TURN_SIGNAL = {makeNeoPixelChannel(9, 1),
                                          makeNeoPixelChannel(10, 1),
                                          makeNeoPixelChannel(11, 1)};

constexpr PedestrianSignal PEDESTRIAN_ACROSS_MAIN = {makeNeoPixelChannel(12, 1),
                                                     makeNeoPixelChannel(13, 1)};
constexpr PedestrianSignal PEDESTRIAN_ACROSS_SIDE = {makeNeoPixelChannel(14, 1),
                                                     makeNeoPixelChannel(15, 1)};

constexpr bool channelWithinStrip(const LightChannel &channel) {
  return channel.pixelLength == 0 ||
         (channel.pixelStart + channel.pixelLength) <= NEOPIXEL_LED_COUNT;
}

constexpr bool validateSignalGroup(const SignalGroup &group) {
  return channelWithinStrip(group.red) && channelWithinStrip(group.amber) &&
         channelWithinStrip(group.green);
}

constexpr bool validatePedestrianSignal(const PedestrianSignal &signal) {
  return channelWithinStrip(signal.dontWalk) &&
         channelWithinStrip(signal.walk);
}

static_assert(validateSignalGroup(MAIN_THROUGH_SIGNAL),
              "Main through NeoPixel range exceeds strip length");
static_assert(validateSignalGroup(MAIN_TURN_SIGNAL),
              "Main turn NeoPixel range exceeds strip length");
static_assert(validateSignalGroup(SIDE_THROUGH_SIGNAL),
              "Side through NeoPixel range exceeds strip length");
static_assert(validateSignalGroup(SIDE_TURN_SIGNAL),
              "Side turn NeoPixel range exceeds strip length");
static_assert(validatePedestrianSignal(PEDESTRIAN_ACROSS_MAIN),
              "Pedestrian across main NeoPixel range exceeds strip length");
static_assert(validatePedestrianSignal(PEDESTRIAN_ACROSS_SIDE),
              "Pedestrian across side NeoPixel range exceeds strip length");

#if defined(ARDUINO_ARCH_ESP32)
const uint8_t FAULT_INPUT_PIN = 4;
const bool FAULT_INPUT_HAS_INTERNAL_PULLUP = true;
#else
const uint8_t FAULT_INPUT_PIN = A4;
const bool FAULT_INPUT_HAS_INTERNAL_PULLUP = true;
#endif

#else  // USE_NEOPIXEL_OUTPUT

#if defined(ARDUINO_ARCH_ESP32)
// Example ESP32 mapping using only safe-to-drive GPIOs. Update the pin numbers
// or replace individual channels with makeNeoPixelChannel() if you prefer to
// drive rings, bars or other NeoPixel assemblies.
const SignalGroup MAIN_THROUGH_SIGNAL = {makeDigitalChannel(33),
                                         makeDigitalChannel(32),
                                         makeDigitalChannel(25)};
const SignalGroup MAIN_TURN_SIGNAL = {makeDigitalChannel(26),
                                      makeDigitalChannel(27),
                                      makeDigitalChannel(14)};
const SignalGroup SIDE_THROUGH_SIGNAL = {makeDigitalChannel(21),
                                         makeDigitalChannel(22),
                                         makeDigitalChannel(23)};
const SignalGroup SIDE_TURN_SIGNAL = {makeDigitalChannel(19),
                                      makeDigitalChannel(18),
                                      makeDigitalChannel(5)};

const PedestrianSignal PEDESTRIAN_ACROSS_MAIN = {makeDigitalChannel(16),
                                                 makeDigitalChannel(17)};
const PedestrianSignal PEDESTRIAN_ACROSS_SIDE = {makeDigitalChannel(12),
                                                 makeDigitalChannel(13)};

const uint8_t FAULT_INPUT_PIN = 4;
const bool FAULT_INPUT_HAS_INTERNAL_PULLUP = true;
#else
// Vehicle signal pin assignments for a dual-carriageway intersection (Arduino
// Uno / Mega defaults). Replace with makeNeoPixelChannel(...) to re-map any
// aspect to a NeoPixel segment when USE_NEOPIXEL_OUTPUT is enabled.
const SignalGroup MAIN_THROUGH_SIGNAL = {makeDigitalChannel(2),
                                         makeDigitalChannel(3),
                                         makeDigitalChannel(4)};
const SignalGroup MAIN_TURN_SIGNAL = {makeDigitalChannel(5),
                                      makeDigitalChannel(6),
                                      makeDigitalChannel(7)};
const SignalGroup SIDE_THROUGH_SIGNAL = {makeDigitalChannel(8),
                                         makeDigitalChannel(9),
                                         makeDigitalChannel(10)};
const SignalGroup SIDE_TURN_SIGNAL = {makeDigitalChannel(11),
                                      makeDigitalChannel(12),
                                      makeDigitalChannel(13)};

const PedestrianSignal PEDESTRIAN_ACROSS_MAIN = {makeDigitalChannel(A0),
                                                 makeDigitalChannel(A1)};
const PedestrianSignal PEDESTRIAN_ACROSS_SIDE = {makeDigitalChannel(A2),
                                                 makeDigitalChannel(A3)};

const uint8_t FAULT_INPUT_PIN = A4;
const bool FAULT_INPUT_HAS_INTERNAL_PULLUP = true;
#endif


const SignalGroup VEHICLE_GROUPS[] = {
  MAIN_THROUGH_SIGNAL,
  MAIN_TURN_SIGNAL,
  SIDE_THROUGH_SIGNAL,
  SIDE_TURN_SIGNAL
};
const size_t VEHICLE_GROUP_COUNT = sizeof(VEHICLE_GROUPS) / sizeof(VEHICLE_GROUPS[0]);

const PedestrianSignal PEDESTRIAN_GROUPS[] = {
  PEDESTRIAN_ACROSS_MAIN,
  PEDESTRIAN_ACROSS_SIDE
};
const size_t PEDESTRIAN_GROUP_COUNT = sizeof(PEDESTRIAN_GROUPS) / sizeof(PEDESTRIAN_GROUPS[0]);

// Timing constants (milliseconds) aligned with common AS 1742.14 practice.
const unsigned long STARTUP_FLASH_DURATION_MS = 10000;
const unsigned long FLASH_INTERVAL_MS = 500;
const unsigned long RED_AMBER_DURATION_MS = 1500;
const unsigned long AMBER_DURATION_MS = 3000;
const unsigned long ALL_RED_CLEARANCE_MS = 2000;

const unsigned long MAIN_TURN_GREEN_DURATION_MS = 7000;
const unsigned long MAIN_THROUGH_GREEN_DURATION_MS = 25000;
const unsigned long SIDE_TURN_GREEN_DURATION_MS = 7000;
const unsigned long SIDE_THROUGH_GREEN_DURATION_MS = 20000;

const unsigned long PEDESTRIAN_WALK_DURATION_MS = 7000;
const unsigned int PEDESTRIAN_FLASH_COUNT = 6;
const unsigned long PEDESTRIAN_FLASH_INTERVAL_MS = 500;

uint32_t toNeoPixelColor(const LightColor &color) {
#if USE_NEOPIXEL_OUTPUT
  return gTrafficStrip.Color(color.red, color.green, color.blue);
#else
  (void)color;
  return 0;
#endif
}

void applyLightChannel(const LightChannel &channel, bool on,
                       const LightColor &color) {
  if (channel.pin != UNUSED_PIN) {
    digitalWrite(channel.pin, on ? HIGH : LOW);
  }
#if USE_NEOPIXEL_OUTPUT
  if (channel.pixelLength > 0) {
    const uint32_t packed = on ? toNeoPixelColor(color) : 0;
    const uint32_t start = channel.pixelStart;
    uint32_t end = start + channel.pixelLength;
    if (end > NEOPIXEL_LED_COUNT) {
      end = NEOPIXEL_LED_COUNT;
    }
    for (uint32_t idx = start; idx < end; ++idx) {
      gTrafficStrip.setPixelColor(idx, packed);
    }
  }
#else
  (void)color;
#endif
}

void commitLightChanges() {
#if USE_NEOPIXEL_OUTPUT
  gTrafficStrip.show();
#endif
}

void setVehicleAspect(const SignalGroup &group, VehicleAspect aspect,
                      bool deferCommit = false) {
  applyLightChannel(group.red,
                    aspect == VEHICLE_RED || aspect == VEHICLE_RED_AMBER,
                    VEHICLE_RED_COLOR);
  applyLightChannel(group.amber,
                    aspect == VEHICLE_AMBER || aspect == VEHICLE_RED_AMBER,
                    VEHICLE_AMBER_COLOR);
  applyLightChannel(group.green, aspect == VEHICLE_GREEN, VEHICLE_GREEN_COLOR);

  if (!deferCommit) {
    commitLightChanges();
  }
}

void setAllVehicleAspect(VehicleAspect aspect) {
  for (size_t i = 0; i < VEHICLE_GROUP_COUNT; ++i) {
    setVehicleAspect(VEHICLE_GROUPS[i], aspect, true);
  }
  commitLightChanges();
}

void setPedestrianAspect(const PedestrianSignal &signal, PedestrianAspect aspect,
                         bool deferCommit = false) {
  switch (aspect) {
    case PED_WALK:
      applyLightChannel(signal.walk, true, PEDESTRIAN_WALK_COLOR);
      applyLightChannel(signal.dontWalk, false, PEDESTRIAN_DONT_WALK_COLOR);
      break;
    case PED_DONT_WALK:
      applyLightChannel(signal.walk, false, PEDESTRIAN_WALK_COLOR);
      applyLightChannel(signal.dontWalk, true, PEDESTRIAN_DONT_WALK_COLOR);
      break;
    default:
      applyLightChannel(signal.walk, false, PEDESTRIAN_WALK_COLOR);
      applyLightChannel(signal.dontWalk, false, PEDESTRIAN_DONT_WALK_COLOR);
      break;
  }

  if (!deferCommit) {
    commitLightChanges();
  }
}

void setAllPedestrianAspect(PedestrianAspect aspect) {
  for (size_t i = 0; i < PEDESTRIAN_GROUP_COUNT; ++i) {
    setPedestrianAspect(PEDESTRIAN_GROUPS[i], aspect, true);
  }
  commitLightChanges();
}

void flashPedestrianDontWalk(const PedestrianSignal &signal, unsigned int flashes,
                             unsigned long intervalMs) {
  for (unsigned int i = 0; i < flashes; ++i) {
    setPedestrianAspect(signal, PED_DONT_WALK);
    delay(intervalMs);
    setPedestrianAspect(signal, PED_OFF);
    delay(intervalMs);
  }
  setPedestrianAspect(signal, PED_DONT_WALK);
}

unsigned long servePedestrianCrossing(const PedestrianSignal &signal,
                                      unsigned long minimumGreenDurationMs) {
  unsigned long elapsed = 0;
  setPedestrianAspect(signal, PED_WALK);
  delay(PEDESTRIAN_WALK_DURATION_MS);
  elapsed += PEDESTRIAN_WALK_DURATION_MS;

  flashPedestrianDontWalk(signal, PEDESTRIAN_FLASH_COUNT, PEDESTRIAN_FLASH_INTERVAL_MS);
  elapsed += (unsigned long)PEDESTRIAN_FLASH_COUNT * 2UL * PEDESTRIAN_FLASH_INTERVAL_MS;

  if (elapsed < minimumGreenDurationMs) {
    delay(minimumGreenDurationMs - elapsed);
    elapsed = minimumGreenDurationMs;
  }

  return elapsed;
}

void holdAllRedClearance() {
  setAllVehicleAspect(VEHICLE_RED);
  setAllPedestrianAspect(PED_DONT_WALK);
  delay(ALL_RED_CLEARANCE_MS);
}

void runStartupFlash() {
  unsigned long start = millis();
  bool amberOn = false;
  while (millis() - start < STARTUP_FLASH_DURATION_MS) {
    amberOn = !amberOn;
    for (size_t i = 0; i < VEHICLE_GROUP_COUNT; ++i) {
      setVehicleAspect(VEHICLE_GROUPS[i], amberOn ? VEHICLE_AMBER : VEHICLE_OFF,
                       true);
    }
    setAllPedestrianAspect(PED_DONT_WALK);
    delay(FLASH_INTERVAL_MS);
  }
  holdAllRedClearance();
}

void enterFlashingAmberFault() {
  bool amberOn = false;
  while (digitalRead(FAULT_INPUT_PIN) == LOW) {
    amberOn = !amberOn;
    for (size_t i = 0; i < VEHICLE_GROUP_COUNT; ++i) {
      setVehicleAspect(VEHICLE_GROUPS[i], amberOn ? VEHICLE_AMBER : VEHICLE_OFF,
                       true);
    }
    setAllPedestrianAspect(PED_DONT_WALK);
    delay(FLASH_INTERVAL_MS);
  }
  holdAllRedClearance();
}

enum CyclePhase {
  CYCLE_MAIN_TURN,
  CYCLE_MAIN_THROUGH,
  CYCLE_SIDE_TURN,
  CYCLE_SIDE_THROUGH
};

CyclePhase nextCyclePhase(CyclePhase phase) {
  switch (phase) {
    case CYCLE_MAIN_TURN:
      return CYCLE_MAIN_THROUGH;
    case CYCLE_MAIN_THROUGH:
      return CYCLE_SIDE_TURN;
    case CYCLE_SIDE_TURN:
      return CYCLE_SIDE_THROUGH;
    default:
      return CYCLE_MAIN_TURN;
  }
}

void runPhase(CyclePhase phase);

void runMainTurnPhase() {
  holdAllRedClearance();
  setVehicleAspect(MAIN_TURN_SIGNAL, VEHICLE_RED_AMBER);
  delay(RED_AMBER_DURATION_MS);
  setVehicleAspect(MAIN_TURN_SIGNAL, VEHICLE_GREEN);
  delay(MAIN_TURN_GREEN_DURATION_MS);
  setVehicleAspect(MAIN_TURN_SIGNAL, VEHICLE_AMBER);
  delay(AMBER_DURATION_MS);
  setVehicleAspect(MAIN_TURN_SIGNAL, VEHICLE_RED);
}

void runMainThroughPhase() {
  holdAllRedClearance();
  setVehicleAspect(MAIN_THROUGH_SIGNAL, VEHICLE_RED_AMBER);
  delay(RED_AMBER_DURATION_MS);
  setVehicleAspect(MAIN_THROUGH_SIGNAL, VEHICLE_GREEN);
  unsigned long served = servePedestrianCrossing(PEDESTRIAN_ACROSS_SIDE,
                                                 MAIN_THROUGH_GREEN_DURATION_MS);
  (void)served; // servePedestrianCrossing guarantees the minimum green was met.
  setVehicleAspect(MAIN_THROUGH_SIGNAL, VEHICLE_AMBER);
  delay(AMBER_DURATION_MS);
  setVehicleAspect(MAIN_THROUGH_SIGNAL, VEHICLE_RED);
}

void runSideTurnPhase() {
  holdAllRedClearance();
  setVehicleAspect(SIDE_TURN_SIGNAL, VEHICLE_RED_AMBER);
  delay(RED_AMBER_DURATION_MS);
  setVehicleAspect(SIDE_TURN_SIGNAL, VEHICLE_GREEN);
  delay(SIDE_TURN_GREEN_DURATION_MS);
  setVehicleAspect(SIDE_TURN_SIGNAL, VEHICLE_AMBER);
  delay(AMBER_DURATION_MS);
  setVehicleAspect(SIDE_TURN_SIGNAL, VEHICLE_RED);
}

void runSideThroughPhase() {
  holdAllRedClearance();
  setVehicleAspect(SIDE_THROUGH_SIGNAL, VEHICLE_RED_AMBER);
  delay(RED_AMBER_DURATION_MS);
  setVehicleAspect(SIDE_THROUGH_SIGNAL, VEHICLE_GREEN);
  unsigned long served = servePedestrianCrossing(PEDESTRIAN_ACROSS_MAIN,
                                                 SIDE_THROUGH_GREEN_DURATION_MS);
  (void)served;
  setVehicleAspect(SIDE_THROUGH_SIGNAL, VEHICLE_AMBER);
  delay(AMBER_DURATION_MS);
  setVehicleAspect(SIDE_THROUGH_SIGNAL, VEHICLE_RED);
}

bool gAutomaticMode = true;
bool gManualPhaseRequested = false;
CyclePhase gRequestedPhase = CYCLE_MAIN_TURN;
CyclePhase gCurrentAutomaticPhase = CYCLE_MAIN_TURN;

void runPhase(CyclePhase phase) {
  switch (phase) {
    case CYCLE_MAIN_TURN:
      runMainTurnPhase();
      break;
    case CYCLE_MAIN_THROUGH:
      runMainThroughPhase();
      break;
    case CYCLE_SIDE_TURN:
      runSideTurnPhase();
      break;
    case CYCLE_SIDE_THROUGH:
      runSideThroughPhase();
      break;
  }
}

void printSerialHelp() {
  Serial.println(F("Traffic lights serial control commands:"));
  Serial.println(F("  a - resume automatic cycle"));
  Serial.println(F("  m - enter manual mode (all red until a phase is requested)"));
  Serial.println(F("  1 - serve main turn phase (manual mode)"));
  Serial.println(F("  2 - serve main through phase (manual mode)"));
  Serial.println(F("  3 - serve side turn phase (manual mode)"));
  Serial.println(F("  4 - serve side through phase (manual mode)"));
  Serial.println(F("  ? - show this help"));
}

void handleSerialCommand(char command) {
  switch (command) {
    case 'a':
    case 'A':
      if (!gAutomaticMode) {
        gAutomaticMode = true;
        gManualPhaseRequested = false;
        gCurrentAutomaticPhase = CYCLE_MAIN_TURN;
        Serial.println(F("Automatic cycle resumed."));
      }
      break;
    case 'm':
    case 'M':
      if (gAutomaticMode) {
        gAutomaticMode = false;
        gManualPhaseRequested = false;
        holdAllRedClearance();
        Serial.println(F("Manual mode enabled. All signals set to red."));
      }
      break;
    case '1':
      if (!gAutomaticMode) {
        gRequestedPhase = CYCLE_MAIN_TURN;
        gManualPhaseRequested = true;
        Serial.println(F("Manual: serving main turn phase."));
      }
      break;
    case '2':
      if (!gAutomaticMode) {
        gRequestedPhase = CYCLE_MAIN_THROUGH;
        gManualPhaseRequested = true;
        Serial.println(F("Manual: serving main through phase."));
      }
      break;
    case '3':
      if (!gAutomaticMode) {
        gRequestedPhase = CYCLE_SIDE_TURN;
        gManualPhaseRequested = true;
        Serial.println(F("Manual: serving side turn phase."));
      }
      break;
    case '4':
      if (!gAutomaticMode) {
        gRequestedPhase = CYCLE_SIDE_THROUGH;
        gManualPhaseRequested = true;
        Serial.println(F("Manual: serving side through phase."));
      }
      break;
    case '?':
      printSerialHelp();
      break;
    default:
      break;
  }
}

void processSerialInput() {
  while (Serial.available() > 0) {
    char incoming = Serial.read();
    if (incoming == '\r' || incoming == '\n') {
      continue;
    }
    handleSerialCommand(incoming);
  }
}

void setup() {
  auto initializeLightChannel = [](const LightChannel &channel) {
    if (channel.pin != UNUSED_PIN) {
      pinMode(channel.pin, OUTPUT);
      digitalWrite(channel.pin, LOW);
    }
  };

  for (size_t i = 0; i < VEHICLE_GROUP_COUNT; ++i) {
    initializeLightChannel(VEHICLE_GROUPS[i].red);
    initializeLightChannel(VEHICLE_GROUPS[i].amber);
    initializeLightChannel(VEHICLE_GROUPS[i].green);
  }
  for (size_t i = 0; i < PEDESTRIAN_GROUP_COUNT; ++i) {
    initializeLightChannel(PEDESTRIAN_GROUPS[i].dontWalk);
    initializeLightChannel(PEDESTRIAN_GROUPS[i].walk);
  }

#if USE_NEOPIXEL_OUTPUT
  gTrafficStrip.begin();
  gTrafficStrip.setBrightness(NEOPIXEL_BRIGHTNESS);
  gTrafficStrip.clear();
  gTrafficStrip.show();
#endif

  if (FAULT_INPUT_HAS_INTERNAL_PULLUP) {
    pinMode(FAULT_INPUT_PIN, INPUT_PULLUP);
  } else {
    pinMode(FAULT_INPUT_PIN, INPUT);
  }

  Serial.begin(115200);
  Serial.println();
  Serial.println(F("Traffic light controller ready."));
  printSerialHelp();

  setAllVehicleAspect(VEHICLE_RED);
  setAllPedestrianAspect(PED_DONT_WALK);
  runStartupFlash();
}

void loop() {
  processSerialInput();

  if (digitalRead(FAULT_INPUT_PIN) == LOW) {
    enterFlashingAmberFault();
    return;
  }

  if (!gAutomaticMode) {
    if (gManualPhaseRequested) {
      gManualPhaseRequested = false;
      runPhase(gRequestedPhase);
      processSerialInput();
    }
    return;
  }

  runPhase(gCurrentAutomaticPhase);
  gCurrentAutomaticPhase = nextCyclePhase(gCurrentAutomaticPhase);
  processSerialInput();
}

#include <Arduino.h>

struct SignalGroup {
  uint8_t redPin;
  uint8_t amberPin;
  uint8_t greenPin;
};

struct PedestrianSignal {
  uint8_t dontWalkPin;
  uint8_t walkPin;
};

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
  PED_WALK
};

// Vehicle signal pin assignments for a dual-carriageway intersection.
const SignalGroup MAIN_THROUGH_SIGNAL = {2, 3, 4};
const SignalGroup MAIN_TURN_SIGNAL = {5, 6, 7};
const SignalGroup SIDE_THROUGH_SIGNAL = {8, 9, 10};
const SignalGroup SIDE_TURN_SIGNAL = {11, 12, 13};

// Pedestrian lanterns for the two crossings.
const PedestrianSignal PEDESTRIAN_ACROSS_MAIN = {A0, A1};
const PedestrianSignal PEDESTRIAN_ACROSS_SIDE = {A2, A3};

// Fault input (active-low). Pull to ground to trigger fail-safe flashing amber.
const uint8_t FAULT_INPUT_PIN = A4;

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

void setVehicleAspect(const SignalGroup &group, VehicleAspect aspect) {
  digitalWrite(group.redPin,
               (aspect == VEHICLE_RED || aspect == VEHICLE_RED_AMBER) ? HIGH : LOW);
  digitalWrite(group.amberPin,
               (aspect == VEHICLE_AMBER || aspect == VEHICLE_RED_AMBER) ? HIGH : LOW);
  digitalWrite(group.greenPin, aspect == VEHICLE_GREEN ? HIGH : LOW);
}

void setAllVehicleAspect(VehicleAspect aspect) {
  for (size_t i = 0; i < VEHICLE_GROUP_COUNT; ++i) {
    setVehicleAspect(VEHICLE_GROUPS[i], aspect);
  }
}

void setPedestrianAspect(const PedestrianSignal &signal, PedestrianAspect aspect) {
  switch (aspect) {
    case PED_WALK:
      digitalWrite(signal.walkPin, HIGH);
      digitalWrite(signal.dontWalkPin, LOW);
      break;
    case PED_DONT_WALK:
      digitalWrite(signal.walkPin, LOW);
      digitalWrite(signal.dontWalkPin, HIGH);
      break;
    default:
      digitalWrite(signal.walkPin, LOW);
      digitalWrite(signal.dontWalkPin, LOW);
      break;
  }
}

void setAllPedestrianAspect(PedestrianAspect aspect) {
  for (size_t i = 0; i < PEDESTRIAN_GROUP_COUNT; ++i) {
    setPedestrianAspect(PEDESTRIAN_GROUPS[i], aspect);
  }
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
      setVehicleAspect(VEHICLE_GROUPS[i], amberOn ? VEHICLE_AMBER : VEHICLE_OFF);
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
      setVehicleAspect(VEHICLE_GROUPS[i], amberOn ? VEHICLE_AMBER : VEHICLE_OFF);
    }
    setAllPedestrianAspect(PED_DONT_WALK);
    delay(FLASH_INTERVAL_MS);
  }
  holdAllRedClearance();
}

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

void setup() {
  for (size_t i = 0; i < VEHICLE_GROUP_COUNT; ++i) {
    pinMode(VEHICLE_GROUPS[i].redPin, OUTPUT);
    pinMode(VEHICLE_GROUPS[i].amberPin, OUTPUT);
    pinMode(VEHICLE_GROUPS[i].greenPin, OUTPUT);
  }
  for (size_t i = 0; i < PEDESTRIAN_GROUP_COUNT; ++i) {
    pinMode(PEDESTRIAN_GROUPS[i].dontWalkPin, OUTPUT);
    pinMode(PEDESTRIAN_GROUPS[i].walkPin, OUTPUT);
  }
  pinMode(FAULT_INPUT_PIN, INPUT_PULLUP);

  setAllVehicleAspect(VEHICLE_RED);
  setAllPedestrianAspect(PED_DONT_WALK);
  runStartupFlash();
}

void loop() {
  if (digitalRead(FAULT_INPUT_PIN) == LOW) {
    enterFlashingAmberFault();
    return;
  }

  runMainTurnPhase();
  runMainThroughPhase();
  runSideTurnPhase();
  runSideThroughPhase();
}

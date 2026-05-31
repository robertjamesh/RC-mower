#include <Bluepad32.h>

// --- PIN DEFINITIONS ---
const int pwmPins[4]   = {16, 23, 4, 22};  // FL, RL, FR, RR
const int speedPins[4] = {35, 32, 34, 33};  // FL, RL, FR, RR (unused, kept for rewire)

const int leftReverseRelay  = 18;
const int rightReverseRelay = 17;
const int bladeRelayPin     = 13;

// =============================================================
// INVERT FLAGS
// If throttle feels backwards change THROTTLE_DIR to -1
// If steering feels backwards change STEER_DIR to -1
// =============================================================
const int THROTTLE_DIR = 1;
const int STEER_DIR    = -1;

// --- CONTROLLER ---
GamepadPtr myGamepad = nullptr;

// --- SAFETY ---
unsigned long lastPacketTime    = 0;
unsigned long lastLoopTime      = 0;
unsigned long lastThrottleTime  = 0;
bool systemArmed                = false;
bool lastCrossState             = false;
const unsigned long INACTIVITY_TIMEOUT = 20000UL;

// --- BLADE ---
bool relayState             = false;
bool lastOptionsState       = false;

// --- SPEED CEILING ---
int  maxSpeedPercent = 5;
bool lastR1State     = false;
bool lastL1State     = false;

// --- RELAY STATE ---
bool leftRelayState  = false;
bool rightRelayState = false;

// --- RAMPED SMOOTHING ---
float rampedSpeeds[4] = {0, 0, 0, 0};
int   targetSpeeds[4] = {0, 0, 0, 0};

const float RAMP_UP   = 80.0f;
const float RAMP_DOWN = 65.0f;

// Steering boost at low speed
const float STEER_BOOST_MAX  = 2.5f;
const float STEER_BOOST_MIN  = 1.0f;
const int   STEER_BOOST_CEIL = 60;

// =============================================================
// RUMBLE FLAGS
// Triggered from loop() not callback to avoid blocking BT
// =============================================================
volatile bool rumbleArmedFlag    = false;
volatile bool rumbleDisarmedFlag = false;

// =============================================================
// HELPERS
// =============================================================
void rotateMotorRaw(int idx, int pwm) {
  ledcWrite(pwmPins[idx], pwm);
}

void stopAll() {
  for (int i = 0; i < 4; i++) {
    rotateMotorRaw(i, 0);
    rampedSpeeds[i] = 0;
    targetSpeeds[i] = 0;
  }
  digitalWrite(leftReverseRelay,  LOW);
  digitalWrite(rightReverseRelay, LOW);
  leftRelayState  = false;
  rightRelayState = false;
  relayState      = false;
  digitalWrite(bladeRelayPin, LOW);
  lastOptionsState = false;
}

void setLED(uint8_t r, uint8_t g, uint8_t b) {
  if (myGamepad && myGamepad->isConnected()) {
    myGamepad->setColorLED(r, g, b);
  }
}

// 3 short rumbles for armed, 1 long for disarmed
// Runs in loop() context via flags
void doRumbleArmed() {
  if (!myGamepad || !myGamepad->isConnected()) return;
  for (int i = 0; i < 3; i++) {
    myGamepad->playDualRumble(0, 150, 0, 200);
    delay(150);
    myGamepad->playDualRumble(0, 0, 0, 0); // stop
    delay(80);
  }
}

void doRumbleDisarmed() {
  if (!myGamepad || !myGamepad->isConnected()) return;
  myGamepad->playDualRumble(0, 500, 0, 200);
  delay(500);
  myGamepad->playDualRumble(0, 0, 0, 0); // stop
}

// =============================================================
// BLUEPAD32 CALLBACKS
// =============================================================
void onConnectedGamepad(GamepadPtr gp) {
  myGamepad = gp;
  Serial.println("Controller Connected.");
  // Green LED on connect — not yet armed
  gp->setColorLED(0, 255, 0);
}

void onDisconnectedGamepad(GamepadPtr gp) {
  Serial.println("Controller Disconnected.");
  if (systemArmed) {
    stopAll();
    systemArmed = false;
    Serial.println("DISARMED — controller lost");
  }
  myGamepad = nullptr;
}

// =============================================================
// PROCESS GAMEPAD — called from loop() after BP32.update()
// =============================================================
void processGamepad(GamepadPtr gp) {
  lastPacketTime = millis();

  // --- CROSS = arm/disarm toggle ---
  // In Bluepad32, PS5 Cross = button A
  bool crossNow = gp->a();
  if (crossNow && !lastCrossState) {
    systemArmed = !systemArmed;
    if (!systemArmed) {
      stopAll();
      rumbleDisarmedFlag = true;
      setLED(0, 255, 0); // Green = safe
      Serial.println("DISARMED");
    } else {
      lastThrottleTime = millis();
      rumbleArmedFlag  = true;
      setLED(255, 0, 0); // Red = armed
      Serial.println("ARMED");
    }
  }
  lastCrossState = crossNow;

  if (!systemArmed) return;

  // --- R1 / L1 — speed ceiling steps: 5, 10, 20, 40, 60, 80 ---
  const int speedSteps[] = {5, 10, 20, 40, 60, 80};
  const int numSteps = 6;

  bool r1Now = gp->r1();
  if (r1Now && !lastR1State) {
    for (int s = 0; s < numSteps - 1; s++) {
      if (maxSpeedPercent == speedSteps[s]) {
        maxSpeedPercent = speedSteps[s + 1];
        break;
      }
    }
    Serial.print("Speed: "); Serial.print(maxSpeedPercent); Serial.println("%");
  }
  lastR1State = r1Now;

  bool l1Now = gp->l1();
  if (l1Now && !lastL1State) {
    for (int s = numSteps - 1; s > 0; s--) {
      if (maxSpeedPercent == speedSteps[s]) {
        maxSpeedPercent = speedSteps[s - 1];
        break;
      }
    }
    Serial.print("Speed: "); Serial.print(maxSpeedPercent); Serial.println("%");
  }
  lastL1State = l1Now;

  // --- OPTIONS button = blade toggle ---
  // In Bluepad32, Options = miscButtons() & 0x02
  bool optionsNow = (gp->miscButtons() & 0x02);
  if (optionsNow && !lastOptionsState) {
    relayState = !relayState;
    digitalWrite(bladeRelayPin, relayState ? HIGH : LOW);
    Serial.print("Blades: "); Serial.println(relayState ? "RUNNING" : "STOPPED");
  }
  lastOptionsState = optionsNow;

  // --- THROTTLE & STEERING ---
  // brake() = L2 (0-1023), throttle() = R2 (0-1023)
  // Map to -255..255 range then apply direction flag
  int l2 = map(gp->brake(),    0, 1023, 0, 255);
  int r2 = map(gp->throttle(), 0, 1023, 0, 255);
  int throttleRaw = (l2 - r2) * THROTTLE_DIR;

  // axisX() range is -511 to 512 — map to -255..255
  int steeringRaw = map(gp->axisX(), -511, 512, -255, 255) * STEER_DIR;

  // Steering deadzone
  if (abs(steeringRaw) < 20) steeringRaw = 0;

  float speedMult = maxSpeedPercent / 100.0f;
  int throttle    = (int)(throttleRaw * speedMult);

  // Reset inactivity timer on meaningful throttle input
  if (abs(throttleRaw) > 10) lastThrottleTime = millis();

  // Steering range scales with speed ceiling
  int maxSteer = map(maxSpeedPercent, 5, 80, 50, 200);
  int steering = map(steeringRaw, -255, 255, -maxSteer, maxSteer);
  steering     = (int)(steering * speedMult);

  // Steering boost at low speed
  float steerBoost = 1.0f;
  if (maxSpeedPercent < STEER_BOOST_CEIL) {
    steerBoost = map(maxSpeedPercent, 5, STEER_BOOST_CEIL,
                     (int)(STEER_BOOST_MAX * 100), (int)(STEER_BOOST_MIN * 100)) / 100.0f;
  }
  steering = (int)(steering * steerBoost);

  // Tank mix — all four outputs kept so 4WD restore is plug and play
  targetSpeeds[0] = throttle + steering; // FL (no ESC currently)
  targetSpeeds[1] = throttle + steering; // RL
  targetSpeeds[2] = throttle - steering; // FR
  targetSpeeds[3] = throttle - steering; // RR
}

// =============================================================
// SETUP
// =============================================================
void setup() {
  Serial.begin(115200);

  pinMode(leftReverseRelay,  OUTPUT);
  pinMode(rightReverseRelay, OUTPUT);
  pinMode(bladeRelayPin,     OUTPUT);
  digitalWrite(leftReverseRelay,  LOW);
  digitalWrite(rightReverseRelay, LOW);
  digitalWrite(bladeRelayPin,     LOW);

  for (int i = 0; i < 4; i++) {
    pinMode(pwmPins[i], OUTPUT);
    ledcAttach(pwmPins[i], 20000, 8); // 20kHz
    rotateMotorRaw(i, 0);
  }

  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  // Forget previously paired controllers so fresh pair is always clean

  Serial.println("Mower Ready. Connect controller then press Cross to arm.");
}

// =============================================================
// LOOP — 20Hz open loop drive engine
// =============================================================
void loop() {
  // Process Bluepad32 — must be called every loop
  bool updated = BP32.update();
  if (updated && myGamepad && myGamepad->isConnected()) {
    processGamepad(myGamepad);
  }

  unsigned long now = millis();

  // Watchdog — controller loss disarms
  if (systemArmed && myGamepad == nullptr) {
    stopAll();
    systemArmed = false;
    Serial.println("CONTROLLER LOST — DISARMED");
  }

  // Inactivity timeout — disarm after 20 seconds of no throttle
  if (systemArmed && (now - lastThrottleTime > INACTIVITY_TIMEOUT)) {
    stopAll();
    systemArmed = false;
    rumbleDisarmedFlag = true;
    setLED(0, 255, 0);
    Serial.println("INACTIVITY TIMEOUT — DISARMED");
  }

  // Trigger rumble from loop() context
  if (rumbleArmedFlag) {
    rumbleArmedFlag = false;
    doRumbleArmed();
  }
  if (rumbleDisarmedFlag) {
    rumbleDisarmedFlag = false;
    doRumbleDisarmed();
  }

  // 20Hz drive loop
  if (now - lastLoopTime >= 50) {
    lastLoopTime = now;

    if (systemArmed) {

      // Relay direction — only switch on explicit opposite command
      bool leftActive  = (targetSpeeds[0] != 0 || targetSpeeds[1] != 0);
      bool rightActive = (targetSpeeds[2] != 0 || targetSpeeds[3] != 0);
      bool wantLeft    = (targetSpeeds[0] < 0 || targetSpeeds[1] < 0);
      bool wantRight   = (targetSpeeds[2] < 0 || targetSpeeds[3] < 0);

      if (leftActive && wantLeft != leftRelayState) {
        rampedSpeeds[0] = 0;
        rampedSpeeds[1] = 0;
        leftRelayState  = wantLeft;
        digitalWrite(leftReverseRelay, leftRelayState ? HIGH : LOW);
      }
      if (rightActive && wantRight != rightRelayState) {
        rampedSpeeds[2] = 0;
        rampedSpeeds[3] = 0;
        rightRelayState  = wantRight;
        digitalWrite(rightReverseRelay, rightRelayState ? HIGH : LOW);
      }

      // Slew rate ramp + PWM output
      for (int i = 0; i < 4; i++) {
        float diff = targetSpeeds[i] - rampedSpeeds[i];
        if      (diff > 0) rampedSpeeds[i] = min((float)targetSpeeds[i], rampedSpeeds[i] + RAMP_UP);
        else if (diff < 0) rampedSpeeds[i] = max((float)targetSpeeds[i], rampedSpeeds[i] - RAMP_DOWN);

        rotateMotorRaw(i, (rampedSpeeds[i] == 0) ? 0 : abs((int)rampedSpeeds[i]));
      }

    } else {
      stopAll();
    }
  }

  delay(5);
}

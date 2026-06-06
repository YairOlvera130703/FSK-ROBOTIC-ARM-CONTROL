#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/* =============================================================================
   MECHANICAL LAYER: ROBOTIC ARM CONTROL (PCA9685)
   ============================================================================= 
   The Arduino does not drive the motors directly because it lacks sufficient 
   precise PWM pins and current capacity. Instead, it sends instructions via 
   the I2C bus (Pins A4 and A5) to the PCA9685 chip, which generates the perfect 
   PWM signals for the 4 servos independently.
*/
Adafruit_PWMServoDriver pwmController = Adafruit_PWMServoDriver();

// Physical pin assignment on the PCA9685 board
#define BASE_CHANNEL 0
#define SHOULDER_CHANNEL 1
#define ELBOW_CHANNEL 2
#define GRIPPER_CHANNEL 3

// Standard calibration for electromagnetic pulse length.
// 150 = 0 degrees, 600 = 180 degrees (approx, depends on the servo brand).
#define PULSE_MIN 150
#define PULSE_MAX 600

/* --- PHYSICAL SAFETY LIMITS ---
   These are the "invisible walls" of the kinematics. 
   They prevent a software error from forcing the motor beyond the physical 
   plastic structure, preventing gear stripping or controller burnout due to overcurrent.
*/
#define BASE_LIMIT_MIN 0
#define BASE_LIMIT_MAX 149
#define BASE_HOME 90

#define SHOULDER_LIMIT_MIN 70
#define SHOULDER_LIMIT_MAX 138
#define SHOULDER_HOME 70

#define ELBOW_LIMIT_MIN 90
#define ELBOW_LIMIT_MAX 100
#define ELBOW_HOME 90

#define GRIPPER_LIMIT_MIN 20  // Safe maximum opening
#define GRIPPER_LIMIT_MAX 90  // Full closure (pressure)
#define GRIPPER_HOME 90       // We keep the arm closed at rest

/* STATE VECTOR (RAM memory of the motors)
   The Arduino stores the exact current position of each motor here.
   It is vital to know from which degree to start moving in the smooth 
   movement function. Initially, we assume they start at their home position.
*/
int currentAngles[4] = { BASE_HOME, SHOULDER_HOME, ELBOW_HOME, GRIPPER_HOME };

// Mathematical mapping function. Converts our human unit (0-180 degrees)
// to the machine unit (pulse length 150-600) understood by the PCA9685.
long mapAngle(long angle) {
  return map(angle, 0, 180, PULSE_MIN, PULSE_MAX);
}

/* =============================================================================
   SMOOTH TRAJECTORY CONTROL
   If a motor jumps from 0 to 180 abruptly, it consumes a huge current spike 
   and destabilizes the arm. This function interpolates the movement degree by degree.
   ============================================================================= */
void moveServoSmoothly(uint8_t channel, int targetAngle, uint8_t speed_ms) {
  int currentAngle = currentAngles[channel];
  int validatedAngle;

  // 1. PROTECTION FILTER: constrain() forces the targetAngle to never
  // exceed the physical limits calibrated above, regardless of the input.
  if (channel == BASE_CHANNEL) validatedAngle = constrain(targetAngle, BASE_LIMIT_MIN, BASE_LIMIT_MAX);
  else if (channel == SHOULDER_CHANNEL) validatedAngle = constrain(targetAngle, SHOULDER_LIMIT_MIN, SHOULDER_LIMIT_MAX);
  else if (channel == ELBOW_CHANNEL) validatedAngle = constrain(targetAngle, ELBOW_LIMIT_MIN, ELBOW_LIMIT_MAX);
  else if (channel == GRIPPER_CHANNEL) validatedAngle = constrain(targetAngle, GRIPPER_LIMIT_MIN, GRIPPER_LIMIT_MAX);
  else return;  // If the channel is invalid, abort the function.

  // 2. LINEAR INTERPOLATION: Compares the current state with the target.
  // If the target is greater, increment the angle (addition loop).
  if (currentAngle < validatedAngle) {
    for (int a = currentAngle; a <= validatedAngle; a++) {
      pwmController.setPWM(channel, 0, mapAngle(a));
      delay(speed_ms);  // The pause dictates the apparent RPM of the motor
    }
  }
  // If the target is lesser, decrement the angle (subtraction loop).
  else if (currentAngle > validatedAngle) {
    for (int a = currentAngle; a >= validatedAngle; a--) {
      pwmController.setPWM(channel, 0, mapAngle(a));
      delay(speed_ms);
    }
  }

  // 3. UPDATE: Save the new real position in the memory vector.
  currentAngles[channel] = validatedAngle;
}

/* =============================================================================
   KINEMATIC STATE MACHINE (Parameterized)
   =============================================================================
   Picks up an object at the absolute frontal coordinate (90 degrees) and takes 
   it to the 'baseTargetAngle' injected by the main loop, based on the decoded QR.
*/
void objectSortingRoutine(int baseTargetAngle) {
  Serial.println(F(">>> Starting frontal pickup sequence..."));

  // PHASE 1: Alignment. Base to center, open gripper. Delay stabilizes inertia.
  moveServoSmoothly(BASE_CHANNEL, 90, 15);
  moveServoSmoothly(GRIPPER_CHANNEL, 20, 10);
  delay(500);

  // PHASE 2: Z-Axis Approach (Vertical). Lower shoulder and elbow towards the part.
  moveServoSmoothly(SHOULDER_CHANNEL, 138, 20);
  moveServoSmoothly(ELBOW_CHANNEL, 140, 20);
  delay(500);

  // PHASE 3: Grasp. Gripper closes.
  moveServoSmoothly(GRIPPER_CHANNEL, 90, 10);
  delay(1000);  // 1 sec to ensure firm mechanical pressure

  // PHASE 4: Z-Axis Lift. Raise the arm to rotate without scraping the ground.
  moveServoSmoothly(ELBOW_CHANNEL, 90, 20);
  moveServoSmoothly(SHOULDER_CHANNEL, 70, 20);
  delay(500);

  // PHASE 5: Rotation. Turn the base to the corresponding hopper (40, 110, or 140 degrees)
  Serial.print(F(">>> Moving object to sorting angle: "));
  Serial.println(baseTargetAngle);
  moveServoSmoothly(BASE_CHANNEL, baseTargetAngle, 15);
  delay(500);

  // PHASE 6: Descent at Destination. Lower the arm again.
  moveServoSmoothly(SHOULDER_CHANNEL, 138, 20);
  moveServoSmoothly(ELBOW_CHANNEL, 140, 20);
  delay(500);

  // PHASE 7: Release. Open the gripper.
  moveServoSmoothly(GRIPPER_CHANNEL, 20, 10);
  delay(1000);

  // PHASE 8: Return to Home. All motors return to their resting position.
  Serial.println(F(">>> Returning to home position..."));
  moveServoSmoothly(ELBOW_CHANNEL, 90, 20);
  moveServoSmoothly(SHOULDER_CHANNEL, 70, 20);
  moveServoSmoothly(BASE_CHANNEL, 90, 15);
  moveServoSmoothly(GRIPPER_CHANNEL, 90, 10);
}


/* =============================================================================
   TELECOMMUNICATIONS LAYER: FSK RECEIVER (Software UART Demodulator)
   Emulates a serial port by analyzing the voltages of the pure demodulated signal.
   ============================================================================= */

#define FSK_INPUT_PIN 3  // Pin where the pulse train enters from the demodulator (XR2211)

// Timing constants mathematically aligned to the transmitter (~16 baud).
// One bit lasts exactly 62.5 milliseconds.
#define BIT_PERIOD_MS 62UL
#define BIT_PERIOD_US 500UL

// Maximum wait times to know when a word ends
#define WORD_SPACE_MS 1500UL
#define MAX_WORD_LENGTH 64
#define SYNC_CHARACTER '~'
#define CHAR_WAIT_TIME_MS (WORD_SPACE_MS + 300UL)

/* DIGITAL DEBOUNCE
   We take "n" samples in 1 millisecond. If more than half of the samples 
   are HIGH (5V), then we confirm that the real level is HIGH, filtering out static.
*/
bool readDigitalAverage(byte sampleCount) {
  int highReadings = 0;
  for (byte i = 0; i < sampleCount; i++) {
    if (digitalRead(FSK_INPUT_PIN) == HIGH) highReadings++;
    delayMicroseconds(200);
  }
  return highReadings > (sampleCount / 2);
}

/* FALLING EDGE DETECTOR (Start Bit)
   In the UART protocol, the "idle" state is HIGH voltage (1).
   When the transmitter is going to send data, it drops to LOW (0). 
   This function locks the processor in a while loop waiting for that voltage drop.
*/
bool waitForStartBit(unsigned long maxWaitTimeMs) {
  unsigned long startTime = millis();
  static bool systemArmed = true;

  while ((millis() - startTime) < maxWaitTimeMs) {
    bool currentLevel = readDigitalAverage(4);

    if (currentLevel) {
      systemArmed = true;  // The line is HIGH (idle), arm the logical state.
    }

    // If it was armed and we read LOW, it's the falling edge (START OF TRANSMISSION)
    if (systemArmed && !currentLevel) {
      systemArmed = false;
      return true;
    }
    delay(1);
  }
  return false;  // Max time exceeded
}

/* CENTER-OF-EYE SAMPLING
   Reads the bit while filtering out noise.
*/
bool readBitCenter() {
  return readDigitalAverage(5);
}

/* BYTE EXTRACTION (1 ASCII Letter)
   Software synchronization to read the 8 data bits that make up a letter.
*/
char receiveCharacter(bool &success) {
  success = false;
  uint8_t byteValue = 0;  // Empty variable (00000000)

  // 1. FINE SYNCHRONIZATION: We just detected the Start Bit.
  // Jump forward in time exactly 1.5 times the duration of a pulse
  // to land perfectly in the middle of the first useful bit, avoiding unstable edges.
  delay(BIT_PERIOD_MS);
  delay(BIT_PERIOD_MS / 2);
  delayMicroseconds(BIT_PERIOD_US / 2);

  // 2. DATA READING: We read from Bit 7 (Most Significant) to Bit 0 (Least Significant)
  for (int bit = 7; bit >= 0; bit--) {
    bool bitState = readBitCenter();

    // If the physical reading is HIGH (1), we write a logical '1' using the
    // OR gate (|) and bit shifting (<<) at the current position.
    if (bitState) byteValue |= (1 << bit);

    // Advance 1 full time period to land in the center of the next bit
    if (bit > 0) {
      delay(BIT_PERIOD_MS);
      delayMicroseconds(BIT_PERIOD_US);
    }
  }

  // 3. STOP BIT: Advance to the end of the frame.
  delay(BIT_PERIOD_MS);
  delayMicroseconds(BIT_PERIOD_US);

  bool stopBit = readBitCenter();
  success = stopBit;  // Standard UART dictates that the Stop Bit MUST be HIGH.

  return (char)byteValue;  // Transform the bits (e.g., 01010110) into its ASCII letter ('V')
}

/* WORD ASSEMBLY
   Accumulates the extracted letters until forming the complete sentence ("QR_VERDE_OBJ")
*/
String receiveWordAuto() {
  String receivedWord = "";
  int counter = 0;
  bool success = false;

  char firstCharacter = receiveCharacter(success);
  if (!success) return "";

  // Discard the '~' characters that the transmitter sends to stabilize
  // the XR2211 PLL circuit before sending the real data.
  if (firstCharacter == SYNC_CHARACTER) {
    if (waitForStartBit(CHAR_WAIT_TIME_MS)) {
      char secondCharacter = receiveCharacter(success);
      if (success && secondCharacter != SYNC_CHARACTER && secondCharacter >= 32 && secondCharacter <= 126) {
        receivedWord += secondCharacter;
        counter++;
      }
    }
  }

  // Keep reading letters until the transmitter stays quiet for longer
  // than defined in CHAR_WAIT_TIME_MS, indicating the end of the message.
  while (counter < MAX_WORD_LENGTH) {
    if (!waitForStartBit(CHAR_WAIT_TIME_MS)) break;

    char letter = receiveCharacter(success);
    if (success) {
      if (letter == SYNC_CHARACTER) continue;  // If garbage slips in, ignore it
      if (letter >= 32 && letter <= 126) {     // Printable ASCII range (Letters and symbols)
        receivedWord += letter;
        counter++;
      }
    }
  }
  return receivedWord;
}


/* =============================================================================
   MAIN PROGRAM (Logic Brain)
   ============================================================================= */

void setup() {
  Serial.begin(9600);               // Communication with the PC USB port for debugging
  pinMode(FSK_INPUT_PIN, INPUT);    // Where the FSK cable enters

  Serial.println(F("======================================================="));
  Serial.println(F("   DATA RECEIVER + 4-DOF ARM"));
  Serial.println(F("======================================================="));

  // Servo I2C driver initialization
  pwmController.begin();
  pwmController.setOscillatorFrequency(27000000);  // 27MHz is the PCA9685 base oscillator
  pwmController.setPWMFreq(50);                    // Servos operate at 50 Hertz
  delay(10);

  // Send to initial home position
  pwmController.setPWM(BASE_CHANNEL, 0, mapAngle(BASE_HOME));
  pwmController.setPWM(SHOULDER_CHANNEL, 0, mapAngle(SHOULDER_HOME));
  pwmController.setPWM(ELBOW_CHANNEL, 0, mapAngle(ELBOW_HOME));
  pwmController.setPWM(GRIPPER_CHANNEL, 0, mapAngle(GRIPPER_HOME));

  Serial.println(F("   Arm at home (Front). Waiting for QR reading..."));
}

void loop() {
  // 1. LOCKOUT: The code freezes here (up to 60 sec) waiting for the pulse train
  if (waitForStartBit(60000UL)) {

    // 2. RECEPTION: Signal enters, reconstruct the sentence sent by LabVIEW.
    String receivedWord = receiveWordAuto();

    if (receivedWord.length() > 0) {
      Serial.print(F(">>> Frame received: \""));
      Serial.print(receivedWord);
      Serial.println(F("\""));

      /* 3. SORTING DECISION MAKING
         We use indexOf() as a defensive technique. It doesn't look for the word 
         to be identical; it looks for the fragment "QR_VERDE_OBJ" to exist within the 
         received string. This way, if there's noise (e.g., "$~QR_VERDE_OBJ"), 
         the system doesn't crash and still processes the instruction.
      */

      if (receivedWord.indexOf("QR_VERDE_OBJ") >= 0) {
        Serial.println(F("-> QR VALIDATED: GREEN. Depositing in left zone (40 degrees)."));
        // Calls the kinematics and injects the polar coordinate 40
        objectSortingRoutine(40);
      }

      else if (receivedWord.indexOf("QR_ROJO_OBJ") >= 0) {
        Serial.println(F("-> QR VALIDATED: RED. Depositing in right zone (140 degrees)."));
        // Calls the kinematics and injects the polar coordinate 140
        objectSortingRoutine(140);
      }

      else if (receivedWord.indexOf("QR_AZUL_OBJ") >= 0) {
        Serial.println(F("-> QR VALIDATED: BLUE. Depositing in center-right zone (110 degrees)."));
        // Calls the kinematics and injects the polar coordinate 110
        objectSortingRoutine(110);
      }

      else {
        // If the cable caught noise that randomly formed characters, it's filtered here.
        Serial.println(F("-> Frame ignored: Does not match any QR of interest."));
      }

      Serial.println(F("======================================================="));
      Serial.println(F("   Waiting for new QR readings..."));
    }
  }
}

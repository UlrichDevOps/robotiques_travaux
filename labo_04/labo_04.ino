#include <MeAuriga.h>
#include <Adafruit_seesaw.h>

#define BAUDS  115200

#define SEUIL 500

#define NB_IR 5

#define LEDNUM 12
#define LEDPIN 44
#define RINGALLLEDS 0

#define ENC_PULSE  9
#define ENC_RATIO 39.267
#define DIA_WHEEL 6.50
#define DIST_WHEEL 15.10
#define CIRC_WHEEL 20.42
#define FULL_TURN_CIRC 94.88
#define FULL_SPIN_CIRC 47.44

Adafruit_seesaw ss;
MeGyro gyro(0, 0x69);
MeRGBLed led(PORT0, LEDNUM);
MeEncoderOnBoard encoder_1(SLOT1);
MeEncoderOnBoard encoder_2(SLOT2);
MeUltrasonicSensor ultrasonicSensor(PORT_10);

typedef enum {
  SETUP,
  SEG_A,
  PIVOT,
  SEG_B,
  SEG_C,
  FIN_A
} State;

typedef enum {
  BOOT,
  ATTENTE,
  ERREUR,
  LAB3,
  NON_CALIB,
  CALIB,
  SUIVRE_LIGNE
} Action;

struct Capteur
{
  int valeurMin = 1023;
  int valeurMax = 0;
  int valeurLue;
  int valeurNormalisee;
};

String commands[] =
{
  "stop",
  "done",
  "lab3",
  "cal",
  "ligne"
};

Capteur capteurs[NB_IR];

Action currentAction = BOOT;

State currentState = SETUP;
State previousState = SETUP;

unsigned long currentTime = 0;

String lastCommand;

bool is_calibrated = false;
bool commandKnown = false;

double targetDistance = 120.0;
double obstacle = 30.0;
short speed = 80;

int tolerance = 2;
short pivotingSpeed = speed / 2;

void isr_process_encoder_1(void)
{
  if (digitalRead(encoder_1.getPortB()) == 0)
  {
    encoder_1.pulsePosMinus();
  }
  else
  {
    encoder_1.pulsePosPlus();
  }
}

void isr_process_encoder_2(void)
{
  if (digitalRead(encoder_2.getPortB()) == 0)
  {
    encoder_2.pulsePosMinus();
  }
  else
  {
    encoder_2.pulsePosPlus();
  }
}

void configureEncoders(void)
{
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  encoder_1.setPulse(ENC_PULSE);
  encoder_2.setPulse(ENC_PULSE);

  encoder_1.setRatio(ENC_RATIO);
  encoder_2.setRatio(ENC_RATIO);

  encoder_1.setPosPid(1.8, 0, 1.2);
  encoder_2.setPosPid(1.8, 0, 1.2);

  encoder_1.setSpeedPid(0.18, 0, 0);
  encoder_2.setSpeedPid(0.18, 0, 0);
}

void encodersLoop(void)
{
  encoder_1.loop();
  encoder_2.loop();
}

void blinkErrorLEDs(unsigned long t, bool firstRun = false);
void blinkUnknowCommandLEDs(unsigned long t, bool firstRun = false);

void setup()
{
  attachInterrupt(encoder_1.getIntNum(), isr_process_encoder_1, RISING);
  attachInterrupt(encoder_2.getIntNum(), isr_process_encoder_2, RISING);

  ss.begin();
  gyro.begin();
  led.setpin(LEDPIN);

  Serial.begin(BAUDS);

  configureEncoders();
}

void loop()
{
  gyro.update();
  encodersLoop();
  currentTime = millis();
  handsOnActions(currentTime);
}

void goStraight(short speed = 100, bool firstRun = 0)
{
  static double zAngleGoal = 0.0;

  static double error = 0.0;
  static double sumError = 0.0;
  static double previousError = 0.0;

  static double correction = 0.0;

  const double kp = 3;
  const double ki = 0;
  const double kd = 1;

  if (firstRun)
  {
    firstRun = false;
    zAngleGoal = gyro.getAngleZ();

    encoder_1.setMotorPwm(-speed);
    encoder_2.setMotorPwm(speed);

    return;
  }

  error = gyro.getAngleZ() - zAngleGoal;
  sumError += error;

  correction = kp * error + ki * sumError + kd * (error - previousError);
  previousError = error;

  encoder_1.setMotorPwm(-speed - correction);
  encoder_2.setMotorPwm(speed - correction);
}

bool spin(short speed, int targetAngle, bool firstRun = false)
{
  static double previousAngle = 0.0;
  static double totalAngle = 0.0;

  if (firstRun)
  {
    totalAngle = 0.0;
    previousAngle = gyro.getAngleZ();

    return false;
  }

  double currentAngle = gyro.getAngleZ();

  double difference = currentAngle - previousAngle;

  
  if (difference > 180)
  {
    difference -= 360;
  }
  else if (difference < -180)
  {
    difference += 360;
  }

  totalAngle += difference;
  previousAngle = currentAngle;

  if (targetAngle > 0)
  {
    encoder_1.setMotorPwm(speed);
    encoder_2.setMotorPwm(speed);

    return totalAngle >= targetAngle - tolerance;
  }
  else
  {
    encoder_1.setMotorPwm(-speed);
    encoder_2.setMotorPwm(-speed);

    return totalAngle <= targetAngle + tolerance;
  }
}

bool isCommandKnown (String cmd)
{
  int size = sizeof(commands) / sizeof(commands[0]);

  for (int i = 0; i < size; i++)
  {
    if (cmd == commands[i])
      return true;
  }

  return false;
}

void stopEngine()
{
  encoder_1.setMotorPwm(0);
  encoder_2.setMotorPwm(0);
}

void showProgressLEDs(double traversed, double start, double end)
{
  int index = map(traversed, start, end, 1, 7);
  led.setColor(index, 0, 255, 0);
  led.show();
}

void turnOffLEDs()
{
  for (int i = 1; i <= 7; i++)
  {
    led.setColorAt(i, 0, 0, 0);
  }
  led.show();
}

void blinkFinishLEDs(unsigned long t)
{
  static unsigned long lastTime = 0;
  static bool ledsOn = false;

  const unsigned long interval = 50;

  if (t - lastTime < interval) return;

  lastTime = t;
  ledsOn = !ledsOn;

  for (int i = 8; i <= 12; i++)
  {
    if (ledsOn)
    {
      led.setColorAt(i, 0, 255, 0);
    }
    else
    {
      led.setColorAt(i, 0, 0, 0);
    }
  }

  led.show();
}

int readDistance(unsigned long t)
{
  static unsigned long previousRead = 0;
  unsigned long rate = 100;
  static int lastResult = 0;

  if (t - previousRead >= rate)
  {
    previousRead = t;

    int distance = ultrasonicSensor.distanceCm();

    if (distance >= 0 && distance <= 400)
    {
      lastResult = distance;
    }
  }

  return lastResult;
}

void stateManager(unsigned long t)
{
  switch(currentState)
  {
    case SETUP: setupState(t); break;
    case SEG_A: segAState(t); break;
    case SEG_B: segBState(t); break;
    case SEG_C: segCState(t); break;
    case PIVOT: pivotState(t); break;
    case FIN_A: finAState(t); break;
  }
}

void handsOnActions(unsigned long ct)
{
  switch (currentAction)
  {
    case BOOT: bootState(ct); break;
    case ATTENTE: attenteState(ct); break;
    case ERREUR: errorState(ct); break;
    case LAB3: lab3State(ct); break;
    case NON_CALIB: nonCalibState(ct); break;
    case CALIB: calibState(ct); break;
    case SUIVRE_LIGNE: suivreLigneState(ct); break;
  }
}

#pragma region Fonction Etat Lab_03

void setupState(unsigned long t)
{
  static bool firstTime = true;
  static unsigned long exitTime = 0;
  int delay = 3000;

  if (firstTime)
  {
    // Entrée
    Serial.println("SETUP");
    exitTime = t + delay;
    firstTime = false;
    return;
  }

  bool transition = t > exitTime;

  if (transition)
  {
    // Sortie
    firstTime = true;
    previousState = SETUP;
    currentState = SEG_A;
  }
}

void segAState(unsigned long t)
{
  static bool firstTime = true;
  static unsigned long exitTime = 0UL;

  static long pos_1 = 0;
  static long pos_2 = 0;

  if (firstTime)
  {
    // Entrée
    Serial.println("SEG_A");
    firstTime = false;

    pos_1 = encoder_1.getCurPos();
    pos_2 = encoder_2.getCurPos();

    goStraight(speed, 1);
    return;
  }

  // Tâche(s) à effectuer
  double degree_1 = (double)encoder_1.getCurPos() - pos_1;
  double degree_2 = (double)encoder_2.getCurPos() - pos_2;

  double averageDegrees = (fabs(degree_1) + fabs(degree_2)) / 2.0;

  double distanceTraveled = averageDegrees * CIRC_WHEEL / 360.0;
  showProgressLEDs(distanceTraveled, 0, targetDistance);
  goStraight(speed);

  bool transition = distanceTraveled >= targetDistance;

  if (transition)
  {
    // Sorties
    stopEngine();
    turnOffLEDs();
    led.setColorAt(LEDNUM - 1, 0, 255, 0);
    led.show();

    firstTime = true;
    previousState = SEG_A;
    currentState = PIVOT;
  }
}

void segBState(unsigned long t)
{
  static bool firstTime = true;
  static double startingDistance = 0.0;

  if (firstTime)
  {
    Serial.println("SEG_B");
    firstTime = false;

    startingDistance = readDistance(t);
    goStraight(speed, 1);
    return;
  }

  double distance = readDistance(t);
  
  showProgressLEDs(distance, startingDistance, obstacle);
  goStraight(speed);

  bool transition = distance <= obstacle;

  if (transition)
  {
    stopEngine();
    turnOffLEDs();
    led.setColorAt(LEDNUM - 2, 0, 255, 0);
    led.show();

    firstTime = true;
    previousState = SEG_B;
    currentState = PIVOT;
  }
}

void segCState(unsigned long t)
{
  static bool firstTime = true;
  static unsigned long exitTime = 0UL;

  static long pos_1 = 0;
  static long pos_2 = 0;

  if (firstTime)
  {
    Serial.println("SEG_C");
    firstTime = false;

    pos_1 = encoder_1.getCurPos();
    pos_2 = encoder_2.getCurPos();

    goStraight(speed, 1);
    return;
  }

  double degree_1 = (double)encoder_1.getCurPos() - pos_1;
  double degree_2 = (double)encoder_2.getCurPos() - pos_2;

  double averageDegrees = (fabs(degree_1) + fabs(degree_2)) / 2.0;

  double distanceTraveled = averageDegrees * CIRC_WHEEL / 360.0;
  showProgressLEDs(distanceTraveled, 0, targetDistance);
  goStraight(speed);

  bool transition = distanceTraveled >= targetDistance;

  if (transition)
  {
    stopEngine();
    turnOffLEDs();
    led.setColorAt(LEDNUM - 3, 0, 255, 0);
    led.show();

    firstTime = true;
    previousState = SEG_C;
    currentState = PIVOT;
  }
}

void pivotState(unsigned long t)
{
  static bool firstTime = true;
  static int pivotAngle = 0;

  if (firstTime)
  {
    Serial.println("PIVOT");

    if (previousState == SEG_A)
    {
      Serial.println("A = PIVOT");
      pivotAngle = -90;
    }
    else if (previousState == SEG_B)
    {
      Serial.println("B = PIVOT");
      pivotAngle = +90;
    }
    else if (previousState == SEG_C)
    {
      Serial.println("C = PIVOT");
      pivotAngle = -180;
    }

    spin(pivotingSpeed, pivotAngle, true);

    firstTime = false;
    return;
  }

  bool transition = spin(pivotingSpeed, pivotAngle);

  if (transition)
  {
    stopEngine();

    firstTime = true;

    if (previousState == SEG_A)
    {
      currentState = SEG_B;
    }
    else if (previousState == SEG_B)
    {
      currentState = SEG_C;
    }
    else if (previousState == SEG_C)
    {
      currentState = FIN_A;
    }

    previousState = PIVOT;

    Serial.println("Sortie etat: PIVOT");
  }
}

void finAState(unsigned long t)
{
  static bool firstTime = true;

  if (firstTime)
  {
    firstTime = false;
    return;
  }

  blinkFinishLEDs(t);

  bool transition = false;

  if (transition)
  {
    firstTime = true;
    //currentState = SEG_A;
  }
}

#pragma endregion

void serialEvent()
{
  if (!Serial.available()) return;

  lastCommand = Serial.readStringUntil('\n');

  lastCommand.trim();

  Serial.print("Robot a reçu : ");
  Serial.println(lastCommand);

  commandKnown = isCommandKnown(lastCommand);
}

void bootState(unsigned long t)
{
  static bool firstTime = true;
  static unsigned long exitTime = 0UL;
  int delay = 3000;

  if (firstTime)
  {
    // Entrée
    Serial.println("BOOT");
    exitTime = t + delay;
    firstTime = false;
    return;
  }

  bool transition = t > exitTime;

  if (transition)
  {
    // Sortie
    firstTime = true;
    currentAction = ATTENTE;
  }
}

void attenteState(unsigned long t)
{
  static bool firstTime = true;

  if (firstTime)
  {
    Serial.println("ATTENTE");
    firstTime = false;
    return;
  }

  if (lastCommand.length() == 0) return;

  if (!commandKnown)
  {
    firstTime = true;
    currentAction = ERREUR;
  }
  else if (lastCommand == "cal")
  {
    firstTime = true;
    currentAction = CALIB;
  }
  else if (lastCommand == "lab3")
  {
    firstTime = true;
    currentAction = LAB3;
  }
  else if (lastCommand == "ligne" && !is_calibrated)
  {
    firstTime = true;
    currentAction = NON_CALIB;
  }
  else if (lastCommand == "ligne" && is_calibrated)
  {
    firstTime = true;
    currentAction = SUIVRE_LIGNE;
  }
  else
  {
    lastCommand = "";
  }
}

void errorState(unsigned long t)
{
  static bool firstTime = true;

  if (firstTime)
  {
    Serial.println("ERREUR");
    blinkErrorLEDs(t, true);
    firstTime = false;
    return;
  }

  blinkErrorLEDs(t);

  bool transition = (lastCommand == "stop");

  if (transition)
  {
    firstTime = true;
    stopEngine();
    lastCommand = "";
    currentAction = ATTENTE;
  }
}

void lab3State(unsigned long t)
{
  static bool firstTime = true;

  if (firstTime)
  {
    Serial.println("LAB3");
    firstTime = false;
    return;
  }

  stateManager(t);

  bool transition = (lastCommand == "stop") || (lastCommand == "done");

  if (transition)
  {
    firstTime = true;
    stopEngine();
    lastCommand = "";
    currentAction = ATTENTE;
  }
}

void nonCalibState(unsigned long t)
{
  static bool firstTime = true;
  static unsigned long exitTime = 0UL;

  if (firstTime)
  {
    Serial.println("NON_CALIB");
    blinkUnknowCommandLEDs(t, true);
    firstTime = false;
    return;
  }

  blinkUnknowCommandLEDs(t);

  bool transition = (lastCommand == "stop");

  if (transition)
  {
    firstTime = true;
    stopEngine();
    lastCommand = "";
    currentAction = ATTENTE;
  }
}

void calibState(unsigned long t)
{
  static bool firstTime = true;

  if (firstTime)
  {
    Serial.println("CALIB");
    firstTime = false;
    return;
  }

  calibrationAutomatic();

  bool transition = is_calibrated || (lastCommand == "stop") || (lastCommand == "done");

  if (transition)
  {
    stopEngine();
    lastCommand = "";
    firstTime = true;
    currentAction = ATTENTE;
  }
}


void suivreLigneState(unsigned long t)
{
  static bool firstTime = true;
  static bool pivoting = false;

  if (firstTime)
  {
    Serial.println("SUIVRE_LIGNE");
    firstTime = false;
    return;
  }

  if (!pivoting)
  {
    followLine();

    bool transition = allSensorsOnBlack();

    if (transition)
    {

      stopEngine();

      spin(pivotingSpeed, 180, true);

      pivoting = true;
      return;
    }
  }
  else
  {
    bool transition = spin(pivotingSpeed, 180);

    if (transition)
    {

      stopEngine();
      pivoting = false;
    }
  }

  bool stop = (lastCommand == "stop");

  if (stop)
  {
    stopEngine();
    pivoting = false;
    lastCommand = "";
    firstTime = true;
    currentAction = ATTENTE;
  }
}


void lireCapteurs()
{
  for (int i = 0; i < NB_IR; i++)
  {
    capteurs[i].valeurLue = ss.analogRead(i);
  }
}

void calibration()
{
  lireCapteurs();

  for (int i = 0; i < NB_IR; i++)
  {
    int val = capteurs[i].valeurLue;

    if (val < capteurs[i].valeurMin)
      capteurs[i].valeurMin = val;

    if (val > capteurs[i].valeurMax)
      capteurs[i].valeurMax = val;
  }
}

double capteurLectureNormalisee(int index)
{
  int valeurLue = capteurs[index].valeurLue;
  int valeurMin = capteurs[index].valeurMin;
  int valeurMax = capteurs[index].valeurMax;

  if (valeurMax == valeurMin) return 0.0;

  return ((valeurLue - valeurMin) * 1000.0) / (valeurMax - valeurMin);
}

void normaliserValeurs()
{
  for (int i = 0; i < NB_IR; i++)
  {
    capteurs[i].valeurNormalisee = capteurLectureNormalisee(i);
  }
}

double calculerPositionLgne()
{
  double numerateur = 0.0;
  double denominateur = 0.0;

  for (int i = 0; i < NB_IR; i++)
  {
    numerateur += capteurs[i].valeurNormalisee * (i - 2);
    denominateur += capteurs[i].valeurNormalisee;
  }

  if (denominateur == 0) return 0.0;

  double position = (numerateur / denominateur) * 1000.0;

  return position;
}

float computePID(float position, float consigne = 0.0f)
{
  static float kp = 0.2;
  static float ki = 0.0;
  static float kd = 0.05;

  static float integral = 0;
  static float derivative = 0;
  static float lastError = 0;

  float error = position - consigne;

  integral += error;
  derivative = error - lastError;
  lastError = error;

  float output = kp * error + ki * integral + kd * derivative;

  return output;
}

bool calibrationAutomatic()
{
  static bool firstRun = true;

  if (is_calibrated)
  {
    return true;
  }

  calibration();

  if (spin(50, 360, firstRun))
  {

    stopEngine();
    is_calibrated = true;
    firstRun = true;

    return true;
  }

  firstRun = false;
  return false;
}

void followLine()
{
  lireCapteurs();

  normaliserValeurs();

  double position = calculerPositionLgne();

  float correction = computePID(position);

  encoder_1.setMotorPwm(-speed - correction);
  encoder_2.setMotorPwm(speed - correction);
}

bool allSensorsOnBlack()
{
  for (int i = 0; i < NB_IR; i++)
  {
    if (capteurs[i].valeurNormalisee > SEUIL)
    {
      return false;
    }
  }

  return true;
}

void blinkErrorLEDs(unsigned long t, bool firstRun)
{
  static unsigned long startTime = 0;
  static unsigned long lastTime = 0;
  static bool ledState = false;

  const unsigned long delay = 250;
  const unsigned long duration = 3000;

  if (firstRun)
  {
    startTime = t;
    lastTime = t;
    ledState = false;

    led.setColor(0, 0, 0, 0);
    led.show();

    return;
  }

  if (t - startTime >= duration)
  {
    if (ledState)
    {
      ledState = false;
      led.setColor(0, 0, 0, 0);
      led.show();
    }

    return;
  }

  if (t - lastTime < delay) return;

  lastTime = t;
  ledState = !ledState;

  if (ledState)
  {
    led.setColor(0, 255, 0, 0);
  }
  else
  {
    led.setColor(0, 0, 0, 0);
  }

  led.show();
}

void blinkUnknowCommandLEDs(unsigned long t, bool firstRun)
{
  static unsigned long startTime = 0;
  static unsigned long lastTime = 0;
  static bool ledState = false;

  const unsigned long delay = 250;
  const unsigned long duration = 3000;

  if (firstRun)
  {
    startTime = t;
    lastTime = t;
    ledState = false;

    led.setColor(0, 0, 0, 0);
    led.show();

    return;
  }

  if (t - startTime >= duration)
  {
    if (ledState)
    {
      ledState = false;
      led.setColor(0, 0, 0, 0);
      led.show();
    }

    return;
  }

  if (t - lastTime < delay) return;

  lastTime = t;
  ledState = !ledState;

  if (ledState)
  {
    led.setColor(0, 255, 255, 0);
  }
  else
  {
    led.setColor(0, 0, 0, 0);
  }

  led.show();
}

#pragma region Structure Fonction

void XState(unsigned long t)
{
  static bool firstTime = true;
  static unsigned long exitTime = 0UL;
  // Autres variables ex: distance, delai, etc.
  // Si nécéssaire!

  if (firstTime)
  {
    // Entrée
    //exitTime = t + delay;
    firstTime = false;
    return;
  }

  // Tâche(s) à effectuer

  bool transition = false;

  if (transition)
  {
    // Sortie
    firstTime = true;
    //currentState = nextState;
  }
}

void task(unsigned long t)
{
  static unsigned long lastTime = 0;
  const int delay = 250;

  if (t - lastTime < delay) return;

  lastTime = t;

  // Chose(s) à faire
}

#pragma endregion
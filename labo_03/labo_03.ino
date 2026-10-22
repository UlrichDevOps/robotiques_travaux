#include <MeAuriga.h>

#define BAUDS  115200

#define LEDNUM 12
#define LEDPIN 44
#define RINGALLLEDS 0

#define ENC_PULSE  9
#define ENC_RATIO 39.267
#define DIA_WHEEL 6.50
#define CIRC_WHEEL 20.42

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
  SEG_C
} State;

State currentState = SETUP;
State previousState = SETUP;

unsigned long currentTime = 0;
double targetDistance = 120.0;
short speed = 100;


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

  // encoder_1.setPosDeadBand(10);
  // encoder_2.setPosDeadBand(10);

  // encoder_1.setFullPositionPidEnabled(true);
  // encoder_2.setFullPositionPidEnabled(true);

}

void encodersLoop(void)
{
  encoder_1.loop();
  encoder_2.loop();
}

void setup()
{
  attachInterrupt(encoder_1.getIntNum(), isr_process_encoder_1, RISING);
  attachInterrupt(encoder_2.getIntNum(), isr_process_encoder_2, RISING);

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
  // float distance = readDistance(currentTime);
  stateManager(currentTime);
  
  
}

void goTo(float dist, short speed = 100)
{
  float degree = (dist * 360) / CIRC_WHEEL;
  
  encoder_1.moveTo(-degree, speed);
  encoder_2.moveTo(degree, speed);
}

void goStraight(short speed = 100, bool firstRun = 0)
{
  static double zAngleGoal = 0.0;

  static double error = 0.0;
  static double sumError = 0.0;
  static double previousError = 0.0;

  static double correction = 0.0;

  const double kp = 6.5;
  const double ki = 0.0;
  const double kd = 1.1;

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

void stopEngine()
{
  encoder_1.setMotorPwm(0);
  encoder_2.setMotorPwm(0);
}

void showProgressLEDs(double traversed, double target)
{
  int index = map(traversed, 0, target, 1, 7);
  led.setColor(index, 0, 255, 0);
  led.show();
}

void turnOffLEDs()
{
  led.setColor(RINGALLLEDS, 0, 0, 0);
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
    case PIVOT: pivotState(t); break;
    case SEG_B: segBState(t); break;
  }
}

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

    goStraight(100, 1);
    return;
  }

  // Tâche(s) à effectuer
  double degree_1 = (double)encoder_1.getCurPos() - pos_1;
  double degree_2 = (double)encoder_2.getCurPos() - pos_2;

  double averageDegrees = (fabs(degree_1) + fabs(degree_2)) / 2.0;

  double distanceTraveled = averageDegrees * CIRC_WHEEL / 360.0;
  showProgressLEDs(distanceTraveled, targetDistance);
  goStraight(100);
  
  bool transition = distanceTraveled >= targetDistance; // False pour condition de sortie

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

void pivotState(unsigned long t)
{
  static bool firstTime = true;
  static unsigned long exitTime = 0UL;
  // Autres variables ex: distance, delai, etc.
  // Si nécéssaire!

  if (firstTime)
  {
    // Entrée
    Serial.println("PIVOT");
    //exitTime = t + delay;
    firstTime = false;
    return;
  }

  // Tâche(s) à effectuer

  bool transition = false; // False pour condition de sortie

  if (transition)
  {
    // Sortie
    firstTime = true;
    //currentState = nextState;
  }
}

void segBState(unsigned long t)
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

  bool transition = false; // False pour condition de sortie

  if (transition)
  {
    // Sortie
    firstTime = true;
    //currentState = nextState;
  }
}

void state_X(unsigned long t)
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

  bool transition = false; // False pour condition de sortie

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



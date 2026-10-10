#include <MeAuriga.h>

#define BAUDS  115200

#define LEDNUM 12
#define LEDPIN 44
#define RINGALLLEDS 0

#define ENC_PULSE  9
#define ENC_RATIO 39.267
#define DIA_WHEEL 6.50
#define CIRC_WHEEL 20.42

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

unsigned long currentTime = 0;

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

void setup()
{
  attachInterrupt(encoder_1.getIntNum(), isr_process_encoder_1, RISING);
  attachInterrupt(encoder_2.getIntNum(), isr_process_encoder_2, RISING);

  led.setpin(LEDPIN);

  Serial.begin(BAUDS);

  configureEncoders();
}

void loop()
{
  encodersLoop();
  currentTime = millis();
  int distance = readDistance(currentTime);
  stateManager(currentTime);
  goTo(0);
  
}
void goTo(float d)
{
  float degree = (d * 360) / CIRC_WHEEL;
  
  encoder_1.moveTo(-degree);
  encoder_2.moveTo(degree);
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
    currentState = SEG_A;
  }
}

void segAState(unsigned long t)
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

void pivotState(unsigned long t)
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


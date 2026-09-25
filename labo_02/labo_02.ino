#include <MeAuriga.h>

#define LEDNUM  12
#define LEDPIN  44
#define RINGALLLEDS 0

#define BAUDS 115200

MeRGBLed led(PORT0, LEDNUM);
MeUltrasonicSensor ultraSensor(PORT_10);

enum Etat {
  MARCHE,
  LENT,
  ARRET,
  RECULE,
  PIVOTE
} etatCourant = MARCHE;

const int M1_PWM = 11;
const int M1_IN1 = 48;
const int M1_IN2 = 49;

const int M2_PWM = 10;
const int M2_IN1 = 47;
const int M2_IN2 = 46;

int vitesseDeCroissiere = 255 * 0.70;
int vitesseDeRalentissement = vitesseDeCroissiere * 0.50;

unsigned long tempsActuel = 0;
int distance = 0;

void setup()
{
  led.setpin(LEDPIN);
  Serial.begin(BAUDS);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_PWM, OUTPUT);

  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
}

void loop()
{
  tempsActuel = millis();
  distance = LectureDeDistance(tempsActuel);
  gestionnaireEtats(tempsActuel);
  AffichageEcran(tempsActuel, distance);
}

int LectureDeDistance(unsigned long t)
{
  static unsigned long TempsDerniereLecture = 0;
  static int dernierResultat = 100;
  int delaiDeLecture = 100;

  if (t - TempsDerniereLecture < delaiDeLecture) return dernierResultat;

  TempsDerniereLecture = t;

  dernierResultat = ultraSensor.distanceCm();

  return dernierResultat;
}

void AffichageEcran(unsigned long t, int d)
{
  static unsigned long dernierAffichage = 0;
  int delaiAffichage = 250;

  if (t - dernierAffichage < delaiAffichage) return;
  dernierAffichage = t;

  Serial.print("Distance: ");
  Serial.print(d);
  Serial.print(" Cm - État: ");

  switch (etatCourant)
  {
    case MARCHE: Serial.println("MARCHE"); break;
    case LENT: Serial.println("LENT"); break;
    case ARRET: Serial.println("ARRET"); break;
    case RECULE: Serial.println("RECULE"); break;
    case PIVOTE: Serial.println("PIVOTE"); break;

  }
}

void EteindreLesLeds()
{
  led.setColor(0, 0, 0);
  led.show();
}

void AllumerLedsAvants()
{
  for (int i = 0; i < LEDNUM / 2; i++)
  {
  
    led.setColorAt(i, 0, 255, 0);
  }
  led.show();

}

void AllumerLedsArrieres()
{
  
  for (int i = LEDNUM / 2; i < LEDNUM; i++)
  {
    led.setColorAt(i, 255, 255, 0);
  }
  led.show();
}

void AllumerAnneauLeds()
{
  led.setColor(RINGALLLEDS, 255, 0, 0);
  led.show();
}

void Avancer(int v)
{
  digitalWrite(M1_IN2, LOW);  digitalWrite(M1_IN1, HIGH);  analogWrite(M1_PWM, v);
  digitalWrite(M2_IN2, LOW); digitalWrite(M2_IN1, HIGH);   analogWrite(M2_PWM, v);
}

void Reculer(int v)
{
  digitalWrite(M1_IN2, HIGH); digitalWrite(M1_IN1, LOW);   analogWrite(M1_PWM, v);
  digitalWrite(M2_IN2, HIGH);  digitalWrite(M2_IN1, LOW);  analogWrite(M2_PWM, v);
}

void Pivoter(int v)
{
  digitalWrite(M1_IN2, HIGH);  digitalWrite(M1_IN1, LOW);  analogWrite(M1_PWM, v);
  digitalWrite(M2_IN2, LOW);  digitalWrite(M2_IN1, HIGH);  analogWrite(M2_PWM, v);
}

void Arreter()
{
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
}

void EtatMarche(){
  static bool premiereFois = true;
  const int distanceRalentissement = 100;

  if (premiereFois)
  {
    AllumerLedsAvants();
    premiereFois = false;
    return;
  }

  Avancer(vitesseDeCroissiere);

  bool transition = distance < distanceRalentissement;

  if (transition)
  {
    EteindreLesLeds();
    premiereFois = true;
    etatCourant = LENT;
  }

}

void EtatLent()
{
  static bool premiereFois = true;
  const int distanceRalentissement = 100;
  const int distanceArret = 30;

  if (premiereFois)
  {
    AllumerLedsArrieres();
    premiereFois = false;
    return;
  }

  Avancer(vitesseDeRalentissement);

  bool transitionA = distance >= distanceRalentissement;
  bool transitionB = distance < distanceArret;

  if (transitionA)
  {
    EteindreLesLeds();
    premiereFois = true;
    etatCourant = MARCHE;
  }

  if (transitionB)
  {
    EteindreLesLeds();
    premiereFois = true;
    etatCourant = ARRET;
  }

}

void EtatArret(unsigned long t)
{
  static unsigned long tempsDeSortie = 0;
  static bool premiereFois = true;
  const int delaiArret = 2000;

  if (premiereFois)
  {
    AllumerAnneauLeds();
    tempsDeSortie = t + delaiArret;
    premiereFois = false;
    return;
  }

  Arreter();

  bool transition = t > tempsDeSortie;

  if (transition)
  {
    EteindreLesLeds();
    premiereFois = true;
    etatCourant = RECULE;
  }

}

void EtatRecule(unsigned long t)
{
  static unsigned long tempsDeSortie = 0;
  static bool premiereFois = true;
  const int delaiDeRecul = 1000;

  if (premiereFois)
  {
    AllumerAnneauLeds();
    tempsDeSortie = t + delaiDeRecul;
    premiereFois = false;
    return;
  }

  Reculer(vitesseDeRalentissement);

  bool transition = t > tempsDeSortie;

  if (transition)
  {
    EteindreLesLeds();
    premiereFois = true;
    etatCourant = PIVOTE;
  }
  
}

void EtatPivote(unsigned long t)
{
  static unsigned long tempsDeSortie = 0;
  static bool premiereFois = true;
  const int delaiDePivotement = 1000;

  if (premiereFois)
  {
    AllumerAnneauLeds();
    tempsDeSortie = t + delaiDePivotement;
    premiereFois = false;
    return;
  }

  Pivoter(vitesseDeRalentissement);

  bool transition = t >= tempsDeSortie;

  if (transition)
  {
    EteindreLesLeds();
    premiereFois = true;
    etatCourant = MARCHE;
  }

  
}

void gestionnaireEtats(unsigned long t)
{
  switch (etatCourant)
  {
    case MARCHE: EtatMarche(); break;
    case LENT: EtatLent(); break;
    case ARRET: EtatArret(t); break;
    case RECULE: EtatRecule(t); break;
    case PIVOTE: EtatPivote(t); break;

  }
}
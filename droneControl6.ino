#include <Fuzzy.h>
#include <Wire.h>  // Necessário apenas para o Arduino 1.6.5 e posterior
//#include <TinyMPU6050.h>
//#include <HCSR04.h>
#include "BluetoothSerial.h"
//#include "FS.h"
//#include "SD.h"
//#include "SPI.h"
#include <ESP32Servo.h>

#define MAX_SIGNAL 1800
#define MIN_SIGNAL 1200
#define MOTOR1_PIN 26
#define MOTOR2_PIN 25
#define MOTOR3_PIN 33
#define MOTOR4_PIN 32

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
//File myFile;

Servo motor1DC;
Servo motor2DC;
Servo motor3DC;
Servo motor4DC;

String valorRecebido = "";
bool abortarMissao = true;
bool motorLigado = false;

void filtroSeguranca(float angleX, float angleY, float angleZ);
void controlaVoo();
void abortarVoo();
void acionaMotor1();
void acionaMotor2();
void acionaMotor3();
void acionaMotor4();

//UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);

double alturaRef = 20;    // 20 cm

double altura = 0.0;
int countAltura = 0;

double gainBase = 0.4;
double gainMotor = 1;

bool testeBase = false;

int limiteBase = 1600;
double criticalAngle = 40;
int securityNumber = 2;
int delaySet = 10;

const int MPU_addr=0x68;
 
int minVal=265;
int maxVal=402;

//int count = 0, num = 35;

const int freq = 50;
const int resolucao = 12;

float baseRef = MIN_SIGNAL;
float base = MIN_SIGNAL;

float motor1Fuzzy = 0.0;
float motor2Fuzzy = 0.0;
float motor3Fuzzy = 0.0;
float motor4Fuzzy = 0.0;

// Fuzzy
Fuzzy *fuzzy = new Fuzzy();

// FuzzyInput pitch
//FuzzySet *erro_negativo_grande_pitch = new FuzzySet(-40, -40, -25, -12);
FuzzySet *erro_negativo_medio_pitch = new FuzzySet(-25, -25, -25, -8);
FuzzySet *erro_negativo_pequeno_pitch = new FuzzySet(-12, -5, -5, 0);
FuzzySet *erro_nulo_pitch = new FuzzySet(-5, 0, 0, 5);
FuzzySet *erro_positivo_pequeno_pitch = new FuzzySet(0, 5, 5, 12);
FuzzySet *erro_positivo_medio_pitch = new FuzzySet(8, 25, 25, 25);
//FuzzySet *erro_positivo_grande_pitch = new FuzzySet(12, 25, 40, 40);

// FuzzyInput derivada pitch
//FuzzySet *erro_negativo_grande_derivada_pitch = new FuzzySet(-2.5, -2.5, -1.6, -1.2);
FuzzySet *erro_negativo_medio_derivada_pitch = new FuzzySet(-6.0, -6.0, -6.0, -2.0);
FuzzySet *erro_negativo_pequeno_derivada_pitch = new FuzzySet(-6.0, -2.0, -2.0, 0);
FuzzySet *erro_nulo_derivada_pitch = new FuzzySet(-0.8, 0, 0, 0.8);
FuzzySet *erro_positivo_pequeno_derivada_pitch = new FuzzySet(0, 2.0, 2.0, 6.0);
FuzzySet *erro_positivo_medio_derivada_pitch = new FuzzySet(2.0, 6.0, 6.0, 6.0);
//FuzzySet *erro_positivo_grande_derivada_pitch = new FuzzySet(1.2, 1.6, 2.5, 2.5);

// FuzzyOutput pitch
//FuzzySet *incremento_negativo_grande_pitch = new FuzzySet(-3, -3, -2, -1);
FuzzySet *incremento_negativo_medio_pitch = new FuzzySet(-0.5, -0.5, -0.5, -0.3);
FuzzySet *incremento_negativo_pequeno_pitch = new FuzzySet(-0.5, -0.3, -0.3, 0);
FuzzySet *incremento_nulo_pitch = new FuzzySet(0, 0, 0, 0);
FuzzySet *incremento_positivo_pequeno_pitch = new FuzzySet(0, 0.3, 0.3, 0.5);
FuzzySet *incremento_positivo_medio_pitch = new FuzzySet(0.3, 0.5, 0.5, 0.5);
//FuzzySet *incremento_positivo_grande_pitch = new FuzzySet(1, 2, 3, 3);

// ----------------

// FuzzyInput roll
////FuzzySet *erro_negativo_grande_pitch = new FuzzySet(-40, -40, -25, -12);
//FuzzySet *erro_negativo_medio_roll = new FuzzySet(-25, -25, -25, -8);
//FuzzySet *erro_negativo_pequeno_roll = new FuzzySet(-12, -8, -8, 3);
//FuzzySet *erro_nulo_roll = new FuzzySet(-5, 0, 0, 5);
//FuzzySet *erro_positivo_pequeno_roll = new FuzzySet(3, 8, 8, 12);
//FuzzySet *erro_positivo_medio_roll = new FuzzySet(8, 25, 25, 25);
////FuzzySet *erro_positivo_grande_roll = new FuzzySet(12, 25, 40, 40);
//
//// FuzzyInput derivada roll
////FuzzySet *erro_negativo_grande_derivada_roll = new FuzzySet(-2.5, -2.5, -1.6, -1.2);
//FuzzySet *erro_negativo_medio_derivada_roll = new FuzzySet(-3.5, -3.5, -1.8, -1.2);
//FuzzySet *erro_negativo_pequeno_derivada_roll = new FuzzySet(-1.8, -1.2, -1.2, 0);
//FuzzySet *erro_nulo_derivada_roll = new FuzzySet(-0.8, 0, 0, 0.8);
//FuzzySet *erro_positivo_pequeno_derivada_roll = new FuzzySet(0, 1.2, 1.2, 1.8);
//FuzzySet *erro_positivo_medio_derivada_roll = new FuzzySet(1.2, 1.8, 3.5, 3.5);
////FuzzySet *erro_positivo_grande_derivada_roll = new FuzzySet(1.2, 1.6, 2.5, 2.5);
//
//// FuzzyOutput roll
////FuzzySet *incremento_negativo_grande_roll = new FuzzySet(-3, -3, -2, -1);
//FuzzySet *incremento_negativo_medio_roll = new FuzzySet(-3, -3, -1, -0.5);
//FuzzySet *incremento_negativo_pequeno_roll = new FuzzySet(-1, -0.5, -0.5, 0);
//FuzzySet *incremento_nulo_roll = new FuzzySet(0, 0, 0, 0);
//FuzzySet *incremento_positivo_pequeno_roll = new FuzzySet(0, 0.5, 0.5, 1);
//FuzzySet *incremento_positivo_medio_roll = new FuzzySet(0.5, 1, 3, 3);
////FuzzySet *incremento_positivo_grande_roll = new FuzzySet(1, 2, 3, 3);

void setup()
{
  Serial.begin(115200);
  
  // FuzzyInput
  FuzzyInput *erroPitch = new FuzzyInput(1);

//  erroPitch->addFuzzySet(erro_negativo_grande_pitch);
  erroPitch->addFuzzySet(erro_negativo_medio_pitch);
  erroPitch->addFuzzySet(erro_negativo_pequeno_pitch);
  erroPitch->addFuzzySet(erro_nulo_pitch);
  erroPitch->addFuzzySet(erro_positivo_pequeno_pitch);
  erroPitch->addFuzzySet(erro_positivo_medio_pitch);
//  erroPitch->addFuzzySet(erro_positivo_grande_pitch);
  fuzzy->addFuzzyInput(erroPitch);

  // FuzzyInput
  FuzzyInput *erroDerivadaPitch = new FuzzyInput(2);

//  erroDerivadaPitch->addFuzzySet(erro_negativo_grande_derivada_pitch);
  erroDerivadaPitch->addFuzzySet(erro_negativo_medio_derivada_pitch);
  erroDerivadaPitch->addFuzzySet(erro_negativo_pequeno_derivada_pitch);
  erroDerivadaPitch->addFuzzySet(erro_nulo_derivada_pitch);
  erroDerivadaPitch->addFuzzySet(erro_positivo_pequeno_derivada_pitch);
  erroDerivadaPitch->addFuzzySet(erro_positivo_medio_derivada_pitch);
//  erroDerivadaPitch->addFuzzySet(erro_positivo_grande_derivada_pitch);
  fuzzy->addFuzzyInput(erroDerivadaPitch);

  // FuzzyInput
//  FuzzyInput *erroRoll = new FuzzyInput(3);
//
////  erroRoll->addFuzzySet(erro_negativo_grande_roll);
//  erroRoll->addFuzzySet(erro_negativo_medio_roll);
//  erroRoll->addFuzzySet(erro_negativo_pequeno_roll);
//  erroRoll->addFuzzySet(erro_nulo_roll);
//  erroRoll->addFuzzySet(erro_positivo_pequeno_roll);
//  erroRoll->addFuzzySet(erro_positivo_medio_roll);
////  erroRoll->addFuzzySet(erro_positivo_grande_roll);
//  fuzzy->addFuzzyInput(erroRoll);
//
//  // FuzzyInput
//  FuzzyInput *erroDerivadaRoll = new FuzzyInput(4);
//
////  erroDerivadaRoll->addFuzzySet(erro_negativo_grande_derivada_roll);
//  erroDerivadaRoll->addFuzzySet(erro_negativo_medio_derivada_roll);
//  erroDerivadaRoll->addFuzzySet(erro_negativo_pequeno_derivada_roll);
//  erroDerivadaRoll->addFuzzySet(erro_nulo_derivada_roll);
//  erroDerivadaRoll->addFuzzySet(erro_positivo_pequeno_derivada_roll);
//  erroDerivadaRoll->addFuzzySet(erro_positivo_medio_derivada_roll);
////  erroDerivadaRoll->addFuzzySet(erro_positivo_grande_derivada_roll);
//  fuzzy->addFuzzyInput(erroDerivadaRoll);
  
  // FuzzyOutput
  FuzzyOutput *saidaPitch = new FuzzyOutput(1);

//  saidaPitch->addFuzzySet(incremento_negativo_grande_pitch);
  saidaPitch->addFuzzySet(incremento_negativo_medio_pitch);
  saidaPitch->addFuzzySet(incremento_negativo_pequeno_pitch);
  saidaPitch->addFuzzySet(incremento_nulo_pitch);
  saidaPitch->addFuzzySet(incremento_positivo_pequeno_pitch);
  saidaPitch->addFuzzySet(incremento_positivo_medio_pitch);
//  saidaPitch->addFuzzySet(incremento_positivo_grande_pitch);
  fuzzy->addFuzzyOutput(saidaPitch);

  // FuzzyOutput
//  FuzzyOutput *saidaRoll = new FuzzyOutput(2);
//
////  saidaRoll->addFuzzySet(incremento_negativo_grande_roll);
//  saidaRoll->addFuzzySet(incremento_negativo_medio_roll);
//  saidaRoll->addFuzzySet(incremento_negativo_pequeno_roll);
//  saidaRoll->addFuzzySet(incremento_nulo_roll);
//  saidaRoll->addFuzzySet(incremento_positivo_pequeno_roll);
//  saidaRoll->addFuzzySet(incremento_positivo_medio_roll);
////  saidaRoll->addFuzzySet(incremento_positivo_grande_roll);
//  fuzzy->addFuzzyOutput(saidaRoll);

  // -------------------------------------------------------------------------------------
  
  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule1 = new FuzzyRuleAntecedent();
//  Rule1->joinWithAND(erro_negativo_grande_derivada_pitch, erro_negativo_grande_pitch);
//  FuzzyRuleConsequent *Consequence1 = new FuzzyRuleConsequent();
//  Consequence1->addOutput(incremento_negativo_grande_pitch);
//  
//  FuzzyRule *fuzzyRule1 = new FuzzyRule(1, Rule1, Consequence1);
//  fuzzy->addFuzzyRule(fuzzyRule1);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule2 = new FuzzyRuleAntecedent();
//  Rule2->joinWithAND(erro_negativo_grande_derivada_pitch, erro_negativo_medio_pitch);
//  FuzzyRuleConsequent *Consequence2 = new FuzzyRuleConsequent();
//  Consequence2->addOutput(incremento_negativo_grande_pitch);
//  
//  FuzzyRule *fuzzyRule2 = new FuzzyRule(2, Rule2, Consequence2);
//  fuzzy->addFuzzyRule(fuzzyRule2);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule3 = new FuzzyRuleAntecedent();
//  Rule3->joinWithAND(erro_negativo_grande_derivada_pitch, erro_negativo_pequeno_pitch);
//  FuzzyRuleConsequent *Consequence3 = new FuzzyRuleConsequent();
//  Consequence3->addOutput(incremento_negativo_grande_pitch);
//  
//  FuzzyRule *fuzzyRule3 = new FuzzyRule(3, Rule3, Consequence3);
//  fuzzy->addFuzzyRule(fuzzyRule3);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule4 = new FuzzyRuleAntecedent();
//  Rule4->joinWithAND(erro_negativo_grande_derivada_pitch, erro_nulo_pitch);
//  FuzzyRuleConsequent *Consequence4 = new FuzzyRuleConsequent();
//  Consequence4->addOutput(incremento_negativo_medio_pitch);
//  
//  FuzzyRule *fuzzyRule4 = new FuzzyRule(4, Rule4, Consequence4);
//  fuzzy->addFuzzyRule(fuzzyRule4);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule5 = new FuzzyRuleAntecedent();
//  Rule5->joinWithAND(erro_negativo_grande_derivada_pitch, erro_positivo_pequeno_pitch);
//  FuzzyRuleConsequent *Consequence5 = new FuzzyRuleConsequent();
//  Consequence5->addOutput(incremento_negativo_medio_pitch);
//  
//  FuzzyRule *fuzzyRule5 = new FuzzyRule(5, Rule5, Consequence5);
//  fuzzy->addFuzzyRule(fuzzyRule5);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule6 = new FuzzyRuleAntecedent();
//  Rule6->joinWithAND(erro_negativo_grande_derivada_pitch, erro_positivo_medio_pitch);
//  FuzzyRuleConsequent *Consequence6 = new FuzzyRuleConsequent();
//  Consequence6->addOutput(incremento_negativo_pequeno_pitch);
//  
//  FuzzyRule *fuzzyRule6 = new FuzzyRule(6, Rule6, Consequence6);
//  fuzzy->addFuzzyRule(fuzzyRule6);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule7 = new FuzzyRuleAntecedent();
//  Rule7->joinWithAND(erro_negativo_grande_derivada_pitch, erro_positivo_grande_pitch);
//  FuzzyRuleConsequent *Consequence7 = new FuzzyRuleConsequent();
//  Consequence7->addOutput(incremento_nulo_pitch);
//  
//  FuzzyRule *fuzzyRule7 = new FuzzyRule(7, Rule7, Consequence7);
//  fuzzy->addFuzzyRule(fuzzyRule7);

//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule8 = new FuzzyRuleAntecedent();
//  Rule8->joinWithAND(erro_negativo_medio_derivada_pitch, erro_negativo_grande_pitch);
//  FuzzyRuleConsequent *Consequence8 = new FuzzyRuleConsequent();
//  Consequence8->addOutput(incremento_negativo_grande_pitch);
//  
//  FuzzyRule *fuzzyRule8 = new FuzzyRule(8, Rule8, Consequence8);
//  fuzzy->addFuzzyRule(fuzzyRule8);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule1 = new FuzzyRuleAntecedent();
  Rule1->joinWithAND(erro_negativo_medio_derivada_pitch, erro_negativo_medio_pitch);
  FuzzyRuleConsequent *Consequence1 = new FuzzyRuleConsequent();
  Consequence1->addOutput(incremento_negativo_medio_pitch);
  
  FuzzyRule *fuzzyRule1 = new FuzzyRule(1, Rule1, Consequence1);
  fuzzy->addFuzzyRule(fuzzyRule1);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule2 = new FuzzyRuleAntecedent();
  Rule2->joinWithAND(erro_negativo_medio_derivada_pitch, erro_negativo_pequeno_pitch);
  FuzzyRuleConsequent *Consequence2 = new FuzzyRuleConsequent();
  Consequence2->addOutput(incremento_negativo_medio_pitch);
  
  FuzzyRule *fuzzyRule2 = new FuzzyRule(2, Rule2, Consequence2);
  fuzzy->addFuzzyRule(fuzzyRule2);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule3 = new FuzzyRuleAntecedent();
  Rule3->joinWithAND(erro_negativo_medio_derivada_pitch, erro_nulo_pitch);
  FuzzyRuleConsequent *Consequence3 = new FuzzyRuleConsequent();
  Consequence3->addOutput(incremento_negativo_pequeno_pitch);
  
  FuzzyRule *fuzzyRule3 = new FuzzyRule(3, Rule3, Consequence3);
  fuzzy->addFuzzyRule(fuzzyRule3);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule4 = new FuzzyRuleAntecedent();
  Rule4->joinWithAND(erro_negativo_medio_derivada_pitch, erro_positivo_pequeno_pitch);
  FuzzyRuleConsequent *Consequence4 = new FuzzyRuleConsequent();
  Consequence4->addOutput(incremento_negativo_pequeno_pitch);
  
  FuzzyRule *fuzzyRule4 = new FuzzyRule(4, Rule4, Consequence4);
  fuzzy->addFuzzyRule(fuzzyRule4);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule5 = new FuzzyRuleAntecedent();
  Rule5->joinWithAND(erro_negativo_medio_derivada_pitch, erro_positivo_medio_pitch);
  FuzzyRuleConsequent *Consequence5 = new FuzzyRuleConsequent();
  Consequence5->addOutput(incremento_nulo_pitch);
  
  FuzzyRule *fuzzyRule5 = new FuzzyRule(5, Rule5, Consequence5);
  fuzzy->addFuzzyRule(fuzzyRule5);

//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule14 = new FuzzyRuleAntecedent();
//  Rule14->joinWithAND(erro_negativo_medio_derivada_pitch, erro_positivo_grande_pitch);
//  FuzzyRuleConsequent *Consequence14 = new FuzzyRuleConsequent();
//  Consequence14->addOutput(incremento_positivo_pequeno_pitch);
//  
//  FuzzyRule *fuzzyRule14 = new FuzzyRule(14, Rule14, Consequence14);
//  fuzzy->addFuzzyRule(fuzzyRule14);

//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule15 = new FuzzyRuleAntecedent();
//  Rule15->joinWithAND(erro_negativo_pequeno_derivada_pitch, erro_negativo_grande_pitch);
//  FuzzyRuleConsequent *Consequence15 = new FuzzyRuleConsequent();
//  Consequence15->addOutput(incremento_negativo_medio_pitch);
//  
//  FuzzyRule *fuzzyRule15 = new FuzzyRule(15, Rule15, Consequence15);
//  fuzzy->addFuzzyRule(fuzzyRule15);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule6 = new FuzzyRuleAntecedent();
  Rule6->joinWithAND(erro_negativo_pequeno_derivada_pitch, erro_negativo_medio_pitch);
  FuzzyRuleConsequent *Consequence6 = new FuzzyRuleConsequent();
  Consequence6->addOutput(incremento_negativo_medio_pitch);
  
  FuzzyRule *fuzzyRule6 = new FuzzyRule(6, Rule6, Consequence6);
  fuzzy->addFuzzyRule(fuzzyRule6);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule7 = new FuzzyRuleAntecedent();
  Rule7->joinWithAND(erro_negativo_pequeno_derivada_pitch, erro_negativo_pequeno_pitch);
  FuzzyRuleConsequent *Consequence7 = new FuzzyRuleConsequent();
  Consequence7->addOutput(incremento_negativo_pequeno_pitch);
  
  FuzzyRule *fuzzyRule7 = new FuzzyRule(7, Rule7, Consequence7);
  fuzzy->addFuzzyRule(fuzzyRule7);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule8 = new FuzzyRuleAntecedent();
  Rule8->joinWithAND(erro_negativo_pequeno_derivada_pitch, erro_nulo_pitch);
  FuzzyRuleConsequent *Consequence8 = new FuzzyRuleConsequent();
  Consequence8->addOutput(incremento_negativo_pequeno_pitch);
  
  FuzzyRule *fuzzyRule8 = new FuzzyRule(8, Rule8, Consequence8);
  fuzzy->addFuzzyRule(fuzzyRule8);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule9 = new FuzzyRuleAntecedent();
  Rule9->joinWithAND(erro_negativo_pequeno_derivada_pitch, erro_positivo_pequeno_pitch);
  FuzzyRuleConsequent *Consequence9 = new FuzzyRuleConsequent();
  Consequence9->addOutput(incremento_nulo_pitch);
  
  FuzzyRule *fuzzyRule9 = new FuzzyRule(9, Rule9, Consequence9);
  fuzzy->addFuzzyRule(fuzzyRule9);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule10 = new FuzzyRuleAntecedent();
  Rule10->joinWithAND(erro_negativo_pequeno_derivada_pitch, erro_positivo_medio_pitch);
  FuzzyRuleConsequent *Consequence10 = new FuzzyRuleConsequent();
  Consequence10->addOutput(incremento_positivo_pequeno_pitch);
  
  FuzzyRule *fuzzyRule10 = new FuzzyRule(10, Rule10, Consequence10);
  fuzzy->addFuzzyRule(fuzzyRule10);

//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule21 = new FuzzyRuleAntecedent();
//  Rule21->joinWithAND(erro_negativo_pequeno_derivada_pitch, erro_positivo_grande_pitch);
//  FuzzyRuleConsequent *Consequence21 = new FuzzyRuleConsequent();
//  Consequence21->addOutput(incremento_positivo_pequeno_pitch);
//  
//  FuzzyRule *fuzzyRule21 = new FuzzyRule(21, Rule21, Consequence21);
//  fuzzy->addFuzzyRule(fuzzyRule21);

//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule22 = new FuzzyRuleAntecedent();
//  Rule22->joinWithAND(erro_nulo_derivada_pitch, erro_negativo_grande_pitch);
//  FuzzyRuleConsequent *Consequence22 = new FuzzyRuleConsequent();
//  Consequence22->addOutput(incremento_negativo_medio_pitch);
//  
//  FuzzyRule *fuzzyRule22 = new FuzzyRule(22, Rule22, Consequence22);
//  fuzzy->addFuzzyRule(fuzzyRule22);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule11 = new FuzzyRuleAntecedent();
  Rule11->joinWithAND(erro_nulo_derivada_pitch, erro_negativo_medio_pitch);
  FuzzyRuleConsequent *Consequence11 = new FuzzyRuleConsequent();
  Consequence11->addOutput(incremento_negativo_pequeno_pitch);
  
  FuzzyRule *fuzzyRule11 = new FuzzyRule(11, Rule11, Consequence11);
  fuzzy->addFuzzyRule(fuzzyRule11);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule12 = new FuzzyRuleAntecedent();
  Rule12->joinWithAND(erro_nulo_derivada_pitch, erro_negativo_pequeno_pitch);
  FuzzyRuleConsequent *Consequence12 = new FuzzyRuleConsequent();
  Consequence12->addOutput(incremento_negativo_pequeno_pitch);
  
  FuzzyRule *fuzzyRule12 = new FuzzyRule(12, Rule12, Consequence12);
  fuzzy->addFuzzyRule(fuzzyRule12);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule13 = new FuzzyRuleAntecedent();
  Rule13->joinWithAND(erro_nulo_derivada_pitch, erro_nulo_pitch);
  FuzzyRuleConsequent *Consequence13 = new FuzzyRuleConsequent();
  Consequence13->addOutput(incremento_nulo_pitch);
  
  FuzzyRule *fuzzyRule13 = new FuzzyRule(13, Rule13, Consequence13);
  fuzzy->addFuzzyRule(fuzzyRule13);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule14 = new FuzzyRuleAntecedent();
  Rule14->joinWithAND(erro_nulo_derivada_pitch, erro_positivo_pequeno_pitch);
  FuzzyRuleConsequent *Consequence14 = new FuzzyRuleConsequent();
  Consequence14->addOutput(incremento_positivo_pequeno_pitch);
  
  FuzzyRule *fuzzyRule14 = new FuzzyRule(14, Rule14, Consequence14);
  fuzzy->addFuzzyRule(fuzzyRule14);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule15 = new FuzzyRuleAntecedent();
  Rule15->joinWithAND(erro_nulo_derivada_pitch, erro_positivo_medio_pitch);
  FuzzyRuleConsequent *Consequence15 = new FuzzyRuleConsequent();
  Consequence15->addOutput(incremento_positivo_pequeno_pitch);
  
  FuzzyRule *fuzzyRule15 = new FuzzyRule(15, Rule15, Consequence15);
  fuzzy->addFuzzyRule(fuzzyRule15);
//
////  // Building FuzzyRule
////  FuzzyRuleAntecedent *Rule28 = new FuzzyRuleAntecedent();
////  Rule28->joinWithAND(erro_nulo_derivada_pitch, erro_positivo_grande_pitch);
////  FuzzyRuleConsequent *Consequence28 = new FuzzyRuleConsequent();
////  Consequence28->addOutput(incremento_positivo_medio_pitch);
////  
////  FuzzyRule *fuzzyRule28 = new FuzzyRule(28, Rule28, Consequence28);
////  fuzzy->addFuzzyRule(fuzzyRule28);
//
////  // Building FuzzyRule
////  FuzzyRuleAntecedent *Rule29 = new FuzzyRuleAntecedent();
////  Rule29->joinWithAND(erro_positivo_pequeno_derivada_pitch, erro_negativo_grande_pitch);
////  FuzzyRuleConsequent *Consequence29 = new FuzzyRuleConsequent();
////  Consequence29->addOutput(incremento_negativo_pequeno_pitch);
////  
////  FuzzyRule *fuzzyRule29 = new FuzzyRule(29, Rule29, Consequence29);
////  fuzzy->addFuzzyRule(fuzzyRule29);
//
  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule16 = new FuzzyRuleAntecedent();
  Rule16->joinWithAND(erro_positivo_pequeno_derivada_pitch, erro_negativo_medio_pitch);
  FuzzyRuleConsequent *Consequence16 = new FuzzyRuleConsequent();
  Consequence16->addOutput(incremento_negativo_pequeno_pitch);
  
  FuzzyRule *fuzzyRule16 = new FuzzyRule(16, Rule16, Consequence16);
  fuzzy->addFuzzyRule(fuzzyRule16);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule17 = new FuzzyRuleAntecedent();
  Rule17->joinWithAND(erro_positivo_pequeno_derivada_pitch, erro_negativo_pequeno_pitch);
  FuzzyRuleConsequent *Consequence17 = new FuzzyRuleConsequent();
  Consequence17->addOutput(incremento_nulo_pitch);
  
  FuzzyRule *fuzzyRule17 = new FuzzyRule(17, Rule17, Consequence17);
  fuzzy->addFuzzyRule(fuzzyRule17);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule18 = new FuzzyRuleAntecedent();
  Rule18->joinWithAND(erro_positivo_pequeno_derivada_pitch, erro_nulo_pitch);
  FuzzyRuleConsequent *Consequence18 = new FuzzyRuleConsequent();
  Consequence18->addOutput(incremento_positivo_pequeno_pitch);
  
  FuzzyRule *fuzzyRule18 = new FuzzyRule(18, Rule18, Consequence18);
  fuzzy->addFuzzyRule(fuzzyRule18);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule19 = new FuzzyRuleAntecedent();
  Rule19->joinWithAND(erro_positivo_pequeno_derivada_pitch, erro_positivo_pequeno_pitch);
  FuzzyRuleConsequent *Consequence19 = new FuzzyRuleConsequent();
  Consequence19->addOutput(incremento_positivo_pequeno_pitch);
  
  FuzzyRule *fuzzyRule19 = new FuzzyRule(19, Rule19, Consequence19);
  fuzzy->addFuzzyRule(fuzzyRule19);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule20 = new FuzzyRuleAntecedent();
  Rule20->joinWithAND(erro_positivo_pequeno_derivada_pitch, erro_positivo_medio_pitch);
  FuzzyRuleConsequent *Consequence20 = new FuzzyRuleConsequent();
  Consequence20->addOutput(incremento_positivo_medio_pitch);
  
  FuzzyRule *fuzzyRule20 = new FuzzyRule(20, Rule20, Consequence20);
  fuzzy->addFuzzyRule(fuzzyRule20);

////  // Building FuzzyRule
////  FuzzyRuleAntecedent *Rule35 = new FuzzyRuleAntecedent();
////  Rule35->joinWithAND(erro_positivo_pequeno_derivada_pitch, erro_positivo_grande_pitch);
////  FuzzyRuleConsequent *Consequence35 = new FuzzyRuleConsequent();
////  Consequence35->addOutput(incremento_positivo_medio_pitch);
////  
////  FuzzyRule *fuzzyRule35 = new FuzzyRule(35, Rule35, Consequence35);
////  fuzzy->addFuzzyRule(fuzzyRule35);
//
////  // Building FuzzyRule
////  FuzzyRuleAntecedent *Rule36 = new FuzzyRuleAntecedent();
////  Rule36->joinWithAND(erro_positivo_medio_derivada_pitch, erro_negativo_grande_pitch);
////  FuzzyRuleConsequent *Consequence36 = new FuzzyRuleConsequent();
////  Consequence36->addOutput(incremento_negativo_pequeno_pitch);
////  
////  FuzzyRule *fuzzyRule36 = new FuzzyRule(36, Rule36, Consequence36);
////  fuzzy->addFuzzyRule(fuzzyRule36);
//
  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule21 = new FuzzyRuleAntecedent();
  Rule21->joinWithAND(erro_positivo_medio_derivada_pitch, erro_negativo_medio_pitch);
  FuzzyRuleConsequent *Consequence21 = new FuzzyRuleConsequent();
  Consequence21->addOutput(incremento_nulo_pitch);
  
  FuzzyRule *fuzzyRule21 = new FuzzyRule(21, Rule21, Consequence21);
  fuzzy->addFuzzyRule(fuzzyRule21);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule22 = new FuzzyRuleAntecedent();
  Rule22->joinWithAND(erro_positivo_medio_derivada_pitch, erro_negativo_pequeno_pitch);
  FuzzyRuleConsequent *Consequence22 = new FuzzyRuleConsequent();
  Consequence22->addOutput(incremento_positivo_pequeno_pitch);
  
  FuzzyRule *fuzzyRule22 = new FuzzyRule(22, Rule22, Consequence22);
  fuzzy->addFuzzyRule(fuzzyRule22);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule23 = new FuzzyRuleAntecedent();
  Rule23->joinWithAND(erro_positivo_medio_derivada_pitch, erro_nulo_pitch);
  FuzzyRuleConsequent *Consequence23 = new FuzzyRuleConsequent();
  Consequence23->addOutput(incremento_positivo_pequeno_pitch);
  
  FuzzyRule *fuzzyRule23 = new FuzzyRule(23, Rule23, Consequence23);
  fuzzy->addFuzzyRule(fuzzyRule23);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule24 = new FuzzyRuleAntecedent();
  Rule24->joinWithAND(erro_positivo_medio_derivada_pitch, erro_positivo_pequeno_pitch);
  FuzzyRuleConsequent *Consequence24 = new FuzzyRuleConsequent();
  Consequence24->addOutput(incremento_positivo_medio_pitch);
  
  FuzzyRule *fuzzyRule24 = new FuzzyRule(24, Rule24, Consequence24);
  fuzzy->addFuzzyRule(fuzzyRule24);

  // Building FuzzyRule
  FuzzyRuleAntecedent *Rule25 = new FuzzyRuleAntecedent();
  Rule25->joinWithAND(erro_positivo_medio_derivada_pitch, erro_positivo_medio_pitch);
  FuzzyRuleConsequent *Consequence25 = new FuzzyRuleConsequent();
  Consequence25->addOutput(incremento_positivo_medio_pitch);
  
  FuzzyRule *fuzzyRule25 = new FuzzyRule(25, Rule25, Consequence25);
  fuzzy->addFuzzyRule(fuzzyRule25);

////  // Building FuzzyRule
////  FuzzyRuleAntecedent *Rule42 = new FuzzyRuleAntecedent();
////  Rule42->joinWithAND(erro_positivo_medio_derivada_pitch, erro_positivo_grande_pitch);
////  FuzzyRuleConsequent *Consequence42 = new FuzzyRuleConsequent();
////  Consequence42->addOutput(incremento_positivo_grande_pitch);
////  
////  FuzzyRule *fuzzyRule42 = new FuzzyRule(42, Rule42, Consequence42);
////  fuzzy->addFuzzyRule(fuzzyRule42);
//
////  // Building FuzzyRule
////  FuzzyRuleAntecedent *Rule43 = new FuzzyRuleAntecedent();
////  Rule43->joinWithAND(erro_positivo_grande_derivada_pitch, erro_negativo_grande_pitch);
////  FuzzyRuleConsequent *Consequence43 = new FuzzyRuleConsequent();
////  Consequence43->addOutput(incremento_nulo_pitch);
////  
////  FuzzyRule *fuzzyRule43 = new FuzzyRule(43, Rule43, Consequence43);
////  fuzzy->addFuzzyRule(fuzzyRule43);
////
////  // Building FuzzyRule
////  FuzzyRuleAntecedent *Rule44 = new FuzzyRuleAntecedent();
////  Rule44->joinWithAND(erro_positivo_grande_derivada_pitch, erro_negativo_medio_pitch);
////  FuzzyRuleConsequent *Consequence44 = new FuzzyRuleConsequent();
////  Consequence44->addOutput(incremento_positivo_pequeno_pitch);
////  
////  FuzzyRule *fuzzyRule44 = new FuzzyRule(44, Rule44, Consequence44);
////  fuzzy->addFuzzyRule(fuzzyRule44);
////
////  // Building FuzzyRule
////  FuzzyRuleAntecedent *Rule45 = new FuzzyRuleAntecedent();
////  Rule45->joinWithAND(erro_positivo_grande_derivada_pitch, erro_negativo_pequeno_pitch);
////  FuzzyRuleConsequent *Consequence45 = new FuzzyRuleConsequent();
////  Consequence45->addOutput(incremento_positivo_medio_pitch);
////  
////  FuzzyRule *fuzzyRule45 = new FuzzyRule(45, Rule45, Consequence45);
////  fuzzy->addFuzzyRule(fuzzyRule45);
////
////  // Building FuzzyRule
////  FuzzyRuleAntecedent *Rule46 = new FuzzyRuleAntecedent();
////  Rule46->joinWithAND(erro_positivo_grande_derivada_pitch, erro_nulo_pitch);
////  FuzzyRuleConsequent *Consequence46 = new FuzzyRuleConsequent();
////  Consequence46->addOutput(incremento_positivo_medio_pitch);
////  
////  FuzzyRule *fuzzyRule46 = new FuzzyRule(46, Rule46, Consequence46);
////  fuzzy->addFuzzyRule(fuzzyRule46);
////
////  // Building FuzzyRule
////  FuzzyRuleAntecedent *Rule47 = new FuzzyRuleAntecedent();
////  Rule47->joinWithAND(erro_positivo_grande_derivada_pitch, erro_positivo_pequeno_pitch);
////  FuzzyRuleConsequent *Consequence47 = new FuzzyRuleConsequent();
////  Consequence47->addOutput(incremento_positivo_grande_pitch);
////  
////  FuzzyRule *fuzzyRule47 = new FuzzyRule(47, Rule47, Consequence47);
////  fuzzy->addFuzzyRule(fuzzyRule47);
////
////  // Building FuzzyRule
////  FuzzyRuleAntecedent *Rule48 = new FuzzyRuleAntecedent();
////  Rule48->joinWithAND(erro_positivo_grande_derivada_pitch, erro_positivo_medio_pitch);
////  FuzzyRuleConsequent *Consequence48 = new FuzzyRuleConsequent();
////  Consequence48->addOutput(incremento_positivo_grande_pitch);
////  
////  FuzzyRule *fuzzyRule48 = new FuzzyRule(48, Rule48, Consequence48);
////  fuzzy->addFuzzyRule(fuzzyRule48);
////
////  // Building FuzzyRule
////  FuzzyRuleAntecedent *Rule49 = new FuzzyRuleAntecedent();
////  Rule49->joinWithAND(erro_positivo_grande_derivada_pitch, erro_positivo_grande_pitch);
////  FuzzyRuleConsequent *Consequence49 = new FuzzyRuleConsequent();
////  Consequence49->addOutput(incremento_positivo_grande_pitch);
////  
////  FuzzyRule *fuzzyRule49 = new FuzzyRule(49, Rule49, Consequence49);
////  fuzzy->addFuzzyRule(fuzzyRule49);
  
  // ------------- ROLL -------------

//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule26 = new FuzzyRuleAntecedent();
//  Rule26->joinWithAND(erro_negativo_medio_derivada_roll, erro_negativo_medio_roll);
//  FuzzyRuleConsequent *Consequence26 = new FuzzyRuleConsequent();
//  Consequence26->addOutput(incremento_negativo_medio_roll);
//  
//  FuzzyRule *fuzzyRule26 = new FuzzyRule(26, Rule26, Consequence26);
//  fuzzy->addFuzzyRule(fuzzyRule26);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule27 = new FuzzyRuleAntecedent();
//  Rule27->joinWithAND(erro_negativo_medio_derivada_roll, erro_negativo_pequeno_roll);
//  FuzzyRuleConsequent *Consequence27 = new FuzzyRuleConsequent();
//  Consequence27->addOutput(incremento_negativo_medio_roll);
//  
//  FuzzyRule *fuzzyRule27 = new FuzzyRule(27, Rule27, Consequence27);
//  fuzzy->addFuzzyRule(fuzzyRule27);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule28 = new FuzzyRuleAntecedent();
//  Rule28->joinWithAND(erro_negativo_medio_derivada_roll, erro_nulo_roll);
//  FuzzyRuleConsequent *Consequence28 = new FuzzyRuleConsequent();
//  Consequence28->addOutput(incremento_negativo_pequeno_roll);
//  
//  FuzzyRule *fuzzyRule28 = new FuzzyRule(28, Rule28, Consequence28);
//  fuzzy->addFuzzyRule(fuzzyRule28);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule29 = new FuzzyRuleAntecedent();
//  Rule29->joinWithAND(erro_negativo_medio_derivada_roll, erro_positivo_pequeno_roll);
//  FuzzyRuleConsequent *Consequence29 = new FuzzyRuleConsequent();
//  Consequence29->addOutput(incremento_negativo_pequeno_roll);
//  
//  FuzzyRule *fuzzyRule29 = new FuzzyRule(29, Rule29, Consequence29);
//  fuzzy->addFuzzyRule(fuzzyRule29);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule30 = new FuzzyRuleAntecedent();
//  Rule30->joinWithAND(erro_negativo_medio_derivada_roll, erro_positivo_medio_roll);
//  FuzzyRuleConsequent *Consequence30 = new FuzzyRuleConsequent();
//  Consequence30->addOutput(incremento_nulo_roll);
//  
//  FuzzyRule *fuzzyRule30 = new FuzzyRule(30, Rule30, Consequence30);
//  fuzzy->addFuzzyRule(fuzzyRule30);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule31 = new FuzzyRuleAntecedent();
//  Rule31->joinWithAND(erro_negativo_pequeno_derivada_roll, erro_negativo_medio_roll);
//  FuzzyRuleConsequent *Consequence31 = new FuzzyRuleConsequent();
//  Consequence31->addOutput(incremento_negativo_medio_roll);
//  
//  FuzzyRule *fuzzyRule31 = new FuzzyRule(31, Rule31, Consequence31);
//  fuzzy->addFuzzyRule(fuzzyRule31);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule32 = new FuzzyRuleAntecedent();
//  Rule32->joinWithAND(erro_negativo_pequeno_derivada_roll, erro_negativo_pequeno_roll);
//  FuzzyRuleConsequent *Consequence32 = new FuzzyRuleConsequent();
//  Consequence32->addOutput(incremento_negativo_pequeno_roll);
//  
//  FuzzyRule *fuzzyRule32 = new FuzzyRule(32, Rule32, Consequence32);
//  fuzzy->addFuzzyRule(fuzzyRule32);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule33 = new FuzzyRuleAntecedent();
//  Rule33->joinWithAND(erro_negativo_pequeno_derivada_roll, erro_nulo_roll);
//  FuzzyRuleConsequent *Consequence33 = new FuzzyRuleConsequent();
//  Consequence33->addOutput(incremento_negativo_pequeno_roll);
//  
//  FuzzyRule *fuzzyRule33 = new FuzzyRule(33, Rule33, Consequence33);
//  fuzzy->addFuzzyRule(fuzzyRule33);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule34 = new FuzzyRuleAntecedent();
//  Rule34->joinWithAND(erro_negativo_pequeno_derivada_roll, erro_positivo_pequeno_roll);
//  FuzzyRuleConsequent *Consequence34 = new FuzzyRuleConsequent();
//  Consequence34->addOutput(incremento_nulo_roll);
//  
//  FuzzyRule *fuzzyRule34 = new FuzzyRule(34, Rule34, Consequence34);
//  fuzzy->addFuzzyRule(fuzzyRule34);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule35 = new FuzzyRuleAntecedent();
//  Rule35->joinWithAND(erro_negativo_pequeno_derivada_roll, erro_positivo_medio_roll);
//  FuzzyRuleConsequent *Consequence35 = new FuzzyRuleConsequent();
//  Consequence35->addOutput(incremento_positivo_pequeno_roll);
//  
//  FuzzyRule *fuzzyRule35 = new FuzzyRule(35, Rule35, Consequence35);
//  fuzzy->addFuzzyRule(fuzzyRule35);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule36 = new FuzzyRuleAntecedent();
//  Rule36->joinWithAND(erro_nulo_derivada_roll, erro_negativo_medio_roll);
//  FuzzyRuleConsequent *Consequence36 = new FuzzyRuleConsequent();
//  Consequence36->addOutput(incremento_negativo_pequeno_roll);
//  
//  FuzzyRule *fuzzyRule36 = new FuzzyRule(36, Rule36, Consequence36);
//  fuzzy->addFuzzyRule(fuzzyRule36);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule37 = new FuzzyRuleAntecedent();
//  Rule37->joinWithAND(erro_nulo_derivada_roll, erro_negativo_pequeno_roll);
//  FuzzyRuleConsequent *Consequence37 = new FuzzyRuleConsequent();
//  Consequence37->addOutput(incremento_negativo_pequeno_roll);
//  
//  FuzzyRule *fuzzyRule37 = new FuzzyRule(37, Rule37, Consequence37);
//  fuzzy->addFuzzyRule(fuzzyRule37);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule38 = new FuzzyRuleAntecedent();
//  Rule38->joinWithAND(erro_nulo_derivada_roll, erro_nulo_roll);
//  FuzzyRuleConsequent *Consequence38 = new FuzzyRuleConsequent();
//  Consequence38->addOutput(incremento_nulo_roll);
//  
//  FuzzyRule *fuzzyRule38 = new FuzzyRule(38, Rule38, Consequence38);
//  fuzzy->addFuzzyRule(fuzzyRule38);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule39 = new FuzzyRuleAntecedent();
//  Rule39->joinWithAND(erro_nulo_derivada_roll, erro_positivo_pequeno_roll);
//  FuzzyRuleConsequent *Consequence39 = new FuzzyRuleConsequent();
//  Consequence39->addOutput(incremento_positivo_pequeno_roll);
//  
//  FuzzyRule *fuzzyRule39 = new FuzzyRule(39, Rule39, Consequence39);
//  fuzzy->addFuzzyRule(fuzzyRule39);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule40 = new FuzzyRuleAntecedent();
//  Rule40->joinWithAND(erro_nulo_derivada_roll, erro_positivo_medio_roll);
//  FuzzyRuleConsequent *Consequence40 = new FuzzyRuleConsequent();
//  Consequence40->addOutput(incremento_positivo_pequeno_roll);
//  
//  FuzzyRule *fuzzyRule40 = new FuzzyRule(40, Rule40, Consequence40);
//  fuzzy->addFuzzyRule(fuzzyRule40);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule41 = new FuzzyRuleAntecedent();
//  Rule41->joinWithAND(erro_negativo_pequeno_derivada_roll, erro_negativo_medio_roll);
//  FuzzyRuleConsequent *Consequence41 = new FuzzyRuleConsequent();
//  Consequence41->addOutput(incremento_negativo_medio_roll);
//  
//  FuzzyRule *fuzzyRule41 = new FuzzyRule(41, Rule41, Consequence41);
//  fuzzy->addFuzzyRule(fuzzyRule41);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule42 = new FuzzyRuleAntecedent();
//  Rule42->joinWithAND(erro_negativo_pequeno_derivada_roll, erro_negativo_pequeno_roll);
//  FuzzyRuleConsequent *Consequence42 = new FuzzyRuleConsequent();
//  Consequence42->addOutput(incremento_negativo_pequeno_roll);
//  
//  FuzzyRule *fuzzyRule42 = new FuzzyRule(42, Rule42, Consequence42);
//  fuzzy->addFuzzyRule(fuzzyRule42);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule43 = new FuzzyRuleAntecedent();
//  Rule43->joinWithAND(erro_negativo_pequeno_derivada_roll, erro_nulo_roll);
//  FuzzyRuleConsequent *Consequence43 = new FuzzyRuleConsequent();
//  Consequence43->addOutput(incremento_negativo_pequeno_roll);
//  
//  FuzzyRule *fuzzyRule43 = new FuzzyRule(43, Rule43, Consequence43);
//  fuzzy->addFuzzyRule(fuzzyRule43);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule44 = new FuzzyRuleAntecedent();
//  Rule44->joinWithAND(erro_negativo_pequeno_derivada_roll, erro_positivo_pequeno_roll);
//  FuzzyRuleConsequent *Consequence44 = new FuzzyRuleConsequent();
//  Consequence44->addOutput(incremento_nulo_roll);
//  
//  FuzzyRule *fuzzyRule44 = new FuzzyRule(44, Rule44, Consequence44);
//  fuzzy->addFuzzyRule(fuzzyRule44);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule45 = new FuzzyRuleAntecedent();
//  Rule45->joinWithAND(erro_negativo_pequeno_derivada_roll, erro_positivo_medio_roll);
//  FuzzyRuleConsequent *Consequence45 = new FuzzyRuleConsequent();
//  Consequence45->addOutput(incremento_positivo_pequeno_roll);
//  
//  FuzzyRule *fuzzyRule45 = new FuzzyRule(45, Rule45, Consequence45);
//  fuzzy->addFuzzyRule(fuzzyRule45);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule46 = new FuzzyRuleAntecedent();
//  Rule46->joinWithAND(erro_positivo_medio_derivada_roll, erro_negativo_medio_roll);
//  FuzzyRuleConsequent *Consequence46 = new FuzzyRuleConsequent();
//  Consequence46->addOutput(incremento_nulo_roll);
//  
//  FuzzyRule *fuzzyRule46 = new FuzzyRule(46, Rule46, Consequence46);
//  fuzzy->addFuzzyRule(fuzzyRule46);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule47 = new FuzzyRuleAntecedent();
//  Rule47->joinWithAND(erro_positivo_medio_derivada_roll, erro_negativo_pequeno_roll);
//  FuzzyRuleConsequent *Consequence47 = new FuzzyRuleConsequent();
//  Consequence47->addOutput(incremento_positivo_pequeno_roll);
//  
//  FuzzyRule *fuzzyRule47 = new FuzzyRule(47, Rule47, Consequence47);
//  fuzzy->addFuzzyRule(fuzzyRule47);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule48 = new FuzzyRuleAntecedent();
//  Rule48->joinWithAND(erro_positivo_medio_derivada_roll, erro_nulo_roll);
//  FuzzyRuleConsequent *Consequence48 = new FuzzyRuleConsequent();
//  Consequence48->addOutput(incremento_positivo_pequeno_roll);
//  
//  FuzzyRule *fuzzyRule48 = new FuzzyRule(48, Rule48, Consequence48);
//  fuzzy->addFuzzyRule(fuzzyRule48);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule49 = new FuzzyRuleAntecedent();
//  Rule49->joinWithAND(erro_positivo_medio_derivada_roll, erro_positivo_pequeno_roll);
//  FuzzyRuleConsequent *Consequence49 = new FuzzyRuleConsequent();
//  Consequence49->addOutput(incremento_positivo_medio_roll);
//  
//  FuzzyRule *fuzzyRule49 = new FuzzyRule(49, Rule49, Consequence49);
//  fuzzy->addFuzzyRule(fuzzyRule49);
//
//  // Building FuzzyRule
//  FuzzyRuleAntecedent *Rule50 = new FuzzyRuleAntecedent();
//  Rule50->joinWithAND(erro_positivo_medio_derivada_roll, erro_positivo_medio_roll);
//  FuzzyRuleConsequent *Consequence50 = new FuzzyRuleConsequent();
//  Consequence50->addOutput(incremento_positivo_medio_roll);
//  
//  FuzzyRule *fuzzyRule50 = new FuzzyRule(50, Rule50, Consequence50);
//  fuzzy->addFuzzyRule(fuzzyRule50);
  
  // -------------------------------------------------------------------------------------
  

  // Calibration
  Serial.println("=====================================");
  Serial.println("Starting calibration...");

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  motor1DC.setPeriodHertz(50);      // Standard 50hz servo
  motor2DC.setPeriodHertz(50);      // Standard 50hz servo
  motor3DC.setPeriodHertz(50);      // Standard 50hz servo
  motor4DC.setPeriodHertz(50);      // Standard 50hz servo

  motor1DC.attach(MOTOR1_PIN, MIN_SIGNAL, MAX_SIGNAL);
  motor2DC.attach(MOTOR2_PIN, MIN_SIGNAL, MAX_SIGNAL);
  motor3DC.attach(MOTOR3_PIN, MIN_SIGNAL, MAX_SIGNAL);
  motor4DC.attach(MOTOR4_PIN, MIN_SIGNAL, MAX_SIGNAL);

  // Desliga os motores
  Serial.println("Motor Desligado");
  motor1DC.writeMicroseconds(MIN_SIGNAL);
  motor2DC.writeMicroseconds(MIN_SIGNAL);
  motor3DC.writeMicroseconds(MIN_SIGNAL);
  motor4DC.writeMicroseconds(MIN_SIGNAL);
  motorLigado = false;
  
  SerialBT.begin("Drone");
  Serial.println("Dispositivo disponível para pareamento");
}

void loop() {
  if(SerialBT.available()) {
    String aux = SerialBT.readString();
    if(aux != "")
      valorRecebido = aux;

    if(valorRecebido != "") {
      Serial.print("Valor:" + aux);
      Serial.println(".FIM");
      if(valorRecebido.startsWith("0")) {
        abortarMissao = true;
        testeBase = false;
      } else if(valorRecebido.startsWith("1")) {
        SerialBT.println("Iniciando Controle de Vôo");
        abortarMissao = false;
        testeBase = false;
      } else if (valorRecebido.startsWith("A") || valorRecebido.startsWith("a")) {
        alturaRef = valorRecebido.substring(1).toDouble();
        SerialBT.println("Nova Referência de Altura: " + String(alturaRef));
      } else if (valorRecebido.startsWith("B") || valorRecebido.startsWith("b")) {
        baseRef = valorRecebido.substring(1).toDouble();
        SerialBT.println("Nova Referência de Base: " + String(baseRef));
        // acionaBase();
      } else if (valorRecebido.startsWith("G") || valorRecebido.startsWith("g")) {
        gainMotor = valorRecebido.substring(1).toDouble();
        SerialBT.println("Nova Referência de Ganho: " + String(gainMotor));
      } else if (valorRecebido.startsWith("D") || valorRecebido.startsWith("d")) {
        delaySet = valorRecebido.substring(1).toInt();
        SerialBT.println("Nova Referência de Delay: " + String(delaySet));
      } else if (valorRecebido.startsWith("T") || valorRecebido.startsWith("t")) {
        testeBase = true;
        acionaBase();
      } else if (valorRecebido.startsWith("M") || valorRecebido.startsWith("m")) {
        double setMotor = valorRecebido.substring(1).toDouble();
        if(setMotor == 1) acionaMotor1();
        else if(setMotor == 2) acionaMotor2();
        else if(setMotor == 3) acionaMotor3();
        else if(setMotor == 4) acionaMotor4();
        else {
          // Desligando os motores
          motor1DC.writeMicroseconds(MIN_SIGNAL);
          motor2DC.writeMicroseconds(MIN_SIGNAL);
          motor3DC.writeMicroseconds(MIN_SIGNAL);
          motor4DC.writeMicroseconds(MIN_SIGNAL);
          motorLigado = false;
          abortarMissao = true;
        }
      }
      valorRecebido = "";
    }
  }

  if(!abortarMissao) {
    controlaVoo();
  } else {
    abortarVoo();
  }

  delay(delaySet);
}

void abortarVoo() {
  if(motorLigado && !testeBase) {
    Serial.println("Desligando motores");
    SerialBT.println("Abortando Vôo");
    
    // Desligando os motores
    motor1DC.writeMicroseconds(MIN_SIGNAL);
    motor2DC.writeMicroseconds(MIN_SIGNAL);
    motor3DC.writeMicroseconds(MIN_SIGNAL);
    motor4DC.writeMicroseconds(MIN_SIGNAL);
    motorLigado = false;
  }
}

void controlaVoo() {
  unsigned long initMillis = millis(); //VARIÁVEL RECEBE O TEMPO ATUAL EM MILISSEGUNDOS
  int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

  double x;
  double y;
  double z;
  
  double angleX = 0.0;
  double angleY = 0.0;
  double angleZ = 0.0;

  double C_angleX = 0.0;
  double C_angleY = 0.0;
  double C_angleZ = 0.0;

  double preAngle[3] = {0.0, 0.0, 0.0};
  
//  float preAngleX = 0.0;
//  float preAngleY = 0.0;
//  float preAngleZ = 0.0;
  
  double dAngleX = 0.0;
  double dAngleY = 0.0;
  double dAngleZ = 0.0;
  
  int maiordAngleX = 0;
  int maiordAngleY = 0;
  int maiordAngleZ = 0;
  
  double resPitch = 0.0;
  double resRoll = 0.0;
  
  double resM1 = 0.0;
  double resM2 = 0.0;
  double resM3 = 0.0;
  double resM4 = 0.0;

  int i = 0;
  
  while(i < 2) {
    int count = 0, num = 35, critico = 0;
    
    while(count < num && critico < num) {
      Wire.beginTransmission(MPU_addr);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_addr,14,true);
      AcX=Wire.read()<<8|Wire.read();
      AcY=Wire.read()<<8|Wire.read();
      AcZ=Wire.read()<<8|Wire.read();
      int xAng = map(AcX,minVal,maxVal,-90,90);
      int yAng = map(AcY,minVal,maxVal,-90,90);
      int zAng = map(AcZ,minVal,maxVal,-90,90);
       
      x = RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
      y = RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
      angleZ = RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
  
      if(x > 180) x = x - 360;
      if(y > 180) y = y - 360;
      if(z > 180) z = z - 360;

      if(abs(x) >= criticalAngle || abs(y) >= criticalAngle) {
        C_angleX = C_angleX + x;
        C_angleY = C_angleY + y;
        C_angleZ = C_angleZ + z;
        
        critico++;
      } else {
        angleX = angleX + x;
        angleY = angleY + y;
        angleZ = angleZ + z;

        count++;
      }
    }

    if(count == num) {
      angleX = (angleX / count);
      angleY = (angleY / count) - 2;
      angleZ = (angleZ / count);
      count = 0;
    
      if(i == 0) {
        preAngle[0] = angleX * 1.0;
        preAngle[1] = angleY * 1.0;
        preAngle[2] = angleZ * 1.0;
      }
    } else {
      C_angleX = (C_angleX / critico);
      C_angleY = (C_angleY / critico) - 2;
      C_angleZ = (C_angleZ / critico);
      count = 0;
    
      if(i == 0) {
        preAngle[0] = C_angleX * 1.0;
        preAngle[1] = C_angleY * 1.0;
        preAngle[2] = C_angleZ * 1.0;
      }
    }
    
    delay(20);
    i++;
  }

  if(critico == num) {
    
    dAngleX = C_angleX - preAngle[0];
    dAngleY = C_angleY - preAngle[1];
    dAngleZ = C_angleZ - preAngle[2];
  
    preAngle[0] = C_angleX;
    preAngle[1] = C_angleY;
    preAngle[2] = C_angleZ;

    filtroSeguranca(C_angleX, C_angleY, C_angleZ);

    unsigned long finalMillis = millis() - initMillis;
    // Txt para CSV
    Serial.println(String(angleY) + "," + String(dAngleY) + "," + String(resPitch) + "," + String(resM1) + "," + String(resM2) + "," + String(resM3) + "," + String(resM4) + "," + String(motor1Fuzzy) + "," + String(motor2Fuzzy) + "," + String(motor3Fuzzy) + "," + String(motor4Fuzzy) + "," + String(finalMillis));

  } else {
    
    dAngleX = angleX - preAngle[0];
    dAngleY = angleY - preAngle[1];
    dAngleZ = angleZ - preAngle[2];
  
    preAngle[0] = angleX;
    preAngle[1] = angleY;
    preAngle[2] = angleZ;
  
    resM1 = 0.0;
    resM2 = 0.0;
    resM3 = 0.0;
    resM4 = 0.0;
    if(abs(angleY) <= 25 && abs(dAngleY) <= 6) {
      fuzzy->setInput(1, (1) * angleY);
      fuzzy->setInput(2, (1) * dAngleY);
      // fuzzy->setInput(3, (-1) * angleX);
      // fuzzy->setInput(4, (-1) * dAngleX);
      fuzzy->fuzzify();

      resPitch = gainMotor * fuzzy->defuzzify(1);
      // resRoll = gainMotor * fuzzy->defuzzify(2);

      if(resPitch < 0) {
       resM1 = (-1) * resPitch;
       resM3 = (-1) * resPitch;
      } else {
       resM2 = resPitch;
       resM4 = resPitch;
      }

      motor1Fuzzy += resM1;
      motor2Fuzzy += resM2;
      motor3Fuzzy += resM3;
      motor4Fuzzy += resM4;
    }
  
    // unsigned long fuzzyTime = millis() - initMillis;
    
    filtroSeguranca(angleX, angleY, angleZ);
    // gravarDados(fuzzyTime);
  
    unsigned long finalMillis = millis() - initMillis;
    // Txt para CSV
    Serial.println(String(angleY) + "," + String(dAngleY) + "," + String(resPitch) + "," + String(resM1) + "," + String(resM2) + "," + String(resM3) + "," + String(resM4) + "," + String(motor1Fuzzy) + "," + String(motor2Fuzzy) + "," + String(motor3Fuzzy) + "," + String(motor4Fuzzy) + "," + String(finalMillis));
  
    if(!abortarMissao) {
        // Ajusta os motores
        motor1DC.writeMicroseconds(base + motor1Fuzzy);
        motor2DC.writeMicroseconds(base + motor2Fuzzy);
        motor3DC.writeMicroseconds(base + motor3Fuzzy);
        motor4DC.writeMicroseconds(base + motor4Fuzzy);
        motorLigado = true;   
    } else {
      // Ajusta os motores
      motor1DC.writeMicroseconds(MIN_SIGNAL);
      motor2DC.writeMicroseconds(MIN_SIGNAL);
      motor3DC.writeMicroseconds(MIN_SIGNAL);
      motor4DC.writeMicroseconds(MIN_SIGNAL);
      motorLigado = false;
    }
  }
  delay(20);
}

void acionaBase() {
  if(!motorLigado) {
    motor1DC.writeMicroseconds(base);
    motor2DC.writeMicroseconds(base);
    motor3DC.writeMicroseconds(base);
    motor4DC.writeMicroseconds(base);
    SerialBT.println("Base Liga");
    motorLigado = true;
  } else {
    motor1DC.writeMicroseconds(MIN_SIGNAL);
    motor2DC.writeMicroseconds(MIN_SIGNAL);
    motor3DC.writeMicroseconds(MIN_SIGNAL);
    motor4DC.writeMicroseconds(MIN_SIGNAL);
    SerialBT.println("Base Desliga");
    motorLigado = false;
  }
}

void acionaMotor1() {
  motor1DC.writeMicroseconds(base);
  motor2DC.writeMicroseconds(MIN_SIGNAL);
  motor3DC.writeMicroseconds(MIN_SIGNAL);
  motor4DC.writeMicroseconds(MIN_SIGNAL);
  SerialBT.println("Motor 1");
  motorLigado = true;
}

void acionaMotor2() {
  motor1DC.writeMicroseconds(MIN_SIGNAL);
  motor2DC.writeMicroseconds(base);
  motor3DC.writeMicroseconds(MIN_SIGNAL);
  motor4DC.writeMicroseconds(MIN_SIGNAL);
  SerialBT.println("Motor 2");
  motorLigado = true;
}

void acionaMotor3() {
  motor1DC.writeMicroseconds(MIN_SIGNAL);
  motor2DC.writeMicroseconds(MIN_SIGNAL);
  motor3DC.writeMicroseconds(base);
  motor4DC.writeMicroseconds(MIN_SIGNAL);
  SerialBT.println("Motor 3");
  motorLigado = true;
}

void acionaMotor4() {
  motor1DC.writeMicroseconds(MIN_SIGNAL);
  motor2DC.writeMicroseconds(MIN_SIGNAL);
  motor3DC.writeMicroseconds(MIN_SIGNAL);
  motor4DC.writeMicroseconds(base);
  SerialBT.println("Motor 4");
  motorLigado = true;
}

void filtroSeguranca(float angleX, float angleY, float angleZ) {
  if(angleX > criticalAngle || angleY > criticalAngle) {
    abortarMissao = true;
    SerialBT.println("Abortando Vôo: Ângulo acima do valor crítico");
  } else {
    // verificar se a base ultrapassou o limite
    if(base > limiteBase) {
      base = limiteBase;
      SerialBT.println("Valor Limite Máximo de Base Atingido");
    } else if (base < 1200) {
      base = 1200;
      SerialBT.println("Valor Limite Mínimo de Base Atingido");
    }

    if(motor1Fuzzy > securityNumber && motor2Fuzzy > securityNumber && motor3Fuzzy > securityNumber && motor4Fuzzy > securityNumber) {
      Serial.println("Reduzindo Valor");
      motor1Fuzzy -= 2;
      motor2Fuzzy -= 2;
      motor3Fuzzy -= 2;
      motor4Fuzzy -= 2;
    }

    if(baseRef > 1290) {
    if(base < baseRef) base += 1;
    else if(base > baseRef) base -= 1;
    } else base = baseRef;
    
    if(motor1Fuzzy > 30) motor1Fuzzy = 30;
    if(motor2Fuzzy > 30) motor2Fuzzy = 30;
    if(motor3Fuzzy > 30) motor3Fuzzy = 30;
    if(motor4Fuzzy > 30) motor4Fuzzy = 30;
  }
}

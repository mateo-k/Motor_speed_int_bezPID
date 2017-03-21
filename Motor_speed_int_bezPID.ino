#include <PID_v1.h>
#include <limits.h>
#include <float.h>

//Stale ADC
const long AREF = 5000;
const float ADC_MAXVAL = 1023.0;

const double MAX_POS_ERROR = 1000;

const int ANALOG_INPUTS = 8;

const int PROCENT = 100;

//Podlaczenie pinow
const int LED = 13;

const int POT1_PIN = A7;
const int POT2_PIN = A0;
const int POT1_ADC_INPUT = 7;
const int POT2_ADC_INPUT = 0;
const int POT1_VCC_PIN = 4;
const int POT2_VCC_PIN = 5;
const int POT1_MAX_VAL = 4990;
const int POT2_MAX_VAL = 5000;

const int Imcu_PIN = A1;
const int Idrv_PIN = A6;
const int Imcu_ADC_INPUT = 1;
const int Idrv_ADC_INPUT = 6; // motor current

const int Vcc_PIN = A2;
const int Vbatt_PIN = A3;
const int Vcc_ADC_INPUT = 2;
const int Vbatt_ADC_INPUT = 3;

//Podlaczenie sterownika silnika
const int PWM_MAX_PIN = 9;
const int DIR_MAX_PIN = 8;
const int nFAULT_MAX_PIN = 7;
const int nEN_MAX_PIN = 6;

//Podlaczenie enkodera silnika
const int ENC_A_PIN = 2;
const int ENC_B_PIN = 3;

//Filtracja enkodera
const int MIN_PERIOD = 0;

//Orientacja rotora
const int CUR_SURGE = 129;  // prąd krańcówki (mA)
const int N_AVG_CUR = 10;   // bufor uśredniania prądu

volatile bool last_enc_a;
volatile bool last_enc_b;
volatile bool fault;

volatile long last_change_tp_a = 0;
volatile long last_change_tp_b = 0;

volatile long last_change_a_per;
volatile long last_change_b_per;
volatile long lowest_change_a_per = LONG_MAX;
volatile long lowest_change_b_per = LONG_MAX;

volatile long position = 0;
volatile long max_pos;
volatile long min_pos;


volatile short cur_act;
volatile short cur_avg;

const long USEC_IN_SEC = 1000000;

const int MAX_PWM = 255;

const long MAX_MOT_PERIOD = 20000;

//////////////////////////////// PID controller
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double Error = FLT_MAX ;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,0.1,0.00,0.00, DIRECT);


//Parametry czujnikow pradu
const int Imcu_CUR_SENSIVITY = 132;//.6;//mV per Amper
const int Imcu_CUR_IMBALANCE = -0.00;
const int Idrv_CUR_SENSIVITY = 132;//.6;//mV per Amper
const int Idrv_CUR_IMBALANCE = -0.00;


volatile int speed;

void setup() {
  pinMode(LED, OUTPUT);

  //Configure pins for potentiometers
  pinMode(POT1_VCC_PIN, OUTPUT);
  digitalWrite(POT1_VCC_PIN, HIGH);
  //pinMode(POT2_VCC_PIN, OUTPUT);
  //digitalWrite(POT2_VCC_PIN, HIGH);
  pinMode(POT2_VCC_PIN, INPUT_PULLUP);
  pinMode(POT2_PIN, INPUT_PULLUP);

  //Configure pins for motor drv
  pinMode(PWM_MAX_PIN, OUTPUT);
  pinMode(DIR_MAX_PIN, OUTPUT);
  //pinMode(eEN_MAX_PIN, OUTPUT);

  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);

  //Configure motor encoders
  last_enc_a = digitalRead(ENC_A_PIN);
  last_enc_b = digitalRead(ENC_B_PIN);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), Enc_a_ISR, CHANGE );
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), Enc_b_ISR, CHANGE );

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  Setpoint = 0;
  Input = 0;

  //turn the PID on
  myPID.SetOutputLimits(-255.,+255.);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);

  orient_motor();
}

void loop() {

//  int pot1 = (PROCENT * AREF * analogRead(POT1_ADC_INPUT) ) / (POT1_MAX_VAL * (long)ADC_MAXVAL );
//  int pwm = ((pot1 - PROCENT / 2) * MAX_PWM) / (PROCENT / 2);
//  bool dir = pwm > 0 ? 1 : 0 ;
//  pwm = pwm > 0 ? pwm : -pwm;

  if (fault)
    digitalWrite(LED, HIGH);
  else
    digitalWrite(LED, LOW);
  fault = false;

 //------------------------------TODO
 //Zmienic skalowanie potencjometru z 128 na od 10 do 90%.
  //set_motor_pos(pot1*128);
  
  go();
}

void MotorControl()
{
  static long loc_lowest_change_a_per;
  static long loc_lowest_change_b_per;
  static long printable_lowest_change_a_per;
  static long printable_lowest_change_b_per;
  static long loc_position;
  static long loc_last_change_a_per;
  static long loc_last_change_b_per;


  noInterrupts();
  loc_lowest_change_a_per = lowest_change_a_per;
  lowest_change_a_per = LONG_MAX;
  interrupts();

  if (loc_lowest_change_a_per < LONG_MAX)
    printable_lowest_change_a_per = loc_lowest_change_a_per;
  else
    printable_lowest_change_a_per = LONG_MAX;

  noInterrupts();
  loc_lowest_change_b_per = lowest_change_b_per;
  lowest_change_b_per = LONG_MAX;
  interrupts();
  printable_lowest_change_b_per = LONG_MAX;

  if (loc_lowest_change_b_per < LONG_MAX)
    printable_lowest_change_b_per = loc_lowest_change_b_per;
  else
    printable_lowest_change_b_per = LONG_MAX;

  noInterrupts();
  loc_position = position;
  interrupts();

  noInterrupts();
  loc_last_change_a_per = last_change_a_per;
  interrupts();

  noInterrupts();
  loc_last_change_b_per = last_change_b_per;
  interrupts();

  unsigned int freq_a = USEC_IN_SEC / loc_last_change_a_per;
  unsigned int freq_b = USEC_IN_SEC / loc_last_change_b_per;

  unsigned int max_freq_a = USEC_IN_SEC / loc_lowest_change_a_per;
  unsigned int max_freq_b = USEC_IN_SEC / loc_lowest_change_b_per;

  // przerwanie z kanału B enkodera wydaje się przychodzić częściej niż
  // powinno. Porównaj kanał A do B na oscyloskopie.
  Serial.print(loc_position);
  Serial.print("\t");
  Serial.print(freq_a);
  Serial.print("\t");
  Serial.print(freq_b);
  Serial.print("\t");
  Serial.print(max_freq_a);
  Serial.print("\t");
  Serial.print(max_freq_b);
  Serial.print("\t");
  Serial.print(printable_lowest_change_a_per);
  Serial.print("\t");
  Serial.print(printable_lowest_change_b_per);
  Serial.print("\n");

  delay(1);        // delay in between reads for stability

}

void go()
{
 
  noInterrupts();
  long pos = position;
  interrupts();

  int pwm = 0; // stop by default
  if( (digitalRead(POT2_VCC_PIN) == 0 ) && (digitalRead(POT2_PIN) == 0 ) )
  {
    Serial.print("11");
  }
  else if(digitalRead(POT2_VCC_PIN) == 0 )
  {
    if(pos < (max_pos - 1500))
      pwm = 255;
    Serial.print("10");  
  }
  else if(digitalRead(POT2_PIN) == 0 )
  {
    if(pos > (min_pos + 2500))
      pwm = -255;
    Serial.print("01");
  }
  else
  {
    Serial.print("00"); 
  }
  drive_motor(pwm);

//#define DRY_TEST
#ifdef DRY_TEST
{
  min_pos = -9000;
  max_pos = 9000;
  
  if(pwm > 0 ) position+=500;
  if(pwm < 0 ) position-=500;
  delay(200);
}
#endif

  Serial.print("\tpwm: ");
  Serial.print(pwm);
  Serial.print("\tpoz: ");
  Serial.print(pos);
  Serial.print("\t,pozycja min: ");
  Serial.print(min_pos);
  Serial.print("\t,pozycja max:");
  Serial.print(max_pos);
  Serial.print("\n");

}



void drive_motor(int pwm) // pwm = [-255:255]
{
  bool dir = pwm < 0 ? 1 : 0 ;
  pwm = pwm > 0 ? pwm : -pwm;

  digitalWrite(DIR_MAX_PIN, dir);
  if (pwm == 255) digitalWrite(PWM_MAX_PIN, HIGH);
  else if (pwm > 100) analogWrite(PWM_MAX_PIN, pwm);
  else digitalWrite(PWM_MAX_PIN, LOW);
}

void orient_motor()
{
  Serial.print("\n");
  Serial.print("Ustawianie pozycji zerowej...");
  Serial.print("\n");

  Serial.print("ustaw skrajna pozycje lewa (min)");
  Serial.print("\n");
  delay(1000);
  drive_motor(255);
  delay(30);
  drive_motor(160);         // wlacz maks. moc silnika w jedna strone
  delay(50);
  
  while( true )   // czekaj na wzrost mocy - krancowka
  {
    if (motor_stop()) break ;    
    delay(1);
  }
   
  drive_motor(0);   
  noInterrupts();
  long loc_max_pos = position ;  // zachowaj pozycje enkodera
  interrupts();
  
  Serial.print("pozycja ustawiona (min): ");
  Serial.print(loc_max_pos);
  Serial.print("\n");
  
  Serial.print("ustaw skrajna pozycje prawa (max)");
  Serial.print("\n");

  delay(100);
  drive_motor(-255);
  delay(30);
  drive_motor(-160) ;        // wlacz maks. moc silnika w druga strone
  delay(50);
  
  while( true )   // czekaj na wzrost mocy - krancowka
  {
    if (motor_stop()) break ;
    delay(1);
  };
  
  drive_motor(0);  
  noInterrupts();
  long loc_min_pos = position ; // zachowaj pozycje enkodera
  interrupts();
  
  Serial.print("pozycja ustawiona (max):");
  Serial.print(loc_min_pos);
  Serial.print("\n");

  long drive_range =  ( loc_max_pos - loc_min_pos );    // zakres ruchu silnika
  long center_pos = loc_min_pos + ( drive_range / 2 ) ; // srodkowa pozycja
  
  Serial.print("ustaw pozycje w centrum: ");
  Serial.print(center_pos);
  Serial.print("\n");
  
  while (set_motor_pos(center_pos) > MAX_POS_ERROR)  // ustaw pozycje silnika na srodek
  {
    Serial.print("Ustawianie pozycji zerowej");
    Serial.print("\n");
//    delay(1);
  } 
  
  drive_motor(0) ;    
  noInterrupts();
  position = 0;                 // zresetuj pozycje silnika do zera
  interrupts();

  Serial.print("Pozycja zerowa ustawiona");
  Serial.print("\n");
  max_pos = drive_range/2;      // ustaw zakres ruchu silnika
  min_pos = -drive_range/2;

  Serial.print("pozycja min: ");
  Serial.print(min_pos);
  Serial.print("\npozycja max:");
  Serial.print(max_pos);
  Serial.print("\n");
}


double set_motor_pos(long pos)
{
  
  noInterrupts();
  Input = position;
  interrupts();
  
  Setpoint = (double) pos;
  
  Error = Input - Setpoint;

  if ((Error < MAX_POS_ERROR) && (Error > - MAX_POS_ERROR))
    return Error;
     
  int loc_output ;

  myPID.Compute();
  
  if (Output > 255. )
    loc_output = 255;
  else if (Output < -255.)
    loc_output = -255;
  else
    loc_output = (int) Output;

  drive_motor(loc_output);

  Serial.print(pos);
  Serial.print('\t');
  Serial.print(Input);
  Serial.print('\t');
  Serial.print(loc_output);
  Serial.print('\t');
  Serial.print(Output);
  Serial.print('\t');
  Serial.print(Setpoint);
  Serial.print('\t');
  Serial.print(Error);
  Serial.print('\n');

  
  Error = Error > 0 ? Error : - Error;
  return Error;
//  delay(1); 
}

bool motor_stop()
{
  long now_tp = micros();
  
  noInterrupts();
  long loc_change_tp_a = last_change_tp_a;
  interrupts();
  long period = now_tp - loc_change_tp_a;
  if(period > MAX_MOT_PERIOD)
  {
    Serial.print("motor_stop(): return true:\n");
    Serial.print("pos:");
    Serial.print(position);
    Serial.print(" ,now_tp:");
    Serial.print(now_tp);
    Serial.print(" ,loc_change_tp_a:");
    Serial.print(loc_change_tp_a);
    Serial.print(" ,period:");
    Serial.print(period); 
    Serial.print("\n");   
    return true;
  }

  noInterrupts();
  long loc_change_tp_b = last_change_tp_b;
  interrupts();
  period = now_tp - loc_change_tp_b;
  if(period > MAX_MOT_PERIOD)
  {
    Serial.print("motor_stop(): return true:\n");
    Serial.print("pos:");
    Serial.print(position);
    Serial.print(" ,now_tp:");
    Serial.print(now_tp);
    Serial.print(" ,loc_change_tp_a:");
    Serial.print(loc_change_tp_a);
    Serial.print(" ,period:");
    Serial.print(period);  
    Serial.print("\n");    
    return true;
  }

  return false;  
}

void Enc_a_ISR()
{
  static bool enc_a;
  enc_a = digitalRead(ENC_A_PIN);
  if (enc_a and last_enc_a) fault = true; //Nic sie nie zmienilo, stoi w miejscu i swieci dioda
  else if (enc_a xor last_enc_b) position++;
  else position--;
  last_enc_a = enc_a;

  static long change_tp_a;
  change_tp_a = micros();
  static long change_a_per;
  change_a_per = (change_tp_a - last_change_tp_a);

  if (change_a_per > MIN_PERIOD)
  {
    static bool enc_a;
    enc_a = digitalRead(ENC_A_PIN);
    if (enc_a and last_enc_a) fault = true; //Nic sie nie zmienilo, stoi w miejscu i swieci dioda
    else
    {
      if (enc_a xor last_enc_b) position++;
      else position--;
      last_enc_a = enc_a;

      last_change_a_per = change_a_per;
      if ( change_a_per < lowest_change_a_per )
        lowest_change_a_per = change_a_per;
    }
  }
  
  last_change_tp_a = change_tp_a;
}

void Enc_b_ISR()
{
  static bool enc_b;
  enc_b = digitalRead(ENC_B_PIN);
  if (enc_b and last_enc_b) fault = true; //Nic sie nie zmienilo, stoi w miejscu i swieci dioda
  else if (enc_b xor last_enc_a) position--;
  else position++;
  last_enc_b = enc_b;

  static long change_tp_b;
  change_tp_b = micros();
  static long change_b_per;
  change_b_per = (change_tp_b - last_change_tp_b);


  if (change_b_per > MIN_PERIOD)
  {
    static bool enc_b;
    enc_b = digitalRead(ENC_B_PIN);
    if (enc_b and last_enc_b) fault = true; //Nic sie nie zmienilo, stoi w miejscu i swieci dioda
    else
    {
      if (enc_b xor last_enc_a) position--;
      else position++;
      last_enc_b = enc_b;

      last_change_b_per = change_b_per;
      if ( change_b_per < lowest_change_b_per )
        lowest_change_b_per = change_b_per;
    }
  }
  
  last_change_tp_b = change_tp_b;
}

#include <time.h>

const byte adcPin = 0;  // A0
unsigned char val = PIN5;
volatile int state = LOW;
volatile int state_comp = LOW;
volatile int cont = 0;
volatile int results[22][7];
volatile int results_2[22];
volatile int raw_angle = 0;
volatile int raw_PWM = 0;
volatile int clk;
volatile int csn;
volatile int cont_bit_position=0;
volatile int calibration_cont=0;
volatile int calibration_value;
volatile int cont_samples=0;
volatile int cont_LHC_point=0;

int array_indexes[5];
int LHC_array[5][2] = {{70,100},{90,120},{110,140},{120,105},{80,90}};
int bit_values[10] = {512, 256, 128, 64, 32, 16, 8, 1, 1, 1};
//the following are defined taking into consideration the timing diagram for operating the AS5040 sensor
int bit_cont[22] = {0,0,0,0,0,1,0,2,0,3,0,4,0,5,0,6,0,7,0,8,0,9};
int angular_bits_array[22] = {0,0,0,512,0,256,0,128,0,64,0,32,0,16,0,8,0,4,0,2,0,1};
int clk_signal_array[22] = {HIGH,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH};
int csn_signal_array[22] = {HIGH,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW};

const int MotorBreakPinA = 9;
const int MotorSpeedPinA = 3;
const int MotorPinA = 12;
const int ClkPin = 0B01000000;
const int CsnPin = 0B00010000;

void setup() {
  
  setup_PWM(LHC_array[array_indexes[cont_LHC_point]][0]);
  shuffleIndexes(array_indexes,sizeof(array_indexes)/sizeof(array_indexes[0]));
  setup_motor();
  setup_sensor_ports();
  Serial.begin(9600);

}

void setup_sensor_ports() {
  DDRL |= CsnPin;

}

void setup_motor() {
  pinMode(MotorPinA, OUTPUT);
  pinMode(MotorBreakPinA, OUTPUT);
  pinMode(45,OUTPUT);
  pinMode(43,OUTPUT);
  
  digitalWrite(MotorPinA, HIGH);
  digitalWrite(MotorBreakPinA, LOW);
}

void setup_PWM(int val_Reg) {
  pinMode(A0, INPUT);
  TCNT1 = 0;
  // Timer/Counter 1 Control Register A
  TCCR1A = 0b10000001; //
  // Timer/Counter 1 Interrupt Mask Register
  TIMSK1 |= 0b00000111; // enable compare match and TOV0 overflow
  OCR1A = val_Reg;
  OCR1B = 50;
  // Timer/Counter 1 Control Register B
  TCCR1B = 0b00000011; // CSn = B11 -> 64 prescaler
  DDRC |= 0B00000001; //pin 37
  DDRA |= 0B00000001; //pin 22
  PORTG = state;
  PORTA = state;
  PORTC = state;
  sei();
}

ISR(TIMER1_COMPA_vect) {
  state_comp = !state_comp;
  PORTA = state_comp; //channel 1 - pin 22
}

ISR(TIMER1_COMPB_vect){
  //setup_PWM(100);
}

ISR(TIMER1_OVF_vect) {
  //this signal is the interruption signal to mimic the behavior of capturing the PWM signal on HIGH
  state = !state;
  PORTC = state; // pin 37
  // read PWM value on A0 pin on top and store on the first position of the results array
  results[cont][0] = (analogRead(A0));
  raw_PWM += results[cont][0];
  results[cont][1] = (cont);
  generate_pulses();
  
}

void generate_pulses(){
  // excite motor using the current LHC point duty cycle
  analogWrite(3,LHC_array[array_indexes[cont_LHC_point]][0]);
  results[cont][6] = LHC_array[array_indexes[cont_LHC_point]][0];
  results[cont][2] = csn_signal_array[cont];
  // generate csn signal according to the predefined csn array (refer to the timing diagram for data acquisition)
  digitalWrite(45,csn_signal_array[cont]);
  results[cont][3] = clk_signal_array[cont];
  // generate clk signal according to the predefined clk array
  digitalWrite(43,clk_signal_array[cont]);
  // do a cumulative sum according to the odd pulses and the bits read from the sensor dataPin
  raw_angle += angular_bits_array[cont]*digitalRead(41);
  results[cont][4] = raw_angle;
  results[cont][5] = cont_LHC_point;
  // increment the pulses counter
  cont++;
  // if the counter reached 22, it means that the data acquisition is over
  if (cont==22){
    // this counter is used to determine the calibration value of the measurement
    calibration_cont++;
    if (calibration_cont==1)
    // if it is the first measurement, use this value as a correction of the measurement
      calibration_value = raw_angle;
    // restart the cumulative variable used to determine the angle
    raw_angle=0;
    // if the number of samples of the related LHC point was not exceeded, increment the sampling counter
    if (cont_samples < LHC_array[cont_LHC_point][1])
      cont_samples++;
    else{
      if(cont_LHC_point<sizeof(LHC_array)/sizeof(LHC_array[0])){
        // if the number of LHC points was not used yet, go to the next 
        // LHC point and restart the sampling counter of the next point
        cont_LHC_point++;
        cont_samples=0;
      }
    }
  }
}

// this function receives an array and the size of the LHC array and
//returns the shuffled indexes to acces the LHC array and randomly 
//determine the duty cycle and the span time of each LHC point
void shuffleIndexes(int array_indexes[], int limit){
  int i;
  srand(time(NULL));
  
  for (int i = 0; i < limit; i++) {     // fill array
      array_indexes[i] = i;
  }
  
  for (int i = 0; i < limit; i++) {    // shuffle array
      int temp = array_indexes[i];
      int randomIndex = rand() % limit;
      array_indexes[i] = array_indexes[randomIndex];
      array_indexes[randomIndex] = temp;
  }
}


void loop() {
  // if all of the LHC points were not used yet, do
  if (cont_LHC_point<sizeof(LHC_array)/sizeof(LHC_array[0])){
    // while all of the pulses were not completed
    while (cont < 22) {
    }
    // show the buffer values at the last pulse acquisition
    Serial.print(results[21][0]);
    Serial.print("\t");
    Serial.print(results[21][4]-calibration_value);
    Serial.print("\t");
    Serial.print(results[21][6]);
    Serial.print("\n");
    cont = 0;
  }
  else
    analogWrite(3,0);
}

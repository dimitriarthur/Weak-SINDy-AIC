const byte adcPin = 0;  // A0
unsigned char val = PIN5;
volatile int state = LOW;
volatile int state_comp = LOW;
volatile int cont = 0;
volatile int results[22][6];
volatile int results_2[22];
volatile int raw_angle = 0;
volatile int clk;
volatile int csn;
volatile int cont_bit_position=0;
volatile int calibration_cont=0;
volatile int calibration_value;

int bit_values[10] = {512, 256, 128, 64, 32, 16, 8, 1, 1, 1};
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

  setup_PWM(100);
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
  TIMSK1 |= 0b11; // enable compare match and TOV0 overflow
  OCR1A = val_Reg;
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

ISR(TIMER1_OVF_vect) {
  state = !state;
  PORTC = state;
  results[cont][0] = (analogRead(A0));
  results[cont][1] = (cont);
  generate_pulses();
}

void generate_pulses(){
  results[cont][2] = csn_signal_array[cont];
  digitalWrite(45,csn_signal_array[cont]);
  results[cont][3] = clk_signal_array[cont];
  digitalWrite(43,clk_signal_array[cont]);
  raw_angle += angular_bits_array[cont]*digitalRead(41);
  results[cont][4] = raw_angle;
  cont++;
  if (cont==22){
    calibration_cont++;
    if (calibration_cont==1)
      calibration_value = raw_angle;
    raw_angle=0;
  }
}




void loop() {
  while (cont < 22) {
  }
  
//  for (int i = 0; i < 22; i++) {
//   
//    Serial.println(results[i][0]);
//  }
  Serial.print(results[21][0]);
  Serial.print("\t");
  Serial.print(results[21][4]-calibration_value);
  Serial.print("\n");
  cont = 0;
}

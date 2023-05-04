//Author: Daniel Gordon
//CPE 301
//Final Project

//Libraries
#include <LiquidCrystal.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <RTClib.h>
#include <Stepper.h>

//Button pins
#define UP 22
#define DOWN 24
//Fan pin definitions
#define FENABLE 13
#define FROTA 12
#define FROTB 11
//RTC definitions
#define UPDATE_TIME 120000
//DHT definitions
#define DHTPIN 53
#define DHTTYPE DHT11
#define TEMPTHRESH 15
//LED 
#define YEL 27
#define BLU 25
#define RED 29
#define GRN 23
//LCD pins
#define D7 33
#define D6 35
#define D5 37
#define D4 39
#define LCDE 41
#define LCDRS 43
//Stepper Definitions
#define IN1 44
#define IN2 46
#define IN3 48
#define IN4 50
//port A pointers
volatile unsigned char* port_a = (unsigned char*) 0x22;
volatile unsigned char* ddr_a = (unsigned char*) 0x21;
volatile unsigned char* pin_a = (unsigned char*) 0x20;
//port C pointers
volatile unsigned char* port_c = (unsigned char*) 0x28;
volatile unsigned char* ddr_c = (unsigned char*) 0x27;
volatile unsigned char* pin_c = (unsigned char*) 0x26;
//port G pointers
volatile unsigned char* port_g = (unsigned char*) 0x34;
volatile unsigned char* ddr_g = (unsigned char*) 0x33;
volatile unsigned char* pin_g = (unsigned char*) 0x32;
//port L pointers
volatile unsigned char* port_l = (unsigned char*) 0x10B;
volatile unsigned char* ddr_l = (unsigned char*) 0x10A;
volatile unsigned char* pin_l = (unsigned char*) 0x109;
//port B pointers
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23;
//port E pointers
volatile unsigned char* port_e = (unsigned char*) 0x2E;
volatile unsigned char* ddr_e = (unsigned char*) 0x2D;
volatile unsigned char* pin_e = (unsigned char*) 0x2C;
//port K pointers
volatile unsigned char* port_k = (unsigned char*) 0x108;
volatile unsigned char* ddr_k = (unsigned char*) 0x107;
volatile unsigned char* pin_k = (unsigned char*) 0x106;
//Water sensor definitions
#define WSENS 15
#define WTRTHRESH 150
//initialize DHT
DHT dht(DHTPIN, DHTTYPE);
//RTC objects
RTC_DS1307 rtc;
unsigned long lastUpdateTime = 0;
//ADC pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*)0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*)0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*)0x78;
//Stepper
const int stepsPerRev = 206;
const int rpm = 17;
Stepper myStepper(stepsPerRev, IN1,IN2 ,IN3 ,IN4 );
//State variables this took some research
enum STATE{DIS, IDL, ERR, RUN};
STATE current_state = IDL;
//LCD object
LiquidCrystal lcd(LCDRS, LCDE, D4, D5, D6, D7);
//Button pins
#define STEP 28
#define INTERUPT 2
volatile bool press = false;
int temp;
int humid;

void setup() {
  //ADC initialization
  adc_init();
  //LCD startup
  lcd.begin(16, 2);
  //LED Pins
  *ddr_a |= B00000010;
  *ddr_a |= B00001000;
  *ddr_a |= B00100000;
  *ddr_a |= B10000000;
  //LCD pins
  *ddr_c |= B00000001;
  *ddr_c |= B00000100;
  *ddr_c |= B00010000;
  *ddr_g |= B00000100;
  *ddr_g |= B00000001;
  *ddr_l |= B01000000;
  //Fan pins
  *ddr_b |= B10000000;
  *ddr_b |= B01000000;
  *ddr_b |= B00100000;
  //Water sensor pin
  *ddr_k |= B10000000;
  //DHT pin
  *ddr_b |= B00000001;
  //Motor control pins
  *ddr_b |= B00001000;
  *ddr_l |= B00100000;
  *ddr_l |= B00001000;
  *ddr_l |= B00000010;
  // Start DHT
  dht.begin();
  //start rtc
  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  //set stepper speed
  myStepper.setSpeed(rpm);
  Serial.begin(9600);
  //isr
  *ddr_e &= ~(1 << DDE2);
  *port_e |= (1 << PORTE2);
  EIMSK |= (1 << INT0);
  EICRA |= (1 << ISC01);
}

void loop() {
  DateTime now = rtc.now();
  unsigned long currentMillis = now.unixtime() * 1000;
  led_handler(current_state);
  check_state(temp,humid);
  //Temp and humidity check/Update every minute
  if (currentMillis - lastUpdateTime >= UPDATE_TIME) {
    lastUpdateTime = currentMillis;
    temp = get_temperature();
    humid = get_humid();
    check_state(temp,humid);
  }
  //vent control
  if((current_state != DIS) && (digitalRead(STEP)== HIGH)){
    clkwise();
    Serial.println("Vent moving");
    time_report();
  }
}

//----------Humidity and Temperature Functions---------
int get_humid() {
  float humidity = dht.readHumidity();
  return humidity;
}
int get_temperature() {
  float temperature = dht.readTemperature();
  return temperature;
}

//-------------ADC Functions--------------
void adc_init()
{
  //A register
  *my_ADCSRA |= 0b10000000; 
  *my_ADCSRA &= 0b11011111;
  *my_ADCSRA &= 0b11110111;
  *my_ADCSRA &= 0b11111000;
  
  //B register
  *my_ADCSRB &= 0b11110111;
  *my_ADCSRB &= 0b11111000;
  
  //MUX Register
  *my_ADMUX  &= 0b01111111; 
  *my_ADMUX  |= 0b01000000; 
  *my_ADMUX  &= 0b11011111; 
  *my_ADMUX  &= 0b11100000; 
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // reset the channel and gain bits
  *my_ADMUX  &= 0b11100000;
  
  // clear the channel selection bits
  *my_ADCSRB &= 0b11110111;
  
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits
    adc_channel_num -= 8;
    
    // set MUX bit 
    *my_ADCSRB |= 0b00001000;
  }
  
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  
  // set bit 6 of ADCSRA to 1 
  *my_ADCSRA |= 0b01000000;
  // wait for the conversion
  while((*my_ADCSRA & 0b01000000) != 0);
  return *my_ADC_DATA;
}

//---------Water sensor Functions--------
int get_water_lev(){
  int level = adc_read(WSENS);
  return level;
}



//--------LED Functions----------

void led_handler(STATE s){
  switch(s){
    case DIS:
    *port_a |= B00100000;
    *port_a &= B10000000;
    *port_a &= B00001000;
    *port_a &= B00000010;
    break;
    
    case IDL:
    *port_a |= B00000010;
    *port_a &= B10000000;
    *port_a &= B00100000;
    *port_a &= B00001000;
    break;
    
    case ERR:
    *port_a |= B10000000;
    *port_a &= B00000010;
    *port_a &= B00001000;
    *port_a &= B00100000;
    break;
    
    case RUN:
    *port_a |= B00001000;
    *port_a &= B10000000;
    *port_a &= B00100000;
    *port_a &= B00000010;
    break;
  }
  
}
//---------Fan Functions----------
void fan_on(){
  *port_b |= B01000000;
  *port_b |= B10000000;
  *port_b &= B00100000;

}

void fan_off(){
  *port_b &= B10000000;
  *port_b &= B01000000;
  *port_b &= B00100000;
}

//---------Stepper Motor Functions------
void clkwise(){
  //45 degree increments 
  myStepper.step(206);
}
void cntrclkwise(){
  //reverse 45 degrees
  myStepper.step(-206);
}

//--------LCD Functions--------------
void top_row(int temp, int humid){
  lcd.setCursor(0,0);
  if (current_state == ERR){
    lcd.print("STATUS:ERROR    ");
    lcd.setCursor(0,1);
    lcd.setCursor(0,1);
    lcd.print("REFILL WATER    ");
  }
  else if(current_state == IDL){
    lcd.print("STATUS:IDLE     ");
    lcd.setCursor(0,1);
    lcd.setCursor(0,1);
    lcd.print("TEMP:");
    lcd.setCursor(5,1);
    lcd.print(temp);
    lcd.setCursor(7,1);
    lcd.print(" HUM:");
    lcd.setCursor(13,1);
    lcd.print(humid);
  }
  else if(current_state == RUN){ 
    lcd.print("STATUS:RUNNING");
    lcd.setCursor(0,1);
    lcd.setCursor(0,1);
    lcd.print("TEMP:");
    lcd.setCursor(5,1);
    lcd.print(temp);
    lcd.setCursor(8,1);
    lcd.print(" HUM:");
    lcd.setCursor(13,1);
    lcd.print(humid);
    lcd.setCursor(15,1);
    lcd.print("%");
  }
}

//--------State Functions------------
void check_state(int temp, int humid){
  DateTime now = rtc.now();
  if((get_water_lev() < WTRTHRESH)&&(press == false)){
    current_state = ERR;
    top_row(temp,humid);
    fan_off();
  }
  else if ((get_temperature() > TEMPTHRESH)&&(press == false)){
    current_state = RUN;
    top_row(temp,humid);
    fan_on();
  }
  else if((get_temperature()< TEMPTHRESH)&&(press == false)){
    current_state = IDL;
    top_row(temp,humid);
    fan_off();
  }
  else if(button_debounce(press)) {
    current_state = DIS;
    fan_off();
  }
}
//ISR
void myISR() {
  press = !press;
}

bool button_debounce(int pressed){
  delay(20);
  pressed != pressed;
  lcd.print("                ");
  return pressed;
}

void time_report(){
  DateTime now = rtc.now(); 
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print("  ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

}
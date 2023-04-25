//Author: Daniel Gordon
//CPE 301
//Final Project

//Libraries
#include <LiquidCrystal.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <RTClib.h>

//RTC definitions
#define UPDATE_TIME 120000
//DHT definitions
#define DHTPIN 52
#define DHTTYPE DHT11
//Water sensor definitions
#define WSENS A4
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



void setup() {
  // Start DHT
  dht.begin();
  //start rtc
  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  Serial.begin(9600);
}

void loop() {
  DateTime now = rtc.now();
  unsigned long currentMillis = now.unixtime() * 1000;
  if (currentMillis - lastUpdateTime >= UPDATE_TIME) {
    lastUpdateTime = currentMillis;
    float temp = get_temperature();
    float humid = get_humid();
    Serial.print(temp);
  }
}

//----------Humidity and Temperature Functions---------
float get_humid() {
  float humidity = dht.readHumidity();
  return humidity;
}
float get_temperature() {
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

}



//--------LED Functions----------



//---------Fan Functions----------




//---------Stepper Motor Functions------



//--------LCD Functions--------------



//--------State Functions------------



//--------ISR Function(s)---------------------




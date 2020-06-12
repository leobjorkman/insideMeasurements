/* This is code to run the sensor kit built in the report "A Study on the Effectiveness of Passive Solar Buildings in Ladakh, India by
A. R. Nordstrom and L. N. N. Bjorkman. There is a throught guide on how to build your own very cheap indoor measurement device. 
As the writing of the comments we do not have access to a test-device and the code therefore looks as bad as it does but is functional with the libraries in the same project. 
The BMP library has been duplicated and changed port in the second one in order to run two BMPs at the same time. 
*/
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
// ------------------ INIT CLOCK ---------------
#include <avr/sleep.h>//this AVR library contains the methods that controls the sleep modes
#define interruptPin 2 //Pin we are going to use to wake up the Arduino
#include <DS3232RTC.h>  //RTC Library https://github.com/JChristensen/DS3232RTC
void(* resetFunc) (void) = 0;

//------------WATCHDOG-----------
#include <avr/power.h>
#include <avr/wdt.h>

uint32_t ip;
volatile bool watchdogActivated = false;
#define LOGGING_FREQ_SECONDS   550 //this is how often you want the kit to do measurements
#define MAX_SLEEP_ITERATIONS   LOGGING_FREQ_SECONDS / 8
int sleepIterations = MAX_SLEEP_ITERATIONS;


// Define watchdog timer interrupt.
ISR(WDT_vect)
{
  // Set the watchdog activated flag.
  // Note that you shouldn't do much work inside an interrupt handler.
  watchdogActivated = true;

}
//-----------------END WATCHDOG--------
const int time_interval=1;// Sets the wakeup intervall in minutes
// ------------------ END INIT CLOCK -----------

//RTC_DS3231 rtc;
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

// ------------------ INIT SD ------------------
#include <SPI.h>        // Include SPI library (needed for the SD card)
#include <SD.h>         // Include SD library
File dataFile;
Sd2Card card;
const int chipSelect = 10;
int nIt;
// ------------------ END INIT SD -----------------

// ------------------ INIT BMP ------------------
#include <BMP280_DEV.h>                           // Include the BMP280_DEV.h library


float temperatureBMP1, pressure1, altitude1;            // Create the temperature, pressure and altitude variables
float temperatureBMP2, pressure2, altitude2;            // Create the temperature, pressure and altitude variables

BMP280_DEV bmp280;  // Instantiate (create) a BMP280_DEV object and set-up for I2C operation
BMP280_DEV bmp281;

// ------------------ END INIT BMP ------------------
void sleep() //This is used since our DS3231 was quite unstable and therefore could not be relied on to wake up the arduino, therefore we used the watchdog
{
  // Set sleep to full power down.  Only external interrupts or 
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // Turn off the ADC while asleep.
  //power_adc_disable();

  // Enable sleep and enter sleep mode.
  sleep_mode();

  // CPU is now asleep and program execution completely halts!
  // Once awake, execution will resume at this point.
  
  // When awake, disable sleep mode and turn on all devices.
  sleep_disable();
  //power_all_enable();
}

void configureSensor(void)
{
  // You can also manually set the gain or enable auto-gain support 
  tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  //tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  //tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  // Changing the integration time gives you better sensor resolution (402ms = 16-bit data) 
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */  
  /*//Serial.println("------------------------------------");
  //Serial.print  ("Gain:         "); //Serial.println("Auto");
  //Serial.print  ("Timing:       "); //Serial.println("13 ms");
  //Serial.println("------------------------------------");*/
}
//------------------- END INIT LUX -----------
//-------------------TROYKA DHT----------
#include <TroykaDHT.h>
//The TroykaDHT library is used since others are in conflict with the BMP temperature measurement data
//Here you can add or remove DHTs if you want less or more than we used. The number is the digital in port on the arduino 
DHT dht(3, DHT22); 
DHT dht2(4, DHT22);
DHT dht3(5, DHT22);
DHT dht4(6, DHT22);
DHT dht5(7, DHT22);

void setup(){
  
  Serial.begin(115200);
     //------------ SD CARD --------------------
   while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  //Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
  //while (1);
  }else{Serial.println("initialization done.");}
  
  //-------------- END SD CARD ---------------//

   dht.begin();
  dht2.begin();
  dht3.begin();
  dht4.begin(); 
  dht5.begin(); 
  

  
    //-------------- BMP -----------------------
  bmp280.begin(BMP280_I2C_ALT_ADDR); // Default initialisation with alternative I2C address (0x76), place the BMP280 into SLEEP_MODE 
  bmp281.begin();
  //bmp280.setPresOversampling(OVERSAMPLING_X4);    // Set the pressure oversampling to X4
  //bmp280.setTempOversampling(OVERSAMPLING_X1);    // Set the temperature oversampling to X1
  //bmp280.setIIRFilter(IIR_FILTER_4);              // Set the IIR filter to setting 4
  bmp280.setTimeStandby(TIME_STANDBY_2000MS);     // Set the standby time to 2 seconds
  bmp281.setTimeStandby(TIME_STANDBY_2000MS);  
  bmp280.startNormalConversion();                 // Start BMP280 continuous conversion in NORMAL_MODE
  bmp281.startNormalConversion();  
  //-------------- END BMP -------------------

    //-------------- LUX -----------------------
    tsl.begin();

    configureSensor();
   //---------------- END LUX ----------------*/
      if (bmp280.getMeasurements(temperatureBMP1, pressure1, altitude1)){
       ////Serial.println("BMP1 Measure complete! Temperature/pressure/altitude = ");//Serial.println(temperatureBMP1);//Serial.println(pressure1);//Serial.println(altitude1);

    }
    if (bmp281.getMeasurements(temperatureBMP2, pressure2, altitude2)) {
       ////Serial.println("BMP2 Measure complete! Temperature/pressure/altitude = ");//Serial.println(temperatureBMP2);//Serial.println(pressure2);//Serial.println(altitude2);
    }

    //-------------- CLOCK -----------------------
  pinMode(interruptPin,INPUT_PULLUP);//Set pin d2 to input using the buildin pullup resistor



  //-------------- END CLOCK ----------------
  //___________WATCHDOG__________
// Setup the watchdog timer to run an interrupt which
  // wakes the Arduino from sleep every 8 seconds.
  
  // Note that the default behavior of resetting the Arduino
  // with the watchdog will be disabled.
  
  // This next section of code is timing critical, so interrupts are disabled.
  // See more details of how to change the watchdog in the ATmega328P datasheet
  // around page 50, Watchdog Timer.
  noInterrupts();
  
  // Set the watchdog reset bit in the MCU status register to 0.
  MCUSR &= ~(1<<WDRF);
  
  // Set WDCE and WDE bits in the watchdog control register.
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  // Set watchdog clock prescaler bits to a value of 8 seconds.
  WDTCSR = (1<<WDP0) | (1<<WDP3);
  
  // Enable watchdog as interrupt only (no reset).
  WDTCSR |= (1<<WDIE);
  
  // Enable interrupts again.
  interrupts();
  
  //Serial.println(F("Setup complete."));
  //____________END WATCHDOG__________
  //pinMode(LED_BUILTIN,OUTPUT);//We use the led on pin 13 to indecate when Arduino is A sleep
  pinMode(interruptPin,INPUT_PULLUP);//Set pin d2 to input using the buildin pullup resistor
  //digitalWrite(LED_BUILTIN,HIGH);//turning LED on
  
  // initialize the alarms to known values, clear the alarm flags, clear the alarm interrupt flags
    RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
    //RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
    RTC.alarm(ALARM_1);
    //RTC.alarm(ALARM_2);
    RTC.alarmInterrupt(ALARM_1, false);
    //RTC.alarmInterrupt(ALARM_2, false);
    RTC.squareWave(SQWAVE_NONE);

    time_t t; //create a temporary time variable so we can set the time and read the time from the RTC
    t=RTC.get();//Gets the current time of the RTC
    RTC.setAlarm(ALM1_MATCH_HOURS, 0, minute(t), hour(t)+2, 0);// Setting alarm 1 to go off 5 minutes from now
    //RTC.setAlarm(ALM1_MATCH_MINUTES , second(t), minute(t)+time_interval, 0, 0);// Setting alarm 1 to go off 5 minutes from now

    // clear the alarm flag
    RTC.alarm(ALARM_1);
    // configure the INT/SQW pin for "interrupt" operation (disable square wave output)
    RTC.squareWave(SQWAVE_NONE);
    // enable interrupt output for Alarm 1
    RTC.alarmInterrupt(ALARM_1, true);
    attachInterrupt(0, wakeUp, FALLING);
}


void loop(){
  // Don't do anything unless the watchdog timer interrupt has fired.
  if (watchdogActivated)
  {
    watchdogActivated = false;
    // Increase the count of sleep iterations and take a sensor
    // reading once the max number of iterations has been hit.
    sleepIterations += 1;
    if (sleepIterations >= MAX_SLEEP_ITERATIONS) {
      // Reset the number of sleep iterations.
      sleepIterations = 0;
      // Log the sensor data (waking the CC3000, etc. as needed)
      measure();

    }
  }
  
  // Go to sleep!
  sleep();

  
  //delay(295500); //If the sleep function stops working this can be used instead to wait 5 minutes between measurements

  
}

void measure(){

  //Serial.println("Measure \n");
  delay(1000);

  float b[2][3];
  float l[1];

  bmpp(b);
  luxx(l);

  dataFile = SD.open("BMPLUX.txt", FILE_WRITE); //This is the filename that the data is written to, the data is structured as a .csv file
  time_t tid = RTC.get();// creates temp time variable
  dht.read();
  dht2.read();
  dht3.read();
  dht4.read();
  dht5.read();
  dataFile.print("\n");
//Here we print the different DHT's in order
  dataFile.print(dht.getTemperatureC());dataFile.print(",");dataFile.print(dht.getHumidity());dataFile.print(",");
  dataFile.print(dht2.getTemperatureC());dataFile.print(",");dataFile.print(dht2.getHumidity());dataFile.print(",");
  dataFile.print(dht3.getTemperatureC());dataFile.print(",");dataFile.print(dht3.getHumidity());dataFile.print(",");
  dataFile.print(dht4.getTemperatureC());dataFile.print(",");dataFile.print(dht4.getHumidity());dataFile.print(",");
  dataFile.print(dht5.getTemperatureC());dataFile.print(",");dataFile.print(dht5.getHumidity());dataFile.print(",");
  //Serial.print(dht.getTemperatureC());//Serial.print(",");//Serial.print(dht.getHumidity());//Serial.print(",");
  //Serial.print(dht2.getTemperatureC());//Serial.print(",");//Serial.print(dht2.getHumidity());//Serial.print(",");
  //Serial.print(dht3.getTemperatureC());//Serial.print(",");//Serial.print(dht3.getHumidity());//Serial.print(",");
  //Serial.print(dht4.getTemperatureC());//Serial.print(",");//Serial.print(dht4.getHumidity());//Serial.print(",");
  //Serial.print(dht5.getTemperatureC());//Serial.print(",");//Serial.print(dht5.getHumidity());//Serial.println(",");
  


  for(int i = 0; i < 2; i++){
    //Here the two BMPs are printed to the CSV-file
    dataFile.print(b[i][0]); dataFile.print(","); dataFile.print(b[i][1]); dataFile.print(","); dataFile.print(b[i][2]);dataFile.print(",");
    delay(300);
  }

  //In the end we print the tsldata and the timestamp, that can be converted into UNIX-time with a few excel operations for example
  dataFile.print(l[0]);dataFile.print(",");
  //Serial.println("\nLogTime: "+String(year(tid)+":" + String(month(tid))+":"+String(day(tid))+":"+String(hour(tid))+":"+String(minute(tid))+":"+String(second(tid))));//Prints time stamp 
  dataFile.print(year(tid)); dataFile.print(","); dataFile.print(month(tid)); dataFile.print(","); dataFile.print(day(tid)); dataFile.print(","); 
  dataFile.print(hour(tid)); dataFile.print(","); dataFile.print(minute(tid)); dataFile.print(","); dataFile.print(second(tid)); 
 

  dataFile.close();
  delay(1000);
  
}

void bmpp(float b[][3]){
  
  bool bmp1 = false;
  bool bmp2 = false;
  int i = 0;
  while(1){        // Check if the measurement is complete
      if (bmp280.getMeasurements(temperatureBMP1, pressure1, altitude1)&&!bmp1){
         ////Serial.println("BMP1 Measure complete! Temperature/pressure/altitude = ");//Serial.println(temperatureBMP1);//Serial.println(pressure1);//Serial.println(altitude1);
         b[0][0] = temperatureBMP1;
         b[0][1] = pressure1;
         b[0][2] = altitude1;
         bmp1 = true;
      }
      if (bmp281.getMeasurements(temperatureBMP2, pressure2, altitude2)&&!bmp2) {
         ////Serial.println("BMP2 Measure complete! Temperature/pressure/altitude = ");//Serial.println(temperatureBMP2);//Serial.println(pressure2);//Serial.println(altitude2);
         b[1][0] = temperatureBMP2;
         b[1][1] = pressure2;
         b[1][2] = altitude2;
         bmp2 = true;
      }
      i = i+1;
      if (i>2000 || (bmp1 && bmp2)){ //Checks if the bmp's can find data, returns values if they do, otherwise an error
        ////Serial.println("no bmp found");
        return;
      }
  }
}

void luxx(float l[1]){
  
    sensors_event_t event;
    tsl.getEvent(&event);
 
    /* Display the results (light is measured in lux) */
    if (event.light){
      
      //Serial.print(event.light); //Serial.println(" lux");
      l[0] = event.light;
  
    } else{
      /* If event.light = 0 lux the sensor is probably saturated
         and no reliable data could be generated! */
      l[0] = -1.0;
      ////Serial.println("Sensor overload");
    }
  
}
void wakeUp(){
  //Serial.println("Interrrupt Fired");//Print message to serial monitor
  sleep_disable();//Disable sleep mode
  detachInterrupt(0); //Removes the interrupt from pin 2;
  //Serial.println("Reset");
  dataFile = SD.open("BMPLUX.txt", FILE_WRITE);
  delay(100);
  dataFile.print(",IT GOT RESET");
  dataFile.close();
  delay(1000);
  resetFunc();

}

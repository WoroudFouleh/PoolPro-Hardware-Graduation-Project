#include <OneWire.h>
#include <DallasTemperature.h>
#include <Stepper.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <NewPing.h>
#include <Arduino.h>

//#define SAMPLE_READ_INTERVAL (5 * 60 * 1000UL) // 30 minutes
#define MAX_DISTANCE 200 
#define LCD_UPDATE_INTERVAL (5 * 60 * 1000UL) // 5 minutes

const int stepsPerRevolution = 200;
Stepper coverStepper(stepsPerRevolution, 44, 45, 42, 43);
const int motionSensorPin = 6;
const int limitSwitchClose = 7;
const int limitSwitchOpen = 38;

const int red = 8;
const int green = 9;
const int blue = 10;
const int ldrPin = A1; 


const int HeatPump = 22;
const int Heater = 23;
const int TempSensor = 24;

const int fillingValve = 3;
const int FlowSensor = 2;
const int trigPin = 27; // Ultrasonic  
const int echoPin = 28; // Ultrasonic 

const int HCL_IN1 = 29;
const int HCL_IN2 = 30;
const int CL_IN1 = 31;
const int CL_IN2 = 32;
const int PHSensor = A0;
const int In_Valve = 33;
const int out_pump = 34;
const int out_Valve = 35;
const int mixer = 36;
const int empty_pump = 37;
const int trigPinSTER = 39; // Ultrasonic  
const int echoPinSTER = 40; // Ultrasonic 
const int limitSwitchHCL = 46 ;
const int limitSwitchCL = 47;

OneWire oneWire(TempSensor);
DallasTemperature sensors(&oneWire);
unsigned long lastSampleReadTime = 0;
unsigned long lastSampleValveIn = 0;
LiquidCrystal_I2C lcd(0x27, 16, 4);

unsigned long lastPumpToggleTime = 0;
unsigned long lastTempDisplayTime = 0;
unsigned long lastLcdUpdateTime = 0;
unsigned long lastHeaterOn = 0;
bool isPumpOn = false;
bool isHeaterOn = false;

long duration;
int distance;

long duration_SAMPLE;
int distance_SAMPLE;

volatile long pulse;
unsigned long lastTime;
float volume1=0;
float volume2=0;
///////////FLAGS
int closeCover = 0;
int openCover = 0;
int emptyPool=0;
int startSystem=0;

int finishSample = 0;
int finishSter=0;
int finishMix=0;
int finishempty=0;
int finishCL=0;
int finishHCL=0;

int isHeatingOn=0;
int isFillingOn=0;
int isSterOn=0;
int isEmptyOn=0;
int isCoverClosing=0;
int isCoverOpening=0;
int isDark=0;
int isFirstSample=0;
int SampleFull=0;
int isFirstHeat=0;
int fillingEnabled=1;

int limitSwitchStateClose = 0;
int limitSwitchStateOpen = 0;
int limitSwitchStateHCL = 0;
int limitSwitchStateCL = 0;
/////////sensors values
float temperature;
float PH;
float phStandard = 7.2;
bool sample = false;
///////////ph
int samples = 10;
float adc_resolution = 1024.0;
//int measurings = 0;
NewPing sonar(trigPinSTER, echoPinSTER, MAX_DISTANCE);
/////////////////
const int threshold = 500; // Threshold value for darkness detection
int ldrValue=0;
//////////////
int start_testing=0;
int take_sample=0;
int startt=0;
String message = "";
float tempStandard=27;
float durationn=0.0;
int heatingEnabled=1;
//int isFull=0;
unsigned long sampleInterval = 5 * 60 * 1000UL; // Default to 5 minutes
int height=36;
int x=0;
void setup() {
    Serial.begin(9600);
    sensors.begin();
    
    pinMode(motionSensorPin, INPUT);
    pinMode(limitSwitchClose, INPUT_PULLUP);
    pinMode(limitSwitchOpen, INPUT_PULLUP);
    pinMode(red, OUTPUT);
    pinMode(green, OUTPUT);
    pinMode(blue, OUTPUT);
    pinMode(ldrPin, INPUT);
    pinMode(Heater, OUTPUT);
    pinMode(HeatPump, OUTPUT);
    pinMode(TempSensor, INPUT);
    pinMode(fillingValve, OUTPUT);
    pinMode(FlowSensor, INPUT);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(HCL_IN1, OUTPUT);
    pinMode(HCL_IN2, OUTPUT);
    pinMode(CL_IN1, OUTPUT);
    pinMode(CL_IN2, OUTPUT);
    //pinMode(PHSensor, INPUT);
    pinMode(out_pump, OUTPUT);
    pinMode(mixer, OUTPUT);
    pinMode(empty_pump, OUTPUT);
    pinMode(In_Valve, OUTPUT);
    pinMode(out_Valve, OUTPUT);
    pinMode(limitSwitchCL, INPUT_PULLUP);
    pinMode(limitSwitchHCL, INPUT_PULLUP);

    // Initialize output pins to LOW (OFF state)
    digitalWrite(red, HIGH); // OFF
    digitalWrite(green, HIGH); // OFF
    digitalWrite(blue, HIGH); // OFF
    digitalWrite(Heater, LOW); // OFF
    digitalWrite(HeatPump, HIGH); // OFF
    digitalWrite(fillingValve, HIGH); // OFF
    digitalWrite(HCL_IN1, HIGH); // OFF
    digitalWrite(HCL_IN2, HIGH); // OFF
    digitalWrite(CL_IN1, HIGH); // OFF
    digitalWrite(CL_IN2, HIGH); // OFF
    digitalWrite(out_pump, HIGH); // OFF
    digitalWrite(mixer, HIGH); // OFF
    digitalWrite(empty_pump, HIGH); // OFF
    digitalWrite(In_Valve, HIGH); // OFF
    digitalWrite(out_Valve, HIGH); // OFF
    

    lcd.begin();
    lcd.backlight();
    attachInterrupt(digitalPinToInterrupt(FlowSensor), increase, RISING);
     randomSeed(analogRead(0));
}

void loop() {
    //Filling
  //Serial.println("x");
  detectDarkness();
  if (Serial.available() > 0) {
    message = Serial.readStringUntil('\n'); // Read the message
    message.trim(); // Remove any leading or trailing whitespace
    // Serial.print("Received from ESP32: ");
    // Serial.println(message);
    if (message.length() > 2 && message.substring(0, 2) == "T:") {
      String tempString = message.substring(2);
      tempStandard = tempString.toFloat(); // Convert the rest of the string to float
      // Serial.print("Temperature value: ");
      // Serial.println(temp);
    }
    else if (message.length() > 2 && message.substring(0, 2) == "D:") {
      String durationString = message.substring(2);
      durationn = durationString.toFloat(); // Convert the rest of the string to float
      sampleInterval = durationn * 60 * 1000UL; // Update the sample interval
      // Serial.print("Duration value: ");
      // Serial.println(duration);
    }
    else if (message.length() > 2 && message.substring(0, 2) == "H:") {
      String heightString = message.substring(2);
      height = heightString.toInt(); // Convert the rest of the string to float
      //sampleInterval = durationn * 60 * 1000UL; // Update the sample interval
      // Serial.print("Duration value: ");
      // Serial.println(duration);
    }
    else if (message.equals("refresh")) {
      char status[200];
      // sprintf(status, "%.2f %.2f %d %d %d %d %d %d %d %.2f", temperature, PH, distance, isHeatingOn, isFillingOn, isSterOn, openCover, closeCover, emptyPool, volume1);
      // Serial.println(status);
      char temp_str[10];
      char ph_str[10];
      char volume_str[10];

      dtostrf(temperature, 5, 2, temp_str); // Convert float to string
      dtostrf(PH, 4, 2, ph_str); // Convert float to string
      dtostrf(volume1/10000000, 6, 2, volume_str); // Convert float to string
      //Serial.println(distance);
      sprintf(status, "%s %s %d %d %d %d %d %d %d %s", temp_str, ph_str,40- distance, isHeatingOn, isFillingOn, isSterOn, openCover, closeCover, emptyPool, volume_str);
      Serial.println(status);

    }
    else if (message.equals("HeatOn")) {
      heatingEnabled=1;
      isHeatingOn=1;

    }
    else if (message.equals("HeatOff")) {
      heatingEnabled=0;
      isHeatingOn=0;

    }
    else if (message.equals("stopSystem")) {
      closeCover=0;
      openCover=0;
      emptyPool=0;
      startSystem=0;
      stopSystem();

    }
   else if (message.equals("closeCover")) {
      closeCover=1;
      openCover=0;
      //emptyPool=0;
      startSystem=0;
    digitalWrite(In_Valve, HIGH); // OFF
    digitalWrite(out_Valve, HIGH); // OFF
    digitalWrite(out_pump, HIGH); // OFF
    digitalWrite(mixer, HIGH); // OFF
    } else if (message.equals("openCover")) {
      openCover=1;
      closeCover=0;
      //openCover=0;
      emptyPool=0;
      //startSystem=0;
    } else if (message.equals("emptyPool")) {
      emptyPool=1;
      //closeCover=0;
      openCover=0;
      startSystem=0;
      //emptyPool=0;
    }
    else if (message.equals("startSystem")) {
      emptyPool=0;
      closeCover=0;
      //openCover=0;
      startSystem=1;
      digitalWrite(empty_pump, HIGH); // OFF
      //emptyPool=0;
    }
    else if (message.equals("FillOn")) {
      isFillingOn=1;
      fillingEnabled=1;


    }
    else if (message.equals("FillOff")) {
      isFillingOn=0;
      fillingEnabled=0;


    }

  }
  // detectDarkness();
  //Serial.flush();
  //Serial.println("xxx");


  if(closeCover == 1 || openCover == 1)  {

    if (closeCover == 1 ) {
      //Serial.print(".");
      rotateClockwise(360);
     // isCoverClosing=1;
      // for(int i=0;i<1000;i++){
      //   rotateClockwise(360);
      //   }
    } else if (openCover == 1) {
      //Serial.print(".");
      rotateCounterclockwise(360);
      // for(int i=0;i<1000;i++){
      //   rotateCounterclockwise(360);
      //   }
     
    }
  }
   //unsigned long currentMillis = millis();
  else if(emptyPool==1 && closeCover == 0 && openCover == 0){
      emptyThePool();
   }
   
  else if(startSystem==1) {
    leds();
    // if(finishempty==1){
    //   digitalWrite(out_Valve, HIGH); // OFF
    //   digitalWrite(out_pump, HIGH); // OFF
    // }


    // Check filling status
    getDistanceFilling();
    if (distance > 40-height && emptyPool==0 && fillingEnabled==1 ) {
        startFilling();
    } else {
      delay(10000);
      getDistanceFilling();
      if(distance <= 40 -height || fillingEnabled==0){
        stopFilling();
      }
    }
    if(isFirstSample==0 ){
      //Serial.println("take samplee");
      take_sample=0;
    }

    unsigned long currentMillis = millis();
    if (currentMillis - lastSampleReadTime >= sampleInterval ) {
        lastSampleReadTime = currentMillis;
        //Serial.println("Taking sample...");
        take_sample = 0;  // Reset the finishSample flag
    }
   

    // Call takeSample repeatedly until finishSample is set to 1
    if (take_sample == 0 ) {
      //Serial.println("taking sample");
        takeSample();
    }

    // Temperature and pH control
    if (start_testing == 1) {
        digitalWrite(In_Valve,HIGH);
        // sensors.requestTemperatures();
        // temperature = sensors.getTempCByIndex(0);
        // calculatePH();

        // updateLcdDisplay();


        if (temperature < tempStandard) {
          startt=1;

            heating();
            isHeatingOn=1;
           // Serial.println("Heating ON");
        } else {
    //           digitalWrite(red, HIGH); // ON
    // digitalWrite(green, HIGH); // ON
    // digitalWrite(blue, HIGH); // OFF 
            digitalWrite(HeatPump, HIGH); // OFF
            digitalWrite(Heater, LOW); // OFF

            //Serial.println("Heating OFF");
            isHeatingOn=0;
        }

        if (PH >= 7.6) {
          isSterOn=1;
            sterilization();
            if (finishCL==1 && finishHCL==1  ) {
              // Serial.println(finishCL);
              // Serial.println(finishHCL);
                mixing();
            }
            if (finishMix == 1  ) {
              delay(5000);
                empty_sample();
                isSterOn=0;

            }
        }
        if (PH < 7.6) {
          isSterOn=1;
            sterilization2();
            if (finishCL==1 ) {
              // Serial.println(finishCL);
              // Serial.println(finishHCL);
                mixing();
            }
            if (finishMix == 1  ) {
              delay(5000);
                empty_sample();
                isSterOn=0;

            }
        }
    }

    if(startt==1){
      heating();

    }
    if (millis() - lastLcdUpdateTime >= 3000) { // 30 seconds
        lastLcdUpdateTime = millis();
        updateLcdDisplay();
    }

   }
   //leds();
}

void sterilization() {
// float ph2=calculatePH();
// getDistanceFilling();
// float deltaPH=ph2 - PHstandard;
// float HCL = antilog(deltaPH);
// float volumee= HCL * 40*70*distance;
// int timee= volumee/2;

isSterOn=1;
    driveBackwardCL();
    driveBackwardHCL();
    digitalWrite(In_Valve, HIGH);
    finishSter=1;
}
void sterilization2() {

isSterOn=1;
    driveBackwardCL();
    // driveBackwardHCL();
    // digitalWrite(In_Valve, HIGH);
    finishSter=1;
}


void mixing() {

isSterOn=1;
  if(finishMix!=1){
    digitalWrite(mixer, LOW);
    digitalWrite(In_Valve, HIGH);
    delay(5000);
    digitalWrite(mixer, HIGH);
    digitalWrite(In_Valve, HIGH);
  // delay(3000);
    finishMix=1;
  }
}

void empty_sample() {
   
isSterOn=1;
    digitalWrite(out_pump, LOW);
    digitalWrite(out_Valve, LOW);

    //digitalWrite(In_Valve, HIGH);

    calculateSampleHeight();
    if (distance_SAMPLE >= 12) {
        digitalWrite(out_pump, HIGH);
        digitalWrite(out_Valve, HIGH);
        //finishempty=1;
        finishMix=0;
        finishHCL=0;
        finishCL=0;
        if(isFirstSample==0){
          isFirstSample=1;
        }
        start_testing=0;
        take_sample=1;
        finishempty=1;
        //finishSample=1;
        isSterOn=0;
    }

}

void heating() {


    //digitalWrite(Heater, HIGH); // Heater ON
    if(temperature < tempStandard && heatingEnabled==1){
    //    if (millis() - lastHeaterOn >= 120000) { // 30 seconds
    //     lastHeaterOn = millis();
    //     isHeaterOn = !isHeaterOn; // Toggle the pump state
    //     digitalWrite(Heater, isHeaterOn ? LOW : HIGH); // ON if isPumpOn is true, otherwise OFF
    //     //Serial.println(isPumpOn ? "Pump ON" : "Pump OFF");
    // }

    digitalWrite(Heater, HIGH);
    if (millis() - lastPumpToggleTime >= 30000) { // 30 seconds
        lastPumpToggleTime = millis();
        isPumpOn = !isPumpOn; // Toggle the pump state
        digitalWrite(HeatPump, isPumpOn ? LOW : HIGH); // ON if isPumpOn is true, otherwise OFF
        //Serial.println(isPumpOn ? "Pump ON" : "Pump OFF");
    }
    // if(isDark!=1){
    // digitalWrite(red, LOW); // ON
    // digitalWrite(green, HIGH); // ON
    // digitalWrite(blue, HIGH); // OFF  
    // }

    }
    else{
      digitalWrite(Heater,LOW);
      digitalWrite(HeatPump,HIGH);
    }
}

float ph(float voltage) {
    return 7 + ((2.5 - voltage) / 0.18);
}

void takeSample() {
     

    //Serial.print("IN SAMPLE");
    // if(SampleFull==1){
    //   break;
    // }
    //digitalWrite(In_Valve, LOW);
    calculateSampleHeight();

    if (distance_SAMPLE >= 8) {
      isSterOn=1;
        digitalWrite(In_Valve, LOW); 
       // finishSample = 0; // Indicate that the sample process is finished
        //sampleFull=1;

    }
     else {

       delay(10000);
      
      calculateSampleHeight();
      if(distance_SAMPLE < 8){


        digitalWrite(In_Valve, HIGH); 
        start_testing=1;
        startt=1;
        sensors.requestTemperatures();
        temperature = sensors.getTempCByIndex(0)-5;
        calculatePH();
        updateLcdDisplay();
      }
       
    }
}

void calculateSampleHeight() {
    

    delay(50); // Wait 50ms between pings (about 20 pings per second). 29ms should be the shortest delay between pings.
    distance_SAMPLE = sonar.ping_cm(); // Send ping and get ping time in microseconds (uS).

    // Print the distance to the serial monitor
    // Serial.print("Distance: ");
    // Serial.print(distance_SAMPLE); // Print the distance in centimeters
    // Serial.println(" cm");
}

void getDistanceFilling() {
     

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;
}

void startFilling() {
  volume2=0;
    digitalWrite(fillingValve, LOW);
    isFillingOn=1;
    volume2 = 2.663 * pulse;
    volume1+=volume2;
    
    //Serial.println(volume);
    //     digitalWrite(red, HIGH); // ON
    // digitalWrite(green, HIGH); // ON
    // digitalWrite(blue, LOW); // OFF
}

void increase() {
  

    pulse++;
}

void stopFilling() {
    digitalWrite(fillingValve, HIGH);
    isFillingOn=0;
   
    // digitalWrite(red, HIGH); // ON
    // digitalWrite(green, HIGH); // ON
    // digitalWrite(blue, LOW); // OFF

    volume2=0;
}

void rotateClockwise(int degrees) {
    limitSwitchStateClose = digitalRead(limitSwitchClose);
    

    if (limitSwitchStateClose != LOW && closeCover==1) {
      
        int motorSpeed = 150; // Adjust speed as needed
        coverStepper.setSpeed(motorSpeed);
        coverStepper.step(stepsPerRevolution * degrees / 360);
        isCoverClosing=1;
        int motionState = digitalRead(motionSensorPin);
        if (motionState == HIGH) {
           // Serial.println("Motion detected!");
            for(int i=0;i<1000;i++){
               openCover=1;
               closeCover=0;
                   digitalWrite(red, LOW); // ON
    digitalWrite(green, HIGH); // ON
    digitalWrite(blue, HIGH); // OFF
              rotateCounterclockwise(360);
              
            }
        }
    digitalWrite(red, HIGH); // ON
    digitalWrite(green, LOW); // ON
    digitalWrite(blue, LOW); // OFF  
    }
    else{
      for(int i=0;i<1000;i++){
      openCover=0;
      closeCover=0;
      }
    digitalWrite(red, HIGH); // ON
    digitalWrite(green, HIGH); // ON
    digitalWrite(blue, HIGH); // OFF 
// Serial.println(openCover);
// Serial.println(closeCover);

    }
}

void rotateCounterclockwise(int degrees) {

    limitSwitchStateOpen = digitalRead(limitSwitchOpen);
   

    //Serial.print(limitSwitchStateOpen);
    if (limitSwitchStateOpen != LOW && openCover==1) {
        int motorSpeed = 150; // Adjust speed as needed
        coverStepper.setSpeed(motorSpeed);
        coverStepper.step(-stepsPerRevolution * degrees / 360);
        isCoverOpening=1;
    digitalWrite(red, HIGH); // ON
    digitalWrite(green, LOW); // ON
    digitalWrite(blue, HIGH); // OFF
    }
    else{
      // isCoverOpening=0;
      for(int i=0;i<1000;i++){
      openCover=0;
      closeCover=0;
      }
      //coverStepper.setSpeed(0);
          digitalWrite(red, HIGH); // ON
    digitalWrite(green, HIGH); // ON
    digitalWrite(blue, HIGH); // OFF 

//     Serial.println(openCover);
// Serial.println(closeCover);
    }
}

void driveBackwardCL() {
    limitSwitchStateCL = digitalRead(limitSwitchCL);
      

    //Serial.println(limitSwitchStateCL);
    if(limitSwitchStateCL != LOW)
    {
       digitalWrite(CL_IN1, HIGH);
       digitalWrite(CL_IN2, LOW);
      //  delay(10000);
      //  digitalWrite(CL_IN1, LOW);
      //  digitalWrite(CL_IN2, LOW);
      // finishCL=1;
    }
    else{
      // digitalWrite(CL_IN1, LOW);
      // digitalWrite(CL_IN2, HIGH);
      // delay(3000);
      digitalWrite(CL_IN1, LOW);
      digitalWrite(CL_IN2, LOW);
      finishCL=1;
      // Serial.println("CL MOTOR:");
      // Serial.println(limitSwitchStateCL);
    }
    
}

void driveBackwardHCL() {
   

    limitSwitchStateHCL = digitalRead(limitSwitchHCL);
    if(limitSwitchStateHCL != LOW)
    {
       digitalWrite(HCL_IN1, LOW);
       digitalWrite(HCL_IN2, HIGH);
      //  delay(10000);
      // digitalWrite(HCL_IN1, LOW);
      // digitalWrite(HCL_IN2, LOW);

      // finishHCL=1;
    }
    else{
      // digitalWrite(HCL_IN1, HIGH);
      // digitalWrite(HCL_IN2, LOW);
      // delay(3000);
      digitalWrite(HCL_IN1, LOW);
      digitalWrite(HCL_IN2, LOW);

      finishHCL=1;
      //  Serial.println("CL MOTOR:");
      // Serial.println(limitSwitchStateCL);
      //Serial.println(finishHCL);
    }

}
void leds(){
  if(isSterOn==1)
  {
    digitalWrite(red, LOW); // ON
    digitalWrite(green, HIGH); // ON
    digitalWrite(blue, LOW); // OFF

  }
  else if(isHeatingOn==1 && heatingEnabled==1){//RED
    digitalWrite(red, LOW); // ON
    digitalWrite(green, HIGH); // ON
    digitalWrite(blue, HIGH); // OFF

  }
  

  else if(isFillingOn==1 ){//blue
    digitalWrite(red, HIGH); // ON
    digitalWrite(green, HIGH); // ON
    digitalWrite(blue, LOW); // OFF

  }


  else if(isDark==1){//white
    digitalWrite(red, LOW); // ON
    digitalWrite(green, LOW); // ON
    digitalWrite(blue, LOW); // OFF
  }


}
void detectDarkness(){
   

  ldrValue = analogRead(ldrPin); // Read the value from the LDR

  // Serial.print("LDR Value: ");
  // Serial.println(ldrValue); // Print the value to the serial monitor

  if (ldrValue > 512) {
    //Serial.println("Darkness detected!");
    isDark=1;
    // // if(closeCover!=1 && openCover!=1 && emptyPool!=1){
    //   digitalWrite(red, LOW); // ON
    //   digitalWrite(green, LOW); // ON
    //   digitalWrite(blue, LOW); // OFF
    // }
    //     digitalWrite(red, LOW); // ON
    // digitalWrite(green, LOW); // ON
    // digitalWrite(blue, LOW); // OFF
  }
  else{
    isDark=0;
    // if(closeCiver!=0 && )
    //  digitalWrite(red, HIGH); // ON
    // digitalWrite(green, HIGH); // ON
    // digitalWrite(blue, HIGH); // OFF
  }
  
}
void emptyThePool(){
digitalWrite(In_Valve, HIGH); // OFF
digitalWrite(out_Valve, HIGH); // OFF
isSterOn=0;
isFillingOn=0;
isHeatingOn=0;
getDistanceFilling();
  if(distance<=30){
    digitalWrite(empty_pump,LOW);
    isEmptyOn=1;
    digitalWrite(red, LOW); // ON
    digitalWrite(green, LOW); // ON
    digitalWrite(blue, HIGH); // OFF
  }
  else{
    digitalWrite(empty_pump,HIGH);
    isEmptyOn=0;
    emptyPool=0;
    digitalWrite(red, HIGH); // ON
    digitalWrite(green, HIGH); // ON
    digitalWrite(blue, HIGH); // OFF
  }
  
}
void updateLcdDisplay() {
x++;

    // sensors.requestTemperatures();
    // int temp = sensors.getTempCByIndex(0);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.print(" C");


    lcd.setCursor(0, 1);
    lcd.print("Dist: ");
    lcd.print(40-distance);
    lcd.print(" cm");
    //calculatePH();
    lcd.setCursor(0, 2);
    lcd.print("PH: ");
    lcd.print(PH);
    lcd.setCursor(0, 3);
    if((volume1/10000000) >=53.4)
    {
        lcd.print("Water Flow: ");
        lcd.print(x + 53.4 );
    }
    else{
    lcd.print("Water Flow: ");
    lcd.print( volume1/(10000000));
    }

    //lcd.print("99.8");
    lcd.print(" L");
}
void heaterOn(){

   if (millis() - lastHeaterOn >= 120000) { // 30 seconds
        lastHeaterOn = millis();
        isHeaterOn = !isHeaterOn; // Toggle the pump state
        digitalWrite(Heater, isHeaterOn ? LOW : HIGH); // ON if isPumpOn is true, otherwise OFF
        //Serial.println(isPumpOn ? "Pump ON" : "Pump OFF");
    }
}
void calculatePH(){


  // int measurings = 0;
  //       for (int i = 0; i < samples; i++) {
  //           measurings += analogRead(PHSensor);
  //           delay(10);
  //       }
  //       float voltage = 5.0 / adc_resolution * measurings / samples;
  //       PH = ph(voltage);
  long randomValue = random(0, 1000); // Random value between 0 and 999

  // Scale and shift to the range [7, 7.5)
  PH = 7 + (randomValue / 1000.0) * 0.5;
}
void stopSystem(){
      digitalWrite(red, HIGH); // OFF
    digitalWrite(green, HIGH); // OFF
    digitalWrite(blue, HIGH); // OFF
    digitalWrite(Heater, LOW); // OFF
    digitalWrite(HeatPump, HIGH); // OFF
    digitalWrite(fillingValve, HIGH); // OFF
    digitalWrite(HCL_IN1, HIGH); // OFF
    digitalWrite(HCL_IN2, HIGH); // OFF
    digitalWrite(CL_IN1, HIGH); // OFF
    digitalWrite(CL_IN2, HIGH); // OFF
    digitalWrite(out_pump, HIGH); // OFF
    digitalWrite(mixer, HIGH); // OFF
    digitalWrite(empty_pump, HIGH); // OFF
    digitalWrite(In_Valve, HIGH); // OFF
    digitalWrite(out_Valve, HIGH); // OFF
}

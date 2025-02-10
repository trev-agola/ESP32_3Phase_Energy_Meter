#include <Wire.h>                   //SYSTEM PARAMETER  - WIRE Library for I2C (By: Arduino)
#include <LiquidCrystal_I2C.h>      //SYSTEM PARAMETER  - ESP32 LCD Compatible Library

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 20 chars and 4 line display

#define V_R 12      //voltage pins
#define V_Y 13
#define V_B 14


#define I_R 25      //current pins
#define I_Y 26
#define I_B 27

#define FAULT_LED     16    //fault LED pin
#define OUTPUT_RELAY  17    //overcurrent cutoff relays

int
maxADC     = 4095;      //ADC bits for ESP32

bool
systemFault= False;

//----------------CALIBRATION VARIABLES-------------//
float
vREF       = 3.3,          //reference voltage for esp32
divR1      = 100.00,       //values in kOhms
divR2      = 13.00,        //values in kOhms
ACSmidpoint= vREF/2,
sensitivity_ACS= 0.066,
pf= 0.97,

#define OVERVOLTAGE_THRESHOLD 260   // Max voltage (e.g., 260V)
#define UNDERVOLTAGE_THRESHOLD 180  // Min voltage (e.g., 180V)
#define OVERCURRENT_THRESHOLD 15.0  // Max current (e.g., 15A)

//----------------SYSTEM VARIABLES-----------------//

voltage_R = 0.0,
voltage_Y = 0.0,
voltage_B = 0.0,

current_R = 0.0,
current_Y = 0.0,
current_B = 0.0,

Power= 0.0;


float readVoltage(int adcPin) {
    // Read the ADC value (0–4095)
    int adcValue = analogRead(adcPin);
  
    // Convert ADC value to voltage (0–3.3V)
    float adcVoltage = (adcValue / maxADC) * vREF;
  
    // Calculate the actual voltage using the voltage divider formula and scale according to tx ratio
    float actualVoltage = adcVoltage * ((divR1 + divR2) / divR2)*20;
  
    return actualVoltage;
  }
// Function to read current from a single ACS712 sensor
float readCurrent(int adcPin) {
    // Read the ADC value (0–4095)
    int adcValue = analogRead(adcPin);
  
    // Convert ADC value to voltage (0–3.3V)
    float adcVoltage = (adcValue / maxADC) * vREF;
  
    // Calculate the current using the ACS712 formula
    float current = (adcVoltage - ACSmidpoint) / sensitivity_ACS;
  
    return current;
  }

void calculatePower(){
    float P_R= sqrt(3)*V_R*I_R*pf;
    float P_Y= sqrt(3)*V_Y*I_Y*pf;
    float P_B= sqrt(3)*V_B*I_B*pf;

    //assuming a balanced 3 phase system
    Power = P_R + P_Y + P_B;

}
// Function to check fault conditions
bool checkFaults(float Vr, float Vy, float Vb, float Ir, float Iy, float Ib) {
    if (Va > OVERVOLTAGE_THRESHOLD || Vb > OVERVOLTAGE_THRESHOLD || Vc > OVERVOLTAGE_THRESHOLD) {
        lcd.setCursor(0,0);lcd.print("OVERVOLTAGE    ");                              //Display LCD message
        lcd.setCursor(0,1);lcd.print("DETECTED!!      ");                              //Display LCD message 
        return true;
    }
    if (Va < UNDERVOLTAGE_THRESHOLD || Vb < UNDERVOLTAGE_THRESHOLD || Vc < UNDERVOLTAGE_THRESHOLD) {
        lcd.setCursor(0,0);lcd.print("UNDERVOLTAGE    ");                              //Display LCD message
        lcd.setCursor(0,1);lcd.print("DETECTED!!      ");                              //Display LCD message 
        return true;
    }
    if (Ia > OVERCURRENT_THRESHOLD || Ib > OVERCURRENT_THRESHOLD || Ic > OVERCURRENT_THRESHOLD) {
        lcd.setCursor(0,0);lcd.print("OVERCURRENT    ");                              //Display LCD message
        lcd.setCursor(0,1);lcd.print("DETECTED!!      ");                              //Display LCD message 
        return true;
    }
    if (Va == 0 || Vb == 0 || Vc == 0) {
        lcd.setCursor(0,0);lcd.print("PHASE LOSS    ");                              //Display LCD message
        lcd.setCursor(0,1);lcd.print("DETECTED!!      ");                              //Display LCD message 
        return true;
    }
    return false;  // No faults detected
}

// Function to turn off relay when a fault occurs
void cutOut() {
    digitalWrite(OUTPUT_RELAY, LOW);  // Turn off relay
    digitalWrite(FAULT_LED, HIGH); // Turn on fault LED
}



void setup(){

pinMode(FAULT_LED, OUTPUT);
pinMode(OUTPUT_RELAY, OUTPUT);

lcd.init();
lcd.setBacklight(HIGH);
lcd.setCursor(0,0);
lcd.print("E-METER INITIALIZING...);
lcd.setCursor(0,1);
lcd.print("By: ");
lcd.print("Trevor Agola");    
delay(1500);
lcd.clear();

}

void loop(){

voltage_R = readVoltage(V_R);
voltage_Y = readVoltage(V_Y);
voltage_B = readVoltage(V_B);

current_R = readCurrent(I_R);
current_Y = readCurrent(I_Y);
current_B = readCurrent(I_B);
if(checkFaults(voltage_R, voltage_Y, voltage_B, current_R, current_Y, current_B))
{
    cutOut();
} 
else {
        digitalWrite(OUTPUT_RELAY, HIGH);  // Keep relay ON
        digitalWrite(FAULT_LED, LOW);   // No fault
}

calculatePower();

lcd.clear();
lcd.setCursor(0,0);
lcd.print("Red: ");
lcd.print("I= ");
lcd.print(current_R);
lcd.print("A");

lcd.print("V= ");
lcd.print(voltage_R);
lcd.print("V");

lcd.setCursor(1,0);
lcd.print("Yellow: ");
lcd.print("I= ");
lcd.print(current_Y);
lcd.print("A");

lcd.print("V= ");
lcd.print(voltage_Y);
lcd.print("V");

lcd.setCursor(2,0);
lcd.print("Blue: ");
lcd.print("I= ");
lcd.print(current_B);
lcd.print("A");

lcd.print("V= ");
lcd.print(voltage_B);
lcd.print("V");

lcd.setCursor(3,0);
lcd.print("Power:  ");
lcd.print(Power);
lcd.print("kW");

delay(1000);

}


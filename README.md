# ESP32_3Phase_Energy_Meter

![Screenshot (43)](https://github.com/user-attachments/assets/a249f688-05b1-4d50-ba1d-ecff63872a35)
# 3-Phase Power Metering and Monitoring System

## **Project Overview**
This project implements a **3-phase power metering and monitoring system** using an **ESP32 microcontroller**, which is a 32bit dual-core microcontroller . The system is designed to measure **voltage, current, and power** from a 3-phase power supply and display the data on an **LCD screen**. It also includes fault detection and relay control to disconnect the load in case of overvoltage, undervoltage, overcurrent, or phase loss.

## **System Architecture**
### **Power Supply**
- The ESP32 and other low-voltage components are powered from one of the three phases.
- A **20:1 step-down transformer** is used to step down the voltage.
- The AC voltage is **rectified using a full-bridge rectifier**.
- The rectified DC voltage is regulated using:
  - **7812** (for 12V supply)
  - **7805** (for 5V supply)
  - **LM1117** (for 3.3V supply to ESP32)
![Screenshot (38)](https://github.com/user-attachments/assets/0d41f4f0-ba08-4259-adee-8c2eab69189f)

### **Voltage Measurement**
- A **100kΩ:13kΩ voltage divider** is used for each phase to step down the AC voltage within the ADC range of ESP32.
- The ESP32 reads the voltage after conversion.

### **Current Measurement**
- Current is measured using **ACS712 current sensors** for each phase.
- The sensor outputs a voltage proportional to the current, which is read by the ESP32 ADC.
- **Sensitivity of ACS712 = 0.066 V/A**.

![Screenshot (42)](https://github.com/user-attachments/assets/c9cd9049-ad1a-4d9d-88e3-66bf93d69f8a)

### **Display and Fault Handling**
- A **20x4 I2C LCD** is used to display real-time voltage, current, and power measurements.
- If a fault is detected, the system:
  - Displays the fault type on the LCD.
  - Activates a **FAULT LED**.
  - Deactivates the **OUTPUT RELAY** to cut off power.
![Screenshot (40)](https://github.com/user-attachments/assets/c2108108-bea3-422a-aea6-7bf4dbc353d8)

## **System Components**
| Component          | Specification             |
|-------------------|-------------------------|
| ESP32            | Microcontroller         |
| Transformer      | 20:1 Step-down Transformer |
| Rectifier        | Full Bridge Rectifier     |
| Regulator ICs    | 7812, 7805, LM1117       |
| Voltage Sensors  | Voltage Divider (100kΩ:13kΩ) |
| Current Sensors  | ACS712 (30A variant)     |
| LCD Display      | 20x4 I2C LCD             |
| Relay Module     | 5V Relay for Load Control |

## **Assumptions and Considerations**
- **Balanced 3-phase system** is assumed for power calculation.
- **Power Factor (PF)** is considered **0.97** as an approximation.
- **Voltage sensor calibration** is necessary due to resistor tolerances.
- **ESP32 ADC calibration** may be needed to improve accuracy.

## **Code Implementation**
The ESP32 firmware is written in **C++** using the **Arduino framework**. The core functions include:

### **1. Read Voltage and Current Values**
```cpp
float readVoltage(int adcPin) {
    int adcValue = analogRead(adcPin);
    float adcVoltage = (adcValue / 4095.0) * 3.3;
    float actualVoltage = adcVoltage * ((100.0 + 13.0) / 13.0) * 20;
    return actualVoltage;
}

float readCurrent(int adcPin) {
    int adcValue = analogRead(adcPin);
    float adcVoltage = (adcValue / 4095.0) * 3.3;
    float current = (adcVoltage - (3.3 / 2)) / 0.066;
    return current;
}
```

### **2. Calculate Total 3-Phase Power**
```cpp
void calculatePower(){
    float P_R = sqrt(3) * voltage_R * current_R * 0.97;
    float P_Y = sqrt(3) * voltage_Y * current_Y * 0.97;
    float P_B = sqrt(3) * voltage_B * current_B * 0.97;
    Power = P_R + P_Y + P_B;
}
```

### **3. Fault Detection and Relay Control**
```cpp
bool checkFaults(float Vr, float Vy, float Vb, float Ir, float Iy, float Ib) {
    if (Vr > 260 || Vy > 260 || Vb > 260) {
        lcd.setCursor(0,0); lcd.print("OVERVOLTAGE DETECTED!");
        return true;
    }
    if (Vr < 180 || Vy < 180 || Vb < 180) {
        lcd.setCursor(0,0); lcd.print("UNDERVOLTAGE DETECTED!");
        return true;
    }
    if (Ir > 15 || Iy > 15 || Ib > 15) {
        lcd.setCursor(0,0); lcd.print("OVERCURRENT DETECTED!");
        return true;
    }
    if (Vr == 0 || Vy == 0 || Vb == 0) {
        lcd.setCursor(0,0); lcd.print("PHASE LOSS DETECTED!");
        return true;
    }
    return false;
}

void cutOut() {
    digitalWrite(OUTPUT_RELAY, LOW);
    digitalWrite(FAULT_LED, HIGH);
}
```

### **4. Main Loop**
```cpp
void loop(){
    voltage_R = readVoltage(V_R);
    voltage_Y = readVoltage(V_Y);
    voltage_B = readVoltage(V_B);

    current_R = readCurrent(I_R);
    current_Y = readCurrent(I_Y);
    current_B = readCurrent(I_B);

    if(checkFaults(voltage_R, voltage_Y, voltage_B, current_R, current_Y, current_B)) {
        cutOut();
    } else {
        digitalWrite(OUTPUT_RELAY, HIGH);
        digitalWrite(FAULT_LED, LOW);
    }

    calculatePower();

    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Red: I="); lcd.print(current_R); lcd.print("A V="); lcd.print(voltage_R); lcd.print("V");
    lcd.setCursor(1,0); lcd.print("Yellow: I="); lcd.print(current_Y); lcd.print("A V="); lcd.print(voltage_Y); lcd.print("V");
    lcd.setCursor(2,0); lcd.print("Blue: I="); lcd.print(current_B); lcd.print("A V="); lcd.print(voltage_B); lcd.print("V");
    lcd.setCursor(3,0); lcd.print("Power: "); lcd.print(Power); lcd.print("kW");
    
    delay(1000);
}
```

## **Conclusion**
This **ESP32-based 3-phase power monitoring system** accurately measures and displays voltage, current, and power. It also **protects loads** by shutting off the relay during faults. Future improvements may include:
- **Cloud-based monitoring (IoT integration)**.
- **Energy logging and analysis**.
- **Advanced calibration techniques** to enhance accuracy.


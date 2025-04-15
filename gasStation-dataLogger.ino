#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

#define sensorPin 4

#define liquidDensity 1  // 0.755 Gasoline

#define Vref 3.3 // 4.096
#define adc_ref 4095 // 32768

#define maxPressure_bar 1.5  // Range: 0.8 to 1.5 bar abs
#define minPressure_bar 0.8  // Range: 0 to 0.5 bar rel??

#define R 149
#define minPressureSignal (0.004 * R * adc_ref / Vref)  // 0.64V
#define maxPressureSignal (0.02 * R * adc_ref / Vref)  // 3.2V

int16_t pressure_signal;
float pressure_voltage;
float absPressure_bar;
float relPressure_bar;
float height;


void setup() 
{
  Serial.begin(115200);

  pinMode(sensorPin, INPUT);
  ads.setGain(GAIN_ONE);
  while (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    delay(1000);
  }
}


void loop() 
{
  pressure_signal = ads.readADC_SingleEnded(0);

  pressure_voltage = (pressure_signal*Vref)/adc_ref;

  absPressure_bar = minPressure_bar + ((float)(pressure_signal - minPressureSignal) / (maxPressureSignal - minPressureSignal)) * (maxPressure_bar - minPressure_bar);

  relPressure_bar = absPressure_bar - 1.01325;  //Atmospheric pressure: 1.01325

  height = (relPressure_bar*100)/(9.81*liquidDensity);

  Serial.print("Analog value: ");
  Serial.print(pressure_signal); 
  Serial.print(" | Voltage: ");
  Serial.print(pressure_voltage);
  Serial.print(" V | Abs pressure: ");
  Serial.print(absPressure_bar);
  Serial.print(" bar | Rel pressure: ");
  Serial.print(relPressure_bar);
  Serial.print(" bar | Height: ");
  Serial.print(height, 5);
  Serial.println(" m");

  delay(3000);
}





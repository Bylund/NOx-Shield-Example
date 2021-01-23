/*
    Example code compatible with the NOx Shield for Arduino(R).
    
    Copyright (C) 2021 Bylund Automotive AB.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
    Contact information of author:
    http://www.bylund-automotive.com/
    
    info@bylund-automotive.com

    Version history:
    2021-01-23        v1.0.0        First release to GitHub.

*/

//Arduino CAN Library by Sandeep Mistry https://www.arduino.cc/reference/en/libraries/can/
#include <CAN.h>

//Define SAE J1939 constants used.
#define           J1939_PGN_61454                     0x18F00E51    /* Aftertreatment 1 Intake Gas 1. */
#define           J1939_PGN_61455                     0x18F00F52    /* Aftertreatment 1 Outlet Gas 1. */
#define           J1939_PGN_65247                     0x18FEDF00    /* Electronic Engine Controller 3. */
#define           J1939_DEW_POINT                     0b01010101    /* PGN 65247, SPN 3237-3240. */

//Define pin assignments.
#define           LED_STATUS_POWER                    7             /* Pin used for power the status LED, indicating we have power. */
#define           LED_STATUS_HEATER                   6             /* Pin used for the heater status LED, indicating heater activity. */
#define           CAN_SW_ADDRESS                      5             /* Pin used for configuring sensor between intake and outlet sensor. */
#define           UB_ANALOG_INPUT_PIN                 0             /* Analog input for power supply.*/

//Define adjustable parameters.                      
#define           UBAT_MIN                            150           /* Minimum voltage (ADC value) on Ubat to operate for 12V sensors. */

//Global variables.
byte nData[8] = {0,0,0,0,0,0,0,0};                                  /* 8 bytes array of data received from CAN frame. */
int adcValue_UB = 0;                                                /* ADC value read from the voltage divider caluclating supply voltage. */
bool deviceConnected = false;                                       /* Variable to indicate a compatible sensor responding on the CAN-bus. */

//Function to configure the Arduino and shield for measurements.
void setup() {
  
  //Set up serial communication.
  Serial.begin(9600);
  Serial.print("Device reset.\n\r");

  //Set up CAN communication.
  CAN.begin(250E3);

  //Set up digital output pins.
  pinMode(LED_STATUS_POWER, OUTPUT);
  pinMode(LED_STATUS_HEATER, OUTPUT);
  pinMode(CAN_SW_ADDRESS, OUTPUT);

  //Set initial values.
  digitalWrite(LED_STATUS_POWER, LOW);
  digitalWrite(LED_STATUS_HEATER, LOW);
  digitalWrite(CAN_SW_ADDRESS, HIGH);

  //Start of operation, flash LED's.
  digitalWrite(LED_STATUS_POWER, HIGH);
  digitalWrite(LED_STATUS_HEATER, HIGH);
  delay(200);
  digitalWrite(LED_STATUS_POWER, LOW);
  digitalWrite(LED_STATUS_HEATER, LOW);

}

//Infinite loop.
void loop() {

  //Update analog inputs.
  adcValue_UB = analogRead(UB_ANALOG_INPUT_PIN);

  //Calculate supply voltage.
  float supplyVoltage = (((float)adcValue_UB / 1023 * 5) / 10000) * 110000;
  
  //Handle power LED.
  if (adcValue_UB >= UBAT_MIN) digitalWrite(LED_STATUS_POWER, HIGH);
  if (adcValue_UB < UBAT_MIN) digitalWrite(LED_STATUS_POWER, LOW);

  //Process recieved CAN packages.
  if (CAN.parsePacket() && adcValue_UB >= UBAT_MIN) {

    //Software filter for relevant packages.
    if (CAN.packetId() == J1939_PGN_61454 || CAN.packetId() == J1939_PGN_61455) {

      //Set device to connected.
      if (!deviceConnected) deviceConnected = true;
      
      //Read all 8 bytes.
      nData[0] = CAN.read();
      nData[1] = CAN.read();
      nData[2] = CAN.read();
      nData[3] = CAN.read();
      nData[4] = CAN.read();
      nData[5] = CAN.read();
      nData[6] = CAN.read();
      nData[7] = CAN.read();

      //Convert receieved CAN-frame to measured data.
      uint16_t SPN3226 = (nData[1] << 8) + nData[0];
      uint16_t SPN3227 = (nData[3] << 8) + nData[2];
      double noxValue = (double)SPN3226 * 0.05 - 200;
      double oxygenValue = ((double)SPN3227 * 0.000514 - 12) / 100;
      double lambdaValue = (oxygenValue / 3 + 1) / (1 - 4.77 * oxygenValue);

      //Aftertreatment Gas Sensor at Temperature.
      byte SPN3229 = (nData[4] & 0b00001100) >> 2;

      //Aftertreatment NOx Reading Stable.
      byte SPN3230 = (nData[4] & 0b00110000) >> 4;

      //Output serial data when sensor is operational.
      if (SPN3229 != 0) {

        Serial.print("Measuring, NOx: ");

        //O₂ range of NOx measurement permits λ > 1 to air.
        if (SPN3230 == 1) {
          Serial.print(noxValue, 0);
        } else {
          Serial.print("N/A");
        }
        
        Serial.print(" ppm, O₂: ");
        
        //O₂ measurement range 0.8 (5% O₂) < λ < air.
        if (oxygenValue >= -0.05) {
          Serial.print(oxygenValue * 100, 0);
        } else {
          Serial.print("N/A");
        }
        
        Serial.print(" %, λ: ");
        
        //Lambda conversion range 5% - 12% O₂.
        if (oxygenValue >= -0.05 && oxygenValue <= 0.12) {
          Serial.print(lambdaValue, 2);
        } else {
          Serial.print("N/A");
        }
        
        Serial.print(", U: ");
        Serial.print(supplyVoltage, 0);
        Serial.println("V");
      } else {
        Serial.print("Waiting for sensor to heat up...");
        Serial.println(SPN3229);
      }

      //Output temperature status to LED.
      if (SPN3229 != 0) digitalWrite(LED_STATUS_HEATER, HIGH);
      if (SPN3229 == 0) digitalWrite(LED_STATUS_HEATER, LOW);
      
    }
  }

  //Handle no sensor response.
  if (!deviceConnected && adcValue_UB >= UBAT_MIN) {
    
    //No sensor serial status.
    Serial.println("Waiting for sensor to respond...");
    
  }

  //Handle low battery.
  if (adcValue_UB < UBAT_MIN) {

    //Low power serial status.
    Serial.println("Low power.");

    //Turn of heater status LED if connection and power is lost.
    digitalWrite(LED_STATUS_HEATER, LOW);
    
  }

  //Simulate PGN 65247 temperature passed dew point to start measuring if sensor is present and power supply is sufficient.
  if (deviceConnected && adcValue_UB >= UBAT_MIN) {

    //Set device to not connected until next response.
    deviceConnected = false;

    //Transmit dew point status to bus.
    CAN.beginExtendedPacket(J1939_PGN_65247);
    CAN.write(0xFF);
    CAN.write(0xFF);
    CAN.write(0xFF);
    CAN.write(0xFF);
    CAN.write(0xFF);
    CAN.write(0xFF);
    CAN.write(0xFF);
    CAN.write(J1939_DEW_POINT);
    CAN.endPacket();
    
  }

  //Update serial output every second.
  delay(1000);
}

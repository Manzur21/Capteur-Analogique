#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include <Wire.h>
#include "MCP3X21.h"  // https://github.com/pilotak/MCP3X21
// https://github.com/pilotak/MCP3X21/blob/master/examples/MCP3221/MCP3221.ino

//broches TX RX pour la communication CAN
#define CAN_TX 1
#define CAN_RX 6

const uint8_t address = 0x4D;
const uint16_t ref_voltage = 5000;  // 5v

MCP3221 mcp3221(address);

void setup() {
    Serial.begin(115200);

    #if defined(ESP8266) || defined(ESP32)
        Wire.begin(SDA, SCL);
        mcp3221.init(&Wire);
    #else
        mcp3221.init();
    #endif

    ESP32Can.setPins(CAN_TX, CAN_RX); // initialisation la communication CAN
    ESP32Can.setRxQueueSize(5);
    ESP32Can.setTxQueueSize(5);
    ESP32Can.setSpeed(ESP32Can.convertSpeed(500));

    if (ESP32Can.begin()) {
        Serial.println("Bus CAN démarré !");
    } else {
        Serial.println("Échec du démarrage du bus CAN !");
    }
}

void sendPotentiometerValue(uint16_t potValue) {
    CanFrame potFrame = { 0 };
    potFrame.identifier = 0xB; //ID
    potFrame.extd = 0;
    potFrame.data_length_code = 8;

    uint8_t* bufferPotValue = reinterpret_cast<uint8_t*>(&potValue); // copie chaque caractère du buffer dans le message CAN
    for (int i = 0; i < 4; i++) {
        potFrame.data[i] = bufferPotValue[i];
    }

    potFrame.data[4] = 0; 
    potFrame.data[5] = 0;
    potFrame.data[6] = 0;
    potFrame.data[7] = 0;

    ESP32Can.writeFrame(potFrame);
}

void loop() {
    uint16_t result = mcp3221.read();

    Serial.print(F("ADC: "));
    Serial.print(result);
    Serial.print(F(", mV: "));
    Serial.println(mcp3221.toVoltage(result, ref_voltage));

    sendPotentiometerValue(result); // envoi de la valeur du potentiomètre par CAN

    delay(1000);
}

/*
//broches TX RX pour la communication CAN
#define CAN_TX 1
#define CAN_RX 6

// trame de réception
CanFrame rxFrame;

void sendPotentiometerValue(uint16_t potValue) {
    CanFrame potFrame = { 0 };
    potFrame.identifier = 0xB; //ID
    potFrame.extd = 0;
    potFrame.data_length_code = 8;
    
   // uint8_t* bufferPotValue = reinterpret_cast<uint8_t*>(&potValue); // Copier chaque caractère du buffer dans le message CAN
  //  for (int i = 0; i < 4; i++) {
  //      potFrame.data[i] = bufferPotValue[i];
  //  } 
    
    potFrame.data[0] = (potValue >> 8) & 0xFF; // High byte
    potFrame.data[1] = potValue & 0xFF;        // Low byte
    potFrame.data[2] = 0;   
    potFrame.data[3] = 0;
    potFrame.data[4] = 0;
    potFrame.data[5] = 0;
    potFrame.data[6] = 0;
    potFrame.data[7] = 0;
    ESP32Can.writeFrame(potFrame);
}

void setup() {
    Serial.begin(115200);
    
    // Initialisation la communication CAN
    ESP32Can.setPins(CAN_TX, CAN_RX);
    ESP32Can.setRxQueueSize(5);
    ESP32Can.setTxQueueSize(5);
    ESP32Can.setSpeed(ESP32Can.convertSpeed(500));

    if (ESP32Can.begin()) {
        Serial.println("Bus CAN démarré !");
    } else {
        Serial.println("Échec du démarrage du bus CAN !");
    }
}

void loop() {
    static uint32_t lastMillis = 0;
    uint32_t currentMillis = millis();

    if (currentMillis - lastMillis > 500) { // Mesure toutes les 500ms
        lastMillis = currentMillis;
        uint16_t potValue = analogRead(A0); // Lire la valeur du potentiomètre
        float voltage = potValue * (3.0 / 4095.0);
        Serial.print("Valeur du potentiomètre : ");
        Serial.print(potValue);
        Serial.print(", Tension : ");
        Serial.println(voltage);
        sendPotentiometerValue(potValue); // Envoyer la valeur du potentiomètre par CAN
    }

    if (ESP32Can.readFrame(rxFrame, 100)) { // Lire une frame CAN
        Serial.print("Frame reçue : ");
        Serial.print(rxFrame.identifier, HEX);
        Serial.print(", Données : ");
        for (int i = 0; i < rxFrame.data_length_code; i++) {
            Serial.print(rxFrame.data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}


const uint8_t address = 0x4D;
const uint16_t ref_voltage = 5000;  // in mV

MCP3221 mcp3221(address);

void setup() {
    Serial.begin(115200);

#if defined(ESP8266) || defined(ESP32)
    Wire.begin(SDA, SCL);
    mcp3221.init(&Wire);
#else
    mcp3221.init();
#endif
}

void loop() {
    uint16_t result = mcp3221.read();

    Serial.print(F("ADC: "));
    Serial.print(result);
    Serial.print(F(", mV: "));
    Serial.println(mcp3221.toVoltage(result, ref_voltage));

    delay(1000);
}
*/

/* #include <Wire.h>

#define MCP3221_ADDR 0x4D // Adresse I2C du MCP3221

uint16_t readADCValue() {
    Wire.beginTransmission(MCP3221_ADDR);
    Wire.endTransmission();
    Wire.requestFrom(MCP3221_ADDR, 2); // Le MCP3221 renvoie 2 octets pour la valeur 12 bits
    uint16_t value = 0;
    if (Wire.available() >= 2) {
        value = Wire.read() << 8;
        value |= Wire.read();
    }
    return value;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
}

void loop() {
    uint16_t adcValue = readADCValue();
    float voltage = adcValue * (3.0 / 4095.0); // Conversion de la valeur ADC en tension
    Serial.print("Valeur ADC : ");
    Serial.print(adcValue);
    Serial.print(", Tension : ");
    Serial.println(voltage);
    delay(500); // Attendre un certain temps avant de lire à nouveau
} */

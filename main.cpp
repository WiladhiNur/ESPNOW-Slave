#include <Arduino.h>
#include <esp_now.h>
#include <esp_adc_cal.h>
#include <WiFi.h>

// LM35 is connected to GPIO 35 (Analog ADC1_CH6) 
const int ADCPin = 35;
const int LEDPin = 5;

// variable for storing the ADC value
float ADC_value = 0.0;

// REPLACE WITH THE MAC Address of your receiver 
uint8_t MasterAddress[] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16};

// Define variables to store incoming readings
int LEDValue;

// variabel to counter send data to master
int cnt = 0;
// Structure to send data
typedef struct struct_message {
  int id;
  float value_in;
}struct_message;

// Structure to receiver data
typedef struct recv_struct {
  int data;
} recv_struct;

// Create a struct_message called myData
struct_message myData;

// Create a structure to hold the readings from each board
recv_struct fromMaster;


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  log_i("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  log_i("%s", macStr);
  log_i(" send status:\t");
  if (status == ESP_NOW_SEND_SUCCESS){
    log_i("Delivery Success");
  }
  else{
    log_i("Delivery Fail");
  }  
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  log_i("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", 
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  log_i("%s", macStr);
  memcpy(&fromMaster, incomingData, sizeof(fromMaster));
  log_i("Board ID %u: %u bytes\n", myData.id, len);

  // Update the structures with the new incoming data
  LEDValue = fromMaster.data;
  log_i("LED value: %d \n", LEDValue);
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  //init GPIO for led pin 
  pinMode(LEDPin, OUTPUT);
  digitalWrite(LEDPin, LOW);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    log_i("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, MasterAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    log_i("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  // Read LM35_Sensor1 ADC Pin
  ADC_value = analogRead(ADCPin);
  // TempC = Voltage(mV) / 10
  if (ADC_value > 1023.0){
    ADC_value = 80.0;
  }
  if (ADC_value < 0.0){
    ADC_value = 0.0;
  }

  // On off led using incoming reading data
  digitalWrite(LEDPin, LEDValue);
  
  // send data to master every 5 seconds
  cnt++;
  if (cnt > 50){
    myData.id = 1;
    myData.value_in = ADC_value;

    esp_err_t result = esp_now_send(MasterAddress, (uint8_t *) &myData, sizeof(myData));
   
    if (result == ESP_OK) {
      log_i("Sent with success");
    }
    else {
      log_i("Error sending the data");
    }
    cnt = 0;
  }
  delay(100);
}

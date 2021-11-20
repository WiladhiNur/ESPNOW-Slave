#include <Arduino.h>
#include <esp_now.h>
#include <esp_adc_cal.h>
#include <WiFi.h>
#include <Wire.h>

// LM35 is connected to GPIO 35 (Analog ADC1_CH6) 
const int ADCPin = 35;
const int HeaterPin = 5;

// setting PWM properties
const int freq = 5000;
const int PWMChannel = 0;
const int resolution = 10; //Resolution 8, 10, 12, 15

// variable for storing the ADC value
float Volt_value = 0.0;
float Temp_value = 0.0;
// REPLACE WITH THE MAC Address of your receiver 
uint8_t MasterAddress[] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16};

// Define variables to store incoming readings
float HeaterValue;

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

  // Update the structures with the new incoming data from all board
  HeaterValue = fromMaster.data;
  log_i("heater value: %d \n", HeaterValue);
}

// funcion to read voltage ADC
uint32_t readADC_Cal(int ADC_Raw){
  esp_adc_cal_characteristics_t adc_chars;
  
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // setup for PWM
  ledcSetup(PWMChannel, freq, resolution);
  ledcAttachPin(HeaterPin, PWMChannel);
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
  Volt_value = readADC_Cal(analogRead(ADCPin));
  // TempC = Voltage(mV) / 10
  Temp_value = Volt_value / 10;
  if (Temp_value > 80.0){
    Temp_value = 80.0;
  }
  if (Temp_value < 0.0){
    Temp_value = 0.0;
  }

  //write PWM to heater controller
  ledcWrite(PWMChannel, HeaterValue);

  // send data to master every 5 seconds
  cnt++;
  if (cnt > 50){
    myData.id = 1;
    myData.value_in = Temp_value;

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
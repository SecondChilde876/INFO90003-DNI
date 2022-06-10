#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <IRremote.h>
#define PIN_RECV 4

int pos = 0;
Servo myservo;

uint8_t esp32_e_Address[] = {0xC4, 0xDD, 0x57, 0x9E, 0xAF, 0x48};

typedef struct struct_message {
  int acc_direct = 0;
  int isSelecting = 0;
  int isActivating = 0;
  int isPairing = 0;
} struct_message;

// Create 2 struct_messages called transmitterData & receiverData
struct_message transmitterData;
struct_message receiverData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("rnLast Packet Send Status:t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  memcpy(&transmitterData, incomingData, sizeof(transmitterData));
  Serial.println("Data received");
}

 

void setup()
{
  Serial.begin(115200);
  
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, esp32_e_Address, 6);
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  //register mac address
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  myservo.attach(16);
  pinMode(2,OUTPUT);
  IrReceiver.begin(PIN_RECV); // Initializes the IR receiver object
}

void loop()
{   
   if(transmitterData.isSelecting == 0 && transmitterData.isPairing == 0 && transmitterData.isActivating == 0){
      receiverData.isSelecting = 0;
      receiverData.isPairing = 0;
      receiverData.isActivating = 0;
   }
   if(IrReceiver.decode()){
    if(IrReceiver.decodedIRData.command == 0x1){
      receiverData.isSelecting = 1;
      Serial.println("receiverData.isSelecting is 1");
      Serial.println(receiverData.isSelecting);
    }
    else{
      receiverData.isSelecting = 0;
    }
    IrReceiver.resume();
   }
   else{
    receiverData.isSelecting = 0; 
    IrReceiver.resume();
   }
   //transmitter in pairing mode, change receiver mode to pairing mode
   if(transmitterData.isPairing == 1){
    receiverData.isSelecting = 0; 
    receiverData.isPairing = 1;
   }
   //transmitter in activating mode, change receiver mode to activating mode
   else if(transmitterData.isActivating == 1){
    if(transmitterData.acc_direct == 1){
      myservo.write(180);
    }
    else if(transmitterData.acc_direct == -1){
      myservo.write(0);
    }
   }
   if(receiverData.isSelecting == 0 && receiverData.isPairing == 0 && receiverData.isActivating == 0){
    digitalWrite(2, LOW);
   }
   else{
    digitalWrite(2, HIGH);
    esp_now_send(esp32_e_Address, (uint8_t *) &receiverData, sizeof(receiverData)); 
   }
   //Serial.println(transmitterData.isSelecting);
   //Serial.println(transmitterData.isPairing);
   //Serial.println(transmitterData.isActivating);
   //Serial.println(transmitterData.acc_direct);
   //Serial.println(receiverData.isSelecting);
   //Serial.println(receiverData.isPairing);
   //Serial.println(receiverData.isActivating);
   //Serial.println(receiverData.acc_direct);
   //Serial.println("---------------------------");
   delay(1000);
}

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <IRremote.h>
#define PIN_SEND 27

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

// MAC address of receivers
uint8_t esp32_ue_Address1[] = {0xE8, 0x31, 0xCD, 0x32, 0x38, 0xDC};
uint8_t esp32_ue_Address2[] = {0xE8, 0x31, 0xCD, 0x31, 0x9C, 0xCC};

// message structure for espnow communication
typedef struct struct_message {
  int acc_direct = 0;
  int isSelecting = 0;
  int isActivating = 0;
  int isPairing = 0;
} struct_message;

//Create 2 struct_messages called transmitterData & receiverData
struct_message transmitterData;
struct_message receiverData;
uint8_t Address[] = {0,0,0,0,0,0};
int active = 1;

// function for feedback of sending messages
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("rnLast Packet Send Status:t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// function for feedback of receiving messages
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  memcpy(Address,mac,sizeof(Address));
  Serial.println("Data received");
}

float rad2deg = 57.29578; 
float roll = 0, pitch = 0; 
int state = HIGH;

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(21, OUTPUT);
  
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // register send feedback function and receive feedback function
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_peer_info_t peerInfo;
  
  // register 2 receiver as peer
  memcpy(peerInfo.peer_addr, esp32_ue_Address1, 6);
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  //register mac address
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  memcpy(peerInfo.peer_addr, esp32_ue_Address2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  // whether mpu6050 is connected
  while (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);//acc ragne±2G
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);//angular velocity±250°/s
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);//frequency21Hz
  IrSender.begin(PIN_SEND); // Initializes IR sender
  delay(100);
}

void loop() {
  if(active == 1){
    /*
    status acquisition
    */
    //get acc angle & temp
    mpu.getEvent(&a, &g, &temp);
    roll = atan((a.acceleration.y) / (a.acceleration.z))*rad2deg;
    pitch = -1*atan((a.acceleration.x) / sqrt(sq(a.acceleration.y) + sq(a.acceleration.z)))*rad2deg;
    /*
    status update
    */
    if(pitch < 45 && pitch > -45){
      // initiating
      // activate irtransmitter and waiting for data from receivers (selecting mode)
      if(transmitterData.isSelecting == 0 && transmitterData.isActivating == 0 && transmitterData.isPairing == 0){
       if(state == HIGH){
          state = LOW;
          digitalWrite(21,state);
        }
        transmitterData.isSelecting = 1; // to select mode
      }
      // if in selecting mode, activate irtransmitter
      // wait for response from receiver
      else if(transmitterData.isSelecting == 1){
        IrSender.sendNEC(0x1, 0x1, true, 0); // activate irtransmitter
        if (receiverData.isSelecting == 1){
          digitalWrite(LED_BUILTIN, HIGH); // activate buildin led
        }
      }
      // from pairing mode to activating mode
      // send acc direction data to paired receiver to control the servo
      else if(transmitterData.isPairing == 1 && receiverData.isPairing == 1){
        transmitterData.isPairing = 0;
        transmitterData.isActivating = 1;
        digitalWrite(21, LOW);
      }
      else if(transmitterData.isActivating == 1){
        if (roll > 30){
          transmitterData.acc_direct = 1;// if roll > 20, acc_direct = 1
        }
        else if (roll < -30){
          transmitterData.acc_direct = -1;// if roll < -20, acc_direct = -1
        }
        else {
          transmitterData.acc_direct = 0;// else, acc_direct = 0
        }
      }
    }
    else{
      // not initiated
      // activate vibration alarm to initiate correctly
      if(transmitterData.isSelecting == 0 && transmitterData.isActivating == 0 && transmitterData.isPairing == 0){
        if(state == LOW){
            state = HIGH;
        }
        digitalWrite(21,state);// activate vibrator
      }
      // from selecting mode to pairing mode
      else if(transmitterData.isSelecting == 1 && receiverData.isSelecting == 1){
        transmitterData.isSelecting = 0;
        transmitterData.isPairing = 1;
        digitalWrite(LED_BUILTIN, LOW);// turn off buildin led
      }
      else if (transmitterData.isPairing == 1 && receiverData.isPairing == 1){
          if(state == HIGH){
            state = LOW;
          }
          else{
            state = HIGH;
          }
          digitalWrite(21,state);// activate vibrator
      }
      // from activating mode to not initiated state
      // break the link with receiver
      else if(transmitterData.isActivating == 1){
        transmitterData.isSelecting = 0;
        transmitterData.isPairing = 0;
        transmitterData.isActivating = 0;
        transmitterData.acc_direct = 0;
        esp_now_send(Address, (uint8_t *) &transmitterData, sizeof(transmitterData));
      }
    }
    if(transmitterData.isActivating != 0 || transmitterData.isPairing != 0){
        esp_now_send(Address, (uint8_t *) &transmitterData, sizeof(transmitterData));
    }
    
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

//ROCKET LAUNCH BASE CODE
//Author: Evan Farrell

#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>

//Syncing vars
long syncCode=0;          //Generated on the controller, used to confirm the base is synced to the controller
long launchCode=0;        //Generated on the base, used to confirm that the controller is synced to the base

//Pin variables
const int LAUNCH_PIN=5;   //When the launch pin is brought high, shorts the battery over the igniter.

//Timing vars
long launchTimeout=0;     //Used to prevent multiple launches back to back (very bad not good)
long lastSentPacket=0;    //Last time a LoRa packet was sent
long launchStart=0;       //Used to measure the time since the launch pin was brought high

//Consts
int LAUNCH_TIME = 3000;   //How long the launch pin is brought high

void setup() {
  Serial.begin(9600);
  Serial.println("---ROCKET LAUNCHER---");

  randomSeed(analogRead(5)); //use analog noise as the random seed
  launchCode=random(1000)+1;

  pinMode(LAUNCH_PIN,OUTPUT);
  digitalWrite(LAUNCH_PIN,0);

  while (!LoRa.begin(915E6)) {}
}

//Pulls the launch pin high for a set interval
void launchFunc(){
  if(millis()-launchTimeout <5000){return;} //sometimes lora packets are 'double-read'
  launchTimeout= millis(); 
  launchStart= millis();
  digitalWrite(LAUNCH_PIN,1);
  while(millis() -launchStart < LAUNCH_TIME){}
  digitalWrite(LAUNCH_PIN,0);
}

//Transmits a message over LoRa, takes in the string of message type to send.
void sendMessage(String type){
  JsonDocument doc;
  String output;
  lastSentPacket=millis();
  if(type == "sync"){
    doc["type"] = "sync";
    doc["syncCode"]=syncCode;
    doc["launchCode"]=launchCode;
  } else if (type == "launch"){
    Serial.println('im laucnhing');
  }
  
  serializeJson(doc, output);
  Serial.print("Sending packet: ");
  Serial.println(output);

  LoRa_toggle();
  LoRa.beginPacket();
  LoRa.print(output);
  LoRa.endPacket();
  LoRa_toggle();
}

//this is basically a work around for the LoRa library working weird with the MKR 1300
//it seems to only be able to strictly send or recieve, to get around this we just 
//restart the transmission between each packet send :) 
void LoRa_toggle(){
  LoRa.end();
  LoRa.begin(915E6);
}

//parses incoming LoRa messages.
void onRecieve(int packetSize) {
  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  Serial.print("Received: ");
  Serial.println(incoming);

  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, incoming);

  if (error) {
    Serial.print("JSON error: ");
    Serial.println(error.c_str());
    return;
  }
  String messageType=String(doc["type"].as<const char*>());

  if(messageType == "sync") {
    syncCode=int(doc["syncCode"]);
    launchCode=random(1000)+1;
    sendMessage("sync");
  } else if (messageType == "launch") {
    if(int(doc["syncCode"]) == syncCode and int(doc["launchCode"]) ==launchCode){
      Serial.println("boom");
      launchFunc();
    }else{
      Serial.println("LAUNCH ABORTED NOT SYNCED");
    }
  }
}

//this guy just hangs around and waits for someone to talk to him... very sad :c
void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    onRecieve(packetSize);
  }
}
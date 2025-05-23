//ROCKET LAUNCH CONTROLLER CODE
//Author: Evan Farrell

#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <LCD_I2C.h>
#include <Wire.h>
#include <ArduinoLowPower.h>

LCD_I2C lcd(0x27, 16, 2);

//Timing vars
long lastSentPacket=0;    //Last time a LoRa packet was sent
long lastDisplayUpdate=0; //Last time the display was updated
long launchTimeCount=0;   //How many ms into the launch sequence we are
long lastLaunchTime=0;    //Last time a sucessfull launch completed
long lastLaunchAbort=0;   //Last time a launch was aborted
int launchPercent=0;      //How long the launch button was held down before launch message sent
int launchSeqEngaged=0;   //True if the launch sequence is currently engaged
long sleepCheck=0;

//Syncing vars
long syncCode=0;          //Generated on the controller, used to confirm the base is synced to the controller
long launchCode=0;        //Generated on the base, used to confirm that the controller is synced to the base

//Pin declaration
int LED_ON_PIN=1;         //The green led
int LED_KEY_PIN=2;        //Red led
int LAUNCH_BUTTON_PIN=3;  
int KEY_OFF_PIN=4;
int KEY_ON_PIN=5;

//Pin variables
int keyOffState;          //True if safety key disengaged
int keyOnState;           //True if safety key engaged
int launchButtonState;    //True if launch button is NOT PRESSED (pullup pin)

//Constants
float LAUNCH_TIME=5000.0;     //How long the button must be held before a launch message is sent
int DISPLAY_UPDATE_RATE=400;  //How often to refresh the display
int LAUNCH_ABORT_TIME=2000;   //How long to show the aborted launch message
int SYNC_CHECK=10000;         //How often to attempt to resync to the base
int ABORT_LAUNCH_PERCENT=20;  //The percentage of a 'full' launch to display the abort message, so feathering the launch button wont show it

void setup() {
  Serial.begin(9600);
  Serial.println("---ROCKET CONTROLLER---");

  randomSeed(analogRead(5)); //use analog noise as the random seed
  syncCode=random(1000)+1;

  pinMode(LED_ON_PIN,OUTPUT);
  digitalWrite(LED_ON_PIN,1);
  pinMode(LED_KEY_PIN,OUTPUT);
  pinMode(LAUNCH_BUTTON_PIN,INPUT_PULLUP);
  pinMode(KEY_OFF_PIN,INPUT_PULLUP);
  pinMode(KEY_ON_PIN,INPUT_PULLUP);

  Wire.begin();
  lcd.begin(&Wire);
  lcd.display();
  lcd.backlight();

  while (!LoRa.begin(915E6)) {}
}

//Transmits a message over LoRa, takes in the string of message type to send.
void sendMessage(String type){
  JsonDocument doc;
  String output;

  lastSentPacket=millis();
  if(type == "sync"){
    doc["type"] = "sync";
    doc["syncCode"]=syncCode;
  } else if (type == "launch"){
    doc["type"] = "launch";
    doc["syncCode"]=syncCode;
    doc["launchCode"]=launchCode;
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
    if(int(doc["syncCode"]) == syncCode){  
      launchCode=int(doc["launchCode"]);
      Serial.println("Synced up!");
    } else {
      Serial.println("Desynced from base");
      launchCode=0; //if desynced restart syncing process
    } 
  } else if (messageType == "launch") {
    Serial.println("boom"); //could have some launch confirmed logic or something here! right not does nothing...
  };
}

//Fills a progress bar across the lcd
void drawProgressBar(int percentage) {
  int blocks = map(percentage, 0, 100, 0, 16);
  lcd.setCursor(0, 1);
  for (int i = 0; i < 16; i++){
    if (i < blocks) {
      lcd.print((char)255); 
    } else {
      lcd.print(" ");       
    }
  }
}

//Updates the led display, also handles the launch sequence.
void displayUpdate(){
  if(millis()-lastDisplayUpdate < 400){return;} //only run the display loop every 400ms
  if(millis()-lastLaunchTime    < 5000){return;} //once the launch is activated, freeze the display loop
  lastDisplayUpdate=millis();

  lcd.setCursor(0, 0);
  lcd.clear();
  
  //If launch is recently aborted show a message for a brief time
  if(millis()-lastLaunchAbort < LAUNCH_ABORT_TIME){
    lcd.print(" ABORTED LAUNCH");
    return;
  }

  //Syncing and pre-launch display
  if (!launchSeqEngaged){
    if (!(launchCode)){
      lcd.print("Attempting Sync");
    }else{
      lcd.print("Synced With Base");
    }

    lcd.setCursor(0,1);
    if(launchCode && keyOffState){
      lcd.print("Turn Safety Key");
    }else if (launchCode && keyOnState) {
      lcd.print("Ready to Launch");
    }

  } else{//launch sequence stuff
    lcd.print("LAUNCH INITIATED");
    lcd.setCursor(0,1);

    //During launch display a bar filling
    if (launchPercent >= 0 && launchPercent <= 100){
        drawProgressBar(launchPercent);
    }
    if (launchPercent>=100){
      lcd.clear();
      lcd.setCursor(0,1);

      //reset launch seq
      launchSeqEngaged=0;
      launchTimeCount=millis();

      lcd.print("   LAUNCHING");
      sendMessage("launch");
      lastLaunchTime=millis(); //Just make it stay on launching  display for 5 secs.
    }
  }
}

void loop() {
  //send sync packets every 3-3.5 seconds to prevent blocking... ironically the case if both the base and controller are in sync sending messages ;)
  if (!(launchCode) && (millis()-lastSentPacket > 3000 + random(500))){
    //If the controller does not have the launch code send a sync message
    sendMessage("sync");
  } else if(millis()-lastSentPacket > SYNC_CHECK && !launchSeqEngaged){ 
    //regardless if synced up, check again at set intervals
    syncCode=random(1000)+1;
    launchCode=0;
    sendMessage("sync");
  }

  //Check for incoming packets
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    onRecieve(packetSize);
  }

  //these are all set to pull-up 
  //i.e button state 1 means not pressed!!
  launchButtonState=digitalRead(LAUNCH_BUTTON_PIN);
  keyOffState=digitalRead(KEY_OFF_PIN);
  keyOnState= digitalRead(KEY_ON_PIN);

  //If the safety key is engaged turn on a led
  if(keyOnState){
    digitalWrite(LED_KEY_PIN,1);
  } else {
    digitalWrite(LED_KEY_PIN,0);
  }

  //The launch button is held, the safety key is engaged, and the devices are synced
  if(!launchButtonState && keyOnState && launchCode){
    launchSeqEngaged=1;
    launchPercent=floor((((millis()-launchTimeCount))/(LAUNCH_TIME))*100);
  } else {
    launchSeqEngaged=0;
    //if launch was initiated set a flag to show the abort display
    if((launchPercent>ABORT_LAUNCH_PERCENT) && (launchPercent < 100)){
      lastLaunchAbort=millis();
    }
    launchPercent=0;
    launchTimeCount=millis();
  }
  displayUpdate();
}
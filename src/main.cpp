#include "Arduino.h"

#include "wifi_config.h"
#include "persistentValue.h"
#include <ESPUI.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

#include <RadarSensor.h>


#define RADAR_RX_PIN D3
#define RADAR_TX_PIN D2

#define NUM_PARAMS 19
#define META_PARAMS 4
#define MAX_DETECT 3


// Exemple d’adresses OSC
struct OSCParam {
  PersistentValue* name;
  PersistentValue* address;
  PersistentValue* minVal;
  PersistentValue* maxVal;
  PersistentValue* sendToTD;
  PersistentValue* sendToAbleton;
  PersistentValue* testOn;
};
OSCParam oscParams[NUM_PARAMS];

struct baseOSCParam {
  String name;
  String address;
  uint8_t minVal;
  uint8_t maxVal;
  bool sendToTD;
  bool sendToAbleton;
  bool testOn;
};
baseOSCParam baseOscParams[] = {
  {"presence", "/presence", 0, 100, false, false, false},
  {"mean_x", "/mean/x", 0, 100, false, false, false},
  {"mean_y", "/mean/y", 0, 100, false, false, false},
  {"mean_dist", "/mean/dist", 0, 100, false, false, false},
  {"1_x", "/1/x", 0, 100, true, false, false},
  {"1_y",  "/1/y",  0, 100, false, false, false},
  {"1_dist",  "/1/dist",  0, 100, false, false, false},
  {"1_speed", "/1/speed", 0, 100, false, false,  false},
  {"1_angle",  "/1/angle",  0, 100, false, false, false},
  {"2_x", "/2/x", 0, 100, false, false, false},
  {"2_y",  "/2/y",  0, 100, false, false, false},
  {"2_dist",  "/2/dist",  0, 100, false, false, false},
  {"2_speed", "/2/speed", 0, 100, false, false, false},
  {"2_angle",  "/2/angle",  0, 100, false, false, false},
  {"3_x", "/3/x", 0, 100, false, false, false},
  {"3_y",  "/3/y",  0, 100, false, false, false},
  {"3_dist",  "/3/dist",  0, 100, false, false, false},
  {"3_speed", "/3/speed", 0, 100, false, false, false},
  {"3_angle",  "/3/angle",  0, 100, false, false, false}
};

// variables liées à la gestion du radar
uint32_t lastRadarReading = 0;
uint32_t lastTest = 0;
bool is_radar_connected = false;

float meanX = 0;
float meanY = 0;
float meanDist = 0;
float meanDetected = 0;

struct radarData {
  bool detected;
  bool reallyDetected;
  float x;
  float y;
  float distance;
  float speed;
  float angle;
};

radarData radarsData[MAX_DETECT];
RadarSensor radar(RADAR_TX_PIN, RADAR_RX_PIN);
uint8_t countMessages[MAX_DETECT] = {0, 0, 0};
uint8_t goodCountMessages[MAX_DETECT] = {0, 0, 0};

//persistentvalues pour les valeurs de contrôle de ESPUI
PersistentValue* isStarted;

PersistentValue* ipAddress;
PersistentValue* abletonPort;
PersistentValue* tdPort;

PersistentValue* maxDist;
PersistentValue* inactivityTimer;
PersistentValue* readingFrequency;
PersistentValue* espuiID;


// initialise les valeurs de moyenne glissante
const uint8_t sizeMean = 10;
int16_t xForMean[MAX_DETECT][sizeMean];
int16_t yForMean[MAX_DETECT][sizeMean];
int16_t distForMean[MAX_DETECT][sizeMean];
int16_t speedForMean[MAX_DETECT][sizeMean];
int16_t angleForMean[MAX_DETECT][sizeMean];

// UDP pour les messages OSC
WiFiUDP Udp;



// ====== SETUP UI ======
void setupUI() {
  uint16_t generalTab = ESPUI.addControl(ControlType::Tab, "Network", "Network");
  uint16_t oscTab1 = ESPUI.addControl(ControlType::Tab, "OSC meta data", "OSC meta data");
  uint16_t oscTab2 = ESPUI.addControl(ControlType::Tab, "OSC 1 radar data", "OSC 1 radar data");
  uint16_t oscTab3 = ESPUI.addControl(ControlType::Tab, "OSC 2 radar data", "OSC 2 radar data");
  uint16_t oscTab4 = ESPUI.addControl(ControlType::Tab, "OSC 3 radar data", "OSC 3 radar data");
  uint16_t radarTab = ESPUI.addControl(ControlType::Tab, "Radar control", "Radar control");
  
  // general tab
  espuiID = new PersistentValue("id", ControlColor::Wetasphalt, 0, 0, 4, generalTab);
  isStarted = new PersistentValue("Start", ControlColor::Alizarin, false, generalTab);
  String baseIP = "192.168.1.108";
  ipAddress = new PersistentValue("IP destination", ControlColor::Peterriver, baseIP, generalTab);
  abletonPort = new PersistentValue("ableton port", ControlColor::Wetasphalt, 8001, 1000, 12000, generalTab);
  tdPort = new PersistentValue("td port", ControlColor::Wetasphalt, 9001, 1000, 12000, generalTab);

  // osc tab
  for(int i = 0; i < META_PARAMS; i++) {
    ESPUI.addControl(ControlType::Separator, "Meta parameters", "Meta parameters", ControlColor::Turquoise, oscTab1);
    String label = baseOscParams[i].name;
    oscParams[i].address = new PersistentValue(label+"_address", ControlColor::Peterriver, baseOscParams[i].address, oscTab1);
    oscParams[i].minVal = new PersistentValue(label+"_min (%)", ControlColor::Wetasphalt, 0, 0, 100, oscTab1);
    oscParams[i].maxVal = new PersistentValue(label+"_max (%)", ControlColor::Wetasphalt, 100, 0, 100, oscTab1);
    // --- Switchs persistants ---
    oscParams[i].sendToTD = new PersistentValue(label+"_TD", ControlColor::Alizarin, baseOscParams[i].sendToTD, oscTab1);
    oscParams[i].sendToAbleton = new PersistentValue(label+"_AB", ControlColor::Alizarin, baseOscParams[i].sendToAbleton, oscTab1);
    // Bouton de test
    oscParams[i].testOn = new PersistentValue(label+"_test", ControlColor::Alizarin, baseOscParams[i].testOn, oscTab1);
  }
  for (int i = META_PARAMS; i < (5+META_PARAMS); i++) {
    ESPUI.addControl(ControlType::Separator, baseOscParams[i].name.c_str(), baseOscParams[i].name.c_str(), ControlColor::Turquoise, oscTab2);
    String label = baseOscParams[i].name;
    oscParams[i].address = new PersistentValue(label+"_address", ControlColor::Peterriver, baseOscParams[i].address, oscTab2);
    oscParams[i].minVal = new PersistentValue(label+"_min (%)", ControlColor::Wetasphalt, 0, 0, 100, oscTab2);
    oscParams[i].maxVal = new PersistentValue(label+"_max (%)", ControlColor::Wetasphalt, 100, 0, 100, oscTab2);
    // --- Switchs persistants ---
    oscParams[i].sendToTD = new PersistentValue(label+"_TD", ControlColor::Alizarin, baseOscParams[i].sendToTD, oscTab2);
    oscParams[i].sendToAbleton = new PersistentValue(label+"_AB", ControlColor::Alizarin, baseOscParams[i].sendToAbleton, oscTab2);
    // Bouton de test
    oscParams[i].testOn = new PersistentValue(label+"_test", ControlColor::Alizarin, baseOscParams[i].testOn, oscTab2);
  }
  for (int i = (5+META_PARAMS); i < (10+META_PARAMS); i++) {
    ESPUI.addControl(ControlType::Separator, baseOscParams[i].name.c_str(), baseOscParams[i].name.c_str(), ControlColor::Turquoise, oscTab3);
    String label = baseOscParams[i].name;
    oscParams[i].address = new PersistentValue(label+"_address", ControlColor::Peterriver, baseOscParams[i].address, oscTab3);
    oscParams[i].minVal = new PersistentValue(label+"_min (%)", ControlColor::Wetasphalt, 0, 0, 100, oscTab3);
    oscParams[i].maxVal = new PersistentValue(label+"_max (%)", ControlColor::Wetasphalt, 100, 0, 100, oscTab3);
    // --- Switchs persistants ---
    oscParams[i].sendToTD = new PersistentValue(label+"_TD", ControlColor::Alizarin, baseOscParams[i].sendToTD, oscTab3);
    oscParams[i].sendToAbleton = new PersistentValue(label+"_AB", ControlColor::Alizarin, baseOscParams[i].sendToAbleton, oscTab3);
    // Bouton de test
    oscParams[i].testOn = new PersistentValue(label+"_test", ControlColor::Alizarin, baseOscParams[i].testOn, oscTab3);
  }
  for (int i = (10+META_PARAMS); i < NUM_PARAMS; i++) {
    ESPUI.addControl(ControlType::Separator, baseOscParams[i].name.c_str(), baseOscParams[i].name.c_str(), ControlColor::Turquoise, oscTab4);
    String label = baseOscParams[i].name;
    oscParams[i].address = new PersistentValue(label+"_address", ControlColor::Peterriver, baseOscParams[i].address, oscTab4);
    oscParams[i].minVal = new PersistentValue(label+"_min (%)", ControlColor::Wetasphalt, 0, 0, 100, oscTab4);
    oscParams[i].maxVal = new PersistentValue(label+"_max (%)", ControlColor::Wetasphalt, 100, 0, 100, oscTab4);
    // --- Switchs persistants ---
    oscParams[i].sendToTD = new PersistentValue(label+"_TD", ControlColor::Alizarin, baseOscParams[i].sendToTD, oscTab4);
    oscParams[i].sendToAbleton = new PersistentValue(label+"_AB", ControlColor::Alizarin, baseOscParams[i].sendToAbleton, oscTab4);
    // Bouton de test
    oscParams[i].testOn = new PersistentValue(label+"_test", ControlColor::Alizarin, baseOscParams[i].testOn, oscTab4);
  }

  // radar tab
  readingFrequency = new PersistentValue("fréquence de lecture du radar", ControlColor::Wetasphalt, 200, 100, 2000, radarTab);
  maxDist = new PersistentValue("max distance", ControlColor::Wetasphalt, 7, 0, 7, radarTab);
  inactivityTimer = new PersistentValue("inactivity timer", ControlColor::Wetasphalt, 5, 0, 10, radarTab);
}


int16_t rollingAverage(int16_t dataArray[sizeMean], int16_t data){
  // calcule la moyenne glissante sur x données (défini par sizeMean)
  int32_t somme = 0;
  for (int i=1; i<sizeMean; i++){
    dataArray[i-1] = dataArray[i];
    somme += dataArray[i-1];
  }
  dataArray[sizeMean-1] = data;
  somme += dataArray[sizeMean-1];
  return somme/sizeMean;
}

// Comparison function for sorting based on age
int compareByDist(const void* a, const void* b){
    return ((struct radarData*)a)->distance - ((struct radarData*)b)->distance;
}

void updateRadarData(uint8_t detectedID, uint16_t detected, int16_t x, int16_t y, int16_t distance, int16_t speed, int16_t angle){
  // met à jour les valeurs du radar d'un boitier avec son id et les nouvelles valeurs
  int bound = maxDist->getInt()*1000;
  radarsData[detectedID].detected = detected;
  radarsData[detectedID].x = constrain(map(x, -bound, bound, 0, 100), 0, 100) / 100.0;
  radarsData[detectedID].y = constrain(map(y, -bound, bound, 0, 100), 0, 100) / 100.0;
  radarsData[detectedID].distance = constrain(map(distance*detected, 0, bound, 0, 100), 0, 100) / 100.0;
  radarsData[detectedID].speed = constrain(map(speed*detected, 0, 100, 0, 100), 0, 100) / 100.0;
  radarsData[detectedID].angle = constrain(map(angle, -90, 90, 0, 100), 0, 100) / 100.0;
}

void testSend(bool toAbleton, bool toTD, String address){
  IPAddress outIP;
  outIP.fromString(ipAddress->getString());
  float val = 0.5;
  OSCMessage msg(address.c_str());
  if(toAbleton){
      Serial.println("sending to ableton..");
      msg.add(val);
      Udp.beginPacket(outIP, abletonPort->getInt());
      msg.send(Udp);
      Udp.endPacket();
      msg.empty();
    }
    if(toTD){
      Serial.println("sending to TD..");
      msg.add(val);
      Udp.beginPacket(outIP, tdPort->getInt());
      msg.send(Udp);
      Udp.endPacket();
      msg.empty();
    }
}

void sendRadarData(uint8_t detectedID, float valuesArray[5]){
  // envoie les valeurs du radar
  IPAddress outIP;
  outIP.fromString(ipAddress->getString());
  for(int i=detectedID*5+META_PARAMS; i<(detectedID*5+5+META_PARAMS); i++){
    OSCMessage msg(oscParams[i].address->getString().c_str());
    bool toAbleton = oscParams[i].sendToAbleton->getBool();
    bool toTD = oscParams[i].sendToTD->getBool();
    if(!toAbleton && !toTD){ continue;}
    float value = (valuesArray[(i-META_PARAMS)%5] * (oscParams[i].maxVal->getInt() - oscParams[i].minVal->getInt()) + oscParams[i].minVal->getInt()) / 100.0;
    if(toAbleton){
      msg.add(value);
      Udp.beginPacket(outIP, abletonPort->getInt());
      msg.send(Udp);
      Udp.endPacket();
      msg.empty();
    }
    if(toTD){
      msg.add(value);
      Udp.beginPacket(outIP, tdPort->getInt());
      msg.send(Udp);
      Udp.endPacket();
      msg.empty();
    }
    delay(5);
  }
}

void sendMetaData(float metaArray[META_PARAMS]){
  IPAddress outIP;
  outIP.fromString(ipAddress->getString());
  for(int a=0; a<META_PARAMS; a++){
    OSCMessage msg(oscParams[a].address->getString().c_str());
    bool toAbleton = oscParams[a].sendToAbleton->getBool();
    bool toTD = oscParams[a].sendToTD->getBool();
    if(!toAbleton && !toTD){ continue;}
    float value = (metaArray[a] * (oscParams[a].maxVal->getInt() - oscParams[a].minVal->getInt()) + oscParams[a].minVal->getInt()) / 100.0;
    if(toAbleton){
      msg.add(value);
      Udp.beginPacket(outIP, abletonPort->getInt());
      msg.send(Udp);
      Udp.endPacket();
      msg.empty();
    }
    if(toTD){
      msg.add(value);
      Udp.beginPacket(outIP, tdPort->getInt());
      msg.send(Udp);
      Udp.endPacket();
      msg.empty();
    }
  }
}


void setup(){

  // ouvre le port série et le port série pour le radar
  Serial.begin(115200); //Feedback over Serial Monitor
  radar.begin(256000);
  delay(500);

  pinMode(LED_BUILTIN, OUTPUT);

  begin_wifi();
  delay(2000);

  // initialise les controls de ESPUI
  ESPUI.begin("The Lights Which Can Be Heard");
  setupUI();

  // rolling average init
  for(int j=0; j<MAX_DETECT; j++){
    for(int i=0; i<sizeMean; i++){
      xForMean[j][i] = -(maxDist->getInt()*1000);
      yForMean[j][i] = -(maxDist->getInt()*1000);
      distForMean[j][i] = 0;
      speedForMean[j][i] = 0;
      angleForMean[j][i] = -90;
    }
  }

  delay(2000);
  Serial.println("Starting programm..");
}

void loop(){
  if((millis() - lastTest) > 2000){
    lastTest = millis();
    for(int a=0; a<NUM_PARAMS; a++){
      if(oscParams[a].testOn->getBool()){
        testSend(oscParams[a].sendToAbleton->getBool(), oscParams[a].sendToTD->getBool(), oscParams[a].address->getString());
      }
    }
  }
  if(radar.update() && (millis() - lastRadarReading) > 1/readingFrequency->getInt()){
    lastRadarReading = millis();
    int bound = maxDist->getInt()*1000;
    for (int i = 0; i < MAX_DETECT; i++) {
      radarsData[i] = {};
      RadarTarget t = radar.getTarget(i);
      bool realData = t.x != 0.0 && abs(t.x) > -bound;
      if(t.detected && realData && t.distance < (maxDist->getInt()*1000)){
        // met à jour les valeurs du radar local avec calcul d'une moyenne glissante
        updateRadarData(i,
          t.detected,
          rollingAverage(xForMean[i], t.x),
          rollingAverage(yForMean[i], t.y),
          rollingAverage(distForMean[i], (int)t.distance),
          rollingAverage(speedForMean[i], t.speed),
          rollingAverage(angleForMean[i], t.angle)
        );
      }
      else{
        updateRadarData(i,
          t.detected,
          rollingAverage(xForMean[i], -bound),
          rollingAverage(yForMean[i], -bound),
          rollingAverage(distForMean[i], 0),
          rollingAverage(speedForMean[i], 0),
          rollingAverage(angleForMean[i], -90)
        );
      }
    }
    qsort(radarsData, MAX_DETECT, sizeof(struct radarData), compareByDist);
    for(int j=0; j<MAX_DETECT; j++){
      if(radarsData[j].x != 0.0){
        countMessages[j] = countMessages[j] + 1;
        goodCountMessages[j] = countMessages[j];
        radarsData[j].reallyDetected = countMessages[j] >= sizeMean;
        if(isStarted->getBool() && radarsData[j].reallyDetected){
          float dataToSend[5] = {radarsData[j].x, radarsData[j].y, radarsData[j].distance,
                                radarsData[j].speed, radarsData[j].angle};
          sendRadarData(j, dataToSend);
        }
      }
      else {
        if(countMessages[j] > 0 && (goodCountMessages[j]-countMessages[j]) < sizeMean){
          countMessages[j] = countMessages[j] - 1;
          float dataToSend[5] = {radarsData[j].x, radarsData[j].y, radarsData[j].distance, 
                                radarsData[j].speed, radarsData[j].angle};
          if(isStarted->getBool()){
            sendRadarData(j, dataToSend);
          }
        }
        else {
          countMessages[j] = 0;
          float dataToSend[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
        }
      }
    }
    meanX = (radarsData[0].x + radarsData[1].x + radarsData[2].x) / MAX_DETECT;
    meanY = (radarsData[0].y + radarsData[1].y + radarsData[2].y) / MAX_DETECT;
    meanDist = (radarsData[0].distance + radarsData[1].distance + radarsData[2].distance) / MAX_DETECT;
    meanDetected = ((radarsData[0].reallyDetected ? 1 : 0) + (radarsData[1].reallyDetected ? 1 : 0) + (radarsData[2].reallyDetected ? 1 : 0)) / float(MAX_DETECT);
    float meanData[META_PARAMS] = {meanDetected, meanX, meanY, meanDist};
    if(isStarted->getBool()){ sendMetaData(meanData); }
  }
  delay(20);
}
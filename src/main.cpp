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

#define NUM_PARAMS 20
#define META_PARAMS 5
#define MAX_DETECT 3


// Struct for OSC parameters corresponding to each data
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
  {"mean_angle", "/mean/angle", 0, 100, false, false, false},
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

// radar data / control variables
bool is_radar_connected = false;

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

// meta parameters variables
float meanDetected = 0;
float meanX = 0;
float meanY = 0;
float meanDist = 0;
float meanAngle = 0;

// data handling variables
uint32_t lastRadarReading = 0;
uint32_t lastTest = 0;
uint8_t countMessages[MAX_DETECT] = {0, 0, 0};
uint8_t goodCountMessages[MAX_DETECT] = {0, 0, 0};


//persistentvalues for ESPUI control values 
PersistentValue* isStarted;

PersistentValue* ipAddress;
PersistentValue* abletonPort;
PersistentValue* tdPort;

PersistentValue* minDist;
PersistentValue* maxDist;
PersistentValue* inactivityTimer;
PersistentValue* readingFrequency;


// initialise rolling average arrays
const uint8_t sizeMean = 10;
int16_t xForMean[MAX_DETECT][sizeMean];
int16_t yForMean[MAX_DETECT][sizeMean];
int16_t distForMean[MAX_DETECT][sizeMean];
int16_t speedForMean[MAX_DETECT][sizeMean];
int16_t angleForMean[MAX_DETECT][sizeMean];


// UDP pour les messages OSC
WiFiUDP Udp;


// ====== SETUP UI ======
void addOscControls(int startIdx, int endIdx, uint16_t tabId) {
  for (int i = startIdx; i < endIdx; i++) {
    ESPUI.addControl(ControlType::Separator, baseOscParams[i].name.c_str(), baseOscParams[i].name.c_str(), ControlColor::Turquoise, tabId);
    String label = baseOscParams[i].name;
    oscParams[i].address = new PersistentValue(label + "_address", ControlColor::Peterriver, baseOscParams[i].address, tabId);
    oscParams[i].minVal = new PersistentValue(label + "_min (%)", ControlColor::Wetasphalt, 0, 0, 100, tabId);
    oscParams[i].maxVal = new PersistentValue(label + "_max (%)", ControlColor::Wetasphalt, 100, 0, 100, tabId);
    oscParams[i].sendToTD = new PersistentValue(label + "_TD", ControlColor::Alizarin, baseOscParams[i].sendToTD, tabId);
    oscParams[i].sendToAbleton = new PersistentValue(label + "_AB", ControlColor::Alizarin, baseOscParams[i].sendToAbleton, tabId);
    oscParams[i].testOn = new PersistentValue(label + "_test", ControlColor::Alizarin, baseOscParams[i].testOn, tabId);
  }
}

void setupUI() {
  uint16_t generalTab = ESPUI.addControl(ControlType::Tab, "Network", "Network");
  uint16_t oscTabs[] = {
    ESPUI.addControl(ControlType::Tab, "OSC meta data", "OSC meta data"),
    ESPUI.addControl(ControlType::Tab, "OSC 1 radar data", "OSC 1 radar data"),
    ESPUI.addControl(ControlType::Tab, "OSC 2 radar data", "OSC 2 radar data"),
    ESPUI.addControl(ControlType::Tab, "OSC 3 radar data", "OSC 3 radar data")
  };
  uint16_t radarTab = ESPUI.addControl(ControlType::Tab, "Radar control", "Radar control");
  
  // general tab
  isStarted = new PersistentValue("Start", ControlColor::Alizarin, false, generalTab);
  String baseIP = "192.168.1.108";
  ipAddress = new PersistentValue("IP destination", ControlColor::Peterriver, baseIP, generalTab);
  abletonPort = new PersistentValue("ableton port", ControlColor::Wetasphalt, 8001, 1000, 12000, generalTab);
  tdPort = new PersistentValue("td port", ControlColor::Wetasphalt, 9001, 1000, 12000, generalTab);

  // osc tabs
  // for each parameter, display controls for address, min & max, toggle for Touch/Ableton send and test button
  addOscControls(0, META_PARAMS, oscTabs[0]);
  addOscControls(META_PARAMS, META_PARAMS + 5, oscTabs[1]);
  addOscControls(META_PARAMS + 5, META_PARAMS + 10, oscTabs[2]);
  addOscControls(META_PARAMS + 10, NUM_PARAMS, oscTabs[3]);

  // radar tab
  readingFrequency = new PersistentValue("Reading frequency (in ms)", ControlColor::Wetasphalt, 100, 50, 2000, radarTab);
  minDist = new PersistentValue("min distance (in m)", ControlColor::Wetasphalt, 2, 0, 7, radarTab);
  maxDist = new PersistentValue("max distance (in m)", ControlColor::Wetasphalt, 7, 0, 7, radarTab);
  inactivityTimer = new PersistentValue("inactivity timer (in ms)", ControlColor::Wetasphalt, 2000, 500, 10000, radarTab);
}


// ====== rolling average calculation on x data (sizeMean) ======
int16_t rollingAverage(int16_t dataArray[sizeMean], int16_t data){
  int32_t somme = 0;
  for (int i=1; i<sizeMean; i++){
    dataArray[i-1] = dataArray[i];
    somme += dataArray[i-1];
  }
  dataArray[sizeMean-1] = data;
  somme += dataArray[sizeMean-1];
  return somme/sizeMean;
}


// ====== Comparison function for sorting based on distance to radar ======
int compareByDist(const void* a, const void* b){
    return ((struct radarData*)a)->distance - ((struct radarData*)b)->distance;
}


// ====== updata radar data into struct ======
void updateRadarData(uint8_t detectedID, uint16_t detected, int16_t x, int16_t y, int16_t distance, int16_t speed, int16_t angle){
  // for each data, normalize between boundaries decided by min and max distance on ESPUI
  int bound = maxDist->getInt()*1000;
  int min = minDist->getInt()*1000;
  radarsData[detectedID].detected = detected;
  radarsData[detectedID].x = constrain(map(x, -bound, bound, 0, 100), 0, 100) / 100.0;
  radarsData[detectedID].y = constrain(map(y, -bound, bound, 0, 100), 0, 100) / 100.0;
  radarsData[detectedID].distance = constrain(map(distance*detected, min, bound, 0, 100), 0, 100) / 100.0;
  radarsData[detectedID].speed = constrain(map(speed*detected, 0, 100, 0, 100), 0, 100) / 100.0;
  radarsData[detectedID].angle = constrain(map(angle, -90, 90, 0, 100), 0, 100) / 100.0;
}


// ====== test OSC send function ======
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

// ====== sending radar data (x, y, dist, speed, angle) ======
void sendRadarData(uint8_t detectedID, float valuesArray[5]){
  // only if toggles to send to Ableton and/or Touch are on
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

// ====== sending meta params data (means for presence, x, y, dist and angle) ======
void sendMetaData(float metaArray[META_PARAMS]){
  // only if toggles to send to Ableton and/or Touch are on
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
  // open serial for USB and radar UART
  Serial.begin(115200);
  radar.begin(256000);
  delay(500);

  begin_wifi();
  delay(2000);

  // ESPUI control init
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
  // test OSC if toggle on
  if((millis() - lastTest) > 2000){
    lastTest = millis();
    for(int a=0; a<NUM_PARAMS; a++){
      if(oscParams[a].testOn->getBool()){
        testSend(oscParams[a].sendToAbleton->getBool(), oscParams[a].sendToTD->getBool(), oscParams[a].address->getString());
      }
    }
  }
  // get new radar data every x ms (based on readingFrequency)
  if(radar.update() && (millis() - lastRadarReading) > readingFrequency->getInt()){
    lastRadarReading = millis();
    int bound = maxDist->getInt()*1000;
    // for loop through all 3 detection possible at once
    for (int i = 0; i < MAX_DETECT; i++) {
      radarsData[i] = {};
      RadarTarget t = radar.getTarget(i);
      bool realData = t.x != 0.0 && abs(t.x) > -bound;
      if(t.detected && realData && t.distance < (maxDist->getInt()*1000) && t.distance > (minDist->getInt()*1000)){
        // if x data is not equal to 0, is between -bound and bound, as well as distance between min and max dist,
        // update radar data with rolling average for all parameters
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
        // if conditions are not fullfilled, then fill radarsData with min values for each param 
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
    // sort data based on distance to avoid disturbances in id (data can jump from id 0 to 1 for instance)
    qsort(radarsData, MAX_DETECT, sizeof(struct radarData), compareByDist);
    for(int j=0; j<MAX_DETECT; j++){
      if(radarsData[j].x > 0.05){
        // create a "reallyDetected" bool based on countMessages
        // If you have data filling all requirements for x time (based on inactivityTimer), reallyDetected = true
        countMessages[j] = countMessages[j] + 1;
        goodCountMessages[j] = countMessages[j];
        radarsData[j].reallyDetected = countMessages[j] >= int(inactivityTimer->getInt() / readingFrequency->getInt());
        if(isStarted->getBool() && radarsData[j].reallyDetected){
          float dataToSend[5] = {radarsData[j].x, radarsData[j].y, radarsData[j].distance,
                                radarsData[j].speed, radarsData[j].angle};
          sendRadarData(j, dataToSend);
        }
      }
      // else if data is not good anymore, decrease countMessages for some time (inactivityTimer) until you stop sending data
      else if(countMessages[j] > 0 && (goodCountMessages[j]-countMessages[j]) < int(inactivityTimer->getInt() / readingFrequency->getInt())){
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
    // part for meta params computing and sending
    meanDetected = ((radarsData[0].reallyDetected ? 1 : 0) + (radarsData[1].reallyDetected ? 1 : 0) + (radarsData[2].reallyDetected ? 1 : 0)) / float(MAX_DETECT);
    meanX = (radarsData[0].x + radarsData[1].x + radarsData[2].x) / MAX_DETECT;
    meanY = (radarsData[0].y + radarsData[1].y + radarsData[2].y) / MAX_DETECT;
    meanDist = (radarsData[0].distance + radarsData[1].distance + radarsData[2].distance) / MAX_DETECT;
    meanAngle = (radarsData[0].angle + radarsData[1].angle + radarsData[2].angle) / MAX_DETECT;
    float meanData[META_PARAMS] = {meanDetected, meanX, meanY, meanDist, meanAngle};
    bool noSends = countMessages[0] == 0 && countMessages[1] == 0 && countMessages[2] == 0;
    if(isStarted->getBool() && !noSends){
      sendMetaData(meanData);
    }
  }
  delay(20);
}
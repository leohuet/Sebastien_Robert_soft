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

#define NUM_PARAMS 15
#define MAX_DETECT 3


// Exemple d’adresses OSC
struct OSCParam {
  PersistentValue* name;
  PersistentValue* address;
  PersistentValue* minVal;
  PersistentValue* maxVal;
  PersistentValue* sendToTD;
  PersistentValue* sendToAbleton;
};
OSCParam oscParams[NUM_PARAMS];

struct baseOSCParam {
  String name;
  String address;
  uint8_t minVal;
  uint8_t maxVal;
  bool sendToTD;
  bool sendToAbleton;
};
baseOSCParam baseOscParams[] = {
  {"1_x", "/1/x", 0, 100, true, false},
  {"1_y",  "/1/y",  0, 100, false, false},
  {"1_dist",  "/1/dist",  0, 100, false, false},
  {"1_speed", "/1/speed", 0, 100, false, false},
  {"1_angle",  "/1/angle",  0, 100, false, false},
  {"2_x", "/2/x", 0, 100, false, false},
  {"2_y",  "/2/y",  0, 100, false, false},
  {"2_dist",  "/2/dist",  0, 100, false, false},
  {"2_speed", "/2/speed", 0, 100, false, false},
  {"2_angle",  "/2/angle",  0, 100, false, false},
  {"3_x", "/3/x", 0, 100, false, false},
  {"3_y",  "/3/y",  0, 100, false, false},
  {"3_dist",  "/3/dist",  0, 100, false, false},
  {"3_speed", "/3/speed", 0, 100, false, false},
  {"3_angle",  "/3/angle",  0, 100, false, false}
};

// variables liées à la gestion du radar
uint32_t lastRadarReading = 0;
bool is_radar_connected = false;

struct radarData {
  bool detected;
  bool reallyDetected;
  int16_t x;
  int16_t y;
  int16_t distance;
  int16_t speed;
  int16_t angle;
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
int16_t xForMean[3][sizeMean];
int16_t yForMean[3][sizeMean];
int16_t distForMean[3][sizeMean];
int16_t speedForMean[3][sizeMean];
int16_t angleForMean[3][sizeMean];

// UDP pour les messages OSC
WiFiUDP Udp;


// void oscParamCallback(Control *sender, int type) {
//   if (type == B_UP) return;

//   for (int i = 0; i < NUM_PARAMS; i++) {
//     if (sender->id == (10 + i)) { // bouton test
//       IPAddress outIP;
//       outIP.fromString(ipAddress->getString());
//       OSCMessage msg(oscParams[i].address->getString().c_str());
//       float val = ((oscParams[i].minVal->getInt() + oscParams[i].maxVal->getInt()) / 2.0) / 100.0;
//       msg.add(val);
//       Udp.beginPacket(outIP, tdPort->getInt());
//       msg.send(Udp);
//       Udp.endPacket();
//       msg.empty();
//       Serial.printf("Sent test OSC: %s = %.2f\n", oscParams[i].address->getString().c_str(), val);
//     }
//   }
// }


// ====== SETUP UI ======
void setupUI() {
  uint16_t generalTab = ESPUI.addControl(ControlType::Tab, "Network", "Network");
  uint16_t oscTab = ESPUI.addControl(ControlType::Tab, "OSC data parameters", "OSC data parameters");
  uint16_t radarTab = ESPUI.addControl(ControlType::Tab, "Radar control", "Radar control");
  
  // general tab
  espuiID = new PersistentValue("id", ControlColor::Wetasphalt, 0, 0, 4, generalTab);
  isStarted = new PersistentValue("Start", ControlColor::Alizarin, false, generalTab);
  String baseIP = "192.168.1.108";
  ipAddress = new PersistentValue("IP destination", ControlColor::Peterriver, baseIP, generalTab);
  abletonPort = new PersistentValue("ableton port", ControlColor::Wetasphalt, 1000, 8001, 12000, generalTab);
  tdPort = new PersistentValue("td port", ControlColor::Wetasphalt, 1000, 9001, 12000, generalTab);

  // osc tab
  for (int i = 0; i < NUM_PARAMS; i++) {
    ESPUI.addControl(ControlType::Separator, baseOscParams[i].name.c_str(), baseOscParams[i].name.c_str(), ControlColor::Turquoise, oscTab);
    // oscParams[i].name = new PersistentValue(baseOscParams[i].name, ControlColor::Peterriver, baseOscParams[i].name, oscTab);
    String label = baseOscParams[i].name;
    // uint16_t group = ESPUI.addControl(Label, label.c_str(), "", None, oscTab);
    oscParams[i].address = new PersistentValue(label+"_address", ControlColor::Peterriver, baseOscParams[i].address, oscTab);
    oscParams[i].minVal = new PersistentValue(label+"_min (%)", ControlColor::Wetasphalt, 0, 0, 100, oscTab);
    oscParams[i].maxVal = new PersistentValue(label+"_max (%)", ControlColor::Wetasphalt, 0, 100, 100, oscTab);
    // --- Switchs persistants ---
    oscParams[i].sendToTD = new PersistentValue(label+"_TD", ControlColor::Alizarin, baseOscParams[i].sendToTD, oscTab);
    oscParams[i].sendToAbleton = new PersistentValue(label+"_AB", ControlColor::Alizarin, baseOscParams[i].sendToAbleton, oscTab);

    // ESPUI.addControl(Button, "Tester", "Send", ControlColor::Peterriver, oscTab, oscParamCallback);
  }

  // radar tab
  readingFrequency = new PersistentValue("fréquence de lecture du radar", ControlColor::Wetasphalt, 100, 200, 2000, radarTab);
  maxDist = new PersistentValue("max distance", ControlColor::Wetasphalt, 7, 0, 7, radarTab);
  inactivityTimer = new PersistentValue("inactivity timer", ControlColor::Wetasphalt, 5, 0, 10, radarTab);
}


int16_t rollingAverage(int16_t dataArray[sizeMean], int16_t data){
  // calcule la moyenne glissante sur x données (défini par sizeMean)
  int16_t somme = 0;
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
  radarsData[detectedID].detected = detected;
  radarsData[detectedID].x = x*detected;
  radarsData[detectedID].y = y*detected;
  radarsData[detectedID].distance = distance*detected;
  radarsData[detectedID].speed = speed*detected;
  radarsData[detectedID].angle = angle*detected;
}

void sendRadarData(uint8_t detectedID, int16_t valuesArray[5]){
  // envoie les valeurs du radar
  // Serial.println("sending data...");
  IPAddress outIP;
  outIP.fromString(ipAddress->getString());
  for(int i=detectedID*5+1; i<(detectedID*5+1+NUM_PARAMS); i++){
    OSCMessage msg(oscParams[i-1].address->getString().c_str());
    if(!oscParams[i-1].sendToAbleton->getBool() && !oscParams[i-1].sendToTD->getBool()){ continue;}
    msg.add(valuesArray[(i-1)%5]);
    if(oscParams[i-1].sendToAbleton->getBool()){
      Udp.beginPacket(outIP, abletonPort->getInt());
      msg.send(Udp);
      Udp.endPacket();
      msg.empty();
    }
    if(oscParams[i-1].sendToTD->getBool()){
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
      xForMean[j][i] = 0;
      yForMean[j][i] = 0;
      distForMean[j][i] = 0;
      speedForMean[j][i] = 0;
      angleForMean[j][i] = 0;
    }
  }

  delay(2000);
  Serial.println("Starting programm..");
}

void loop(){
  uint8_t id = espuiID->getInt();

  if(radar.update() && (millis() - lastRadarReading) > 1/readingFrequency->getInt()){
    lastRadarReading = millis();
    for (int i = 0; i < MAX_DETECT; i++) {
      radarsData[i] = {};
      RadarTarget t = radar.getTarget(i);
      bool realData = t.x != 0.0 && abs(t.x) < maxDist->getInt()*1000;
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
        // Serial.printf("Target %d: X=%d mm  Y=%d mm  Speed=%d mm/s  Dist=%.1f  Angle=%.1f°\n",
        //               i, (int)t.x, (int)t.y, (int)t.speed, t.distance, t.angle);
      }
    }
    qsort(radarsData, MAX_DETECT, sizeof(struct radarData), compareByDist);

    for(int j=0; j<MAX_DETECT; j++){
      if(radarsData[j].x != 0.0){
        countMessages[j] = countMessages[j] + 1;
        goodCountMessages[j] = countMessages[j];
        radarsData[j].reallyDetected = countMessages[j] >= 4;
        if(isStarted->getBool() && radarsData[j].reallyDetected){
          // Serial.printf("Good target %d: X=%d mm  Y=%d mm  Speed=%d mm/s  Dist=%d  Angle=%d°\n",
          //            j, (int)radarsData[j].x, (int)radarsData[j].y, (int)radarsData[j].speed, 
          //            radarsData[j].distance, radarsData[j].angle);
          int16_t dataToSend[5] = {radarsData[j].x, radarsData[j].y, radarsData[j].distance, 
                                  radarsData[j].speed, radarsData[j].angle};
          sendRadarData(j, dataToSend);
        }
      }
      else {
        if(countMessages[j] > 0 && (goodCountMessages[j]-countMessages[j]) < 5){countMessages[j] = countMessages[j] - 1;}
        else {countMessages[j] = 0;}
      }
    }
  }
  delay(20);
}
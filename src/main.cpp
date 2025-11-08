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

#define NUM_PARAMS 3


// OSC parameters
String ip = "192.168.1.108";
uint16_t tdPort = 8001;
uint16_t abletonPort = 9001;

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
  {"volume", "/volume", 0, 100, false, true},
  {"pitch",  "/pitch",  0, 100, true, false},
  {"speed",  "/speed",  0, 100, true, true}
};

// variables liées à la gestion du radar
uint32_t last_radar_reading = 0;
bool is_radar_connected = false;

struct Radar_data {
  bool detected;
  uint16_t x;
  uint16_t y;
  uint16_t distance;
  uint16_t speed;
  uint16_t angle;
};

Radar_data radars_data[3];
RadarSensor radar(RADAR_TX_PIN, RADAR_RX_PIN);

//persistentvalues pour les valeurs de contrôle de ESPUI
PersistentValue* isStarted;

PersistentValue* ip_address;
PersistentValue* ableton_port;
PersistentValue* td_port;

PersistentValue* moving_max_distance;
uint8_t old_mov_max_dist = 7;
PersistentValue* stationary_max_distance;
uint8_t old_stat_max_dist = 7;
PersistentValue* inactivity_timer;
PersistentValue* reading_frequency;
PersistentValue* espui_id;


// initialise les valeurs de moyenne glissante
const uint8_t size_mean = 10;
uint16_t x_for_mean[3][size_mean];
uint16_t y_for_mean[3][size_mean];
uint16_t dist_for_mean[3][size_mean];
uint16_t speed_for_mean[3][size_mean];
uint16_t angle_for_mean[3][size_mean];

// UDP pour les messages OSC
WiFiUDP Udp;


void oscParamCallback(Control *sender, int type) {
  if (type == B_UP) return;

  for (int i = 0; i < NUM_PARAMS; i++) {
    if (sender->id == (10 + i)) { // bouton test
      IPAddress outIP;
      outIP.fromString(ip_address->getString());
      OSCMessage msg(oscParams[i].address->getString().c_str());
      float val = ((oscParams[i].minVal->getInt() + oscParams[i].maxVal->getInt()) / 2.0) / 100.0;
      msg.add(val);
      Udp.beginPacket(outIP, td_port->getInt());
      msg.send(Udp);
      Udp.endPacket();
      msg.empty();
      Serial.printf("Sent test OSC: %s = %.2f\n", oscParams[i].address->getString().c_str(), val);
    }
  }
}


// ====== SETUP UI ======
void setupUI() {
  uint16_t generalTab = ESPUI.addControl(ControlType::Tab, "Network", "Network");
  uint16_t oscTab = ESPUI.addControl(ControlType::Tab, "OSC data parameters", "OSC data parameters");
  uint16_t radarTab = ESPUI.addControl(ControlType::Tab, "Radar control", "Radar control");
  
  // general tab
  espui_id = new PersistentValue("id", ControlColor::Wetasphalt, 0, 0, 4, generalTab);
  isStarted = new PersistentValue("Start", ControlColor::Alizarin, false, generalTab);
  String baseIP = "192.168.1.108";
  ip_address = new PersistentValue("IP destination", ControlColor::Peterriver, baseIP, generalTab);
  ableton_port = new PersistentValue("ableton port", ControlColor::Wetasphalt, 1000, 8001, 12000, generalTab);
  td_port = new PersistentValue("td port", ControlColor::Wetasphalt, 1000, 9001, 12000, generalTab);

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

    ESPUI.addControl(Button, "Tester", "Send", ControlColor::Peterriver, oscTab, oscParamCallback);
  }

  // radar tab
  reading_frequency = new PersistentValue("fréquence de lecture du radar", ControlColor::Wetasphalt, 100, 1000, 2000, radarTab);
  moving_max_distance = new PersistentValue("moving max distance", ControlColor::Wetasphalt, 7, 0, 7, radarTab);
  stationary_max_distance = new PersistentValue("stationary max distance", ControlColor::Wetasphalt, 7, 0, 7, radarTab);
  inactivity_timer = new PersistentValue("inactivity timer", ControlColor::Wetasphalt, 5, 0, 10, radarTab);
  old_mov_max_dist = moving_max_distance->getInt();
  old_stat_max_dist = stationary_max_distance->getInt();

}


uint16_t moyenne_glissante(uint16_t data_array[size_mean], uint16_t data){
  // calcule la moyenne glissante sur x données (défini par size_mean)
  uint16_t somme = 0;
  for (int i=1; i<size_mean; i++){
    data_array[i-1] = data_array[i];
    somme += data_array[i-1];
  }
  data_array[size_mean-1] = data;
  somme += data_array[size_mean-1];
  return somme/size_mean;
}

void updateRadarData(uint8_t detected_id, uint16_t detected, uint16_t x, uint8_t y, uint16_t distance, uint16_t speed, uint8_t angle){
  // met à jour les valeurs du radar d'un boitier avec son id et les nouvelles valeurs
  radars_data[detected_id].detected = detected;
  radars_data[detected_id].x = x*detected;
  radars_data[detected_id].y = y*detected;
  radars_data[detected_id].distance = distance*detected;
  radars_data[detected_id].speed = speed*detected;
  radars_data[detected_id].angle = angle*detected;
}

void sendRadarData(uint8_t detected_id){
  // envoie les valeurs du radar du boitier à tous autres
  IPAddress outIP;
  outIP.fromString(ip_address->getString());
  OSCMessage msg("/broadcast");
  msg.add(detected_id);
  msg.add(radars_data[detected_id].x);
  msg.add(radars_data[detected_id].y);
  msg.add(radars_data[detected_id].distance);
  msg.add(radars_data[detected_id].speed);
  msg.add(radars_data[detected_id].angle);
  Udp.beginPacket(outIP, ableton_port->getInt());
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
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
  for(int j=0; j<3; j++){
    for(int i=0; i<size_mean; i++){
      x_for_mean[j][i] = 0;
      y_for_mean[j][i] = 0;
      dist_for_mean[j][i] = 0;
      speed_for_mean[j][i] = 0;
      angle_for_mean[j][i] = 0;
    }
  }

  delay(2000);
  Serial.println("Starting programm..");
}

void loop(){
  uint8_t id = espui_id->getInt();

  // Serial.println(oscParams[0].sendToAbleton);

  if(radar.update() && millis() - last_radar_reading > 1/reading_frequency->getInt()){
    last_radar_reading = millis();
    for (int i = 0; i < 3; i++) {
      RadarTarget t = radar.getTarget(i);
      if(t.x > 0.0 && t.x < 7000){
        // met à jour les valeurs du radar local avec calcul d'une moyenne glissante
        updateRadarData(i,
          t.detected,
          moyenne_glissante(x_for_mean[i], t.x),
          moyenne_glissante(y_for_mean[i], t.y),
          moyenne_glissante(dist_for_mean[i], t.distance),
          moyenne_glissante(speed_for_mean[i], t.speed),
          moyenne_glissante(angle_for_mean[i], t.angle)
        );
        Serial.printf("Target %d: X=%d mm  Y=%d mm  Speed=%d mm/s  Dist=%.1f  Angle=%.1f°\n",
                      i, (int)t.x, (int)t.y, (int)t.speed, t.distance, t.angle);
        // envoie ces valeurs mises à jour aux autres boitiers
        if(isStarted->getBool())
        sendRadarData(i);
        // debugRadar(id);
      }
    }
  }
  delay(20);
}
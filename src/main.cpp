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

#define OSC_OUT_PORT 2107


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
PersistentValue* moving_max_distance;
uint8_t old_mov_max_dist = 7;
PersistentValue* stationary_max_distance;
uint8_t old_stat_max_dist = 7;
PersistentValue* inactivity_timer;
PersistentValue* motion_sensitivity[8];
PersistentValue* stationary_sensitivity[8];
uint8_t motion_sensitivity_rst[8] = {70, 60, 50, 50, 40, 30, 20, 10};
uint8_t stationary_sensitivity_rst[8] = {50, 50, 40, 40, 30, 30, 20, 20};
uint8_t old_motion_sensitivity[8] = {70, 60, 50, 50, 40, 30, 20, 10};
uint8_t old_stat_sensitivity[8] = {50, 50, 40, 40, 30, 30, 20, 20};
PersistentValue* reading_frequency;
PersistentValue* espui_id;


// initialise les valeurs de moyenne glissante
const uint8_t size_mean = 4;
uint16_t x_for_mean[size_mean];
uint16_t y_for_mean[size_mean];
uint16_t dist_for_mean[size_mean];
uint16_t speed_for_mean[size_mean];
uint16_t angle_for_mean[size_mean];

// UDP pour les messages OSC
WiFiUDP Udp;


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
  OSCMessage msg("/broadcast");
  msg.add(detected_id);
  msg.add(radars_data[detected_id].x);
  msg.add(radars_data[detected_id].y);
  msg.add(radars_data[detected_id].distance);
  msg.add(radars_data[detected_id].speed);
  msg.add(radars_data[detected_id].angle);
  Udp.beginPacket(WiFi.broadcastIP(), OSC_OUT_PORT);
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
  ESPUI.begin("MAGIQUE_MAIF");
  uint16_t general_tab = ESPUI.addControl(ControlType::Tab, "Général", "Général");
  uint16_t fan_tab = ESPUI.addControl(ControlType::Tab, "Contrôle des ventilateurs", "Contrôle des ventilateurs");
  uint16_t radar_tab = ESPUI.addControl(ControlType::Tab, "Contrôle du radar", "Contrôle du radar");
  
  // id du boitier
  espui_id = new PersistentValue("id", ControlColor::Wetasphalt, 0, 0, 4, general_tab);

  // valeurs de contrôle du radar
  reading_frequency = new PersistentValue("fréquence de lecture du radar", ControlColor::Wetasphalt, 100, 1000, 2000, radar_tab);
  moving_max_distance = new PersistentValue("moving max distance", ControlColor::Wetasphalt, 7, 0, 7, radar_tab);
  stationary_max_distance = new PersistentValue("stationary max distance", ControlColor::Wetasphalt, 7, 0, 7, radar_tab);
  inactivity_timer = new PersistentValue("inactivity timer", ControlColor::Wetasphalt, 5, 0, 10, radar_tab);
  old_mov_max_dist = moving_max_distance->get();
  old_stat_max_dist = stationary_max_distance->get();
  ESPUI.addControl(ControlType::Separator, "motion sensitivity", "motion sensitivity", ControlColor::Wetasphalt, radar_tab);
  for(int i=0; i<moving_max_distance->get(); i++){
    motion_sensitivity[i] = new PersistentValue(String(i)+" motion sensitivity", ControlColor::Wetasphalt, motion_sensitivity_rst[i], 0, 100, radar_tab);
    old_motion_sensitivity[i] = motion_sensitivity[i]->get();
  }
  ESPUI.addControl(ControlType::Separator, "stationary sensitivity", "stationary sensitivity", ControlColor::Wetasphalt, radar_tab);
  for(int i=0; i<stationary_max_distance->get(); i++){
    stationary_sensitivity[i] = new PersistentValue(String(i)+" stationary sensitivity", ControlColor::Wetasphalt, stationary_sensitivity_rst[i], 0, 100, radar_tab);
    old_stat_sensitivity[i] = stationary_sensitivity[i]->get();
  }

  // initialise les valeurs pour moyenne glissante
  for(int i=0; i<size_mean; i++){
    x_for_mean[i] = 0;
    y_for_mean[i] = 0;
    dist_for_mean[i] = 0;
    speed_for_mean[i] = 0;
    angle_for_mean[i] = 0;
  }

  delay(2000);
  Serial.println("Starting programm..");
}

void loop(){
  uint8_t id = espui_id->get();

  if(radar.update() && millis() - last_radar_reading > 1/reading_frequency->get()){
    last_radar_reading = millis();
    for (int i = 0; i < 3; i++) {
      RadarTarget t = radar.getTarget(i);
      if(t.x > 0.0 && t.x < 7000){
        // met à jour les valeurs du radar local avec calcul d'une moyenne glissante
        updateRadarData(i,
          t.detected,
          moyenne_glissante(x_for_mean, t.x),
          moyenne_glissante(y_for_mean, t.y),
          moyenne_glissante(dist_for_mean, t.distance),
          moyenne_glissante(speed_for_mean, t.speed),
          moyenne_glissante(angle_for_mean, t.angle)
        );
        Serial.printf("Target %d: X=%d mm  Y=%d mm  Speed=%d mm/s  Dist=%.1f  Angle=%.1f°\n",
                      i, (int)t.x, (int)t.y, (int)t.speed, t.distance, t.angle);
        // envoie ces valeurs mises à jour aux autres boitiers
        sendRadarData(i);
        // debugRadar(id);
      }
    }
  }
  delay(20);
}
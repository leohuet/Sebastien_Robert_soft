#include "Arduino.h"

#include "wifi_config.h"
#include "persistentValue.h"
#include <ESPUI.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

#include "ld2410.h"
#include <HardwareSerial.h>


#define RADAR_RX_PIN D3
#define RADAR_TX_PIN D2

#define OSC_OUT_PORT 2107


// variables liées à la gestion du radar
uint32_t last_radar_reading = 0;
bool is_radar_connected = false;
struct Radar_data {
  bool stationary;
  uint16_t stationary_distance;
  uint16_t stationary_energy;
  bool moving;
  uint16_t moving_distance;
  uint16_t moving_energy;
};

Radar_data radars_data[4];
HardwareSerial SerialPort(1);
ld2410 radar;


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
uint16_t stat_for_mean[size_mean];
uint16_t stat_d_for_mean[size_mean];
uint16_t stat_e_for_mean[size_mean];
uint16_t mov_for_mean[size_mean];
uint16_t mov_d_for_mean[size_mean];
uint16_t mov_e_for_mean[size_mean];

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

void changeRadarParams(){
  // change les paramètres du radar
  Serial.print("max value ");
  uint8_t maxm = moving_max_distance->get();
  uint8_t maxs = stationary_max_distance->get();
  uint16_t inact = inactivity_timer->get();
  Serial.println();
  radar.setMaxValues(maxm, maxs, inact);
  if(radar.setMaxValues(maxm, maxs, inact)){
    Serial.println(F("OK, now restart to apply settings"));
  }
  else{
    Serial.println(F("failed"));
  }
  delay(1000);
  for(int i=0; i<moving_max_distance->get(); i++){
    radar.setGateSensitivityThreshold(i, motion_sensitivity[i]->get(), stationary_sensitivity[i]->get());
    if(radar.setGateSensitivityThreshold(i, motion_sensitivity[i]->get(), stationary_sensitivity[i]->get())){
      Serial.println(F("OK, now restart to apply settings"));
    }
    else{
      Serial.println(F("failed"));
    }
  }

  delay(1000);

  if(radar.requestCurrentConfiguration()){
    Serial.println(F("OK"));
    Serial.print(F("Maximum gate ID: "));
    Serial.println(radar.max_gate);
    Serial.print(F("Maximum gate for moving targets: "));
    Serial.println(radar.max_moving_gate);
    Serial.print(F("Maximum gate for stationary targets: "));
    Serial.println(radar.max_stationary_gate);
    Serial.print(F("Idle time for targets: "));
    Serial.println(radar.sensor_idle_time);
    Serial.println(F("Gate sensitivity"));
    for(uint8_t gate = 0; gate <= radar.max_gate; gate++)
    {
      Serial.print(F("Gate "));
      Serial.print(gate);
      Serial.print(F(" moving targets: "));
      Serial.print(radar.motion_sensitivity[gate]);
      Serial.print(F(" stationary targets: "));
      Serial.println(radar.stationary_sensitivity[gate]);
    }
  }
  else
  {
    Serial.println(F("Failed"));
  }

  radar.requestRestart();

  ESP.restart();
}

void updateRadarData(uint8_t radar_id, uint16_t stat, uint16_t stat_dist, uint8_t stat_energy, uint16_t moving, uint16_t moving_dist, uint8_t moving_energy){
  // met à jour les valeurs du radar d'un boitier avec son id et les nouvelles valeurs
  radars_data[radar_id].stationary = stat;
  radars_data[radar_id].stationary_distance = stat_dist*stat;
  radars_data[radar_id].stationary_energy = stat_energy*stat;
  radars_data[radar_id].moving = moving;
  radars_data[radar_id].moving_distance = moving_dist*moving;
  radars_data[radar_id].moving_energy = moving_energy*moving;
}

void sendRadarData(uint8_t radar_id){
  // envoie les valeurs du radar du boitier à tous autres
  OSCMessage msg("/broadcast");
  msg.add(radar_id);
  msg.add(radars_data[radar_id].stationary);
  msg.add(radars_data[radar_id].stationary_distance);
  msg.add(radars_data[radar_id].stationary_energy);
  msg.add(radars_data[radar_id].moving);
  msg.add(radars_data[radar_id].moving_distance);
  msg.add(radars_data[radar_id].moving_energy);
  Udp.beginPacket(WiFi.broadcastIP(), OSC_OUT_PORT);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}


void debugRadar(uint8_t id){
  Serial.println();
  Serial.print("Radar ");
  Serial.print(id);
  Serial.print(" ");
  Serial.print("is_stationary: ");
  Serial.print(radars_data[id].stationary);
  Serial.print(" stationary_distance: ");
  Serial.print(radars_data[id].stationary_distance);
  Serial.print(" stationary_energy: ");
  Serial.print(radars_data[id].stationary_energy);
  Serial.print(" is_moving: ");
  Serial.print(radars_data[id].moving);
  Serial.print(" moving_distance: ");
  Serial.print(radars_data[id].moving_distance);
  Serial.print(" moving_energy: ");
  Serial.print(radars_data[id].moving_energy);
  Serial.println();
  Serial.println();
  Serial.println();
}


void setup(){

  // ouvre le port série et le port série pour le radar
  Serial.begin(115200); //Feedback over Serial Monitor
  Serial1.begin(256000, SERIAL_8N1, RADAR_TX_PIN, RADAR_RX_PIN); //UART for monitoring the radar
  delay(500);

  pinMode(LED_BUILTIN, OUTPUT);

  begin_wifi();
  delay(2000);

  Serial.print(F("LD2410 radar sensor initialising: "));

  if(radar.begin(Serial1)){
    Serial.println(F("OK"));
    Serial.print(F("LD2410 firmware version: "));
    Serial.print(radar.firmware_major_version);
    Serial.print('.');
    Serial.print(radar.firmware_minor_version);
    Serial.print('.');
    Serial.println(radar.firmware_bugfix_version, HEX);
  }
  else{
    Serial.println(F("not connected"));
  }

  // initialise les controls de ESPUI
  ESPUI.begin("MAGIQUE_MAIF");
  uint16_t general_tab = ESPUI.addControl(ControlType::Tab, "Général", "Général");
  uint16_t fan_tab = ESPUI.addControl(ControlType::Tab, "Contrôle des ventilateurs", "Contrôle des ventilateurs");
  uint16_t radar_tab = ESPUI.addControl(ControlType::Tab, "Contrôle du radar", "Contrôle du radar");
  
  // id du boitier
  espui_id = new PersistentValue("id", ControlColor::Wetasphalt, 0, 0, 4, general_tab);

  // valeurs de contrôle du radar
  reading_frequency = new PersistentValue("fréquence de lecture du radar", ControlColor::Wetasphalt, 1, 1, 2, radar_tab);
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
    stat_d_for_mean[i] = 0;
    stat_e_for_mean[i] = 0;
    mov_d_for_mean[i] = 0;
    mov_e_for_mean[i] = 0;
  }

  // changeRadarParams();
  delay(2000);
  Serial.println("Starting programm..");
}

void loop(){
  uint8_t id = espui_id->get();

  if(moving_max_distance->get() != old_mov_max_dist){
    old_mov_max_dist = moving_max_distance->get();
    changeRadarParams();
    return;
  }
  else if(stationary_max_distance->get() != old_stat_max_dist){
    old_stat_max_dist = stationary_max_distance->get();
    changeRadarParams();
    return;
  }
  for(int i=0; i<stationary_max_distance->get(); i++){
    if(stationary_sensitivity[i]->get() != old_stat_sensitivity[i]){
      old_stat_sensitivity[i] = stationary_sensitivity[i]->get();
      changeRadarParams();
      return;
    }
  }
  for(int i=0; i<moving_max_distance->get(); i++){
    if(motion_sensitivity[i]->get() != old_motion_sensitivity[i]){
      old_motion_sensitivity[i] = motion_sensitivity[i]->get();
      changeRadarParams();
      return;
    }
  }

  // lis les valeurs du radar et récupère les données de présence
  radar.read();

  // si le radar est connecté
  if(radar.isConnected() && millis() - last_radar_reading > 1/reading_frequency->get()*1000){
    last_radar_reading = millis();

    // met à jour les valeurs du radar local avec calcul d'une moyenne glissante
    updateRadarData(id,
      radar.presenceDetected(),
      moyenne_glissante(stat_d_for_mean, radar.stationaryTargetDistance()),
      moyenne_glissante(stat_e_for_mean, radar.stationaryTargetEnergy()),
      radar.movingTargetDetected(),
      moyenne_glissante(mov_d_for_mean, radar.movingTargetDistance()),
      moyenne_glissante(mov_e_for_mean, radar.movingTargetEnergy())
    );
    // envoie ces valeurs mises à jour aux autres boitiers
    sendRadarData(id);
    // debugRadar(id);
  }
  delay(50);
}
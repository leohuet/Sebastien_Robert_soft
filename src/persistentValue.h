#include <Preferences.h>
#include <ESPUI.h>
#include <Math.h>

// min max macro
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))


Preferences preferences;

class PersistentValue {
private:
  static bool initialized;  // Static boolean to keep track of initialization
  String key;  // Key limited to 15 chars
  String label;
  unsigned int value;
  ControlColor color;
  int min_val;
  int max_val;

public:


  PersistentValue(String k, ControlColor c, int defaultVal, int min, int max, uint16_t parentControl)
    : color(c), min_val(min), max_val(max) {
    
    // Initialize Preferences if not already initialized
    if (!initialized) {
      begin();
    }

    label = k;
    // trim key to 15 chars max
    key = k.substring(0, 15);
    
    value = preferences.getUInt(key.c_str(), min(max_val, max(min_val, defaultVal)));
    registerWithESPUI(parentControl);
  }

  static void begin() {
    Serial.println("initiating prefs...");
    preferences.begin("UC1", false);
    initialized = true;
  }

  unsigned int get() const {
    return min(max_val, max(min_val, value));
  }

void registerWithESPUI(uint16_t parentControl) {
    uint16_t controlID = ESPUI.addControl(ControlType::Number, label.c_str(), String(value), color, parentControl,
      [](Control *sender, int eventname, void* UserParameter) {
        if(UserParameter) {
          reinterpret_cast<PersistentValue*>(UserParameter)->callback(sender, eventname);
        }
      },
      this  // <- UserParameter for extended callback
    );

    ESPUI.addControl(ControlType::Min, label.c_str(), String(min_val), ControlColor::None, controlID);
    ESPUI.addControl(ControlType::Max, label.c_str(), String(max_val), ControlColor::None, controlID);
  }

  // Updated callback method
  void callback(Control *sender, int eventname) {

    // Update value based on eventname if necessary
    value = (uint16_t) sender->value.toInt();

    preferences.putUInt(key.c_str(), min(max_val, max(min_val, value)));
  }
};

bool PersistentValue::initialized = false;
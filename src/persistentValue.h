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
  String stringValue;
  unsigned int intValue;
  bool boolValue;
  bool isNumeric;
  bool isBool;
  ControlColor color;
  int min_val;
  int max_val;

public:

  // --- Constructeur pour les nombres ---
  PersistentValue(String k, ControlColor c, int defaultVal, int min, int max, uint16_t parentControl)
    : color(c), min_val(min), max_val(max), isNumeric(true), isBool(false) {
    init(k);
    intValue = preferences.getUInt(key.c_str(), constrain(defaultVal, min, max));
    registerWithESPUI(parentControl);
  }

  // --- Constructeur pour les strings ---
  PersistentValue(String k, ControlColor c, String defaultVal, uint16_t parentControl)
    : color(c), isNumeric(false), isBool(false) {
    init(k);
    stringValue = preferences.getString(key.c_str(), defaultVal);
    registerWithESPUI(parentControl);
  }

  // --- Constructeur pour bool ---
  PersistentValue(String k, ControlColor c, bool defaultVal, uint16_t parentControl)
    : color(c), min_val(0), max_val(1), isNumeric(false), isBool(true) {
    init(k);
    boolValue = preferences.getBool(key.c_str(), defaultVal);
    registerWithESPUI(parentControl);
  }

  static void begin() {
    Serial.println("initiating prefs...");
    preferences.begin("UC1", false);
    initialized = true;
  }

  void init(String k) {
    if (!initialized) begin();
    label = k;
    key = k.substring(0, 15);
  }

  // --- Récupérer la valeur ---
  unsigned int getInt() const { return min(max_val, max(min_val, intValue)); }
  String getString() const { return stringValue; }
  bool getBool() const { return boolValue;}


  // --- Enregistrer dans ESPUI ---
  void registerWithESPUI(uint16_t parentControl) {
    if (isNumeric) {
      uint16_t controlID = ESPUI.addControl(
        ControlType::Number, label.c_str(), String(intValue), color, parentControl,
        [](Control *sender, int eventname, void *UserParameter) {
          if (UserParameter)
            reinterpret_cast<PersistentValue *>(UserParameter)->callback(sender, eventname);
        },
        this
      );

      ESPUI.addControl(ControlType::Min, label.c_str(), String(min_val), ControlColor::None, controlID);
      ESPUI.addControl(ControlType::Max, label.c_str(), String(max_val), ControlColor::None, controlID);

    } 
    else if(isBool) {
      uint16_t controlID = ESPUI.addControl(
      ControlType::Switcher, label.c_str(), String(boolValue), color, parentControl,
        [](Control *sender, int eventname, void* UserParameter) {
          if (UserParameter) reinterpret_cast<PersistentValue*>(UserParameter)->callback(sender, eventname);
        },
        this
      );
    }
    else {
      uint16_t controlID = ESPUI.addControl(
        ControlType::Text, label.c_str(), stringValue.c_str(), color, parentControl,
        [](Control *sender, int eventname, void *UserParameter) {
          if (UserParameter)
            reinterpret_cast<PersistentValue *>(UserParameter)->callback(sender, eventname);
        },
        this
      );
    }
  }

  // --- Callback ---
  void callback(Control *sender, int eventname) {
    if (isNumeric) {
      intValue = constrain(sender->value.toInt(), min_val, max_val);
      preferences.putUInt(key.c_str(), intValue);
    }
    else if(isBool) {
      boolValue = sender->value.toInt() ? 1 : 0;
      preferences.putBool(key.c_str(), boolValue);
    }
    else {
      stringValue = sender->value;
      preferences.putString(key.c_str(), stringValue);
    }
  }
};

bool PersistentValue::initialized = false;
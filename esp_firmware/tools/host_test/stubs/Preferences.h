// Заглушка NVS: значения хранятся в памяти процесса (общие для всех экземпляров, как во flash)
#pragma once
#include "Arduino.h"
#include <map>
#include <string>
class Preferences {
public:
  bool begin(const char *ns, bool = false) { _ns = ns; return true; }
  void end() {}
  bool isKey(const char *k) { return store().count(_ns + "/" + k) > 0; }
  float getFloat(const char *k, float def = 0.0f) {
    auto it = store().find(_ns + "/" + k);
    return it == store().end() ? def : it->second;
  }
  void putFloat(const char *k, float v) { store()[_ns + "/" + k] = v; }
  void putBool(const char *k, bool v) { store()[_ns + "/" + k] = v ? 1.0f : 0.0f; }
  int32_t getInt(const char *k, int32_t def = 0) {
    auto it = store().find(_ns + "/" + k);
    return it == store().end() ? def : (int32_t)it->second;
  }
  void putInt(const char *k, int32_t v) { store()[_ns + "/" + k] = (float)v; }
private:
  std::string _ns;
  static std::map<std::string, float> &store() { static std::map<std::string, float> m; return m; }
};

#pragma once
#include <esphome/core/defines.h>
#ifdef USE_SENSOR

#include <map>
#include <cmath>
#include <string>

#include "const.h"
#include "hap_entity.h"

#include <esphome/core/application.h>
#include <hap.h>
#include <hap_apple_chars.h>
#include <hap_apple_servs.h>

namespace esphome {
namespace homekit {

class SensorEntity : public HAPEntity {
 private:
  static constexpr const char *TAG = "SensorEntity";
  static constexpr float CO2_DETECTED_THRESHOLD_PPM = 1200.0f;

  sensor::Sensor *sensorPtr;

  static inline bool is_integer_float(float v) {
    return std::isfinite(v) && (std::ceil(v) == v);
  }

  static inline void set_numeric_char_value(hap_char_t *ch, float v) {
    if (ch == nullptr) return;
    hap_val_t state{};
    if (is_integer_float(v)) {
      state.u = static_cast<uint32_t>(v);
    } else {
      state.f = v;
    }
    hap_char_update_val(ch, &state);
  }

  static void update_co2_detected_if_needed(hap_serv_t *service, sensor::Sensor *obj, float v) {
    if (service == nullptr || obj == nullptr) return;
    if (obj->get_device_class() != "carbon_dioxide") return;

    hap_char_t *det = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_CARBON_DIOXIDE_DETECTED);
    if (det == nullptr) return;

    hap_val_t dv{};
    dv.b = (std::isfinite(v) && v >= CO2_DETECTED_THRESHOLD_PPM);
    hap_char_update_val(det, &dv);
    ESP_LOGD(TAG, "CO2 detected state: %s (%.1f ppm, threshold %.1f)",
             dv.b ? "ON" : "OFF", v, CO2_DETECTED_THRESHOLD_PPM);
  }

  static void on_sensor_update(sensor::Sensor *obj, float v) {
    ESP_LOGD(TAG, "%s value: %.2f", obj->get_name().c_str(), v);

    hap_acc_t *acc =
        hap_acc_get_by_aid(hap_get_unique_aid(std::to_string(obj->get_object_id_hash()).c_str()));
    if (!acc) return;

    hap_serv_t *hs = hap_serv_get_next(hap_acc_get_first_serv(acc));
    if (!hs) return;

    hap_char_t *value_char = hap_serv_get_first_char(hs);
    if (value_char == nullptr) return;

    auto *cur = hap_char_get_val(value_char);
    ESP_LOGD(TAG, "HAP CURRENT VALUE - f:%.2f u:%lu", cur->f, cur->u);

    set_numeric_char_value(value_char, v);
    update_co2_detected_if_needed(hs, obj, v);
  }

  static int sensor_read(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv) {
    (void) status_code;
    (void) read_priv;

    if (!serv_priv) return HAP_FAIL;

    auto *sensorPtr = static_cast<sensor::Sensor *>(serv_priv);
    ESP_LOGD(TAG, "Read called for Accessory %s (%s)",
             std::to_string(sensorPtr->get_object_id_hash()).c_str(),
             sensorPtr->get_name().c_str());

    const float v = sensorPtr->get_state();
    set_numeric_char_value(hc, v);

    hap_serv_t *service = hap_char_get_service(hc);
    update_co2_detected_if_needed(service, sensorPtr, v);

    return HAP_SUCCESS;
  }

  static int acc_identify(hap_acc_t *ha) {
    (void) ha;
    ESP_LOGI(TAG, "Accessory identified");
    return HAP_SUCCESS;
  }

 public:
  explicit SensorEntity(sensor::Sensor *sensorPtr)
      : HAPEntity({{MODEL, "HAP-SENSOR"}}), sensorPtr(sensorPtr) {}

  void setup() {
    hap_serv_t *service = nullptr;

    std::string device_class = sensorPtr->get_device_class();
    if (device_class == "temperature") {
      service = hap_serv_temperature_sensor_create(sensorPtr->state);
    } else if (device_class == "humidity") {
      service = hap_serv_humidity_sensor_create(sensorPtr->state);
    } else if (device_class == "illuminance") {
      service = hap_serv_light_sensor_create(sensorPtr->state);
    } else if (device_class == "aqi") {
      service = hap_serv_air_quality_sensor_create(sensorPtr->state);
    } else if (device_class == "carbon_dioxide") {
      // Initial value false; gets updated dynamically in callbacks/read
      service = hap_serv_carbon_dioxide_sensor_create(false);
    } else if (device_class == "carbon_monoxide") {
      service = hap_serv_carbon_monoxide_sensor_create(false);
    } else if (device_class == "pm10") {
      service = hap_serv_create(HAP_SERV_UUID_AIR_QUALITY_SENSOR);
      hap_serv_add_char(service, hap_char_pm_10_density_create(sensorPtr->state));
    } else if (device_class == "pm25") {
      service = hap_serv_create(HAP_SERV_UUID_AIR_QUALITY_SENSOR);
      hap_serv_add_char(service, hap_char_pm_2_5_density_create(sensorPtr->state));
    }

    if (!service) return;

    hap_acc_cfg_t acc_cfg = {
        .name = nullptr,
        .manufacturer = strdup(accessory_info[MANUFACTURER]),
        .model = strdup(accessory_info[MODEL]),
        .serial_num = nullptr,
        .fw_rev = strdup(accessory_info[FW_REV]),
        .hw_rev = nullptr,
        .pv = strdup("1.1.0"),
        .cid = HAP_CID_BRIDGE,
        .identify_routine = acc_identify,
    };

    std::string accessory_name = sensorPtr->get_name();
    if (accessory_info[NAME] == nullptr) {
      acc_cfg.name = strdup(accessory_name.c_str());
    } else {
      acc_cfg.name = strdup(accessory_info[NAME]);
    }

    if (accessory_info[SN] == nullptr) {
      acc_cfg.serial_num = strdup(std::to_string(sensorPtr->get_object_id_hash()).c_str());
    } else {
      acc_cfg.serial_num = strdup(accessory_info[SN]);
    }

    hap_acc_t *accessory = hap_acc_create(&acc_cfg);

    ESP_LOGD(TAG, "ID HASH: %lu", sensorPtr->get_object_id_hash());
    hap_serv_set_priv(service, sensorPtr);
    hap_serv_set_read_cb(service, sensor_read);

    hap_acc_add_serv(accessory, service);
    hap_add_bridged_accessory(
        accessory,
        hap_get_unique_aid(std::to_string(sensorPtr->get_object_id_hash()).c_str()));

    if (!sensorPtr->is_internal()) {
      sensorPtr->add_on_state_callback([this](float v) { SensorEntity::on_sensor_update(sensorPtr, v); });
    }

    ESP_LOGI(TAG, "Sensor '%s' linked to HomeKit", accessory_name.c_str());
  }
};

}  // namespace homekit
}  // namespace esphome

#endif

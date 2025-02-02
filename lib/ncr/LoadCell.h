#ifndef LOADCELL_H
#define LOADCELL_H

#include <HX711_ADC.h>

class LoadCell {
public:
    LoadCell(String pid_label, uint8_t dout, uint8_t sck, float calibration_value) 
    : pid_label_(pid_label), calibration_value_(calibration_value), driver_(dout, sck) { }

    // Returns a reference to the HX711 driver instance
    HX711_ADC* driver() { return &driver_; }

    void set_calibration_value(float calibration_value) {
        calibration_value_ = calibration_value; 
        driver_.setCalFactor(calibration_value_);
    }

    String pid_label() const { return pid_label_; }
    float calibration_value() const { return calibration_value_; }

private:
    String pid_label_;
    HX711_ADC driver_;
    float calibration_value_;
};

#endif // LOADCELL_H
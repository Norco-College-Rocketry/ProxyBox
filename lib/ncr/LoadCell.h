#ifndef LOADCELL_H
#define LOADCELL_H

#include <HX711_ADC.h>

struct LoadCell {
    String pid_label;
    HX711_ADC driver;
    float calibration_value;
};

#endif // LOADCELL_H
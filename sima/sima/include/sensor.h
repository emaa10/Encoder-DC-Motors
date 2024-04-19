#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

int readToF(){
    int last_range;

    VL53L0X_RangingMeasurementData_t measure;
    
    Serial.print("Reading a measurement... ");
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    
    if (measure.RangeStatus != 4){ // phase failures have incorrect data
        Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
        last_range = measure.RangeMilliMeter;
    } else{
        Serial.println("-1");
        last_range = -1;
    }
    return last_range;
}
#ifndef DATA_STRUCT_H
#define DATA_STRUCT_H

typedef struct {
    float plate_temp;
    int voltage;
    int batery;
    uint32_t pressure;
    float temperature;
    short gyro_x;
    short gyro_y;
    short gyro_z;
    short accel_x;
    short accel_y;
    short accel_z;
    int pulses;
    
} SensorData;

extern SensorData sensor_data;

#endif

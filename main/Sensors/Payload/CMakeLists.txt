idf_component_register(
    SRCS plate_temperature.c plate_voltage.c pulse_counter.c
    INCLUDE_DIRS .
    REQUIRES ds18b20 driver/gpio freertos
)

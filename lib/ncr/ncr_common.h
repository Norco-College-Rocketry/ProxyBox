#ifndef NCR_COMMON_H
#define NCR_COMMON_H

#define UART_BAUD 460800
#define CAN_BAUD 500000

typedef enum {
    TELEMETRY_PACKET = 0x10,
    LED_COMMAND_PACKET = 0x21,
    VALVE_COMMAND_PACKET = 0x22,
    PT_CALIBRATION_COMMAND_PACKET = 0x23,
    SENSOR_MODE_COMMAND_PACKET = 0x24,
} PACKET_TYPE;

typedef enum {
    CLOSED = 0,
    OPEN = 1
} ValvePosition;

typedef struct {
    uint16_t id;
    ValvePosition position;    
} ValveCommand;

#endif // NCR_COMMON_H
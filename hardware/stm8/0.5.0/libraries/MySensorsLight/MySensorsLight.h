#ifndef MYSENSORSLIGHT_H
#define MYSENSORSLIGHT_H
#include "MyMessage.h"

bool MySensors_init(uint8_t address);
void MySensors_process();
void MySensors_setParentNodeId(uint8_t parentNodeId);
bool MySensors_send(MyMessage *message, const bool requestEcho);
bool MySensors_sendBatteryLevel(const uint8_t value, const bool requestEcho);
bool MySensors_sendHeartbeat(const bool requestEcho);
bool MySensors_present(const uint8_t childSensorId, const mysensors_sensor_t sensorType,
                       const char *description,
                       const bool requestEcho);
bool MySensors_sendSketchInfo(const char *name, const char *version, const bool requestEcho);
bool MySensors_request(const uint8_t childSensorId, const uint8_t variableType, const uint8_t destination);
bool MySensors_requestTime(const bool requestEcho);

// weak functions, but weak is not supported by sdcc
void MySensors_receive(const MyMessage*) /* __attribute__((weak)) */;
void MySensors_receiveTime(uint32_t) /* __attribute__((weak)) */;
void MySensors_presentation(void) /* __attribute__((weak)) */;



#endif // MYSENSORSLIGHT_H

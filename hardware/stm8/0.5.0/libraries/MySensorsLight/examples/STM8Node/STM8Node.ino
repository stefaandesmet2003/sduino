/* 
 * A demo MySensors node with NRF24L01+ radio running on STM8
 * Radio connection : 
 * CE : PC4
 * CS : SS (A3)
 * SCK : PC5
 * MOSI : PC6
 * MISO : PC7
 * 
 */
#include "MySensorsLight.h"

#define MY_NODE_ID 100
#define CHILD_ID 0   // Id of the sensor child

// SDCC doesn't know weak functions
void MySensors_receiveTime(uint32_t controllerTime){(void) controllerTime;}
void MySensors_receive(const MyMessage *message){(void) message;}

MyMessage msg; // a message for the gateway

void setup()
{
  bool ok;
  pinMode (LED_BUILTIN,OUTPUT);
  MyMessage_init2(&msg,CHILD_ID,V_STATUS); // init msg structure
  ok = MySensors_init(MY_NODE_ID);
  digitalWrite(LED_BUILTIN, !ok);
}

void MySensors_presentation()
{
  // Send the sketch version information to the gateway and controller
  MySensors_sendSketchInfo("STM8 node", "1.0",false);

  // Register all sensors to gw (they will be created as child devices)
  MySensors_present(CHILD_ID, S_BINARY,"aToggle", false);
}
bool toggle = false;
uint32_t toggleMillis;

void loop()
{
  MySensors_process(); // call this as much as possible
  if ((millis() - toggleMillis) > 5000) {
    toggleMillis = millis();
    toggle = !toggle;
    digitalWrite(LED_BUILTIN,toggle);
    MySensors_send (MyMessage_setByte(&msg,toggle),false);
  }
}

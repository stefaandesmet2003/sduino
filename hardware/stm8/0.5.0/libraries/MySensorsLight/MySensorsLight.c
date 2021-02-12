#include "Arduino.h"
#include "RF24.h"
#include "MySensorsLight.h"

#define GATEWAY_ADDRESS       ((uint8_t)0)		//!< Node ID for GW sketch
#define NODE_SENSOR_ID        ((uint8_t)255)	//!< Node child is always created/presented when a node is started
#define BROADCAST_ADDRESS     (255u)			//!< broadcasts are addressed to ID 255
#define MYSENSORS_LIBRARY_VERSION "2.3.2"
#define MYSENSORS_LIBRARY_VERSION_INT (0x020302FF)
#define MAX_SUBSEQ_MSGS 5

static MyMessage _msg; // voor de receive polling
static MyMessage _msgTmp;

static uint8_t MYS_NodeId;
static uint8_t MYS_ParentNodeId = GATEWAY_ADDRESS;

static void presentNode(void);
static bool _sendRoute(MyMessage *message);
static bool _processInternalCoreMessage(void);
static bool transportHALReceive(MyMessage *inMsg, uint8_t *msgLength);
static void transportProcessMessage(void);

// Inline function and macros
// remove inline to save flash
static /*inline*/ MyMessage* build(MyMessage *msg, const uint8_t destination, const uint8_t sensor,
                               const mysensors_command_t command, const uint8_t type, const bool requestEcho)
{
	MyMessage_setSender(msg,MYS_NodeId);
	MyMessage_setDestination(msg,destination);
	MyMessage_setSensor(msg,sensor);
	MyMessage_setType(msg,type);
	MyMessage_setCommand(msg,command);
	MyMessage_setRequestEcho(msg,requestEcho);
	MyMessage_setEcho(msg,false);
	return msg;
}

static void presentNode(void)
{
	// Present node and request config
	(void)MySensors_present(NODE_SENSOR_ID, S_ARDUINO_NODE,"",false);

	// Send a configuration exchange request to controller
	// Node sends parent node. Controller answers with latest node configuration
	//(void)_sendRoute(MyMessage_setByte(build(&_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL,
	//                       I_CONFIG,false),MYS_ParentNodeId));

	// Wait configuration reply.
  // TODO
	//(void)wait3(2000, C_INTERNAL, I_CONFIG);

	MySensors_presentation();
} // presentNode

// Message delivered through _msg
static bool _processInternalCoreMessage(void)
{
	const uint8_t type = MyMessage_getType(&_msg);
	if (MyMessage_getSender(&_msg) == GATEWAY_ADDRESS) {
		if (type == I_REBOOT) {
      // TODO!
			//hwReboot();
		} else if (type == I_CONFIG) {
      // TODO : Pick up configuration from controller (currently only metric/imperial) and store it in eeprom if changed
      // metric if _msg.data[0] == 0x00 || _msg.data[0] == 'M';
		} else if (type == I_PRESENTATION) {
			// Re-send node presentation to controller
			presentNode();
		} else if (type == I_HEARTBEAT_REQUEST) {
			(void)MySensors_sendHeartbeat(false);
		} else if (type == I_VERSION) {
			(void)_sendRoute(MyMessage_setULong(build(&_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL,
			                       I_VERSION,false),MYSENSORS_LIBRARY_VERSION_INT));
		} else if (type == I_TIME) {
			// Deliver time to callback
			MySensors_receiveTime(MyMessage_getULong(&_msg));
		} else {
			return false; // further processing required
		}
	} else {
		// sender is a node
    return false; // sds - not handled here, of moeten we hier gewoon true zetten?
	}
	return true; // if not GW or no further processing required
} // _processInternalCoreMessage

static bool transportHALReceive(MyMessage *inMsg, uint8_t *msgLength)
{
	// set pointer to first byte of data structure
	uint8_t *rx_data;
	uint8_t payloadLength;

  rx_data = &inMsg->last;
  payloadLength = RF24_readMessage((void *)rx_data);
	// Reject messages with incorrect protocol version
  // TODO : what was the point of this tmp variable?? (-> removed here)
	if (!MyMessage_isProtocolVersionValid(inMsg)) {
		return false;
	}
	*msgLength = MyMessage_getLength(inMsg);
	// Reject payloads with incorrect length
	const uint8_t expectedMessageLength = MyMessage_getExpectedMessageSize(inMsg);
	if (payloadLength != expectedMessageLength) {
		return false;
	}
	return true;
} // transportHALReceive

static void transportProcessMessage() {
	uint8_t payloadLength;
	// last is the first byte of the payload buffer
	if (!transportHALReceive(&_msg, &payloadLength)) {
		return;
	}
	// get message length and limit size
	const uint8_t msgLength = MyMessage_getLength(&_msg);
	// calculate expected length

	const uint8_t command = MyMessage_getCommand(&_msg);
	const uint8_t type = MyMessage_getType(&_msg);
	const uint8_t sender = MyMessage_getSender(&_msg);
	const uint8_t last = MyMessage_getLast(&_msg);
	const uint8_t destination = MyMessage_getDestination(&_msg);

	// Is message addressed to this node?
	if (destination == MYS_NodeId) { // _transportConfig.nodeId : TODO variable maken
		// null terminate data
		_msg.data[msgLength] = 0u;
		// Check if sender requests an echo.
		if (MyMessage_getRequestEcho(&_msg)) {
			_msgTmp = _msg;	// Copy message
			// Reply without echo flag (otherwise we would end up in an eternal loop)
			MyMessage_setRequestEcho(&_msgTmp,false);
			MyMessage_setEcho(&_msgTmp,true); // set ECHO flag
			MyMessage_setSender(&_msgTmp,MYS_NodeId);
			MyMessage_setDestination(&_msgTmp,sender);
			// send ECHO, use transportSendRoute since ECHO reply is not internal, i.e. if !transportOK do not reply
			(void)_sendRoute(&_msgTmp);
		}
		if(!MyMessage_isEcho(&_msg)) {
			// only process if not ECHO
			if (command == C_INTERNAL) {
				// general
				if (type == I_PING) {
					(void)_sendRoute(MyMessage_setByte(build(&_msgTmp, sender, NODE_SENSOR_ID, C_INTERNAL,
					                                  I_PONG,false),1));
					return; // no further processing required
				}
        // no precessing required for other types
        // I_PONG, I_SIGNAL_REPORT_REVERSE, I_SIGNAL_REPORT_REQUEST
				if (_processInternalCoreMessage()) {
					return; // no further processing required
				}
			}
		}
		// Call incoming message callback if available
		MySensors_receive(&_msg);
		
	} else if (destination == BROADCAST_ADDRESS) { // broadcast msg
		if (command == C_INTERNAL) {
			if (type == I_DISCOVER_REQUEST) {
				if (last == MYS_ParentNodeId) {
					// random wait to minimize collisions
					delay(millis() & 0x3ff);
					(void)_sendRoute(MyMessage_setByte(build(&_msgTmp, sender, NODE_SENSOR_ID, C_INTERNAL,
					                                  I_DISCOVER_RESPONSE,false),MYS_ParentNodeId));
				}
			}
		}

		// Callback for BC, only for non-internal messages
		if (command != C_INTERNAL) {
			// only proceed if message received from parent
			if (last != MYS_ParentNodeId) {
				return;
			}
			MySensors_receive(&_msg);
		}
	} // broadcast msg
} // transportProcessMessage

static bool _sendRoute(MyMessage *message) {
	const uint8_t destination = MyMessage_getDestination(message);
	const uint8_t totalMsgLength = HEADER_SIZE + MyMessage_getLength(message);
	const bool noACK = (destination == BROADCAST_ADDRESS);
	uint8_t route;

	MyMessage_setLast(message, MYS_NodeId); // Update last

	if (destination == GATEWAY_ADDRESS) {
		route = MYS_ParentNodeId;		// message to GW always routes via parent
	} else if (destination == BROADCAST_ADDRESS) {
		route = BROADCAST_ADDRESS;		// message to BC does not require routing
	} else {
		if (destination > GATEWAY_ADDRESS && destination < BROADCAST_ADDRESS) {
			// node2node traffic: assume node is in vincinity. If transmission fails, hand over to parent
      if (RF24_sendMessage(destination, message, totalMsgLength, noACK)) {
				return true;
			}
		}
		route = MYS_ParentNodeId;	// not a repeater, all traffic routed via parent
	}
	// send message
  return RF24_sendMessage(route,message, totalMsgLength, noACK);
} // _sendRoute

// public interface
bool MySensors_init(uint8_t address) {
	MyMessage_clear(&_msg);
	MyMessage_clear(&_msgTmp);

  MYS_NodeId = address;
  bool ok;
  ok = RF24_initialize();
  RF24_setNodeAddress(MYS_NodeId);
	RF24_startListening();

  //todo : MY_NODE_ID in eeprom opslaan

  // sanity check : wordt dat gebruikt?
  //return RF24_sanityCheck();
  return ok;

  // go to sleep / powerdown
	//RF24_sleep();
	//RF24_standBy();
	//RF24_powerDown();
	//RF24_powerUp();
} // MYS_init

// fix configuration to keep it simple
// used for routing, if parentNodeId != gateway
void MySensors_setParentNodeId(uint8_t parentNodeId) {
  MYS_ParentNodeId = parentNodeId;
}

void MySensors_process() {
	uint8_t _processedMessages = MAX_SUBSEQ_MSGS;
	// process all msgs in FIFO or counter exit
	while (RF24_isDataAvailable() && _processedMessages--) {
    transportProcessMessage();
	}
} // MYS_process

bool MySensors_send(MyMessage *message, const bool requestEcho)
{
	MyMessage_setSender(message, MYS_NodeId);
	MyMessage_setCommand(message,C_SET);
	MyMessage_setRequestEcho(message,requestEcho);
	return _sendRoute(message);
}

bool MySensors_sendBatteryLevel(const uint8_t value, const bool requestEcho)
{
	return _sendRoute(MyMessage_setByte(build(&_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL, I_BATTERY_LEVEL,
	                        requestEcho),value));
}

bool MySensors_sendHeartbeat(const bool requestEcho)
{
	const uint32_t heartbeat = millis();
	return _sendRoute(MyMessage_setULong(build(&_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL, I_HEARTBEAT_RESPONSE,
	                        requestEcho),heartbeat));
}

bool MySensors_present(const uint8_t childSensorId, const mysensors_sensor_t sensorType,
             const char *description,
             const bool requestEcho)
{
	return _sendRoute(MyMessage_setString(build(&_msgTmp, GATEWAY_ADDRESS, childSensorId, C_PRESENTATION,
	                        (uint8_t)(sensorType),
	                        requestEcho),childSensorId == NODE_SENSOR_ID ? MYSENSORS_LIBRARY_VERSION : description));
}

bool MySensors_sendSketchInfo(const char *name, const char *version, const bool requestEcho)
{
	bool result = true;
	if (name) {
		result &= _sendRoute(MyMessage_setString(build(&_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL, I_SKETCH_NAME,
		                           requestEcho),name));
	}
	if (version) {
		result &= _sendRoute(MyMessage_setString(build(&_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL, I_SKETCH_VERSION,
		                           requestEcho),version));
	}
	return result;
}

bool MySensors_request(const uint8_t childSensorId, const uint8_t variableType, const uint8_t destination)
{
	return _sendRoute(MyMessage_setString(build(&_msgTmp, destination, childSensorId, C_REQ, variableType,false),""));
}

bool MySensors_requestTime(const bool requestEcho)
{
	return _sendRoute(MyMessage_setString(build(&_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL, I_TIME,
	                        requestEcho),""));
}

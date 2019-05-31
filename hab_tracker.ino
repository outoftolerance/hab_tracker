#include <Servo.h>

#include <Timer.h>
#include <Log.h>
#include <SimpleHDLC.h>
#include <TinyGPS++.h>

#define PAN_SERVO 2
#define TILT_SERVO 3

Servo pan_servo;	/*< Pan servo object */
Servo tilt_servo;	/*< Tilt servo object */

Stream& logging_output_stream = Serial;             /**< Logging output stream, this is of type Serial_ */
Stream& command_input_stream = Serial;              /**< Message and command interface stream */

enum MESSAGE_TYPES {
    MESSAGE_TYPE_REPORT_TRACKER_POINT,

    MESSAGE_TYPE_COMMAND_SET_TRACKER_LOCATION,
    MESSAGE_TYPE_COMMAND_SET_TARGET_POSITION,

    MESSAGE_TYPE_COMMAND_REQUEST_REPORT,

    MESSAGE_TYPE_PROTO_ACK
};

/**
 * @brief      Callback function handles new messages from HDLC
 *
 * @param[in]  message  The message to be handled
 */
void handleMessageCallback(hdlcMessage message);

SimpleHDLC usb(command_input_stream, &handleMessageCallback);        /**< HDLC messaging object, linked to message callback */
Log logger(logging_output_stream, LOG_LEVELS::DEBUG);                /**< Log object */

/**
 * @brief System setup function
 * @details Initialises all system componenets at start-up
 */
void setup() {
    //Sleep 5s so that debug can connect
    delay(5000);

    //Set servo pins
    pan_servo.attach(PAN_SERVO);
    tilt_servo.attach(TILT_SERVO);

    //Start serial port to command PC
    Serial.begin(57600);
}

/**
 * @brief Main program loop
 * @details Called after setup() function, loops inifiteley, everything happens here
 */
void loop() {
	while(1)
	{
		//Get messages from serial port

		//Check if base location is set yet

			//Calculate bearing and azimuth to target

			//Get current tilt

			//Move tilt until at azimuth

			//Get current bearing

			//Move pan until at bearing
	}
}

float azimuthTo(float base_altitude, float target_horizontal_distance, float target_altitude)
{
	float altitude_difference = target_altitude - base_altitude;

	return arctan(altitude_difference, target_horizontal_distance);
}

void handleMessageCallback(hdlcMessage message)
{
    logger.event(LOG_LEVELS::INFO, "Received a message!");

    switch(message.command)
    {
        case MESSAGE_TYPES::MESSAGE_TYPE_REPORT_TELEMETRY:
            logger.event(LOG_LEVELS::INFO, "Received telemetry report.");
            handleMessageTelemetryReport(message);
            break;
        case MESSAGE_TYPES::MESSAGE_TYPE_REPORT_POSITION:
            logger.event(LOG_LEVELS::INFO, "Received position report.");
            handleMessagePositionReport(message);
            break;
        case MESSAGE_TYPES::MESSAGE_TYPE_COMMAND_ARM:
            logger.event(LOG_LEVELS::INFO, "Received takeoff command.");
            handleMessageCommandArm(message);
            break;
        case MESSAGE_TYPES::MESSAGE_TYPE_COMMAND_DISARM:
            logger.event(LOG_LEVELS::INFO, "Received abort takeoff message.");
            handleMessageCommandDisarm(message);
            break;
        case MESSAGE_TYPES::MESSAGE_TYPE_COMMAND_SET_STATE:
            logger.event(LOG_LEVELS::INFO, "Received set state command.");
            handleMessageCommandSetState(message);
            break;
        case MESSAGE_TYPES::MESSAGE_TYPE_PROTO_ACK:
            logger.event(LOG_LEVELS::INFO, "Received acknowledgement.");
            handleMessageProtoAck(message);
            break;
    }
}

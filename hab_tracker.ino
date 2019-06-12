#include <Wire.h>
#include <Math.h>

#include <Adafruit_PWMServoDriver.h>
#include <Timer.h>
#include <Log.h>
#include <SimpleHDLC.h>
#include <TinyGPS++.h>

#define PAN_SERVO 0
#define TILT_SERVO 1

#define SERVO_PWM_FREQUENCY 60  // Analog servos run at ~60 Hz updates

#define PAN_SERVO_PWM_MIN  150  // this is the 'minimum' pulse length count (out of 4096)
#define PAN_SERVO_PWM_MAX  500  // this is the 'maximum' pulse length count (out of 4096)
#define TILT_SERVO_PWM_MIN  258 // this is the 'minimum' pulse length count (out of 4096)
#define TILT_SERVO_PWM_MAX  480 // this is the 'maximum' pulse length count (out of 4096)

Stream& logging_output_stream = Serial;                     /**< Logging output stream, this is of type Serial_ */
Stream& command_input_stream = Serial;                      /**< Message and command interface stream */

Adafruit_PWMServoDriver servo_driver = Adafruit_PWMServoDriver();    /**< Adafruit servo driver object */

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

    //Start serial port to command PC
    Serial.begin(57600);

    //Initialize PWM driver
    servo_driver.begin();
    servo_driver.setPWMFreq(SERVO_PWM_FREQUENCY);
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

        servo_driver.setPWM(PAN_SERVO, 0, PAN_SERVO_PWM_MIN);
        servo_driver.setPWM(TILT_SERVO, 0, TILT_SERVO_PWM_MIN);

        delay(1000);

        servo_driver.setPWM(PAN_SERVO, 0, PAN_SERVO_PWM_MAX);
        servo_driver.setPWM(TILT_SERVO, 0, TILT_SERVO_PWM_MAX);

        delay(1000);
	}
}

float azimuthTo(float base_altitude, float target_horizontal_distance, float target_altitude)
{
	float altitude_difference = target_altitude - base_altitude;

	return atan2(altitude_difference, target_horizontal_distance);
}

void handleMessageCallback(hdlcMessage message)
{
    logger.event(LOG_LEVELS::INFO, "Received a message!");

    switch(message.command)
    {
        case MESSAGE_TYPES::MESSAGE_TYPE_PROTO_ACK:
            logger.event(LOG_LEVELS::INFO, "Received acknowledgement.");
            //handleMessageProtoAck(message);
            break;
    }
}

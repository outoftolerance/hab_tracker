#include <Wire.h>
#include <Math.h>

#include <Adafruit_PWMServoDriver.h>
#include <Timer.h>
#include <Log.h>
#include <Telemetry.h>
#include <SimpleHDLC.h>
#include <SimpleMessageProtocol.h>
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

/**
 * @brief      Callback function handles new messages from HDLC
 *
 * @param[in]  message  The message to be handled
 */
void handleMessageCallback(hdlcMessage message);

SimpleHDLC usb(command_input_stream, &handleMessageCallback);        /**< HDLC messaging object, linked to message callback */
Log logger(logging_output_stream, LOG_LEVELS::DEBUG);                /**< Log object */
Telemetry telemetry(IMU_TYPES::IMU_TYPE_ADAFRUIT_9DOF);              /**< Telemetry object */

Timer timer_execution_led;            /**< Timer sets interval between run led blinks */
Timer timer_telemetry_check;          /**< Timer sets interval between telemetry checks */

/**
 * @brief System setup function
 * @details Initialises all system componenets at start-up
 */
void setup() {
    //Sleep 5s so that debug can connect
    delay(5000);

    //Setup pin modes
    pinMode(LED_BUILTIN, OUTPUT);

    //Start logger
    logger.init();
    logger.event(LOG_LEVELS::INFO, "HAB Tracker systems starting...");

    //Start command interface over USB
    logger.event(LOG_LEVELS::INFO, "Starting USB serial interface...");
    //static_cast<HardwareSerial&>(command_input_stream).begin(57600);

    //Initialise the telemetry system
    logger.event(LOG_LEVELS::INFO, "Initialising Telemetry subsystem...");
    if(!telemetry.init())
    {
        logger.event(LOG_LEVELS::FATAL, "Failed to initialise Telemetry subsystem!");
        while(1);
    }

    //Initialize PWM driver
    logger.event(LOG_LEVELS::INFO, "Starting PWM Driver...");
    servo_driver.begin();
    servo_driver.setPWMFreq(SERVO_PWM_FREQUENCY);
}

/**
 * @brief Main program loop
 * @details Called after setup() function, loops inifiteley, everything happens here
 */
void loop() {
    timer_execution_led.setInterval(500);                   /**< Sets execution blinky LED interval to 500ms */
    timer_execution_led.start();
    timer_telemetry_check.setInterval(100);
    timer_telemetry_check.start();
    TelemetryStruct current_telemetry;                      /**< Current telemetry */

	while(1)
	{
		//Get messages from command interface
        usb.receive();

        //Telemetry Update
        if(timer_telemetry_check.check())
        {
            //Get latest telemetry
            logger.event(LOG_LEVELS::DEBUG, "Getting update from Telemetry subsystem.");

            if(!telemetry.get(current_telemetry))
            {
                logger.event(LOG_LEVELS::ERROR, "Failed to get update from Telemetry subsystem!");
            }
            else
            {
                logger.event(LOG_LEVELS::DEBUG, "Telemetry updated completed.");
                timer_telemetry_check.reset();
            }
        }

		//Check if base location is set yet

			//Calculate bearing and azimuth to target
            //target_bearing = ;
            //target_azimuth = azimuthTo(tracker.pose.altitude, target.);

			//Get current tilt

			//Move tilt until at azimuth

			//Get current bearing

			//Move pan until at bearing

        //Execution LED indicator blinkies
        if(timer_execution_led.check())
        {
            if(digitalRead(LED_BUILTIN) == HIGH)
            {
                digitalWrite(LED_BUILTIN, LOW);
            }
            else
            {
                digitalWrite(LED_BUILTIN, HIGH);
            }
        }

        //Print a bunch of debug information
        logger.event(LOG_LEVELS::DEBUG, "Current GPS Latitude   ", current_telemetry.latitude);
        logger.event(LOG_LEVELS::DEBUG, "Current GPS Longitude  ", current_telemetry.longitude);
        logger.event(LOG_LEVELS::DEBUG, "Current GPS Altitude   ", current_telemetry.altitude);
        logger.event(LOG_LEVELS::DEBUG, "Current GPS Course     ", current_telemetry.course);
        logger.event(LOG_LEVELS::DEBUG, "Current IMU Roll       ", current_telemetry.roll);
        logger.event(LOG_LEVELS::DEBUG, "Current IMU Pitch      ", current_telemetry.pitch);
        logger.event(LOG_LEVELS::DEBUG, "Current IMU Heading    ", current_telemetry.heading);
        logger.event(LOG_LEVELS::DEBUG, "Current IMU Altitude   ", current_telemetry.altitude_barometric);
        logger.event(LOG_LEVELS::DEBUG, "Current IMU Pressure   ", current_telemetry.pressure);
        logger.event(LOG_LEVELS::DEBUG, "Current IMU Temperature", current_telemetry.temperature);

        delay(500);
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

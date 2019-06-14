#include <Wire.h>
#include <Math.h>

#include <Adafruit_PWMServoDriver.h>
#include <Timer.h>
#include <Log.h>
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

Timer timer_execution_led;            /**< Timer sets intercal between run led blinks */

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
    logger.event(LOG_LEVELS::INFO, "HAB systems starting...");

    //Start command interface over USB
    logger.event(LOG_LEVELS::INFO, "Starting USB serial interface...");
    static_cast<HardwareSerial&>(radio_input_output_stream).begin(57600);

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

	while(1)
	{
		//Get messages from command interface
        usb.receive();

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

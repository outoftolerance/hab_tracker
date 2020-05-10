#include <Wire.h>
#include <Math.h>
#include <PID_v1.h>

#include <Adafruit_PWMServoDriver.h>
#include <TinyGPS++.h>
#include <SimpleServo.h>
#include <Timer.h>
#include <Log.h>
#include <Telemetry.h>
#include <SimpleHDLC.h>
#include <SimpleMessageProtocol.h>
#include <TinyGPS++.h>

#define PAN_SERVO_CHANNEL 5
#define TILT_SERVO_CHANNEL 4
#define GPS_FIX_STATUS 9

#define PAN_SERVO_PWM_MIN  150  // this is the 'minimum' pulse length count (out of 4096) Left
#define PAN_SERVO_PWM_MAX  600  // this is the 'maximum' pulse length count (out of 4096) Right
#define TILT_SERVO_PWM_MIN  260 // Vertical
#define TILT_SERVO_PWM_MAX  470 // Horizontal

Stream& logging_output_stream = Serial;                     /**< Logging output stream, this is of type Serial_ */
Stream& command_input_stream = Serial;                      /**< Message and command interface stream */
Stream& gps_input_stream = Serial1;                         /**< GPS device input stream, this is of type HardwareSerial */

Adafruit_PWMServoDriver servo_driver = Adafruit_PWMServoDriver();    /**< Adafruit servo driver object */
SimpleServo tilt_servo(TILT_SERVO_PWM_MAX, TILT_SERVO_PWM_MIN, TILT_SERVO_CHANNEL, &servo_driver);
SimpleServo pan_servo(PAN_SERVO_PWM_MIN, PAN_SERVO_PWM_MAX, PAN_SERVO_CHANNEL, &servo_driver);

SimpleHDLC usb(command_input_stream, &handleMessageCallback);                               /**< HDLC messaging object, linked to message callback */
Log logger(logging_output_stream, LOG_LEVELS::INFO);                                        /**< Log object */
Telemetry telemetry(IMU_TYPES::IMU_TYPE_ADAFRUIT_9DOF, &gps_input_stream, GPS_FIX_STATUS);  /**< Telemetry object */
uint8_t node_id_ = 2;
uint8_t node_type_ = NODE_TYPES::NODE_TYPE_TRACKER;

uint8_t target_node_id = 1;
Telemetry::TelemetryStruct target_location;

Timer timer_execution_led;            /**< Timer sets interval between run led blinks */
Timer timer_telemetry_check;          /**< Timer sets interval between telemetry checks */
Timer timer_servo_update_delay;       /**< Timer sets interval between position updates */

/**
 * @brief System setup function
 * @details Initialises all system componenets at start-up
 */
void setup() {
    //Sleep until debug can connect
    while(!Serial);

    //Setup pin modes
    pinMode(LED_BUILTIN, OUTPUT);

    //Start logger
    logger.init();
    logger.event(LOG_LEVELS::INFO, "HAB Tracker systems starting...");

    //Start command interface over USB
    logger.event(LOG_LEVELS::INFO, "Starting USB serial interface...");
    //static_cast<HardwareSerial&>(command_input_stream).begin(57600);
    logger.event(LOG_LEVELS::INFO, "Done!");

    //Initialise the telemetry system
    logger.event(LOG_LEVELS::INFO, "Initialising Telemetry subsystem...");
    if(!telemetry.init())
    {
        logger.event(LOG_LEVELS::FATAL, "Failed to initialise Telemetry subsystem!");
        stop();
    }
    logger.event(LOG_LEVELS::INFO, "Done!");

    //Initialize PWM driver and servos
    logger.event(LOG_LEVELS::INFO, "Starting PWM Driver...");
    servo_driver.begin();
    servo_driver.setPWMFreq(SERVO_PWM_FREQUENCY);
    logger.event(LOG_LEVELS::INFO, "Done!");

    logger.event(LOG_LEVELS::INFO, "Finished initialisation, starting program!");
}

/**
 * @brief Main program loop
 * @details Called after setup() function, loops inifiteley, everything happens here
 */
void loop() {
    timer_execution_led.setInterval(1000);
    timer_telemetry_check.setInterval(5);
    
    Telemetry::TelemetryStruct current_telemetry;                      /**< Current telemetry */

    timer_execution_led.start();
    timer_telemetry_check.start();
    timer_servo_update_delay.start();

    double tilt_setpoint, pan_setpoint;

    tilt_servo.start();
    pan_servo.start();

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

		//Calculate bearing and azimuth to target
        tilt_setpoint = calcAzimuthTo(current_telemetry.altitude, target_location.altitude, TinyGPSPlus::distanceBetween(current_telemetry.latitude, current_telemetry.longitude, target_location.latitude, target_location.longitude));
        pan_setpoint = TinyGPSPlus::courseTo(current_telemetry.latitude, current_telemetry.longitude, target_location.latitude, target_location.longitude);

		//Move servos until at target
        tilt_servo.setTargetAngle(tilt_setpoint);
        tilt_servo.move();
        
        pan_servo.setTargetAngle(pan_setpoint);
        pan_servo.move();

        //Update current tracker pose
        tracker_location.pitch = tilt_servo.getCurrentAngle();
        tracker_location.heading = pan_servo.getCurrentAngle();

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

            //Print a bunch of debug information
            logger.event(LOG_LEVELS::DEBUG, "Current GPS Latitude    ", current_telemetry.latitude);
            logger.event(LOG_LEVELS::DEBUG, "Current GPS Longitude   ", current_telemetry.longitude);
            logger.event(LOG_LEVELS::DEBUG, "Current GPS Altitude    ", current_telemetry.altitude);
            logger.event(LOG_LEVELS::DEBUG, "Current GPS Elevation   ", current_telemetry.elevation);
            logger.event(LOG_LEVELS::DEBUG, "Current GPS Azimuth     ", current_telemetry.azimuth);
            logger.event(LOG_LEVELS::DEBUG, "Current GPS Course      ", current_telemetry.course);
            logger.event(LOG_LEVELS::DEBUG, "Current IMU Roll        ", current_telemetry.roll);
            logger.event(LOG_LEVELS::DEBUG, "Current IMU Pitch       ", current_telemetry.pitch);
            logger.event(LOG_LEVELS::DEBUG, "Current IMU Heading     ", current_telemetry.heading);
            logger.event(LOG_LEVELS::DEBUG, "Current IMU Altitude    ", current_telemetry.altitude_barometric);
            logger.event(LOG_LEVELS::DEBUG, "Current IMU Pressure    ", current_telemetry.pressure);
            logger.event(LOG_LEVELS::DEBUG, "Current IMU Temperature ", current_telemetry.temperature);

            timer_execution_led.reset();
        }
	}
}

float calcAzimuthTo(float base_altitude, float target_horizontal_distance, float target_altitude)
{
	float altitude_difference = target_altitude - base_altitude;

	return atan2(altitude_difference, target_horizontal_distance) * 180 / M_PI;
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
        case MESSAGE_TYPES::MESSAGE_TYPE_PROTO_NACK:
            logger.event(LOG_LEVELS::INFO, "Received non-acknowledgement.");
            //handleMessageProtoNack(message);
            break;
        case MESSAGE_TYPES::MESSAGE_TYPE_REPORT_TELEMETRY:
            logger.event(LOG_LEVELS::INFO, "Received message: Position Report.");
            handleMessageTelemetryReport(message);
            break;
        case MESSAGE_TYPES::MESSAGE_TYPE_COMMAND_SET_TRACKER_TARGET_LOCATION:
            logger.event(LOG_LEVELS::INFO, "Received a command to set tracker target.");
            handleMessageSetTrackerTargetLocation(message);
            break;
        case MESSAGE_TYPES::MESSAGE_TYPE_COMMAND_SET_TRACKER_LOCATION:
            logger.event(LOG_LEVELS::INFO, "Received a command to set tracker location.");
            handleMessageSetTrackerLocation(message);
            break;
        case MESSAGE_TYPES::MESSAGE_TYPE_COMMAND_SET_TRACKER_POSE:
            logger.event(LOG_LEVELS::INFO, "Received a command to set tracker pose.");
            handleMessageSetTrackerPose(message);
            break;
        case MESSAGE_TYPES::MESSAGE_TYPE_COMMAND_REQUEST_REPORT:
            logger.event(LOG_LEVELS::INFO, "Received a command to request a report.");
            handleMessageRequestReport(message);
            break;
    }
}

void handleMessageTelemetryReport(hdlcMessage& message)
{
    //Is this a telemetry message from the target?
    if(message.node_id == target_node_id)
    {
        //Decode and parse for target location
        smpMessageReportTelemetry report;
        smpMessageReportTelemetryDecode(message, report);

        target_location.latitude = report.latitude;
        target_location.longitude = report.longitude;
        target_location.altitude = report.altitude;
    }
    else
    {
        logger.event(LOG_LEVELS::INFO, "Ignoring telemetry report message.");
    }
}

void handleMessageSetTrackerTargetLocation(hdlcMessage message)
{
    target_location.latitude = 0;
    target_location.longitude = 0;
    target_location.altitude = 0;
}

void handleMessageSetTrackerPose(hdlcMessage message)
{
    
}

void handleMessageRequestReport(hdlcMessage message)
{
    
}

void sendReportTelemetry(Telemetry::TelemetryStruct& telemetry)
{
    hdlcMessage message;
    smpMessageReportTelemetry telemetry_report;

    telemetry_report.latitude.value = telemetry.latitude;
    telemetry_report.longitude.value = telemetry.longitude;
    telemetry_report.altitude.value = telemetry.altitude;
    telemetry_report.altitude_ellipsoid.value = telemetry.altitude_ellipsoid;
    telemetry_report.altitude_relative.value = telemetry.altitude_relative;
    telemetry_report.altitude_barometric.value = telemetry.altitude_barometric;
    telemetry_report.elevation.value = telemetry.elevation;
    telemetry_report.azimuth.value = telemetry.azimuth;
    telemetry_report.gps_snr.value = telemetry.gps_snr;
    telemetry_report.velocity_horizontal.value = telemetry.velocity_horizontal;
    telemetry_report.velocity_vertical.value = telemetry.velocity_vertical;
    telemetry_report.roll.value = telemetry.roll;
    telemetry_report.pitch.value = telemetry.pitch;
    telemetry_report.heading.value = telemetry.heading;
    telemetry_report.course.value = telemetry.course;
    telemetry_report.temperature.value = telemetry.temperature;
    telemetry_report.pressure.value = telemetry.pressure;

    smpMessageReportTelemetryEncode(node_id_, node_type_, telemetry_report, message);

    usb.send(message);
}

void stop()
{
    while(1)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50); 
    }
}

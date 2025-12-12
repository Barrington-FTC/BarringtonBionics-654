package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

    /**
     * A utility class for the REV Robotics Color Sensor V3.
     * This class simplifies initialization and provides methods to check for specific colors.
     */
    public class ColorSensorV3 {

        private ColorSensor sensor;
        private Telemetry telemetry;
        // --- COLOR THRESHOLDS ---
        // These values will need to be tuned for your specific lighting conditions.
        // Use the color sensor telemetry in your main OpMode to find the right values.

        // Thresholds for GREEN
        private static final int GREEN_RED_MAX = 360;
        private static final int GREEN_GREEN_MIN = 1100;
        private static final int GREEN_BLUE_MAX = 925;

        // Thresholds for PURPLE
        private static final int PURPLE_RED_MIN = 250;
        private static final int PURPLE_GREEN_MAX = 1100;
        private static final int PURPLE_BLUE_MIN = 500;


        /**
         * Constructor to initialize the color sensor.
         * @param hardwareMap The hardware map from your OpMode.
         * @param telemetry The telemetry object from your OpMode to display data.
         * @param deviceName The name of the sensor in your robot's configuration (e.g., "colorSensor").
         */
        public ColorSensorV3(HardwareMap hardwareMap, Telemetry telemetry, String deviceName) {
            this.sensor = hardwareMap.get(ColorSensor.class, deviceName);
            this.telemetry = telemetry;
            // You can normalize the color values to a 0-255 range if needed, but raw values are often fine.
            // sensor.normalize();
        }

        /**
         * Checks if the color currently detected by the sensor is green.
         * @return true if the color is green, false otherwise.
         */
        public boolean isGreen() {
            return (sensor.red() < GREEN_RED_MAX &&
                    sensor.green() > GREEN_GREEN_MIN &&
                    sensor.blue() < GREEN_BLUE_MAX);
        }

        /**
         * Checks if the color currently detected by the sensor is purple.
         * @return true if the color is purple, false otherwise.
         */
        public boolean isPurple() {
            return (sensor.red() > PURPLE_RED_MIN &&
                    sensor.green() < PURPLE_GREEN_MAX &&
                    sensor.blue() > PURPLE_BLUE_MIN);
        }

        /**
         * Displays the raw Red, Green, and Blue values from the sensor on the Driver Hub.
         * This is very useful for tuning your color threshold constants.
         */
        public void addTelemetry() {
            telemetry.addData("Color Sensor", "---");
            telemetry.addData("Red", sensor.red());
            telemetry.addData("Green", sensor.green());
            telemetry.addData("Blue", sensor.blue());
            telemetry.addData("Is Green?", isGreen());
            telemetry.addData("Is Purple?", isPurple());
        }
    }

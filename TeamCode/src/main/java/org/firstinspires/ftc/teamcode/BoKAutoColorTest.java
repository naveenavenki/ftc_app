package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Krishna Saxena on 9/28/2016.
 */
public class BoKAutoColorTest implements BokAutoTest {
    // Constants
    private static final int RED_THRESHOLD = 30;
    private static final int BLUE_THRESHOLD = 15;
    private static final float LEFT_MOTOR_POWER = 0.2f;
    private static final float RIGHT_MOTOR_POWER = 0.2f;

    public void runTest(BoKAuto opMode, BoKHardwareBot robot) throws InterruptedException {
        runToRed(opMode, robot);
        runToBlue(opMode, robot);
    }

    private void runToRed(BoKAuto opMode, BoKHardwareBot robot) throws InterruptedException {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToMotors(LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER);

            // run until the end of the match (driver presses STOP)
            while (opMode.opModeIsActive()) {
                // go to red
                int current_red = robot.colorSensor.red();
                int current_blue = robot.colorSensor.blue();
                int current_green = robot.colorSensor.green();
                while (current_red < RED_THRESHOLD) {
                    // Display the color info on the driver station
                    opMode.telemetry.addData("r: ", current_red + " b: " + current_blue + " g: " + current_green);
                    opMode.telemetry.update();

                    // Allow time for other processes to run.
                    opMode.idle();

                    current_red = robot.colorSensor.red();
                    current_blue = robot.colorSensor.blue();
                    current_green = robot.colorSensor.green();
                } // while (current_red < RED_THRESHOLD)

                robot.setPowerToMotors(0.0f, 0.0f);
                // Allow time for other processes to run.
                opMode.idle();
            } // while (opModeIsActive())
        } // if (opModeIsActive())
    }

    private void runToBlue(BoKAuto opMode, BoKHardwareBot robot) throws InterruptedException
    {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setPowerToMotors(-LEFT_MOTOR_POWER, -RIGHT_MOTOR_POWER);

            // run until the end of the match (driver presses STOP)
            while (opMode.opModeIsActive()) {
                // back to blue
                int current_red = robot.colorSensor.red();
                int current_blue = robot.colorSensor.blue();
                int current_green = robot.colorSensor.green();
                while (current_blue < BLUE_THRESHOLD) {
                    // Display the color info on the driver station
                    opMode.telemetry.addData("r: ", current_red + " b: " + current_blue + " g: " + current_green);
                    opMode.telemetry.update();

                    // Allow time for other processes to run.
                    opMode.idle();

                    current_red = robot.colorSensor.red();
                    current_blue = robot.colorSensor.blue();
                    current_green = robot.colorSensor.green();
                } // while (current_blue < BLUE_THRESHOLD)

                robot.setPowerToMotors(0.0f, 0.0f);

                // Pause for metronome tick.  40 mS each cycle = update 25 times a second
                //robot.waitForTick(40);
                opMode.idle(); // Always call idle() at the bottom of your while(opModeIsActive(
            } // while (opModeIsActive())
        } // if (opModeIsActive())
    }
}

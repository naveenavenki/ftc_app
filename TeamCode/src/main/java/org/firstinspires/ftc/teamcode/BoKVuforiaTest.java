package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Krishna Saxena on 10/5/2016.
 */
public class BoKVuforiaTest implements BoKAuto {
    private VuforiaTrackables beacons;

    @Override
    public void initSoftware(LinearOpMode opMode, BoKHardwareBot robot, BoKAlliance redOrBlue) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.beacon_color_detect_view); //new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = "ASodJvL/////AAAAGa9Isk5Oa0brtCRz7Z0fQngHmf2Selfx3RDk3MzjmK9DFWkQRsg1dH8Q8VvU/9L9nr9krXa+2nY5zoK6moC4UpRIg+gRsnG5M504q50Dd+z8DDATBaamc8t5qa8OeLjFKQ/+blHLbe8tjXSdVdl/xxGdowpeuQ18dnlf129q5NjM7Z9s/M8l693yEl28b+/LLJ4SiFLBTXwkEpVblemfJKZVHO5I8JmGmQ4jcwCWFIMCFxPRbCeDVqdCeqQzFa3BcCiuuGUgZDBCaidiW0/pzEFzdXcVCQfJPMgdZUWkPAk0QXVC8zYXaweeLuAONyTDkanRiyzqZbDVpJhVHaLBsUaC3OmZ/Xo+ThguyX3tNs3G";

        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");

        /** Start tracking the data sets we care about. */
        beacons.activate();
    }

    @Override
    public void runSoftware(LinearOpMode opMode, BoKHardwareBot robot)
    {
        opMode.telemetry.addData("Status", "Run to white");
        opMode.idle();
        runToWhite(opMode, robot);
        opMode.telemetry.addData("Status", "turn till pic");
        opMode.idle();
        turnTillPicIsVisible(opMode, robot);
        while (opMode.opModeIsActive()) {

            for (VuforiaTrackable beac : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)beac.getListener()).getPose();
                if (pose != null) {
                    VectorF translation = pose.getTranslation();
                    opMode.telemetry.addData(beac.getName() + "-Translation: ", translation);
                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), translation.get(2)));
                    opMode.telemetry.addData(beac.getName() + "-Degrees: ", degreesToTurn);
                }

            }
            opMode.telemetry.update();
            opMode.idle();

        }
    }

    private void runToWhite(LinearOpMode opMode, BoKHardwareBot robot) {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToDTMotors(LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER);

            // run until the end of the match (driver presses STOP)
            while (opMode.opModeIsActive()) {
                // go to red
                int current_red = robot.colorSensor.red();
                int current_blue = robot.colorSensor.blue();
                int current_green = robot.colorSensor.green();
                while ((current_red < 10) &&
                        (current_blue < 10) &&
                        (current_green < 10) &&
                        (opMode.opModeIsActive())) {
                    // Display the color info on the driver station
                    opMode.telemetry.addData("Status", "r: ", current_red + " b: " + current_blue + " g: " + current_green);
                    opMode.telemetry.update();

                    // Allow time for other processes to run.
                    opMode.idle();

                    current_red = robot.colorSensor.red();
                    current_blue = robot.colorSensor.blue();
                    current_green = robot.colorSensor.green();
                } // while (current_red < RED_THRESHOLD)

                robot.setPowerToDTMotors(0.0f, 0.0f);
                // Allow time for other processes to run.
                opMode.idle();
                break;
            } // while (opModeIsActive())
        } // if (opModeIsActive())
    }

    private void turnTillPicIsVisible(LinearOpMode opMode, BoKHardwareBot robot) {
        boolean picIsVisible = false;

        if (opMode.opModeIsActive()) {
            robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToDTMotors(-LEFT_MOTOR_POWER/3.5, RIGHT_MOTOR_POWER/3.5);
            opMode.telemetry.addData("Status: ", "turn till pic 2");

            // run until the end of the match (driver presses STOP)
            while (opMode.opModeIsActive()) {
                for (VuforiaTrackable beac : beacons) {
                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
                    if (pose != null) {
                        VectorF translation = pose.getTranslation();
                        opMode.telemetry.addData(beac.getName() + "-Translation: ", translation);
                        //double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), translation.get(2)));
                        //opMode.telemetry.addData(beac.getName() + "-Degrees: ", degreesToTurn);
                        picIsVisible = true;
                        break;
                    }

                }
                if (picIsVisible == true) {
                    robot.setPowerToDTMotors(0, 0);
                    break;
                }
                opMode.telemetry.update();
                opMode.idle();
            }

            // run until the end of the match (driver presses STOP)
            boolean facingBeacon = false;
            while (opMode.opModeIsActive() && (facingBeacon == false)) {
                for (VuforiaTrackable beac : beacons) {
                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
                    if (pose != null) {
                        VectorF translation = pose.getTranslation();
                        opMode.telemetry.addData(beac.getName() + "-Translation: ", translation);

                        float x = translation.get(0);
                        if ((x > -10) && (x < 10)) {
                            robot.setPowerToDTMotors(0, 0);
                            facingBeacon = true;
                            break;
                        }

                        if (x > 0) {
                            robot.setPowerToDTMotors(+LEFT_MOTOR_POWER/3.5, -RIGHT_MOTOR_POWER/3.5);
                        }
                        else {
                            robot.setPowerToDTMotors(-LEFT_MOTOR_POWER/3.5, +RIGHT_MOTOR_POWER/3.5);
                        }


                        //double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), translation.get(2)));
                        //opMode.telemetry.addData(beac.getName() + "-Degrees: ", degreesToTurn);
                        //picIsVisible = true;
                        break;
                    }
                }
                opMode.telemetry.update();
                opMode.idle();
            }
        }

    }

}

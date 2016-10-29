package org.firstinspires.ftc.teamcode;

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
 * Created by shiv on 10/5/2016.
 */
public class BensonAuto implements BokAutoTest {
    private VuforiaTrackables beacons;
    private static final int RED_THRESHOLD = 5;
    private static final int BLUE_THRESHOLD= 5;
    @Override
    public void initTest(BoKAuto opMode, BoKHardwareBot robot) {

    }

    @Override
    public void runTest(BoKAuto opMode, BoKHardwareBot robot) throws InterruptedException
    {
        opMode.telemetry.addData("Status", "Run to red");
        opMode.idle();
        shootball(opMode, robot);
        runToRedOrBlue(opMode, robot);
    }

    private void runToRedOrBlue(BoKAuto opMode, BoKHardwareBot robot) throws InterruptedException {
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
                while ((current_red < RED_THRESHOLD && current_blue < BLUE_THRESHOLD) && opMode.opModeIsActive()) {
                    // Display the color info on the driver station
                    opMode.telemetry.addData("r: ", current_red + " b: " + current_blue + " g: " + current_green);
                    opMode.telemetry.update();
                    robot.setPowerToSweeper(-1);

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
    private void shootball(BoKAuto opMode, BoKHardwareBot robot) throws InterruptedException {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToMotors(0, 0);

            // run until the end of the match (driver presses STOP)
            while (opMode.opModeIsActive()) {
               robot.setPowerToShooter(1);
                robot.setPowerToSweeper(1);
                Thread.sleep(10000);
                robot.setPowerToMotors(0.0f, 0.0f);
                // Allow time for other processes to run.
                opMode.idle();
            } // while (opModeIsActive())
        } // if (opModeIsActive())
    }

}

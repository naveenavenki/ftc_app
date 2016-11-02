package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
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
public class League0Auto implements BokAutoTest {

    private static final double WAIT_FOR_SEC = 5.0;
    private static final int RED_THRESHOLD  = 5;
    private static final int BLUE_THRESHOLD = 5;
    private ElapsedTime runTime  = new ElapsedTime();

    @Override
    public void initTest(BoKAuto opMode, BoKHardwareBot robot) {
    }

    @Override
    public void runTest(BoKAuto opMode, BoKHardwareBot robot) throws InterruptedException
    {
        shootBall(opMode, robot, WAIT_FOR_SEC);
        runToRedOrBlue(opMode, robot);
    }

    private void runToRedOrBlue(BoKAuto opMode, BoKHardwareBot robot) throws InterruptedException {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToMotors(LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER);
            robot.setPowerToSweeper(-1);

            // run until the end of the match (driver presses STOP)
            while (opMode.opModeIsActive()) {
                // go to red or blue line
                int current_red = robot.colorSensor.red();
                int current_blue = robot.colorSensor.blue();
                //int current_green = robot.colorSensor.green();

                while ((current_red < RED_THRESHOLD && current_blue < BLUE_THRESHOLD) && opMode.opModeIsActive()) {
                    // Display the color info on the driver station
                    // opMode.telemetry.addData("r: ", current_red + " b: " + current_blue + " g: " + current_green);
                    opMode.telemetry.addData("r: ", current_red + " b: " + current_blue);
                    opMode.telemetry.update();

                    current_red = robot.colorSensor.red();
                    current_blue = robot.colorSensor.blue();
                    // current_green = robot.colorSensor.green();
                } // while (current_red < RED_THRESHOLD)

                robot.setPowerToMotors(0.0f, 0.0f); // stop the robot
                break; // done
            } // while (opModeIsActive())
        } // if (opModeIsActive())
    }

    private void shootBall(BoKAuto opMode, BoKHardwareBot robot, double waitForSec) throws InterruptedException {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToMotors(0, 0);
            robot.setPowerToShooter(1);
            robot.setPowerToSweeper(1);
            runTime.reset();

            // run until the end of the match (driver presses STOP)
            while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {
                opMode.telemetry.addData("Ball shooter: ", "%2.1f sec elapsed", runTime.seconds());
                opMode.telemetry.update();
            } // while (opModeIsActive())

            robot.setPowerToShooter(0.0f); // stop the ball shooter
            robot.setPowerToSweeper(0.0f); // stop the sweeper
        } // if (opModeIsActive())
    }

}

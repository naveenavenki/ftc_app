package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import java.util.Arrays;

/**
 * Created by Krishna Saxena on 11/15/2016.
 * Implements the common algorithms used both by BoKAutoBlueBeacon and BoKAutoRedBeacon.
 * Its primary responsibilities include:
 * initSoftware method which
 * 1. initialize Vuforia
 * 2. initialize OpenCV
 * 3. set up servos and DC motors for autonomous
 */
public abstract class BoKMecanumAutoCommon implements BoKAuto {
    protected AppUtil appUtil = AppUtil.getInstance();

    protected BoKAlliance alliance;

    private static final double P_TURN_COEFF = 0.1;
    private static final double HEADING_THRESHOLD = 1;
    private static final double TURN_SPEED_LOW  = 0.16;
    private static final double TURN_SPEED_HIGH = 0.2;

    private CvC capDevice;
    private ElapsedTime runTime  = new ElapsedTime();

    private BaseLoaderCallback loaderCallback = new BaseLoaderCallback(appUtil.getActivity()) {
        @Override
        public void onManagerConnected(int status) {
            super.onManagerConnected(status);
        }
    };


    public void initSoftware(LinearOpMode opMode, BoKHardwareBot robot, BoKAlliance redOrBlue) {

        Log.v("BOK", "Initializing OpenCV");
        // Initialize OpenCV
        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0,
                    appUtil.getActivity(), loaderCallback);
        }
        else {
            loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.initGyro(opMode);
        // let the drive team know if the gyro initialized correctly
        opMode.telemetry.addData("Gyro", robot.gyroSensor.getIntegratedZValue());
        opMode.telemetry.update();
        Log.v("BOK", "Gyro: integrated: " + robot.gyroSensor.getIntegratedZValue());

        // set the initial position (both pointed down)
        robot.pusherLeftServo.setPosition(BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT);
        robot.pusherRightServo.setPosition(BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT);

        robot.shooterServo.setPosition(BoKHardwareBot.INITIAL_SHOOTER_SERVO_POS_AUTO-0.1);
        /*
        robot.clawLockServo.setPosition(BoKHardwareBot.INITIAL_SERVO_POS_CAP_CLAW);
        robot.partLiftGateServo.setPosition(BoKHardwareBot.INITIAL_SERVO_POS_PART_GATE);
        */
        alliance = redOrBlue;

        capDevice =

    }

    @Override
    public abstract void runSoftware(LinearOpMode opMode,
                                     BoKHardwareBot robot);

    // Algorithm to control the speed for shooter motors based on battery level
    protected double getShooterMotorsPowerBasedOnBatteryLevel(BoKHardwareBot robot) {
        double voltage12V = robot.voltageSensor.getVoltage();
        double shooterMotorsPower =  BoKHardwareBot.SHOOTER_MOTORS_POWER_NORMAL ;

        Log.v("BOK", "Battery voltage: " + voltage12V);
        if (voltage12V >= ROBOT_BATTERY_LEVEL_HIGH_THRESHOLD) {
            shooterMotorsPower = BoKHardwareBot.SHOOTER_MOTORS_POWER_NORMAL -
                    SHOOTER_MOTOR_POWER_CHANGE;
        }
        else if (voltage12V < ROBOT_BATTERY_LEVEL_MED_THRESHOLD) {
            shooterMotorsPower = BoKHardwareBot.SHOOTER_MOTORS_POWER_NORMAL +
                    SHOOTER_MOTOR_POWER_CHANGE;
        }
        return shooterMotorsPower;
    }

    // Shoot particles for as long as necessary by turning on the lift and the shooter
    protected void shootBall(LinearOpMode opMode, BoKHardwareBot robot,
                             double shooterMotorsPower,
                             double waitForSec)  {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            robot.setPowerToDTMotors(0, 0, 0, 0); // Do not move the robot
            //robot.partLiftGateServo.setPosition(BoKHardwareBot.FINAL_SERVO_POS_PART_GATE);
            robot.shooterServo.setPosition(BoKHardwareBot.INITIAL_SHOOTER_SERVO_POS_AUTO);
            robot.setPowerToShooterMotors(shooterMotorsPower);
            robot.sweeperMotor.setPower(BoKHardwareBot.SWEEPER_MOTOR_POWER_NORMAL);
            runTime.reset();

            // run until we either press STOP or run out of time
            while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {
                //opMode.telemetry.addData("Ball shooter: ",
                // "%2.1f sec elapsed", runTime.seconds());
                opMode.telemetry.update();
            } // while (opModeIsActive())

            robot.setPowerToShooterMotors(0.0f); // stop the ball shooter
            robot.sweeperMotor.setPower(0.0f); // stop the sweeper
        } // if (opModeIsActive())
    }

    // Algorithm to move forward using encoder sensor on the DC motors on the drive train
    // Note that we use the abstract class BoKHardwareBot to completely separate out the
    // underlying drive train implementation
    protected void moveForward(LinearOpMode opMode, BoKHardwareBot robot,
                               double leftPower, double rightPower,  double inchesForward,
                               double waitForSec) {
        double degreesOfWheelTurn, degreesOfMotorTurn, targetEncCount;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            degreesOfWheelTurn = (360.0 / (Math.PI * BoK4MotorsDTBot.WHEEL_DIAMETER_INCHES)) *
                    inchesForward;
            degreesOfMotorTurn = BoK4MotorsDTBot.DRIVE_GEAR_REDUCTION * degreesOfWheelTurn;
            targetEncCount = (BoK4MotorsDTBot.COUNTS_PER_MOTOR_REV * degreesOfMotorTurn) / 360.0;

            robot.setDTMotorEncoderTarget((int) -targetEncCount, (int) targetEncCount);
            robot.setPowerToDTMotors(leftPower, leftPower, -rightPower, -rightPower);

            runTime.reset();
            while (opMode.opModeIsActive() &&
                    (robot.getDTCurrentPosition() == false) &&
                    (runTime.seconds() < waitForSec)) {
                //opMode.telemetry.update();
                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
            }

            // Stop all motion;
            robot.setPowerToDTMotors(0, 0, 0, 0);
        }
    }

    // Algorithm to run to white line using the Optical Distance Sensor
    // Again, note that we use the abstract class BoKHardwareBot to completely separate out the
    // underlying drive train implementation
    protected boolean runToWhiteSideways(LinearOpMode opMode, BoKHardwareBot robot,
                                 double waitForSec) {
        double current_alpha = 0, distance = 0;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            //robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToDTMotors(+LEFT_MOTOR_POWER/2, -LEFT_MOTOR_POWER/2,
                                     +RIGHT_MOTOR_POWER/2, -RIGHT_MOTOR_POWER/2);
            opMode.idle();

            // go to white line
            current_alpha = robot.odsSensor.getLightDetected();
            //distance = robot.rangeSensorLeft.cmUltrasonic();
            runTime.reset();

            Log.v("BOK", "AlphaW " + String.format("%.2f", current_alpha) + " d: " + distance);
            while (opMode.opModeIsActive() &&
                    (current_alpha < WHITE_LINE) && (runTime.seconds() < waitForSec)) {

                //distance = robot.rangeSensorLeft.cmUltrasonic();
                current_alpha = robot.odsSensor.getLightDetected();

                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
                Log.v("BOK", "ALPHAW " + String.format("%.2f", current_alpha));
                // distance + " sec: " + String.format("%.2f", runTime.seconds()));
            } // while (current_alpha < WHITE_LINE)

            robot.setPowerToDTMotors(0.0f, 0.0f, 0.0f, 0.0f); // stop the robot
            opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_LONG);

            current_alpha = robot.odsSensor.getLightDetected();
            Log.v("BOK", "ALPHAW Final " + String.format("%.2f", current_alpha) + " d: " +
                    distance + " sec: " + String.format("%.2f", runTime.seconds()));
        } // if (opModeIsActive())
        return (current_alpha >= WHITE_LINE);
    }

    protected void alignToWall(LinearOpMode opMode, BoKHardwareBot robot, double distance,
                               double waitForSec){
        if (opMode.opModeIsActive()) {
            byte[] rangeLeft;
            byte[] rangeRight;

            rangeLeft = robot.rsLeftReader.read(0x04, 2);
            rangeRight = robot.rsRightReader.read(0x04, 2);

            double cmLeft = rangeLeft[0] & 0xFF;
            double cmRight = rangeRight[0] & 0xFF;
            //double threshold = 1.0;
            //double diffLeftAbs = Math.abs(cmLeft - distance);
            //double diffRightAbs = Math.abs(cmRight - distance);
            runTime.reset();

            Log.v("BOK", "Ls: " + cmLeft + ", distanceRs " + cmRight);

//            while ((cmLeft != distance) &&
//                    (cmRight != distance) &&
//                    (opMode.opModeIsActive()) &&
//                    (runTime.seconds() < waitForSec)) {

                Log.v("BOK", "L: " + cmLeft + ", distanceR " + cmRight);

                while (cmRight > distance) {
                    robot.setPowerToDTMotors(0, 0, -RIGHT_MOTOR_POWER/2, -RIGHT_MOTOR_POWER/2);
                    rangeRight = robot.rsRightReader.read(0x04, 2);
                    cmRight = rangeRight[0] & 0xFF;
                    Log.v("BOK", "distanceR " + cmRight);
                }
                robot.setPowerToDTMotors(0, 0, 0, 0);
                while (cmLeft > distance) {
                    robot.setPowerToDTMotors(LEFT_MOTOR_POWER/2, LEFT_MOTOR_POWER/2, 0, 0);
                    rangeLeft = robot.rsLeftReader.read(0x04, 2);
                    cmLeft = rangeLeft[0] & 0xFF;
                    Log.v("BOK", "distanceL " + cmLeft);
                }

                rangeLeft = robot.rsLeftReader.read(0x04, 2);
                rangeRight = robot.rsRightReader.read(0x04, 2);
                cmRight = rangeRight[0] & 0xFF;
                cmLeft = rangeLeft[0] & 0xFF;
                opMode.idle();
                robot.setPowerToDTMotors(0, 0, 0, 0);
//            }

        }
    }

    // Algorithm to go back from wall using the range sensor
    protected void goBackFromWall(LinearOpMode opMode, BoKHardwareBot robot,
                                  double targetDistance,
                                  double waitForSec)
    {
        double distance = 0;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            //robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToDTMotors(-LEFT_MOTOR_POWER, -RIGHT_MOTOR_POWER);
            //distance = robot.rangeSensorLeft.cmUltrasonic();
            runTime.reset();

            Log.v("BOK", "goBackward: " + distance);
            // run till red or blue line or if the user presses stop
            while (opMode.opModeIsActive() &&
                    (distance < targetDistance) &&
                    (runTime.seconds() < waitForSec)) {
                //distance = robot.rangeSensorLeft.cmUltrasonic();

                //opMode.telemetry.addData("BOK", "distance: " + distance);
                //opMode.telemetry.update();
                //distance = robot.rangeSensorLeft.cmUltrasonic();
                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
                //Log.v("BOK", "goBwd: " + distance);
            } // while (opModeIsActive())
            robot.setPowerToDTMotors(0.0f, 0.0f); // stop the robot
        } // if (opModeIsActive())
    }

    // Algorithm to go forward to the wall but stop when we reach the white line
    protected void goForwardToWall(LinearOpMode opMode, BoKHardwareBot robot,
                                   double targetDistance,
                                   double waitForSec)
    {
        double distance = 0, current_alpha;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            //robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToDTMotors(-LEFT_MOTOR_POWER/2, LEFT_MOTOR_POWER/2,
                                      RIGHT_MOTOR_POWER/2, -RIGHT_MOTOR_POWER/2);

            //distance = robot.rangeSensorLeft.cmUltrasonic();
            current_alpha = robot.odsSensor.getLightDetected();
            runTime.reset();

            Log.v("BOK", "goForward: " + distance + ", a: " + String.format(".2f", current_alpha) +
                    ", ang: " + robot.gyroSensor.getIntegratedZValue());
            // stop when near the white line and the distance < target distance
            while (opMode.opModeIsActive() &&
                    ((distance > targetDistance) &&
                     (current_alpha <= WHITE_LINE)) &&
                     (runTime.seconds() < waitForSec)) {
                //distance = robot.rangeSensorLeft.cmUltrasonic();
                current_alpha = robot.odsSensor.getLightDetected();

                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
                //Log.v("BOK", "goFwd: " + distance + " alpha: " + current_alpha +
                // " sec " + runTime.seconds());

            } // while (opModeIsActive())
            robot.setPowerToDTMotors(0,0,0,0); // stop the robot
        } // if (opModeIsActive())
    }

    // Push the beacon making sure that we do not get too close to the wall using the range sensor.
    protected void goForwardTillBeacon(LinearOpMode opMode, BoKHardwareBot robot,
                                       double targetDistance,
                                       double waitForSec)
    {
        double distance = 0;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            //robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToDTMotors(-LEFT_MOTOR_POWER/2, LEFT_MOTOR_POWER/2,
                                      RIGHT_MOTOR_POWER/2, -RIGHT_MOTOR_POWER/2);

            //distance = robot.rangeSensorLeft.cmUltrasonic();
            runTime.reset();

            Log.v("BOK", "goForward: " + distance + ", ang: " +
                    robot.gyroSensor.getIntegratedZValue());
            // stop when near the white line and the distance < target distance
            while (opMode.opModeIsActive() &&
                    (distance > targetDistance) &&
                    (runTime.seconds() < waitForSec)) {
                //distance = robot.rangeSensorLeft.cmUltrasonic();


                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
                //Log.v("BOK", "goFwd: " + distance + " alpha: " + current_alpha +
                // " sec " + runTime.seconds());

            } // while (opModeIsActive())
            robot.setPowerToDTMotors(0,0, 0, 0); // stop the robot
        } // if (opModeIsActive())
    }

    // Code copied from the sample PushbotAutoDriveByGyro_Linear
    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn( LinearOpMode opMode, BoKHardwareBot robot, double speed, double angle) {

        //robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() && !onHeading(opMode, robot, speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            opMode.telemetry.update();
            //opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
        }

        Log.v("BOK", "turnF: " + robot.gyroSensor.getIntegratedZValue() + "a: " +
                String.format("%.2f", robot.odsSensor.getLightDetected()));
        while (opMode.opModeIsActive() && !onHeading(opMode, robot, speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            opMode.telemetry.update();
            //opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
        }
        Log.v("BOK", "turnF: " + robot.gyroSensor.getIntegratedZValue() + "a: " +
                String.format("%.2f", robot.odsSensor.getLightDetected()));
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    private boolean onHeading(LinearOpMode opMode, BoKHardwareBot robot, double speed,
                              double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false;
        double   leftSpeed;
        double   rightSpeed;

        // determine turn power based on +/- error
        error = getError(robot, angle);
        if (Math.abs(error) <= (Math.abs(angle)/2))
            speed /= 2;

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);

            rightSpeed  = speed * steer;
            if (rightSpeed > 0)
                rightSpeed = Range.clip(rightSpeed, TURN_SPEED_LOW, TURN_SPEED_HIGH);
            else
                rightSpeed = Range.clip(rightSpeed, -TURN_SPEED_HIGH, -TURN_SPEED_LOW);

            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.setPowerToDTMotors(leftSpeed, leftSpeed, rightSpeed, rightSpeed);

        //Log.v("BOK", "Err: " + error + ", Steer: " + String.format("%.2f", steer));
        //Log.v("BOK", "Left Speed: " + String.format("%5.2f", leftSpeed) + ", Right Speed: " +
        // String.format("%5.2f", rightSpeed));
        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle
     *          Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of
     *          reference; +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double getError(BoKHardwareBot robot, double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.gyroSensor.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}

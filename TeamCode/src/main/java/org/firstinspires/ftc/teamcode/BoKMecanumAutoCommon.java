package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;
import android.view.SurfaceView;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Created by Krishna Saxena on 11/15/2016.
 * Implements the common algorithms used both by BoKAutoBlueBeacon and BoKAutoRedBeacon.
 * Its primary responsibilities include:
 * initSoftware method which
 * 1. initialize Vuforia
 * 2. initialize OpenCV
 * 3. set up servos and DC motors for autonomous
 */
public abstract class BoKMecanumAutoCommon extends BoKAutoCommon {

    public void initSoftware(LinearOpMode opMode, BoKHardwareBot robot, BoKAlliance redOrBlue) {
        super.initSoftware(opMode, robot, redOrBlue);

        //beacons.activate();
        //Log.v("BOK", "Done initializing Vuforia");
    }

    // Algorithm to move forward using encoder sensor on the DC motors on the drive train
    // Note that we use the abstract class BoKHardwareBot to completely separate out the
    // underlying drive train implementation
    protected void moveForward(LinearOpMode opMode, BoKHardwareBot robot,
                               double leftPower, double rightPower,  double inchesForward,
                               double waitForSec) {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            double targetEncCount = getTargetEncCount(inchesForward);

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

    // Algorithm to run to white line sideways using the Optical Distance Sensor
    protected boolean runToWhiteSideways(LinearOpMode opMode, BoKHardwareBot robot,
                                         double leftPower, double rightPower,
                                         boolean right, double waitForSec) {
        double current_alpha = 0, distance = 0;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            if (right) {
                robot.setPowerToDTMotors(+leftPower, -leftPower,
                        +rightPower, -rightPower);
            }
            else {
                robot.setPowerToDTMotors(-leftPower, +leftPower,
                        -rightPower, +rightPower);
            }
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
                //Log.v("BOK", "ALPHAW " + String.format("%.2f", current_alpha));
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
                               double speed, boolean right, double waitForSec) {
        byte[] rangeRight;
        byte[] rangeLeft;

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() &&
                !onFacingWall(opMode, robot, speed, distance, right, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            opMode.telemetry.update();
            //opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
        }

        rangeRight = robot.rsRightReader.read(0x04, 2);
        rangeLeft = robot.rsLeftReader.read(0x04, 2);
        Log.v("BOK", "alignR: " + (rangeRight[0] & 0xFF));
        Log.v("BOK", "alignL: " + (rangeLeft[0] & 0xFF));
    }

    protected boolean onFacingWall(LinearOpMode opMode, BoKHardwareBot robot, double speed,
                                double distance, boolean right, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false;
        double   leftSpeed;
        double   rightSpeed;
        double   cmCurrent;
        if (right) {
            byte[] rangeRight = robot.rsRightReader.read(0x04, 2);
            cmCurrent = rangeRight[0] & 0xFF;
        }
        else {
            byte[] rangeLeft = robot.rsLeftReader.read(0x04, 2);
            cmCurrent = rangeLeft[0] & 0xFF;
        }

        // determine turn power based on +/- error
        error = cmCurrent - distance;
        if (Math.abs(error) <= (Math.abs(distance)/2))
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

            leftSpeed   = rightSpeed;
        }

        // Send desired speeds to motors.
        if (right)
            robot.setPowerToDTMotors(0, 0, -rightSpeed, -rightSpeed);
        else
            robot.setPowerToDTMotors(leftSpeed, leftSpeed, 0, 0);

        //Log.v("BOK", "Err: " + error + ", Steer: " + String.format("%.2f", steer));
        //Log.v("BOK", "Left Speed: " + String.format("%5.2f", leftSpeed) + ", Right Speed: " +
        //  String.format("%5.2f", rightSpeed));
        return onTarget;
    }

    protected void alignToWall(LinearOpMode opMode, BoKHardwareBot robot, double distance,
                                   double waitForSec) {
        byte[] rangeLeft;
        byte[] rangeRight;

        if (opMode.opModeIsActive()) {

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

    public void slideSideways (LinearOpMode opMode, BoKHardwareBot robot,
                                   double leftPower, double rightPower,
                                   boolean right, double waitForSec){
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            if (right) {
                robot.setPowerToDTMotors(+leftPower, -leftPower,
                        +rightPower, -rightPower);
            }
            else {
                robot.setPowerToDTMotors(-leftPower, +leftPower,
                        -rightPower, +rightPower);
            }
            opMode.idle();

            runTime.reset();
            while (opMode.opModeIsActive() &&
                    (runTime.seconds() < waitForSec)) {


                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
                //Log.v("BOK", "ALPHAW " + String.format("%.2f", current_alpha));
                // distance + " sec: " + String.format("%.2f", runTime.seconds()));
            } // while (current_alpha < WHITE_LINE)

            robot.setPowerToDTMotors(0.0f, 0.0f, 0.0f, 0.0f); // stop the robot
            opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_LONG);

        } // if (opModeIsActive())
    }

    public void runParallelToWall (LinearOpMode opMode, BoKHardwareBot robot,
                               double leftPower, double rightPower, double distanceToWall,
                               boolean right, double waitForSec){
        byte[] rangeLeft;
        double current_alpha = 0, distance = 0;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            if (right) {
                robot.setPowerToDTMotors(+leftPower, -leftPower,
                        +rightPower, -rightPower);
            }
            else {
                robot.setPowerToDTMotors(-leftPower, +leftPower,
                        -rightPower, +rightPower);
            }
            opMode.idle();

            runTime.reset();
            rangeLeft = robot.rsLeftReader.read(0x04, 2);
            distance = rangeLeft[0] & 0xFF;
            current_alpha = robot.odsSensor.getLightDetected();

            Log.v("BOK", "RunParallel " + String.format("%.2f", current_alpha) + " d: " + distance);

            while (opMode.opModeIsActive() &&
                    (current_alpha < WHITE_LINE) && (runTime.seconds() < waitForSec)) {
                double error;
                current_alpha = robot.odsSensor.getLightDetected();
                rangeLeft = robot.rsLeftReader.read(0x04, 2);
                distance = rangeLeft[0] & 0xFF;

                error = distanceToWall - distance;
                if (Math.abs(error) <= HEADING_THRESHOLD) {
                    if (right) {
                        robot.setPowerToDTMotors(+leftPower, -leftPower,
                                +rightPower, -rightPower);
                    }
                    else {
                        robot.setPowerToDTMotors(-leftPower, +leftPower,
                                -rightPower, +rightPower);
                    }
                }
                else {
                    double steer = getSteer(error, P_TURN_COEFF);

                    double rightSpeed  = rightPower * steer;
                    if (rightSpeed > 0)
                        rightSpeed = Range.clip(rightSpeed, 0, 1);
                    else
                        rightSpeed = Range.clip(rightSpeed, -1, 0);

                    if (right) {
                        robot.setPowerToDTMotors(+rightSpeed, -leftPower,
                                +rightPower, -rightSpeed);
                    }
                    else {
                        robot.setPowerToDTMotors(-rightSpeed, +leftPower,
                                -rightPower, +rightSpeed);
                    }
                    Log.v("BOK", "RunParS " + String.format("%.2f", rightSpeed) +
                            distance + " left: " + String.format("%.2f", leftPower));
                }


                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
                Log.v("BOK", "RunPar " + String.format("%.2f", current_alpha) +
                    distance + " sec: " + String.format("%.2f", runTime.seconds()));
            } // while (current_alpha < WHITE_LINE)

            Log.v("BOK", "RunPar Final " + String.format("%.2f", current_alpha) +
                    distance + " sec: " + String.format("%.2f", runTime.seconds()));
            robot.setPowerToDTMotors(0.0f, 0.0f, 0.0f, 0.0f); // stop the robot
            opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_LONG);

        } // if (opModeIsActive())
    }

    // Algorithm that uses Vuforia to move the robot back till the image of the beacon is visible
    protected boolean detectBeaconColor(LinearOpMode opMode, BoKHardwareBot robot, int beacon,
                                                double waitForSec)
    {
        double distance = 0;

        if (opMode.opModeIsActive()) {
            /** Start tracking the data sets we care about. */
            //beacons.activate();

            //distance = robot.rangeSensorLeft.cmUltrasonic();

            Log.v("BOK", "detectBeaconColor!");
            runTime.reset();
            // run until the end of the match (driver presses STOP)
            //while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {
                List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
                Mat image = new Mat();
                boolean foundImage = false, foundRed = false;
                //Imgproc.cvtColor(image, image, Imgproc.COLOR_BGR2HSV);
                //Size sz = new Size(3, 3);
                //Imgproc.blur(image, image, sz);
                Imgproc.cvtColor(mRgba, mRgba, Imgproc.COLOR_BGR2RGB);
                Imgcodecs.imwrite("/sdcard/FIRST/myImage0.png", mRgba);
                Imgproc.cvtColor(mRgba, mRgba, Imgproc.COLOR_RGB2BGR);

                Core.inRange(mRgba, new Scalar(0, 0, 0), new Scalar(30, 30, 30), image);
                Mat kernel = new Mat(10, 10, CvType.CV_8UC1, new Scalar(1));

                Imgproc.morphologyEx(image, image, Imgproc.MORPH_OPEN, kernel);
                Imgproc.findContours(image, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
                Log.v("BOK", "Contours: " + contours.size());
                for (int i = 0; i < contours.size(); i++) {
                    Point center= new Point(0, 0);
                    float[] radius = new float[2];
                    MatOfPoint2f contour2f = new MatOfPoint2f( contours.get(i).toArray() );
                    Imgproc.minEnclosingCircle(contour2f, center, radius);
                    //System.out.println(center);
                    if ((center.x >= 350) && (center.x <= 420) && (center.y >= 100) && (center.y <= 200)) {
                        foundImage = true;
                        //if((radius[0] < 50) && (radius[0] > 20))
                        Imgproc.circle(image,center, (int)radius[0], new Scalar(255,255,255), 2);
                        //if (false) {
                        Mat hist = new Mat();
                        MatOfInt histSize = new MatOfInt(180);
                        MatOfFloat ranges = new MatOfFloat(0f, 180f);

                        Rect roi = new Rect((int)(center.x - 100), (int)(center.y - 10), 50, 50);
                        Mat subImg = mRgba.submat(roi);
                        int p, nRedPixels = 0, numPixels = 50*50;
                        Imgproc.rectangle(image, new Point(roi.x, roi.y),
                                new Point(roi.x + 50, roi.y + 50),
                                new Scalar(255, 255, 255));
                        Mat mask = new Mat(subImg.rows(), subImg.cols(),
                                CvType.CV_8UC1, new Scalar(255));
                        float[] resFloat = new float[180];

                        //Log.v("BOK", "Saving image");
                        //Imgproc.cvtColor(image, image, Imgproc.COLOR_BGR2RGB);
                        Imgcodecs.imwrite("/sdcard/FIRST/myImage1.png", image);

                        //Mat subMask = mask.submat(roi);
                        //subMask.setTo(new Scalar(255));

                        Imgproc.cvtColor(subImg, subImg, Imgproc.COLOR_BGR2HSV);
                        Imgproc.calcHist(Arrays.asList(subImg), new MatOfInt(0),
                                mask, hist, histSize, ranges);
                        //Imgcodecs.imwrite("/sdcard/FIRST/myImageH.png", img);
                        //Core.normalize(hist, hist, 256, 0, Core.NORM_MINMAX);
                        hist.get(0, 0, resFloat);
                        // String filename = "C:/Users/shiv/Documents/Images/subimg.png";
                        // System.out.println(String.format("Writing %s", filename));
                        // Imgcodecs.imwrite(filename, subImg);

                        for (p = 0; p < 180; p++) if (resFloat[p] > 0) Log.v("BOK", "p: " + p + ": " + resFloat[p]);
                        // Red is 0 (in HSV),
                        // but we need to check between 160-179 and 0-9
                        for (p = 0; p < 10; p++) {
                            nRedPixels += (int) resFloat[p];
                        }
                        for (p = 160; p < 180; p++) {
                            nRedPixels += (int) resFloat[p];
                        }

                        if (nRedPixels >= (numPixels / 2))
                            foundRed = true;
                        break;
                        //}
                    }

                }
            //Imgproc.cvtColor(image, image, Imgproc.COLOR_BGR2RGB);
            Imgcodecs.imwrite("/sdcard/FIRST/myImage2.png", image);

                if (foundImage) {
                    if (foundRed) Log.v("BOK", "RED!!");
                    else Log.v("BOK", "BLUE!!");
                }

            //} // while (opMode.isOpModeActive)
            Log.v("BOK", "detectBeaconColor: " + String.format("%.2f", runTime.seconds()) +
                    robot.gyroSensor.getIntegratedZValue());
        }
        return true;
    }

    // Algorithm to go back from wall using the range sensor
    protected void goBackFromWall(LinearOpMode opMode, BoKHardwareBot robot,
                                  double targetDistance, double leftPower, double rightPower,
                                  double waitForSec)
    {
        double distance = 0;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            byte[] rangeLeft;
            rangeLeft = robot.rsLeftReader.read(0x04, 2);

            robot.setPowerToDTMotors(-leftPower, -leftPower, rightPower, rightPower);

            distance = rangeLeft[0] & 0xFF;
            runTime.reset();

            Log.v("BOK", "goBackward: " + distance);
            // run till red or blue line or if the user presses stop
            while (opMode.opModeIsActive() &&
                    (distance < targetDistance) &&
                    (runTime.seconds() < waitForSec)) {
                rangeLeft = robot.rsLeftReader.read(0x04, 2);
                distance = rangeLeft[0] & 0xFF;

                //opMode.telemetry.addData("BOK", "distance: " + distance);
                //opMode.telemetry.update();
                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
                //Log.v("BOK", "goBwd: " + distance);
            } // while (opModeIsActive())
            robot.setPowerToDTMotors(0.0f, 0.0f, 0.0f, 0.0f); // stop the robot
        } // if (opModeIsActive())
    }

    // Push the beacon making sure that we do not get too close to the wall using the range sensor.
    protected void goForwardTillBeacon(LinearOpMode opMode, BoKHardwareBot robot,
                                       double targetDistance, double leftPower, double rightPower,
                                       double waitForSec)
    {
        double distance = 0;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            byte[] rangeLeft;
            rangeLeft = robot.rsLeftReader.read(0x04, 2);

            robot.setPowerToDTMotors(leftPower, leftPower, -rightPower, -rightPower);

            distance = rangeLeft[0] & 0xFF;
            runTime.reset();

            Log.v("BOK", "goForward: " + distance + ", ang: " +
                    robot.gyroSensor.getIntegratedZValue());
            // stop when near the white line and the distance < target distance
            while (opMode.opModeIsActive() &&
                    (distance > targetDistance) &&
                    (runTime.seconds() < waitForSec)) {
                rangeLeft = robot.rsLeftReader.read(0x04, 2);
                distance = rangeLeft[0] & 0xFF;

                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
                //Log.v("BOK", "goFwd: " + distance + " alpha: " + current_alpha +
                // " sec " + runTime.seconds());

            } // while (opModeIsActive())
            robot.setPowerToDTMotors(0, 0, 0, 0); // stop the robot
        } // if (opModeIsActive())
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
    protected boolean onHeading(LinearOpMode opMode, BoKHardwareBot robot, double speed,
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

            leftSpeed   = rightSpeed;
        }

        // Send desired speeds to motors.
        robot.setPowerToDTMotors(-leftSpeed, -leftSpeed, -rightSpeed, -rightSpeed);

        //Log.v("BOK", "Err: " + error + ", Steer: " + String.format("%.2f", steer));
        //Log.v("BOK", "Left Speed: " + String.format("%5.2f", leftSpeed) + ", Right Speed: " +
        //      String.format("%5.2f", rightSpeed));
        return onTarget;
    }

 }

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
import org.opencv.imgcodecs.Imgcodecs;
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
public abstract class BoKMecanumAutoCommon extends BoKAutoCommon {

    public void initSoftware(LinearOpMode opMode, BoKHardwareBot robot, BoKAlliance redOrBlue) {
        super.initSoftware(opMode, robot, redOrBlue);

        beacons.activate();
        Log.v("BOK", "Done initializing Vuforia");
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

            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.setPowerToDTMotors(leftSpeed, -leftSpeed, -rightSpeed, rightSpeed);

        //Log.v("BOK", "Err: " + error + ", Steer: " + String.format("%.2f", steer));
        //Log.v("BOK", "Left Speed: " + String.format("%5.2f", leftSpeed) + ", Right Speed: " +
        // String.format("%5.2f", rightSpeed));
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

    // Algorithm that uses Vuforia to move the robot back till the image of the beacon is visible
    protected boolean detectBeaconColor(LinearOpMode opMode, BoKHardwareBot robot,
                                                double waitForSec)
    {
        boolean picIsVisible = false, foundBeacon = false, imgProcessed = false;
        double distance = 0;

        if (opMode.opModeIsActive()) {
            /** Start tracking the data sets we care about. */
            beacons.activate();

            //distance = robot.rangeSensorLeft.cmUltrasonic();

            Log.v("BOK", "detectBeaconColor!");
            runTime.reset();
            // run until the end of the match (driver presses STOP)
            while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {

                Log.v("BOK", "Img: " + runTime.seconds());
                for (VuforiaTrackable beac : beacons) {
                    //OpenGLMatrix pose =
                    // ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
                    VuforiaTrackableDefaultListener listener =
                            (VuforiaTrackableDefaultListener)beac.getListener();
                    OpenGLMatrix rawPose = listener.getRawUpdatedPose();
                    if (rawPose != null) {
                        VuforiaLocalizer.CloseableFrame frame;
                        // takes the frame at the head of the queue
                        try {
                            frame = vuforiaFTC.getFrameQueue().take();
                        } catch (InterruptedException e) {
                            break;
                        }
                        Log.v("BOK", "Img processing: " + String.format("%.2f", runTime.seconds()));

                        long numImages = frame.getNumImages();
                        for (int i = 0; i < numImages; i++) {
                            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                                Image rgb = frame.getImage(i);
                                /*rgb is now the Image object that we’ve used in the video*/
                                if (rgb != null) {
                                    picIsVisible = foundBeacon = imgProcessed = true;
                                    boolean foundRed = false;
                                    int p, nRedPixels = 0;
                                    int numPixels = 90*50;
                                    Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(),
                                            Bitmap.Config.RGB_565);
                                    bm.copyPixelsFromBuffer(rgb.getPixels());

                                    Mat img = new Mat(rgb.getHeight(), rgb.getWidth(),
                                            CvType.CV_8UC3);
                                    Utils.bitmapToMat(bm, img);

                                    Mat hist = new Mat();
                                    MatOfInt histSize = new MatOfInt(180);
                                    MatOfFloat ranges = new MatOfFloat(0f, 180f);
                                    Mat mask = new Mat(img.rows(), img.cols(),
                                            CvType.CV_8UC1, new Scalar(0));
                                    float[] resFloat = new float[180];

                                    //Log.v("BOK", "Saving image");
                                    //Imgcodecs.imwrite("/sdcard/FIRST/myImage.png", img);

                                    // OpenCV only deals with BGR
                                    Imgproc.cvtColor(img, img, Imgproc.COLOR_RGB2BGR);
                                    Rect roi = new Rect(560, 24, 90, 50);
                                    Imgproc.rectangle(img, new Point(roi.x, roi.y),
                                            new Point(roi.x + 90, roi.y + 50),
                                            new Scalar(255, 255, 255));
                                    //Imgcodecs.imwrite("/sdcard/FIRST/myImage.png", img);
                                    Mat subMask = mask.submat(roi);
                                    subMask.setTo(new Scalar(255));

                                    Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2HSV);
                                    Imgproc.calcHist(Arrays.asList(img), new MatOfInt(0),
                                            mask, hist, histSize, ranges);
                                    //Imgcodecs.imwrite("/sdcard/FIRST/myImageH.png", img);
                                    //Core.normalize(hist, hist, 256, 0, Core.NORM_MINMAX);
                                    hist.get(0, 0, resFloat);

                                    // Red is 0 (in HSV),
                                    // but we need to check between 160-179 and 0-9
                                    for (p = 0; p < 10; p++) {
                                        nRedPixels += (int) resFloat[p];
                                    }
                                    for (p = 160; p < 180; p++) {
                                        nRedPixels += (int) resFloat[p];
                                    }

                                    if (nRedPixels >= (numPixels/2))
                                        foundRed = true;

                                    if (foundRed == true) {
                                        Log.v("BOK", beac.getName() + " Right: RED");
                                        if (alliance == BoKAlliance.BOK_ALLIANCE_RED) {
                                            robot.pusherRightServo.setPosition(
                                                    BoKHardwareBot.FINAL_SERVO_POS_PUSHER_RIGHT);
                                        }
                                        else {
                                            robot.pusherLeftServo.setPosition(
                                                    BoKHardwareBot.FINAL_SERVO_POS_PUSHER_LEFT);
                                        }
                                    }
                                    else {
                                        Log.v("BOK",beac.getName() + " RIGHT: BLUE");
                                        if (alliance == BoKAlliance.BOK_ALLIANCE_RED) {
                                            robot.pusherLeftServo.setPosition(
                                                    BoKHardwareBot.FINAL_SERVO_POS_PUSHER_LEFT);
                                        }
                                        else {
                                            robot.pusherRightServo.setPosition(
                                                    BoKHardwareBot.FINAL_SERVO_POS_PUSHER_RIGHT);
                                        }
                                    }

                                } // if (rgb != null)
                                break;
                            }//if (rgb != null)
                        }//for (int i = 0; i < numImages; i++)
                        frame.close();
                        break;
                    } // rawPose != null
                    else {
                        Log.v("BOK", "null");
                    }
                } // for beacons
                if ((picIsVisible == true) && (foundBeacon == true) && (imgProcessed == true)){
                    //robot.setPowerToMotors(0, 0);
                    //robot.waitForTick(METRONOME_TICK);
                    Log.v("BOK", "Stopping " + String.format("%.2f", runTime.seconds()) +
                            robot.gyroSensor.getIntegratedZValue());
                    break;
                }

                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
            } // while (opMode.isOpModeActive)
            Log.v("BOK", "Go back Final: " + String.format("%.2f", runTime.seconds()) +
                    robot.gyroSensor.getIntegratedZValue());
            robot.setPowerToDTMotors(0, 0);
        }
        return ((picIsVisible == true) && (foundBeacon == true) && (imgProcessed == true));
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

            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.setPowerToDTMotors(leftSpeed, -leftSpeed, -rightSpeed, rightSpeed);

        Log.v("BOK", "Err: " + error + ", Steer: " + String.format("%.2f", steer));
        Log.v("BOK", "Left Speed: " + String.format("%5.2f", leftSpeed) + ", Right Speed: " +
              String.format("%5.2f", rightSpeed));
        return onTarget;
    }

 }

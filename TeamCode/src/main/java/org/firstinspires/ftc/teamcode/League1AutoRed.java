package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
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
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;

/**
 * Created by Krishna Saxena on 10/5/2016.
 */
public class League1AutoRed implements BokAutoTest {
    protected AppUtil appUtil = AppUtil.getInstance();

    private int alliance = ALLIANCE_RED;
    private static final double WAIT_FOR_SEC_SHOOTER = 8.0;
    private static final double WAIT_FOR_SEC_LINE = 4.0;

    private VuforiaLocalizer vuforiaFTC;

    private final int BEACON_AREA_UR_WRT_CENTER_OF_IMAGE_X = 80; // in mm from center of image which is 254
    private final int BEACON_AREA_UR_WRT_CENTER_OF_IMAGE_Y = 240;
    private final int BEACON_AREA_BL_WRT_CENTER_OF_IMAGE_X = 40;
    private final int BEACON_AREA_BL_WRT_CENTER_OF_IMAGE_Y = 200;

    private static final float WHITE_LINE = 0.3f;
    private static final float MIDDLE_LINE = 0.2f;
    private static final double LEFT_POWER_LINE_FOLLOW = 0.2;
    private static final double RIGHT_POWER_LINE_FOLLOW = 0.225;

    private VuforiaTrackables beacons;
    private ElapsedTime runTime  = new ElapsedTime();

    private BaseLoaderCallback loaderCallback = new BaseLoaderCallback(appUtil.getActivity()) {
        @Override
        public void onManagerConnected(int status) {
            super.onManagerConnected(status);
        }
    };

    @Override
    public void initTest(BoKAutoRed opMode, BoKHardwareBot robot) {
        // Initialize OpenCV
        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, appUtil.getActivity(), loaderCallback);
        }
        else {
            loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        // Initialize Vuforia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = "ASodJvL/////AAAAGa9Isk5Oa0brtCRz7Z0fQngHmf2Selfx3RDk3MzjmK9DFWkQRsg1dH8Q8VvU/9L9nr9krXa+2nY5zoK6moC4UpRIg+gRsnG5M504q50Dd+z8DDATBaamc8t5qa8OeLjFKQ/+blHLbe8tjXSdVdl/xxGdowpeuQ18dnlf129q5NjM7Z9s/M8l693yEl28b+/LLJ4SiFLBTXwkEpVblemfJKZVHO5I8JmGmQ4jcwCWFIMCFxPRbCeDVqdCeqQzFa3BcCiuuGUgZDBCaidiW0/pzEFzdXcVCQfJPMgdZUWkPAk0QXVC8zYXaweeLuAONyTDkanRiyzqZbDVpJhVHaLBsUaC3OmZ/Xo+ThguyX3tNs3G";

        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforiaFTC = ClassFactory.createVuforiaLocalizer(parameters);

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforiaFTC.setFrameQueueCapacity(1); // change the frame queue capacity to 1

        beacons = vuforiaFTC.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");

        /** Start tracking the data sets we care about. */
        beacons.activate();

        robot.setupMotorEncoders(opMode);

        // set the initial position (both pointed down)
        robot.pusherLeftServo.setPosition(BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT);
        robot.pusherRightServo.setPosition(BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT);

        //robot.shooterServo.setPosition(BoKHardwareBot.INITIAL_SHOOTER_SERVO_POS);
    }

    @Override
    public void runTest(BoKAutoRed opMode, BoKHardwareBot robot) throws InterruptedException
    {
        // First shoot the two balls by turning on the sweeper and the ball shooter
        //shootBall(opMode, robot, WAIT_FOR_SEC_SHOOTER);

        // Move forward for 8 inch and turn 35 degrees (in 0.5 sec or less)
        moveForwardAndTurn(opMode, robot, 8.0, 35.0, 0.5);

        // Run to white
        runToWhite(opMode, robot, 5);
        turnToWhite(opMode, robot, 1);
        proportionalLineFollower(opMode, robot, 15); // 15 cm; give enough time for the robot to straighten up

        goBackTillBeaconIsVisible(opMode, robot, 2);

        goForwardToWall(opMode, robot, 50, 4); // 50 cm
        proportionalLineFollower(opMode, robot, 8);  // 8 cm
        goBackFromWall(opMode, robot, 20, 0.5);

        beacons.deactivate();
    }

    private void shootBall(BoKAutoRed opMode, BoKHardwareBot robot, double waitForSec) throws InterruptedException {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToMotors(0, 0); // Do not move the robot
            robot.setPowerToShooter(BoKHardwareBot.SHOOTER_MOTORS_POWER);   // start the ball shooter
            robot.sweeperMotor.setPower(BoKHardwareBot.SWEEPER_MOTOR_POWER_NORMAL);   // start the sweeper
            runTime.reset();

            // run until the end of the match (driver presses STOP)
            while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {
                opMode.telemetry.addData("Ball shooter: ", "%2.1f sec elapsed", runTime.seconds());
                opMode.telemetry.update();
            } // while (opModeIsActive())

            robot.setPowerToShooter(0.0f); // stop the ball shooter
            robot.sweeperMotor.setPower(0.0f); // stop the sweeper
        } // if (opModeIsActive())
    }

    private void moveForwardAndTurn(BoKAutoRed opMode, BoKHardwareBot robot, double inchesForward, double degreesToTurn, double waitForSec) throws InterruptedException {
        double degreesOfWheelTurn, degreesOfMotorTurn, targetEncCount;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            degreesOfWheelTurn = (360.0/(Math.PI*BoK6WDHardwareBot.WHEEL_DIAMETER_INCHES))*inchesForward;
            degreesOfMotorTurn = BoK6WDHardwareBot.DRIVE_GEAR_REDUCTION*degreesOfWheelTurn;
            targetEncCount = (BoK6WDHardwareBot.COUNTS_PER_MOTOR_REV*degreesOfMotorTurn)/360.0;
            robot.setMotorEncoderTarget((int)(targetEncCount), (int)targetEncCount);
            robot.setPowerToMotors(LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER);
            robot.waitForTick(METRONOME_TICK);

            while (opMode.opModeIsActive() && (robot.getCurrentPosition(opMode) == false)) {
                opMode.telemetry.update();
            }

            // Stop all motion;
            robot.setPowerToMotors(0, 0);
            robot.waitForTick(METRONOME_TICK);
            //opMode.idle();

            degreesOfWheelTurn = (BoK6WDHardwareBot.WHEEL_BASE * degreesToTurn)/ BoK6WDHardwareBot.WHEEL_DIAMETER_INCHES;
            degreesOfMotorTurn = BoK6WDHardwareBot.DRIVE_GEAR_REDUCTION*degreesOfWheelTurn;
            targetEncCount = (BoK6WDHardwareBot.COUNTS_PER_MOTOR_REV*degreesOfMotorTurn)/360.0;

            robot.setMotorEncoderTarget((int)(-targetEncCount), (int)targetEncCount);
            robot.waitForTick(METRONOME_TICK);

            runTime.reset();
            robot.setPowerToMotors(LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER);
            robot.waitForTick(METRONOME_TICK);

            while (opMode.opModeIsActive() && (robot.getCurrentPosition(opMode) == false) && (runTime.seconds() < waitForSec)) {
                //Log.v("BOK", "sec: " + runTime.seconds());
                opMode.telemetry.update();
            }

            // Stop all motion;
            robot.setPowerToMotors(0, 0);
            robot.waitForTick(METRONOME_TICK);

            // Turn off RUN_TO_POSITION
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            opMode.idle();
        }
    }

    private void runToWhite(BoKAutoRed opMode, BoKHardwareBot robot, double waitForSec) throws InterruptedException {
        double current_alpha;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToMotors(LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER);

            // start the sweeper in reverse
            runTime.reset();

            // run till red or blue line or if the user presses stop
            while (opMode.opModeIsActive()) {
                // go to red or blue line
                current_alpha=robot.odsSensor.getLightDetected();

                //Log.v("BOK", "Alpha " + current_alpha );
                while ((current_alpha < WHITE_LINE) && opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {
                    // Display the color info on the driver station

                    current_alpha = robot.odsSensor.getLightDetected();
                    //opMode.telemetry.addData("a: ", current_alpha + " sec: " + runTime.seconds());
                    opMode.telemetry.update();
                    //Log.v("BOK", "ALPHA " + current_alpha + " sec: " + runTime.seconds());
                } // while (current_alpha < WHITE_LINE) && (current_blue < BLUE_THRESHOLD)

                robot.setPowerToMotors(0.0f, 0.0f); // stop the robot
                robot.waitForTick(METRONOME_TICK);
                break; // done4
            } // while (opModeIsActive())
        } // if (opModeIsActive())
    }

    private void turnToWhite(BoKAutoRed opMode, BoKHardwareBot robot, double waitForSec) throws InterruptedException {
        double current_alpha;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToMotors(-LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER);
            runTime.reset();

            // run till red or blue line or if the user presses stop
            while (opMode.opModeIsActive()) {
                // go to red or blue line
                current_alpha=robot.odsSensor.getLightDetected();

                //Log.v("BOK", "Alpha " + current_alpha );

                while ((current_alpha <= MIDDLE_LINE) && opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {
                    // Display the color info on the driver station
                    opMode.telemetry.addData("a: ", current_alpha + " sec: " + runTime.seconds());
                    opMode.telemetry.update();
                    //Log.v("BOK", "ALPHA " + current_alpha + " sec: " + runTime.seconds());

                    current_alpha = robot.odsSensor.getLightDetected();
                } // while (current_red < RED_THRESHOLD) && (current_blue < BLUE_THRESHOLD)

                robot.setPowerToMotors(0.0f, 0.0f); // stop the robot
                robot.waitForTick(METRONOME_TICK);
                break; // done
            } // while (opModeIsActive())
        } // if (opModeIsActive())
    }

    private void goBackTillBeaconIsVisible(BoKAutoRed opMode, BoKHardwareBot robot, double waitForSec) throws InterruptedException
    {
        boolean picIsVisible = false, foundBeacon = false;

        if (opMode.opModeIsActive()) {
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToMotors(-LEFT_MOTOR_POWER/2.5, -RIGHT_MOTOR_POWER/2.5);
            runTime.reset();

            Log.v("BOK", "Go back!");
            // run until the end of the match (driver presses STOP)
            while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {
                //Log.v("BOK", "Img: " + runTime.seconds());
                for (VuforiaTrackable beac : beacons) {
                    //OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
                    OpenGLMatrix rawPose = ((VuforiaTrackableDefaultListener)beac.getListener()).getRawPose();
                    if (rawPose != null) {
                        //VectorF translation = pose.getTranslation();
                        Log.v("BOK", "Img processing: " + runTime.seconds());

                        //opMode.telemetry.addData(beac.getName() + "-Translation: ", translation);
                        picIsVisible = true;

                        // Convert the data in rawPose back to the format that Vuforia expects -
                        // 3x4 row major matrix. where as the OpenGLMatrix is 4x4 column major matrix.
                        Matrix34F raw = new Matrix34F();
                        float[] rawData = Arrays.copyOfRange(rawPose.transposed().getData(), 0, 12);
                        raw.setData(rawData);
                        // Now call Vuforia's projectPoint to convert 3D point in space to camera image coordinates
                        Vec2F pointUR = Tool.projectPoint(vuforiaFTC.getCameraCalibration(), raw, new Vec3F(BEACON_AREA_UR_WRT_CENTER_OF_IMAGE_X, BEACON_AREA_UR_WRT_CENTER_OF_IMAGE_Y, 0));
                        Vec2F pointBL = Tool.projectPoint(vuforiaFTC.getCameraCalibration(), raw, new Vec3F(BEACON_AREA_BL_WRT_CENTER_OF_IMAGE_X, BEACON_AREA_BL_WRT_CENTER_OF_IMAGE_Y, 0));
                        //opMode.telemetry.addData(beac.getName() + "-Img point: ", pointUL.getData()[0] + ", " +  pointUL.getData()[1]);

                        VuforiaLocalizer.CloseableFrame frame = vuforiaFTC.getFrameQueue().take(); //takes the frame at the head of the queue
                        long numImages = frame.getNumImages();

                        for (int i = 0; i < numImages; i++) {
                            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                                Image rgb = frame.getImage(i);
                                /*rgb is now the Image object that we’ve used in the video*/
                                if (rgb != null) {
                                    //opMode.telemetry.addData("rgb is not null!", "Yay: " + i);
                                    //opMode.telemetry.update();
                                    Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                                    bm.copyPixelsFromBuffer(rgb.getPixels());

                                    Mat img = new Mat(rgb.getHeight(), rgb.getWidth(), CvType.CV_8UC3);
                                    Utils.bitmapToMat(bm, img);
                                    Imgproc.cvtColor(img, img, Imgproc.COLOR_RGB2HSV);

                                    Mat hist = new Mat();
                                    MatOfInt histSize = new MatOfInt(180);
                                    MatOfFloat ranges = new MatOfFloat(0f, 180f);
                                    Mat mask = new Mat(img.rows(), img.cols(), CvType.CV_8UC1, new Scalar(0));
                                    float[] resFloat = new float[180];

                                    // the image is rotated 90 degrees
                                    int roiPixelsX = (int)pointBL.getData()[0] - (int)pointUR.getData()[0];
                                    int roiPixelsY = (int)pointBL.getData()[1] - (int)pointUR.getData()[1];
                                    int numPixels = roiPixelsX*roiPixelsY;

                                    Rect roi = new Rect((int)pointUR.getData()[0], (int)pointUR.getData()[1], roiPixelsX, roiPixelsY);
                                    Log.v("BOK", "NumPixels: " + numPixels + roi.toString() + "ROI: " + roiPixelsX + ", " + roiPixelsY);

                                    // Make sure that our region of interest fits in the image
                                    if ((roi.x >= 0) && (roi.x < (rgb.getWidth()-roiPixelsX)) && (roi.y >= 0) && (roi.y < (rgb.getHeight()-roiPixelsY))) {
                                        boolean foundRed = false;
                                        foundBeacon = true;
                                        Mat subMask = mask.submat(roi);
                                        subMask.setTo(new Scalar(255));

                                        Imgproc.calcHist(Arrays.asList(img), new MatOfInt(0), mask, hist, histSize, ranges);
                                        //Core.normalize(hist, hist, 256, 0, Core.NORM_MINMAX);
                                        hist.get(0, 0, resFloat);
                                        for (int p = 170; p < 180; p++) {
                                            if (((int) resFloat[p]) > 5) {
                                                Log.v("BOK", "num #: " + p + ", " + (int) resFloat[p]);
                                                foundRed = true;
                                                break;
                                            }
                                        }
                                        if (foundRed == false) {
                                            for (int p = 0; p < 10; p++) {
                                                if (((int) resFloat[p]) > 0) {
                                                    Log.v("BOK", "num #: " + p + ", " + (int) resFloat[p]);
                                                    foundRed = true;
                                                    break;
                                                }
                                            }
                                        }
                                        if (foundRed == true) {
                                            opMode.telemetry.addData(beac.getName() + " Right: ", "RED");
                                            Log.v("BOK",beac.getName() + "Right: RED");
                                            if (alliance == ALLIANCE_RED) {
                                                robot.pusherRightServo.setPosition(BoKHardwareBot.FINAL_SERVO_POS_PUSHER_LEFT);
                                            }
                                            else {
                                                robot.pusherLeftServo.setPosition(BoKHardwareBot.FINAL_SERVO_POS_PUSHER_LEFT);
                                            }

                                            //opMode.telemetry.addData("BL x:", String.valueOf(pointBL.getData()[0]));
                                            //opMode.telemetry.addData("BL y:", String.valueOf(pointBL.getData()[1]));
                                        }
                                        else {
                                            opMode.telemetry.addData(beac.getName() + "Right: ", "BLUE");
                                            if (alliance == ALLIANCE_RED) {
                                                robot.pusherLeftServo.setPosition(BoKHardwareBot.FINAL_SERVO_POS_PUSHER_LEFT);
                                            }
                                            else {
                                                robot.pusherRightServo.setPosition(BoKHardwareBot.FINAL_SERVO_POS_PUSHER_LEFT);
                                            }
                                            Log.v("BOK",beac.getName() + "RIGHT: BLUE");
                                        }
                                    }
                                }
                                break;
                            }//if (rgb != null)
                        }//for (int i = 0; i < numImages; i++)
                        frame.close();
                        break;
                    } // rawPose != null
                } // for beacons
                if ((picIsVisible == true) && (foundBeacon == true)){
                    robot.setPowerToMotors(0, 0);
                    robot.waitForTick(METRONOME_TICK);
                    Log.v("BOK", "Stopping " + runTime.seconds());
                    break;
                }
                opMode.telemetry.update();
            } // while (opMode.isOpModeActive)
        }
    }

    private void proportionalLineFollower(BoKAutoRed opMode, BoKHardwareBot robot, double distanceToWall) throws InterruptedException
    {
        double alpha, distance;
        float delta;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // run till red or blue line or if the user presses stop
            while (opMode.opModeIsActive()) {
                distance = robot.rangeSensor.cmUltrasonic();

                while (opMode.opModeIsActive() && (distance > distanceToWall)) {
                    double left_power, right_power;
                    distance = robot.rangeSensor.cmUltrasonic();
                    alpha = robot.odsSensor.getLightDetected();

                    delta = (float) (MIDDLE_LINE-alpha); // if you are on white, delta is negative, otherwise it is positive
                    if (delta > 0) {
                        left_power  = LEFT_POWER_LINE_FOLLOW-(delta*2);
                        right_power = RIGHT_POWER_LINE_FOLLOW;
                        robot.setPowerToMotors(left_power, right_power);
                    }
                    else {
                        left_power = LEFT_POWER_LINE_FOLLOW;
                        right_power = RIGHT_POWER_LINE_FOLLOW + (delta/2);
                        robot.setPowerToMotors(left_power, right_power);
                    }

                    //opMode.telemetry.addData("BoK", "ALPHA: " + alpha + " DELTA: " + delta + " LEFT_POWER: " + left_power + " RIGHT: " + right_power);
                    opMode.telemetry.update();

                } // while (opModeIsActive())
                robot.setPowerToMotors(0.0f, 0.0f); // stop the robot

                //opMode.telemetry.addData("BoK", "ALPHA FINAL " + alpha + " Difference: " + delta + " Percent " + steer);
                opMode.telemetry.update();

                break; // done
            } // while (opModeIsActive())
        } // if (opModeIsActive())
    }

    private void goBackFromWall(BoKAutoRed opMode, BoKHardwareBot robot, double targetDistance, double waitForSec) throws InterruptedException
    {
        double distance;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToMotors(-LEFT_MOTOR_POWER, -RIGHT_MOTOR_POWER);
            distance = robot.rangeSensor.cmUltrasonic();
            runTime.reset();

            //Log.v("BOK", "goBackward: " + distance);
            // run till red or blue line or if the user presses stop
            while (opMode.opModeIsActive() && (distance < targetDistance) && (runTime.seconds() < waitForSec)) {
                distance = robot.rangeSensor.cmUltrasonic();

                opMode.telemetry.addData("BOK", "distance: " + distance);
                opMode.telemetry.update();
                distance = robot.rangeSensor.cmUltrasonic();
                //Log.v("BOK", "goBwd: " + distance);
            } // while (opModeIsActive())
            robot.setPowerToMotors(0.0f, 0.0f); // stop the robot
        } // if (opModeIsActive())
    }

    private void goForwardToWall(BoKAutoRed opMode, BoKHardwareBot robot, double targetDistance, double waitForSec) throws InterruptedException
    {
        double distance, current_alpha;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToMotors(LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER);
            distance = robot.rangeSensor.cmUltrasonic();
            current_alpha = robot.odsSensor.getLightDetected();
            runTime.reset();

            //Log.v("BOK", "goForward: " + distance);
            // stop when near the white line or the distance < target distance
            while (opMode.opModeIsActive() && (distance > targetDistance) && (current_alpha <= MIDDLE_LINE) && (runTime.seconds() < waitForSec)) {
                distance = robot.rangeSensor.cmUltrasonic();
                current_alpha = robot.odsSensor.getLightDetected();

                opMode.telemetry.addData("BOK", "distance: " + distance + " alpha: " + current_alpha);
                opMode.telemetry.update();
                //Log.v("BOK", "goFwd: " + distance + " alpha: " + current_alpha);

            } // while (opModeIsActive())
            robot.setPowerToMotors(0.0f, 0.0f); // stop the robot
        } // if (opModeIsActive())
    }
}

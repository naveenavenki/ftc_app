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

import java.util.Arrays;

/**
 * Created by Krishna Saxena on 11/15/2016.
 */
public class BoKAutoCommon implements BoKAuto {
    protected AppUtil appUtil = AppUtil.getInstance();

    protected BoKAlliance alliance;
    private VuforiaLocalizer vuforiaFTC;

    private final int BEACON_AREA_UR_WRT_CENTER_OF_IMAGE_X = 80; // in mm from center of image which is 254
    private final int BEACON_AREA_UR_WRT_CENTER_OF_IMAGE_Y = 240;
    private final int BEACON_AREA_BL_WRT_CENTER_OF_IMAGE_X = 40;
    private final int BEACON_AREA_BL_WRT_CENTER_OF_IMAGE_Y = 200;

    protected static final float WHITE_LINE = 0.4f;
    protected static final float LINE_EDGE = 0.25f;
    protected static final double LEFT_POWER_LINE_FOLLOW = 0.2;
    protected static final double RIGHT_POWER_LINE_FOLLOW = 0.225;

    private static final double P_DRIVE_COEFF = 0.15;
    private static final double P_TURN_COEFF = 0.1;
    private static final double HEADING_THRESHOLD = 1;

    private VuforiaTrackables beacons;
    private ElapsedTime runTime  = new ElapsedTime();


    private BaseLoaderCallback loaderCallback = new BaseLoaderCallback(appUtil.getActivity()) {
        @Override
        public void onManagerConnected(int status) {
            super.onManagerConnected(status);
        }
    };

    @Override
    public void initSoftware(LinearOpMode opMode, BoKHardwareBot robot, BoKAlliance redOrBlue) {
        Log.v("BOK", "Initializing OpenCV");
        // Initialize OpenCV
        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, appUtil.getActivity(), loaderCallback);
        }
        else {
            loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.initGyro(opMode);
        opMode.telemetry.addData("Gyro", robot.gyroSensor.getIntegratedZValue());
        opMode.telemetry.update();
        Log.v("BOK", "Gyro: integrated: " + robot.gyroSensor.getIntegratedZValue());

        // set the initial position (both pointed down)
        robot.pusherLeftServo.setPosition(BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT);
        robot.pusherRightServo.setPosition(BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT);

        robot.shooterServo.setPosition(BoKHardwareBot.INITIAL_SHOOTER_SERVO_POS_AUTO);

        Log.v("BOK", "Calling setupMotorEncoders");

        alliance = redOrBlue;
        Log.v("BOK", "Initializing Vuforia");

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
        Log.v("BOK", "Done initializing Vuforia");
    }

    @Override
    public void runSoftware(LinearOpMode opMode, BoKHardwareBot robot) throws InterruptedException {
        // Do nothing
    }

    public void exitSoftware()
    {
        beacons.deactivate();
    }

    protected void shootBall(LinearOpMode opMode, BoKHardwareBot robot, double shooterMotorsPower, double waitForSec) throws InterruptedException {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            robot.setPowerToMotors(0, 0); // Do not move the robot
            robot.setPowerToShooter(shooterMotorsPower);   // start the ball shooter
            robot.sweeperMotor.setPower(BoKHardwareBot.SWEEPER_MOTOR_POWER_NORMAL);   // start the sweeper
            runTime.reset();

            // run until the end of the match (driver presses STOP)
            while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {
                //opMode.telemetry.addData("Ball shooter: ", "%2.1f sec elapsed", runTime.seconds());
                opMode.telemetry.update();
            } // while (opModeIsActive())

            robot.setPowerToShooter(0.0f); // stop the ball shooter
            robot.sweeperMotor.setPower(0.0f); // stop the sweeper
        } // if (opModeIsActive())
    }

    protected void moveForward(LinearOpMode opMode, BoKHardwareBot robot, double inchesForward, double waitForSec) throws InterruptedException {
        double degreesOfWheelTurn, degreesOfMotorTurn, targetEncCount;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            degreesOfWheelTurn = (360.0 / (Math.PI * BoK6WDHardwareBot.WHEEL_DIAMETER_INCHES)) * inchesForward;
            degreesOfMotorTurn = BoK6WDHardwareBot.DRIVE_GEAR_REDUCTION * degreesOfWheelTurn;
            targetEncCount = (BoK6WDHardwareBot.COUNTS_PER_MOTOR_REV * degreesOfMotorTurn) / 360.0;

            robot.setMotorEncoderTarget((int) (targetEncCount), (int) targetEncCount);
            robot.setPowerToMotors(LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER);

            runTime.reset();
            while (opMode.opModeIsActive() && (robot.getCurrentPosition() == false) && (runTime.seconds() < waitForSec)) {
                //opMode.telemetry.update();
                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS);
            }

            // Stop all motion;
            robot.setPowerToMotors(0, 0);

            // Turn off RUN_TO_POSITION
            // robot.setModeForMotors(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    protected void runToWhite(LinearOpMode opMode, BoKHardwareBot robot, double waitForSec) throws InterruptedException {
        double current_alpha, distance;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            //robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToMotors(LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER);
            opMode.idle();

            // go to white line
            current_alpha = robot.odsSensor.getLightDetected();
            distance = robot.rangeSensor.cmUltrasonic();
            runTime.reset();

            Log.v("BOK", "AlphaW " + String.format("%.2f", current_alpha) + " d: " + distance);
            while (opMode.opModeIsActive() && (current_alpha < WHITE_LINE) && (runTime.seconds() < waitForSec)) {
                // Display the color info on the driver station

                distance = robot.rangeSensor.cmUltrasonic();
                current_alpha = robot.odsSensor.getLightDetected();
                //opMode.telemetry.addData("a: ", current_alpha + " sec: " + runTime.seconds());
                //opMode.telemetry.update();
                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS);
                Log.v("BOK", "ALPHAW " + String.format("%.2f", current_alpha) + " d: " + distance + " sec: " + String.format("%.2f", runTime.seconds()));
            } // while (current_alpha < WHITE_LINE)

            robot.setPowerToMotors(0.0f, 0.0f); // stop the robot
            opMode.sleep(250);
            current_alpha = robot.odsSensor.getLightDetected();
            Log.v("BOK", "ALPHAW Final " + String.format("%.2f", current_alpha) + " d: " + distance + " sec: " + String.format("%.2f", runTime.seconds()));
        } // if (opModeIsActive())
    }

    protected void turnToWhite(LinearOpMode opMode, BoKHardwareBot robot, boolean turnLeft, double waitForSec) throws InterruptedException {
        double current_alpha;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            current_alpha=robot.odsSensor.getLightDetected();
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            Log.v("BOK", "AlphaT " + String.format("%.2f", current_alpha));

            if (turnLeft)
                robot.setPowerToMotors(-LEFT_MOTOR_POWER/2, RIGHT_MOTOR_POWER/2);
            else
                robot.setPowerToMotors(LEFT_MOTOR_POWER/2, -RIGHT_MOTOR_POWER/2);

            runTime.reset();

            while (opMode.opModeIsActive() && (current_alpha <= WHITE_LINE) && (runTime.seconds() < waitForSec)) {
                // Display the color info on the driver station
                //opMode.telemetry.addData("a: ", current_alpha + " sec: " + runTime.seconds());
                //opMode.telemetry.update();
                current_alpha = robot.odsSensor.getLightDetected();
                Log.v("BOK", "ALPHAT " + String.format("%.2f",current_alpha) + " sec: " + String.format("%.2f", runTime.seconds()));

                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS);
            } // while (current_alpha <= LINE_EDGE)

            robot.setPowerToMotors(0.0f, 0.0f); // stop the robot
            opMode.sleep(250);
            current_alpha = robot.odsSensor.getLightDetected();
            Log.v("BOK", "ALPHAT Final " +String.format("%.2f", current_alpha) + " sec: " + String.format("%.2f", runTime.seconds()));
        } // if (opModeIsActive())
    }

    protected void proportionalLineFollower(LinearOpMode opMode, BoKHardwareBot robot, boolean left, double distanceToWall) throws InterruptedException
    {
        double alpha, distance;
        float delta;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            distance = robot.rangeSensor.cmUltrasonic();
            alpha = robot.odsSensor.getLightDetected();
            Log.v("BOK", "Line follower " + distance + " a: " + String.format("%.2f", alpha));

            while (opMode.opModeIsActive() && (distance > distanceToWall)) {
                double left_power, right_power;
                distance = robot.rangeSensor.cmUltrasonic();
                alpha = robot.odsSensor.getLightDetected();

                if (left) {
                    delta = (float) (alpha - LINE_EDGE); // if you are on white, delta is positive, otherwise it is negative
                    if (delta > 0) {
                        left_power = LEFT_POWER_LINE_FOLLOW - (delta / 2);
                        right_power = RIGHT_POWER_LINE_FOLLOW;
                    } else {
                        left_power = LEFT_POWER_LINE_FOLLOW;
                        right_power = RIGHT_POWER_LINE_FOLLOW + (delta*2);
                    }
                }
                else { // right edge
                    delta = (float) (LINE_EDGE - alpha); // if you are on white, delta is negative, otherwise it is positive
                    if (delta > 0) {
                        left_power = LEFT_POWER_LINE_FOLLOW - (delta / 2);
                        right_power = RIGHT_POWER_LINE_FOLLOW;
                    } else {
                        left_power = LEFT_POWER_LINE_FOLLOW;
                        right_power = RIGHT_POWER_LINE_FOLLOW + (delta / 2);
                    }
                }
                robot.setPowerToMotors(left_power, right_power);

                Log.v("BOK", "ALPHA: " + String.format("%.2f", alpha) + " DELTA: " + String.format("%.2f", delta) + " LEFT_POWER: " + String.format("%.2f", left_power) + " RIGHT: " + String.format("%.2f", right_power));
                //opMode.telemetry.addData("BoK", "ALPHA: " + alpha + " DELTA: " + delta + " LEFT_POWER: " + left_power + " RIGHT: " + right_power);
                //opMode.telemetry.update();

            } // while (opModeIsActive())
            robot.setPowerToMotors(0.0f, 0.0f); // stop the robot

            //opMode.telemetry.addData("BoK", "ALPHA FINAL " + alpha + " Difference: " + delta + " Percent " + steer);
            //opMode.telemetry.update();
        } // if (opModeIsActive())
    }

    protected void goBackTillBeaconIsVisible(LinearOpMode opMode, BoKHardwareBot robot, double waitForSec) throws InterruptedException
    {
        boolean picIsVisible = false, foundBeacon = false;
        double distance;

        if (opMode.opModeIsActive()) {
            //robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToMotors(-LEFT_MOTOR_POWER/3, -RIGHT_MOTOR_POWER/3);
            distance = robot.rangeSensor.cmUltrasonic();

            Log.v("BOK", "Go back!");
            runTime.reset();
            // run until the end of the match (driver presses STOP)
            while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {

                if (distance <= 45) {
                    opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS);
                    distance = robot.rangeSensor.cmUltrasonic();
                    continue;
                }

                opMode.sleep(250);

                //Log.v("BOK", "Img: " + runTime.seconds());
                for (VuforiaTrackable beac : beacons) {
                    //OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
                    VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)beac.getListener();
                    OpenGLMatrix rawPose = listener.getRawUpdatedPose();
                    if (rawPose != null) {
                        //VectorF translation = pose.getTranslation();
                        Log.v("BOK", "Img processing: " + String.format("%.2f", runTime.seconds()));

                        // Convert the data in rawPose back to the format that Vuforia expects -
                        // 3x4 row major matrix. where as the OpenGLMatrix is 4x4 column major matrix.
                        Matrix34F raw = new Matrix34F();
                        float[] rawData = Arrays.copyOfRange(rawPose.transposed().getData(), 0, 12);
                        raw.setData(rawData);

                        Vec2F pointCenter = Tool.projectPoint(vuforiaFTC.getCameraCalibration(), raw, new Vec3F(0, 0, 0));
                        if (((int)pointCenter.getData()[0] >= 1280) || ((int)pointCenter.getData()[1] >= 720))
                            break;

                        // Now call Vuforia's projectPoint to convert 3D point in space to camera image coordinates
                        Vec2F pointUR = Tool.projectPoint(vuforiaFTC.getCameraCalibration(), raw, new Vec3F(BEACON_AREA_UR_WRT_CENTER_OF_IMAGE_X, BEACON_AREA_UR_WRT_CENTER_OF_IMAGE_Y, 0));
                        Vec2F pointBL = Tool.projectPoint(vuforiaFTC.getCameraCalibration(), raw, new Vec3F(BEACON_AREA_BL_WRT_CENTER_OF_IMAGE_X, BEACON_AREA_BL_WRT_CENTER_OF_IMAGE_Y, 0));
                        //opMode.telemetry.addData(beac.getName() + "-Img point: ", pointUL.getData()[0] + ", " +  pointUL.getData()[1]);

                        picIsVisible = true;

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
                                        int p, nRedPixels = 0;
                                        foundBeacon = true;

                                        robot.setPowerToMotors(0, 0);
                                        //Log.v("BOK", "Saving image");
                                        //Imgcodecs.imwrite("/sdcard/FIRST/myImage.png", img);

                                        Log.v("BOK", "Center: " + (int)pointCenter.getData()[0] + ", " + (int)pointCenter.getData()[1]);

                                        Imgproc.cvtColor(img, img, Imgproc.COLOR_RGB2BGR); // OpenCV only deals with BGR
                                        Imgproc.rectangle(img, new Point(roi.x, roi.y), new Point(roi.x + roiPixelsX, roi.y + roiPixelsY), new Scalar(255, 255, 255));
                                        Imgcodecs.imwrite("/sdcard/FIRST/myImageO.png", img);
                                        Mat subMask = mask.submat(roi);
                                        subMask.setTo(new Scalar(255));

                                        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2HSV);
                                        Imgproc.calcHist(Arrays.asList(img), new MatOfInt(0), mask, hist, histSize, ranges);
                                        //Imgcodecs.imwrite("/sdcard/FIRST/myImageH.png", img);
                                        //Core.normalize(hist, hist, 256, 0, Core.NORM_MINMAX);
                                        hist.get(0, 0, resFloat);
                                        for (p = 0; p < 10; p++) { // Red is 0 (in HSV), but we need to check between 170-179 and 0-9
                                            nRedPixels += (int) resFloat[p];
                                        }
                                        for (p = 160; p < 180; p++) {
                                            nRedPixels += (int) resFloat[p];
                                        }

                                        if (nRedPixels >= (numPixels/2))
                                            foundRed = true;

                                        if (foundRed == true) {
                                            //opMode.telemetry.addData(beac.getName() + " Right: ", "RED");
                                            Log.v("BOK", beac.getName() + " Right: RED");
                                            if (alliance == BoKAlliance.BOK_ALLIANCE_RED) {
                                                robot.pusherRightServo.setPosition(BoKHardwareBot.FINAL_SERVO_POS_PUSHER_RIGHT);
                                            }
                                            else {
                                                robot.pusherLeftServo.setPosition(BoKHardwareBot.FINAL_SERVO_POS_PUSHER_LEFT);
                                            }

                                            //opMode.telemetry.addData("BL x:", String.valueOf(pointBL.getData()[0]));
                                            //opMode.telemetry.addData("BL y:", String.valueOf(pointBL.getData()[1]));
                                        }
                                        else {
                                            //opMode.telemetry.addData(beac.getName() + "Right: ", "BLUE");
                                            if (alliance == BoKAlliance.BOK_ALLIANCE_RED) {
                                                robot.pusherLeftServo.setPosition(BoKHardwareBot.FINAL_SERVO_POS_PUSHER_LEFT);
                                            }
                                            else {
                                                robot.pusherRightServo.setPosition(BoKHardwareBot.FINAL_SERVO_POS_PUSHER_RIGHT);
                                            }
                                            Log.v("BOK",beac.getName() + "RIGHT: BLUE");
                                        }
                                    } // if roi is within the window

                                } // if (rgb != null)
                                break;
                            }//if (rgb != null)
                        }//for (int i = 0; i < numImages; i++)
                        frame.close();
                        break;
                    } // rawPose != null
                } // for beacons
                if ((picIsVisible == true) && (foundBeacon == true)){
                    //robot.setPowerToMotors(0, 0);
                    //robot.waitForTick(METRONOME_TICK);
                    Log.v("BOK", "Stopping " + runTime.seconds());
                    break;
                }
                //opMode.telemetry.update();
                //opMode.idle();
                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS);
            } // while (opMode.isOpModeActive)
        }
    }

    protected void goBackFromWall(LinearOpMode opMode, BoKHardwareBot robot, double targetDistance, double waitForSec) throws InterruptedException
    {
        double distance;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            //robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToMotors(-LEFT_MOTOR_POWER/2, -RIGHT_MOTOR_POWER/2);
            distance = robot.rangeSensor.cmUltrasonic();
            runTime.reset();

            Log.v("BOK", "goBackward: " + distance);
            // run till red or blue line or if the user presses stop
            while (opMode.opModeIsActive() && (distance < targetDistance) && (runTime.seconds() < waitForSec)) {
                distance = robot.rangeSensor.cmUltrasonic();

                //opMode.telemetry.addData("BOK", "distance: " + distance);
                //opMode.telemetry.update();
                distance = robot.rangeSensor.cmUltrasonic();
                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS);
                //Log.v("BOK", "goBwd: " + distance);
            } // while (opModeIsActive())
            robot.setPowerToMotors(0.0f, 0.0f); // stop the robot
        } // if (opModeIsActive())
    }

    protected void goForwardToWall(LinearOpMode opMode, BoKHardwareBot robot, double targetDistance, double waitForSec) throws InterruptedException
    {
        double distance, current_alpha;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            //robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToMotors(LEFT_MOTOR_POWER/2, RIGHT_MOTOR_POWER/2);

            distance = robot.rangeSensor.cmUltrasonic();
            current_alpha = robot.odsSensor.getLightDetected();
            runTime.reset();

            Log.v("BOK", "goForward: " + distance + ", " + current_alpha);
            // stop when near the white line and the distance < target distance
            while (opMode.opModeIsActive() && ((distance > targetDistance) && (current_alpha <= WHITE_LINE)) && (runTime.seconds() < waitForSec)) {
                distance = robot.rangeSensor.cmUltrasonic();
                current_alpha = robot.odsSensor.getLightDetected();

                //opMode.telemetry.addData("BOK", "distance: " + distance + " alpha: " + current_alpha);
                //opMode.telemetry.update();
                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS);
                Log.v("BOK", "goFwd: " + distance + " alpha: " + current_alpha + " sec " + runTime.seconds());

            } // while (opModeIsActive())
            robot.setPowerToMotors(0.0f, 0.0f); // stop the robot
        } // if (opModeIsActive())
    }

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
    public void gyroTurn (  LinearOpMode opMode, BoKHardwareBot robot, double speed, double angle) {

        //robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() && !onHeading(opMode, robot, speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            opMode.telemetry.update();
            //opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS);
        }
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
    boolean onHeading(LinearOpMode opMode, BoKHardwareBot robot, double speed, double angle, double PCoeff) {
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
            //if (steer >= 0) steer = Range.clip(steer, 0.5, 1.0);
            //else steer = Range.clip(steer, -1.0, -0.5);
            rightSpeed  = speed * steer;
            if (rightSpeed > 0) rightSpeed = Range.clip(rightSpeed, 0.11, 0.17);
            else rightSpeed = Range.clip(rightSpeed, -0.17, -0.11);

            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.setPowerToMotors(leftSpeed, rightSpeed);

        // Display it for the driver.
        //opMode.telemetry.addData("Target", "%5.2f", angle);
        //opMode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        //opMode.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        Log.v("BOK", "Err: " + error + ", Steer: " + steer);
        //Log.v("BOK", "Left Speed: " + String.format("%5.2f", leftSpeed) + ", Right Speed: " + String.format("%5.2f", rightSpeed));
        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(BoKHardwareBot robot, double targetAngle) {

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
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}

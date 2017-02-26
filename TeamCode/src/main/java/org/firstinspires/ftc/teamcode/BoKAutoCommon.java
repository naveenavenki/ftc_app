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
public abstract class BoKAutoCommon implements BoKAuto {
    protected AppUtil appUtil = AppUtil.getInstance();

    protected BoKAlliance alliance;
    protected VuforiaLocalizer vuforiaFTC;

    private static final int MIN_DISTANCE_FOR_IMAGE = 45; //cm

    // in mm from center of image (World Coordinates)
    private static final int BEACON_AREA_UR_WRT_CENTER_OF_IMAGE_X = 80;
    private static final int BEACON_AREA_UR_WRT_CENTER_OF_IMAGE_Y = 240;
    private static final int BEACON_AREA_BL_WRT_CENTER_OF_IMAGE_X = 40;
    private static final int BEACON_AREA_BL_WRT_CENTER_OF_IMAGE_Y = 200;

    protected static final double P_TURN_COEFF = 0.1;
    protected static final double HEADING_THRESHOLD = 1;
    protected static final double TURN_SPEED_LOW  = 0.16;
    protected static final double TURN_SPEED_HIGH = 0.2;

    protected VuforiaTrackables beacons;
    protected ElapsedTime runTime  = new ElapsedTime();

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

        Log.v("BOK", "Initializing Vuforia");

        // Initialize Vuforia
        VuforiaLocalizer.Parameters parameters =
                new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        // Vuforia License Key
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

        Log.v("BOK", "Done initializing Vuforia");
    }

    @Override
    public abstract void runSoftware(LinearOpMode opMode,
                                     BoKHardwareBot robot);

    public void exitSoftware()
    {
        beacons.deactivate();
    }

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

            robot.setPowerToDTMotors(0, 0); // Do not move the robot
            //robot.partLiftGateServo.setPosition(BoKHardwareBot.FINAL_SERVO_POS_PART_GATE);
            //robot.shooterServo.setPosition(BoKHardwareBot.INITIAL_SHOOTER_SERVO_POS_AUTO);
            //robot.setPowerToShooterMotors(shooterMotorsPower);
            //robot.sweeperMotor.setPower(BoKHardwareBot.SWEEPER_MOTOR_POWER_NORMAL);
            runTime.reset();

            // run until we either press STOP or run out of time
            while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {
                //opMode.telemetry.addData("Ball shooter: ",
                // "%2.1f sec elapsed", runTime.seconds());
                opMode.telemetry.update();
            } // while (opModeIsActive())

            //robot.setPowerToShooterMotors(0.0f); // stop the ball shooter
            //robot.sweeperMotor.setPower(0.0f); // stop the sweeper
        } // if (opModeIsActive())
    }

    protected double getTargetEncCount(double inchesForward) {
        double degreesOfWheelTurn, degreesOfMotorTurn;
        degreesOfWheelTurn = (360.0 / (Math.PI * BoK4MotorsDTBot.WHEEL_DIAMETER_INCHES)) *
                inchesForward;
        degreesOfMotorTurn = BoK4MotorsDTBot.DRIVE_GEAR_REDUCTION * degreesOfWheelTurn;
        return (BoK4MotorsDTBot.COUNTS_PER_MOTOR_REV * degreesOfMotorTurn) / 360.0;
    }

    // Algorithm to move forward using encoder sensor on the DC motors on the drive train
    // Note that we use the abstract class BoKHardwareBot to completely separate out the
    // underlying drive train implementation
    protected void moveForward(LinearOpMode opMode, BoKHardwareBot robot,
                               double leftPower, double rightPower, double inchesForward,
                               double waitForSec) {

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            double targetEncCount = getTargetEncCount(inchesForward);
            robot.setDTMotorEncoderTarget((int) (targetEncCount), (int) targetEncCount);
            robot.setPowerToDTMotors(leftPower, rightPower);

            runTime.reset();
            while (opMode.opModeIsActive() &&
                    (robot.getDTCurrentPosition() == false) &&
                    (runTime.seconds() < waitForSec)) {
                //opMode.telemetry.update();
                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
            }

            // Stop all motion;
            robot.setPowerToDTMotors(0, 0);
        }
    }

    // Algorithm to run to white line using the Optical Distance Sensor
    // Again, note that we use the abstract class BoKHardwareBot to completely separate out the
    // underlying drive train implementation
    protected boolean runToWhite(LinearOpMode opMode, BoKHardwareBot robot,
                                 double waitForSec) {
        double current_alpha = 0, distance = 0;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            //robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToDTMotors(LEFT_MOTOR_POWER/2, RIGHT_MOTOR_POWER/2);
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
                //Log.v("BOK", "ALPHAW " + String.format("%.2f", current_alpha) + " d: " +
                // distance + " sec: " + String.format("%.2f", runTime.seconds()));
            } // while (current_alpha < WHITE_LINE)

            robot.setPowerToDTMotors(0.0f, 0.0f); // stop the robot
            opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_LONG);

            current_alpha = robot.odsSensor.getLightDetected();
            Log.v("BOK", "ALPHAW Final " + String.format("%.2f", current_alpha) + " d: " +
                    distance + " sec: " + String.format("%.2f", runTime.seconds()));
        } // if (opModeIsActive())
        return (current_alpha >= WHITE_LINE);
    }

    // Algorithm to cross the white line and position the robot on either the left or right edge
    protected void turnToWhite(LinearOpMode opMode, BoKHardwareBot robot,
                               boolean turnLeft, double waitForSec) {
        double current_alpha;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            current_alpha=robot.odsSensor.getLightDetected();
            robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            Log.v("BOK", "AlphaT " + String.format("%.2f", current_alpha));

            if (turnLeft)
                robot.setPowerToDTMotors(-LEFT_MOTOR_POWER/2, RIGHT_MOTOR_POWER/2);
            else
                robot.setPowerToDTMotors(LEFT_MOTOR_POWER/2, -RIGHT_MOTOR_POWER/2);

            runTime.reset();

            while (opMode.opModeIsActive() &&
                    (current_alpha <= WHITE_LINE) &&
                    (runTime.seconds() < waitForSec)) {

                current_alpha = robot.odsSensor.getLightDetected();
                //Log.v("BOK", "ALPHAT " + String.format("%.2f",current_alpha) +
                // " sec: " + String.format("%.2f", runTime.seconds()));

                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
            } // while (current_alpha <= LINE_EDGE)

            robot.setPowerToDTMotors(0.0f, 0.0f); // stop the robot
            opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_LONG);
            current_alpha = robot.odsSensor.getLightDetected();
            Log.v("BOK", "ALPHAT Final " +String.format("%.2f", current_alpha) +
                    " sec: " + String.format("%.2f", runTime.seconds()));
        } // if (opModeIsActive())
    }

    // Proportional Line Following algorithm for either the left or right edge
    protected void proportionalLineFollower(LinearOpMode opMode, BoKHardwareBot robot,
                                            boolean left,
                                            double distanceToWall)
    {
        double alpha, distance = 0;
        float delta;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //distance = robot.rangeSensorLeft.cmUltrasonic();
            alpha = robot.odsSensor.getLightDetected();
            Log.v("BOK", "Line follower " + distance + " a: " + String.format("%.2f", alpha));

            while (opMode.opModeIsActive() && (distance > distanceToWall)) {
                double left_power, right_power;
                //distance = robot.rangeSensorLeft.cmUltrasonic();
                alpha = robot.odsSensor.getLightDetected();

                if (left) {
                    delta = (float) (alpha - LINE_EDGE);
                    // if you are on white, delta is positive, otherwise it is negative
                    if (delta > 0) {
                        left_power = LEFT_POWER_LINE_FOLLOW - (delta / 2);
                        right_power = RIGHT_POWER_LINE_FOLLOW;
                    } else {
                        left_power = LEFT_POWER_LINE_FOLLOW;
                        right_power = RIGHT_POWER_LINE_FOLLOW + (delta*2);
                    }
                }
                else { // right edge; this needs to be lower for blue beacon
                    delta = (float) (LINE_EDGE - alpha);
                    // if you are on white, delta is negative, otherwise it is positive
                    if (delta > 0) {
                        left_power = LEFT_POWER_LINE_FOLLOW - (delta*2);
                        right_power = RIGHT_POWER_LINE_FOLLOW;
                    } else {
                        left_power = LEFT_POWER_LINE_FOLLOW;
                        right_power = RIGHT_POWER_LINE_FOLLOW + (delta / 3);
                    }
                }
                robot.setPowerToDTMotors(left_power, right_power);

                //Log.v("BOK", "ALPHA: " + String.format("%.2f", alpha) + " DELTA: " +
                // String.format("%.2f", delta) + " LEFT_POWER: " +
                // String.format("%.2f", left_power) + " RIGHT: " +
                // String.format("%.2f", right_power));

            } // while (opModeIsActive())
            robot.setPowerToDTMotors(0.0f, 0.0f); // stop the robot

            Log.v("BOK", "Line Final: " + String.format("%.2f",
                    robot.odsSensor.getLightDetected()) + " gyro: " +
                    robot.gyroSensor.getIntegratedZValue() + " d: " );//+
                    //robot.rangeSensorLeft.cmUltrasonic());
        } // if (opModeIsActive())
    }

    // Algorithm that uses Vuforia to move the robot back till the image of the beacon is visible
    protected boolean goBackTillBeaconIsVisible(LinearOpMode opMode, BoKHardwareBot robot,
                                                double waitForSec)
    {
        boolean picIsVisible = false, foundBeacon = false, imgProcessed = false;
        double distance = 0;

        if (opMode.opModeIsActive()) {
            /** Start tracking the data sets we care about. */
            beacons.activate();

            //robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToDTMotors(-LEFT_MOTOR_POWER/3, -RIGHT_MOTOR_POWER/3);
            //distance = robot.rangeSensorLeft.cmUltrasonic();

            Log.v("BOK", "Go back!");
            runTime.reset();
            // run until the end of the match (driver presses STOP)
            while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {

                if (distance <= MIN_DISTANCE_FOR_IMAGE) {
                    opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
                    //distance = robot.rangeSensorLeft.cmUltrasonic();
                    continue;
                }
                opMode.sleep(SLEEP_250_MS);

                //Log.v("BOK", "Img: " + runTime.seconds());
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

                        // Convert the data in rawPose back to the format that Vuforia expects -
                        // 3x4 row major matrix. where as the OpenGLMatrix is 4x4
                        // column major matrix.
                        Matrix34F raw = new Matrix34F();
                        float[] rawData = Arrays.copyOfRange(rawPose.transposed().getData(), 0, 12);
                        raw.setData(rawData);

                        Vec2F pointCenter =
                                Tool.projectPoint(vuforiaFTC.getCameraCalibration(),
                                        raw, new Vec3F(0, 0, 0));
                        if (((int)pointCenter.getData()[0] >= 1280) ||
                                ((int)pointCenter.getData()[1] >= 720)) {
                            // center of the image is not visible yet, note the camera's resolution
                            // is 1280x720
                            break;
                        }

                        // Now call Vuforia's projectPoint to convert 3D point in space to camera image coordinates
                        Vec2F pointUR = Tool.projectPoint(vuforiaFTC.getCameraCalibration(), raw,
                                new Vec3F(BEACON_AREA_UR_WRT_CENTER_OF_IMAGE_X,
                                        BEACON_AREA_UR_WRT_CENTER_OF_IMAGE_Y, 0));
                        Vec2F pointBL = Tool.projectPoint(vuforiaFTC.getCameraCalibration(), raw,
                                new Vec3F(BEACON_AREA_BL_WRT_CENTER_OF_IMAGE_X,
                                        BEACON_AREA_BL_WRT_CENTER_OF_IMAGE_Y, 0));
                        picIsVisible = true;

                        long numImages = frame.getNumImages();
                        for (int i = 0; i < numImages; i++) {
                            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                                Image rgb = frame.getImage(i);
                                /*rgb is now the Image object that we’ve used in the video*/
                                if (rgb != null) {
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

                                    // the image is rotated 90 degrees
                                    int roiPixelsX = (int)pointBL.getData()[0] -
                                            (int)pointUR.getData()[0];
                                    int roiPixelsY = (int)pointBL.getData()[1] -
                                            (int)pointUR.getData()[1];
                                    int numPixels = roiPixelsX*roiPixelsY;

                                    Rect roi = new Rect((int)pointUR.getData()[0],
                                            (int)pointUR.getData()[1], roiPixelsX, roiPixelsY);
                                    Log.v("BOK", "NumPixels: " + numPixels + roi.toString() +
                                            "ROI: " + roiPixelsX + ", " + roiPixelsY);

                                    // Make sure that our region of interest fits in the image
                                    if ((roi.x >= 0) &&
                                            (roi.x < (rgb.getWidth()-roiPixelsX)) &&
                                            (roi.y >= 0) &&
                                            (roi.y < (rgb.getHeight()-roiPixelsY))) {
                                        boolean foundRed = false;
                                        int p, nRedPixels = 0;

                                        if (!foundBeacon) {
                                            foundBeacon = true;
                                            robot.setPowerToDTMotors(0, 0);
                                            break;
                                        }

                                        imgProcessed = true;
                                        //Log.v("BOK", "Saving image");
                                        //Imgcodecs.imwrite("/sdcard/FIRST/myImage.png", img);

                                        Log.v("BOK", "Center: " + (int)pointCenter.getData()[0] +
                                                ", " + (int)pointCenter.getData()[1]);

                                        // OpenCV only deals with BGR
                                        Imgproc.cvtColor(img, img, Imgproc.COLOR_RGB2BGR);
                                        Imgproc.rectangle(img, new Point(roi.x, roi.y),
                                                new Point(roi.x + roiPixelsX, roi.y + roiPixelsY),
                                                new Scalar(255, 255, 255));
                                        //Imgcodecs.imwrite("/sdcard/FIRST/myImageO.png", img);
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
                                    } // if roi is within the window

                                } // if (rgb != null)
                                break;
                            }//if (rgb != null)
                        }//for (int i = 0; i < numImages; i++)
                        frame.close();
                        break;
                    } // rawPose != null
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
            robot.setPowerToDTMotors(LEFT_MOTOR_POWER/2.5, RIGHT_MOTOR_POWER/2.5);

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
            robot.setPowerToDTMotors(0.0f, 0.0f); // stop the robot
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
            robot.setPowerToDTMotors(LEFT_MOTOR_POWER/2, RIGHT_MOTOR_POWER/2);

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
            robot.setPowerToDTMotors(0.0f, 0.0f); // stop the robot
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
    public void gyroTurn (  LinearOpMode opMode, BoKHardwareBot robot, double speed, double angle) {

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
        robot.setPowerToDTMotors(leftSpeed, rightSpeed);

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
    protected double getError(BoKHardwareBot robot, double targetAngle) {

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
    protected double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}
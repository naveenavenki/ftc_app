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
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
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
public class League1Auto implements BokAutoTest {
    protected AppUtil appUtil = AppUtil.getInstance();

    private static final double WAIT_FOR_SEC_SHOOTER = 8.0;
    private static final double WAIT_FOR_SEC_LINE = 4.0;
    //private static final double ALPHA_THRESHOLD  = 0.2;


    private double positionLeft = BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT;
    private double positionRight = BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT;

    private static final double SHOOTER_MOTOR_POWER = 1.0;
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
    private double distance;
    private float delta;
    private double steer;
    private double alpha;
    private double current_alpha;

    private BaseLoaderCallback loaderCallback = new BaseLoaderCallback(appUtil.getActivity()) {
        @Override
        public void onManagerConnected(int status) {
            super.onManagerConnected(status);
        }
    };

    @Override
    public void initTest(BoKAuto opMode, BoKHardwareBot robot) {
        // Initialize OpenCV
        /*
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
*/
        /** Start tracking the data sets we care about. */
  //      beacons.activate();
    }

    @Override
    public void runTest(BoKAuto opMode, BoKHardwareBot robot) throws InterruptedException
    {
        // set the initial position (both pointed down)
        //robot.setLeftPusherPos(positionLeft);
        //robot.setRightPusherPos(positionRight);
/*
        robot.setShooterServoPos(SHOOTER_SERVO_POS);
        // First shoot the two balls by turning on the sweeper and the ball shooter
        shootBall(opMode, robot, WAIT_FOR_SEC_SHOOTER);
        // Run to red or blue line
        runToRedOrBlue(opMode, robot, WAIT_FOR_SEC_LINE);*/
        //beaconTest(opMode, robot, 1.0);
        runToWhite(opMode, robot, 5);
        turnToWhite(opMode, robot, 5);
        //turnTillPicIsVisible(opMode, robot);
        proportionalLineFollowerTest(opMode, robot, 8);
        ultrasonicTest(opMode, robot, 0.5);
    }

    private void turnTillPicIsVisible(BoKAuto opMode, BoKHardwareBot robot) throws InterruptedException {
        boolean picIsVisible = false;

        if (opMode.opModeIsActive()) {
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToMotors(-LEFT_MOTOR_POWER / 3, RIGHT_MOTOR_POWER / 2.6);
            opMode.telemetry.addData("Status: ", "turn till pic 2");
            Log.v("BOK", "turn till pic 2");
            // run until the end of the match (driver presses STOP)
            while (opMode.opModeIsActive()) {
                for (VuforiaTrackable beac : beacons) {
                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
                    OpenGLMatrix rawPose = ((VuforiaTrackableDefaultListener)beac.getListener()).getRawPose();
                    if (pose != null) {
                        VectorF translation = pose.getTranslation();
                        opMode.telemetry.addData(beac.getName() + "-Translation: ", translation);
                        //double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), translation.get(2)));
                        //opMode.telemetry.addData(beac.getName() + "-Degrees: ", degreesToTurn);
                        picIsVisible = true;
                        // Now determine which beacon to push
                        robot.setPowerToMotors(0,0);
                        Log.v("BOK", "stopped");

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
                                if (rgb != null) {
                                    //opMode.telemetry.addData("rgb is not null!", "Yay: " + i);
                                    //opMode.telemetry.update();
                                /*rgb is now the Image object that we’ve used in the video*/
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
                                        Mat subMask = mask.submat(roi);
                                        subMask.setTo(new Scalar(255));

                                        Imgproc.calcHist(Arrays.asList(img), new MatOfInt(0), mask, hist, histSize, ranges);
                                        //Core.normalize(hist, hist, 256, 0, Core.NORM_MINMAX);
                                        hist.get(0, 0, resFloat);
                                        for (int p = 170; p < 180; p++) {
                                            if (((int) resFloat[p]) > 0) {
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
                                            opMode.telemetry.addData(beac.getName() + "Left: ", "RED");
                                            Log.v("BOK",beac.getName() + "Left: RED");
                                            opMode.telemetry.addData("BL x:", String.valueOf(pointBL.getData()[0]));
                                            opMode.telemetry.addData("BL y:", String.valueOf(pointBL.getData()[1]));
                                        }
                                        else {
                                            opMode.telemetry.addData(beac.getName() + "Left: ", "BLUE");
                                            Log.v("BOK",beac.getName() + "RIGHT: BLUE");
                                        }
                                    }
                                }
                                break;
                            }//if (rgb != null)
                        }//for (int i = 0; i < numImages; i++)
                        frame.close();



                        break;
                    }

                }
                if (picIsVisible == true) {
                    //robot.setPowerToMotors(0, 0);
                    Log.v("BOK", "Stopping ");
                    break;
                }
                opMode.telemetry.update();
                opMode.idle();
            }
        }
    }


        private void runToWhite(BoKAuto opMode, BoKHardwareBot robot, double waitForSec) throws InterruptedException {
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

                Log.v("BOK", "Alpha " + current_alpha );
                //int current_green = robot.colorSensor.green();

                while ((current_alpha < WHITE_LINE) && opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {
                    // Display the color info on the driver station
                    // opMode.telemetry.addData("r: ", current_red + " b: " + current_blue + " g: " + current_green);
                    opMode.telemetry.addData("a: ", current_alpha + " sec: " + runTime.seconds());
                    opMode.telemetry.update();
                    Log.v("BOK", "ALPHA " + current_alpha + " sec: " + runTime.seconds());

                    current_alpha = robot.odsSensor.getLightDetected();

                    // current_green = robot.colorSensor.green();
                } // while (current_red < RED_THRESHOLD) && (current_blue < BLUE_THRESHOLD)

                robot.setPowerToMotors(0.0f, 0.0f); // stop the robot
                break; // done4
            } // while (opModeIsActive())
        } // if (opModeIsActive())
    }
    private void turnToWhite(BoKAuto opMode, BoKHardwareBot robot, double waitForSec) throws InterruptedException {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToMotors(-LEFT_MOTOR_POWER*1.5, RIGHT_MOTOR_POWER*1.5);
            // start the sweeper in reverse
            runTime.reset();

            // run till red or blue line or if the user presses stop
            while (opMode.opModeIsActive()) {
                // go to red or blue line
                current_alpha=robot.odsSensor.getLightDetected();

                Log.v("BOK", "Alpha " + current_alpha );
                //int current_green = robot.colorSensor.green();

                while ((current_alpha <= MIDDLE_LINE) && opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {
                    // Display the color info on the driver station
                    // opMode.telemetry.addData("r: ", current_red + " b: " + current_blue + " g: " + current_green);
                    opMode.telemetry.addData("a: ", current_alpha + " sec: " + runTime.seconds());
                    opMode.telemetry.update();
                    Log.v("BOK", "ALPHA " + current_alpha + " sec: " + runTime.seconds());

                    current_alpha = robot.odsSensor.getLightDetected();

                    // current_green = robot.colorSensor.green();
                } // while (current_red < RED_THRESHOLD) && (current_blue < BLUE_THRESHOLD)

                robot.setPowerToMotors(0.0f, 0.0f); // stop the robot
                break; // done4
            } // while (opModeIsActive())
        } // if (opModeIsActive())
    }
    private void shootBall(BoKAuto opMode, BoKHardwareBot robot, double waitForSec) throws InterruptedException {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToMotors(0, 0); // Do not move the robot
            robot.setPowerToShooter(SHOOTER_MOTOR_POWER);   // start the ball shooter
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


    private void ultrasonicTest(BoKAuto opMode, BoKHardwareBot robot, double waitForSec) throws InterruptedException {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
//            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.setPowerToMotors(LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER);
            //robot.setPowerToSweeper(-1); // start the sweeper in reverse
            runTime.reset();
            robot.setPowerToMotors(-0.3,-0.3);

            // run till red or blue line or if the user presses stop
            while (opMode.opModeIsActive()) {
                // go to red or blue line
                /*int current_red = robot.colorSensor.red();
                int current_blue = robot.colorSensor.blue();
                Log.v("BOK", "Red " + current_red + " Blue " + current_blue);
                //int current_green = robot.colorSensor.green();
                */
                distance = robot.rangeSensor.cmUltrasonic();

                while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {
                    opMode.telemetry.addData("BOK", "distance: " + distance);
                    opMode.telemetry.update();
                    distance = robot.rangeSensor.cmUltrasonic();

                } // while (opModeIsActive())


                robot.setPowerToMotors(0.0f, 0.0f); // stop the robot
                break; // done
            } // while (opModeIsActive())
        } // if (opModeIsActive())
    }

    private void proportionalLineFollowerTest(BoKAuto opMode, BoKHardwareBot robot, double distanceToWall) throws InterruptedException {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.setPowerToMotors(LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER);

            runTime.reset();
            distance = robot.rangeSensor.cmUltrasonic();

            // run till red or blue line or if the user presses stop
            while (opMode.opModeIsActive()) {
                // go to red or blue line
                /*int current_red = robot.colorSensor.red();
                int current_blue = robot.colorSensor.blue();
                Log.v("BOK", "Red " + current_red + " Blue " + current_blue);
                //int current_green = robot.colorSensor.green();
                */
                alpha = robot.odsSensor.getLightDetected();
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
                    //steer = (double) ((delta)/MIDDLE_LINE); // if delta is negative, steering is also negative
/*
                    if(steer < 0){ // when you are on white, turn left towards gray
                        double right_power = (0.5*steer)-RIGHT_POWER_LINE_FOLLOW;
                        if (right_power < -RIGHT_POWER_LINE_FOLLOW) right_power = -RIGHT_POWER_LINE_FOLLOW;
                        robot.setPowerToMotors(right_power, -right_power); // -ve power on left wheels, +ve power on right wheels
                       // robot.setPowerToMotors((1*steer)-0.5, 0.5);
                    }
                    else if(steer > 0){ // when you are on gray, turn right toweards white
                        double left_power = (0.5*steer)+LEFT_POWER_LINE_FOLLOW;
                        if (left_power > LEFT_POWER_LINE_FOLLOW) left_power = LEFT_POWER_LINE_FOLLOW;
                        robot.setPowerToMotors(-left_power, RIGHT_POWER_LINE_FOLLOW);
                    }
                    else{
                        robot.setPowerToMotors(LEFT_POWER_LINE_FOLLOW, RIGHT_POWER_LINE_FOLLOW);
                    }

                    opMode.telemetry.addData("BoK", "Alpha: " + alpha + " Difference: " + delta + " Percent " + steer);
                    */
                    opMode.telemetry.addData("BoK", "ALPHA: " + alpha + " DELTA: " + delta + " LEFT_POWER: " + left_power + " RIGHT: " + right_power);
                    opMode.telemetry.update();

                } // while (opModeIsActive())
                robot.setPowerToMotors(0.0f, 0.0f); // stop the robot
                opMode.telemetry.addData("BoK", "ALPHA FINAL " + alpha + " Difference: " + delta + " Percent " + steer);
                opMode.telemetry.update();

                break; // done
            } // while (opModeIsActive())
        } // if (opModeIsActive())
    }
}

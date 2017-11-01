package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
/*
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;
*/
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
/*
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
*/
import java.util.Arrays;

/**
 * Created by Krishna Saxena on 11/15/2016.
 * Implements the common algorithms used both by BoKAutoBlue* and BoKAutoRed*.
 * Its primary responsibilities include:
 * initSoftware() method which
 *   1. initialize OpenCV
 *   2. initialize Vuforia
 * moveForward() method
 */
public abstract class BoKAutoCommon implements BoKAuto
{
    protected AppUtil appUtil = AppUtil.getInstance();
    private VuforiaLocalizer vuforiaFTC;
    private VuforiaTrackable relicTemplate;
    private VuforiaTrackables relicTrackables;

    protected ElapsedTime runTime  = new ElapsedTime();

    protected BoKAllianceColor alliance; // BOK_ALLIANCE_RED or BOK_ALLIANCE_BLUE
    protected LinearOpMode opMode;  // save a copy of the current opMode and robot
    protected BoKHardwareBot robot;

    protected RelicRecoveryVuMark cryptoColumn;
    protected boolean foundRedOnLeft = false;

    protected static final double P_TURN_COEFF = 0.5;    
    protected static final double HEADING_THRESHOLD = 1;

    private Orientation angles;
/*
    private BaseLoaderCallback loaderCallback = new BaseLoaderCallback(appUtil.getActivity())
    {
        @Override
        public void onManagerConnected(int status) {
            super.onManagerConnected(status);
        }
    };
*/
    @Override
    public BoKAutoStatus initSoftware(LinearOpMode opMode,
                                      BoKHardwareBot robot,
                                      BoKAllianceColor redOrBlue)
    {
/*
        Log.v("BOK", "Initializing OpenCV");
        // Initialize OpenCV
        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0,
                    appUtil.getActivity(), loaderCallback);
        }
        else {
            loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
*/
        Log.v("BOK", "Initializing Vuforia");
        // Initialize Vuforia
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the
         * RC phone); If no camera monitor is desired, use the parameterless constructor instead.
         */
        int cameraMonitorViewId =
                opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                        "id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters =
                new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        // Vuforia License Key
        parameters.vuforiaLicenseKey = "ASodJvL/////AAAAGa9Isk5Oa0brtCRz7Z0fQngHmf2Selfx3RDk3MzjmK9DFWkQRsg1dH8Q8VvU/9L9nr9krXa+2nY5zoK6moC4UpRIg+gRsnG5M504q50Dd+z8DDATBaamc8t5qa8OeLjFKQ/+blHLbe8tjXSdVdl/xxGdowpeuQ18dnlf129q5NjM7Z9s/M8l693yEl28b+/LLJ4SiFLBTXwkEpVblemfJKZVHO5I8JmGmQ4jcwCWFIMCFxPRbCeDVqdCeqQzFa3BcCiuuGUgZDBCaidiW0/pzEFzdXcVCQfJPMgdZUWkPAk0QXVC8zYXaweeLuAONyTDkanRiyzqZbDVpJhVHaLBsUaC3OmZ/Xo+ThguyX3tNs3G";
        vuforiaFTC = ClassFactory.createVuforiaLocalizer(parameters);
        //Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforiaFTC.setFrameQueueCapacity(1); // change the frame queue capacity to 1

        /*
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one
         * template, but differ in their instance id information.
         */
        relicTrackables = vuforiaFTC.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        //robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        alliance = redOrBlue;

        Log.v("BOK", "Done initializing software");

        this.opMode = opMode;
        this.robot = robot;

        //robot.resetDTEncoders(); // prepare for autonomous
        return BoKAutoStatus.BOK_AUTO_SUCCESS;
    }

    String format(OpenGLMatrix transformationMatrix)
    {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    @Override
    public abstract void runSoftware();

    public void getCryptoColumn(double waitForSec)
    {
        boolean vuMarkVisible = false;
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        // activate
        relicTrackables.activate();
        runTime.reset();

        while (!vuMarkVisible && runTime.seconds() < waitForSec) {
            /**
             * See if any of the instances of relicTemplate are currently visible.
             * RelicRecoveryVuMark is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by RelicRecoveryVuMark from VuforiaTrackable.
             */
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                opMode.telemetry.addData("VuMark", "%s visible", vuMark);
                vuMarkVisible = true;
                Log.v("BOK", "VuMark " + vuMark + " visible");

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose =
                        ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getRawUpdatedPose();
                // opMode.telemetry.addData("Pose", format(pose));


                if (pose != null) {
                    Log.v("BOK", "Img processing: " + String.format("%.2f", runTime.seconds()));

                    VuforiaLocalizer.CloseableFrame frame;
                    // takes the frame at the head of the queue
                    try {
                        frame = vuforiaFTC.getFrameQueue().take();
                    } catch (InterruptedException e) {
                        Log.v("BOK", "Exception!!");
                        break;
                    }
/*
                    // Convert the data in rawPose back to the format that Vuforia expects -
                    // 3x4 row major matrix. where as the OpenGLMatrix is 4x4
                    // column major matrix.
                    Matrix34F raw = new Matrix34F();
                    Log.v("BOK", "copy");
                    float[] rawData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                    raw.setData(rawData);

                    Vec2F pointCenter =
                            Tool.projectPoint(vuforiaFTC.getCameraCalibration(),
                                    raw, new Vec3F(0, 0, 0));

                    Log.v("BOK", "Center: " + (int)pointCenter.getData()[0] +
                            ", " + (int)pointCenter.getData()[1]);

                    long numImages = frame.getNumImages();
                    for (int i = 0; i < numImages; i++) {
                        if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                            Image rgb = frame.getImage(i);
                            // rgb is now the Image object that we’ve used in the video
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

                                Log.v("BOK", "Saving image");
                                Imgproc.cvtColor(img, img, Imgproc.COLOR_RGB2BGR);
                                Imgcodecs.imwrite("/sdcard/FIRST/myImage.png", img);


                                // OpenCV only deals with BGR

                                Rect roi = new Rect(505, 540, 95, 100);
                                Imgproc.rectangle(img, new Point(roi.x, roi.y),
                                        new Point(roi.x + 95, roi.y + 100),
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

                                int p, nRedPixels = 0, numPixels = 95*100;
                                // Red is 0 (in HSV),
                                // but we need to check between 160-179 and 0-9
                                for (p = 0; p < 10; p++) {
                                    nRedPixels += (int) resFloat[p];
                                }
                                for (p = 160; p < 180; p++) {
                                    nRedPixels += (int) resFloat[p];
                                }

                                boolean foundRed = false;
                                foundRedOnLeft = false;
                                if (nRedPixels >= (numPixels / 2))
                                    foundRed = true;

                                Log.v("BOK", "numPixels: " + nRedPixels + ", " + numImages);
                                if (foundRed == true) {
                                    foundRedOnLeft = true;
                                    Log.v("BOK", "Left is RED");
                                } else {
                                    Log.v("BOK", "Left is BLUE");
                                }
                            }
                            break;
                        } // if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565)
                    } // for (int i = 0; i < numImages; i++)

                    frame.close();
                    */
                } // if (pose != null)
                foundRedOnLeft = true;
                relicTrackables.deactivate();
            } else {
                opMode.telemetry.addData("VuMark", "not visible");
                opMode.telemetry.update();
                // Log.v("BOK", "VuMark not visible");
            }
        } // while (!vuMarkVisible)
        cryptoColumn = vuMark;
        Log.v("BOK", "Detecting Crypto Column: " + runTime.seconds());
    }

    public void setJewelFlicker()
    {
        robot.jewelFlicker.setPosition(robot.JF_FINAL);
        robot.jewelArm.setPosition(robot.JA_FINAL);
    }

    // Algorithm to move forward using encoder sensor on the DC motors on the drive train
    protected void move(double leftPower,
                        double rightPower,
                        double inches,
                        boolean forward,
                        double waitForSec)
    {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            robot.resetDTEncoders();
            robot.startMove(leftPower, rightPower, inches, forward);

            runTime.reset();
            while (opMode.opModeIsActive() &&
                    /*(robot.getDTCurrentPosition() == false) &&*/
                    robot.areDTMotorsBusy() &&
                    (runTime.seconds() < waitForSec)) {
                //opMode.telemetry.update();
                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
            }

            robot.stopMove();
        }
    }

    protected void strafe(double power,
                          double rotations,
                          boolean right,
                          double waitForSec)
    {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            robot.resetDTEncoders();
            robot.startStrafe(power, rotations, right);

            runTime.reset();
            while (opMode.opModeIsActive() &&
                    /*(robot.getDTCurrentPosition() == false) &&
                    robot.areDTMotorsBusy() &&*/
                    (runTime.seconds() < waitForSec)) {
                //opMode.telemetry.update();
                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
            }

            robot.stopMove();
        }
    }

    public boolean hitCryptoWithTouch(boolean forward, double waitForSeconds)
    {
        boolean flickerTouchState = true;
        ((BoKMecanumDT)robot).setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runTime.reset();
        if(forward){
            Log.v("BOK", "forward");
            robot.setPowerToDTMotors(DT_POWER_FOR_CRYPTO,
                                     DT_POWER_FOR_CRYPTO,
                                     -DT_POWER_FOR_CRYPTO,
                                     -DT_POWER_FOR_CRYPTO);
        }
        else{
            robot.setPowerToDTMotors(-DT_POWER_FOR_CRYPTO,
                                     -DT_POWER_FOR_CRYPTO,
                                     DT_POWER_FOR_CRYPTO,
                                     DT_POWER_FOR_CRYPTO);
        }

        flickerTouchState = robot.flickerTouch.getState();
        while(opMode.opModeIsActive() &&
                (runTime.seconds() < waitForSeconds) &&
                (flickerTouchState == true)){
            flickerTouchState = robot.flickerTouch.getState();
        }

        robot.setPowerToDTMotors(0, 0, 0, 0);
        // Return true if we touched the crypto box
        return (flickerTouchState == false);
    }

    public void moveWithRangeSensor(boolean forward, int distance, double waitForSeconds)
    {
        //byte[] range;
        double cmCurrent;
        //((BoKMecanumDT)robot).setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runTime.reset();

        if(forward){
            Log.v("BOK", "forward");
            //robot.setPowerToDTMotors(DT_POWER_FOR_WALL,
            //        DT_POWER_FOR_WALL,
            //        -DT_POWER_FOR_WALL,
            //        -DT_POWER_FOR_WALL);
            cmCurrent = robot.rangeSensorFront.cmUltrasonic();
        }
        else{
            //robot.setPowerToDTMotors(-DT_POWER_FOR_WALL,
            //        -DT_POWER_FOR_WALL,
            //        DT_POWER_FOR_WALL,
            //        DT_POWER_FOR_WALL);
            cmCurrent = robot.rangeSensorBack.cmUltrasonic();
        }

        while(opMode.opModeIsActive() /*&&
                (cmCurrent >= distance) &&
                (runTime.seconds() < waitForSeconds)*/) {
            if (forward) {
                cmCurrent = robot.rangeSensorFront.cmUltrasonic();
            }
            else {
                cmCurrent = robot.rangeSensorBack.cmUltrasonic();
            }
            opMode.telemetry.addData("Distance", cmCurrent);
            opMode.telemetry.update();
        }

        //robot.setPowerToDTMotors(0, 0, 0, 0);
    }

    // Code copied from the sample PushbotAutoDriveByGyro_Linear
    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn ( double speed, double angle) {

        //robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() && 
               !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            opMode.telemetry.update();
            //opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
        }

        //Log.v("BOK", "turnF: " + robot.imu.getIntegratedZValue());
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
    protected boolean onHeading(double speed,
                                double angle, 
                                double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false;
        double   leftSpeed;
        double   rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);
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
                rightSpeed = Range.clip(rightSpeed, DT_TURN_SPEED_LOW, DT_TURN_SPEED_HIGH);
            else
                rightSpeed = Range.clip(rightSpeed, -DT_TURN_SPEED_HIGH, -DT_TURN_SPEED_LOW);

            leftSpeed   = rightSpeed;
        }

        // Send desired speeds to motors.
        robot.setPowerToDTMotors(-leftSpeed, -leftSpeed, -rightSpeed, -rightSpeed);

        //Log.v("BOK", "Err: " + error + ", Steer: " + String.format("%.2f", steer));
        //Log.v("BOK", "Left Speed: " + String.format("%5.2f", leftSpeed) + ", Right Speed: " +
        //      String.format("%5.2f", rightSpeed));
        return onTarget;
    }
    
    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle
     *          Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of
     *          reference; +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    protected double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        robotError = targetAngle - angles.thirdAngle;
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

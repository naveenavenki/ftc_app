package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
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
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;

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

    protected ElapsedTime runTime  = new ElapsedTime();

    protected BoKAllianceColor alliance; // BOK_ALLIANCE_RED or BOK_ALLIANCE_BLUE
    protected LinearOpMode opMode;  // save a copy of the current opMode and robot
    protected BoKHardwareBot robot;

    private BaseLoaderCallback loaderCallback = new BaseLoaderCallback(appUtil.getActivity())
    {
        @Override
        public void onManagerConnected(int status) {
            super.onManagerConnected(status);
        }
    };

    @Override
    public BoKAutoStatus initSoftware(LinearOpMode opMode,
                                      BoKHardwareBot robot,
                                      BoKAllianceColor redOrBlue)
    {

        Log.v("BOK", "Initializing OpenCV");
        // Initialize OpenCV
        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0,
                    appUtil.getActivity(), loaderCallback);
        }
        else {
            loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        Log.v("BOK", "Initializing Vuforia");
        // Initialize Vuforia
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the
         * RC phone); If no camera monitor is desired, use the parameterless constructor instead.
         */
        VuforiaLocalizer vuforiaFTC;
        VuforiaTrackable relicTemplate;
        boolean vuMarkVisible = false;
        int cameraMonitorViewId =
                opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                        "id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters =
                new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        // Vuforia License Key
        parameters.vuforiaLicenseKey = "ASodJvL/////AAAAGa9Isk5Oa0brtCRz7Z0fQngHmf2Selfx3RDk3MzjmK9DFWkQRsg1dH8Q8VvU/9L9nr9krXa+2nY5zoK6moC4UpRIg+gRsnG5M504q50Dd+z8DDATBaamc8t5qa8OeLjFKQ/+blHLbe8tjXSdVdl/xxGdowpeuQ18dnlf129q5NjM7Z9s/M8l693yEl28b+/LLJ4SiFLBTXwkEpVblemfJKZVHO5I8JmGmQ4jcwCWFIMCFxPRbCeDVqdCeqQzFa3BcCiuuGUgZDBCaidiW0/pzEFzdXcVCQfJPMgdZUWkPAk0QXVC8zYXaweeLuAONyTDkanRiyzqZbDVpJhVHaLBsUaC3OmZ/Xo+ThguyX3tNs3G";
        vuforiaFTC = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforiaFTC.setFrameQueueCapacity(1); // change the frame queue capacity to 1

        /*
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one
         * template, but differ in their instance id information.
         */
        VuforiaTrackables relicTrackables = vuforiaFTC.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        // activate
        relicTrackables.activate();
        runTime.reset();

        while (!vuMarkVisible) {
            /**
             * See if any of the instances of relicTemplate are currently visible.
             * RelicRecoveryVuMark is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by RelicRecoveryVuMark from VuforiaTrackable.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
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
                    
                    frame.close();
                } // if (pose != null)
                relicTrackables.deactivate();
            } else {
                opMode.telemetry.addData("VuMark", "not visible");
                opMode.telemetry.update();
                // Log.v("BOK", "VuMark not visible");
            }
        } // while (!vuMarkVisible)

        //robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        alliance = redOrBlue;

        Log.v("BOK", "Done initializing software");

        this.opMode = opMode;
        this.robot = robot;

        robot.resetDTEncoders(); // prepare for autonomous
        return BoKAutoStatus.BOK_AUTO_SUCCESS;
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    @Override
    public abstract void runSoftware();

    // Algorithm to move forward using encoder sensor on the DC motors on the drive train
    protected void move(double leftPower,
                        double rightPower,
                        double inches,
                        boolean forward,
                        double waitForSec)
    {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

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

}

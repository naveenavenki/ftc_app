package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.OpenCVLoader;

/**
 * Created by shiv on 10/5/2016.
 */
//@Autonomous(name="BoK Auto Vuforia", group="BoK6WD")
public class BoKOpenCVTest implements BokAutoTest {
    private VuforiaTrackables beacons;
    VuforiaLocalizer vuforia;

    @Override
    public void initTest(BoKAuto opMode, BoKHardwareBot robot) {
        //if (!OpenCVLoader.initDebug()) {
            //OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, null, null);

        //}
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = "ASodJvL/////AAAAGa9Isk5Oa0brtCRz7Z0fQngHmf2Selfx3RDk3MzjmK9DFWkQRsg1dH8Q8VvU/9L9nr9krXa+2nY5zoK6moC4UpRIg+gRsnG5M504q50Dd+z8DDATBaamc8t5qa8OeLjFKQ/+blHLbe8tjXSdVdl/xxGdowpeuQ18dnlf129q5NjM7Z9s/M8l693yEl28b+/LLJ4SiFLBTXwkEpVblemfJKZVHO5I8JmGmQ4jcwCWFIMCFxPRbCeDVqdCeqQzFa3BcCiuuGUgZDBCaidiW0/pzEFzdXcVCQfJPMgdZUWkPAk0QXVC8zYXaweeLuAONyTDkanRiyzqZbDVpJhVHaLBsUaC3OmZ/Xo+ThguyX3tNs3G";

        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");

        /** Start tracking the data sets we care about. */
        beacons.activate();
    }

    @Override
    public void runTest(BoKAuto opMode, BoKHardwareBot robot) throws InterruptedException {
        while (opMode.opModeIsActive()) {

            for (VuforiaTrackable beac : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)beac.getListener()).getPose();
                if (pose != null) {
                    VectorF translation = pose.getTranslation();
                    opMode.telemetry.addData(beac.getName() + "-Translation: ", translation);
                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), translation.get(2)));
                    opMode.telemetry.addData(beac.getName() + "-Degrees: ", degreesToTurn);

                    VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take() ;//takes the frame at the head of the queue
                    Image rgb = null;

                    long numImages = frame.getNumImages();
                   // opMode.telemetry.addData("Num frames ", numImages);
                   // opMode.telemetry.update();

                    for (int i = 0; i < numImages; i++) {
                        if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                            rgb = frame.getImage(i);
                            opMode.telemetry.addData("rgb is not null!","Yay");
                            opMode.telemetry.update();
                            break;
                        }//if
                    }//for

/*rgb is now the Image object that we’ve used in the video*/

                }

            }
            opMode.telemetry.update();
            opMode.idle();

        }


    }


}

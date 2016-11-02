package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;

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
import org.opencv.core.Core;
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
//@Autonomous(name="BoK Auto Vuforia", group="BoK6WD")
public class BoKOpenCVTest implements BokAutoTest {
    protected AppUtil appUtil = AppUtil.getInstance();
    private VuforiaTrackables beacons;
    private VuforiaLocalizer vuforiaFTC;

    private final int BEACON_AREA_UR_WRT_CENTER_OF_IMAGE_X = -40; // in mm from center of image which is 254
    private final int BEACON_AREA_UR_WRT_CENTER_OF_IMAGE_Y = 240;
    private final int BEACON_AREA_BL_WRT_CENTER_OF_IMAGE_X = -80;
    private final int BEACON_AREA_BL_WRT_CENTER_OF_IMAGE_Y = 200;

    private BaseLoaderCallback loaderCallback = new BaseLoaderCallback(appUtil.getActivity()) {
        @Override
        public void onManagerConnected(int status) {
            super.onManagerConnected(status);
        }
    };

    @Override
    public void initTest(BoKAuto opMode, BoKHardwareBot robot) {
        // Initialize OpenCV
        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, appUtil.getActivity(), loaderCallback);
        }
        else {
            loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = "ASodJvL/////AAAAGa9Isk5Oa0brtCRz7Z0fQngHmf2Selfx3RDk3MzjmK9DFWkQRsg1dH8Q8VvU/9L9nr9krXa+2nY5zoK6moC4UpRIg+gRsnG5M504q50Dd+z8DDATBaamc8t5qa8OeLjFKQ/+blHLbe8tjXSdVdl/xxGdowpeuQ18dnlf129q5NjM7Z9s/M8l693yEl28b+/LLJ4SiFLBTXwkEpVblemfJKZVHO5I8JmGmQ4jcwCWFIMCFxPRbCeDVqdCeqQzFa3BcCiuuGUgZDBCaidiW0/pzEFzdXcVCQfJPMgdZUWkPAk0QXVC8zYXaweeLuAONyTDkanRiyzqZbDVpJhVHaLBsUaC3OmZ/Xo+ThguyX3tNs3G";

        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        parameters.useExtendedTracking = false; // disable extended tracking
        vuforiaFTC = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1); // set simultaneous image targets to 1
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforiaFTC.setFrameQueueCapacity(1); // change the frame queue capacity to 1

        beacons = vuforiaFTC.loadTrackablesFromAsset("FTC_2016-17");
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
                OpenGLMatrix rawPose = ((VuforiaTrackableDefaultListener)beac.getListener()).getRawPose();
                if (pose != null) {
                    VectorF translation = pose.getTranslation();
                    opMode.telemetry.addData(beac.getName() + "-Translation: ", translation);

                    // Convert the data in rawPose back to the format that Vuforia expects -
                    // 3x4 row major matrix. where as the OpenGLMatrix is 4x4 column major matrix.
                    Matrix34F raw = new Matrix34F();
                    float[] rawData = Arrays.copyOfRange(rawPose.transposed().getData(), 0, 12);
                    raw.setData(rawData);

                    // Now call Vuforia's projectPoint to convert 3D point in space to camera image coordinates
                    Vec2F pointUR = Tool.projectPoint(vuforiaFTC.getCameraCalibration(), raw, new Vec3F(BEACON_AREA_UR_WRT_CENTER_OF_IMAGE_X, BEACON_AREA_UR_WRT_CENTER_OF_IMAGE_Y, 0));
                    Vec2F pointBL = Tool.projectPoint(vuforiaFTC.getCameraCalibration(), raw, new Vec3F(BEACON_AREA_BL_WRT_CENTER_OF_IMAGE_X, BEACON_AREA_BL_WRT_CENTER_OF_IMAGE_Y, 0));
                    //opMode.telemetry.addData(beac.getName() + "-Img point: ", pointUL.getData()[0] + ", " +  pointUL.getData()[1]);

                    VuforiaLocalizer.CloseableFrame frame = vuforiaFTC.getFrameQueue().take() ;//takes the frame at the head of the queue
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
                                Log.v("BOL", "NumPixels: " + numPixels);

                                Rect roi = new Rect((int)pointUR.getData()[0], (int)pointUR.getData()[1], roiPixelsX, roiPixelsY);

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
                                    }
                                    else {
                                        opMode.telemetry.addData(beac.getName() + "Left: ", "BLUE");
                                    }
                                }
                            }
                            break;
                        }//if (rgb != null)
                    }//for (int i = 0; i < numImages; i++)
                    frame.close();
                }
            }
            opMode.telemetry.update();
            opMode.idle();
        }

    }


}

package org.firstinspires.ftc.teamcode;

/**
 * Created by shiv on 10/25/2017.
 */
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Gyro", group = "test")
public class GyroTest extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;
   // Acceleration gravity;
  //  BoKMecanumDT robot;

    public void calibrate(){
        sleep(3000);
        telemetry.addData("Status","Calibration complete");
        telemetry.update();
    }


    @Override
    public void runOpMode() throws InterruptedException {
        calibrate();
        imu = hardwareMap.get(BNO055IMU.class,"imu_bot");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        angles = new Orientation();
        imu.initialize(parameters);
       // robot.initHardware(this);

        waitForStart();

       while (opModeIsActive()) {
               angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
               telemetry.addData("Angle", angles.firstAngle + "degrees");
               telemetry.addData("Angle", angles.secondAngle + "degrees");
               telemetry.addData("Angle", angles.thirdAngle + "degrees");
               telemetry.update();
       }
    }

 /*  public void goToAngle(float targetHeading) {
        while (opModeIsActive() && angles.thirdAngle!=targetHeading) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            if ((angles.thirdAngle) == (targetHeading)) {
                robot.setPowerToDTMotors(0,0,0,0);
            } else if ((angles.thirdAngle) > targetHeading){
                robot.setPowerToDTMotors(-0.05,0.05);
            }
            else{
                robot.setPowerToDTMotors(0.05,-0.05);
            }
        }
    }
    */
    
}

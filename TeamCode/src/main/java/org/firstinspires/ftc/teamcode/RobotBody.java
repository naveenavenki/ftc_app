package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by james on 10/13/2017.
 */

public class RobotBody {
    protected DcMotor leftFront, rightFront, leftBack, rightBack, turnTable, upperArm;

    protected final String LEFT_FRONT_MOTOR_NAME = "lf";
    protected final String RIGHT_FRONT_MOTOR_NAME = "rf";
    protected final String LEFT_BACK_MOTOR_NAME = "lb";
    protected final String RIGHT_BACK_MOTOR_NAME = "rb";
    private final double GAME_STICK_DEAD_ZONE = 0.05;

    protected final String TURN_TABLE = "tt";
    protected final String UPPER_ARM = "ua";


    private Servo clawWrist, clawGrab;

    protected final String CLAW_WRIST_SERVO = "cw";
    protected final String CLAW_GRAB_SERVO = "cg";

    private double uADegrees = 0;
    private final int lowerUADegrees = 0;
    private final int upperUADegrees = 165;
    private double tTDegrees = 0;
    private final int lowerTTDegrees = -180;
    private final int upperTTDegrees = 0;
    private final int encoderPulses = 7;
    private final int gearbox40 = 40;


    protected int initDriveTrain(LinearOpMode opMode) {
        leftFront = opMode.hardwareMap.dcMotor.get(LEFT_FRONT_MOTOR_NAME);
        if(leftFront == null) {
            return -1;
        }
        rightFront = opMode.hardwareMap.dcMotor.get(RIGHT_FRONT_MOTOR_NAME);
        if(rightFront == null) {
            return -1;
        }
        leftBack = opMode.hardwareMap.dcMotor.get(LEFT_BACK_MOTOR_NAME);
        if(leftBack == null) {
            return -1;
        }
        rightBack = opMode.hardwareMap.dcMotor.get(RIGHT_BACK_MOTOR_NAME);
        if(rightBack == null) {
            return -1;
        }
        turnTable = opMode.hardwareMap.dcMotor.get(TURN_TABLE);
        if(turnTable == null) {
            return -1;
        }
        upperArm = opMode.hardwareMap.dcMotor.get(UPPER_ARM);
        if(upperArm == null) {
            return -1;
        }
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        return 0;
    }

    protected int initRobot(LinearOpMode opMode) {
        clawWrist = opMode.hardwareMap.servo.get(CLAW_WRIST_SERVO);
        if(clawWrist == null) {
            return -1;
        }
        clawGrab = opMode.hardwareMap.servo.get(CLAW_GRAB_SERVO);
        if(clawGrab == null) {
            return -1;
        }
        clawWrist.setPosition(0);
        clawGrab.setPosition(0);

        return 0;
    }

    public void setDriveTrainPower(double lf, double rf, double lb, double rb) {
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
    }

    public void moveArm (LinearOpMode opMode){
        double gamePad2RightStickY, gamePad2LeftStickX;
        gamePad2RightStickY = opMode.gamepad2.right_stick_y;
        gamePad2LeftStickX = opMode.gamepad2.left_stick_x;
        if(gamePad2RightStickY >= GAME_STICK_DEAD_ZONE && uADegrees < upperUADegrees) {
            if(uADegrees + (gamePad2RightStickY / 10) > upperUADegrees) {
                uADegrees = upperUADegrees;
            }
            else {
                uADegrees += gamePad2RightStickY / 10;
            }
            upperArm.setTargetPosition(degreesToEncoder(uADegrees, gearbox40 * 5));
            upperArm.setPower(0.5);
        }
        else if(gamePad2RightStickY <= -GAME_STICK_DEAD_ZONE && uADegrees > lowerUADegrees) {
            if(uADegrees + (gamePad2RightStickY / 10) < lowerUADegrees) {
                uADegrees = lowerUADegrees;
            }
            else {
                uADegrees += gamePad2RightStickY / 10;
            }
            upperArm.setTargetPosition(degreesToEncoder(uADegrees, gearbox40 * 5));
            upperArm.setPower(0.5);
        }
        moveArmServo(uADegrees);
        if(gamePad2LeftStickX >= GAME_STICK_DEAD_ZONE && tTDegrees < upperTTDegrees) {
            if(tTDegrees + (gamePad2LeftStickX / 10) > upperTTDegrees) {
                tTDegrees = upperTTDegrees;
            }
            else {
                tTDegrees += gamePad2LeftStickX / 10;
            }
            turnTable.setTargetPosition(degreesToEncoder(tTDegrees, gearbox40 * 2));
            turnTable.setPower(0.5);
        }
        else if(gamePad2LeftStickX <= -GAME_STICK_DEAD_ZONE && tTDegrees > lowerUADegrees) {
            if(tTDegrees + (gamePad2LeftStickX / 10) < lowerUADegrees) {
                tTDegrees = lowerTTDegrees;
            }
            else {
                tTDegrees += gamePad2LeftStickX / 10;
            }
            turnTable.setTargetPosition(degreesToEncoder(tTDegrees, gearbox40 * 2));
            turnTable.setPower(0.5);
        }
    }

    private int degreesToEncoder(double degrees, int gearRatio) {
        int encoderValue = (int)(degrees * gearRatio * encoderPulses);
        return encoderValue;
    }

    private void moveArmServo(double uADegrees) {

    }

    public void moveDriveTrain(LinearOpMode opMode) {
        double gamePad1LeftStickY, gamePad1LeftStickX, gamePad1RightStickX;
        double motorPowerLF, motorPowerLB, motorPowerRF,motorPowerRB;

        gamePad1LeftStickY = opMode.gamepad1.left_stick_y;
        gamePad1LeftStickX = opMode.gamepad1.left_stick_x;
        gamePad1RightStickX = opMode.gamepad1.right_stick_x;

        motorPowerLF = 0;
        motorPowerLB = 0;
        motorPowerRF = 0;
        motorPowerRB = 0;

        if ((Math.abs(gamePad1LeftStickY) > GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1LeftStickY) < -GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1LeftStickX) > GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1LeftStickX) < -GAME_STICK_DEAD_ZONE)) {
            motorPowerLF = -gamePad1LeftStickY - (-gamePad1LeftStickX);
            motorPowerLB = -gamePad1LeftStickY - gamePad1LeftStickX;
            motorPowerRF = gamePad1LeftStickY - (-gamePad1LeftStickX);
            motorPowerRB = gamePad1LeftStickY - gamePad1LeftStickX;
        }

        // Right joystick is for turning
        else if ((gamePad1RightStickX > GAME_STICK_DEAD_ZONE) ||
                (gamePad1RightStickX < -GAME_STICK_DEAD_ZONE)) {
            motorPowerLF = motorPowerLB = gamePad1RightStickX;
            motorPowerRF = motorPowerRB = gamePad1RightStickX;
        }

        setDriveTrainPower(motorPowerLF, motorPowerRF, motorPowerLB, motorPowerRB);
    }
}

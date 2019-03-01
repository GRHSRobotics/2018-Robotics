package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class HardwareDefinitions extends LinearOpMode{

    //SERVO POSITION CONSTANTS
    public final double opener1Closed = 1;
    public final double opener1Open = 0;

    public final double opener2Closed = 0;
    public final double opener2Open = 1;

    public final double markerDropperOuterHold = 0.5;
    public final double markerDropperOuterRelease = 0;

    public final double markerDropperInnerHold = 0.5;
    public final double markerDropperInnerRelease =  0;

    public final double landerLockHold = 0.5;
    public final double landerLockRelease = 1;





    //GYRO HEADING
    public double gyroHeading;

    //INSTANTIATE MOTORS
    public DcMotor motorL1;
    public DcMotor motorL2;
    public DcMotor motorR1;
    public DcMotor motorR2;
    public DcMotor liftMotor1;
    public DcMotor liftMotor2;
    public DcMotor landerMotor;

    //INSTANTIANTE TEAM MARKER SERVO
    public Servo markerDropperOuter;
    public Servo markerDropperInner;

    //INSTANTIATE OPENER SERVOS
    public Servo opener1;
    public Servo opener2;

    //INSTATIATE LANDER LOCK SERVO
    public Servo landerLock;


    //INSTANTIATE IMU
    public BNO055IMU imu;

    //INSTANTIATE LED CONTROLLER
    public RevBlinkinLedDriver LEDController;
    public RevBlinkinLedDriver.BlinkinPattern redLEDPattern;
    public RevBlinkinLedDriver.BlinkinPattern blueLEDPattern;
    public RevBlinkinLedDriver.BlinkinPattern endgameLEDPattern;
    public RevBlinkinLedDriver.BlinkinPattern autonLEDPattern;
/*
    //MAGNETIC LIMIT SWITCHES
    public DigitalChannel topLimit;
    public DigitalChannel bottomLimit;
*/
    //CREATE AND DEFINE NEW HardwareMap
    HardwareMap robotMap;
    public void init(HardwareMap robotMap){

        //DEFINE MOTORS
        motorL1 = robotMap.get(DcMotor.class, "motorL1");
        motorL2 = robotMap.get(DcMotor.class, "motorL2");
        motorR1 = robotMap.get(DcMotor.class, "motorR1");
        motorR2 = robotMap.get(DcMotor.class, "motorR2");
        liftMotor1 = robotMap.get(DcMotor.class, "liftMotor1");
        liftMotor2 = robotMap.get(DcMotor.class, "liftMotor2");
        landerMotor = robotMap.get(DcMotor.class, "landerMotor");

        //DEFINE TEAM MARKER SERVO
        markerDropperOuter = robotMap.get(Servo.class, "markerDropperOuter");
        markerDropperInner = robotMap.get(Servo.class, "markerDropperInner");

        //DEFINE OPENER SERVOS
        opener1 = robotMap.get(Servo.class, "opener1");
        opener2 = robotMap.get(Servo.class, "opener2");

        //DEFINE LANDER SERVO
        landerLock = robotMap.get(Servo.class, "landerLock");

        //DEFINE LED CONTROLLER
        LEDController = hardwareMap.get(RevBlinkinLedDriver.class, "LEDController");

/*
        //DEFINE MAGNETIC LIMIt SWITCHES
        topLimit = robotMap.get(DigitalChannel.class, "topLimitSwitch");
        bottomLimit = robotMap.get(DigitalChannel.class, "bottomLimitSwitch");
*/
        //SET MOTOR POWER TO 0
        motorL1.setPower(0);
        motorL2.setPower(0);
        motorR1.setPower(0);
        motorR2.setPower(0);
        liftMotor1.setPower(0);
        liftMotor2.setPower(0);

        // RESET MOTOR ENCODERS
        motorL1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        landerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // SET MOTOR MODE
        motorL1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorL2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        landerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // SET MOTOR ZeroPowerBehavior
        motorL1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        landerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //REVERSE RIGHT DRIVE MOTORS
        motorL1.setDirection(DcMotor.Direction.FORWARD);
        motorL2.setDirection(DcMotor.Direction.FORWARD);
        motorR1.setDirection(DcMotor.Direction.REVERSE);
        motorR2.setDirection(DcMotor.Direction.REVERSE);

        //SET TEAM MARKER SERVO START POSITION
        markerDropperOuter.setPosition(markerDropperOuterRelease);
        markerDropperInner.setPosition(markerDropperInnerHold);

/*
        //SET MAGNETIC LIMIT SWITCH MODES
        topLimit.setMode(DigitalChannel.Mode.INPUT);
        bottomLimit.setMode(DigitalChannel.Mode.INPUT);
*/

        telemetry.addData("Hardware Initialized", "");

    }

    public void initIMU(HardwareMap robotMap){ //we do this one separately so we don't waste time initializing the IMU for teleop when we don't need it
        //DEFINE REV HUB IMU
        imu = robotMap.get(BNO055IMU.class, "hub1imu");

        //SET IMU PARAMETERS
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        //INIT IMU
        imu.initialize(parameters);

        telemetry.addData("Mode", "Calibrating Gyro");

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());

        //put LED stuff here because it only gets called in autonomous
        autonLEDPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        LEDController.setPattern(autonLEDPattern);
    }

    public void stopRobot(){

        //END GAME, TURN OFF MOTORS
        motorL1.setPower(0);
        motorL2.setPower(0);
        motorR1.setPower(0);
        motorR2.setPower(0);
        liftMotor1.setPower(0);

    }


    public void setDriveEncoderMode(DcMotor.RunMode RunMode){

        //RESET ENCODERS
        motorL1.setMode(RunMode);
        motorL2.setMode(RunMode);
        motorR1.setMode(RunMode);
        motorR2.setMode(RunMode);

    }



    @Override
    public void runOpMode(){
        this.init(hardwareMap);
    }

}

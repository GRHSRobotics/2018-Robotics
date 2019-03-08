package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.teleop.IntakeTest;

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

    //INTAKE
    public final double INTAKE_P_COEFF = 2.25;
    public final double intakeUpPosition = 0.9; //upper limit as a reading of the potentiometer voltage
    public final double intakeDownPosition = 2.0; //lower limit as read from the potentiometer
    public final double intakeHingeMaxPowerUp = 1;
    public final double intakeHingeMaxPowerDown = 0.55;
    public final double intakeMinPower = 0.3;



    //GYRO HEADING
    public double gyroHeading;

    //INSTANTIATE MOTORS
    public DcMotor motorL1;
    public DcMotor motorR1;
    public DcMotor liftMotor1;
    public DcMotor liftMotor2;
    public DcMotor landerMotor;
    public DcMotor intakeActuator;
    public DcMotor intakeHinge;
    public DcMotor intakeSpinner;

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

    //INTAKE SENSORS
    public AnalogInput intakePotentiometer;
    public DigitalChannel intakeLimitSwitch;

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
        motorR1 = robotMap.get(DcMotor.class, "motorR1");
        liftMotor1 = robotMap.get(DcMotor.class, "liftMotor1");
        liftMotor2 = robotMap.get(DcMotor.class, "liftMotor2");
        landerMotor = robotMap.get(DcMotor.class, "landerMotor");
        intakeActuator = robotMap.get(DcMotor.class, "intakeActuator");
        intakeHinge = robotMap.get(DcMotor.class, "intakeHinge");
        intakeSpinner = robotMap.get(DcMotor.class, "intakeSpinner");

        //DEFINE TEAM MARKER SERVO
        markerDropperOuter = robotMap.get(Servo.class, "markerDropperOuter");
        markerDropperInner = robotMap.get(Servo.class, "markerDropperInner");

        //DEFINE OPENER SERVOS
        opener1 = robotMap.get(Servo.class, "opener1");
        opener2 = robotMap.get(Servo.class, "opener2");

        //DEFINE LANDER SERVO
        landerLock = robotMap.get(Servo.class, "landerLock");

        //INTAKE SENSORS
        intakePotentiometer = robotMap.get(AnalogInput.class, "intakePotentiometer");
        intakeLimitSwitch = robotMap.get(DigitalChannel.class, "intakeLimitSwitch");

        //DEFINE LED CONTROLLER
        LEDController = hardwareMap.get(RevBlinkinLedDriver.class, "LEDController");

/*
        //DEFINE MAGNETIC LIMIt SWITCHES
        topLimit = robotMap.get(DigitalChannel.class, "topLimitSwitch");
        bottomLimit = robotMap.get(DigitalChannel.class, "bottomLimitSwitch");
*/
        //SET MOTOR POWER TO 0
        motorL1.setPower(0);
        motorR1.setPower(0);
        liftMotor1.setPower(0);
        liftMotor2.setPower(0);

        // RESET MOTOR ENCODERS
        motorL1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        landerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeHinge.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // SET MOTOR MODE
        motorL1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        landerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeActuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeHinge.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // SET MOTOR ZeroPowerBehavior
        motorL1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        landerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeHinge.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //REVERSE RIGHT DRIVE MOTORS
        motorL1.setDirection(DcMotor.Direction.FORWARD);
        motorR1.setDirection(DcMotor.Direction.REVERSE);

        //SET OTHER MOTOR DIRECTIONS
        liftMotor1.setDirection(DcMotor.Direction.FORWARD);
        liftMotor2.setDirection(DcMotor.Direction.FORWARD);
        landerMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeActuator.setDirection(DcMotor.Direction.REVERSE);
        intakeHinge.setDirection(DcMotor.Direction.REVERSE);
        intakeSpinner.setDirection(DcMotor.Direction.FORWARD);

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
        motorR1.setPower(0);
        liftMotor1.setPower(0);

    }


    public void setDriveEncoderMode(DcMotor.RunMode RunMode){

        //RESET ENCODERS
        motorL1.setMode(RunMode);
        motorR1.setMode(RunMode);

    }

    public enum HingePosition{
        UP,
        DOWN
    }

    public double getIntakePower(HingePosition position){
        double intakeHingeMaxPower;
        double targetPosition;
        if(position == HingePosition.DOWN){
            intakeHingeMaxPower = intakeHingeMaxPowerDown;
            targetPosition = intakeDownPosition;
        } else {
            intakeHingeMaxPower = intakeHingeMaxPowerUp;
            targetPosition = intakeUpPosition;
        }

        int powerSign; //whether the power is positive or negative
        if(targetPosition > intakePotentiometer.getVoltage()){
            powerSign = -1;
        } else {
            powerSign = 1;
        }


        //what percent of the total path the arm is from the target position
        double error = Math.abs((targetPosition-intakePotentiometer.getVoltage()) / (intakeDownPosition - intakeUpPosition));

        //final modifier to the intake power
        double adjustment = Range.clip(error * powerSign * INTAKE_P_COEFF, -1, 1);
        /*
        if(position == HingePosition.UP){
            adjustment = Range.clip(error * powerSign * INTAKE_P_COEFF, -1, 1);
        } else {
            adjustment = Range.clip(error * powerSign * INTAKE_P_COEFF, -1, 1);
            if(adjustment < intakeMinPower && adjustment > 0){
                adjustment = intakeMinPower * powerSign;
            }
        }
    */

        if(position == HingePosition.DOWN){
            adjustment /= 2;
        }

        //only runs motor if predicted power is above a certain threshold to prevent stalling
        if(Math.abs(adjustment * intakeHingeMaxPower) > 0.1){
            return adjustment * intakeHingeMaxPower;
        } else{
            return 0;
        }
    }


    @Override
    public void runOpMode(){
        this.init(hardwareMap);
    }

}

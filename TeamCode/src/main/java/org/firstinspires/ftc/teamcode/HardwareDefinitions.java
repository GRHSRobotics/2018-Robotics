package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class HardwareDefinitions {

    //CONSTANTS
    static final double COUNTS_PER_ROTATION = 1120 ;    // REV HD Hex Motor 40:1
    static final double WHEEL_CIRCUMFERENCE_INCHES = 9.42 ;
    static final double COUNTS_PER_INCH = (COUNTS_PER_ROTATION) / (WHEEL_CIRCUMFERENCE_INCHES);
    static final double ROBOT_DIAMETER = 18; //in inches

    // OLD AUTONOMOUS CONSTANTS
/*    int toTheBalls;
    int ninetyDegreeTurn;

    int crater_toTheWall;
    int crater_setParallelToWall;
    int crater_toTheDepot;

    int depot_toTheWall;
    int depot_setParallelToWall;
    int depot_toTheDepot;
*/
    //SERVO POSITION CONSTANTS
    double opener1Closed = 1;
    double opener1Open = 0;

    double opener2Closed = 0;
    double opener2Open = 1;

    double markerDropperBack = 1;
    double markerDropperForward = 0;

    //INSTANTIATE MOTORS
    public DcMotor motorL1;
    public DcMotor motorL2;
    public DcMotor motorR1;
    public DcMotor motorR2;
    public DcMotor intakeMotor;
    public DcMotor liftMotor;

    //INSTANTIANTE TEAM MARKER SERVO
    public Servo markerDropper;

    //INSTANTIATE OPENER SERVOS
    public Servo opener1;
    public Servo opener2;

    //CREATE AND DEFINE NEW HardwareMap
    HardwareMap robotMap;
    public void init(HardwareMap robotMap){

        //DEFINE MOTORS
        motorL1 = robotMap.get(DcMotor.class, "motorL1");
        motorL2 = robotMap.get(DcMotor.class, "motorL2");
        motorR1 = robotMap.get(DcMotor.class, "motorR1");
        motorR2 = robotMap.get(DcMotor.class, "motorR2");
        intakeMotor = robotMap.get(DcMotor.class, "intakeMotor");
        liftMotor = robotMap.get(DcMotor.class, "liftMotor");

        //DEFINE TEAM MARKER SERVO
        markerDropper = robotMap.get(Servo.class, "markerDropper");

        //DEFINE OPENER SERVOS
        opener1 = robotMap.get(Servo.class, "opener1");
        opener2 = robotMap.get(Servo.class, "opener2");

        //SET MOTOR POWER TO 0
        motorL1.setPower(0);
        motorL2.setPower(0);
        motorR1.setPower(0);
        motorR2.setPower(0);
        intakeMotor.setPower(0);
        liftMotor.setPower(0);

        // RESET MOTOR ENCODERS
        motorL1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // SET MOTOR MODE
        motorL1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorL2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // SET MOTOR ZeroPowerBehavior
        motorL1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //REVERSE LEFT DRIVE MOTORS
        motorL1.setDirection(DcMotor.Direction.FORWARD);
        motorL2.setDirection(DcMotor.Direction.FORWARD);
        motorR1.setDirection(DcMotor.Direction.REVERSE);
        motorR2.setDirection(DcMotor.Direction.REVERSE);

        //SET TEAM MARKER SERVO START POSITION
        markerDropper.setPosition(markerDropperBack);

        //SET SERVO START POSITIONS
        opener1.setPosition(opener1Closed);
        opener2.setPosition(opener2Closed);

    }

    public void stop(){

        //END GAME, TURN OFF MOTORS
        motorL1.setPower(0);
        motorL2.setPower(0);
        motorR1.setPower(0);
        motorR2.setPower(0);
        intakeMotor.setPower(0);
        liftMotor.setPower(0);

        //SET OPENER SERVOS TO DEFAULT POSITIONS
        opener1.setPosition(opener1Closed);
        opener2.setPosition(opener2Closed);

    }

    public void setDriveEncoderMode(DcMotor.RunMode RunMode){

        //RESET ENCODERS
        motorL1.setMode(RunMode);
        motorL2.setMode(RunMode);
        motorR1.setMode(RunMode);
        motorR2.setMode(RunMode);

    }

}

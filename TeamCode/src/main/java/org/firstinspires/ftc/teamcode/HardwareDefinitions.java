package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class HardwareDefinitions extends LinearOpMode{

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
    public final double opener1Closed = 1;
    public final double opener1Open = 0;

    public final double opener2Closed = 0;
    public final double opener2Open = 1;

    public final double markerDropperBack = 1;
    public final double markerDropperForward = 0;

    //GYRO HEADING
    public double gyroHeading;

    //INSTANTIATE MOTORS
    public DcMotor motorL1;
    public DcMotor motorL2;
    public DcMotor motorR1;
    public DcMotor motorR2;
    public DcMotor intakeMotor;
    public DcMotor liftMotor;
    public DcMotor landerMotor1;
    public DcMotor landerMotor2;

    //INSTANTIANTE TEAM MARKER SERVO
    public Servo markerDropper;

    //INSTANTIATE OPENER SERVOS
    public Servo opener1;
    public Servo opener2;

    //INSTANTIATE IMU
    public BNO055IMU imu;

    //MAGNETIC LIMIT SWITCHES
    public DigitalChannel topLimit;
    public DigitalChannel bottomLimit;

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
        landerMotor1 = robotMap.get(DcMotor.class, "landerMotor1");
        landerMotor2 = robotMap.get(DcMotor.class, "landerMotor2");

        //DEFINE TEAM MARKER SERVO
        markerDropper = robotMap.get(Servo.class, "markerDropper");

        //DEFINE OPENER SERVOS
        opener1 = robotMap.get(Servo.class, "opener1");
        opener2 = robotMap.get(Servo.class, "opener2");

        //DEFINE REV HUB IMU
        imu = robotMap.get(BNO055IMU.class, "hub4imu");

        //DEFINE MAGNETIC LIMIt SWITCHES
        topLimit = robotMap.get(DigitalChannel.class, "topLimitSwitch");
        bottomLimit = robotMap.get(DigitalChannel.class, "bottomLimitSwitch");

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
        landerMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        landerMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // SET MOTOR MODE
        motorL1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorL2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        landerMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        landerMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // SET MOTOR ZeroPowerBehavior
        motorL1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        landerMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        landerMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //REVERSE RIGHT DRIVE MOTORS
        motorL1.setDirection(DcMotor.Direction.FORWARD);
        motorL2.setDirection(DcMotor.Direction.FORWARD);
        motorR1.setDirection(DcMotor.Direction.REVERSE);
        motorR2.setDirection(DcMotor.Direction.REVERSE);

        //SET TEAM MARKER SERVO START POSITION
        markerDropper.setPosition(markerDropperBack);

        //SET SERVO START POSITIONS
        opener1.setPosition(opener1Closed);
        opener2.setPosition(opener2Closed);

        //SET MAGNETIC LIMIT SWITCH MODES
        topLimit.setMode(DigitalChannel.Mode.INPUT);
        bottomLimit.setMode(DigitalChannel.Mode.INPUT);

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
        telemetry.addData("Mode", "Waiting for Start");

    }

    public void stopRobot(){

        //END GAME, TURN OFF MOTORS
        motorL1.setPower(0);
        motorL2.setPower(0);
        motorR1.setPower(0);
        motorR2.setPower(0);
        intakeMotor.setPower(0);
        liftMotor.setPower(0);

        //SET OPENER SERVOS TO DEFAULT POSITIONS
        opener1.setPosition(opener1Open);
        opener2.setPosition(opener2Open);

    }


    public void setDriveEncoderMode(DcMotor.RunMode RunMode){

        //RESET ENCODERS
        motorL1.setMode(RunMode);
        motorL2.setMode(RunMode);
        motorR1.setMode(RunMode);
        motorR2.setMode(RunMode);

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double maxTimeS) {


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            ElapsedTime runtime = new ElapsedTime();

            // Determine new target position
            int newL1Target = motorL1.getCurrentPosition() + (int) Math.round(leftInches * COUNTS_PER_INCH);
            int newL2Target = motorL2.getCurrentPosition() + (int) Math.round(leftInches * COUNTS_PER_INCH);
            int newR1Target = motorR1.getCurrentPosition() + (int) Math.round(rightInches * COUNTS_PER_INCH);
            int newR2Target = motorR2.getCurrentPosition() + (int) Math.round(rightInches * COUNTS_PER_INCH);

            //give new target position to motors
            motorL1.setTargetPosition(newL1Target);
            motorL2.setTargetPosition(newL2Target);
            motorR1.setTargetPosition(newR1Target);
            motorR2.setTargetPosition(newR2Target);

            // Turn On RUN_TO_POSITION
            motorL1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorL2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorL1.setPower(Math.abs(speed));
            motorL2.setPower(Math.abs(speed));
            motorR1.setPower(Math.abs(speed));
            motorR2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stopRobot.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < maxTimeS) &&
                    (motorL1.isBusy() && motorL2.isBusy() && motorR1.isBusy() && motorR2.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newL1Target, newL2Target, newR1Target, newR2Target);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        motorL1.getCurrentPosition(),
                        motorL2.getCurrentPosition(),
                        motorR1.getCurrentPosition(),
                        motorR2.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorL1.setPower(0);
            motorL2.setPower(0);
            motorR1.setPower(0);
            motorR2.setPower(0);

            // Turn off RUN_TO_POSITION
            motorL1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorL2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorR2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(400);
        }
    }

    public void encoderTurn(double speed, double angle, boolean clockwise, double maxTimeS){

        //determine inches to travel for each side using formula (pi * ROBOT_DIAMETER * angle)/360
        double leftInches = (ROBOT_DIAMETER * angle * Math.PI) / 360;
        double rightInches = (ROBOT_DIAMETER * angle * Math.PI) / 360;

        //Make sure that each motor is moving the right way
        if(clockwise){ //if the robot is turning clockwise, the left motors need to turn backwards
            leftInches = -leftInches;
        } else{ //if the robot is turning counterclockwise, the right motors need to turn backwards
            rightInches = -rightInches;
        }

        //plug the tread movement distances into the encoderDrive method
        encoderDrive(speed, leftInches, rightInches, maxTimeS);
    }

    public void gyroTurnAbsolute(double speed, double angle, double maxTimeS){

        ElapsedTime runtime = new ElapsedTime();

        gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        motorL1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runtime.reset();

        gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        while(opModeIsActive() && (Math.abs(gyroHeading - angle) > 3) && (runtime.seconds() < maxTimeS)){

            if(angle > 0){ //if turn is left
                motorL1.setPower(-0.25);
                motorL2.setPower(-0.25);
                motorR1.setPower(0.25);
                motorR2.setPower(0.25);
            }
            else if(angle < 0){ //if turn is right
                motorL1.setPower(0.25);
                motorL2.setPower(0.25);
                motorR1.setPower(-0.25);
                motorR2.setPower(-0.25);
            }

            //update gyro position variable
            gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        }

        //turn off motor power when turn is done
        motorL1.setPower(0);
        motorL2.setPower(0);
        motorR1.setPower(0);
        motorR2.setPower(0);

        sleep(250);

    }

    public void gyroTurn(double speed, double target, double maxTimeS){
        gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        gyroTurnAbsolute(speed, (gyroHeading + target), maxTimeS);
    }

    public void dropFromLander(){

        boolean bottomReached = false;

        while (opModeIsActive() && !bottomReached){
            landerMotor1.setPower(0.1);
            landerMotor2.setPower(0.1);

            if(bottomLimit.getState()){
                bottomReached = true;
            }
        }

        //add stuff to undo the hook thing

    }

    @Override
    public void runOpMode(){
        this.init(hardwareMap);
    }

}

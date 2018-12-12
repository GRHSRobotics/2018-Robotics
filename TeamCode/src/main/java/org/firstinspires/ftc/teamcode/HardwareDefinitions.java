package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class HardwareDefinitions extends LinearOpMode{

    //CONSTANTS
    static final double COUNTS_PER_ROTATION = 560 ;    // REV HD Hex Motor 20:1
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

    public final double markerDropperOuterHold = 0.5;
    public final double markerDropperOuterRelease = 0;

    public final double markerDropperInnerHold = 0.5;
    public final double markerDropperInnerRelease =  0;
/*
    //GYRO HEADING
    public double gyroHeading;
*/
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
    public Servo markerDropperOuter;
    public Servo markerDropperInner;

    //INSTANTIATE OPENER SERVOS
    public Servo opener1;
    public Servo opener2;

/*
    //INSTANTIATE IMU
    public BNO055IMU imu;
*/
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
        intakeMotor = robotMap.get(DcMotor.class, "intakeMotor");
        liftMotor = robotMap.get(DcMotor.class, "liftMotor");
        landerMotor1 = robotMap.get(DcMotor.class, "landerMotor1");
        landerMotor2 = robotMap.get(DcMotor.class, "landerMotor2");

        //DEFINE TEAM MARKER SERVO
        markerDropperOuter = robotMap.get(Servo.class, "markerDropperOuter");
        markerDropperInner = robotMap.get(Servo.class, "markerDropperInner");

        //DEFINE OPENER SERVOS
        opener1 = robotMap.get(Servo.class, "opener1");
        opener2 = robotMap.get(Servo.class, "opener2");
/*
        //DEFINE REV HUB IMU
        imu = robotMap.get(BNO055IMU.class, "hub4imu");
*/
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
        landerMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        landerMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        markerDropperOuter.setPosition(markerDropperOuterRelease);
        markerDropperInner.setPosition(markerDropperInnerHold);

        //SET SERVO START POSITIONS
        opener1.setPosition(opener1Closed);
        opener2.setPosition(opener2Closed);
/*
        //SET MAGNETIC LIMIT SWITCH MODES
        topLimit.setMode(DigitalChannel.Mode.INPUT);
        bottomLimit.setMode(DigitalChannel.Mode.INPUT);
*/
/*
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
*/
    }

    public void stopRobot(){

        //END GAME, TURN OFF MOTORS
        motorL1.setPower(0);
        motorL2.setPower(0);
        motorR1.setPower(0);
        motorR2.setPower(0);
        intakeMotor.setPower(0);
        liftMotor.setPower(0);

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

            motorL1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            rightInches = -rightInches;
        } else{ //if the robot is turning counterclockwise, the right motors need to turn backwards
            leftInches = -leftInches;
        }

        //plug the tread movement distances into the encoderDrive method
        encoderDrive(speed, leftInches, rightInches, maxTimeS);
    }

    public void dropFromLander(){

        moveLanderWithEncoder(9, 8);

        encoderTurn(0.4, 30, true, 5);

        encoderDrive(0.4, -2, -2, 5);

        encoderTurn(0.4, 30, false, 5);

        telemetry.addData("Landing sequence:", "Complete");

    }

    public void moveLanderWithEncoder(double rotations, double maxTimeS){ //rotations should be 3.63
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            int counts_per_rotation = 288; //using Core Hex Motors
            double rotationsUp = rotations; //modify this one, will most likely be the same as rotationsDown

            ElapsedTime landerRuntime = new ElapsedTime();

            int targetPositionUp1 = landerMotor1.getCurrentPosition() + (int)(counts_per_rotation * rotationsUp);
            int targetPositionUp2 = landerMotor2.getCurrentPosition() + (int)(counts_per_rotation * rotationsUp);

            landerMotor1.setTargetPosition(targetPositionUp1);
            landerMotor2.setTargetPosition(targetPositionUp2);

            // Turn On RUN_TO_POSITION
            landerMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            landerMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            landerRuntime.reset();
            landerMotor1.setPower(0.4);
            landerMotor2.setPower(0.4);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stopRobot.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (landerMotor1.isBusy() && landerMotor2.isBusy())
                    && landerRuntime.seconds() < maxTimeS) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", targetPositionUp1, targetPositionUp2);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        landerMotor1.getCurrentPosition(),
                        landerMotor2.getCurrentPosition()
                );
                telemetry.update();
            }

            // Stop all motion;
            landerMotor1.setPower(0);
            landerMotor2.setPower(0);

            sleep(400);

        }
    }

    @Override
    public void runOpMode(){
        this.init(hardwareMap);
    }

}

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;

//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.GyroSensor;


//See Google Drive for TODO

@TeleOp(name="DriverControlled2", group="Teleop")
public class DriverControlled2 extends HardwareDefinitions{

    //LIFT SERVO VARIABLES
    boolean opener1Changed = false;
    boolean opener1On = false;

    boolean opener2Changed = false;
    boolean opener2On = false;

    //TANK DRIVE VARIABLES
    double rightPower;
    double leftPower;


    @Override
    public void runOpMode(){
        init(hardwareMap);
        telemetry.addData("Robot is initialized", "");

        waitForStart();
        telemetry.addData("Robot is started", "" );

        landerMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        landerMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while(opModeIsActive()){

            //TANK DRIVE

            rightPower = Range.clip(Math.pow(-gamepad1.right_stick_y, 3), -1, 1);
            leftPower = Range.clip(Math.pow(-gamepad1.left_stick_y, 3), -1, 1);

            motorL1.setPower(leftPower); //up on the stick is negative, so for up=forwards we need to
            motorL2.setPower(leftPower); //take the negative value of the stick
            motorR1.setPower(rightPower);
            motorR2.setPower(rightPower);

            //INTAKE SYSTEM TOGGLE
            if(gamepad1.dpad_up){
                intakeMotor.setPower(-1); //INTAKE
            }
            if(gamepad1.dpad_down){
                intakeMotor.setPower(0); //TURN OFF
            }
            if(gamepad1.dpad_left){
                intakeMotor.setPower(1); //REVERSE
            }
/*
            //MARKER DROPPER SYSTEM
            if(gamepad1.x){
                markerDropperOuter.setPosition(markerDropperOuterHold);
            }
            if(gamepad1.y){
                markerDropperOuter.setPosition(markerDropperOuterRelease);
            }
*/
            //LIFT MOTOR
            if(gamepad1.left_trigger > 0){
                liftMotor.setPower(-Range.clip(gamepad1.left_trigger, 0, 1)); //clip method limits value to between 0 and 1
            } else if(gamepad1.right_trigger > 0){
                liftMotor.setPower(Range.clip(gamepad1.right_trigger, 0, 1));
            } else {
                liftMotor.setPower(0);
            }

            //DOOR OPENER 1 SYSTEM
            if(gamepad1.left_bumper && !opener1Changed) {
                opener1.setPosition(opener1On ? opener1Open : opener1Closed);
                opener1On = !opener1On;
                opener1Changed = true;
            } else if(!gamepad1.left_bumper) {
                opener1Changed = false;
            }

            //DOOR OPENER 2 SYSTEM
            if(gamepad1.right_bumper && !opener2Changed) {
                opener2.setPosition(opener2On ? opener2Open : opener2Closed); //this is a shorthand if/else statement, (condition ? if true : if false)
                opener2On = !opener2On;
                opener2Changed = true;
            } else if(!gamepad1.right_bumper) {
                opener2Changed = false;
            }

            //LANDING SYSTEM
            if(gamepad1.x){
                landerMotor1.setPower(-1); //bring the lift down
                landerMotor2.setPower(-1);
            } else if(gamepad1.y){
                landerMotor1.setPower(1); //bring the lift up
                landerMotor2.setPower(1);
            } else {
                landerMotor1.setPower(0); //stop the lift
                landerMotor2.setPower(0);
            }

            if(gamepad1.dpad_right){
                dropFromLander();
            }
        }
        stopRobot();
        telemetry.addData("Robot is stopped", "" );
    }

    public void dropFromLander(){


        landerMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        landerMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int counts_per_rotation = 288; //using Core Hex Motors
        double rotationsUp = 3.63; //modify this one, will most likely be the same as rotationsDown

        int targetPositionUp1 = landerMotor1.getCurrentPosition() + (int)(counts_per_rotation * rotationsUp);
        int targetPositionUp2 = landerMotor2.getCurrentPosition() + (int)(counts_per_rotation * rotationsUp);

        landerMotor1.setTargetPosition(targetPositionUp1);
        landerMotor2.setTargetPosition(targetPositionUp2);

        landerMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        landerMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //raise the lift and undo the passive latch
        landerMotor1.setPower(0.4);
        landerMotor2.setPower(0.4);

        while(opModeIsActive() && (landerMotor1.isBusy() || landerMotor2.isBusy())){
            //let the motors keep spinning
        }

        landerMotor1.setPower(0);
        landerMotor2.setPower(0);

        encoderTurn(0.4, 45, true, 5);

        //bring the lift back down
        double rotationsDown = 3.63; //modify this one

        int targetPositionDown1 = landerMotor1.getCurrentPosition() - (int)(counts_per_rotation * rotationsDown);
        int targetPositionDown2 = landerMotor2.getCurrentPosition() - (int)(counts_per_rotation * rotationsDown);


        landerMotor1.setTargetPosition(targetPositionDown1);
        landerMotor2.setTargetPosition(targetPositionDown2);

        landerMotor1.setPower(0.4);
        landerMotor2.setPower(0.4);

        while(opModeIsActive() && (landerMotor1.isBusy() || landerMotor2.isBusy())){
            //let the motors keep spinning
        }

        landerMotor1.setPower(0);
        landerMotor2.setPower(0);

        encoderTurn(0.4, 45, false, 5);

        telemetry.addData("Landing sequence:", "Complete");

    }
}

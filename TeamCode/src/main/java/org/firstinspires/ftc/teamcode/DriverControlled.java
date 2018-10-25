package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

//See Google Drive for TODO

@TeleOp(name="DriverControlled", group="Teleop")
public class DriverControlled extends OpMode{

    //LIFT SERVO VARIABLES
    boolean opener1Changed = false;
    boolean opener1On = false;

    boolean opener2Changed = false;
    boolean opener2On = false;

    //DEFINE ROBOT HARDWARE
    HardwareDefinitions robot = new HardwareDefinitions();


    @Override
    public void init(){
        robot.init(hardwareMap);
        telemetry.addData("Robot is initialized", "");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        telemetry.addData("Robot is started", "" );
    }

    @Override
    public void loop(){

        //TANK DRIVE
        robot.motorL1.setPower(-gamepad1.right_stick_y);
        robot.motorL2.setPower(-gamepad1.right_stick_y);
        robot.motorR1.setPower(-gamepad1.left_stick_y);
        robot.motorR2.setPower(-gamepad1.left_stick_y);

        //INTAKE SYSTEM TOGGLE
        if(gamepad1.dpad_up){
            robot.intakeMotor.setPower(-1); //INKAKE
        }
        if(gamepad1.dpad_down){
            robot.intakeMotor.setPower(0); //TURN OFF
        }
        if(gamepad1.dpad_left){
            robot.intakeMotor.setPower(1); //REVERSE
        }

        //MARKER DROPPER SYSTEM
        if(gamepad1.x){
            robot.markerDropper.setPosition(robot.markerDropperBack);
        }
        if(gamepad1.y){
            robot.markerDropper.setPosition(robot.markerDropperForward);
        }

        //LIFT MOTOR
        if(gamepad1.left_trigger > 0){
            robot.liftMotor.setPower(-Range.clip(gamepad1.left_trigger, 0, 1)); //clip method limits value to between 0 and 1
        } else if(gamepad1.right_trigger > 0){
            robot.liftMotor.setPower(Range.clip(gamepad1.right_trigger, 0, 1));
        } else {
            robot.liftMotor.setPower(0);
        }

        //DOOR OPENER 1 SYSTEM
        if(gamepad1.left_bumper && !opener1Changed) {
            robot.opener1.setPosition(opener1On ? robot.opener1Open : robot.opener1Closed);
            opener1On = !opener1On;
            opener1Changed = true;
        } else if(!gamepad1.left_bumper) {
            opener1Changed = false;
        }

        //DOOR OPENER 2 SYSTEM
        if(gamepad1.right_bumper && !opener2Changed) {
            robot.opener2.setPosition(opener2On ? robot.opener2Open : robot.opener2Closed); //this is a shorthand if/else statement, (condition ? if true : if false)
            opener2On = !opener2On;
            opener2Changed = true;
        } else if(!gamepad1.right_bumper) {
            opener2Changed = false;
        }

    }

    @Override
    public void stop() {
        robot.stop();
        telemetry.addData("Robot is stopped", "" );
    }
}

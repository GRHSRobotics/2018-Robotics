package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;

//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.GyroSensor;


//See Google Drive for TODO

@TeleOp(name="DriverControlled", group="Teleop")
public class DriverControlled extends HardwareDefinitions{

    //LIFT SERVO VARIABLES
    boolean opener1Changed = false;
    boolean opener1On = false;

    boolean opener2Changed = false;
    boolean opener2On = false;


    @Override
    public void runOpMode(){
        init(hardwareMap);
        telemetry.addData("Robot is initialized", "");

        waitForStart();
        telemetry.addData("Robot is started", "" );


        while(opModeIsActive()){
        //TANK DRIVE
        motorL1.setPower(-gamepad1.right_stick_y); //up on the stick is negative, so for up=forwards we need to
        motorL2.setPower(-gamepad1.right_stick_y); //take the negative value of the stick
        motorR1.setPower(-gamepad1.left_stick_y);
        motorR2.setPower(-gamepad1.left_stick_y);

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

        //MARKER DROPPER SYSTEM
        if(gamepad1.x){
            markerDropper.setPosition(markerDropperBack);
        }
        if(gamepad1.y){
            markerDropper.setPosition(markerDropperForward);
        }

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

    }
        stopRobot();
        telemetry.addData("Robot is stopped", "" );
    }
}

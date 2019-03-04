package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;

@TeleOp(name = "IntakeTest", group = "Test")
public class IntakeTest extends HardwareDefinitions {

    @Override
    public void runOpMode(){

        init(hardwareMap);
        telemetry.addData("Robot is initialized", "");

        waitForStart();
        telemetry.addData("Robot is started", "" );


        while(opModeIsActive()){


            if(gamepad1.x){

            }

            if(gamepad1.y){

            }

            if(gamepad1.a){
            }

            if(gamepad1.b){
            }

            if(gamepad1.right_trigger > 0){
                intakeActuator.setPower(1);
            } else if(gamepad1.right_bumper){
                intakeActuator.setPower(-1);
            } else{
                intakeActuator.setPower(0);
            }

            if(gamepad1.left_trigger > 0){
                intakeHinge.setPower(1);
            } else if (gamepad1.left_bumper){
                intakeHinge.setPower(-1);
            } else{
                intakeHinge.setPower(0);
            }

            if(gamepad1.dpad_down){
                intakeSpinner.setPower(-1);
            }
            if(gamepad1.dpad_up){
                intakeSpinner.setPower(1);
            }
            if(gamepad1.dpad_right){
                intakeSpinner.setPower(0);
            }

            telemetry.update();

        }


    }


}

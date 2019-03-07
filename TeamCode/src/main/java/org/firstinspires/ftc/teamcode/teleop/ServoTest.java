package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "ServoTest", group = "Test")
public class ServoTest extends HardwareDefinitions {


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

            if(gamepad1.dpad_down){
            }
            if(gamepad1.dpad_up){
            }
            if(gamepad1.dpad_right){
            }

            telemetry.update();

        }


    }


}

package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

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

                markerDropperInner.setPosition(markerDropperInnerHold);
            }

            if(gamepad1.y){

                markerDropperInner.setPosition(markerDropperInnerRelease);
            }

            if(gamepad1.dpad_left){

                markerDropperOuter.setPosition(markerDropperOuterHold);
            }

            if(gamepad1.dpad_up){

                markerDropperOuter.setPosition(markerDropperOuterRelease);
            }


        }


    }


}

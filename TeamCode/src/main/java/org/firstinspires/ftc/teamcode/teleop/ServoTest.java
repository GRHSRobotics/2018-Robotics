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

                landerLock.setPosition(landerLockHold);
                telemetry.addData("position:", landerLockHold);
            }

            if(gamepad1.y){

                landerLock.setPosition(landerLockRelease);
                telemetry.addData("position: ", landerLockRelease);
            }

            if(gamepad1.dpad_left){


            }

            if(gamepad1.dpad_up){

                markerDropperOuter.setPosition(markerDropperOuterRelease);
            }

            if(gamepad1.a){
                markerDropperOuter.setPosition(markerDropperOuterRelease);
                sleep(1000);
                markerDropperInner.setPosition(markerDropperInnerRelease);
                sleep(1500);
                markerDropperInner.setPosition(markerDropperInnerHold);
                sleep(1000);
                markerDropperOuter.setPosition(markerDropperOuterHold);
            }


        }


    }


}

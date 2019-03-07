package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousDefinitions;

@Disabled
@TeleOp(name="DetectGoldTest", group="Test")
public class DetectGoldTest extends AutonomousDefinitions {

    @Override
    public void runOpMode() {

        initTFodAndVuforia();
        telemetry.update();
        telemetry.addData("Robot is initialized", "");
        telemetry.update();



        waitForStart();
        telemetry.addData("Robot is started", "");
        telemetry.update();

        detectGold(inferMineral.RIGHT, 10000);

/*
        if(gamepad1.a){
            telemetry.addData("Standard detectGold method:", "getLeft()");
            telemetry.update();
            sleep(3000);
            detectGold(TFLiteHandler.inferMineral.RIGHT, 100000);

        }
        if(gamepad1.b){
            telemetry.addData("Alternate detectGold method:", "getBottom()");
            telemetry.update();
            sleep(3000);
            detectGoldAlternate(TFLiteHandler.inferMineral.RIGHT, 100000);

        }
*/
    }





}

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;

@Disabled
@TeleOp(name="LEDTest", group="Test")
public class LEDTest extends HardwareDefinitions {

    @Override
    public void runOpMode(){

        init(hardwareMap);
        telemetry.addData("Robot is initialized", "");
        telemetry.update();

        waitForStart();
        telemetry.addData("Robot is started", "" );
        telemetry.update();

        while(opModeIsActive()){

            //dpad has all the crazy patterns
            if(gamepad1.dpad_up){
                LEDController.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
                telemetry.addData("LED Pattern:", "Confetti");
            }
            if(gamepad1.dpad_left){
                LEDController.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
                telemetry.addData("LED Pattern:", "COLOR_WAVES_LAVA_PALETTE");
            }
            if(gamepad1.dpad_right){
                LEDController.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE);
                telemetry.addData("LED Pattern:", "BEATS_PER_MINUTE_FOREST_PALETTE");
            }
            if(gamepad1.dpad_down){
                LEDController.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
                telemetry.addData("LED Pattern:", "SINELON_OCEAN_PALETTE");
            }

            //front buttons
            if(gamepad1.a){
                LEDController.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_MEDIUM);
                telemetry.addData("LED Pattern:", "FIRE_MEDIUM");
            }
            if(gamepad1.b){
                LEDController.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                telemetry.addData("LED Pattern:", "HOT_PINK");
            }
            if(gamepad1.x){
                LEDController.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);
                telemetry.addData("LED Pattern:", "LARSON_SCANNER_RED");
            }
            if(gamepad1.y){
                LEDController.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
                telemetry.addData("LED Pattern:", "SHOT_RED");
            }

            //compare red and blue shades
            if(gamepad1.right_bumper){
                LEDController.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                telemetry.addData("LED Pattern:", "RED");
            }
            if(gamepad1.right_trigger > 0){
                LEDController.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
                telemetry.addData("LED Pattern:", "DARK_RED");
            }
            if(gamepad1.left_bumper){
                LEDController.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                telemetry.addData("LED Pattern:", "BLUE");
            }
            if(gamepad1.left_trigger > 0){
                LEDController.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
                telemetry.addData("LED Pattern:", "DARK_BLUE");

            }

            //make all the messages show up
            telemetry.update();

            //slow down the loop to limit accidental double presses
            sleep(50);
        }
    }
}

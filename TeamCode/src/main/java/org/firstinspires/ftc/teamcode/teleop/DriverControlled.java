package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.teamcode.HardwareDefinitions;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

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

    //TANK DRIVE VARIABLES
    double rightPower;
    double leftPower;

    // LED STUFF
    boolean switchedToEndgame = false;
    ElapsedTime timer = new ElapsedTime();
    boolean LEDChanged = false;
    boolean LEDOn = false; //don't worry about the awful variable names, it just works

    //TTS
    int randomLine;


    @Override
    public void runOpMode(){


        List<String> pickupLines = new ArrayList<>();

        pickupLines.add("Are you a judge, because I really hope I’m good enough for you");
        pickupLines.add("Are you our alliance partner, because I really hope you don’t suck");
        pickupLines.add("Are you the Engineering Notebook, because I’d stay up until 2am doing you");
        pickupLines.add("Are you this year’s robot, because you make me have a mental breakdown");
        pickupLines.add("I’m like our robot, because my nuts drop whenever I’m around you");
        pickupLines.add("Are you FRC, because I can’t afford you");
        pickupLines.add("Are you a mecanum wheel, because I’d stick my shaft right in you");
        pickupLines.add("Hey girl are you our robot because you keep falling apart but I love you anyways");
        pickupLines.add("Hey girl are you the permission slip because nobody is going to do you");
        pickupLines.add("Are you our lift system because I can’t get it up");
        pickupLines.add("Are you our robot, because I’d commit my life to you");
        pickupLines.add("Are you a depot, because I want to claim you");
        pickupLines.add("Are you our teams robot, because you look like you're good at sucking balls");
        pickupLines.add("I hope your my robot because size matters");
        pickupLines.add("Are you a mineral because I'd like to sample you");
        pickupLines.add("Are you Parker because you mount me sideways");
        pickupLines.add("Does your robot have lights because I hope you flash me");
        pickupLines.add("Are you a crater because I want to hop on in you");
        pickupLines.add("Are you an alliance captain because you're on top");
        pickupLines.add("Are you a beacon because I want to tap that");
        pickupLines.add("Are you the lander because I’m looking for a hookup");
        pickupLines.add("Are you the intake because you’d be really good at sucking my balls");
        pickupLines.add("Are we the phones because I’m feeling a connection");
        pickupLines.add("Are you the queuing volunteers because I hope you’ll be coming for me soon");


        AndroidTextToSpeech textToSpeech = new AndroidTextToSpeech();

        textToSpeech.initialize();

        init(hardwareMap);
        telemetry.addData("Robot is initialized", "");
        telemetry.update();

        //LED init stuff
        blueLEDPattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        redLEDPattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        endgameLEDPattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER;


        waitForStart();
        telemetry.addData("Robot is started", "" );
        telemetry.update();

        markerDropperInner.setPosition(0.8);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        landerMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        landerMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        timer.reset();

        while(opModeIsActive()){

            //TANK DRIVE

            leftPower = Range.clip(Math.pow(gamepad1.right_stick_y, 3), -1, 1);
            rightPower = Range.clip(Math.pow(gamepad1.left_stick_y, 3), -1, 1);

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
            if(gamepad1.dpad_right){
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

            //GAMEPAD 2
            if(timer.seconds() > 90 && !switchedToEndgame){
                LEDController.setPattern(endgameLEDPattern);
                switchedToEndgame = true;
                telemetry.addData("Now in Endgame", "");


            } else if(timer.seconds() <= 90){

                if(gamepad2.b) {
                    LEDController.setPattern(redLEDPattern);
                    telemetry.addData("LED Color:", "Red");


                }
                if(gamepad2.x) {
                    LEDController.setPattern(blueLEDPattern);
                    telemetry.addData("LED Color:", "Blue");
                }
            }
            if (gamepad2.a && !textToSpeech.isSpeaking()) {

                randomLine = randomWihRange(0, (pickupLines.size() - 1));
                textToSpeech.speak(pickupLines.get(randomLine));

            }


            telemetry.update();

        }
        stopRobot();
        telemetry.addData("Robot is stopped", "" );
        telemetry.update();
    }

    public int randomWihRange(int min, int max){

        Random random = new Random();

        return random.nextInt(max - min + 1) + min;

    }


}

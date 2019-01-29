package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.teamcode.HardwareDefinitions;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;


@TeleOp(name="TextToSpeechTest", group="Test")
public class TextToSpeechTest extends HardwareDefinitions {


    float pitch = 1;
    float pitchIncrement = (float) 0.005;
    float speechRate = 1;
    float speechRateIncrement = (float) 0.005;

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



        init(hardwareMap);

        AndroidTextToSpeech textToSpeech = new AndroidTextToSpeech();

        textToSpeech.initialize();


        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.a && !textToSpeech.isSpeaking()) {

                randomLine = randomWihRange(0, (pickupLines.size() - 1));
                textToSpeech.speak(pickupLines.get(randomLine));

            }
            if (gamepad1.left_stick_y > 0.1) {
                //down on the gamepad is positive so this has to bring the pitch value downwards
                pitch -= pitchIncrement;
                textToSpeech.setPitch(pitch);
            } else if (gamepad1.left_stick_y < -0.1) {
                //up on the gamepad is negative so this has to bring th pitch up
                pitch += pitchIncrement;
                textToSpeech.setPitch(pitch);
            }
            if (gamepad1.right_stick_y > 0.1) {
                speechRate -= speechRateIncrement;
                textToSpeech.setSpeechRate(speechRate);
            } else if (gamepad1.right_stick_y < -0.1) {
                speechRate += speechRateIncrement;
                textToSpeech.setSpeechRate(speechRate);
            }

            telemetry.addData("Current Pitch:", pitch);
            telemetry.addData("Current Speech Rate:", speechRate);
            telemetry.addData("Current Line:", pickupLines.get(randomLine));
            telemetry.update();


        }

    }

    /**
     *
     * @param min = the minimum value to be able to be returned
     * @param max = the maximum value to be able to be returned
     * @return a random integer in a range from the inputted minimum to the inputted maximum
     */
    public int randomWihRange(int min, int max){

        Random random = new Random();

        return random.nextInt(max - min + 1) + min;

    }


}

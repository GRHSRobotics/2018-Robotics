package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class TextToSpeechHandler {

    List<String> pickupLines = new ArrayList<>();

    AndroidTextToSpeech textToSpeech = new AndroidTextToSpeech();

    public TextToSpeechHandler(){

        pickupLines.add("Are you a judge, because I really hope I am good enough for you");
        pickupLines.add("Are you our alliance partner, because I really hope you do not suck");
        pickupLines.add("Are you the Engineering Notebook, because I would stay up until 2am doing you");
        pickupLines.add("Are you this years robot, because you make me have a mental breakdown");
        pickupLines.add("I'm like our robot, because my nuts drop whenever I’m around you");
        pickupLines.add("Are you FRC, because I can not afford you");
        pickupLines.add("Are you a mecanum wheel, because I would stick my shaft right in you");
        pickupLines.add("Hey girl are you our robot because you keep falling apart but I love you anyways");
        pickupLines.add("Hey girl are you the permission slip because nobody is going to do you");
        pickupLines.add("Are you our lift system because I can not get it up");
        pickupLines.add("Are you our robot, because I would commit my life to you");
        pickupLines.add("Are you a depot, because I want to claim you");
        pickupLines.add("Are you our teams robot, because you look like you are good at sucking balls");
        pickupLines.add("I hope your my robot because size matters");
        pickupLines.add("Are you a mineral because I would like to sample you");
        pickupLines.add("Are you Parker because you mount me sideways");
        pickupLines.add("Does your robot have lights because I hope you flash me");
        pickupLines.add("Are you a crater because I want to hop on in you");
        pickupLines.add("Are you an alliance captain because you are on top");
        pickupLines.add("Are you a beacon because I want to tap that");
        pickupLines.add("Are you the lander because I’m looking for a hookup");
        pickupLines.add("Are you the intake because you’d be really good at sucking my balls");
        pickupLines.add("Are we the phones because I’m feeling a connection");
        pickupLines.add("Are you the queuing volunteers because I hope you will be coming for me soon");

        textToSpeech.initialize();

    }

    public void speak(String line){

            textToSpeech.speak(line);

    }

    public void speakRandomLine(){

            int randomLine = randomWihRange(0, (pickupLines.size() - 1));
            textToSpeech.speak(pickupLines.get(randomLine));

    }

    private int randomWihRange(int min, int max){

        Random random = new Random();

        return random.nextInt(max - min + 1) + min;

    }

    public boolean isSpeaking(){

        return textToSpeech.isSpeaking();
    }

}

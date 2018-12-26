package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.TFLiteHandler;

@Autonomous(name = "VisionDepotSide", group = "Vision")



//define
public class VisionDepotSide extends AutonomousDefinitions {


    @Override
    public void runOpMode() {

        init(hardwareMap);
        initIMU(hardwareMap);

        TFLiteHandler TF = new TFLiteHandler();

        TF.initTFodAndVuforia();

        telemetry.addData("Robot is initialized", "");
        telemetry.update();
        waitForStart();



        //add movement to

        markerDropperOuter.setPosition(markerDropperOuterHold);

        //dropFromLander();
        encoderDrive(0.4 ,14, 14, 5);
        //moveLanderWithEncoder((38*4), 8);
        encoderTurn(0.25, 105, false, 5);
        encoderDrive(0.4, 7, 7, 5);

        markerDropperOuter.setPosition(markerDropperOuterRelease);

        TF.detectGold_inferRight(5);

        //movement stuff

        markerDropperOuter.setPosition(markerDropperOuterHold);


        switch(TF.getMineralPosition(true)){
            case 1:
                //encoderDrive(0.4, 7, 7, 5);
                encoderTurn(0.25, 70, true, 5); //turn left and drive towards the gold
                encoderDrive(0.4, 27, 27, 10);
                encoderTurn(0.25, 95, true, 5);
                encoderDrive(0.4, 21, 21, 5);
                encoderTurn(0.25, 155, false, 5);

                //drop the marker
                moveBoxMechanism(3, 3);
                dropMarker();


                encoderTurn(0.25, 100, false, 5);

                break;

            case 2:

                encoderDrive(0.35, -5.5, -5.5, 5); //drive straight towards the gold
                encoderTurn(0.25, 105, true, 5);
                encoderDrive(0.35, 42, 42, 10);
                encoderTurn(0.25, 105, false, 5);

                //drop the marker
                moveBoxMechanism(3, 3);
                dropMarker();

                break;


            case 3:

                encoderDrive(0.4, -17, -17, 5);
                encoderTurn(0.25, 130, true, 5); //turn right and drive towards the gold
                encoderDrive(0.35, 25, 25, 10);
                encoderTurn(0.25, 75, false, 5);

                encoderDrive(0.4, 25, 25, 5);
                encoderTurn(0.25, 105, false, 5);

                //drop the marker
                moveBoxMechanism(3, 3);
                dropMarker();

                break;

            default:

                encoderDrive(0.4, 7, 7, 5);
                encoderTurn(0.25, 90, true, 5); //turn left and drive towards the gold
                encoderDrive(0.35, 23, 23, 10);
                encoderTurn(0.25, 75, true, 5);
                encoderDrive(0.4, 21, 21, 5);
                encoderTurn(0.25, 105, false, 5);

                //drop the marker
                moveBoxMechanism(3, 3);
                dropMarker();

                break;
        }



    }


}


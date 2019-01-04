package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;
import org.firstinspires.ftc.teamcode.TFLiteHandler;
@Disabled
@Autonomous(name = "VisionDepotSide", group = "Vision")
public class GyroVisionDepotSide extends HardwareDefinitions {


    @Override
    public void runOpMode() {

        init(hardwareMap);
        //initIMU(hardwareMap);
/*
        TFLiteHandler TF = new TFLiteHandler();

        TF.initTFodAndVuforia();

        telemetry.addData("Robot is initialized", "");
        telemetry.update();
        */
        waitForStart();



        //add movement to
/*
        markerDropperOuter.setPosition(markerDropperOuterHold);

        //dropFromLander();
        gyroDrive(0.4 ,14, 0, 5);
        //moveLanderWithEncoder((38*4), 8);
        gyroTurn(0.25, 90,5);
        gyroDrive(0.4, 7, 0, 5);

        markerDropperOuter.setPosition(markerDropperOuterRelease);

        TF.detectGold_inferRight(5);

        //movement stuff

        markerDropperOuter.setPosition(markerDropperOuterHold);


        switch(TF.getMineralPosition(true)){
            case 1:
                //encoderDrive(0.4, 7, 7, 5);
                gyroTurn(0.25, -60, 5); //turn left and drive towards the gold
                gyroDrive(0.4, 27, 0, 10);
                gyroTurn(0.25, -85,  5);
                gyroDrive(0.4, 21, 0, 5);
                gyroTurn(0.25, 140, 5);

                //drop the marker
                moveBoxMechanism(3, 3);
                dropMarker();


                gyroTurn(0.25, 90, 5);

                break;

            case 2:

                gyroDrive(0.35, -5.5, 0, 5); //drive straight towards the gold
                gyroTurn(0.25, -90, 5);
                gyroDrive(0.35, 42, 0, 10);
                gyroTurn(0.25, 95, 5);

                //drop the marker
                moveBoxMechanism(3, 3);
                dropMarker();

                break;


            case 3:

                gyroDrive(0.4, -17, 0, 5);
                gyroTurn(0.25, -110, 5); //turn right and drive towards the gold
                gyroDrive(0.35, 25, 0, 10);
                gyroTurn(0.25, 65, 5);

                gyroDrive(0.4, 25, 0, 5);
                gyroTurn(0.25, 90, 5);

                //drop the marker
                moveBoxMechanism(3, 3);
                dropMarker();

                break;

            default:

                //encoderDrive(0.4, 7, 7, 5);
                gyroTurn(0.25, -60, 5); //turn left and drive towards the gold
                gyroDrive(0.4, 27, 0, 10);
                gyroTurn(0.25, -85,  5);
                gyroDrive(0.4, 21, 0, 5);
                gyroTurn(0.25, 140, 5);

                //drop the marker
                moveBoxMechanism(3, 3);
                dropMarker();


                gyroTurn(0.25, 90, 5);

                break;
        }

*/

    }


}


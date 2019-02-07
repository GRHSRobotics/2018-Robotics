package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.TFLiteHandler;


@Autonomous(name = "GyroVisionDepotSide - With Landing", group = "Gyro Vision")
public class GyroVisionDepotSide extends AutonomousDefinitions {


    @Override
    public void runOpMode() {

        init(hardwareMap);
        telemetry.update();

        initIMU(hardwareMap);
        telemetry.update();

        initTFodAndVuforia();

        telemetry.addData("Robot is initialized", "");
        telemetry.update();
        waitForStart();


        //add movement to

        markerDropperOuter.setPosition(markerDropperOuterHold);

        dropFromLander(true);
        encoderDrive(0.4, 14, 14, 5);
        //moveLanderWithEncoder((38*4), 8);
        gyroTurn(0.3, 90, 5);
        driveToMineral(0.25,  5);


        markerDropperOuter.setPosition(markerDropperOuterRelease);


        detectGold(TFLiteHandler.inferMineral.RIGHT, 2);

        //movement stuff

        markerDropperOuter.setPosition(markerDropperOuterHold);


        switch (getMineralPosition(false)) {
            case 1:
                //encoderDrive(0.4, 7, 7, 5);
                gyroTurn(0.25, 60, 5); //turn left and drive towards the gold
                encoderDrive(0.4, 29, 29, 10);
                gyroTurn(0.25, -45, 5);
                encoderDrive(0.4, 25, 25, 5);
                gyroTurn(0.25, 110, 5);


                //drop the marker
                moveBoxMechanism(2, 2);
                dropMarker();
                //moveBoxMechanism(-2, 3);


                //encoderTurn(0.25, 100, false, 5);

                break;

            case 2:

                encoderDrive(0.35, -3, -3, 5); //drive straight towards the gold
                encoderTurn(0.25, 105, true, 5);
                encoderDrive(0.35, 46, 46, 10);
                encoderTurn(0.25, 105, false, 5);

                //drop the marker
                moveBoxMechanism(2, 2);
                dropMarker();
                //moveBoxMechanism(-2, 3);

                encoderDrive(0.4, 10, 10, 5);

                break;


            case 3:

                encoderDrive(0.4, -17, -17, 5);
                encoderTurn(0.25, 130, true, 5); //turn right and drive towards the gold
                encoderDrive(0.35, 28, 28, 10);
                encoderTurn(0.25, 75, false, 5);

                encoderDrive(0.4, 29, 29, 5);
                encoderTurn(0.25, 80, false, 5);

                //drop the marker
                moveBoxMechanism(2, 2);
                dropMarker();
                //moveBoxMechanism(-2, 3);

                encoderDrive(0.4, 10, 10, 5);

                break;

            default:

                encoderDrive(0.4, 7, 7, 5);
                encoderTurn(0.25, 90, true, 5); //turn left and drive towards the gold
                encoderDrive(0.35, 23, 23, 10);
                encoderTurn(0.25, 75, true, 5);
                encoderDrive(0.4, 21, 21, 5);
                encoderTurn(0.25, 105, false, 5);

                //drop the marker
                moveBoxMechanism(2, 2);
                dropMarker();
                //moveBoxMechanism(-2, 3);

                break;
        }


    }







}




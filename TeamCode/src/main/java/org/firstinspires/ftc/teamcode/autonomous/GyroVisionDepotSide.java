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
        encoderDrive(0.4, 6, 6, 5);
        //driveToMineral(0.3, 5, true, false);

        markerDropperOuter.setPosition(markerDropperOuterRelease);

        detectGold(TFLiteHandler.inferMineral.RIGHT, 4);

        //movement stuff

        markerDropperOuter.setPosition(markerDropperOuterHold);


        switch (getMineralPosition(false)) {
            case 1:
                //encoderDrive(0.4, 7, 7, 5);
                gyroTurn(0.25, 50, 5); //turn left and drive towards the gold
                encoderDrive(0.4, 29, 29, 10);
                gyroTurn(0.25, -35, 5);
                encoderDrive(0.4, 28, 28, 5);
                gyroTurn(0.25, 90, 5);


                //drop the marker
                dropMarker();
                //moveBoxMechanism(-2, 3);


                //encoderTurn(0.25, 100, false, 5);

                break;

            case 2:

                encoderDrive(0.35, -2, -2, 5); //drive straight towards the gold
                gyroTurn(0.25, 0, 5);
                encoderDrive(0.35, 46, 46, 10);
                gyroTurn(0.25, 90, 5);

                //drop the marker
                dropMarker();
                //moveBoxMechanism(-2, 3);

                encoderDrive(0.4, 10, 10, 5);

                break;


            case 3:

                encoderDrive(0.4, -17, -17, 5);
                gyroTurn(0.25, -15, 5); //turn right and drive towards the gold
                encoderDrive(0.35, 28, 28, 10);
                gyroTurn(0.25, 45, 5);

                encoderDrive(0.4, 32, 32, 5);
                gyroTurn(0.25, 90, 5);

                //drop the marker
                dropMarker();
                //moveBoxMechanism(-2, 3);

                encoderDrive(0.4, 10, 10, 5);

                break;

            default:

                //encoderDrive(0.4, 7, 7, 5);
                gyroTurn(0.25, 50, 5); //turn left and drive towards the gold
                encoderDrive(0.4, 29, 29, 10);
                gyroTurn(0.25, -35, 5);
                encoderDrive(0.4, 28, 28, 5);
                gyroTurn(0.25, 90, 5);


                //drop the marker
                dropMarker();
                //moveBoxMechanism(-2, 3);


                //encoderTurn(0.25, 100, false, 5);

                break;
        }


    }







}




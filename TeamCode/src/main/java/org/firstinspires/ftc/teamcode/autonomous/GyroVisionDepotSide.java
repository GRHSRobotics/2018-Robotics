package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "GyroVisionDepotSide", group = "Gyro Vision")
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
        encoderDrive(0.4, 11, 11, 5);
        gyroTurn(0.3, 85, 5);
        encoderDrive(0.4, 6.5, 6.5, 5);

        markerDropperOuter.setPosition(markerDropperOuterRelease); //get the marker holder out of the way for vision

        detectGold(inferMineral.RIGHT, 4);

        markerDropperOuter.setPosition(markerDropperOuterHold);


        switch (getMineralPosition(false)) {
            case LEFT:

                gyroTurn(0.25, 50, 5); //turn left and drive towards the gold
                encoderDrive(0.4, 28, 28, 10);
                gyroTurn(0.25, -35, 5);
                encoderDrive(0.4, 28, 28, 5);
                gyroTurn(0.25, 90, 5);

                dropMarker();

                break;

            case CENTER:

                encoderDrive(0.35, -4, -4, 5); //drive straight towards the gold
                gyroTurn(0.25, 5, 5);
                encoderDrive(0.35, 46, 46, 10);
                gyroTurn(0.25, 90, 5);

                dropMarker();

                //encoderDrive(0.4, 10, 10, 5);

                break;


            case RIGHT:

                encoderDrive(0.4, -15, -15, 5);
                gyroTurn(0.25, -15, 5); //turn right and drive towards the gold
                encoderDrive(0.35, 30, 30, 10);
                gyroTurn(0.25, 45, 5);

                encoderDrive(0.4, 28, 28, 5);
                gyroTurn(0.25, 90, 5);

                dropMarker();

                //encoderDrive(0.4, 10, 10, 5);

                break;

            default:

                gyroTurn(0.25, 50, 5); //turn left and drive towards the gold
                encoderDrive(0.4, 29, 29, 10);
                gyroTurn(0.25, -35, 5);
                encoderDrive(0.4, 28, 28, 5);
                gyroTurn(0.25, 90, 5);

                dropMarker();

                break;
        }

        moveBoxMechanism(-0.3, 0.3);
        moveLanderWithEncoder(-82, 8);


    }







}




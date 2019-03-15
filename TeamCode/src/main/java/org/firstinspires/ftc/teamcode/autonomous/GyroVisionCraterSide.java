package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "GyroVisionCraterSide", group = "Gyro Vision")
public class GyroVisionCraterSide extends AutonomousDefinitions {


    @Override
    public void runOpMode() {

        init(hardwareMap);
        telemetry.update();
        initIMU(hardwareMap);
        telemetry.update();

        initTFodAndVuforia();
        telemetry.update();

        telemetry.addData("Robot is initialized", "");
        telemetry.update();
        waitForStart();



        //add movement to

        dropFromLander(true);
        encoderDrive(0.4 ,11, 11, 5);
        gyroTurn(0.35, 85, 5);
        encoderDrive(0.4, 6.5, 6.5, 5);

        markerDropperOuter.setPosition(markerDropperOuterRelease); //get the marker holder out of the way for vision

        detectGold(inferMineral.RIGHT, 2);

        markerDropperOuter.setPosition(markerDropperOuterHold);


        switch(getMineralPosition(false)){

            case LEFT:

                gyroTurn(0.25, 40, 5); //turn left and drive towards the gold
                encoderDrive(0.7, 22, 22, 10);
                moveLeftTread(0.4, 15, 8);

                break;

            case CENTER:

                encoderDrive(0.35, -3, -3, 5); //drive straight towards the gold
                gyroTurn(0.25, 0, 5);
                encoderDrive(0.7, 26, 26, 10);

                break;

            case RIGHT:

                encoderDrive(0.4, -13, -13, 5);
                gyroTurn(0.25, -20, 5); //turn right and drive towards the gold
                encoderDrive(0.7, 20, 20, 10);
                moveRightTread(0.4, 10, 5);

                break;

            default:

                gyroTurn(0.25, 40, 5); //turn left and drive towards the gold
                encoderDrive(0.7, 20, 20, 10);
                moveLeftTread(0.4, 15, 8);

                break;
        }

        //finesse the robot over the crater boundary
        intakeMotor.setPower(-1);
        sleep(1000);
        intakeMotor.setPower(0);

        moveLanderWithEncoder(-82, 8);


    }




}


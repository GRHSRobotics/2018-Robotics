package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;


@Autonomous(name="LinearDepotSide", group="Autonomous")
public class LinearDepotSide extends AutonomousDefinitions {


    @Override
    public void runOpMode() {

        //INITIALIZATION PERIOD
        init(hardwareMap);

        setDriveEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Robot is initialized", "");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        markerDropperOuter.setPosition(markerDropperOuterHold);
        markerDropperInner.setPosition(markerDropperInnerHold);

        dropFromLander(false);

        //START PERIOD

        //move to depot and push mineral in
        encoderDrive(0.5, -52, -52, 15);

        encoderTurn(0.25, 100, false, 5);

        markerDropperOuter.setPosition(markerDropperOuterRelease);
        sleep(1000);
        markerDropperInner.setPosition(markerDropperInnerRelease);
        sleep(1500);
        markerDropperInner.setPosition(markerDropperInnerHold);
        sleep(1000);
        markerDropperOuter.setPosition(markerDropperOuterHold);

/*
        //move back from depot to clear marker
        encoderDrive(0.5, -36, -36, 10);

        //turn towards crater
        encoderTurn(0.25, 90, false, 10);

        //drive towards wall
        encoderDrive(0.5, 35, 35, 10);

        //turn so that the robot is parallel to the wall
        encoderTurn(0.25, 120, false, 5);

        //move forward towards the crater
        encoderDrive(0.5, 36, 36, 10);
*/


        telemetry.addData("Completed Autonomous","");

    }
}

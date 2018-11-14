package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;


@Autonomous(name="LinearCraterSide", group="Autonomous")
public class LinearCraterSide extends HardwareDefinitions {

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


        //START PERIOD

        //drive to crater, push center mineral
        encoderDrive(0.5, 35, 35, 15);

        //back to in front of minerals
        encoderDrive(0.5 ,-23, -23, 10);

        //turn 90 degrees
        encoderTurn(0.25, 110, false, 10);

        //move towards the game wall
        encoderDrive(0.5, 45, 45, 10);

        //turn so that the robot is parallel to the wall
        encoderTurn(0.25, 73, false, 5);

        //move to the depot
        encoderDrive(0.5, 50, 50, 10);

        //release team marker
        markerDropper.setPosition(markerDropperForward);
        sleep(1500);
        markerDropper.setPosition(markerDropperBack);

        //make sure that we don't hit crater side ball/mineral
        encoderTurn(0.25, 8, false, 5);

        encoderDrive(0.5, -78, -78, 10);


        telemetry.addData("Completed Autonomous","");

    }
}


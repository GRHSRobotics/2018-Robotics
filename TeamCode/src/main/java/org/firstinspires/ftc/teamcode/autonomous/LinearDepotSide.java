package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;


@Autonomous(name="LinearDepotSide", group="Autonomous")
public class LinearDepotSide extends HardwareDefinitions {


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

        //move to depot and push mineral in
        encoderDrive(0.5, 50, 50, 15);

        //release team marker
        markerDropper.setPosition(markerDropperForward);
        sleep(1500);
        markerDropper.setPosition(markerDropperBack);

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



        telemetry.addData("Completed Autonomous","");

    }
}

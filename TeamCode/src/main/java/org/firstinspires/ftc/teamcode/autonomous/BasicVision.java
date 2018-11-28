package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;


@Autonomous(name="BasicVision", group="Autonomous")
public class BasicVision extends HardwareDefinitions {

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




    }
}
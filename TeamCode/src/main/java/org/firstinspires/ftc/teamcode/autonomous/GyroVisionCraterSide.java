package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TFLiteHandler;

@Autonomous(name = "GyroVisionCraterSide - With Landing", group = "Gyro Vision")
public class GyroVisionCraterSide extends AutonomousDefinitions {


    @Override
    public void runOpMode() {

        init(hardwareMap);
        initIMU(hardwareMap);

        ElapsedTime timer = new ElapsedTime();

        initTFodAndVuforia();


        telemetry.addData("Robot is initialized", "");
        telemetry.update();
        waitForStart();



        //add movement to

        markerDropperOuter.setPosition(markerDropperOuterHold);

        dropFromLander(true);
        encoderDrive(0.4 ,14, 14, 5);
        //moveLanderWithEncoder((38*4), 8);
        gyroTurn(0.35, 90, 5);
        encoderDrive(0.4, 3.5, 3.5, 5);

        markerDropperOuter.setPosition(markerDropperOuterRelease);

        detectGold(inferMineral.RIGHT, 2);

        //movement stuff

        markerDropperOuter.setPosition(markerDropperOuterHold);


        switch(getMineralPosition(false)){

            case LEFT:

                //encoderDrive(0.4, 3, 3, 5);
                gyroTurn(0.25, 40, 5); //turn left and drive towards the gold
                encoderDrive(0.7, 26, 26, 10);
                moveLeftTread(0.4, 15, 8);
                //encoderDrive(0.35, -12, -12, 10);
                //encoderTurn(0.25, 105, false, 5);

                break;

            case CENTER:

                encoderDrive(0.35, -9, -9, 5); //drive straight towards the gold
                gyroTurn(0.25, 0, 5);
                encoderDrive(0.7, 26, 26, 10);
                //encoderDrive(0.35, -15, -15, 10);
                //encoderTurn(0.25, 105, false, 5);
                //encoderDrive(0.4, 14, 14, 10);

                break;

            case RIGHT:

                encoderDrive(0.4, -20, -20, 5);
                gyroTurn(0.25, -40, 5); //turn right and drive towards the gold
                encoderDrive(0.7, 26, 26, 10);
                moveRightTread(0.4, 10, 5);
                //encoderDrive(0.35, -15, -15, 10);
                //encoderTurn(0.25, 105, false, 5);
                //encoderDrive(0.4, 29, 29, 10);

                break;

            default:

                //encoderDrive(0.4, 3, 3, 5);
                gyroTurn(0.25, 40, 5); //turn left and drive towards the gold
                encoderDrive(0.7, 26, 26, 10);
                moveLeftTread(0.4, 15, 8);
                //encoderDrive(0.35, -12, -12, 10);
                //encoderTurn(0.25, 105, false, 5);

                break;
        }

        //finesse the robot over the crater boundary
        intakeMotor.setPower(-1);
        sleep(3000);
        intakeMotor.setPower(0);


    }




}


package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;
import org.firstinspires.ftc.teamcode.TFLiteHandler;
@Disabled
@Autonomous(name = "VisionCraterSide", group = "Vision")
public class GyroVisionCraterSide extends HardwareDefinitions {


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


/*
        //add movement to

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

                //encoderDrive(0.4, 3, 3, 5);
                gyroTurn(0.25, -60, 5); //turn left and drive towards the gold
                gyroDrive(0.7, 26, 0, 10);
                encoderDrive(0.4, 15, 0, 8);
                //encoderDrive(0.35, -12, -12, 10);
                //encoderTurn(0.25, 105, false, 5);

                break;

            case 2:

                gyroDrive(0.35, -5, 0, 5); //drive straight towards the gold
                gyroTurn(0.25, -90, 5);
                gyroDrive(0.7, 26, 0, 10);
                //encoderDrive(0.35, -15, -15, 10);
                //encoderTurn(0.25, 105, false, 5);
                //encoderDrive(0.4, 14, 14, 10);

                break;

            case 3:

                gyroDrive(0.4, -20, 0, 5);
                gyroTurn(0.25, -110, 5); //turn right and drive towards the gold
                encoderDrive(0.7, 26, 26, 10);
                encoderDrive(0.4, 0, 10, 5);
                //encoderDrive(0.35, -15, -15, 10);
                //encoderTurn(0.25, 105, false, 5);
                //encoderDrive(0.4, 29, 29, 10);

                break;

            default:

                //encoderDrive(0.4, 3, 3, 5);
                gyroTurn(0.25, -60, 5); //turn left and drive towards the gold
                gyroDrive(0.7, 26, 0, 10);
                encoderDrive(0.4, 15, 0, 8);
                //encoderDrive(0.35, -12, -12, 10);
                //encoderTurn(0.25, 105, false, 5);

                break;
        }

        //finesse the robot over the crater boundary
        intakeMotor.setPower(1);
        sleep(3000);
        intakeMotor.setPower(0);
*/

    }


}


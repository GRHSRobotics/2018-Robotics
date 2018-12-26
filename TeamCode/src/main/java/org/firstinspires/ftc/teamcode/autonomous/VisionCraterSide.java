package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.TFLiteHandler;

@Autonomous(name = "VisionCraterSide", group = "Vision")



//define
public class VisionCraterSide extends AutonomousDefinitions {


    @Override
    public void runOpMode() {

        init(hardwareMap);
        initIMU(hardwareMap);

        TFLiteHandler TF = new TFLiteHandler();

        TF.initTFodAndVuforia();


        telemetry.addData("Robot is initialized", "");
        telemetry.update();
        waitForStart();



        //add movement to

        markerDropperOuter.setPosition(markerDropperOuterHold);

        //dropFromLander();
        encoderDrive(0.4 ,14, 14, 5);
        //moveLanderWithEncoder((38*4), 8);
        encoderTurn(0.25, 105, false, 5);
        encoderDrive(0.4, 7, 7, 5);

        markerDropperOuter.setPosition(markerDropperOuterRelease);

        TF.detectGold_inferRight(5);

        //movement stuff

        markerDropperOuter.setPosition(markerDropperOuterHold);


        switch(TF.getMineralPosition()){

            case 1:

                //encoderDrive(0.4, 3, 3, 5);
                encoderTurn(0.25, 70, true, 5); //turn left and drive towards the gold
                encoderDrive(0.7, 26, 26, 10);
                encoderDrive(0.4, 15, 0, 8);
                //encoderDrive(0.35, -12, -12, 10);
                //encoderTurn(0.25, 105, false, 5);

                break;

            case 2:

                encoderDrive(0.35, -5, -5, 5); //drive straight towards the gold
                encoderTurn(0.25, 105, true, 5);
                encoderDrive(0.7, 26, 26, 10);
                //encoderDrive(0.35, -15, -15, 10);
                //encoderTurn(0.25, 105, false, 5);
                //encoderDrive(0.4, 14, 14, 10);

                break;

            case 3:

                encoderDrive(0.4, -20, -20, 5);
                encoderTurn(0.25, 115, true, 5); //turn right and drive towards the gold
                encoderDrive(0.7, 26, 26, 10);
                encoderDrive(0.4, 0, 10, 5);
                //encoderDrive(0.35, -15, -15, 10);
                //encoderTurn(0.25, 105, false, 5);
                //encoderDrive(0.4, 29, 29, 10);

                break;

            default:

                //encoderDrive(0.4, 3, 3, 5);
                encoderTurn(0.25, 70, true, 5); //turn left and drive towards the gold
                encoderDrive(0.7, 26, 26, 10);
                encoderDrive(0.4, 15, 0, 8);
                //encoderDrive(0.35, -12, -12, 10);
                //encoderTurn(0.25, 105, false, 5);

                break;
        }

        //finesse the robot over the crater boundary
        intakeMotor.setPower(1);
        sleep(3000);
        intakeMotor.setPower(0);


    }


}


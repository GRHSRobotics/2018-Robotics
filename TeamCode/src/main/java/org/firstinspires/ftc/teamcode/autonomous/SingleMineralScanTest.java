package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="SingleMineralScanTest", group="Test")
public class SingleMineralScanTest extends AutonomousDefinitions {

    @Override
    public void runOpMode(){

        init(hardwareMap);
        telemetry.update();
        initIMU(hardwareMap);
        telemetry.update();
        initTFodAndVuforia();
        telemetry.update();

        waitForStart();



        checkGoldInFrame(3);

        if(isGoldInFrame()){
            encoderDrive(0.3, 3, 3, 3);
            encoderTurn(0.3, 100, true, 5);
            encoderDrive(0.3, 8, 8, 3);
        } else{
            encoderDrive(0.4, 12, 12, 5);

            checkGoldInFrame(3);

            if(isGoldInFrame()){
                encoderDrive(0.3, 3, 3, 3);
                encoderTurn(0.3, 100, true, 5);
                encoderDrive(0.3, 8, 8, 3);
            } else{
                encoderDrive(0.4, 12, 12, 5);

                encoderDrive(0.3, 3, 3, 3);
                encoderTurn(0.3, 100, true, 5);
                encoderDrive(0.3, 8, 8, 3);


            }
        }






    }


}

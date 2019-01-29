package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HardwareDefinitions;
import org.firstinspires.ftc.teamcode.TFLiteHandler;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "GyroVisionCraterSide - With Landing", group = "Gyro Vision")
public class GyroVisionCraterSide extends HardwareDefinitions {


    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AR1NGjD/////AAABmRNHhw4urkcYu6OsCz4GxO9HaxexcrZrSNGBfCYsc8miWAyyHlu53AsvQ0AMdhXKpFuLLm0Dej3xk4agW4J4tOXGu+hPnigkbDyr5HhVrGXPGxFyNCpJUHx+Sr6UMygVYr5b+z78sdhUeN2o4KBHClV+VzRnAuG0h4GiWh+58fPYhqIIRboPe41XAbmNWwCIqAG+1y5XXaENN0jq99vO4e4GgzYzQdAQtK4Jrq4pkIZev+fI5K2B500kIkiVv3YrnC1JkQNIfibntc+98DKcN7hbJ3TWJmHndB9vesnlzPnDEJ/q9j+V+w82/icXhZ58Jcu+QMu/iuo7eEZeCLQ8S5BqotKIbxP3mCW31jh93Btc"; //put vuforia key in quotes

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    ElapsedTime timer = new ElapsedTime();

    int currentDetectionValue = 0; //1 is left, 2 is center, 3 is right. Holds the current guess on the position of the mineral

    List<Integer> detectionValues = new ArrayList<>();

    @Override
    public void runOpMode() {

        init(hardwareMap);
        initIMU(hardwareMap);

        TFLiteHandler TF = new TFLiteHandler(hardwareMap, telemetry);
        TF.initTFodAndVuforia();


        telemetry.addData("Robot is initialized", "");
        telemetry.update();
        waitForStart();



        //add movement to

        markerDropperOuter.setPosition(markerDropperOuterHold);

        dropFromLander(true);
        encoderDrive(0.4 ,14, 14, 5);
        //moveLanderWithEncoder((38*4), 8);
        gyroTurn(0.35, -90, 5);
        encoderDrive(0.4, 3.5, 3.5, 5);

        markerDropperOuter.setPosition(markerDropperOuterRelease);

        TF.detectGold(TFLiteHandler.inferMineral.RIGHT, 2);

        //movement stuff

        markerDropperOuter.setPosition(markerDropperOuterHold);


        switch(TF.getMineralPosition(false)){

            case 1:

                //encoderDrive(0.4, 3, 3, 5);
                encoderTurn(0.25, 70, true, 5); //turn left and drive towards the gold
                encoderDrive(0.7, 26, 26, 10);
                moveLeftTread(0.4, 15, 8);
                //encoderDrive(0.35, -12, -12, 10);
                //encoderTurn(0.25, 105, false, 5);

                break;

            case 2:

                encoderDrive(0.35, -9, -9, 5); //drive straight towards the gold
                encoderTurn(0.25, 105, true, 5);
                encoderDrive(0.7, 26, 26, 10);
                //encoderDrive(0.35, -15, -15, 10);
                //encoderTurn(0.25, 105, false, 5);
                //encoderDrive(0.4, 14, 14, 10);

                break;

            case 3:

                encoderDrive(0.4, -20, -20, 5);
                encoderTurn(0.25, 128, true, 5); //turn right and drive towards the gold
                encoderDrive(0.7, 26, 26, 10);
                moveRightTread(0.4, 10, 5);
                //encoderDrive(0.35, -15, -15, 10);
                //encoderTurn(0.25, 105, false, 5);
                //encoderDrive(0.4, 29, 29, 10);

                break;

            default:

                //encoderDrive(0.4, 3, 3, 5);
                encoderTurn(0.25, 70, true, 5); //turn left and drive towards the gold
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


package org.firstinspires.ftc.teamcode.autonomous.old;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousDefinitions;

import java.util.ArrayList;
import java.util.List;
@Disabled
@Autonomous(name = "VisionCraterSide - With Landing", group = "Vision")
public class VisionCraterSide extends AutonomousDefinitions {


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

        //initIMU(hardwareMap);

        initTFodAndVuforia();


        telemetry.addData("Robot is initialized", "");
        telemetry.update();
        waitForStart();

        autonLEDPattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
        LEDController.setPattern(autonLEDPattern);

        //add movement to

        markerDropperOuter.setPosition(markerDropperOuterHold);

        dropFromLander(false);
        encoderTurn(0.4, 10, true, 3);
        encoderDrive(0.4 ,14, 14, 5);
        //moveLanderWithEncoder((38*4), 8);
        encoderTurn(0.25, 105, false, 5);
        encoderDrive(0.4, 5, 5, 5);

        markerDropperOuter.setPosition(markerDropperOuterRelease);

        detectGold_inferRight(2);

        //movement stuff

        markerDropperOuter.setPosition(markerDropperOuterHold);


        switch(getMineralPosition(false)){

            case LEFT:

                //encoderDrive(0.4, 3, 3, 5);
                encoderTurn(0.25, 70, true, 5); //turn left and drive towards the gold
                encoderDrive(0.7, 26, 26, 10);
                moveLeftTread(0.4, 15, 8);
                //encoderDrive(0.35, -12, -12, 10);
                //encoderTurn(0.25, 105, false, 5);

                break;

            case CENTER:

                encoderDrive(0.35, -5, -5, 5); //drive straight towards the gold
                encoderTurn(0.25, 105, true, 5);
                encoderDrive(0.7, 26, 26, 10);
                //encoderDrive(0.35, -15, -15, 10);
                //encoderTurn(0.25, 105, false, 5);
                //encoderDrive(0.4, 14, 14, 10);

                break;

            case RIGHT:

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

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void initTFodAndVuforia() {
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
            telemetry.addData("Initialized TensorFlow and Vuforia", "");
        } else {
            telemetry.addData("OOPS!!", "Your device can't work with TFOD");
        }
    }

    public void detectGold_inferRight(double maxTimeS){ //looks at left and center mineral

        timer.reset();

        if (opModeIsActive()) {
            /** Start Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive() && timer.seconds() < maxTimeS) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineralX = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineralX == -1) {
                                    silverMineralX = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX == -1) {
                                //telemetry.addData("Mineral Position:", "Left");
                                telemetry.addData("Mineral Position:", "Right");
                                //currentDetectionValue = 1;
                                currentDetectionValue = 3;
                            } else if (goldMineralX != -1){
                                if(goldMineralX < silverMineralX){
                                    //telemetry.addData("Mineral Position:", "Center");
                                    telemetry.addData("Mineral Position:", "Left");
                                    //currentDetectionValue = 2;
                                    currentDetectionValue = 1;
                                } else if(goldMineralX > silverMineralX){
                                    //telemetry.addData("Mineral Position:", "Right");
                                    telemetry.addData("Mineral Position:", "Center");
                                    //currentDetectionValue = 3;
                                    currentDetectionValue = 2;
                                } else {
                                    telemetry.addData("Mineral Position:", "Confused");
                                }
                            }
                        }
                        telemetry.update();
                    }
                }

                if(timer.seconds() >= 0.5){ //give the robot a little bit of time to come to a stop before recording values
                    detectionValues.add(currentDetectionValue);
                }
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }



}


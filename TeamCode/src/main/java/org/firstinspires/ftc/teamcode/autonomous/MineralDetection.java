package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HardwareDefinitions;

import java.util.List;

@Autonomous(name = "Mineral Detection", group = "Vision")



//define
public class MineralDetection extends HardwareDefinitions {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AR1NGjD/////AAABmRNHhw4urkcYu6OsCz4GxO9HaxexcrZrSNGBfCYsc8miWAyyHlu53AsvQ0AMdhXKpFuLLm0Dej3xk4agW4J4tOXGu+hPnigkbDyr5HhVrGXPGxFyNCpJUHx+Sr6UMygVYr5b+z78sdhUeN2o4KBHClV+VzRnAuG0h4GiWh+58fPYhqIIRboPe41XAbmNWwCIqAG+1y5XXaENN0jq99vO4e4GgzYzQdAQtK4Jrq4pkIZev+fI5K2B500kIkiVv3YrnC1JkQNIfibntc+98DKcN7hbJ3TWJmHndB9vesnlzPnDEJ/q9j+V+w82/icXhZ58Jcu+QMu/iuo7eEZeCLQ8S5BqotKIbxP3mCW31jh93Btc"; //put vuforia key in quotes

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    ElapsedTime timer = new ElapsedTime();

    int goldPosition = 0; //1 is left, 2 is center, 3 is right

    @Override
    public void runOpMode() {

        init(hardwareMap);

        //open camera
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("OOPS!!", "Your device can't work with TFOD");
        }


        telemetry.addData("Robot is initialized", "");
        telemetry.update();
        waitForStart();



        //add movement to

        //dropFromLander();
        encoderDrive(0.4 ,17, 17, 5);
        encoderTurn(0.25, 105, false, 5);
        encoderDrive(0.4, 7, 7, 5);

        timer.reset();

        if (opModeIsActive()) {
            /** Start Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive() && timer.seconds() < 5) {
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
                                //goldPosition = 1;
                                goldPosition = 3;
                            } else if (goldMineralX != -1){
                                if(goldMineralX < silverMineralX){
                                    //telemetry.addData("Mineral Position:", "Center");
                                    telemetry.addData("Mineral Position:", "Left");
                                    //goldPosition = 2;
                                    goldPosition = 1;
                                } else if(goldMineralX > silverMineralX){
                                    //telemetry.addData("Mineral Position:", "Right");
                                    telemetry.addData("Mineral Position:", "Center");
                                    //goldPosition = 3;
                                    goldPosition = 2;
                                } else {
                                    telemetry.addData("Mineral Position:", "Confused");
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        //movement stuff


        if(goldPosition == 1){ //gold is left
            encoderDrive(0.4, 4, 4, 5);
            encoderTurn(0.25, 105, true, 5); //turn left and drive towards the gold
            encoderDrive(0.35, 15, 15, 10);
            encoderDrive(0.35, -12, -12, 10);
            encoderTurn(0.25, 105, false, 5);

        } else if (goldPosition == 2){ //gold is center
            encoderDrive(0.35, -10, -10, 5); //drive straight towards the gold
            encoderTurn(0.25, 105, true, 5);
            encoderDrive(0.35, 15, 15, 10);
            encoderDrive(0.35, -15, -15, 10);
            encoderTurn(0.25, 105, false, 5);
            encoderDrive(0.4, 14, 14, 10);

        } else if(goldPosition == 3){ //gold is right
            encoderDrive(0.4, -25, -25, 5);
            encoderTurn(0.25, 105, true, 5); //turn right and drive towards the gold
            encoderDrive(0.35, 15, 15, 10);
            encoderDrive(0.35, -15, -15, 10);
            encoderTurn(0.25, 105, false, 5);
            encoderDrive(0.4, 29, 29, 10);


        } else { //Tensorflow doesn't know
            encoderDrive(0.4, 4, 4, 5);
            encoderTurn(0.25, 105, true, 5); //turn left and drive towards the gold
            encoderDrive(0.35, 15, 15, 10);
            encoderDrive(0.35, -15, -15, 10);
            encoderTurn(0.25, 105, false, 5);

        }

        //for crater side only TODO make depot side stuff
        //this may or may not be worth it, HIGH RISK OF DESCORING ALLIANCE MINERAL
        /*
        encoderDrive(0.4, 25, 25, 5);
        encoderTurn(0.35, 60, false, 5);
        encoderDrive(0.5, 50, 50, 10);



        encoderTurn(0.25, 100, false, 5);

        markerDropperOuter.setPosition(markerDropperOuterRelease);
        sleep(1000);
        markerDropperInner.setPosition(markerDropperInnerRelease);
        sleep(1500);
        markerDropperInner.setPosition(markerDropperInnerHold);
        sleep(1000);
        markerDropperOuter.setPosition(markerDropperOuterHold);

        encoderTurn(0.25, 100, true, 5);

        encoderDrive(0.4, -60, -60, 10);
        */

    }

    private void initVuforia() {
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
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }


}


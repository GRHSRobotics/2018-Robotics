package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HardwareDefinitions;

import java.util.List;

@Autonomous(name = "Mineral Detection", group = "Test")



//define
public class MineralDetection extends HardwareDefinitions {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AR1NGjD/////AAABmRNHhw4urkcYu6OsCz4GxO9HaxexcrZrSNGBfCYsc8miWAyyHlu53AsvQ0AMdhXKpFuLLm0Dej3xk4agW4J4tOXGu+hPnigkbDyr5HhVrGXPGxFyNCpJUHx+Sr6UMygVYr5b+z78sdhUeN2o4KBHClV+VzRnAuG0h4GiWh+58fPYhqIIRboPe41XAbmNWwCIqAG+1y5XXaENN0jq99vO4e4GgzYzQdAQtK4Jrq4pkIZev+fI5K2B500kIkiVv3YrnC1JkQNIfibntc+98DKcN7hbJ3TWJmHndB9vesnlzPnDEJ/q9j+V+w82/icXhZ58Jcu+QMu/iuo7eEZeCLQ8S5BqotKIbxP3mCW31jh93Btc"; //put vuforia key in quotes

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    ElapsedTime runtime = new ElapsedTime();

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

        dropFromLander();
        encoderDrive(0.4 ,-11, -11, 5);
        encoderTurn(0.25, 80, false, 5);

        runtime.reset();

        if (opModeIsActive()) {
            /** Start Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive() && runtime.seconds() < 5) {
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
                                telemetry.addData("Mineral Position:", "Left");
                                goldPosition = 1;
                            } else if (goldMineralX != -1){
                                if(goldMineralX < silverMineralX){
                                    telemetry.addData("Mineral Position:", "Center");
                                    goldPosition = 2;
                                } else if(goldMineralX > silverMineralX){
                                    telemetry.addData("Mineral Position:", "Right");
                                    goldPosition = 3;
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

        encoderTurn(0.25, 80, true, 5);

        if(goldPosition == 1){ //gold is left
            encoderTurn(0.25, 45, false, 5); //turn left and drive towards the gold
            encoderDrive(0.35, -20, -20, 10);

        } else if (goldPosition == 2){ //gold is center
            encoderDrive(0.35, -20, -20, 10); //drive straight towards the gold

        } else if(goldPosition == 3){ //gold is right
            encoderTurn(0.25, 45, true, 5); //turn right and drive towards the gold
            encoderDrive(0.35, -20, -20, 10);

        } else { //Tensorflow doesn't know
            encoderDrive(0.35, -20, -20, 10); //just go straight

        }

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


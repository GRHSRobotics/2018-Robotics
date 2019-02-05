package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HardwareDefinitions;
import org.firstinspires.ftc.teamcode.TFLiteHandler;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "VisionDepotSide - With Landing", group = "Vision")
public class VisionDepotSide extends AutonomousDefinitions {

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
        encoderDrive(0.4 ,14, 14, 5);
        //moveLanderWithEncoder((38*4), 8);
        encoderTurn(0.25, 105, false, 5);
        encoderDrive(0.4, 5, 5, 5);

        markerDropperOuter.setPosition(markerDropperOuterRelease);

        detectGold_inferRight(2);

        //movement stuff

        markerDropperOuter.setPosition(markerDropperOuterHold);


        switch(getMineralPosition(false)){
            case 1:
                //encoderDrive(0.4, 7, 7, 5);
                encoderTurn(0.25, 70, true, 5); //turn left and drive towards the gold
                encoderDrive(0.4, 27, 27, 10);
                encoderTurn(0.25, 70, true, 5);
                encoderDrive(0.4, 25, 21, 5);
                encoderTurn(0.25, 130, false, 5);

                //drop the marker
                moveBoxMechanism(2, 2);
                dropMarker();
                //moveBoxMechanism(-2, 3);


                //encoderTurn(0.25, 100, false, 5);

                break;

            case 2:

                encoderDrive(0.35, -5.5, -5.5, 5); //drive straight towards the gold
                encoderTurn(0.25, 105, true, 5);
                encoderDrive(0.35, 46, 46, 10);
                encoderTurn(0.25, 105, false, 5);

                //drop the marker
                moveBoxMechanism(2, 2);
                dropMarker();
                //moveBoxMechanism(-2, 3);

                encoderDrive(0.4, 10, 10, 5);

                break;


            case 3:

                encoderDrive(0.4, -17, -17, 5);
                encoderTurn(0.25, 130, true, 5); //turn right and drive towards the gold
                encoderDrive(0.35, 28, 28, 10);
                encoderTurn(0.25, 75, false, 5);

                encoderDrive(0.4, 29, 29, 5);
                encoderTurn(0.25, 80, false, 5);

                //drop the marker
                moveBoxMechanism(2, 2);
                dropMarker();
                //moveBoxMechanism(-2, 3);

                encoderDrive(0.4, 10, 10, 5);

                break;

            default:

                encoderDrive(0.4, 7, 7, 5);
                encoderTurn(0.25, 90, true, 5); //turn left and drive towards the gold
                encoderDrive(0.35, 23, 23, 10);
                encoderTurn(0.25, 75, true, 5);
                encoderDrive(0.4, 21, 21, 5);
                encoderTurn(0.25, 105, false, 5);

                //drop the marker
                moveBoxMechanism(2, 2);
                dropMarker();
                //moveBoxMechanism(-2, 3);

                break;
        }



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

    public int getMineralPosition(boolean useAveragingSystem){

        if(!useAveragingSystem){
            return currentDetectionValue;

        } else { //average all detection values to try to lower the chance of a fluke reading messing up the final result

            int detectionSum = 0;
            for(int i = 0; i < detectionValues.size(); i++){
                detectionSum += detectionValues.get(i);
            }

            double detectionAverage = detectionSum / detectionValues.size(); //should come out to a decimal between 1 and 3

            telemetry.addData("List Size: ", detectionValues.size()); //allows us to manually check if things are working properly
            telemetry.addData("List Sum: ", detectionSum);
            telemetry.addData("Calculated Average: ", detectionAverage);
            telemetry.update();

            if(detectionAverage < 1.5){
                return 1;

            } else if(detectionAverage > 2.5){
                return 3;

            } else {
                return 2;

            }

        }
    }

    /*
    Drive forwards until two minerals are centered in the camera's view
    Speed should be a low value so that the robot doesn't overshoot the minerals
     */
    public void driveToMinerals(double maxTimeS){

        int CAMERA_MAX_LEFT = 0;
        int CAMERA_MAX_RIGHT = 1920; //this should be the same as the horizontal number of pixels of the camera
        int ERROR_THRESHOLD = 20; //maximum tolerance for the minerals being off center

        boolean centered = false;

        int mineral1XLeft;
        int mineral2XRight;

        int leftDifference;
        int rightDifference;

        timer.reset();

        if (opModeIsActive()) {
            /** Start Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            motorL1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorL2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorR2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorL1.setPower(0.3);
            motorL2.setPower(0.3);
            motorR1.setPower(0.3);
            motorR2.setPower(0.3);

            while (opModeIsActive() && timer.seconds() < maxTimeS && !centered) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 2) {
                            motorL1.setPower(0.1);
                            motorL2.setPower(0.1);
                            motorR1.setPower(0.1);
                            motorR2.setPower(0.1);

                            mineral1XLeft = (int) updatedRecognitions.get(0).getLeft();
                            mineral2XRight = (int) updatedRecognitions.get(1).getRight();

                            leftDifference = Math.abs(CAMERA_MAX_LEFT - mineral1XLeft);
                            rightDifference = Math.abs(CAMERA_MAX_RIGHT - mineral2XRight);

                            if(leftDifference > rightDifference &&
                                    Math.abs(leftDifference - rightDifference) > ERROR_THRESHOLD){

                                motorL1.setPower(0.1); //the minerals are too far right in the camera frame, so keep driving
                                motorL2.setPower(0.1);
                                motorR1.setPower(0.1);
                                motorR2.setPower(0.1);
                            } else if(leftDifference < rightDifference &&
                                    Math.abs(leftDifference - rightDifference) > ERROR_THRESHOLD){

                                motorL1.setPower(-0.1); //the minerals are too far left in the camera frame, so drive in reverse
                                motorL2.setPower(-0.1);
                                motorR1.setPower(-0.1);
                                motorR2.setPower(-0.1);
                            } else {
                                motorL1.setPower(0); //the minerals are within the threshold, so stop movement
                                motorL2.setPower(0);
                                motorR1.setPower(0);
                                motorR2.setPower(0);

                                centered = true; //breaks loop

                            }

                        }
                    }
                }
            }
        }
    }


}


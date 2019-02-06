package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HardwareDefinitions;

import java.util.ArrayList;
import java.util.List;

//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.GyroSensor;


//See Google Drive for TODO

@TeleOp(name="DriverControlledTest", group="Test")
public class DriverControlledTest extends HardwareDefinitions {

    //LIFT SERVO VARIABLES
    boolean opener1Changed = false;
    boolean opener1On = false;

    boolean opener2Changed = false;
    boolean opener2On = false;

    //TANK DRIVE VARIABLES
    double rightPower;
    double leftPower;


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
    public void runOpMode(){
        init(hardwareMap);
        initIMU(hardwareMap);

        initTFodAndVuforia();


        telemetry.addData("Robot is initialized", "");

        waitForStart();
        telemetry.addData("Robot is started", "" );

        landerMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        landerMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while(opModeIsActive()){

            //TANK DRIVE

            rightPower = Range.clip(Math.pow(-gamepad1.right_stick_y, 3), -1, 1);
            leftPower = Range.clip(Math.pow(-gamepad1.left_stick_y, 3), -1, 1);



            motorL1.setPower(leftPower); //up on the stick is negative, so for up=forwards we need to
            motorL2.setPower(leftPower); //take the negative value of the stick
            motorR1.setPower(rightPower);
            motorR2.setPower(rightPower);

            //INTAKE SYSTEM TOGGLE
            if(gamepad1.dpad_up){
                intakeMotor.setPower(-1); //INTAKE
            }
            if(gamepad1.dpad_down){
                intakeMotor.setPower(0); //TURN OFF
            }
            if(gamepad1.dpad_left){
                intakeMotor.setPower(1); //REVERSE
            }

            //MARKER DROPPER SYSTEM
            if(gamepad1.a){
                markerDropperOuter.setPosition(markerDropperOuterHold);
                telemetry.addData("MDOuterHold", "");
            }
            if(gamepad1.b){
                markerDropperOuter.setPosition(markerDropperOuterRelease);
                telemetry.addData("MDOuterRelease", "");
            }

            //LIFT MOTOR
            if(gamepad1.left_trigger > 0){
                liftMotor.setPower(-Range.clip(gamepad1.left_trigger, 0, 1)); //clip method limits value to between 0 and 1
            } else if(gamepad1.right_trigger > 0){
                liftMotor.setPower(Range.clip(gamepad1.right_trigger, 0, 1));
            } else {
                liftMotor.setPower(0);
            }

            //DOOR OPENER 1 SYSTEM
            if(gamepad1.left_bumper && !opener1Changed) {
                opener1.setPosition(opener1On ? opener1Open : opener1Closed);
                opener1On = !opener1On;
                opener1Changed = true;
            } else if(!gamepad1.left_bumper) {
                opener1Changed = false;
            }

            //DOOR OPENER 2 SYSTEM
            if(gamepad1.right_bumper && !opener2Changed) {
                opener2.setPosition(opener2On ? opener2Open : opener2Closed); //this is a shorthand if/else statement, (condition ? if true : if false)
                opener2On = !opener2On;
                opener2Changed = true;
            } else if(!gamepad1.right_bumper) {
                opener2Changed = false;
            }

            //LANDING SYSTEM
            if(gamepad1.x){
                landerMotor1.setPower(-1); //bring the lift down
                landerMotor2.setPower(-1);
            } else if(gamepad1.y){
                landerMotor1.setPower(1); //bring the lift up
                landerMotor2.setPower(1);
            } else {
                landerMotor1.setPower(0); //stop the lift
                landerMotor2.setPower(0);
            }

            gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;



            telemetry.addData("Gyro heading:", gyroHeading);
            telemetry.addData("Raw Optical: ", rangeSensor.rawOptical());
            telemetry.addData("cm Optical: ", rangeSensor.cmOptical());
            telemetry.addData("Final inches", rangeSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();


        }
        stopRobot();
        telemetry.addData("Robot is stopped", "" );
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


}

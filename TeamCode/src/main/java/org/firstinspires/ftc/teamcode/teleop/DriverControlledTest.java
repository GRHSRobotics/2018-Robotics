package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousDefinitions;


//See Google Drive for TODO
@Disabled
@TeleOp(name="DriverControlledTest", group="Test")
public class DriverControlledTest extends AutonomousDefinitions {

    //LIFT SERVO VARIABLES
    boolean opener1Changed = false;
    boolean opener1On = false;

    boolean opener2Changed = false;
    boolean opener2On = false;

    //TANK DRIVE VARIABLES
    double rightPower;
    double leftPower;



    @Override
    public void runOpMode(){
        init(hardwareMap);
        initIMU(hardwareMap);

        initTFodAndVuforia();


        telemetry.addData("Robot is initialized", "");

        waitForStart();
        telemetry.addData("Robot is started", "" );

        landerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



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
                detectGold(inferMineral.RIGHT, 5000);
            }
            if(gamepad1.b){
                markerDropperOuter.setPosition(markerDropperOuterRelease);
                telemetry.addData("MDOuterRelease", "");
            }

            //LIFT MOTOR
            if(gamepad1.left_trigger > 0){
                liftMotor1.setPower(-Range.clip(gamepad1.left_trigger, 0, 1)); //clip method limits value to between 0 and 1
                liftMotor2.setPower(-Range.clip(gamepad1.left_trigger, 0, 1));
            } else if(gamepad1.right_trigger > 0){
                liftMotor1.setPower(Range.clip(gamepad1.right_trigger, 0, 1));
                liftMotor2.setPower(Range.clip(gamepad1.right_trigger, 0, 1));
            } else {
                liftMotor1.setPower(0);
                liftMotor2.setPower(0);
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
                landerMotor.setPower(-1); //bring the lift down
            } else if(gamepad1.y){
                landerMotor.setPower(1); //bring the lift up
            } else {
                landerMotor.setPower(0); //stop the lift
            }

            gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


/*
            telemetry.addData("Gyro heading:", gyroHeading);
            telemetry.addData("Raw Optical: ", rangeSensor.rawOptical());
            telemetry.addData("cm Optical: ", rangeSensor.cmOptical());
            telemetry.addData("Final inches", rangeSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
*/

        }
        stopRobot();
        telemetry.addData("Robot is stopped", "" );
    }




}

package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;

@TeleOp(name = "IntakeTest", group = "Test")
public class IntakeTest extends HardwareDefinitions {

    boolean useHingeEncoders = false;

    public int hingeCountsPerRotation = 288;
    public int hingeIncrement = 1;
    public int hingeInitialPosition;
    public int hingeMaxDelta = hingeCountsPerRotation / 4;
    public int hingeExtendedPosition;

    @Override
    public void runOpMode(){

        init(hardwareMap);
        telemetry.addData("Robot is initialized", "");

        waitForStart();
        hingeInitialPosition = intakeHinge.getCurrentPosition();
        hingeExtendedPosition = hingeInitialPosition + hingeMaxDelta;

        telemetry.addData("Robot is started", "" );


        while(opModeIsActive()){


            if(gamepad1.x){

            }

            if(gamepad1.y){

            }

            if(gamepad1.a){
                useHingeEncoders = true;
                telemetry.addData("useHingeEncoders: ", "true");
            }

            if(gamepad1.b){
                useHingeEncoders = false;
                telemetry.addData("useHingeEncoders: ", "false");
            }

            if(gamepad1.right_trigger > 0){
                intakeActuator.setPower(1);

            } else if(gamepad1.right_bumper){
                intakeActuator.setPower(-1);
            } else{
                intakeActuator.setPower(0);
            }

            if(useHingeEncoders) {
                intakeHinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (gamepad1.left_trigger > 0) {
                    if (intakeHinge.getCurrentPosition() < hingeExtendedPosition) {
                        intakeHinge.setTargetPosition(intakeHinge.getCurrentPosition() + hingeIncrement);
                        intakeHinge.setPower(1);
                    }
                } else if (gamepad1.left_bumper) {
                    if (intakeHinge.getCurrentPosition() > hingeInitialPosition) {
                        intakeHinge.setTargetPosition(intakeHinge.getCurrentPosition() - hingeIncrement);
                        intakeHinge.setPower(1);
                    }
                } else {
                    if (intakeHinge.isBusy()) {
                        intakeHinge.setPower(1);
                    } else {
                        intakeHinge.setPower(0);
                    }
                }
            } else {
                intakeHinge.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(gamepad1.left_trigger > 1){
                    intakeHinge.setPower(1);
                } else if (gamepad1.left_bumper){
                    intakeHinge.setPower(-1);
                } else{
                    intakeHinge.setPower(0);
                }
            }

            if(gamepad1.dpad_down){
                intakeSpinner.setPower(-1);
            }
            if(gamepad1.dpad_up){
                intakeSpinner.setPower(1);
            }
            if(gamepad1.dpad_right){
                intakeSpinner.setPower(0);
            }

            telemetry.update();

        }


    }


}

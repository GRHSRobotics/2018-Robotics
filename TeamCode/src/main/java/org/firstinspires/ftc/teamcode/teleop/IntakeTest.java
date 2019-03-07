package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;

@TeleOp(name = "IntakeTest", group = "Test")
public class IntakeTest extends HardwareDefinitions {



    public HingePosition hingePosition = HingePosition.UP;

    public boolean autoHinge = false;

    public int actuatorInitialPosition;

    public int actuatorLowerLimit; //absolute lower encoder limit for the actuator

    public int actuatorMinimumUp = 900; //minimum delta above the starting position the actuator must be

    @Override
    public void runOpMode(){

        init(hardwareMap);
        telemetry.addData("Robot is initialized", "");
        telemetry.update();

        waitForStart();
        actuatorInitialPosition = intakeActuator.getCurrentPosition();
        actuatorLowerLimit = actuatorInitialPosition + actuatorMinimumUp;
        telemetry.addData("Robot is started", "" );


        while(opModeIsActive()){


            if(gamepad1.x){
                autoHinge = true;
            }

            if(gamepad1.y){
                autoHinge = false;
            }

            if(gamepad1.a){
                hingePosition = HingePosition.DOWN;
            }

            if(gamepad1.b){
                hingePosition = HingePosition.UP;
            }
            if(intakeActuator.getCurrentPosition() - actuatorLowerLimit > 100){
                if(gamepad1.right_trigger > 0){
                    intakeActuator.setPower(1);

                } else if(gamepad1.right_bumper){
                    intakeActuator.setPower(-1);
                } else{
                    intakeActuator.setPower(0);
                }
            } else {
                if(gamepad1.right_trigger > 0){
                    intakeActuator.setPower(1);
                } else {
                    intakeActuator.setPower(0.5);
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

            telemetry.addData("autoHinge: ", autoHinge);

            if(autoHinge) {
                switch (hingePosition) {
                    case UP:
                        intakeHinge.setPower(getIntakePower(HingePosition.UP));
                        break;
                    case DOWN:
                        intakeHinge.setPower(getIntakePower(HingePosition.DOWN));
                        break;
                }
                telemetry.addData("Intake Position: ", hingePosition);
            } else {
                if(gamepad1.left_trigger > 0){
                    intakeHinge.setPower(-1); //up
                } else if (gamepad1.left_bumper){
                    intakeHinge.setPower(1); //down
                } else{
                    intakeHinge.setPower(0);
                }
            }

            telemetry.addData("potentiometer position: ", intakePotentiometer.getVoltage());
            telemetry.addData("actuator initial position: ", actuatorInitialPosition);
            telemetry.addData("actuator current position: ", intakeActuator.getCurrentPosition());
            telemetry.update();

        }


    }

    public void moveActuator(int encoderTicks, double maxTimeS){ //rotations should be 3.63
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            double counts_per_rotation = 36.4; //goBilda 1,150 RPM 5.2:1 gearbox motor

            ElapsedTime intakeActuatorRuntime = new ElapsedTime();

            int targetPosition = intakeActuator.getCurrentPosition() + encoderTicks;

            intakeActuator.setTargetPosition(targetPosition);

            // Turn On RUN_TO_POSITION
            intakeActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            intakeActuatorRuntime.reset();
            intakeActuator.setPower(1);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stopRobot.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (intakeActuator.isBusy())
                    && intakeActuatorRuntime.seconds() < maxTimeS) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", targetPosition);
                telemetry.addData("Path2", "Running at %7d",
                        intakeActuator.getCurrentPosition()
                );
                telemetry.update();
            }

            // Stop all motion;
            intakeActuator.setPower(0);

            sleep(100);

        }
    }


}

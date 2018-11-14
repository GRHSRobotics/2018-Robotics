package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;


@Autonomous(name="LinearCraterSide", group="Autonomous")
public class LinearCraterSide extends HardwareDefinitions {

    //DEFINE ROBOT HARDWARE
    HardwareDefinitions robot = new HardwareDefinitions();

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        //INITIALIZATION PERIOD
        robot.init(hardwareMap);

        robot.setDriveEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Robot is initialized", "");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        //START PERIOD

        //drive to crater, push center mineral
        robot.encoderDrive(0.5, 35, 35, 15);

        //back to in front of minerals
        robot.encoderDrive(0.5 ,-23, -23, 10);

        //turn 90 degrees
        robot.encoderTurn(0.25, 110, false, 10);

        //move towards the game wall
        robot.encoderDrive(0.5, 45, 45, 10);

        //turn so that the robot is parallel to the wall
        robot.encoderTurn(0.25, 73, false, 5);

        //move to the depot
        robot.encoderDrive(0.5, 50, 50, 10);

        //release team marker
        robot.markerDropper.setPosition(robot.markerDropperForward);
        sleep(1500);
        robot.markerDropper.setPosition(robot.markerDropperBack);

        //make sure that we don't hit crater side ball/mineral
        robot.encoderTurn(0.25, 8, false, 5);

        robot.encoderDrive(0.5, -78, -78, 10);


        //OLD non-encoder procedure
/*
        robot.motorL1.setPower(0.25);
        robot.motorL2.setPower(0.25);
        robot.motorR1.setPower(0.25);
        robot.motorR2.setPower(0.25);

        sleep(3500);

        robot.motorL1.setPower(0);
        robot.motorL2.setPower(0);
        robot.motorR1.setPower(0);
        robot.motorR2.setPower(0);
*/

        // run until the end of the match (driver presses STOP)
        /*while (opModeIsActive()) {
        }*/

        telemetry.addData("Completed Autonomous","");

    }
/*
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double maxTimeS) {


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position
            int newL1Target = robot.motorL1.getCurrentPosition() + (int) Math.round(leftInches * robot.COUNTS_PER_INCH);
            int newL2Target = robot.motorL2.getCurrentPosition() + (int) Math.round(leftInches * robot.COUNTS_PER_INCH);
            int newR1Target = robot.motorR1.getCurrentPosition() + (int) Math.round(rightInches * robot.COUNTS_PER_INCH);
            int newR2Target = robot.motorR2.getCurrentPosition() + (int) Math.round(rightInches * robot.COUNTS_PER_INCH);

            //give new target position to motors
            robot.motorL1.setTargetPosition(newL1Target);
            robot.motorL2.setTargetPosition(newL2Target);
            robot.motorR1.setTargetPosition(newR1Target);
            robot.motorR2.setTargetPosition(newR2Target);

            // Turn On RUN_TO_POSITION
            robot.motorL1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorL2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorR2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorL1.setPower(Math.abs(speed));
            robot.motorL2.setPower(Math.abs(speed));
            robot.motorR1.setPower(Math.abs(speed));
            robot.motorR2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stopRobot.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < maxTimeS) &&
                    (robot.motorL1.isBusy() && robot.motorL2.isBusy() && robot.motorR1.isBusy() && robot.motorR2.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newL1Target, newL2Target, newR1Target, newR2Target);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        robot.motorL1.getCurrentPosition(),
                        robot.motorL2.getCurrentPosition(),
                        robot.motorR1.getCurrentPosition(),
                        robot.motorR2.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorL1.setPower(0);
            robot.motorL2.setPower(0);
            robot.motorR1.setPower(0);
            robot.motorR2.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorL1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorL2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorR2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(400);
        }
    }

    public void encoderTurn(double speed, double angle, boolean clockwise, double maxTimeS){

        //determine inches to travel for each side using formula (pi * ROBOT_DIAMETER * angle)/360
        double leftInches = (robot.ROBOT_DIAMETER * angle * Math.PI) / 360;
        double rightInches = (robot.ROBOT_DIAMETER * angle * Math.PI) / 360;

        //Make sure that each motor is moving the right way
        if(clockwise){ //if the robot is turning clockwise, the left motors need to turn backwards
            leftInches = -leftInches;
        } else{ //if the robot is turning counterclockwise, the right motors need to turn backwards
            rightInches = -rightInches;
        }

        //plug the tread movement distances into the encoderDrive method
        encoderDrive(speed, leftInches, rightInches, maxTimeS);
    }
*/
}


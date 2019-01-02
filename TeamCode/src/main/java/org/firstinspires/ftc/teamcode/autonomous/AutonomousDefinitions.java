package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;

public class AutonomousDefinitions extends HardwareDefinitions {



    //CONSTANTS FOR DRIVE METHODS
    static final double COUNTS_PER_ROTATION = 560 ;    // REV HD Hex Motor 20:1
    static final double WHEEL_CIRCUMFERENCE_INCHES = 9.42 ;
    static final double COUNTS_PER_INCH = (COUNTS_PER_ROTATION) / (WHEEL_CIRCUMFERENCE_INCHES);
    static final double ROBOT_DIAMETER = 18; //in inches

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable





    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double maxTimeS) {


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            motorL1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            ElapsedTime runtime = new ElapsedTime();

            // Determine new target position
            int newL1Target = motorL1.getCurrentPosition() + (int) Math.round(leftInches * COUNTS_PER_INCH);
            int newL2Target = motorL2.getCurrentPosition() + (int) Math.round(leftInches * COUNTS_PER_INCH);
            int newR1Target = motorR1.getCurrentPosition() + (int) Math.round(rightInches * COUNTS_PER_INCH);
            int newR2Target = motorR2.getCurrentPosition() + (int) Math.round(rightInches * COUNTS_PER_INCH);

            //give new target position to motors
            motorL1.setTargetPosition(newL1Target);
            motorL2.setTargetPosition(newL2Target);
            motorR1.setTargetPosition(newR1Target);
            motorR2.setTargetPosition(newR2Target);

            // Turn On RUN_TO_POSITION
            motorL1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorL2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorL1.setPower(Math.abs(speed));
            motorL2.setPower(Math.abs(speed));
            motorR1.setPower(Math.abs(speed));
            motorR2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stopRobot.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < maxTimeS) &&
                    ((motorL1.isBusy() && motorL2.isBusy()) || (motorR1.isBusy() && motorR2.isBusy()))) { //looking at each tread individually allows single tread motion

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newL1Target, newL2Target, newR1Target, newR2Target);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        motorL1.getCurrentPosition(),
                        motorL2.getCurrentPosition(),
                        motorR1.getCurrentPosition(),
                        motorR2.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorL1.setPower(0);
            motorL2.setPower(0);
            motorR1.setPower(0);
            motorR2.setPower(0);

            // Turn off RUN_TO_POSITION
            motorL1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorL2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorR2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(400);
        }
    }

    public void encoderTurn(double speed, double angle, boolean clockwise, double maxTimeS){

        //determine inches to travel for each side using formula (pi * ROBOT_DIAMETER * angle)/360
        double leftInches = (ROBOT_DIAMETER * angle * Math.PI) / 360;
        double rightInches = (ROBOT_DIAMETER * angle * Math.PI) / 360;

        //Make sure that each motor is moving the right way
        if(clockwise){ //if the robot is turning clockwise, the left motors need to turn backwards
            rightInches = -rightInches;
        } else{ //if the robot is turning counterclockwise, the right motors need to turn backwards
            leftInches = -leftInches;
        }

        //plug the tread movement distances into the encoderDrive method
        encoderDrive(speed, leftInches, rightInches, maxTimeS);
    }

    public void dropFromLander(){

        moveLanderWithEncoder((100*4), 8);

        encoderTurn(0.4, 70, false, 5);

        //encoderDrive(0.4, 2, 2, 5);

        moveLanderWithEncoder((-20*4), 8);

        encoderTurn(0.4, 70, true, 5);

        telemetry.addData("Landing sequence:", "Complete");

    }

    public void moveLanderWithEncoder(double rotations, double maxTimeS){ //rotations should be 3.63
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            double counts_per_rotation = 145.6; //goBilda 1,150 RPM 5.2:1 gearbox motor
            double rotationsUp = rotations; //modify this one, will most likely be the same as rotationsDown

            ElapsedTime landerRuntime = new ElapsedTime();

            int targetPositionUp1 = landerMotor1.getCurrentPosition() + (int)(counts_per_rotation * rotationsUp);
            int targetPositionUp2 = landerMotor2.getCurrentPosition() + (int)(counts_per_rotation * rotationsUp);

            landerMotor1.setTargetPosition(targetPositionUp1);
            landerMotor2.setTargetPosition(targetPositionUp2);

            // Turn On RUN_TO_POSITION
            landerMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            landerMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            landerRuntime.reset();
            landerMotor1.setPower(0.4);
            landerMotor2.setPower(0.4);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stopRobot.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (landerMotor1.isBusy() && landerMotor2.isBusy())
                    && landerRuntime.seconds() < maxTimeS) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", targetPositionUp1, targetPositionUp2);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        landerMotor1.getCurrentPosition(),
                        landerMotor2.getCurrentPosition()
                );
                telemetry.update();
            }

            // Stop all motion;
            landerMotor1.setPower(0);
            landerMotor2.setPower(0);

            sleep(400);

        }
    }

    public void moveBoxMechanism(double rotations, double maxTimeS){
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            double counts_per_rotation = 288; //core hex motor
            double rotationsUp = rotations; //modify this one, will most likely be the same as rotationsDown

            ElapsedTime liftRuntime = new ElapsedTime();

            int liftTarget = liftMotor.getCurrentPosition() + (int)(counts_per_rotation * rotationsUp);

            liftMotor.setTargetPosition(liftTarget);


            // Turn On RUN_TO_POSITION
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            liftRuntime.reset();
            liftMotor.setPower(0.4);



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stopRobot.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    liftMotor.isBusy()
                    && liftRuntime.seconds() < maxTimeS) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", liftTarget);
                telemetry.addData("Path2", "Running at %7d",
                        liftMotor.getCurrentPosition()

                );
                telemetry.update();
            }

            // Stop all motion;
            liftMotor.setPower(0);


            sleep(400);

        }

    }

    public void dropMarker(){

        markerDropperOuter.setPosition(markerDropperOuterRelease);
        sleep(1000);
        markerDropperInner.setPosition(markerDropperInnerRelease);
        sleep(1500);
        markerDropperInner.setPosition(markerDropperInnerHold);
        sleep(1000);
        markerDropperOuter.setPosition(markerDropperOuterHold);

    }
/*
    public void gyroTurnAbsolute(double speed, double angle, double maxTimeS){
        ElapsedTime runtime = new ElapsedTime();

        telemetry.addData("Gyro heading:", getGyroHeading());

        motorL1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorL1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        runtime.reset();

        while(opModeIsActive() && (Math.abs(getGyroHeading() - angle) > 3) && (runtime.seconds() < maxTimeS)){
            if(angle > 0){ //if turn is left
                motorL1.setPower(-speed);
                motorL2.setPower(-speed);
                motorR1.setPower(speed);
                motorR2.setPower(speed);
            }
            else if(angle < 0){ //if turn is right
                motorL1.setPower(speed);
                motorL2.setPower(speed);
                motorR1.setPower(-speed);
                motorR2.setPower(-speed);
            }

            telemetry.addData("Gyro heading:", getGyroHeading());
        }
        //turn off motor power when turn is done
        motorL1.setPower(0);
        motorL2.setPower(0);
        motorR1.setPower(0);
        motorR2.setPower(0);
        sleep(250);
    }
    public void gyroTurn(double speed, double target, double maxTimeS){
        gyroTurnAbsolute(speed, (getGyroHeading() + target), maxTimeS);
    }

    public double getGyroHeading(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
*/
    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed,
                             double distance, double angle,
                             double maxTimeS) {


        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        double max;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            motorL1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            ElapsedTime runtime = new ElapsedTime();

            // Determine new target position
            int newL1Target = motorL1.getCurrentPosition() + (int) Math.round(distance * COUNTS_PER_INCH);
            int newL2Target = motorL2.getCurrentPosition() + (int) Math.round(distance * COUNTS_PER_INCH);
            int newR1Target = motorR1.getCurrentPosition() + (int) Math.round(distance * COUNTS_PER_INCH);
            int newR2Target = motorR2.getCurrentPosition() + (int) Math.round(distance * COUNTS_PER_INCH);

            //give new target position to motors
            motorL1.setTargetPosition(newL1Target);
            motorL2.setTargetPosition(newL2Target);
            motorR1.setTargetPosition(newR1Target);
            motorR2.setTargetPosition(newR2Target);

            // Turn On RUN_TO_POSITION
            motorL1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorL2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorL1.setPower(Math.abs(speed));
            motorL2.setPower(Math.abs(speed));
            motorR1.setPower(Math.abs(speed));
            motorR2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stopRobot.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < maxTimeS) &&
                    ((motorL1.isBusy() && motorL2.isBusy()) || (motorR1.isBusy() && motorR2.isBusy()))) { //looking at each tread individually allows single tread motion

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                motorL1.setPower(leftSpeed);
                motorL2.setPower(leftSpeed);
                motorR1.setPower(rightSpeed);
                motorR2.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newL1Target, newL2Target, newR1Target, newR2Target);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        motorL1.getCurrentPosition(),
                        motorL2.getCurrentPosition(),
                        motorR1.getCurrentPosition(),
                        motorR2.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            motorL1.setPower(0);
            motorL2.setPower(0);
            motorR1.setPower(0);
            motorR2.setPower(0);

            // Turn off RUN_TO_POSITION
            motorL1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorL2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorR2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(400);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle, double maxTimeS) {
        ElapsedTime timer = new ElapsedTime();
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF) && timer.seconds() < maxTimeS) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        motorL1.setPower(0);
        motorL2.setPower(0);
        motorR1.setPower(0);
        motorR2.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        motorL1.setPower(leftSpeed);
        motorL2.setPower(leftSpeed);
        motorR1.setPower(rightSpeed);
        motorR2.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}

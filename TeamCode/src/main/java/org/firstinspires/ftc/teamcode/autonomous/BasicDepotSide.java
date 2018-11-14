package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.GyroSensor;


@Disabled
@Autonomous(name = "BasicDepotSide", group = "Autonomous")
public class BasicDepotSide extends OpMode {

    //DEFINE ROBOT HARDWARE
    HardwareDefinitions robot = new HardwareDefinitions();

    //CONSTANTS
    int toDepot;

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Robot is initialized", "");
    }

    @Override
    public void init_loop(){
    }

    @Override
    public void start(){

        telemetry.addData("Robot is started", "" );


        //SET MOTOR POWER
        robot.motorL1.setPower(0.25);
        robot.motorL2.setPower(0.25);
        robot.motorR1.setPower(0.25);
        robot.motorR2.setPower(0.25);

        //ENABLE ENCODERS
        robot.motorL1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorL2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorR2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //robot.sleep(1000);

        //DRIVE ROBOT TO RIGHT IN FRONT OF THE 3 BALLS
        robot.motorL1.setTargetPosition(toDepot);
        robot.motorL2.setTargetPosition(toDepot);
        robot.motorR1.setTargetPosition(toDepot);
        robot.motorR2.setTargetPosition(toDepot);

        //robot.sleep(1000);

        //RELEASE TEAM MARKER
        robot.markerDropper.setPosition(robot.markerDropperForward);
        //robot.sleep(1000);
        robot.markerDropper.setPosition(robot.markerDropperBack);


        telemetry.addData("Robot has completed autonomous", "");
    }

    @Override
    public void loop(){}

    @Override
    public void stop(){
        robot.stopRobot();
        telemetry.addData("Robot is stopped", "" );
    }

}

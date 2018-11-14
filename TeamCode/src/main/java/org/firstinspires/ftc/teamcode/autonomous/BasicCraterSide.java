package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareDefinitions;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.GyroSensor;


@Disabled
@Autonomous(name = "BasicCraterSide", group = "Autonomous")
public class BasicCraterSide extends OpMode {

    //DEFINE ROBOT HARDWARE
    HardwareDefinitions robot = new HardwareDefinitions();

    //CONSTANTS
    int rotations = 6; //circumference is 9.42in
    int toCrater = 1120*rotations; //-1 * counts per rotation * rotations

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

        telemetry.addData("Initial Encoder Value", robot.motorL1.getCurrentPosition());

        //ENABLE ENCODERS
        robot.motorL1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorL2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorR2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //DRIVE ROBOT TO RIGHT IN FRONT OF THE 3 BALLS
        robot.motorL1.setTargetPosition(toCrater);
        robot.motorL2.setTargetPosition(toCrater);
        robot.motorR1.setTargetPosition(toCrater);
        robot.motorR2.setTargetPosition(toCrater);

        do{
            robot.motorL1.setPower(-0.25);
            robot.motorL2.setPower(-0.25);
            robot.motorR1.setPower(-0.25);
            robot.motorR2.setPower(-0.25);
        }while((robot.motorL1.getCurrentPosition()+robot.motorL2.getCurrentPosition()+robot.motorR1.getCurrentPosition()+robot.motorR2.getCurrentPosition())/4 < toCrater);

        robot.motorL1.setPower(0);
        robot.motorL2.setPower(0);
        robot.motorR1.setPower(0);
        robot.motorR2.setPower(0);

        telemetry.addData("Final Encoder Value", robot.motorL1.getCurrentPosition());

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

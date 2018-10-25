package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "BasicForward", group = "Autonomous")
public class BasicForward extends OpMode {

    //DEFINE ROBOT HARDWARE
    HardwareDefinitions robot = new HardwareDefinitions();

    //CONSTANTS
    int beforeBalls;

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
        robot.motorL1.setPower(-0.25);
        robot.motorL2.setPower(-0.25);
        robot.motorR1.setPower(-0.25);
        robot.motorR2.setPower(-0.25);

       //robot.sleep(5000);

        robot.motorL1.setPower(0);
        robot.motorL2.setPower(0);
        robot.motorR1.setPower(0);
        robot.motorR2.setPower(0);


        telemetry.addData("Robot has completed autonomous", "");
    }

    @Override
    public void loop(){}

    @Override
    public void stop(){
        robot.stop();
        telemetry.addData("Robot is stopped", "" );
    }

}

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.hardwareMap;
import org.firstinspires.ftc.teamcode.purepursuit.RobotMovement;

public class BasicOpMode extends OpMode {

    hardwareMap robot = new hardwareMap();
    RobotMovement robotMovement;


    @Override
    public void init(){
        robot.init(hardwareMap);
        robotMovement  = new RobotMovement(robot);


    }

    @Override
    public void loop(){

    }
}

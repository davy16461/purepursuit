package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.hardwareMecanum;
import org.firstinspires.ftc.teamcode.purepursuit.RobotMovement;
import org.firstinspires.ftc.teamcode.utilities.MoveVars;

@Autonomous(name="MovementTest", group="PP")
public class MoveForwardTest extends LinearOpMode {

    hardwareMecanum robot = new hardwareMecanum();
    RobotMovement robotMovement;


    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        robotMovement  = new RobotMovement(robot);

        robot.mecanum.driveRobotCentric(0,1,0);
        sleep(300);
        robot.mecanum.driveRobotCentric(0,0,0);
    }

}

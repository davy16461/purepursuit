package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.hardwareMecanum;
import org.firstinspires.ftc.teamcode.purepursuit.RobotMovement;
import org.firstinspires.ftc.teamcode.utilities.MoveVars;
import org.firstinspires.ftc.teamcode.utilities.MoveVars.*;

@Autonomous(name="Test", group="PP")
public class BasicOpMode extends OpMode {

    hardwareMecanum robot = new hardwareMecanum();
    RobotMovement robotMovement;


    @Override
    public void init(){
        robot.init(hardwareMap);
        robotMovement  = new RobotMovement(robot);

        //robot.mecanum.driveRobotCentric(0,1,0);


    }

    @Override
    public void loop(){
        robot.odometry.updatePose();
        if(!robotMovement.goToPosition(0,100,0.7,0,0.7)) {

            robot.mecanum.driveFieldCentric(MoveVars.movement_x, -MoveVars.movement_y, MoveVars.movement_turn, robot.checkOrientation());

        }else{
            stop();
        }
        telemetry.addData("MoveX", MoveVars.movement_x);
        telemetry.addData("MoveY", MoveVars.movement_y);
        telemetry.addData("MoveTurn", MoveVars.movement_turn);

        telemetry.addData("X", robot.position.getX());
        telemetry.addData("Y", robot.position.getY());
        telemetry.addData("Heading", robot.position.getH());
    }
}

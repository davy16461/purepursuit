package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.hardwareMecanum;
import org.firstinspires.ftc.teamcode.purepursuit.RobotMovement;
import org.firstinspires.ftc.teamcode.utilities.MoveVars;

@Autonomous(name="LocalizationTest", group="PP")
public class LocalizationTest extends OpMode {

    hardwareMecanum robot = new hardwareMecanum();
    RobotMovement robotMovement;


    @Override
    public void init(){
        robot.init(hardwareMap);
        robotMovement  = new RobotMovement(robot);
    }

    @Override
    public void loop(){
        robot.odometry.updatePose();
        //robotMovement.goToPosition(100,0,0.5,0,0.5);
        //robot.mecanum.driveRobotCentric(MoveVars.movement_y, MoveVars.movement_x, MoveVars.movement_turn);

        telemetry.addData("X", robot.odometry.getPose().getTranslation().getX());
        telemetry.addData("Y", robot.odometry.getPose().getTranslation().getY());
        telemetry.addData("Heading", robot.odometry.getPose().getHeading());
    }
}

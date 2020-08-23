package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.hardware.hardwareMecanum;

public class Positions {
    hardwareMecanum robot;
    public Positions(hardwareMecanum ahwmap){
        robot = ahwmap;
    }

    public double getX(){
        return robot.odometry.getPose().getTranslation().getX();
    }

    public double getY(){
        return robot.odometry.getPose().getTranslation().getY();
    }

    public double getH(){
        return robot.odometry.getPose().getHeading();
    }
}

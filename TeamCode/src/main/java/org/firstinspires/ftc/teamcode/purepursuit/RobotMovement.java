package org.firstinspires.ftc.teamcode.purepursuit;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.hardwareMap;
import org.firstinspires.ftc.teamcode.utilities.MathFunctions;
import org.opencv.core.Point;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.utilities.MathFunctions.lineCircleIntersection;
import static org.firstinspires.ftc.teamcode.utilities.MoveVars.movement_turn;
import static org.firstinspires.ftc.teamcode.utilities.MoveVars.movement_x;
import static org.firstinspires.ftc.teamcode.utilities.MoveVars.movement_y;

import static org.firstinspires.ftc.teamcode.utilities.MathFunctions.AngleWrap;

public class RobotMovement {
    hardwareMap robot;

    public RobotMovement(hardwareMap ahwmap){
        robot = ahwmap;
    }

    public void followCurve(ArrayList<CurvePoint> allPoints, double followAngle){
        for(int i = 0; i <  allPoints.size()-1; i++){
            //Debug in video
        }
        CurvePoint followMe = getFollowPointPath(allPoints, new Point(robot.odometry.getPose().getTranslation().getX(), robot.odometry.getPose().getTranslation().getY()), allPoints.get(0).followDistance);

        goToPosition(followMe.x,followMe.y,followMe.moveSpeed,followAngle,followMe.turnSpeed);

    }


    public CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for(int i=0; i < pathPoints.size()-1; i++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i+1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());

            //just a big number
            double closestAngle = 1000000;

            for(Point thisIntersection : intersections){
                double globalY = robot.odometry.getPose().getTranslation().getY();
                double globalX = robot.odometry.getPose().getTranslation().getX();
                double globalHeading = robot.odometry.getPose().getHeading();
                double angle = Math.atan2(thisIntersection.y - globalY, thisIntersection.x - globalX);
                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle-globalHeading));

                if(deltaAngle<closestAngle){
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }

            }
        }
        return followMe;
    }

    public void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed){
        double globalY = robot.odometry.getPose().getTranslation().getY();
        double globalX = robot.odometry.getPose().getTranslation().getX();
        double globalHeading = robot.odometry.getPose().getHeading();

        double distanceToTarget = Math.hypot(x-globalX, y-globalY);
        double angleToTarget = Math.atan2(y-globalY,x-globalX);

        double relativeAngleToTarget = AngleWrap(angleToTarget - globalHeading - Math.toRadians(90));
        double relativeXToTarget = Math.cos(relativeAngleToTarget) * distanceToTarget;
        double relativeYToTarget = Math.sin(relativeAngleToTarget) * distanceToTarget;

        double movementXPower = relativeXToTarget / (Math.abs(relativeXToTarget) + Math.abs(relativeYToTarget));
        double movementYPower = relativeYToTarget / (Math.abs(relativeXToTarget) + Math.abs(relativeYToTarget));

        movement_x = movementXPower;
        movement_y = movementYPower;

        double relativeTurnAngle = relativeAngleToTarget - Math.toRadians(180) + preferredAngle;
        movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;

        if(distanceToTarget<10){
            movement_turn = 0;
        }
    }
}

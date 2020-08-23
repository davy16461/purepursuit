package org.firstinspires.ftc.teamcode.purepursuit;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.hardwareMecanum;
import org.firstinspires.ftc.teamcode.utilities.MathFunctions;
import org.opencv.core.Point;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.utilities.MathFunctions.lineCircleIntersection;
import static org.firstinspires.ftc.teamcode.utilities.MoveVars.movement_turn;
import static org.firstinspires.ftc.teamcode.utilities.MoveVars.movement_x;
import static org.firstinspires.ftc.teamcode.utilities.MoveVars.movement_y;

import static org.firstinspires.ftc.teamcode.utilities.MathFunctions.AngleWrap;

public class RobotMovement {
    hardwareMecanum robot;

    public static profileStates state_movement_y_prof = profileStates.gunningIt;
    public static profileStates state_movement_x_prof = profileStates.gunningIt;
    public static profileStates state_turning_prof = profileStates.gunningIt;

    public static double movement_y_min = 0.091;
    public static double movement_x_min = 0.11;
    public static double movement_turn_min = 0.10;


    public enum profileStates{
        gunningIt,
        slipping,
        fineAdjustment,

        memes;

        private static profileStates[] vals = values();
        public profileStates next(){
            return vals[(this.ordinal()+1) % vals.length];
        }
    }

    public static final double smallAdjustSpeed = 0.135;


    public RobotMovement(hardwareMecanum ahwmap){
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
                double globalY = robot.position.getY();
                double globalX = robot.position.getX();
                double globalHeading = robot.position.getH();
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

    public boolean goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed){
        double globalY = robot.position.getY();
        double globalX = robot.position.getX();
        double globalHeading = robot.position.getH();

        double distanceToTarget = Math.hypot(x-globalX, y-globalY);

        double angleToTarget = Math.atan2(y-globalY,x-globalX);

        double relativeAngleToTarget = AngleWrap(angleToTarget - (globalHeading-Math.toRadians(90)));
        double relativeXToTarget = Math.cos(relativeAngleToTarget) * distanceToTarget;
        double relativeYToTarget = Math.sin(relativeAngleToTarget) * distanceToTarget;

        double relative_abs_x = Math.abs(relativeXToTarget);
        double relative_abs_y = Math.abs(relativeYToTarget);

        double movementXPower = (relativeXToTarget / (relative_abs_x+relative_abs_y))*movementSpeed;
        double movementYPower = (relativeYToTarget / (relative_abs_x+relative_abs_y))*movementSpeed;

        movement_x = movementXPower;
        movement_y = movementYPower;

        //every movement has two states, the fast "gunning" section and the slow refining part. turn this var off when close to target
        if(state_movement_y_prof == profileStates.gunningIt) {
            if(relative_abs_y < Math.abs(SpeedOmeter.currSlipDistanceY() * 2) || relative_abs_y < 3){
                state_movement_y_prof = state_movement_y_prof.next();
            }
        }
        if(state_movement_y_prof == profileStates.slipping){
            movementYPower = 0;
            if(Math.abs(SpeedOmeter.getSpeedY()) <  0.03){
                state_movement_y_prof = state_movement_y_prof.next();
            }
        }
        if(state_movement_y_prof == profileStates.fineAdjustment){
            movementYPower = Range.clip(((relativeYToTarget/8.0) * 0.15),-0.15,0.15);
        }

        if(state_movement_x_prof == profileStates.gunningIt) {
            if(relative_abs_x < Math.abs(SpeedOmeter.currSlipDistanceY() * 1.2) || relative_abs_x < 3){
                state_movement_x_prof = state_movement_x_prof.next();
            }
        }
        if(state_movement_x_prof == profileStates.slipping){
            movementXPower = 0;
            if(Math.abs(SpeedOmeter.getSpeedY()) < 0.03){
                state_movement_x_prof = state_movement_x_prof.next();
            }
        }
        if(state_movement_x_prof == profileStates.fineAdjustment){
            movementXPower = Range.clip(((relativeXToTarget/2.5) * smallAdjustSpeed),-smallAdjustSpeed,smallAdjustSpeed);
        }

        double rad_to_target = AngleWrap(preferredAngle-globalHeading);
        double turnPower = 0;

        //every movement has two states, the fast "gunning" section and the slow refining part. turn this var off when close to target
        if(state_turning_prof == profileStates.gunningIt) {
            turnPower = rad_to_target > 0 ? movementSpeed : -movementSpeed;
            if(Math.abs(rad_to_target) < Math.abs(SpeedOmeter.currSlipAngle() * 1.2) || Math.abs(rad_to_target) < Math.toRadians(3.0)){
                state_turning_prof = state_turning_prof.next();
            }

        }
        if(state_turning_prof == profileStates.slipping){
            if(Math.abs(SpeedOmeter.getDegPerSecond()) < 60){
                state_turning_prof = state_turning_prof.next();
            }

        }

        if(state_turning_prof == profileStates.fineAdjustment){
            //this is a var that will go from 0 to 1 in the course of 10 degrees from the target
            turnPower = (rad_to_target/Math.toRadians(10)) * smallAdjustSpeed;
            turnPower = Range.clip(turnPower,-smallAdjustSpeed,smallAdjustSpeed);
        }

        movement_turn = turnPower;
        movement_x = movementXPower;
        movement_y = movementYPower;

        allComponentsMinPower();

        if(distanceToTarget < 3){
            return true;
        }else{
            return false;
        }

        /*
        double relativeTurnAngle = relativeAngleToTarget + preferredAngle + Math.toRadians(180);
        movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;

        if(distanceToTarget<10){
            movement_turn = 0;
        }

            double[] toReturn = new double[] {angleToTarget, distanceToTarget};
        return toReturn;

         */
    }

    //makes sure the largest movement component doesn't fall under the threshold of movement
    private static void allComponentsMinPower() {
        if(Math.abs(movement_x) > Math.abs(movement_y)){
            if(Math.abs(movement_x) > Math.abs(movement_turn)){
                movement_x = minPower(movement_x,movement_x_min);
            }else{
                movement_turn = minPower(movement_turn,movement_turn_min);
            }
        }else{
            if(Math.abs(movement_y) > Math.abs(movement_turn)){
                movement_y = minPower(movement_y, movement_y_min);
            }else{
                movement_turn = minPower(movement_turn,movement_turn_min);
            }
        }
    }

    public static double minPower(double val, double min){
        if(val >= 0 && val <= min){
            return min;
        }
        if(val < 0 && val > -min){
            return -min;
        }
        return val;
    }
}

package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.ConstantVeloMecanumOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.function.DoubleSupplier;

public class hardwareMap {
    //Declare Motors
    public yellowJacket435 fl = null;
    public yellowJacket435 fr = null;
    public yellowJacket435 bl = null;
    public yellowJacket435 br = null;

    //Declare hwMap
    HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();

    BNO055IMU imu;
    Orientation angles;

    public DoubleSupplier leftValue, rightValue, horizontalValue, heading;
    public MecanumDrive mecanum;
    public ConstantVeloMecanumOdometry odometry;



    public hardwareMap(){

    }

    public double checkOrientation() {
        // read the orientation of the robot
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        this.imu.getPosition();
        // and save the heading
        double curHeading = angles.firstAngle;
        return curHeading;
    }

    public double ticksToCM(double input) {
        return input*8196*3.8*Math.PI;
    }

    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        //Assign Motors to custom motor class
        fl = new yellowJacket435(hwMap, "fl");
        fr = new yellowJacket435(hwMap, "fr");
        br = new yellowJacket435(hwMap, "br");
        bl = new yellowJacket435(hwMap, "bl");

        //Inversions
        fl.setInverted(false);
        fr.setInverted(false);
        br.setInverted(false);
        bl.setInverted(false);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        heading = () -> checkOrientation();
        leftValue = () -> -ticksToCM(fl.getEncoderCount());
        rightValue = () -> ticksToCM(fr.getEncoderCount());
        horizontalValue = () -> -ticksToCM(br.getEncoderCount());

        mecanum = new MecanumDrive(fl, fr, bl, br);
        odometry = new ConstantVeloMecanumOdometry(heading, leftValue, rightValue, horizontalValue, 12.8, 7);


    }

}

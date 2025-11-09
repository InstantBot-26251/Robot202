package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Chassis {
    public DcMotorEx fl;
    public DcMotorEx fr;
    public DcMotorEx bl;
    public DcMotorEx br;

    IMU imu;
//    SparkFunOTOS otos;


    public static double modifierFL = 0.88095238095;
    public static double modifierFR = 0.89516129032;
    public static double modifierBR = 0.9652173913;

    public Chassis (HardwareMap hardwareMap) {

        // Initialize motors
        fl = hardwareMap.get(DcMotorEx.class, "lf");
        fr = hardwareMap.get(DcMotorEx.class, "rf");
        bl = hardwareMap.get(DcMotorEx.class, "lr");
        br = hardwareMap.get(DcMotorEx.class, "rr");

        // Set motor directions and zero power behavior
        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        bl.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.REVERSE);

        // Initialize x
        imu = hardwareMap.get(IMU.class, "imu");

//        // Set up IMU parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

        // Initialize x
//        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
//        otos.setOffset(new SparkFunOTOS.Pose2D(2.5,3.75, Math.PI));

    }
    public void resetYaw() {
        imu.resetYaw();
    }

    public void drive(double x, double y, double rx) {
//        // Get the robot's current heading
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        double botHeading = otos.getPosition().h;

        // Adjust the input values for field-centric control
        double adjustedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double adjustedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        adjustedX = adjustedX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(adjustedY) + Math.abs(adjustedX) + Math.abs(rx), 1);
        double frontLeftPower = (adjustedY + adjustedX + rx) / denominator;
        double backLeftPower = (adjustedY - adjustedX + rx) / denominator;
        double frontRightPower = (adjustedY - adjustedX - rx) / denominator;
        double backRightPower = (adjustedY + adjustedX - rx) / denominator;

        // Set motor powers
        fl.setPower(frontLeftPower * modifierFL);
        fr.setPower(frontRightPower * modifierFR);
        bl.setPower(backLeftPower);
        br.setPower(backRightPower * modifierBR);
    }

    public void moveBackward() {
        fl.setPower(-0.5);
        fr.setPower(-0.5);
        bl.setPower(-0.5);
        br.setPower(-0.5);
    }

    public void stopMotors() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

//    public void resetYaw() {
//        otos.resetTracking();
//    }
}
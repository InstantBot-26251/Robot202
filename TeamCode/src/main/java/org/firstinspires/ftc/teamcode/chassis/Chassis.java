package org.firstinspires.ftc.teamcode.chassis;

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

    public Chassis (HardwareMap hardwareMap) {
        // TODO: Add Modifiers
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

        // Set up IMU parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);
    }
    public void resetYaw() {
        imu.resetYaw();
    }

    public void drive(double x, double y, double rx) {
        // Get the robot's current heading
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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
        fl.setPower(frontLeftPower);
        fr.setPower(frontRightPower);
        bl.setPower(backLeftPower);
        br.setPower(backRightPower);
    }
}
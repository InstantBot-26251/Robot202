package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Chassis {
    public DcMotor fl, fr, bl, br;
//    IMU imu;

    public Chassis(HardwareMap hardwareMap) {
        // Initialize motors
        fl = hardwareMap.get(DcMotor.class, "lf");
        fr = hardwareMap.get(DcMotor.class, "rf");
        bl = hardwareMap.get(DcMotor.class, "lr");
        br = hardwareMap.get(DcMotor.class, "rr");

        // Set motor directions and zero power behavior
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);


//        // Initialize x
//        imu = hardwareMap.get(IMU.class, "imu");
//
//        // Set up IMU parameters
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
//        ));
//        imu.initialize(parameters);
    }
//    public void resetYaw() {
//        imu.resetYaw();
//    }


    public void drive(double x, double y, double rx) {
        // Get the robot's current heading
//        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

//        // Adjust the input values for field-centric control
//        double adjustedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double adjustedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
        x = x * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // Set motor powers
        fl.setPower(frontLeftPower);
        fr.setPower(frontRightPower);
        bl.setPower(backLeftPower);
        br.setPower(backRightPower);
    }
}
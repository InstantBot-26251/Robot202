package org.firstinspires.ftc.teamcode.chassis;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Chassis {
    public DcMotorEx fl, fr, bl, br;
    SparkFunOTOS otos;

    public static double modifierFL = 0.88095238095;
    public static double modifierFR = 0.89516129032;
    public static double modifierBR = 0.9652173913;

    public Chassis(HardwareMap hardwareMap) {
        // Initialize motors
        fl = hardwareMap.get(DcMotorEx.class, "lf");
        fr = hardwareMap.get(DcMotorEx.class, "rf");
        bl = hardwareMap.get(DcMotorEx.class, "lr");
        br = hardwareMap.get(DcMotorEx.class, "rr");

        // Set motor directions and zero power behavior
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);


// Initialize x
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.setOffset(new SparkFunOTOS.Pose2D(2.5,3.75,Math.PI / 2));

    }


    public void drive(double x, double y, double rx) {
    //     Get the robot's current heading
        double botHeading = otos.getPosition().h;

        // Adjust the input values for field-centric control
        double adjustedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double adjustedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        adjustedX = adjustedX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
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

    public void resetYaw() {
        otos.resetTracking();
    }
}
package org.firstinspires.ftc.teamcode.util.testing.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo; // TODO

import org.firstinspires.ftc.teamcode.robot.RobotMap;
//import org.firstinspires.ftc.teamcode.shooter.Hood;// TODO
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.util.hardware.InstantMotor;
//import org.firstinspires.ftc.teamcode.util.math.MathPM; // TODO

@Config
@TeleOp(name = "Shooter Testing", group = "testing")
public class ShooterTestingOpMode extends OpMode {
    DcMotorEx shooter;
    CRServo hood;

    @Override
    public void init() {
        hood = hardwareMap.get(CRServo.class, "hood");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop() {
        double shooterVelocity = gamepad2.left_stick_y;
//        hood.setAngle(MathPM.calculateLaunchAngle(5, shooterVelocity, MathPM.inchesToMeters(7)));
        double hoodAngle = gamepad2.right_stick_y;
        shooter.setPower(shooterVelocity);
        hood.setPower(hoodAngle);
    }
}

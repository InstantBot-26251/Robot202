package org.firstinspires.ftc.teamcode.util.testing.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.shooter.Hood;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.util.Math.MathPM;

@Config
@TeleOp(name = "Shooter Testing", group = "testing")
public class ShooterTestingOpMode extends OpMode {
    Shooter shooter;
    Hood hood;

    @Override
    public void init() {
        RobotMap.getInstance().init(hardwareMap);
        shooter = Shooter.getInstance();
        hood = Hood.getInstance();
        shooter.onTeleopInit();
        hood.onTeleopInit();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void init_loop() {
        telemetry.addData("Shooter State: ", shooter.getState());
        telemetry.addData("Hood Angle: ", hood.getCurrentAngle());
        telemetry.update();
    }

    @Override
    public void loop() {
        double shooterVelocity = gamepad2.left_stick_y;
        hood.setAngle(MathPM.calculateLaunchAngle(5, shooterVelocity, MathPM.inchesToMeters(7)));
        shooter.startShooting(shooterVelocity);
    }
}

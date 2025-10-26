package org.firstinspires.ftc.teamcode.util.testing.outreach;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Outreach OpMode")
public class Outreach extends OpMode {
    DcMotorEx shooter;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        if (gamepad2.y) {
            shooter.setPower(0.25);
        }
        if (gamepad2.x) {
            shooter.setPower(0.5);
        }
        if (gamepad2.b) {
            shooter.setPower(0.75);
        }
        if (gamepad2.a) {
            shooter.setPower(1);
        }
        if (gamepad2.right_bumper) {
            shooter.setPower(0);
        }
    }
}

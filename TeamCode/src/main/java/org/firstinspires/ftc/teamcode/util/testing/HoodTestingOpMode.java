package org.firstinspires.ftc.teamcode.util.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.shooter.Hood;

@TeleOp
public class HoodTestingOpMode extends OpMode {
    Servo hood;

    @Override
    public void init() {
        hood = hardwareMap.get(Servo.class, "hood");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        if (gamepad2.y) {
            hood.setPosition(50);
        }
        if (gamepad2.x) {
            hood.setPosition(100);
        }
        if (gamepad2.b) {
            hood.setPosition(150);
        }
        if (gamepad2.a) {
            hood.setPosition(200);
        }
    }
}

package org.firstinspires.ftc.teamcode.util.testing.shooter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class DualShooterTesting extends OpMode {
    DcMotorEx shooter1, shooter2;

    @Override
    public void init() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        if (gamepad2.a) {
            shooter1.setPower(-0.9);

            shooter2.setPower(-0.9);
        }


        if (gamepad2.right_bumper) {
            shooter1.setPower(0);
        }

        if (gamepad2.left_bumper) {
            shooter2.setPower(0);
        }

        if (gamepad1.b) {
            shooter1.setPower(0.9);
        }

        if (gamepad2.y) {
            shooter2.setPower(0.9);
        }
    }
}

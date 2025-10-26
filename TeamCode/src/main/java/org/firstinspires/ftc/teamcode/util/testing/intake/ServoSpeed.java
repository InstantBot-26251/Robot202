package org.firstinspires.ftc.teamcode.util.testing.intake;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Servo Speed Tester")
public class ServoSpeed extends OpMode {
    CRServo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(CRServo.class, "intake");

    }

    @Override
    public void loop() {
        if (gamepad2.right_bumper) {
            servo.setPower(1);
        }
    }
}

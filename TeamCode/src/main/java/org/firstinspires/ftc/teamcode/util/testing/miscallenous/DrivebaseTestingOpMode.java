package org.firstinspires.ftc.teamcode.util.testing.miscallenous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.chassis.Chassis;

@TeleOp(name = "Drive Testing")
public class DrivebaseTestingOpMode extends OpMode {
    Chassis chassis;
    double x;
    double y;
    double rx;

    @Override
    public void init() {
        chassis = new Chassis(hardwareMap);
    }

    @Override
    public void loop() {

        y = -applyResponseCurve(gamepad1.left_stick_y);
        x = applyResponseCurve(gamepad1.left_stick_x);
        rx = applyResponseCurve(gamepad1.right_stick_x);

        if (gamepad1.a) {
            chassis.fl.setPower(1);
            chassis.fr.setPower(1);
            chassis.bl.setPower(1);
            chassis.br.setPower(1);
        }


        // TODO: FL AND BL ARE REVERSED
        if (gamepad1.dpad_up) {
            chassis.fl.setPower(0.5);  // Test front left
        } else if (gamepad1.dpad_right) {
            chassis.fr.setPower(0.5);  // Test front right
        } else if (gamepad1.dpad_down) {
            chassis.bl.setPower(0.5);  // Test back left
        } else if (gamepad1.dpad_left) {
            chassis.br.setPower(0.5);  // Test back right
        }

        telemetry.addData("Left stick y", gamepad1.left_stick_y);
        telemetry.addData("Left stick x", gamepad1.left_stick_x);
        telemetry.addData("Right stick x", gamepad1.right_stick_x);
        telemetry.addData("Adjusted x", x);
        telemetry.addData("Adjusted y", y);
        telemetry.addData("Adjusted rx", rx);

//        if(gamepad1.y){
//             chassis.resetYaw();
//        }

        chassis.drive(x, y, rx);
        telemetry.update();
    }

    public double applyResponseCurve(double input) {
        double exponent = 2;
        return Math.signum(input) * Math.pow(Math.abs(input), exponent);
    }

}

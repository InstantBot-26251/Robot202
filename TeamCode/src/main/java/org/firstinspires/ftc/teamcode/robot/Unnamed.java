package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.SubsystemTemplate;
import org.firstinspires.ftc.teamcode.chassis.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.chassis.Drivetrain;


import java.util.ArrayList;
import java.util.List;

public class Unnamed extends Robot {
    private Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

    private final List<SubsystemTemplate> subsystems = new ArrayList<>();

    private static final Unnamed INSTANCE = new Unnamed();

    public static Unnamed getInstance() {
        return INSTANCE;
    }

    private GamepadEx avy;
    private GamepadEx ishu;

    private static final double D_RESPONSE_CURVE = 1.5;
    private static final double ROTATIONAL_SENSITIVITY = 1.5;
    private static final double TRIGGER_DEADZONE = 0.1;

    private final ElapsedTime timer = new ElapsedTime();

    private Unnamed() {
        reset();
        robotInit();
        Log.i("Fieri", "===============ROBOT CREATED===============");
    }

    @Override
    public void reset() {
//        RobotStatus.robotState = RobotStatus.RobotState.DISABLED;
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearButtons();
        Log.i("Fieri", "===============COMMAND SCHEDULER CLEARED AURA HAS INCREASED===============");
    }

    private void registerSubsystems() {
        for (SubsystemTemplate s : subsystems) {
            register(s);
        }
    }

    private void robotInit() {
        subsystems.clear();
        subsystems.add(Drivetrain.getInstance().initialize());
        registerSubsystems();
    }

    public void disabledInit() {
//        RobotStatus.robotState = RobotStatus.RobotState.DISABLED;
        telemetry = FtcDashboard.getInstance().getTelemetry();
        Log.i("Fieri", "===============ROBOT DISABLED===============");
    }

    // Auto INIT
    public void autonomousInit(Telemetry telemetry, HardwareMap hardwareMap) {
        reset();
        registerSubsystems();
//        RobotStatus.robotState = RobotStatus.RobotState.AUTONOMOUS_INIT;

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        RobotMap.getInstance().init(hardwareMap);
//        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }

        subsystems.forEach(SubsystemTemplate::onAutonomousInit);
        Log.i("Fieri", "============INITIALIZED AUTONOMOUS GOOD BOY============");
    }

    // TeleOp INIT
    public void teleopInit(Telemetry telemetry, HardwareMap hardwareMap, Gamepad drive, Gamepad manip) {
        reset();
        registerSubsystems();
//        RobotStatus.robotState = RobotStatus.RobotState.TELEOP_INIT;

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        RobotMap.getInstance().init(hardwareMap);
//        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//            }
        avy = new GamepadEx(drive);
        ishu = new GamepadEx(manip);

        subsystems.forEach(SubsystemTemplate::onTeleopInit);

        // Driver Controls
        Drivetrain.getInstance().setDefaultCommand(new TeleOpDriveCommand(
                () -> applyResponseCurve(avy.getLeftY(), D_RESPONSE_CURVE),
                () -> applyResponseCurve(avy.getLeftX(), D_RESPONSE_CURVE),
                () -> applyResponseCurve(avy.getRightX(), ROTATIONAL_SENSITIVITY)
        ));
        new Trigger(() -> avy.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_DEADZONE)
                .whenActive(Drivetrain.getInstance()::enableSlowMode)
                .whenInactive(Drivetrain.getInstance()::disableSlowMode);

        // Reset Heading (IMPORTANTIAL)
        avy.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(Drivetrain.getInstance()::resetHeading);

        // Turn Field Centric ON/OFF (IMPORTANTIAL)
        avy.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(Drivetrain.getInstance()::toggleRobotCentric);



    //MANIPULATOR CONTROLS

    //Collect

    //Launch

    //Reject



  Log.i("Fieri", "============INITIALIZED TELEOP I HOPE WE WON OR GOT THREE RP============");
}

    // Response Curve Method
    private double applyResponseCurve(double input, double scale) {
        // Limit Input to 1 (MAX) and -1 (MIN)
        input = Math.max(-1, Math.min(1, input));

        // Apply Response Curve
        double output = Math.signum(input) * Math.pow(Math.abs(input), scale);

        return output;
    }

    // Get Telemetry
    public Telemetry getTelemetry() {
        return telemetry;
    }

//    public void periodic() {
//        if (!RobotStatus.isEnabled() && RobotStatus.isTeleop()) RobotStatus.robotState = RobotStatus.RobotState.TELEOP_ENABLED;
//        if (!RobotStatus.isEnabled() && !RobotStatus.isTeleop()) RobotStatus.robotState = RobotStatus.RobotState.AUTONOMOUS_ENABLED;
//
//        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
//            hub.clearBulkCache();
//        }
//
//        run();
//
//        telemetry.addLine();
//        telemetry.addData("Alliance", RobotStatus.alliance);
//        telemetry.addData("Loop Time", timer.milliseconds());
//        telemetry.addData("Status", RobotStatus.robotState);
//        telemetry.update();
//        timer.reset();
//    }
}

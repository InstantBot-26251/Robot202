package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.indexer.Indexer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooter.Hood;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.shooter.commands.ShooterCommands;
import org.firstinspires.ftc.teamcode.util.SubsystemTemplate;
import org.firstinspires.ftc.teamcode.chassis.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.chassis.Drivetrain;

import java.util.function.Supplier;


import java.util.ArrayList;
import java.util.List;

public class Enigma extends Robot {
    private Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

    private final List<SubsystemTemplate> subsystems = new ArrayList<>();
    private static final Enigma INSTANCE = new Enigma();

    public static Enigma getInstance() {
        return INSTANCE;
    }

    private GamepadEx avy;
    private GamepadEx ishu;

    private Gamepad drivePad;
    private Gamepad manipPad;

    private static final double D_RESPONSE_CURVE = 1.5;
    private static final double M_RESPONSE_CURVE = 1.5;
    private static final double ROTATIONAL_SENSITIVITY = 1.5;
    private static final double TRIGGER_DEADZONE = 0.1;

    private final ElapsedTime timer = new ElapsedTime();

    // Pedro follower (2.0.3)
//    private Follower follower;
// //     Optional field-centric flip (0 = Red/default, PI = Blue); toggle if you like
//    private double teleOpOffsetHeading = 0.0;

//    public static Pose startingPose; // you can set this from Auto to carry over into TeleOp
//    private TelemetryManager telemetryM;
////
//    // TeleOp helper state (mirrors ExampleTeleOp)
//    private boolean automatedDrive = false;
//    private boolean slowMode = false;
//    private double slowModeMultiplier = 0.5;
//    private Supplier<PathChain> pathChain;
//
//    // Simple edge-detection (so we can do “wasPressed” with plain Gamepad)
//    private boolean prevA, prevB, prevRB, prevX, prevY;

    private Enigma() {
        reset();
        robotInit();
        Log.i("Enigma", "===============ROBOT CREATED===============");
    }

    @Override
    public void reset() {
        RobotStatus.robotState = RobotStatus.RobotState.DISABLED;
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearButtons();
        Log.i("Enigma", "===============COMMAND SCHEDULER CLEARED AURA HAS INCREASED===============");
    }

    private void registerSubsystems() {
        for (SubsystemTemplate s : subsystems) {
            register(s);
        }
    }

    private void robotInit() {
        subsystems.clear();
        subsystems.add(Drivetrain.getInstance().initialize());
        subsystems.add(Indexer.getInstance().initialize());
        subsystems.add(Shooter.getInstance().initialize());
        subsystems.add(Hood.getInstance().initialize());
        registerSubsystems();
    }

    public void disabledInit() {
        RobotStatus.robotState = RobotStatus.RobotState.DISABLED;
        telemetry = FtcDashboard.getInstance().getTelemetry();
        Log.i("Enigma", "===============ROBOT DISABLED===============");
    }

    // Auto INIT
    public void autonomousInit(Telemetry opTel, HardwareMap hardwareMap) {
        reset();
        registerSubsystems();
        RobotStatus.robotState = RobotStatus.RobotState.AUTONOMOUS_INIT;

        this.telemetry = new MultipleTelemetry(opTel, FtcDashboard.getInstance().getTelemetry());
        RobotMap.getInstance().init(hardwareMap);
        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
//
//        // Pedro follower (official pattern)
//        follower = createFollower(hardwareMap);
//        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
//
//        // Panels Telemetry (optional but recommended in docs)
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


        subsystems.forEach(SubsystemTemplate::onAutonomousInit);
        Log.i("Enigma", "============INITIALIZED AUTONOMOUS GOOD BOY============");
    }

    // TeleOp INIT
    public void teleopInit(Telemetry opTel, HardwareMap hardwareMap, Gamepad drive, Gamepad manip) {
        reset();
        registerSubsystems();
        RobotStatus.robotState = RobotStatus.RobotState.TELEOP_INIT;

        this.telemetry = new MultipleTelemetry(opTel, FtcDashboard.getInstance().getTelemetry());
        RobotMap.getInstance().init(hardwareMap);
        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        avy = new GamepadEx(drive);
        ishu = new GamepadEx(manip);
        drivePad = drive;
        manipPad = manip;
//        resetEdgeDetectors();
//
//        follower = createFollower(hardwareMap);
//        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
//        follower.update(); // extra safety update on init (mirrors example)
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        follower = createFollower(hardwareMap);
//        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
//        follower.update(); // extra safety update on init (mirrors example)
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        follower.startTeleopDrive();


        subsystems.forEach(SubsystemTemplate::onTeleopInit);

//        // Driver Controls
//        Drivetrain.getInstance().setDefaultCommand(new TeleOpDriveCommand(
//                () -> applyResponseCurve(avy.getLeftY(), D_RESPONSE_CURVE),
//                () -> applyResponseCurve(avy.getLeftX(), D_RESPONSE_CURVE),
//                () -> applyResponseCurve(avy.getRightX(), ROTATIONAL_SENSITIVITY)
//        ));
//        new Trigger(() -> avy.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_DEADZONE)
//                .whenActive(Drivetrain.getInstance()::enableSlowMode)
//                .whenInactive(Drivetrain.getInstance()::disableSlowMode);
//
//        // Reset Heading (IMPORTANTIAL)
//        avy.getGamepadButton(GamepadKeys.Button.START)
//                .whenPressed(Drivetrain.getInstance()::resetHeading);
//
//        // Turn Field Centric ON/OFF (IMPORTANTIAL)
//        avy.getGamepadButton(GamepadKeys.Button.BACK)
//                .whenPressed(Drivetrain.getInstance()::toggleRobotCentric);


        //MANIPULATOR CONTROLS


        ishu.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(ShooterCommands.AUTO_AIM.get());

//    // Shoot
//    ishu.getGamepadButton(GamepadKeys.Button.X)
//            .whenPressed(ShooterCommands.SHOOT.get());

        // Testing
        ishu.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(ShooterCommands.SPIN_UP.get());

        // Stop Shooting
        ishu.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(ShooterCommands.STOP.get());

        // Reject
        ishu.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(ShooterCommands.REJECT.get());


        Log.i("Enigma", "============INITIALIZED TELEOP I HOPE WE WON OR GOT THREE RP============");
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

    public void periodic() {
        if (!RobotStatus.isEnabled() && RobotStatus.isTeleop())
            RobotStatus.robotState = RobotStatus.RobotState.TELEOP_ENABLED;
        if (!RobotStatus.isEnabled() && !RobotStatus.isTeleop())
            RobotStatus.robotState = RobotStatus.RobotState.AUTONOMOUS_ENABLED;

        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.clearBulkCache();
        }


//        if (follower != null) {
//            if (RobotStatus.isTeleop() && drivePad != null) {
//                // Manual drive unless automatedDrive is active
//                if (!automatedDrive) {
//                    double lx = -drivePad.left_stick_x;
//                    double ly = -drivePad.left_stick_y;
//                    double rx = -drivePad.right_stick_x;
//
//                    if (!slowMode) {
//                        follower.setTeleOpDrive(ly, lx, rx, true); // robot-centric
//                    } else {
//                        follower.setTeleOpDrive(
//                                ly * slowModeMultiplier,
//                                lx * slowModeMultiplier,
//                                rx * slowModeMultiplier,
//                                true
//                        );
//                    }
//
//                    // A pressed -> start automated path
//                    if (edgePressed(drivePad.a, prevA)) {
//                        follower.followPath(pathChain.get());
//                        automatedDrive = true;
//                    }
//                }
//
//                // If automated: B cancels OR auto completes
//                if (automatedDrive && (edgePressed(drivePad.b, prevB) || !follower.isBusy())) {
//                    follower.startTeleopDrive();
//                    automatedDrive = false;
//                }
//
//                // Slow mode toggle on Right Bumper
//                if (edgePressed(drivePad.right_bumper, prevRB)) {
//                    slowMode = !slowMode;
//                }
//
//                // Optional: adjust slow strength (+/-)
//                if (edgePressed(drivePad.x, prevX)) slowModeMultiplier += 0.25;
//                if (manipPad != null && edgePressed(manipPad.y, prevY)) slowModeMultiplier -= 0.25;
//
//                // update edge states
//                captureEdges();

                run();

                telemetry.addLine();
                telemetry.addData("Alliance", RobotStatus.alliance);
                telemetry.addData("Servo Position: ", Hood.getInstance().hoodServo.getPosition());
                telemetry.addData("Left Y: ", ishu.getLeftY());
                telemetry.addData("Loop Time", timer.milliseconds());
                telemetry.addData("Status", RobotStatus.robotState);
                telemetry.update();
                timer.reset();
            }

        }

//    private void resetEdgeDetectors() {
//        prevA = prevB = prevRB = prevX = prevY = false;
//    }
//    private boolean edgePressed(boolean now, boolean prev) { return now && !prev; }
//    private void captureEdges() {
//        prevA  = drivePad.a;
//        prevB  = drivePad.b;
//        prevRB = drivePad.right_bumper;
//        prevX  = drivePad.x;
//        if (manipPad != null) prevY = manipPad.y;
//    }

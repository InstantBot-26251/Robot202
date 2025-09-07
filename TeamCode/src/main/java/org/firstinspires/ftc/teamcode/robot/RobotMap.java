package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public class RobotMap {
    public HardwareMap hardwareMap;
    private final List<HardwareDevice> devices = new ArrayList<>();


    public SparkFunOTOS OTOS;

    // Drive motors
    public DcMotorEx MOTOR_FL;
    public DcMotorEx MOTOR_FR;
    public DcMotorEx MOTOR_BL;
    public DcMotorEx MOTOR_BR;

    private static RobotMap instance = null;

    // Returns an instance of this
    public static RobotMap getInstance() {
        if (instance == null) {
            instance = new RobotMap();
        }
        return instance;
    }

    public void init(final HardwareMap hardwareMap) {
        devices.clear();

        this.hardwareMap = hardwareMap;

        OTOS = hardwareMap.get(SparkFunOTOS.class, "otos");

        MOTOR_FL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        MOTOR_FR = hardwareMap.get(DcMotorEx.class, "frontRight");
        MOTOR_BL = hardwareMap.get(DcMotorEx.class, "backLeft");
        MOTOR_BR = hardwareMap.get(DcMotorEx.class, "backRight");

        addDevices();
    }

    private void addDevices() {
        devices.add(getInstance().OTOS);
        devices.add(getInstance().MOTOR_FL);
        devices.add(getInstance().MOTOR_FR);
        devices.add(getInstance().MOTOR_BL);
        devices.add(getInstance().MOTOR_BR);
    }

    // Getter Methods
    public List<HardwareDevice> getDevices() {
        return devices;
    }

    public List<LynxModule> getLynxModules() {
        return hardwareMap.getAll(LynxModule.class);
    }

    public HardwareMap getHardwareMap() {
        return this.hardwareMap;
    }
}

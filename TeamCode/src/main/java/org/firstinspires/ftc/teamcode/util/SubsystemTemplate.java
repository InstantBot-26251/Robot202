package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.SubsystemBase;

public abstract class SubsystemTemplate extends SubsystemBase {
    public SubsystemTemplate initialize() { return this; }
    public abstract void onAutonomousInit();
    public abstract void onTeleopInit();
}
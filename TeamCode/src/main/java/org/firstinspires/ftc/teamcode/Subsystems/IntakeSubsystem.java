package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import java.util.Hashtable;

public class IntakeSubsystem extends SubsystemBase {
    private MotorEx intakeMotor;
    private Hashtable<Modes, Double> modes;

    public IntakeSubsystem(MotorEx intakeMotor) {
        this.intakeMotor = intakeMotor;
        this.modes = new Hashtable<>();
        this.modes.put(Modes.PASSIVE, 0.5);
        this.modes.put(Modes.INTAKE, 1.0);
        this.modes.put(Modes.OUTTAKE, -1.0);
        this.modes.put(Modes.OFF, 0.0);
    }

    public enum Modes {
        PASSIVE,
        INTAKE,
        OUTTAKE,
        OFF
    }

    public void operate(Modes intakeMode) {
        this.intakeMotor.set(this.modes.get(intakeMode));
    }

    public void stop() {
        this.intakeMotor.stopMotor();
    }
}

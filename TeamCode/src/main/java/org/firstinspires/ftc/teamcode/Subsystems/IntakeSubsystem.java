package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import java.util.Hashtable;

public class IntakeSubsystem extends SubsystemBase {
    private MotorEx intakeMotor;
    private Modes intakeMode;

    public IntakeSubsystem(MotorEx intakeMotor) {
        this.intakeMotor = intakeMotor;
        this.intakeMode = Modes.OFF;
    }

    public enum Modes {
        PASSIVE(0.5),
        INTAKE(1.0),
        OUTTAKE(-1.0),
        OFF(0.0);

        private final double power;

        Modes(double power) {
            this.power = power;
        }

        public double getPower() {
            return this.power;
        }
    }

    public void operate(Modes intakeMode) {
        this.intakeMode = intakeMode;
        this.intakeMotor.set(this.intakeMode.getPower());
    }

    public void stop() {
        this.intakeMotor.stopMotor();
    }
}

package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private static final IntakeSubsystem instance = new IntakeSubsystem();

    private MotorEx intakeMotor;
    private Modes intakeMode;

    private Telemetry telemetry;

    private IntakeSubsystem() {}

    public static synchronized IntakeSubsystem getInstance() {
        return instance;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        this.intakeMotor = new MotorEx(hardwareMap, "Intake");
        this.intakeMotor.setRunMode(Motor.RunMode.RawPower);
        this.intakeMode = Modes.OFF;
    }

    public enum Modes {
        PASSIVE(0.1),
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

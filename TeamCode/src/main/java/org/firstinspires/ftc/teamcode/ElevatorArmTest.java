package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ElevatorArmSubsystem;

@TeleOp(name = "Elevator Arm Test")
public class ElevatorArmTest extends LinearOpMode {
    private MotorEx leftElevatorMotor;
    private MotorEx rightElevatorMotor;
    private MotorEx armMotor;

    private GamepadEx controller;

    private ElevatorArmSubsystem elevatorArm;

    @Override
    public void runOpMode() {
        leftElevatorMotor = new MotorEx(hardwareMap, "leftElevator");
        rightElevatorMotor = new MotorEx(hardwareMap, "rightElevator");
        armMotor = new MotorEx(hardwareMap, "arm");

        controller = new GamepadEx(gamepad1);

        elevatorArm = new ElevatorArmSubsystem(
                hardwareMap,
                telemetry
        );

        waitForStart();
        while (opModeIsActive()) {
            if (controller.getButton(GamepadKeys.Button.A)) {
                elevatorArm.setTargetLevel(ElevatorArmSubsystem.Levels.GROUNDED);
            } else if (controller.getButton(GamepadKeys.Button.B)) {
                elevatorArm.setTargetLevel(ElevatorArmSubsystem.Levels.L1);
            } else if (controller.getButton(GamepadKeys.Button.X)) {
                elevatorArm.setTargetLevel(ElevatorArmSubsystem.Levels.L2);
            } else if (controller.getButton(GamepadKeys.Button.Y)) {
                elevatorArm.setTargetLevel(ElevatorArmSubsystem.Levels.L3);
            }

            elevatorArm.periodic();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

@TeleOp(name="Field Centric Drive")
public class FieldCentricTest extends LinearOpMode {
    private MotorEx frontLeftMotor;
    private MotorEx frontRightMotor;
    private MotorEx backLeftMotor;
    private MotorEx backRightMotor;

    private GyroEx gyro;

    private GamepadEx driverJoystick;
    private GamepadEx operatorJoystick;

    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        frontLeftMotor = new MotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_312);
        frontRightMotor = new MotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_312);
        backLeftMotor = new MotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_312);
        backRightMotor = new MotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_312);

        gyro = new RevIMU(hardwareMap, "imu");

        driverJoystick = new GamepadEx(gamepad1);
        operatorJoystick = new GamepadEx(gamepad2);

        frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setInverted(true);
        frontRightMotor.setInverted(true);
        backLeftMotor.setInverted(true);
        backRightMotor.setInverted(true);

        drive = new MecanumDrive(frontLeftMotor,frontRightMotor,backLeftMotor,backRightMotor);

        waitForStart();

        while (opModeIsActive()) {
            if (driverJoystick.getButton(GamepadKeys.Button.START) && driverJoystick.getButton(GamepadKeys.Button.BACK)){
                gyro.reset();
            }

            drive.driveFieldCentric(
                    driverJoystick.getLeftX(),
                    driverJoystick.getLeftY(),
                    driverJoystick.getRightX(),
                    gyro.getHeading() + 180
            );

            telemetry.addData("Driver LeftJoyX", driverJoystick.getLeftX());
            telemetry.addData("Driver LeftJoyY", driverJoystick.getLeftY());
            telemetry.addData("Driver RightJoyX", driverJoystick.getRightX());
            telemetry.addData("Absolute Heading", gyro.getAbsoluteHeading());
            telemetry.addData("Heading", gyro.getHeading());

            telemetry.update();
        }
    }
}

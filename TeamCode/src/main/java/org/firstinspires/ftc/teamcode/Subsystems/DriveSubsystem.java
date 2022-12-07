package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import lib.autoNavigation.controller.HolonomicDriveController;
import lib.autoNavigation.trajectory.OmniTrajectory;

public class DriveSubsystem extends SubsystemBase {
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime pathTime = new ElapsedTime();

    // Create the variables to hold the four motor objects
    private final MotorEx frontLeftMotor;
    private final MotorEx frontRightMotor;
    private final MotorEx backLeftMotor;
    private final MotorEx backRightMotor;

    private final GyroEx gyro;

    private final double WHEEL_DIAMETER = 101.6; // Wheel diameter in millimeters

    private final double COUNTS_TO_MM = (WHEEL_DIAMETER * Math.PI) / Motor.GoBILDA.RPM_312.getCPR();
    private final double METERS_TO_COUNTS = (1 / COUNTS_TO_MM) * 1000; // Convert meters per second to ticks per second
    private final double MAX_SPEED_M_PER_S = Motor.GoBILDA.RPM_312.getAchievableMaxTicksPerSecond() * COUNTS_TO_MM;

    private final double MAX_ACCEL_M_PER_S_SQ = 1.0; //TODO: Set this number to be accurate;

    private final MecanumDriveKinematics kinematics;
    private Rotation2d gyroAngle;
    private final MecanumDriveOdometry odometry;

    private final Telemetry telemetry;

    private final HolonomicDriveController driveController;

    private OmniTrajectory trajectory;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.frontLeftMotor = new MotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_312);
        this.frontRightMotor = new MotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_312);
        this.backLeftMotor = new MotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_312);
        this.backRightMotor = new MotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_312);
        this.gyro = new RevIMU(hardwareMap);

        this.telemetry = telemetry;

        //TODO: Set and tune PID coefficients to be good
        PIDController xController = new PIDController(3.0, 0.0, 0.0);
        PIDController yController = new PIDController(3.0, 0.0, 0.0);
        ProfiledPIDController thetaController = new ProfiledPIDController(3.0, 0.0, 0.0,
                new TrapezoidProfile.Constraints(
                        MAX_SPEED_M_PER_S,
                        MAX_ACCEL_M_PER_S_SQ));

        this.driveController = new HolonomicDriveController(
                xController,
                yController,
                thetaController
        );

        // Set motors to run at a specified velocity
        frontLeftMotor.setRunMode(Motor.RunMode.VelocityControl);
        frontRightMotor.setRunMode(Motor.RunMode.VelocityControl);
        backLeftMotor.setRunMode(Motor.RunMode.VelocityControl);
        backRightMotor.setRunMode(Motor.RunMode.VelocityControl);

        // Invert the direction of the left motors
        frontLeftMotor.setInverted(true);
        backLeftMotor.setInverted(true);

        // Set the motors to brake on stop
        frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Reset the motor encoders
        frontLeftMotor.resetEncoder();
        frontRightMotor.resetEncoder();
        backLeftMotor.resetEncoder();
        backRightMotor.resetEncoder();

        // Set how far the wheels move in millimeters per encoder tick
        frontLeftMotor.setDistancePerPulse(COUNTS_TO_MM);
        frontRightMotor.setDistancePerPulse(COUNTS_TO_MM);
        backLeftMotor.setDistancePerPulse(COUNTS_TO_MM);
        backRightMotor.setDistancePerPulse(COUNTS_TO_MM);

        gyro.init();

        // Setup the odometry system
        // Set the locations of the wheels on the robot
        // TODO: Update wheel locations to match this year's chassis
        Translation2d frontLeftLocation = new Translation2d(0.168,0.207);
        Translation2d frontRightLocation = new Translation2d(0.168,-0.207);
        Translation2d backLeftLocation = new Translation2d(-0.168,0.207);
        Translation2d backRightLocation = new Translation2d(-0.168,-0.207);

        // Create a mecanum kinematics object from the wheel locations
        kinematics = new MecanumDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
        // Create an object to hold the angle of the robot as reported by the IMU
        gyroAngle = gyro.getRotation2d();
        // Create an odometry object - format is (kinematics object, gyro heading, start position on field)
        odometry = new MecanumDriveOdometry(kinematics, gyroAngle, new Pose2d(0, 0, new Rotation2d(0)));
    }

    public void periodic() {
        updateGyro();
        updateOdometry();

        telemetry.addData("Current Pos", odometry.getPoseMeters());
        telemetry.addData("Current Heading", gyroAngle);
    }

    public void driveTeleOp(double forward, double right, double rotation, boolean fieldCentric) {
        ChassisSpeeds chassisSpeeds;

        if (fieldCentric){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, right, rotation, gyroAngle);
        } else {
            chassisSpeeds = new ChassisSpeeds(forward, right, rotation);
        }

        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        wheelSpeeds.normalize(MAX_SPEED_M_PER_S);
        setMotors(wheelSpeeds);
    }

    public void driveAuto() {
        if (trajectory != null) {
            double curTime = pathTime.time();
            OmniTrajectory.State desiredState = trajectory.sample(curTime);

            ChassisSpeeds targetChassisSpeeds = driveController.calculate(
                    getOdometryLocation(),
                    desiredState,
                    desiredState.poseMeters.getRotation()
            );

            MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(targetChassisSpeeds);
            wheelSpeeds.normalize(MAX_SPEED_M_PER_S);

            setMotors(wheelSpeeds);
        } else {
            setMotors(new MecanumDriveWheelSpeeds());
        }
    }

    public Pose2d getOdometryLocation() {
        return odometry.getPoseMeters();
    }

    private void updateOdometry() {
        odometry.updateWithTime(runtime.time(),
                gyroAngle,
                new MecanumDriveWheelSpeeds(
                        frontLeftMotor.getVelocity() / METERS_TO_COUNTS,
                        frontRightMotor.getVelocity() / METERS_TO_COUNTS,
                        backLeftMotor.getVelocity() / METERS_TO_COUNTS,
                        backRightMotor.getVelocity() / METERS_TO_COUNTS));
    }

    private void updateGyro() {
        gyroAngle = gyro.getRotation2d();
    }

    private void setMotors(@NonNull MecanumDriveWheelSpeeds wheelSpeeds) {
        telemetry.addData("commanded speeds", wheelSpeeds);
        frontLeftMotor.setVelocity(wheelSpeeds.frontLeftMetersPerSecond * METERS_TO_COUNTS);
        frontRightMotor.setVelocity(wheelSpeeds.frontRightMetersPerSecond * METERS_TO_COUNTS);
        backLeftMotor.setVelocity(wheelSpeeds.rearLeftMetersPerSecond * METERS_TO_COUNTS);
        backRightMotor.setVelocity(wheelSpeeds.rearRightMetersPerSecond * METERS_TO_COUNTS);
    }

    /** Stop all motors from running. */
    public void stop() {
        frontLeftMotor.stopMotor();
        frontRightMotor.stopMotor();
        backLeftMotor.stopMotor();
        backRightMotor.stopMotor();
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void resetOdometry(Pose2d robotPose) {
        odometry.resetPosition(robotPose, gyroAngle);
    }

    public OmniTrajectory getTrajectory() {
        return trajectory;
    }

    public void setTrajectory(OmniTrajectory trajectory) {
        this.trajectory = trajectory;
        this.pathTime.reset();
    }
}

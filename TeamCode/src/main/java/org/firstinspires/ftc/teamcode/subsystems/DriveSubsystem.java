package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.*;

public class DriveSubsystem extends SubsystemBase {
    private static final DriveSubsystem instance = new DriveSubsystem();

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime pathTime = new ElapsedTime();

    // Create the variables to hold the four motor objects
    private MotorEx frontLeftMotor;
    private MotorEx frontRightMotor;
    private MotorEx backLeftMotor;
    private MotorEx backRightMotor;

    private GyroEx gyro;

    private MecanumDriveKinematics kinematics;
    private Rotation2d gyroAngle;
    private MecanumDriveOdometry odometry;

    private Telemetry telemetry;

    private DriveSubsystem() {}

    public static synchronized DriveSubsystem getInstance() {
        return instance;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.frontLeftMotor = new MotorEx(hardwareMap, FRONT_LEFT, Motor.GoBILDA.RPM_312);
        this.frontRightMotor = new MotorEx(hardwareMap, FRONT_RIGHT, Motor.GoBILDA.RPM_312);
        this.backLeftMotor = new MotorEx(hardwareMap, BACK_LEFT, Motor.GoBILDA.RPM_312);
        this.backRightMotor = new MotorEx(hardwareMap, BACK_RIGHT, Motor.GoBILDA.RPM_312);
        this.gyro = new RevIMU(hardwareMap);

        this.telemetry = telemetry;

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

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        wheelSpeeds.normalize(MAX_SPEED_M_PER_S);
        setMotors(wheelSpeeds);
    }

    public Pose2d getOdometryLocation() {
        return odometry.getPoseMeters();
    }

    private void updateOdometry() {
        odometry.updateWithTime(runtime.time(),
                gyroAngle,
                new MecanumDriveWheelSpeeds(
                        frontLeftMotor.getVelocity() / METERS_TO_TICKS,
                        frontRightMotor.getVelocity() / METERS_TO_TICKS,
                        backLeftMotor.getVelocity() / METERS_TO_TICKS,
                        backRightMotor.getVelocity() / METERS_TO_TICKS));
    }

    private void updateGyro() {
        gyroAngle = gyro.getRotation2d();
    }

    private void setMotors(@NonNull MecanumDriveWheelSpeeds wheelSpeeds) {
        //telemetry.addData("commanded speeds", wheelSpeeds);
        frontLeftMotor.setVelocity(wheelSpeeds.frontLeftMetersPerSecond * METERS_TO_TICKS);
        frontRightMotor.setVelocity(wheelSpeeds.frontRightMetersPerSecond * METERS_TO_TICKS);
        backLeftMotor.setVelocity(wheelSpeeds.rearLeftMetersPerSecond * METERS_TO_TICKS);
        backRightMotor.setVelocity(wheelSpeeds.rearRightMetersPerSecond * METERS_TO_TICKS);
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

    public void disableMotors() {
        frontLeftMotor.disable();
        frontRightMotor.disable();
        backLeftMotor.disable();
        backRightMotor.disable();
    }
}

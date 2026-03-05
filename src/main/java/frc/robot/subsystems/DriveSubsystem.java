package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// For CAN
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.*;

import com.ctre.phoenix6.hardware.Pigeon2; //Gyro

public class DriveSubsystem extends SubsystemBase {

    private final Pigeon2 pigeon = new Pigeon2(17);

    private SparkMax leftMaster = new SparkMax(
            Constants.Subsystems.Drive.kLEFT_MASTER, MotorType.kBrushless);
    private SparkMax leftFollower = new SparkMax(
            Constants.Subsystems.Drive.kLEFT_FOLLOWER, MotorType.kBrushless);
    private SparkMax rightMaster = new SparkMax(
            Constants.Subsystems.Drive.kRIGHT_MASTER, MotorType.kBrushless);
    private SparkMax rightFollower = new SparkMax(
            Constants.Subsystems.Drive.kRIGHT_FOLLOWER, MotorType.kBrushless);
    // Encoders
    private final RelativeEncoder rightEncoder = rightMaster.getEncoder();
    private final RelativeEncoder leftEncoder = leftMaster.getEncoder();

    // Tuning constant for correction
    private static final double kP = 0.05; // Proportional gain
    double wheelCircumference = Math.PI * 0.2032; // 8 inch diameter in meters

    private final MotorControllerGroup leftGroup = new MotorControllerGroup(leftMaster, leftFollower);
    private final MotorControllerGroup rightGroup = new MotorControllerGroup(rightMaster, rightFollower);

    private final DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

    private Pose2d currentPose = new Pose2d();

    public DriveSubsystem() {

        pigeon.setYaw(0.0);

        drive.setSafetyEnabled(true);

        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    this::getPose,
                    this::resetPose,
                    this::getRobotRelativeSpeeds,
                    (speeds, feedforwards) -> driveRobotRelative(speeds),
                    new PPLTVController(0.02),
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this);
        } catch (Exception e) {
            e.printStackTrace();
            // RobotConfig config = new RobotConfig(55, 4.6, null, 0.66);

        }
    }

    public void arcadeDrive(double fwd, double rot) {
        // if (Math.abs(fwd) > 0.05 && Math.abs(rot) < 0.05) {
        // // double correction = leftEncoder.getVelocity() - rightEncoder.getVelocity()
        // * 0.00001;
        // // drive.arcadeDrive(fwd, -correction);
        // // return;
        // } else {
        // Normal arcade drive
        drive.arcadeDrive(rot, fwd);
        // }
    }

    public void tankDrive(double left, double right) {
        drive.tankDrive(left, right);
    }

    public void stop() {
        drive.stopMotor();
    }

    public Pose2d getPose() {
        return currentPose;
    }

    public void resetPose(Pose2d pose) {
        currentPose = pose;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {

        Double yaw = pigeon.getYaw().getValueAsDouble();
        double yawRateRadPerSec = Math.toRadians(yaw);
        double averageSpeed = ((leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2.0)
                * (wheelCircumference / 8.45 / 60.0);
        DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(),
                rightEncoder.getVelocity());
        return new ChassisSpeeds(averageSpeed, 0, yawRateRadPerSec);
        // Vy is 0 because we are only doing differential drive, so we can't move
        // sideways. Vx is the speed of our drive train, omega is the rate of rotation
        // from the gyro.
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        drive.arcadeDrive(
                speeds.vxMetersPerSecond,
                speeds.omegaRadiansPerSecond);
    }

    public Command resetPigeon() {
        return this.run(() -> {

            pigeon.setYaw(0.0);
            System.out.println("Pigeon initialized and yaw reset.");
        });

    }

    @Override
    public void periodic() {
        // Put periodic subsystem code here (telemetry, safety checks)

        // Display motor temperatures on SmartDashboard
        // SmartDashboard.putNumber("Left Front Temp C",
        // leftMaster.getMotorTemperature());
        // SmartDashboard.putNumber("Right Front Temp C",
        // rightMaster.getMotorTemperature());
        // SmartDashboard.putNumber("Left Back Temp C",
        // leftFollower.getMotorTemperature());
        // SmartDashboard.putNumber("Right Back Temp C",
        // rightFollower.getMotorTemperature());
        // SmartDashboard.putNumber("Left Side Velocity", leftEncoder.getVelocity());
        // SmartDashboard.putNumber("Right Side Velocity", rightEncoder.getVelocity());

        boolean isHot = leftMaster.getMotorTemperature() > 50 || rightMaster.getMotorTemperature() > 50
                || leftFollower.getMotorTemperature() > 50 || rightFollower.getMotorTemperature() > 50;
        SmartDashboard.putBoolean("Drive Overheating", isHot);

        SmartDashboard.putNumber("Pigeon Yaw", pigeon.getYaw().getValueAsDouble());

    }
}

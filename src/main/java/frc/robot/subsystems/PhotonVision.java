// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.MathUtil;

import java.util.*;

// PhotonVision imports
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision extends SubsystemBase {
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;
  private final ClimberSubsystem m_climberSubsystem;

  private PhotonCamera camera;

  PIDController turnPID = new PIDController(0.05, 0, 0);
  PIDController drivePID = new PIDController(0.5, 0, 0);

  /**
   * Construct PhotonVision with shared subsystem references.
   * 
   * @param drive   shared DriveSubsystem
   * @param shooter shared ShooterSubsystem
   */
  public PhotonVision(DriveSubsystem drive, ShooterSubsystem shooter, ClimberSubsystem climber) {
    this.m_driveSubsystem = drive;
    this.m_ShooterSubsystem = shooter;
    m_climberSubsystem = climber;

    camera = new PhotonCamera("MainCamera");

    turnPID.setTolerance(Math.toRadians(2)); // degrees
    drivePID.setTolerance(0.1); // meters
  }

  /**
   * Example command factory method.
   *
   * @return a command
   * 
   *         public Command exampleMethodCommand() {
   *         // Inline construction of command goes here.
   *         // Subsystem::RunOnce implicitly requires `this` subsystem.
   *         return runOnce(
   *         () -> {
   *         one-time action goes here
   *         });
   *         }
   * 
   *         /**
   *         An example method querying a boolean state of the subsystem (for
   *         example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   * 
   *         public boolean exampleCondition() {
   *         // Query some boolean state, such as a digital sensor.
   *         return false;
   *         }
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public Command AimClimb() {
    return new RunCommand(() -> {
      var result = camera.getLatestResult();

      Pose3d robotPose = new Pose3d();
      var allianceOptional = Optional.empty();
      double distanceToTarget = 0.0;
      Rotation2d targetYaw = new Rotation2d();
      boolean varsNotSet = true;
      DriverStation.Alliance alliance = null;

      if (result.hasTargets()) {
        if (varsNotSet) {
          varsNotSet = false;

          if (alliance == DriverStation.Alliance.Red) {
            // Distance
            distanceToTarget = PhotonUtils.getDistanceToPose(robotPose.toPose2d(),
                Constants.Subsystems.Vision.redClimbPos);
            System.out.println("Distance to Target: " + distanceToTarget);
            // Rotation
            targetYaw = PhotonUtils.getYawToPose(robotPose.toPose2d(), Constants.Subsystems.Vision.redClimbPos);
          } else if (alliance == DriverStation.Alliance.Blue){
                        // Distance
            distanceToTarget = PhotonUtils.getDistanceToPose(robotPose.toPose2d(),
                Constants.Subsystems.Vision.blueClimbPos);
            System.out.println("Distance to Target: " + distanceToTarget);
            // Rotation
            targetYaw = PhotonUtils.getYawToPose(robotPose.toPose2d(), Constants.Subsystems.Vision.blueClimbPos);
          } else {
            System.out.println("Error loading Allance color");
          }

          System.out.println("Distance: " + distanceToTarget + " Target Yaw: " + targetYaw.getDegrees());
        }

          double rotaioionSpeed = turnPID.calculate(targetYaw.getRadians(), Constants.Subsystems.Vision.kYawTarget);
          double driveSpeed = drivePID.calculate(distanceToTarget, Constants.Subsystems.Vision.kDistanceTarget);

          // Clamp to safty range
          rotaioionSpeed = MathUtil.clamp(rotaioionSpeed, -Constants.Subsystems.Drive.kMaxNormalSpeed,
              Constants.Subsystems.Drive.kMaxNormalSpeed);
          driveSpeed = MathUtil.clamp(driveSpeed, -Constants.Subsystems.Drive.kMaxRotSpeed,
              Constants.Subsystems.Drive.kMaxRotSpeed);

          m_driveSubsystem.arcadeDrive(driveSpeed, rotaioionSpeed);

          if (turnPID.atSetpoint() && drivePID.atSetpoint()) {
            m_driveSubsystem.arcadeDrive(0, 0);
            m_climberSubsystem.UpClimb();
          } else {
            m_climberSubsystem.StopClimb();
          }
      } else {
        m_climberSubsystem.StopClimb();
      }
    }, m_driveSubsystem, m_climberSubsystem);
  }

  public Command AimShoot() {
    return new RunCommand(() -> {
      var result = camera.getLatestResult();

      if (result.hasTargets()) {

        PhotonTrackedTarget target = result.getBestTarget();

        // Calculate robot feild relative pose
        if (Constants.Subsystems.Vision.kAprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
          Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
              Constants.Subsystems.Vision.kAprilTagFieldLayout.getTagPose(target.getFiducialId()).get(),
              Constants.Subsystems.Vision.kCameraToRobot);

          double distanceToTarget = PhotonUtils.getDistanceToPose(robotPose.toPose2d(),
                Constants.Subsystems.Vision.kHubPoseBlue);
          Rotation2d targetYaw = PhotonUtils.getYawToPose(robotPose.toPose2d(), Constants.Subsystems.Vision.kHubPoseBlue);



          
          var allianceOptional = DriverStation.getAlliance();
          DriverStation.Alliance alliance = allianceOptional.get();



          if (alliance == DriverStation.Alliance.Red) {
            // Distance
            distanceToTarget = PhotonUtils.getDistanceToPose(robotPose.toPose2d(),
                Constants.Subsystems.Vision.kHubPoseBlue);
            System.out.println("Distance to Target: " + distanceToTarget);
            // Rotation
            targetYaw = PhotonUtils.getYawToPose(robotPose.toPose2d(), Constants.Subsystems.Vision.kHubPoseRed);
          } else if (alliance == DriverStation.Alliance.Blue){
                        // Distance
            distanceToTarget = PhotonUtils.getDistanceToPose(robotPose.toPose2d(),
                Constants.Subsystems.Vision.kHubPoseBlue);
            System.out.println("Distance to Target: " + distanceToTarget);
            // Rotation
            targetYaw = PhotonUtils.getYawToPose(robotPose.toPose2d(), Constants.Subsystems.Vision.kHubPoseBlue);
          } else {
            System.out.println("Error loading Allance color");
          }




          System.out.println("Distance: " + distanceToTarget + " Target Yaw: " + targetYaw.getDegrees());

          double rotaioionSpeed = turnPID.calculate(targetYaw.getRadians(), Constants.Subsystems.Vision.kYawTarget);
          double driveSpeed = drivePID.calculate(distanceToTarget, Constants.Subsystems.Vision.kDistanceTarget);

          // Clamp to safty range
          rotaioionSpeed = MathUtil.clamp(rotaioionSpeed, -Constants.Subsystems.Drive.kMaxNormalSpeed,
              Constants.Subsystems.Drive.kMaxNormalSpeed);
          driveSpeed = MathUtil.clamp(driveSpeed, -Constants.Subsystems.Drive.kMaxRotSpeed,
              Constants.Subsystems.Drive.kMaxRotSpeed);

          m_driveSubsystem.arcadeDrive(driveSpeed, rotaioionSpeed);

          if (turnPID.atSetpoint() && drivePID.atSetpoint()) {
            m_driveSubsystem.arcadeDrive(0, 0);
            m_ShooterSubsystem.StartShoot();
          } else {
            m_ShooterSubsystem.StopShoot();
          }

          // if (targetYaw.getDegrees() > 2) {
          // m_driveSubsystem.arcadeDrive(-0.4, 0);
          // System.out.println("Turning -0.4");
          // } else if (targetYaw.getDegrees() < -2) {
          // m_driveSubsystem.arcadeDrive(0.4, 0);
          // System.out.println("Turning 0.4");
          // } else {

          // if (Math.abs(distanceToTarget) > 3) {
          // m_driveSubsystem.arcadeDrive(0, 0.4);
          // System.out.println("Driving Forward 0.4");
          // } else if (Math.abs(distanceToTarget) < 2.75) {
          // m_driveSubsystem.arcadeDrive(0, -0.4);
          // System.out.println("Driving Backward -0.4");
          // } else {
          // m_driveSubsystem.arcadeDrive(0, 0);
          // m_ShooterSubsystem.StartShoot();
          // }
          // }

        }
      } else {
        m_driveSubsystem.arcadeDrive(0, 0);
      }

    }, m_driveSubsystem, m_ShooterSubsystem);

  }
}

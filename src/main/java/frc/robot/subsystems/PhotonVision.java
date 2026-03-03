// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.math.MathUtil;

// PhotonVision imports
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
// removed duplicate Pose3d import

//Subsystem imports
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;



public class PhotonVision extends SubsystemBase {
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;
  
  private PhotonCamera camera;
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  
  PIDController turnPID = new PIDController(0.05, 0, 0);
  PIDController drivePID = new PIDController(0.5, 0, 0);

  
  
    public static final Pose2d HubPoseBlue = new Pose2d(
4.625594,
4.034536,
    new Rotation2d(0.0)
    );




    Transform3d cameraToRobot = new Transform3d(
            0.2,  // forward from robot center cm
            0.0,  // left/right camera is centered cm
            0.5,  // up from floor cm
            new edu.wpi.first.math.geometry.Rotation3d(0, 0, 0)
    );



  /**
   * Construct PhotonVision with shared subsystem references.
   * @param drive shared DriveSubsystem
   * @param shooter shared ShooterSubsystem
   */
  public PhotonVision(DriveSubsystem drive, ShooterSubsystem shooter) {
    this.m_driveSubsystem = drive;
    this.m_ShooterSubsystem = shooter;

    camera = new PhotonCamera("MainCamera");
    // Set tolerances (angles in radians)
    turnPID.setTolerance(Math.toRadians(1.5));
    drivePID.setTolerance(0.1);
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



  public Command AimShoot() {
    return new RunCommand(() -> 
    {
      var result = camera.getLatestResult();

      if (result.hasTargets()){
   
      PhotonTrackedTarget target = result.getBestTarget();

      // Calculate robot feild relative pose
      if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()){
         Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot);
         //Distance
         double distanceToTarget = PhotonUtils.getDistanceToPose(robotPose.toPose2d(), HubPoseBlue);
          System.out.println("Distance to Target: " + distanceToTarget);
         //Rotation
         Rotation2d targetYaw = PhotonUtils.getYawToPose(robotPose.toPose2d(), HubPoseBlue);
          System.out.println("Target Yaw (deg): " + targetYaw.getDegrees());
          System.out.println("Target Yaw (rad): " + targetYaw.getRadians());

          // Use radians for PID
          double rotationSpeed = turnPID.calculate(targetYaw.getRadians(), 0);
          double driveSpeed = drivePID.calculate(distanceToTarget, 1.0);

          // Clamp outputs to safe ranges
          double maxDrive = Constants.SpeedChange.maxNormalSpeed;
          double maxRot = 0.8; // adjust/tune as needed
          driveSpeed = MathUtil.clamp(driveSpeed, -maxDrive, maxDrive);
          rotationSpeed = MathUtil.clamp(rotationSpeed, -maxRot, maxRot);

          m_driveSubsystem.arcadeDrive(driveSpeed, rotationSpeed);

          if (turnPID.atSetpoint() && drivePID.atSetpoint()) {
            m_ShooterSubsystem.StartShoot();
          } else {
            m_ShooterSubsystem.StopShoot();
          }
      }} else {
        m_driveSubsystem.arcadeDrive(0, 0);
      }

  }, m_driveSubsystem, m_ShooterSubsystem);

}
}
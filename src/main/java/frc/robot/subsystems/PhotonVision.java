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
import edu.wpi.first.wpilibj.DriverStation;
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

  
      
    public static final Pose2d HubPoseBlue = new Pose2d(
4.625594, //4.625594
4.034536,
    new Rotation2d(0.0)
    );




    Transform3d cameraToRobot = new Transform3d(
            -0.1016,  // forward from robot center 
            0.0,  // left/right camera is centered 
            0.5588,  // up from floor 
            new edu.wpi.first.math.geometry.Rotation3d(0, 0, 180)
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

         System.out.println("Distance: " + distanceToTarget + " Target Yaw: " + targetYaw.getDegrees());

         if (targetYaw.getDegrees() > 2) {
                   m_driveSubsystem.arcadeDrive(0, -0.4);
        
        } else if (targetYaw.getDegrees() < -2) {
                   m_driveSubsystem.arcadeDrive(0, 0.4);
         } else {

            if (Math.abs(distanceToTarget) > 3) { 
                   m_driveSubsystem.arcadeDrive(0.4, 0);
        } else if (Math.abs(distanceToTarget) < 2.75) {  
                   m_driveSubsystem.arcadeDrive(-0.4, 0);

        } else {
                    m_driveSubsystem.arcadeDrive(0, 0);
                    m_ShooterSubsystem.StartShoot();
        }
      }


          




       
      }} else {
        m_driveSubsystem.arcadeDrive(0, 0);
      }

  }, m_driveSubsystem, m_ShooterSubsystem);

}
}
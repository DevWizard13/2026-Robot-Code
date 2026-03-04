// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Controls.Driver;
import frc.robot.Constants.Controls.Operator;
import frc.robot.Constants.SpeedChange;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVision;
import java.util.*;
import org.photonvision.PhotonCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.AgitatorSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Max speed variable for drive scaling
  double speed = SpeedChange.maxNormalSpeed;
  public boolean m_arcade = true;

  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final AgitatorSubsystem m_AgitatorSubsystem = new AgitatorSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final PhotonVision m_photonVision = new PhotonVision(new Pose2d(), m_driveSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(Driver.kJoystickID);
  private final CommandXboxController m_operatorController = new CommandXboxController(Operator.kJoystickID);
  private final SendableChooser<Command> autoChooser;

  // PhotonVision stuff, dw about it and DON'T mess with it without understanding what it does
  double totalRot = 0;
  double photonRot = 0;
  double totalFwd = 0;
  double photonFwd = 0;
  boolean isAimed = false;
  boolean remoteOperated = true;

  // Drive mode: false = tank, true = arcade (Default)

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();

    boolean remoteOperated = true;
    // please note that remoteOperated is **reserved for photonvision stuff**,
    // use a different variable for other stuff

    // Configure the trigger bindings
    registerNamedCommands();
    configureBindings();
    // drive command
       m_driveSubsystem.setDefaultCommand(
         new RunCommand(
          () -> {
            if (remoteOperated) {
              if (m_arcade) {
                double rot = applyDeadbandAndScale(m_driverController.getRightX());
                double fwd = applyDeadbandAndScale(m_driverController.getLeftY());
               
                m_driveSubsystem.arcadeDrive(rot, fwd);
              } else {
                double left = applyDeadbandAndScale(-m_driverController.getLeftY());
                double right = applyDeadbandAndScale(m_driverController.getRightY());
                m_driveSubsystem.tankDrive(left, right);
              }
            } else {
              photonVisionOperation();
            }
          },
          m_driveSubsystem));

    // Toggle drive mode -- false = tank, true = arcade
  //  m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_arcade = !m_arcade));
  SmartDashboard.putData("Toggle Drive Mode", new InstantCommand(() -> m_arcade = !m_arcade));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Speed boost
    m_driverController.leftTrigger()
        // Speed Boost ON
        .onTrue(new InstantCommand(() -> speed = SpeedChange.maxBoostSpeed))
        // Speed Boost OFF
        .onFalse(new InstantCommand(() -> speed = SpeedChange.maxNormalSpeed));
  }

  private double applyDeadbandAndScale(double value) {
    if (Math.abs(value) < SpeedChange.stickDeadband) {
      return 0.0;
    }

    return Math.copySign(Math.abs(value - SpeedChange.stickDeadband) / (1.0 - SpeedChange.stickDeadband) * speed,
        value);
  }


  void photonVisionOperation() {
    totalRot = m_photonVision.getRotToTarget();
    totalFwd = m_photonVision.getDriveToTarget();
    
    if (totalRot > 5) {
      photonRot = -1;
    } else if (totalRot < -5) {
      photonRot = 1;
    }
    else if (totalRot > 1) {
      photonRot = -0.5;
    }
    else if (totalRot < -1) {
      photonRot = 0.5;
    }
    else {
      m_ShooterSubsystem.StartShoot();
      photonRot = 0.0;
    }
    
    if (totalFwd > 5) {
      photonFwd = 1;
    }
  
    m_driveSubsystem.arcadeDrive(photonRot, photonFwd);
  }

    //Register named commands for use in PathPlanner autonomous paths
  private void registerNamedCommands() {
    NamedCommands.registerCommand("Shoot_1", m_ShooterSubsystem.shoot_1_Command());
    NamedCommands.registerCommand("Shooter_ON", m_ShooterSubsystem.StartShoot());
    NamedCommands.registerCommand("Shooter_OFF", m_ShooterSubsystem.StopShoot());
    NamedCommands.registerCommand("Climb_UP", m_ClimberSubsystem.UpClimb());
    NamedCommands.registerCommand("Climb_DOWN", m_ClimberSubsystem.DownClimb());
    NamedCommands.registerCommand("Intake_ON", m_IntakeSubsystem.StartIntake());
    NamedCommands.registerCommand("Intake_OFF", m_IntakeSubsystem.StopIntake());
    NamedCommands.registerCommand("Agitator_ON", m_AgitatorSubsystem.StartAgitator());
    NamedCommands.registerCommand("Agitator_OFF", m_AgitatorSubsystem.StopAgitator());  
  };

  private void configureBindings() {
    // Driver Controls
    // Shooter control

    // Start Shooter (constant speed)
    m_driverController.rightTrigger()
        .onTrue(m_ShooterSubsystem.StartShoot())
        .onFalse(m_ShooterSubsystem.StopShoot());


    m_driverController.rightBumper()
      .onTrue(new InstantCommand(() -> {
        if (m_photonVision.canSeeTags()) {
          remoteOperated = false;
          System.out.println("Saw tags, activating photonvision");
        }
        else {
          System.out.println("WARNING: can't see tags");
          System.out.println("Deactivating photonvision");
          remoteOperated = true;
        }
      }))
      .onFalse(new InstantCommand(() -> {
        m_ShooterSubsystem.StopShoot();
        System.out.println("Bumper released, deactivating photonvision");
      }));

    // Testing to see if the camera returns anything
    // m_driverController.rightTrigger().onTrue(new InstantCommand(() ->
    // m_photonVision.getPose("Arducam OV9782 USB Camera")));
    // Opertor Controls
    // Climber control
      
  
      m_operatorController.a()
      .onTrue(
          m_ClimberSubsystem.OverDown())
      
      .onFalse(
        m_ClimberSubsystem.StopClimb());
      
  



    // Climber Up All the way
    m_operatorController.povUp()
        .onTrue(m_ClimberSubsystem.UpClimb());

    // Climber Down All the way
    m_operatorController.povDown()
        .onTrue(m_ClimberSubsystem.DownClimb());

    // Stop Climb
    m_operatorController.x()
        .onTrue(m_ClimberSubsystem.StopClimb());

    // Intake control
    // Start Intake
    m_operatorController.rightTrigger()
        .onTrue(m_IntakeSubsystem.StartIntake())
        .onFalse(m_IntakeSubsystem.StopIntake());

    // Stop Intake
    m_operatorController.rightBumper()
        .onTrue(m_IntakeSubsystem.ReverseIntake())
        .onFalse(m_IntakeSubsystem.StopIntake());

    // Agitator Control
    // Agitaor Forward
    m_operatorController.leftTrigger()
        .onTrue(m_AgitatorSubsystem.StartAgitator())
        .onFalse(m_AgitatorSubsystem.StopAgitator());

    // Agitator reverse
    m_operatorController.leftBumper()
        .onTrue(m_AgitatorSubsystem.ReverseAgitator())
        .onFalse(m_AgitatorSubsystem.StopAgitator());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}

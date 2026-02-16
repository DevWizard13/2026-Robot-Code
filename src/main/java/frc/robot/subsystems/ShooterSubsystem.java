// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Controls.Driver;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// For CAN
import com.revrobotics.spark.SparkMax;
import org.ejml.equation.IntegerSequence.For;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;


// For PWM
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax Shooter1Motor = new SparkMax(Constants.SubsystemPorts.Shooter1Port, MotorType.kBrushless);
  private PWMVictorSPX Shooter2Motor = new PWMVictorSPX(Constants.SubsystemPorts.Shooter2Port);
  private final RelativeEncoder ShooterEncoder = Shooter1Motor.getEncoder();
  //private final PIDController pid = new PIDController(0, 0.0, 0.0);
  SparkClosedLoopController m_pidController = Shooter1Motor.getClosedLoopController();
  SparkMaxConfig m_config = new SparkMaxConfig();



  public ShooterSubsystem() {
     m_config.closedLoop
          .p(0.0005)
          .i(0)
          .d(0.0001)
          .velocityFF(0.000175)
          .outputRange(-1, 1);


  Shooter1Motor.configure(m_config, 
  SparkMax.ResetMode.kResetSafeParameters,
  SparkMax.PersistMode.kPersistParameters);



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
    SmartDashboard.putNumber("Shooter RPM", ShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Temp C", Shooter1Motor.getMotorTemperature());
    SmartDashboard.putNumber("Shooter Current A", Shooter1Motor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter Voltage V", Shooter1Motor.getBusVoltage());
  }

  public Command StartShoot() {
    return this.run(() -> {

      m_pidController.setReference(3000, SparkMax.ControlType.kVelocity);
      Shooter2Motor.set(0.4);

    });
  }

  public Command StopShoot() {
    return this.run(() -> {
      Shooter1Motor.set(0.0);
      Shooter2Motor.set(0.0);
    });
  }

  public Command ReverseShoot() {
    return this.run(() -> {
      Shooter1Motor.set(Constants.MotorSpeeds.MaxShooterSpeedIn);
      Shooter2Motor.set(0.4);
      System.out.println("Reverse Shoot Command Executed");
    });
  }

    //Named command for PathPlanner
      public Command shoot_1_Command() {
    return this.run(() -> {
      ShooterEncoder.setPosition(0.0); // Reset encoder position to 0
      if (ShooterEncoder.getPosition() < 42) { 
        StartShoot();
      } else {
        StopShoot();
      }
    });
  }





}

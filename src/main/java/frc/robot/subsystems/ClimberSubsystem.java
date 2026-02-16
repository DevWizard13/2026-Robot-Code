// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final TalonFX ClimbMotor = new TalonFX(Constants.SubsystemPorts.ClimberPort);
  // Duty cycle control request (percent output)
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
  double positionRotations = ClimbMotor.getPosition().getValueAsDouble();
  private final DutyCycleOut percentOutput = new DutyCycleOut(0);
  private final VelocityDutyCycle velocityControl = new VelocityDutyCycle(0);

  boolean STOP = false;

  public ClimberSubsystem() {

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

    // ----- SUPPLY CURRENT LIMIT -----
    currentLimits.SupplyCurrentLimit = 10; // Amps
    currentLimits.SupplyCurrentLimitEnable = true;
    // currentLimits.SupplyCurrentThreshold = 60; // Amps before limiting kicks in
    // currentLimits.SupplyTimeThreshold = 0.5; // Seconds over threshold before
    // limiting

    // ----- STATOR CURRENT LIMIT -----
    currentLimits.StatorCurrentLimit = 10; // Amps
    currentLimits.StatorCurrentLimitEnable = true;
    System.out.println("Climber Current Limits Configured");
    // Apply the configuration to the motor
    ClimbMotor.getConfigurator().apply(currentLimits);

    // Make the motor brake when no power is applied
        // ClimbMotor.setNeutralMode(NeutralModeValue.Brake);
    ClimbMotor.setPosition(0.0);
    ClimbMotor.setNeutralMode(NeutralModeValue.Brake);
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

        SmartDashboard.putNumber("Climber Supply Current", ClimbMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Climber Stator Current", ClimbMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Climber Temp C", ClimbMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Climber Position (Rotations)", ClimbMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climber Supply Voltage", ClimbMotor.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Climber Applied Voltage", ClimbMotor.getMotorVoltage().getValueAsDouble());
  

  }

  public Command StopClimb() {
    return this.run(() -> {
      ClimbMotor.setControl(percentOutput.withOutput(0.0));
      STOP = true;
    });
  }

  public Command ZeroClimb() {
    return this.runOnce(() -> {
      ClimbMotor.setPosition(0.0);
      System.out.println("Climber Zeroed");
    });
  }


  public Command UpClimb() {
    return this.run(() -> {
      STOP = false;
      if (ClimbMotor.getPosition().getValueAsDouble() < 46 && !STOP) {
        ClimbMotor.setControl(percentOutput.withOutput(0.2));
      } else {
        StopClimb();
      }
    });
  }


  public Command DownClimb() {
    return this.run(() -> {
      STOP = false;
      if (ClimbMotor.getPosition().getValueAsDouble() > 4 && !STOP) {
        ClimbMotor.setControl(percentOutput.withOutput(-0.2));
      } else {
        StopClimb();
      }

    });

  }
}

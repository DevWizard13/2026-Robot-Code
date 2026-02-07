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

import com.revrobotics.spark.SparkClosedLoopController;

// For PWM
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax Shooter1Motor = new SparkMax(Constants.SubsystemPorts.Shooter1Port, MotorType.kBrushless);
  private SparkMax Shooter2Motor = new SparkMax(Constants.SubsystemPorts.Shooter2Port, MotorType.kBrushless);
  private final RelativeEncoder ShooterEncoder = Shooter1Motor.getEncoder();

  private SparkClosedLoopController pidController;

  // PID controller for speed control

  // Working

  // Working Here
  private final PIDController pid = new PIDController(0.0005, 0.0, 0.00); // Tune these values

  // Target speed in encoder ticks per second
  private double targetSpeed = 3000.0;

  /*
   * ShooterEncoder.setPositionConversionFactor(1.0); // Adjust based on your
   * encoder specs
   * ShooterEncoder.reset();
   * 
   * // Set PID tolerance
   * pid.setTolerance(5.0); // ±5 ticks/sec tolerance
   */
  public ShooterSubsystem() {
    // For CAN
    pidController = Shooter1Motor.getClosedLoopController();
    SparkMaxConfig config = new SparkMaxConfig();

    // Configure the PID gains
    config.closedLoop
        .p(0.1)
        .i(0.0)
        .d(0.01)
        .velocityFF(0.00015)
        .outputRange(-1, 1);

    // Apply the configuration to the motor
    Shooter1Motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    // For PWM
    // Shooter1Motor = new PWMSparkMax(Constants.SubsystemPorts.Shooter1Port);
    // Shooter2Motor = new PWMSparkMax(Constants.SubsystemPorts.Shooter2Port);

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

  public Command StartShoot() {
    return this.run(() -> {

      double currentSpeedRPM = ShooterEncoder.getVelocity(); // NEO default: RPM

      System.out.println("currentSpeed: " + currentSpeedRPM);

      // Calculate motor output from PID
      double output = pid.calculate(currentSpeedRPM, targetSpeed);


      System.out.println("Output: " + output);

      pidController.setReference(output, SparkMax.ControlType.kVelocity);

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
      Shooter2Motor.set(Constants.MotorSpeeds.MaxShooterSpeedIn);
    });
  }

}

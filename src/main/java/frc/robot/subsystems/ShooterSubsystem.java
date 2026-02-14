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

// For PWM
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax Shooter1Motor = new SparkMax(Constants.SubsystemPorts.Shooter1Port, MotorType.kBrushless);
   private PWMVictorSPX Shooter2Motor = new PWMVictorSPX(Constants.SubsystemPorts.Shooter2Port);
  private final RelativeEncoder ShooterEncoder = Shooter1Motor.getEncoder();
    private final PIDController pid = new PIDController(0.1, 0.0, 0.0);






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
  double currentSpeedRPM = ShooterEncoder.getVelocity(); // NEO default: RPM
  SmartDashboard.putNumber("Shooter RPM", currentSpeedRPM);
  }

  public Command StartShoot() {
    return this.run(() -> {
    
   double output = pid.calculate(ShooterEncoder.getVelocity(), Constants.SpeedChange.ShooterTargetSpeed);
     SmartDashboard.putNumber("Shooter Output", output);
        Shooter1Motor.set(output); // Send computed output to motor



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
      System.out.println("Reverse Shoot Command Executed");
    });
   }

}

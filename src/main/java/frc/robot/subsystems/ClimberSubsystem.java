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

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
 private final TalonFX ClimbMotor = new TalonFX(Constants.SubsystemPorts.ClimberPort);
  // Duty cycle control request (percent output)
 private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
  double positionRotations = ClimbMotor.getPosition().getValueAsDouble();
       private final DutyCycleOut percentOutput = new DutyCycleOut(0);
    private final VelocityDutyCycle velocityControl = new VelocityDutyCycle(0);
  
  boolean UpOn = true;
     boolean DownOn = true;

  public ClimberSubsystem() {

    // Make the motor brake when no power is applied
  //  ClimbMotor.setNeutralMode(NeutralModeValue.Brake);
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
  // }

  // public Command UpClimb() {
  /*   return this.run(() -> {

       UpOn = true;
      while(UpOn) {
      positionRotations = ClimbMotor.getPosition().getValueAsDouble();
      System.out.println("Climber Position: " + positionRotations);

      if (positionRotations < 50) { // Prevents the climber from going too high
        ClimbMotor.setControl(dutyCycleRequest.withOutput(0.10));
      } else {
        ClimbMotor.setControl(dutyCycleRequest.withOutput(0.0));
        UpOn = false;
      }

    }
    });*/
  }

  // public Command StopClimb() {
  //   // return this.run(() -> {

  //   //   System.out.println("Climber Position: " + positionRotations);
  //   //   ClimbMotor.setControl(dutyCycleRequest.withOutput(0.0));
  //   //   DownOn = false;
  //   //   UpOn = false;
  //   // });
  
  // }

  // public Command DownClimb() {
  //   return this.run(() -> {

  //     /* 
  //       DownOn = true;

  //      while(DownOn){
  //     positionRotations = ClimbMotor.getPosition().getValueAsDouble();
  //     System.out.println("Climber Position: " + positionRotations);

  //     if (positionRotations > 0) { // Prevents the climber from going too low
  //       ClimbMotor.setControl(dutyCycleRequest.withOutput(-0.10));
  //     } else {
  //       ClimbMotor.setControl(dutyCycleRequest.withOutput(0.0));
  //       DownOn = false;
  //     }}

  //     */
  //   });
  // }

  public Command StopClimb() {
    return this.run(() -> {
      ClimbMotor.setControl(percentOutput.withOutput(0.0));
    });
  }

  public Command ZeroClimb() {
    return this.run(() -> {
      ClimbMotor.setPosition(0.0);
      System.out.println("Climber Zeroed");
    });
  }
  public Command UpClimb() {
    return this.run(() -> {
         positionRotations = ClimbMotor.getPosition().getValueAsDouble();
      if (positionRotations < 52) { // Prevents the climber from going too high 
      ClimbMotor.setControl(percentOutput.withOutput(0.2));
      System.out.println("Climber Position: " + positionRotations);
    }});
  }
  public Command DownClimb() {
    return this.run(() -> {
      positionRotations = ClimbMotor.getPosition().getValueAsDouble();
      if (positionRotations > 4) { // Prevents the climber from going too low
      ClimbMotor.setControl(percentOutput.withOutput(-0.2));
      System.out.println("Climber Position: " + positionRotations);
    }  });
  }
   

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.lib.util.COTSTalonFXSwerveConstants;
//import frc.lib.util.SwerveModuleConstants;
// Credit to Kye for all this, I just ctrl c'd and v'd it
// I don't have the patience to manually resolve dependancies lol

import java.lang.annotation.Target;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class SpeedChange {
        public static final double stickDeadband = 0.07;
        public static final double maxBoostSpeed = 1.00;
        public static final double maxNormalSpeed = 0.70;
        public static final double ShooterTargetSpeed = 3427.0; // Target speed in RPM for the shooter
    }

    public static final class DrivePorts {
        // Drive Motor Ports
        public static final int LEFT_MASTER = 4;
        public static final int LEFT_FOLLOWER = 6;
        public static final int RIGHT_MASTER = 3;
        public static final int RIGHT_FOLLOWER = 5;

    }

    public static final class SubsystemPorts {
        // CAN Ports
        // Shooter Ports
        public static final int Shooter1Port = 10;
        public static final int Shooter2Port = 6;
        // Climber Port
        public static final int ClimberPort = 9;
        // Intake Port
        public static final int IntakePort = 8;
        // Agitator Port
        public static final int AgitatorPort = 15;
    }

    public static final class MotorSpeeds {
        public static final double MaxShooterSpeedOut = 0.55;// Shooter = 55%
        public static final double MaxShooterSpeedIn = -0.24; // Shooter = 8%
        public static final double MaxIntakeSpeed = 0.40; // IntakeSpeed = 40%
        public static final double MaxAgitatorSpeed = 0.30; // Agitator Speed = 30%
        public static final double ClimberSpeed = 0.10; // Climber Speed = 10%
    }

    // Joysticks and Buttons
    public static final class Controls {
        // Driver Joystick and Buttons
        public static final class Driver {
            public static final int kJoystickID = 0;
        }

        // Operator Joystick and Buttons
        public static final class Operator {
            public static final int kJoystickID = 1;
        }
    }


   public static final class vision {
     public static final AprilTagFieldLayout kTagLayout =
                         AprilTagFields.kDefaultField.loadAprilTagLayoutField();

     public static final String[] localizationCameraName = {"dc1", "dc2", "dc3", "sc"};
     // Update the number of cameras later, dc stands for "drive camera" and sc
     // for "shooter camera"
     public static final Transform3d[] localizationCameraToRobot = new Transform3d[4];
     // TODO: add real code for each Transform3d -- actually nvm it won't be used
     
     public static final float[] cameraoffset = {10, -10};
   }


    public static final class ClimberConstants {
        public static final double ClimbUpSpeed = 0.2;
        public static final double ClimbDownSpeed = -0.2;
        public static final double ClimbTarget = 52; // Default target is 52 rotations
    }


}

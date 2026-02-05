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


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

     public static class SpeedChange{
         public static final double stickDeadband = 0.07;
         // TODO: make slider to change on shuffleboard
         public static final double maxStartSpeed = 50.00;
     }


   // Joysticks and Buttons
   public static final class DrivePorts {
       public static final int LEFT_MASTER = 0;
       public static final int LEFT_FOLLOWER = 1;
       public static final int RIGHT_MASTER = 3;
       public static final int RIGHT_FOLLOWER = 2;
   }

   public static final class vision {
     public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

     public static final String[] localizationCameraName = {"dc1", "dc2", "dc3", "sc"};
     // Update the number of cameras later, dc stands for "drive camera" and sc
     // for "shooter camera"
     public static final Transform3d[] localizationCameraToRobot = new Transform3d[4];
     // TODO: add real code for each Transform3d
   }

       // Joysticks and Buttons
       public static final class Controls {
       //Driver Joystick and Buttons
       public static final class Driver {
           public static final int kJoystickID = 0;
       }
       //Operator Joystick and Buttons
       public static final class Operator {
           public static final int kJoystickID = 2;
       }
   }
}

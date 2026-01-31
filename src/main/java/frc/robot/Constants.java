// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


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
          // TODO make slider to change on shuffleboard
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
      public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.LoadAprilTagLayoutField();

      public static final String[4] localizationCameraName = ["dc1", "dc2", "dc3", "sc"];
      // Update the number of cameras later, dc stands for "drive camera" and sc
      // for "shooter camera"
      public static final Transform3d[4] localizationCameraToRobot = [
        new Transform3d(), new Transform3d(), new Transform3d(), new Transform3d()
        // I'm gonna add REAL code here later, but I don't feel like writing
        // that rn
      ];
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

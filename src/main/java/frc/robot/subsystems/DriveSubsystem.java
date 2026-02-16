package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// For CAN
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;




import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveSubsystem extends SubsystemBase {


    
   
    private SparkMax leftMaster = new SparkMax(
        Constants.DrivePorts.LEFT_MASTER, MotorType.kBrushless);
    private SparkMax leftFollower = new SparkMax(
        Constants.DrivePorts.LEFT_FOLLOWER, MotorType.kBrushless);
    private SparkMax rightMaster = new SparkMax(
        Constants.DrivePorts.RIGHT_MASTER, MotorType.kBrushless);
    private SparkMax rightFollower = new SparkMax(
        Constants.DrivePorts.RIGHT_FOLLOWER, MotorType.kBrushless);
        //Encoders
    private final RelativeEncoder rightEncoder = rightMaster.getEncoder();
    private final RelativeEncoder leftEncoder = leftMaster.getEncoder();



     // Tuning constant for correction
    private static final double kP = 0.05; // Proportional gain



    private final MotorControllerGroup leftGroup =
        new MotorControllerGroup(leftMaster, leftFollower);
    private final MotorControllerGroup rightGroup =
        new MotorControllerGroup(rightMaster, rightFollower);


    private final DifferentialDrive drive =
        new DifferentialDrive(leftGroup, rightGroup);


        












    private Pose2d currentPose = new Pose2d();


    public DriveSubsystem() {











        drive.setSafetyEnabled(true);


       
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            config = new RobotConfig(0, 0, null, 0);


       
        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPLTVController(0.02),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    }
}


   
   
    public void arcadeDrive(double fwd, double rot) {
    // if (Math.abs(fwd) > 0.05 && Math.abs(rot) < 0.05) {
    //     //    double correction = leftEncoder.getVelocity() - rightEncoder.getVelocity() * 0.00001;
    //     //     drive.arcadeDrive(fwd, -correction);
    //     //     return;
    //      } else {
        // Normal arcade drive
        drive.arcadeDrive(fwd, rot);
      //  }
    }




    public void tankDrive(double left, double right) {
        drive.tankDrive(left, right);
    }


    public void stop() {
        drive.stopMotor();
    }



    public Pose2d getPose() {
        return currentPose;
    }


    public void resetPose(Pose2d pose) {
        currentPose = pose;
    }


    public ChassisSpeeds getRobotRelativeSpeeds() {
       
        return new ChassisSpeeds(0, 0, 0);
    }


    public void driveRobotRelative(ChassisSpeeds speeds) {
        drive.arcadeDrive(
            speeds.vxMetersPerSecond,
            speeds.omegaRadiansPerSecond
        );
    }






    @Override
    public void periodic() {
        // Put periodic subsystem code here (telemetry, safety checks)

        // Display motor temperatures on SmartDashboard
        // SmartDashboard.putNumber("Left Front Temp C", leftMaster.getMotorTemperature());
        // SmartDashboard.putNumber("Right Front Temp C", rightMaster.getMotorTemperature());
        // SmartDashboard.putNumber("Left Back Temp C", leftFollower.getMotorTemperature());
        // SmartDashboard.putNumber("Right Back Temp C", rightFollower.getMotorTemperature());
        // SmartDashboard.putNumber("Left Side Velocity", leftEncoder.getVelocity());
        // SmartDashboard.putNumber("Right Side Velocity", rightEncoder.getVelocity());



        boolean isHot =  leftMaster.getMotorTemperature() > 50 ||  rightMaster.getMotorTemperature() > 50 ||  leftFollower.getMotorTemperature() > 50 ||  rightFollower.getMotorTemperature() > 50;
        SmartDashboard.putBoolean("Drive Overheating", isHot);








}}




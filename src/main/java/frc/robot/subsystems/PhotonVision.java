/*package frc.robot.subsystems;
import java.util.Optional;

import java.util.*;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.*;
import frc.robot.Constants;

public class PhotonVision {
	List<PhotonCamera> robotCameras;
  List<PhotonPoseEstimator> cameraEst = new ArrayList<>();
  Pose2d targetPose = new Pose2d();
  DriveSubsystem drive = new DriveSubsystem();
  double turnModifier = 0.01;
  float[] offset = Constants.vision.cameraoffset;
  float maxDistance = Constants.vision.maxDistanceToTarget;

	public PhotonVision (List<PhotonCamera> cameras, Pose2d target) {
    robotCameras = cameras;
    targetPose = target;

    // Make a list of PhotonPoseEstimators
    for (int i = 0; i < robotCameras.size(); i++) {
    	cameraEst.add(new PhotonPoseEstimator(Constants.vision.kTagLayout,
    		                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    		                        Constants.vision.localizationCameraToRobot[i]));

    	cameraEst.get(i).setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
	}

  /*

	public Optional<EstimatedRobotPose> getPose(String cameraName) {
		// check to see which camera matches the desired name, and when it's found,
		// return the pose
		Optional<EstimatedRobotPose> visionEst = Optional.empty();
		for (PhotonCamera i : robotCameras) {
			if (i.getName() == cameraName) {
        int index = robotCameras.indexOf(i);
		    visionEst = cameraEst.get(index).update(i.getLatestResult());
		    break;
		    // there better not be multiple cameras of the same name
		    // if so, I'm gonna have a talk with someone! /hj
			}
		}

		if (visionEst.isPresent()) {
			return visionEst;
		}
		else {
      System.out.println("ERROR: Could not identify any cameras");
			return Optional.empty();
		}
	}

  public float getDistanceToTag() {
    Optional<EstimatedRobotPose> robotPose = getPose(Constants.vision.localizationCameraName[0]);
    if (robotPose.isEmpty()) {
      System.out.println("ERROR: cannot determine pose");
      return (float)Double.POSITIVE_INFINITY;
    }
    else {
      Pose2d myPose = robotPose.get().estimatedPose.toPose2d();
      return (float)PhotonUtils.getDistanceToPose(myPose, targetPose);
    }
  }

  public boolean aimAtTarget() {
    // I made it a bool so you can identify if it's corrrectly aimed

    PhotonCamera camera = robotCameras.get(0);
    boolean targetVisible = false;
    double turn = 0.0;
    double speed = 0.0;
    List<Double> targetYaw = new ArrayList<>();
    // I'll replace it with the camera at the front

    for (PhotonCamera myCamera : robotCameras) {
      var results = myCamera.getAllUnreadResults();
      
      if (!results.isEmpty()) {
        // Camera processed a new frame since last
        // Get the last one in the list.
      
        var result = results.get(results.size() - 1);
      
        if (result.hasTargets()) {
          // At least one AprilTag was seen by the camera
          for (var target : result.getTargets()) {
            if (target.getFiducialId() == 7) {
              // Found Tag 7, record its information
              targetYaw.add(target.getYaw() + 
                  offset[robotCameras.indexOf(myCamera)]);
      
              targetVisible = true;
            }
          }
        }
      }
    }

    double avgTargetYaw = calculateAvg(targetYaw);

    if (targetVisible) {
      if (Math.abs(avgTargetYaw) > 5.0) {
        if (avgTargetYaw > 0) {
          turn = -1.0;
        } else {
          turn = 1.0;
        }
      }
      else if (Math.abs(avgTargetYaw) < 5.0) {
        if (avgTargetYaw > 0) {
          turn = -0.5;
        } else {
          turn = 0.5;
        }
      } 
    }
    else {
      System.out.println("Target not visible");
    }

    if (getDistanceToTag() > maxDistance) {
      speed = 1.0;
      drive.arcadeDrive(speed, turn);
      return false;
    }

    drive.arcadeDrive(speed, turn);

    if (Math.round(avgTargetYaw) != 0) {
      return true;
    }
    else {
      return false;
    }
  }

  double calculateAvg(List<Double> val) {
    double x = 0.0;
    for (double i : val){
      x += i;
    };
    x = x / val.size();
    return x;
  }
  */
}
*/
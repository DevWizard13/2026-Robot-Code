package frc.robot.subsystems;

import java.util.*;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.*;
import frc.robot.Constants;

public class PhotonVision {
	List<PhotonCamera> robotCameras;
  List<PhotonPoseEstimator> cameraEst = new ArrayList<>();

	public PhotonVision (List<PhotonCamera> cameras) {
    List<PhotonCamera> robotCameras = cameras;

    // Make a list of PhotonPoseEstimators
    for (int i = 0; i < robotCameras.size(); i++) {
    	cameraEst.add(new PhotonPoseEstimator(Constants.vision.kTagLayout,
    		                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    		                        Constants.vision.localizationCameraToRobot[i]));

    	cameraEst.get(i).setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
	}

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

  public float getDistanceToTag(String cameraName, Pose2d targetPose) {
    Optional<EstimatedRobotPose> robotPose = getPose(cameraName);
    if (robotPose.isEmpty()) {
      System.out.println("ERROR: cannot determine pose");
      return 0f;
    }
    else {
      Pose2d myPose = robotPose.get().estimatedPose.toPose2d();
      return (float)PhotonUtils.getDistanceToPose(myPose, targetPose);
    }
  }
}

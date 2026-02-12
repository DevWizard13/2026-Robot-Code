package frc.robot.subsystems;

import java.util.Optional;
import java.util.*;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.robot.Constants;

public class PhotonVision {
	List<PhotonCamera> robotCameras;
  List<PhotonPoseEstimator> cameraEst = new ArrayList<>();

	public PhotonVision (List<PhotonCamera> cameras) {
    List<PhotonCamera> robotCameras = cameras;

    // Make a list of PhotonPoseEstimators
    for (int i = 0; i < robotCameras.size(); i++) {
    	PhotonCamera camera = new PhotonCamera(Constants.vision.localizationCameraName[i]);
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
}
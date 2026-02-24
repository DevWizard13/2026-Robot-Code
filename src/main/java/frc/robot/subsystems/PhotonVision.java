package frc.robot.subsystems;
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

  public double getRotToTarget() {
    List<Pose2d> pos = new ArrayList();
    double x = 0.0;
    double y = 0.0;

    for (String name: Constants.vision.localizationCameraName) {
      Optional<EstimatedRobotPose> visionEst = getPose(name);

	    if (visionEst.isPresent()) {
	    	pos.add(visionEst.get().estimatedPose.toPose2d());
	    }
    }
    for (Pose2d location : pos) {
      x += location.getX();
      y += location.getY();

      x /= pos.size();
      y /= pos.size();
    }

    return Math.atan(x/y);
  }

  public double getDriveToTarget() {
    List<Pose2d> pos = new ArrayList();
    double x = 0.0;
    double y = 0.0;

    for (String name: Constants.vision.localizationCameraName) {
      Optional<EstimatedRobotPose> visionEst = getPose(name);

	    if (visionEst.isPresent()) {
	    	pos.add(visionEst.get().estimatedPose.toPose2d());
	    }
    }
    for (Pose2d location : pos) {
      x += location.getX();
      y += location.getY();
      
      x /= pos.size();
      y /= pos.size();
    }

    return Math.sqrt((x * x) + (y * y));
  }
}


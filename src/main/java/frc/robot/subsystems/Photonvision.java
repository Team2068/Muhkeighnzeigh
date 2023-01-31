// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.RobotConstants;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Photonvision extends SubsystemBase {

  PhotonCamera camera = new PhotonCamera("OV5647");
  AprilTagFieldLayout aprilTagFieldLayout;

  ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
  PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT, camera, RobotConstants.robotToCam);

  public Photonvision() {
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
    } catch (Exception e) {
      System.out.println(e);
    }
    camList.add(new Pair<PhotonCamera, Transform3d>(camera, RobotConstants.robotToCam));
    GameConstants.tagMap.put(1, new Double[]{1551.35, 107.16, 46.27, 180.0});
    GameConstants.tagMap.put(2, new Double[]{1551.35, 274.80, 46.27, 180.0});
    GameConstants.tagMap.put(3, new Double[]{1551.35, 442.44, 46.27, 180.0});
    GameConstants.tagMap.put(4, new Double[]{1617.87, 674.97, 69.54, 180.0});
    GameConstants.tagMap.put(5, new Double[]{36.19, 674.97, 69.54, 0.0});
    GameConstants.tagMap.put(6, new Double[]{102.743, 442.44, 46.27, 0.0});
    GameConstants.tagMap.put(7, new Double[]{102.743, 274.80, 46.27, 0.0});
    GameConstants.tagMap.put(8, new Double[]{102.743, 107.16, 46.27, 0.0});

  }

  public class CameraData {
    public boolean hasTargets;
    public double targetPitch;
    public double targetYaw;
    public double targetArea;
    public double targetSkew;
    public Transform3d targetPose;
  }

  public class AprilTagData {
    public int targetId;
    public double poseAmbiguity;
    //public Optional<Pose3d> tagPose;
    public Double[] tagPose2;
    public Transform3d alternateCameraToTarget;
  }

  public CameraData data = new CameraData();
  public AprilTagData tagData = new AprilTagData();

  public int getPipelineIndex() {
    return camera.getPipelineIndex();
  }

  public void togglePipeline() {
    camera.setPipelineIndex( (getPipelineIndex() == 1) ? 2 : 1);
  }

  public void updateData() {
    PhotonPipelineResult results = camera.getLatestResult();

    if (results == null) {
      return;
    }

    PhotonTrackedTarget bestTarget = results.getBestTarget();

    if (!results.hasTargets()) {
      return;
    }

    if (bestTarget == null) {
      return;
    }

    data.hasTargets = results.hasTargets();
    data.targetPitch = bestTarget.getPitch();
    data.targetYaw = bestTarget.getYaw();
    data.targetArea = bestTarget.getArea();
    data.targetSkew = bestTarget.getSkew();
    data.targetPose = bestTarget.getBestCameraToTarget();

    if (getPipelineIndex() == 2) { // if target is an apriltag target
      tagData.targetId = bestTarget.getFiducialId();
      tagData.poseAmbiguity = bestTarget.getPoseAmbiguity();
      //tagData.tagPose = aprilTagFieldLayout.getTagPose(tagData.targetId);
      tagData.tagPose2 = GameConstants.tagMap.get(tagData.targetId);
      tagData.alternateCameraToTarget = bestTarget.getAlternateCameraToTarget();
    }
    return;
    
  }

  public double getDistance(PhotonPipelineResult results) {
    return (!results.hasTargets()) ? 0
        : PhotonUtils.calculateDistanceToTargetMeters(RobotConstants.camHeight,
            GameConstants.aprilTagHeight, RobotConstants.camAngle,
            Units.degreesToRadians(results.getBestTarget().getPitch()));
  }

  public Translation2d getTranslationFromTarget(PhotonPipelineResult results) {
    return PhotonUtils.estimateCameraToTargetTranslation(getDistance(results), Rotation2d.fromDegrees(-data.targetYaw));
  }

  public Pose3d getFieldPose(PhotonPipelineResult results, PhotonTrackedTarget bestTarget, DriveSubsystem drive) {
    Pose2d p = drive.getPose();
    Transform3d val = new Transform3d(new Translation3d(p.getX(), RobotConstants.camHeight, p.getY()), drive.getGyroRotation());
    return PhotonUtils.estimateFieldToRobotAprilTag(data.targetPose, aprilTagFieldLayout.getTagPose(tagData.targetId).get(), val);
  }

  @Override
  public void periodic() {
    if(camera.getLatestResult() == null)
      return;
      if (!camera.getLatestResult().hasTargets())
        return;
      
    updateData();

    SmartDashboard.putNumber("target pitch", data.targetPitch);
    SmartDashboard.putNumber("target yaw", data.targetYaw);
    SmartDashboard.putNumber("target area", data.targetArea);
    SmartDashboard.putNumber("target skew", data.targetSkew);

    SmartDashboard.putNumber("target x", data.targetPose.getX());
    SmartDashboard.putNumber("target y", data.targetPose.getY());
    SmartDashboard.putNumber("target z", data.targetPose.getZ());
    SmartDashboard.putNumber("target rotation", data.targetPose.getRotation().getAngle());

    // SmartDashboard.putNumber("apriltag id", tagData.targetId);
    // SmartDashboard.putNumber("apriltag pose ambiguity", tagData.poseAmbiguity);

    // SmartDashboard.putNumber("apriltag x pos", tagData.tagPose2[0]);
    // SmartDashboard.putNumber("apriltag y pos", tagData.tagPose2[1]);
    // SmartDashboard.putNumber("apriltag z pos", tagData.tagPose2[2]);
    // SmartDashboard.putNumber("apriltag rotation", tagData.tagPose2[3]);

    SmartDashboard.putNumber("distance", getDistance(camera.getLatestResult()));
    
  }
}

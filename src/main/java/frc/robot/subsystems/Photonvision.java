// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class Photonvision extends SubsystemBase {

  PhotonCamera camera = new PhotonCamera("photonvision");
  AprilTagFieldLayout aprilTagFieldLayout;

  /** Creates a new Photonvision. */
  public Photonvision() {
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
    }
    catch (Exception e) {
      System.out.println(e);
    }
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
    public boolean hasTargets;
    public double targetPitch;
    public double targetYaw;
    public double targetArea;
    public int targetId;
    public double poseAmbiguity;
    public Optional<Pose3d> targetPose;
    public Transform3d bestCameraToTarget;
    public Transform3d alternateCameraToTarget;
  }

  private CameraData data = new CameraData();
  private AprilTagData tagData = new AprilTagData();

  public void togglePipeline () {
      if (camera.getPipelineIndex() == 1) {
        camera.setPipelineIndex(2);
      }
      else {
        camera.setPipelineIndex(1);
      }
  }

  public void updateData (PhotonPipelineResult results, PhotonTrackedTarget bestTarget) {
    
    if (camera.getPipelineIndex() == 1) { //if target is an apriltag target
      tagData.hasTargets = results.hasTargets();
      tagData.targetPitch = bestTarget.getPitch();
      tagData.targetYaw = bestTarget.getYaw();
      tagData.targetArea = bestTarget.getArea();
      tagData.targetId = bestTarget.getFiducialId();
      tagData.poseAmbiguity = bestTarget.getPoseAmbiguity();
      tagData.targetPose = aprilTagFieldLayout.getTagPose(tagData.targetId);
      tagData.bestCameraToTarget = bestTarget.getBestCameraToTarget();
      tagData.alternateCameraToTarget = bestTarget.getAlternateCameraToTarget();
    }
    else {
      data.hasTargets = results.hasTargets();
      data.targetPitch = bestTarget.getPitch();
      data.targetYaw = bestTarget.getYaw();
      data.targetArea = bestTarget.getArea();
      data.targetSkew = bestTarget.getSkew();
      data.targetPose = bestTarget.getBestCameraToTarget();
    }
  }

  public double getDistance (PhotonPipelineResult results, PhotonTrackedTarget bestTarget) {
    double distance;
    if (results.hasTargets()) {
      distance = PhotonUtils.calculateDistanceToTargetMeters(Constants.RobotConstants.camHeight, Constants.GameConstants.aprilTagHeight, Constants.RobotConstants.camAngle, Units.degreesToRadians(bestTarget.getPitch()));
    }
    else {
      distance = 0;
    }

    return distance;
  }

  public Translation2d getTranslationFromTarget (PhotonPipelineResult results, PhotonTrackedTarget bestTarget) {
    return PhotonUtils.estimateCameraToTargetTranslation(this.getDistance(results, bestTarget), Rotation2d.fromDegrees(-data.targetYaw));
  }

  // public Pose3d getFieldPose (PhotonPipelineResult results, PhotonTrackedTarget bestTarget) {
  //   return PhotonUtils.estimateFieldToRobotAprilTag(tagData.bestCameraToTarget, targetData.targetPose, null);
  // }

  // public Rotation2d getXOffset (PhotonPipelineResult results, PhotonTrackedTarget bestTarget) {
  //   if (camera.getPipelineIndex() == 1) {
  //     return PhotonUtils.getYawToPose(robotPose, tagData.targetPose);
  //   }
  //   else {
  //     return PhotonUtils.getYawToPose(robotPose, targetPose);
  //   }
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    PhotonPipelineResult results = camera.getLatestResult();
    PhotonTrackedTarget bestTarget = results.getBestTarget();

    updateData(results, bestTarget);
    
  }
}

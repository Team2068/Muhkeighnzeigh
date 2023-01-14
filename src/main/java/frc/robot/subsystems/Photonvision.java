// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.RobotConstants;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Photonvision extends SubsystemBase {

  PhotonCamera camera = new PhotonCamera("photonvision");
  AprilTagFieldLayout aprilTagFieldLayout;

  public Photonvision() {
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
    } catch (Exception e) {
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
    public int targetId;
    public double poseAmbiguity;
    public Optional<Pose3d> tagPose;
    public Transform3d alternateCameraToTarget;
  }

  public CameraData data = new CameraData();
  public AprilTagData tagData = new AprilTagData();

  public int getPipelineIndex() {
    return camera.getPipelineIndex();
  }

  public void togglePipeline() {
    camera.setPipelineIndex( (camera.getPipelineIndex() == 1) ? 2 : 1);
  }

  public void updateData() {
    PhotonPipelineResult results = camera.getLatestResult();
    PhotonTrackedTarget bestTarget = results.getBestTarget();

    data.hasTargets = results.hasTargets();
    data.targetPitch = bestTarget.getPitch();
    data.targetYaw = bestTarget.getYaw();
    data.targetArea = bestTarget.getArea();
    data.targetSkew = bestTarget.getSkew();
    data.targetPose = bestTarget.getBestCameraToTarget();

    if (camera.getPipelineIndex() == 1) { // if target is an apriltag target
      tagData.targetId = bestTarget.getFiducialId();
      tagData.poseAmbiguity = bestTarget.getPoseAmbiguity();
      tagData.tagPose = aprilTagFieldLayout.getTagPose(tagData.targetId);
      tagData.alternateCameraToTarget = bestTarget.getAlternateCameraToTarget();
    }
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
    updateData();
  }
}

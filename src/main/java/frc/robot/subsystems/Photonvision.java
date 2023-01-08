// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class Photonvision extends SubsystemBase {

  PhotonCamera camera = new PhotonCamera("photonvision");

  /** Creates a new Photonvision. */
  public Photonvision() {
    try {
      AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
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
    public double[] targetPose;
  }
  
  public class AprilTagData extends CameraData {
    public int targetId;
    public double poseAmbiguity;
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
    data.hasTargets = results.hasTargets();
    data.targetPitch = bestTarget.getPitch();
    data.targetYaw = bestTarget.getYaw();
    data.targetArea = bestTarget.getArea();
    data.targetSkew = bestTarget.getSkew();

    if (true) { //if target is an apriltag target
      tagData.targetId = bestTarget.getFiducialId();
      tagData.poseAmbiguity = bestTarget.getPoseAmbiguity();
      tagData.bestCameraToTarget = bestTarget.getBestCameraToTarget();
      tagData.alternateCameraToTarget = bestTarget.getAlternateCameraToTarget();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    PhotonPipelineResult results = camera.getLatestResult();
    PhotonTrackedTarget bestTarget = results.getBestTarget();

    updateData(results, bestTarget);
    
  }
}

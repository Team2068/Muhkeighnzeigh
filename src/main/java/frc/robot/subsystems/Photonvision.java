// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.PhotonConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Photonvision extends SubsystemBase {
  public PhotonCamera camera;
  public Servo mount = new Servo(PhotonConstants.SERVO_PORT);

  public Photonvision(String camName) {
    camera = new PhotonCamera(camName);
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
    public Pose3d tagPose2;
    public Transform3d alternateCameraToTarget;
  }

  public CameraData data = new CameraData();
  public AprilTagData tagData = new AprilTagData();

  public void togglePipeline() {
    if (camera.getPipelineIndex() == PhotonConstants.REFLECTIVE_TAPE_PIPELINE_INDEX) {
      camera.setPipelineIndex(PhotonConstants.APRILTAG_PIPELINE_INDEX);
      camera.setLED(VisionLEDMode.kOff);
      System.out.println(camera.getLEDMode());
    } else if (camera.getPipelineIndex() == PhotonConstants.APRILTAG_PIPELINE_INDEX) {
      camera.setPipelineIndex(PhotonConstants.REFLECTIVE_TAPE_PIPELINE_INDEX);
      camera.setLED(VisionLEDMode.kOn);
      // NetworkTableInstance.getDefault().getTable("photonvision").getEntry("ledMode").setInteger(0);
      System.out.println(camera.getLEDMode());
    }
  }

  public void updateData() {
    PhotonPipelineResult results = camera.getLatestResult();

    if (results == null) {
      return;
    }

    PhotonTrackedTarget bestTarget = results.getBestTarget();

    if (!results.hasTargets() || bestTarget == null) {
      return;
    }

    data.hasTargets = results.hasTargets();
    data.targetPitch = bestTarget.getPitch();
    data.targetYaw = bestTarget.getYaw();
    data.targetArea = bestTarget.getArea();
    data.targetSkew = bestTarget.getSkew();
    data.targetPose = bestTarget.getBestCameraToTarget();

    if (camera.getPipelineIndex() == PhotonConstants.APRILTAG_PIPELINE_INDEX) { // if target is an apriltag target
      tagData.targetId = bestTarget.getFiducialId();
      tagData.poseAmbiguity = bestTarget.getPoseAmbiguity();
      // tagData.tagPose2 = new Pose3d(
      //     GameConstants.TAG_ARRAY[tagData.targetId - 1][0],
      //     GameConstants.TAG_ARRAY[tagData.targetId - 1][1],
      //     GameConstants.TAG_ARRAY[tagData.targetId - 1][2],
      //     new Rotation3d(0, 0, GameConstants.TAG_ARRAY[tagData.targetId - 1][3]));
      tagData.alternateCameraToTarget = bestTarget.getAlternateCameraToTarget();
    }
    return;
  }

  public double getDistance(PhotonPipelineResult results) {
    if (!results.hasTargets()) {
      return 0;
    } else if (camera.getPipelineIndex() == PhotonConstants.REFLECTIVE_TAPE_PIPELINE_INDEX) {
      return PhotonUtils.calculateDistanceToTargetMeters(PhotonConstants.CAM_HEIGHT,
          GameConstants.REFLTAPE_HEIGHT_LOWER, PhotonConstants.CAM_ANGLE,
          Units.degreesToRadians(data.targetPitch));
    } else {
      return PhotonUtils.calculateDistanceToTargetMeters(PhotonConstants.CAM_HEIGHT,
          GameConstants.APRILTAG_HEIGHT, PhotonConstants.CAM_ANGLE,
          Units.degreesToRadians(data.targetPitch));
    }
  }

  public void rotateMount() {
    mount.setAngle((mount.getAngle() == PhotonConstants.FORWARD_ANGLE) ? PhotonConstants.BACKWARD_ANGLE : PhotonConstants.FORWARD_ANGLE );
  }

  public void rotateMount(double armAngle){
    mount.setAngle((armAngle < 0) ? PhotonConstants.BACKWARD_ANGLE : PhotonConstants.FORWARD_ANGLE);
  }

  public boolean isFlipped () {
    return mount.getAngle() > 90;
  }

  @Override
  public void periodic() {
    if (camera.getLatestResult() == null)
      return;
    if (!camera.getLatestResult().hasTargets())
      return;

    updateData();

    SmartDashboard.putNumber("servo angle", mount.getAngle());
    SmartDashboard.putBoolean("isFlipped", isFlipped());
  }
}

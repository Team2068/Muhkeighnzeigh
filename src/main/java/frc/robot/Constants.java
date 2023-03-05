// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final double DRIVE_MAX_VELOCITY_METERS_PER_SECOND = 0.2;
  public static final double DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.2;

  public static int CURRENT_LIMIT = 30;

  public enum ChassisConfiguration {
    MAIN,
    PRACTICE
  }

  public final static class ControllerConstants {
    public static final int RIGHT_TRIGGER = 3;
    public static final int LEFT_TRIGGER = 2;
    public static final double TRIGGER_ACTIVATION_THRESHOLD = .3;
    public static final int POV_ANGLE_UP = 0;
    public static final int POV_ANGLE_LEFT = 270;
    public static final int POV_ANGLE_RIGHT = 90;
  }

  public static ChassisConfiguration getChassisConfiguration() {
    return System.getenv("PRACTICE_ROBOT") != null ? ChassisConfiguration.PRACTICE : ChassisConfiguration.MAIN;
  }

  public static final class DriveConstants {
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(19.5);
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(21.5);

    public static final int FRONT_LEFT_DRIVE_MOTOR = 6;
    public static final int FRONT_LEFT_TURN_MOTOR = 7;
    public static final int FRONT_LEFT_ENCODER = 15;
    public static double FRONT_LEFT_ENCODER_OFFSET;

    public static final int FRONT_RIGHT_DRIVE_MOTOR = 8;
    public static final int FRONT_RIGHT_TURN_MOTOR = 9;
    public static final int FRONT_RIGHT_ENCODER = 14;
    public static double FRONT_RIGHT_ENCODER_OFFSET;

    public static final int BACK_LEFT_DRIVE_MOTOR = 4;
    public static final int BACK_LEFT_TURN_MOTOR = 5;
    public static final int BACK_LEFT_ENCODER = 16;
    public static double BACK_LEFT_ENCODER_OFFSET;

    public static final int BACK_RIGHT_DRIVE_MOTOR = 10;
    public static final int BACK_RIGHT_TURN_MOTOR = 11;
    public static final int BACK_RIGHT_ENCODER = 13;
    public static double BACK_RIGHT_ENCODER_OFFSET;

    public static final int PIGEON_ID = 19;

    public static final void setOffsets() {
      if (Constants.getChassisConfiguration() == ChassisConfiguration.MAIN) {
        FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(337);
        FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(151);
        BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(228);
        BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(144);
      } else {
        FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(346);
        FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(210);
        BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(230);
        BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(138);
      }
    }
  }

  public static final class ArmConstants {
    public static final int ARM_1_MOTOR = 2;
    public static final int ARM_2_MOTOR = 3;

    public static final double ARM_SPEED = -.25;
    public static final double ARM_OFFSET = 0.427;
  }

  public static final class ClawConstants {
    public static final int WRIST_MOTOR = 17;
    public static final int INTAKE_MOTOR = 18;

    public static final double WRIST_OFFSET = 0;
    public static final double INTAKE_SPEED = .75;
    public static final double WRIST_SPEED = .5;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.2;
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 * Math.PI;

    public static final double kPXController = 2;
    public static final double kPYController = 2;
    public static final double kPThetaController = 2.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(1,
        1);
  }

  public static final class TelescopeConstants {
    public static final int TELESCOPE_MOTOR = 12;
    public static final double TELESCOPE_SPEED = .25;
  }

  public static class Paths {
    public static final PathPlannerTrajectory bounce = PathPlanner.loadPath("Bounce", new PathConstraints(1, 0.75));
    public static final PathPlannerTrajectory funny = PathPlanner.loadPath("Funny", new PathConstraints(2, 2));
    public static final PathPlannerTrajectory loop = PathPlanner.loadPath("Loop", new PathConstraints(1, 0.75));
  }

  public static class RobotConstants {
    public static final double camHeight = 0.1524; // meters
    public static final double camAngle = Units.degreesToRadians(20); // replace with actual angle of the camera
    public static final Transform3d robotToCam = new Transform3d(
        new Translation3d(-3, 0.5, 5.5),
        new Rotation3d(0, camAngle, 0));
    public static final String camName1 = "OV5647";
  }

  public static class GameConstants {
    public static final double[][] tagArray = {
        { 1551.35, 107.16, 46.27, 180.0 },
        { 1551.35, 274.80, 46.27, 180.0 },
        { 1551.35, 442.44, 46.27, 180.0 },
        { 1617.87, 674.97, 69.54, 180.0 },
        { 36.19, 674.97, 69.54, 0.0 },
        { 102.743, 442.44, 46.27, 0.0 },
        { 102.743, 274.80, 46.27, 0.0 },
        { 102.743, 107.16, 46.27, 0.0 } };
    public static final double aprilTagHeight = Units.inchesToMeters(17.5); // CM
    public static final double reflectiveTapeHeightLower = 0.6096; // meters
    public static final double reflectiveTapeHeightUpper = 1.0668; // meters
  }

  public static class AimbotConstants {
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double speed = 0.5;
    public static final double minimumAdjustment = 0.5;
  }
}

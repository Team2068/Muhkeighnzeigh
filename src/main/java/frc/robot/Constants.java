// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double DRIVE_MAX_VELOCITY_METERS_PER_SECOND = .2;
  public static final double DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = .2;

  public static int CURRENT_LIMIT = 30;

  public enum ChassisConfiguration {
    MAIN,
    PRACTICE
  }

  public static final class DriveConstants {
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(19.5);
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(21.5);

    public static final int FRONT_LEFT_DRIVE_MOTOR = 4;
    public static final int FRONT_LEFT_TURN_MOTOR = 5;
    public static final int FRONT_LEFT_ENCODER = 10;
    public static double FRONT_LEFT_ENCODER_OFFSET;

    public static final int FRONT_RIGHT_DRIVE_MOTOR = 6;
    public static final int FRONT_RIGHT_TURN_MOTOR = 7;
    public static final int FRONT_RIGHT_ENCODER = 11;
    public static double FRONT_RIGHT_ENCODER_OFFSET;

    public static final int BACK_LEFT_DRIVE_MOTOR = 2;
    public static final int BACK_LEFT_TURN_MOTOR = 3;
    public static final int BACK_LEFT_ENCODER = 12;
    public static double BACK_LEFT_ENCODER_OFFSET;

    public static final int BACK_RIGHT_DRIVE_MOTOR = 8;
    public static final int BACK_RIGHT_TURN_MOTOR = 9;
    public static final int BACK_RIGHT_ENCODER = 13;
    public static double BACK_RIGHT_ENCODER_OFFSET;

    public static final void setOffsets(ChassisConfiguration chassis) {
      if (chassis == ChassisConfiguration.MAIN) {
        FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(359);
        FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(344);
        BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(45);
        BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(291);
      } else {
        FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(346);
        FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(68);
        BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(230);
        BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(84);
      }
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.2;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 4;
    public static final double kPYController = 4;
    public static final double kPThetaController = 2;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(1,
        1);
  }

  public static class Paths {
    public static final PathPlannerTrajectory bounce = PathPlanner.loadPath("Bounce", new PathConstraints(2, 0.75));
    public static final PathPlannerTrajectory funny = PathPlanner.loadPath("Funny", new PathConstraints(2, 2));
  }

  public static class RobotConstants {
    public static final double camHeight = 6.875; //replace w actual height in cm
    public static final double camAngle = Units.degreesToRadians(20); //replace with actual angle of the camera
    public static final Transform3d robotToCam = new Transform3d(
      new Translation3d(0, 0, 0),
      new Rotation3d(0, camAngle, 0)
    );
  }

  public static class GameConstants {
    //public static final HashMap<Integer, Double[]> tagMap = new HashMap<Integer, Double[]>(8);
    public static final Double[][] tagArray = {
      {1551.35, 107.16, 46.27, 180.0}, 
      {1551.35, 274.80, 46.27, 180.0}, 
      {1551.35, 442.44, 46.27, 180.0}, 
      {1617.87, 674.97, 69.54, 180.0}, 
      {36.19, 674.97, 69.54, 0.0}, 
      {102.743, 442.44, 46.27, 0.0}, 
      {102.743, 274.80, 46.27, 0.0}, 
      {102.743, 107.16, 46.27, 0.0}};
    public static final double aprilTagHeight = 48; // CM
    public static final double reflectiveTapeHeight = 49; //cm
    //public static final HashMap<Integer, Double[]> tagMap = new HashMap<Integer, Double[]>(8);
  }
  
  public static class AimbotConstants {
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double speed = 0.5;
    public static final double minimumAdjustment = 0.5;
  }
 
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static final int FRONT_LEFT_DRIVE_MOTOR = 5;
    public static final int FRONT_LEFT_TURN_MOTOR = 6;
    public static final int FRONT_LEFT_ENCODER = 11;
    public static double FRONT_LEFT_ENCODER_OFFSET;

    public static final int FRONT_RIGHT_DRIVE_MOTOR = 7;
    public static final int FRONT_RIGHT_TURN_MOTOR = 8;
    public static final int FRONT_RIGHT_ENCODER = 12;
    public static double FRONT_RIGHT_ENCODER_OFFSET;

    public static final int BACK_LEFT_DRIVE_MOTOR = 3;
    public static final int BACK_LEFT_TURN_MOTOR = 4;
    public static final int BACK_LEFT_ENCODER = 13;
    public static double BACK_LEFT_ENCODER_OFFSET;

    public static final int BACK_RIGHT_DRIVE_MOTOR = 9;
    public static final int BACK_RIGHT_TURN_MOTOR = 10;
    public static final int BACK_RIGHT_ENCODER = 14;
    public static double BACK_RIGHT_ENCODER_OFFSET;

    public static final void setOffsets() {
      if (Constants.getChassisConfiguration() == ChassisConfiguration.MAIN) {
        FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(9);
        FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(344);
        BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(128);
        BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(291);
      } else {
        FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(346);
        FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(68);
        BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(223);
        BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(48);
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
    public static final double ArmEncoder1 = 18;
    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(1,
        1);
  }

  public static final class ArmConstants {
    public static final int ArmMotor1 = 15;
    public static final int ArmMotor2 = 16;
    public static final double ArmLiftSpeed = .25;
    public static final double ArmLowerSpeed = -.25;
    public static final double ARMOFFSET = -Math.toRadians(0);
  }

  public static final class ClawConstants {
    public static final int IntakeMotor = 17;
    public static final int Claw = 18;
    public static final double IntakeSpeed = .75;
    public static final double ClawSpeed = .5;
  }

  public static final class TelescopeConstants {
    public static final int TelescopeMotor = 18;
    public static final double telescopeSpeed = .25;

  }

  public static class Paths {
    public static final PathPlannerTrajectory bounce = PathPlanner.loadPath("Bounce", new PathConstraints(2, 0.75));
    public static final PathPlannerTrajectory funny = PathPlanner.loadPath("Funny", new PathConstraints(2, 2));
  }
}

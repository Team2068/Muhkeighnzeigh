// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double DRIVE_MAX_VELOCITY_METERS_PER_SECOND = .5;
  public static final double DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = .5;
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(19.5);
  public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(21.5);
  public static int CURRENT_LIMIT = 30;
  public static final class DriveConstants{

    public static final int FRONT_RIGHT_DRIVE_MOTOR = 2;
    public static final int FRONT_RIGHT_TURN_MOTOR = 3;
    public static final int FRONT_RIGHT_ENCODER = 10;
    public static final double FRONT_RIGHT_ENCODER_OFFSET = Math.toRadians(0);

    public static final int BACK_RIGHT_DRIVE_MOTOR = 4;
    public static final int BACK_RIGHT_TURN_MOTOR = 5;
    public static final int BACK_RIGHT_ENCODER = 11;
    public static final double BACK_RIGHT_ENCODER_OFFSET = Math.toRadians(0);

    public static final int FRONT_LEFT_DRIVE_MOTOR = 6;
    public static final int FRONT_LEFT_TURN_MOTOR = 7;
    public static final int FRONT_LEFT_ENCODER = 12;
    public static final double  FRONT_LEFT_ENCODER_OFFSET = Math.toRadians(0);

    public static final int BACK_LEFT_DRIVE_MOTOR = 8;
    public static final int BACK_LEFT_TURN_MOTOR = 9;
    public static final int BACK_LEFT_ENCODER = 13;
    public static final double BACK_LEFT_ENCODER_OFFSET = Math.toRadians(0);
  }

  public static class RobotConstants {
    public static final double camHeight = 9.0; //replace w actual height
    public static final double camAngle = Units.degreesToRadians(69); //replace with actual angle of the camera
    public static final Transform3d robotToCam = new Transform3d(
      new Translation3d(0, 0, 0),
      new Rotation3d(0, camAngle, 0)
    );
  }

  public static class GameConstants {
    public static final double aprilTagHeight = 20; // CM
    public static final HashMap<Integer, Double[]> tagMap = new HashMap<Integer, Double[]>(8);
  }
  
  public static class AimbotConstants {
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double speed = 0.1;
    public static final double minimumAdjustment = 0.5;
  }
 
}

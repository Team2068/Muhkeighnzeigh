// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem; 

public final class Constants {
  public static final double DRIVE_MAX_VELOCITY_METERS_PER_SECOND = 0.2;
  public static final double DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.2;

  public static int CURRENT_LIMIT = 30;

  public enum ChassisConfiguration {
    MAIN,
    PRACTICE
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
    public static final int BACK_RIGHT_ENCODER =  13;
    public static double BACK_RIGHT_ENCODER_OFFSET;

    public static final int PIGEON_ID = 21;

    public static final void setOffsets() {
      if (Constants.getChassisConfiguration() == ChassisConfiguration.MAIN) {
        FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(359);
        FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(344);
        BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(315);
        BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(293);
      } else {
        FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(346);
        FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(148);
        BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(230);
        BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(138);
      }
    }
  }

  public static final class ArmConstants{
    public static final int ARM_1_MOTOR = 2;
    public static final int ARM_2_MOTOR = 3;
  }

  public static final class ClawConstants{
    public static final int CLAW_MOTOR = 17;
    public static final int INTAKE_MOTOR = 18;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.2;
    public static final double kMaxAngularSpeedRadiansPerSecond = 2*Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2*Math.PI;

    public static final double kPXController = 2;
    public static final double kPYController = 2;
    public static final double kPThetaController = 2.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(1, 1);
  }

  public static class Paths {
    public static HashMap<String, Command> eventMap = new HashMap<String, Command>();

    public static void initEventMap(){
      eventMap.put("Pickup", null); // TODO: replace null /w the command
      
      eventMap.put("Score Cube High", null);
      eventMap.put("Score Cone High", null);
      
      eventMap.put("Score Cube Mid", null);
      eventMap.put("Score Cone Mid", null);
    }

    // Testing
    public static final PathPlannerTrajectory bounce = PathPlanner.loadPath("Bounce", new PathConstraints(2, 0.75));
    public static final PathPlannerTrajectory funny = PathPlanner.loadPath("Funny", new PathConstraints(2, 2));
    public static final PathPlannerTrajectory loop = PathPlanner.loadPath("Loop", new PathConstraints(1, 0.75));

    // Scenario 1
    PathPlannerTrajectory Scenario14Cargo = PathPlanner.loadPath("(Scenario 1) 4 Cargo", new PathConstraints(4, 3));
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup(
    "(Scenario 1) 4 Cargo",
    new PathConstraints(4,3),
    new PathConstraints(4,3));

      //Scenario 2
      PathPlannerTrajectory Scenario21ConeCargo = PathPlanner.loadPath("(Scenario 2) 1 Cone Cargo", new PathConstraints(4, 3));
      List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup(
        "(Scenario 2) 1 Cone Cargo",
        new PathConstraints(4,3),
        new PathConstraints(4,3));
    
      //Scenario 3
      PathPlannerTrajectory Scenario31BlockCargo = PathPlanner.loadPath("(Scenario 3) 1 Block Cargo", new PathConstraints(4,3));
      List<PathPlannerTrajectory> pathGroup3 = PathPlanner.loadPathGroup(
      "(Scenario 3) 1 Block Cargo", 
      new PathConstraints(4,3),
      new PathConstraints(4,3));
      
      //Scenario 4
      PathPlannerTrajectory Scenario42ConeCargo = PathPlanner.loadPath("(Scenario 4) 2 Cone Cargo", new PathConstraints(4,3));
      List<PathPlannerTrajectory> pathGroup4 = PathPlanner.loadPathGroup(
      "(Scenario 4) 2 Cone Cargo",
      new PathConstraints(4,3),
      new PathConstraints(4,3));
  
      //Scenario 5
      PathPlannerTrajectory Scenario52BlockCargo = PathPlanner.loadPath("(Scenario 5) 2 Block Cargo", new PathConstraints(4,3));
      List<PathPlannerTrajectory> pathGroup5 = PathPlanner.loadPathGroup(
      "(Scenario 5) 2 Block Cargo",
      new PathConstraints(4,3),
      new PathConstraints(4,3));
  
  
      //Scenario 6
      PathPlannerTrajectory Scenario61Cand1BCargo = PathPlanner.loadPath("(Scenario 6) 1C and 1B Cargo", new PathConstraints(4,3));
      List<PathPlannerTrajectory> pathGroup6 = PathPlanner.loadPathGroup(
      "(Scenario 6) 1C and 1B Cargo",
      new PathConstraints(4,3),
      new PathConstraints(4,3));
  }

  public static class RobotConstants {
    public static final double camHeight = 0.1524; // Metres
    public static final double camAngle = Units.degreesToRadians(20); //replace with actual angle of the camera
    public static final Transform3d robotToCam = new Transform3d(
      new Translation3d(-3, 0.5, 5.5),
      new Rotation3d(0, camAngle, 0)
    );
    public static final String camName1 = "OV5647";
  }

  public static class GameConstants {
    public static final double[][] tagArray = {
      {1551.35, 107.16, 46.27, 180.0}, 
      {1551.35, 274.80, 46.27, 180.0}, 
      {1551.35, 442.44, 46.27, 180.0}, 
      {1617.87, 674.97, 69.54, 180.0}, 
      {36.19, 674.97, 69.54, 0.0}, 
      {102.743, 442.44, 46.27, 0.0}, 
      {102.743, 274.80, 46.27, 0.0}, 
      {102.743, 107.16, 46.27, 0.0}};
    public static final double aprilTagHeight = Units.inchesToMeters(17.5); // CM
    public static final double reflectiveTapeHeightLower = 0.6096; //meters
    public static final double reflectiveTapeHeightUpper = 1.0668; //meters
    }

  public static class AimbotConstants {
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double speed = 0.5;
    public static final double minimumAdjustment = 0.5;
  }
}

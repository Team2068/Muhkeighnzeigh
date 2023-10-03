// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import frc.robot.commands.Score;
import frc.robot.commands.SetClawPosition;
import frc.robot.commands.SetTelescopePosition;
import frc.robot.utilities.General;
import frc.robot.utilities.IO;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public final class Constants {
  public static final double DRIVE_MAX_VELOCITY_METERS_PER_SECOND = 0.2;
  public static final double DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.2;

  public static int CURRENT_LIMIT = 30;
  
  public static boolean isPracticeBot() {
    return System.getenv("PRACTICE_ROBOT") != null;
  }

  public static void initEventMap(IO io) {
    Paths.eventMap.put("pickup", new SequentialCommandGroup(
        General.Instant(io.claw::openClaw),
        new SetClawPosition(io.claw, ClawConstants.INTAKE_POSITION).withTimeout(1),
        General.Instant(io.claw::intake)));
    Paths.eventMap.put("closeclaw", General.Instant(io.claw::closeClaw));
    Paths.eventMap.put("openclaw", General.Instant(io.claw::openClaw));
    Paths.eventMap.put("stopintake", General.Instant(io.claw::stopClaw));
    Paths.eventMap.put("intakeposition",
        new SetClawPosition(io.claw, ClawConstants.INTAKE_POSITION).withTimeout(1));
    Paths.eventMap.put("liftarm", new SetClawPosition(io.claw, ClawConstants.CARRY_POSITION).withTimeout(1));
    Paths.eventMap.put("scorehigh", new Score(io, true)
        .andThen(new SetTelescopePosition(io.telescope, io.arm, 0)));
    Paths.eventMap.put("scorelow", new Score(io, false)
        .andThen(new SetTelescopePosition(io.telescope,io.arm, 0)));
  }

  public final static class ControllerConstants {
    public static final int RIGHT_TRIGGER = 3;
    public static final int LEFT_TRIGGER = 2;
    public static final double TRIGGER_ACTIVATION_THRESHOLD = .3;
    public static final int POV_ANGLE_UP = 0;
    public static final int POV_ANGLE_LEFT = 270;
    public static final int POV_ANGLE_RIGHT = 90;
  }
    public static final class DriveConstants {
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(19.5);
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(21.5);

    public static final int FRONT_LEFT_DRIVE_MOTOR = 6;
    public static final int FRONT_LEFT_TURN_MOTOR = 7;
    public static final int FRONT_LEFT_ENCODER = 17;
    public static double FRONT_LEFT_ENCODER_OFFSET;

    public static final int FRONT_RIGHT_DRIVE_MOTOR = 8;
    public static final int FRONT_RIGHT_TURN_MOTOR = 9;
    public static final int FRONT_RIGHT_ENCODER = 16;
    public static double FRONT_RIGHT_ENCODER_OFFSET;

    public static final int BACK_LEFT_DRIVE_MOTOR = 5;
    public static final int BACK_LEFT_TURN_MOTOR = 4;
    public static final int BACK_LEFT_ENCODER = 18;
    public static double BACK_LEFT_ENCODER_OFFSET;

    public static final int BACK_RIGHT_DRIVE_MOTOR = 10;
    public static final int BACK_RIGHT_TURN_MOTOR = 11;
    public static final int BACK_RIGHT_ENCODER = 15;
    public static double BACK_RIGHT_ENCODER_OFFSET;

    public static final int PIGEON_ID = 19;
  
    public static final void setOffsets() { 
      if (!isPracticeBot()) {
        FRONT_LEFT_ENCODER_OFFSET = -314;
        FRONT_RIGHT_ENCODER_OFFSET = -100;
        BACK_LEFT_ENCODER_OFFSET = -164;
        BACK_RIGHT_ENCODER_OFFSET = -53;
      } else {}

      SmartDashboard.putString("Robot Configuration", (isPracticeBot()) ? "Practice" : "Main");
    }
  }

  public static final class ArmConstants {
    public static final int ARM_1_MOTOR = 2;
    public static final int ARM_2_MOTOR = 3;
    
    public static double ARM_OFFSET;
    public static double ARM_LIMIT;

    public static void setOffsets() {
      if(!isPracticeBot()) {
        ARM_OFFSET = 0;
        ARM_LIMIT = 0.4;
      } else {}
    }
  }

  public static final class ClawConstants {
    public static final int WRIST_MOTOR = 13;
    public static final int INTAKE_MOTOR = 14;
    public static final double WRIST_VOLTAGE = 10;

    public static final double WRIST_OFFSET = -0.47;
    public static final double INTAKE_SPEED = .75;
    public static final double WRIST_SPEED = .5;

    public static final double INTAKE_POSITION = 75;
    public static final double FLAT_POSITION = 90;
    public static final double CARRY_POSITION = 138;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.2;
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 * Math.PI;

    public static final double kPXController = 2;
    public static final double kPYController = 2;
    public static final double kPThetaController = 2.5;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(1,1);
  }

  public static final class TelescopeConstants {
    public static final int TELESCOPE_MOTOR = 12;
    public static final double TELESCOPE_SPEED = 0.8;

    public static final double HIGH_POSITION = 90;
    public static final double LOW_POSITION = 10;
  }

  public static class Paths {
    public static final HashMap<String, Command> eventMap = new HashMap<String, Command>();
    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(4, 3);
    
    public static final PathPlannerTrajectory bounce = PathPlanner.loadPath("Bounce", PATH_CONSTRAINTS);
    public static final PathPlannerTrajectory funny = PathPlanner.loadPath("Funny", PATH_CONSTRAINTS);
    public static final PathPlannerTrajectory loop = PathPlanner.loadPath("Loop", PATH_CONSTRAINTS);
    public static final PathPlannerTrajectory park = PathPlanner.loadPath("(Scenario 7) Dock Only", PATH_CONSTRAINTS); 
    public static final PathPlannerTrajectory leaveCommunity = PathPlanner.loadPath("(Scenario 8) Exit Zone", new PathConstraints(2, 2));
    public static final List<PathPlannerTrajectory> cooking = PathPlanner.loadPathGroup("(Scenario 8) Exit Zone + Cooking", new PathConstraints(2, 1.5));
    public static final PathPlannerTrajectory leaveCommunityPark = PathPlanner.loadPath("(Scenario 10) Leave C and Park", PATH_CONSTRAINTS); 
    public static final List<PathPlannerTrajectory> picking = PathPlanner.loadPathGroup("(Scenario 0) pickup and go", PATH_CONSTRAINTS); 
    public static final List<PathPlannerTrajectory> flatSidePickup = PathPlanner.loadPathGroup("(Scenario 11) Flat Side Pickup One", PATH_CONSTRAINTS);
    public static final List<PathPlannerTrajectory> weBall = PathPlanner.loadPathGroup("we ball", PATH_CONSTRAINTS);
  }

  public static class PhotonConstants {
    public static final int REFLECTIVE_TAPE_PIPELINE_INDEX = 0;
    public static final int APRILTAG_PIPELINE_INDEX = 1;
    public static final double CAM_HEIGHT = 0.1524; 
    public static final double CAM_ANGLE = Units.degreesToRadians(20);
    public static final int SERVO_PORT = 6;
    public static final int FORWARD_ANGLE =  0;
    public static final int BACKWARD_ANGLE = 135;
  }

  public static class LEDConstants {
    public static final int LED_PORT = 0;
    public static final int LED_LENGTH = 3000;
    public static final Color YELLOW_LOW_POWER = new Color(0.2, 0.15, 0);
    public static final Color BLUE_LOW_POWER = new Color(0, 0, 0.25);
  }

  public static class GameConstants {
    public static final double[][] TAG_ARRAY = {
      {1551.35, 107.16, 46.27, 180.0}, 
      {1551.35, 274.80, 46.27, 180.0}, 
      {1551.35, 442.44, 46.27, 180.0}, 
      {1617.87, 674.97, 69.54, 180.0}, 
      {36.19, 674.97, 69.54, 0.0}, 
      {102.743, 442.44, 46.27, 0.0}, 
      {102.743, 274.80, 46.27, 0.0}, 
      {102.743, 107.16, 46.27, 0.0}};
    public static final double APRILTAG_HEIGHT = Units.inchesToMeters(17.5); // CM
    public static final double REFLTAPE_HEIGHT_LOWER = 0.6096; //meters
    public static final double REFLTAPE_HEIGHT_UPPER = 1.0668; //meters
  }

  public static class AimbotConstants {
    public static final double kP = 0.25;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double speed = 1;
    public static final double minimumAdjustment = 2.5;
  }
}

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

  public static ChassisConfiguration getChassisConfiguration() {
    return System.getenv("PRACTICE_ROBOT") != null ? ChassisConfiguration.PRACTICE : ChassisConfiguration.MAIN;
  }

  public static final class DriveConstants {
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(19.5);
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(21.5);

    public static final int FRONT_LEFT_DRIVE_MOTOR = 4;  // 8
    public static final int FRONT_LEFT_TURN_MOTOR = 5;   // 9
    public static final int FRONT_LEFT_ENCODER = 10;     // 2
    public static double FRONT_LEFT_ENCODER_OFFSET;

    public static final int FRONT_RIGHT_DRIVE_MOTOR = 6; // 10
    public static final int FRONT_RIGHT_TURN_MOTOR = 7;  // 11
    public static final int FRONT_RIGHT_ENCODER = 11;    // 3
    public static double FRONT_RIGHT_ENCODER_OFFSET;

    public static final int BACK_LEFT_DRIVE_MOTOR = 2;   // 12
    public static final int BACK_LEFT_TURN_MOTOR = 3;    // 13
    public static final int BACK_LEFT_ENCODER = 12;      // 4
    public static double BACK_LEFT_ENCODER_OFFSET;

    public static final int BACK_RIGHT_DRIVE_MOTOR = 8;  // 14
    public static final int BACK_RIGHT_TURN_MOTOR = 9;   // 15
    public static final int BACK_RIGHT_ENCODER =  13;    // 5
    public static double BACK_RIGHT_ENCODER_OFFSET;

    public static final void setOffsets() {
      if (Constants.getChassisConfiguration() == ChassisConfiguration.MAIN) {
        FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(359);
        FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(344);
        BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(315);
        BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(293);
      } else {
        FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(346);
        FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(68);
        BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(230);
        BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(84);
      }
    }
  }

  public static final class ArmConstants{
    public static final int ARM_1_MOTOR = 6;
    public static final int ARM_2_MOTOR = 7;
  }

  public static final class ClawConstants{
    public static final int CLAW_MOTOR = 16;
    public static final int INTAKE_MOTOR = 17;
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
    public static final PathPlannerTrajectory bounce = PathPlanner.loadPath("Bounce", new PathConstraints(1, 0.75));
    public static final PathPlannerTrajectory funny = PathPlanner.loadPath("Funny", new PathConstraints(2, 2));
    public static final PathPlannerTrajectory loop = PathPlanner.loadPath("Loop", new PathConstraints(1, 0.75));
  }

  public static class RobotConstants {
    public static final double camHeight = 0.1524; // Metres
    public static final double camAngle = Units.degreesToRadians(20); //replace with actual angle of the camera
    public static final Transform3d robotToCam = new Transform3d(
      new Translation3d(-3, 0.5, 5.5),
      new Rotation3d(0, camAngle, 0)
    );
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

  public static class Trajectories {
  // Scenario 1
  public static final PathPlannerTrajectory Step1_4Cargo = PathPlanner.loadPath("Step 1_4Cargo", 1, 1);
  public static final PathPlannerTrajectory Step2_4Cargo = PathPlanner.loadPath("Step 2_4Cargo", 1, 1);
  public static final PathPlannerTrajectory Step3_4Cargo = PathPlanner.loadPath("Step 3_4cargo", 1, 1);
  public static final PathPlannerTrajectory Step4_4Cargo = PathPlanner.loadPath("Step 4_4Cargo", 1, 1);
  public static final PathPlannerTrajectory Step5_4Cargo = PathPlanner.loadPath("Step 5_4Cargo", 1, 1);
  public static final PathPlannerTrajectory Step6_4Cargo = PathPlanner.loadPath("Step 6_4Cargo", 1, 1);
  public static final PathPlannerTrajectory Step7_4Cargo = PathPlanner.loadPath("Step 7_4Cargo", 1, 1);
  public static final PathPlannerTrajectory Step8_4Cargo = PathPlanner.loadPath("Step 8_4Cargo", 1, 1);
  public static final PathPlannerTrajectory Step9_4Cargo = PathPlanner.loadPath("Step 9_4Cargo", 1, 1);
  // Scenario 2
  public static final PathPlannerTrajectory Step1_2CargoCandB = PathPlanner.loadPath("Step 1_  2 Cargo C and B", 1, 1);
  public static final PathPlannerTrajectory Step2_2CargoCandB = PathPlanner.loadPath("Step 2_  2 Cargo C and B", 1, 1);
  public static final PathPlannerTrajectory Step3_2CargoCandB = PathPlanner.loadPath("Step 3_  2 Cargo C and B", 1, 1);
  public static final PathPlannerTrajectory Step4_2CargoCandB = PathPlanner.loadPath("Step 4_  2 Cargo C and B", 1, 1);
  public static final PathPlannerTrajectory Step5_2CargoCandB = PathPlanner.loadPath("Step 5_  2 Cargo C and B", 1, 1);
  // Scenario 3
  public static final PathPlannerTrajectory Step1_2BlockCargo = PathPlanner.loadPath("Step 1_ 2 Block Cargo", 1, 1);
  public static final PathPlannerTrajectory Step2_2BlockCargo = PathPlanner.loadPath("Step 2_ 2 Block Cargo", 1, 1);
  public static final PathPlannerTrajectory Step3_2BlockCargo = PathPlanner.loadPath("Step 3_ 2 Block Cargo", 1, 1);
  public static final PathPlannerTrajectory Step4_2BlockCargo = PathPlanner.loadPath("Step 4_ 2 Block Cargo", 1, 1);
  public static final PathPlannerTrajectory Step5_2BlockCargo = PathPlanner.loadPath("Step 5_ 2 Block Cargo", 1, 1);
  // Scenario 4
  public static final PathPlannerTrajectory Step1_2ConeCargo = PathPlanner.loadPath("Step 1_ 2 Cone Cargo", 1, 1);
  public static final PathPlannerTrajectory Step2_2ConeCargo = PathPlanner.loadPath("Step 2_ 2 Cone Cargo", 1, 1);
  public static final PathPlannerTrajectory Step3_2ConeCargo = PathPlanner.loadPath("Step 3_ 2 Cone Cargo", 1, 1);
  public static final PathPlannerTrajectory Step4_2ConeCargo = PathPlanner.loadPath("Step 4_ 2 Cone Cargo", 1, 1);
  public static final PathPlannerTrajectory Step5_2ConeCargo = PathPlanner.loadPath("Step 5_ 2 Cone Cargo", 1, 1);
  // Scenario 5
  public static final PathPlannerTrajectory Step1_1ConeCargo = PathPlanner.loadPath("Step 1_ 1 Cone Cargo", 1, 1);
  public static final PathPlannerTrajectory Step2_1ConeCargo = PathPlanner.loadPath("Step 2_ 1 Cone Cargo", 1, 1);
  public static final PathPlannerTrajectory Step3_1ConeCargo = PathPlanner.loadPath("Step 2_ 1 Cone Cargo", 1, 1);
  // Scenario 6
  public static final PathPlannerTrajectory Step1_1BlockCargo = PathPlanner.loadPath("Step 1_1 Block Cargo",1,1);
  public static final PathPlannerTrajectory Step2_1BlockCargo = PathPlanner.loadPath("Step 2_1 Block Cargo", 1,1);
  //Scenario 7 
  public static final PathPlannerTrajectory Step1_Park = PathPlanner.loadPath("Step 1_ Park", 1, 1);
}
}

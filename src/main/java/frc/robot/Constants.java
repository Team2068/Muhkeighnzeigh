// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
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
  public static final double DRIVE_MAX_VELOCITY_METERS_PER_SECOND = .5;
  public static final double DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = .5;
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(19.5);
  public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(21.5);
  public static int CURRENT_LIMIT = 30;

  public static final class DriveConstants {

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
    public static final double FRONT_LEFT_ENCODER_OFFSET = Math.toRadians(0);

    public static final int BACK_LEFT_DRIVE_MOTOR = 8;
    public static final int BACK_LEFT_TURN_MOTOR = 9;
    public static final int BACK_LEFT_ENCODER = 13;
    public static final double BACK_LEFT_ENCODER_OFFSET = Math.toRadians(0);

  }

  public static class Trajectories {
    // Scenario 1
  public static final Trajectory Step1_4Cargo = PathPlanner.loadPath("Step 1_4Cargo", 1, 1);
  public static final Trajectory Step2_4Cargo = PathPlanner.loadPath("Step 2_4Cargo", 1, 1);
  public static final Trajectory Step3_4Cargo = PathPlanner.loadPath("Step 3_4cargo", 1, 1);
  public static final Trajectory Step4_4Cargo = PathPlanner.loadPath("Step 4_4Cargo", 1, 1);
  public static final Trajectory Step5_4Cargo = PathPlanner.loadPath("Step 5_4Cargo", 1, 1);
  public static final Trajectory Step6_4Cargo = PathPlanner.loadPath("Step 6_4Cargo", 1, 1);
  public static final Trajectory Step7_4Cargo = PathPlanner.loadPath("Step 7_4Cargo", 1, 1);
  public static final Trajectory Step8_4Cargo = PathPlanner.loadPath("Step 8_4Cargo", 1, 1);
  public static final Trajectory Step9_4Cargo = PathPlanner.loadPath("Step 9_4Cargo", 1, 1);
  // Scenario 2
  public static final Trajectory Step1_2CargoCandB = PathPlanner.loadPath("Step 1_  2 Cargo C and B", 1, 1);
  public static final Trajectory Step2_2CargoCandB = PathPlanner.loadPath("Step 2_  2 Cargo C and B", 1, 1);
  public static final Trajectory Step3_2CargoCandB = PathPlanner.loadPath("Step 3_  2 Cargo C and B", 1, 1);
  public static final Trajectory Step4_2CargoCandB = PathPlanner.loadPath("Step 4_  2 Cargo C and B", 1, 1);
  public static final Trajectory Step5_2CargoCandB = PathPlanner.loadPath("Step 5_  2 Cargo C and B", 1, 1);
  // Scenario 3
  public static final Trajectory Step1_2BlockCargo = PathPlanner.loadPath("Step 1_ 2 Block Cargo", 1, 1);
  public static final Trajectory Step2_2BlockCargo = PathPlanner.loadPath("Step 2_ 2 Block Cargo", 1, 1);
  public static final Trajectory Step3_2BlockCargo = PathPlanner.loadPath("Step 3_ 2 Block Cargo", 1, 1);
  public static final Trajectory Step4_2BlockCargo = PathPlanner.loadPath("Step 4_ 2 Block Cargo", 1, 1);
  public static final Trajectory Step5_2BlockCargo = PathPlanner.loadPath("Step 5_ 2 Block Cargo", 1, 1);
  // Scenario 4
  public static final Trajectory Step1_2ConeCargo = PathPlanner.loadPath("Step 1_ 2 Cone Cargo", 1, 1);
  public static final Trajectory Step2_2ConeCargo = PathPlanner.loadPath("Step 2_ 2 Cone Cargo", 1, 1);
  public static final Trajectory Step3_2ConeCargo = PathPlanner.loadPath("Step 3_ 2 Cone Cargo", 1, 1);
  public static final Trajectory Step4_2ConeCargo = PathPlanner.loadPath("Step 4_ 2 Cone Cargo", 1, 1);
  public static final Trajectory Step5_2ConeCargo = PathPlanner.loadPath("Step 5_ 2 Cone Cargo", 1, 1);
  // Scenario 5
  public static final Trajectory Step1_1ConeCargo = PathPlanner.loadPath("Step 1_ 1 Cone Cargo", 1, 1);
  public static final Trajectory Step2_1ConeCargo = PathPlanner.loadPath("Step 2_ 1 Cone Cargo", 1, 1);
  public static final Trajectory Step3_1ConeCargo = PathPlanner.loadPath("Step 2_ 1 Cone Cargo", 1, 1);
  }
}

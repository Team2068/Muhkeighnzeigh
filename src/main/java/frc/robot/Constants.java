// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand; 

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

  public static final class DriveConstants {
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(19.5);
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(21.5);

    public static final int FRONT_LEFT_DRIVE_MOTOR = 4;
    public static final int FRONT_LEFT_TURN_MOTOR = 5;
    public static final int FRONT_LEFT_ENCODER = 10;
    public static final double FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(346);

    public static final int FRONT_RIGHT_DRIVE_MOTOR = 6;
    public static final int FRONT_RIGHT_TURN_MOTOR = 7;
    public static final int FRONT_RIGHT_ENCODER = 11;
    public static final double FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(68);

    public static final int BACK_LEFT_DRIVE_MOTOR = 2;
    public static final int BACK_LEFT_TURN_MOTOR = 3;
    public static final int BACK_LEFT_ENCODER = 12;
    public static final double BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(223);

    public static final int BACK_RIGHT_DRIVE_MOTOR = 8;
    public static final int BACK_RIGHT_TURN_MOTOR = 9;
    public static final int BACK_RIGHT_ENCODER = 13;
    public static final double BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(48);
  }

  public static final class AutoConstants {
    public static final double MAX_Speed_MetersPerSecond = 0.2;
    public static final double MAX_Acceleration_MetersPerSecondSquared = 0.2;
    public static final double MAX_AngularSpeed_RadiansPerSecond = Math.PI;
    public static final double Max_AngularSpeed_RadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 4;
    public static final double kPYController = 4;
    public static final double kPThetaController = 2;

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

    // Scenario 1
    PathPlannerTrajectory Scenario14Cargo = PathPlanner.loadPath("(Scenario 1) 4 Cargo", new PathConstraints(4, 3));
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup(
    "(Scenario 1) 4 Cargo",
    new PathConstraints(4,3),
    new PathConstraints(4,3));
  }

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




    


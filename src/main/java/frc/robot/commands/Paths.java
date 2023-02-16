package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Paths extends CommandBase {

    private final Timer m_timer = new Timer();
    private final Trajectory m_trajectory;
    private final Pose2d m_pose;
    private final SwerveDriveKinematics m_kinematics;
    private final HolonomicDriveController m_controller;
    private final DriveSubsystem m_drivetrain;

    @SuppressWarnings("ParameterName")
    public Paths(Trajectory trajectory, DriveSubsystem drivetrainSubsystem){
        this.m_drivetrain = drivetrainSubsystem;
        this.m_trajectory = trajectory;
        this.m_kinematics = drivetrainSubsystem.getKinematics();
        this.m_pose = drivetrainSubsystem.getPose();

        ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_controller = new HolonomicDriveController(
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController
        );
        
        addRequirements(m_drivetrain);
    }
    @Override
    public void initialize() {
        m_drivetrain.resetOdometry(m_trajectory.getInitialPose());
        m_timer.reset();
        m_timer.start();
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        double curTime = m_timer.get();
        var desiredState = (PathPlannerState) m_trajectory.sample(curTime);

        var targetChassisSpeeds = m_controller.calculate(m_drivetrain.getPose(), desiredState, desiredState.holonomicRotation);
        // var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);
        DriverStation.reportWarning("TARGET: " + desiredState.poseMeters.toString() + "\n ACTUAL: " + m_pose.toString(), false);

        m_drivetrain.drive(targetChassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}



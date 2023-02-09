package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper.GearRatio;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    public static double MAX_VOLTAGE = 5;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 3;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = (MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2,
                    DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2));

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

    public final WPI_Pigeon2 pigeon2 = new WPI_Pigeon2(15);

    private final SwerveDriveOdometry odometry;
    private final SwerveModule frontLeftModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backRightModule;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

    private boolean fieldOriented = false;
    private Pose2d mPose;

    public DriveSubsystem() {
        DriveConstants.setOffsets();
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        frontLeftModule = Mk4SwerveModuleHelper.createNeo(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                GearRatio.L2,
                DriveConstants.FRONT_LEFT_DRIVE_MOTOR,
                DriveConstants.FRONT_LEFT_TURN_MOTOR,
                DriveConstants.FRONT_LEFT_ENCODER,
                DriveConstants.FRONT_LEFT_ENCODER_OFFSET);
        frontRightModule = Mk4SwerveModuleHelper.createNeo(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                GearRatio.L2,
                DriveConstants.FRONT_RIGHT_DRIVE_MOTOR,
                DriveConstants.FRONT_RIGHT_TURN_MOTOR,
                DriveConstants.FRONT_RIGHT_ENCODER,
                DriveConstants.FRONT_RIGHT_ENCODER_OFFSET);
        backLeftModule = Mk4SwerveModuleHelper.createNeo(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                GearRatio.L2,
                DriveConstants.BACK_LEFT_DRIVE_MOTOR,
                DriveConstants.BACK_LEFT_TURN_MOTOR,
                DriveConstants.BACK_LEFT_ENCODER,
                DriveConstants.BACK_LEFT_ENCODER_OFFSET);
        backRightModule = Mk4SwerveModuleHelper.createNeo(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                GearRatio.L2,
                DriveConstants.BACK_RIGHT_DRIVE_MOTOR,
                DriveConstants.BACK_RIGHT_TURN_MOTOR,
                DriveConstants.BACK_RIGHT_ENCODER,
                DriveConstants.BACK_RIGHT_ENCODER_OFFSET);

        odometry = new SwerveDriveOdometry(
                kinematics, getGyroscopeRotation(), getModulePositions(), new Pose2d(0, 0, new Rotation2d()));

        // pigeon2.configMountPose(AxisDirection.PositiveY, AxisDirection.NegativeZ);
        pigeon2.configMountPose(AxisDirection.PositiveX, AxisDirection.NegativeZ);
        pigeon2.calibrate();
        zeroGyro();
    }

    public void resetPosition() {
        frontLeftModule.resetDrivePosition();
        frontRightModule.resetDrivePosition();
        backLeftModule.resetDrivePosition();
        backRightModule.resetDrivePosition();
    }

    public void zeroGyro() {
        pigeon2.setYaw(0);
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public Rotation2d getGyroscopeRotation() {
        // double rot = Math.abs(pigeon2.getYaw()) % 360 * ( (pigeon2.getYaw() < 0) ? -1 : 1);
        // if (rot<0) rot += 360;
        // return Rotation2d.fromDegrees(rot);
        return pigeon2.getRotation2d();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    private SwerveModulePosition getModulePosition(SwerveModule module) {
        return new SwerveModulePosition(module.getDrivePosition(), Rotation2d.fromRadians(module.getSteerAngle()));
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] pos = {
                getModulePosition(frontLeftModule),
                getModulePosition(frontRightModule),
                getModulePosition(backLeftModule),
                getModulePosition(backRightModule)
        };
        SmartDashboard.putNumber("FL Distance", pos[0].distanceMeters);
        SmartDashboard.putNumber("FR Distance", pos[1].distanceMeters);
        SmartDashboard.putNumber("BL Distance", pos[2].distanceMeters);
        SmartDashboard.putNumber("BR Distance", pos[3].distanceMeters);
        return pos;
    }

    public Pose2d getPose() {
        return mPose;
    }

    public void resetOdometry() {
        zeroGyro();
        resetPosition();
        odometry.resetPosition(getGyroscopeRotation(), getModulePositions(), new Pose2d());
    }

    public void resetOdometry(Pose2d pose) {
        zeroGyro();
        resetPosition();
        odometry.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        frontLeftModule.set((states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND) * MAX_VOLTAGE,
                states[0].angle.getRadians());
        frontRightModule.set((states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND) * MAX_VOLTAGE,
                states[1].angle.getRadians());
        backLeftModule.set((states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND) * MAX_VOLTAGE,
                states[2].angle.getRadians());
        backRightModule.set((states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND) * MAX_VOLTAGE,
                states[3].angle.getRadians());
    }

    public boolean isFieldOriented() {
        return fieldOriented;
    }

    public void toggleFieldOriented() {
        fieldOriented = !fieldOriented;
    }

    public void maxSpeed() {
        MAX_VOLTAGE = 12;

    }

    public void normalSpeed() {
        MAX_VOLTAGE = 9;
    }

    public void minSpeed() {
        MAX_VOLTAGE = 5;
    }

    public void resetSteerPositions() {
        frontLeftModule.set(0, 0);
        frontRightModule.set(0, 0);
        backLeftModule.set(0, 0);
        backRightModule.set(0, 0);
    }

    public Command followPath(PathPlannerTrajectory path){
        return new SequentialCommandGroup(
            new InstantCommand(this::resetOdometry)
            .andThen(() -> resetOdometry(path.getInitialHolonomicPose())),
            new PPSwerveControllerCommand(path, this::getPose,
            new PIDController(AutoConstants.kPXController, 0, 0.01),
            new PIDController(AutoConstants.kPYController, 0, 0.01),
            new PIDController(AutoConstants.kPThetaController, 0, 0.01),
            this::drive,
            this)
        );
    }

    public void periodic() {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(states);
        mPose = odometry.update(getGyroscopeRotation(), getModulePositions());
        // Pose2d pose = getPose();
        SmartDashboard.putData(pigeon2);
        SmartDashboard.putNumber("X position", mPose.getX());
        SmartDashboard.putNumber("Y position", mPose.getY());

        double[] ypr = new double[3];
        pigeon2.getYawPitchRoll(ypr);
        SmartDashboard.putNumber("ypr rotation", ypr[0]);
        SmartDashboard.putNumber("Odometry rotation", getGyroscopeRotation().getDegrees());
        SmartDashboard.putNumber("Pigeon rotation", pigeon2.getYaw());
        SmartDashboard.putNumber("Pigeon Pitch", pigeon2.getPitch());
        SmartDashboard.putNumber("Pigeon Roll", pigeon2.getRoll());

        SmartDashboard.putString("Drive Mode", fieldOriented ? "Field" : "Robot");
    }

    public ChassisSpeeds getChassisSpeed() {
        return chassisSpeeds;
    }

}

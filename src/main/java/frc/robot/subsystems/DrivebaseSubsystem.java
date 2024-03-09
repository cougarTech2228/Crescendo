package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class DrivebaseSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final double DRIVEBASE_RADIUS_METERS = 0.45085;
    private static final double STATOR_CURRENT_LIMIT = 70.0; // Amps

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private static DrivebaseSubsystem mInstance = null;

    private final static Vector<N3> odometryStandardDeviation = VecBuilder.fill(0.05, 0.05, 0.01);
    private final static Vector<N3> visionStandardDeviation = VecBuilder.fill(0.3, 0.3, 99);

    public static DrivebaseSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new DrivebaseSubsystem(
                    TunerConstants.DrivetrainConstants,
                    TunerConstants.FrontLeft,
                    TunerConstants.FrontRight,
                    TunerConstants.BackLeft,
                    TunerConstants.BackRight);
        }
        return mInstance;
    }

    private DrivebaseSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, 0, odometryStandardDeviation, visionStandardDeviation, modules);
        /*
         * SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants... modules
         */
        configurePathPlanner();

        // Apply current limits to the motors to smooth out the accelleration and
        // braking
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);

        // apply the limits to all 4 drive motors
        getModule(0).getDriveMotor().getConfigurator().apply(limits);
        getModule(1).getDriveMotor().getConfigurator().apply(limits);
        getModule(2).getDriveMotor().getConfigurator().apply(limits);
        getModule(3).getDriveMotor().getConfigurator().apply(limits);

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Pose2d getCurrentPose() {
        return this.getState().Pose;
    }

    private HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(3.4202, 0, 0),
            new PIDConstants(5.1, 0, 0.0),
            TunerConstants.kSpeedAt12VoltsMps,
            DRIVEBASE_RADIUS_METERS,
            new ReplanningConfig(true, true),
            0.004);

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
                pathFollowerConfig,
                this::getShouldFlipPath,
                this); // Subsystem for requirements
    }

    public boolean getShouldFlipPath() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public void setCurrentRobotChassisSpeeds(ChassisSpeeds speeds) {
        setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds));
    }

    public Command getFollowPathCommand(PathPlannerPath path, boolean autoFlip) {
        return new FollowPathHolonomic(
                path,
                () -> this.getState().Pose, // Robot pose supplier
                this::getCurrentRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setCurrentRobotChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE
                                                    // ChassisSpeeds
                pathFollowerConfig,
                () -> {
                    if (autoFlip == false) {
                        return false;
                    }
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    private double m_lastSimTime;
    private Notifier m_simNotifier = null;
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}

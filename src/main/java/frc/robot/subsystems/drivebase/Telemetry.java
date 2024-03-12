package frc.robot.subsystems.drivebase;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Telemetry {
    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(Supplier<Pigeon2> pigeonSupplier) {
        this.pigeonSupplier = pigeonSupplier;
    }

    private final Supplier<Pigeon2> pigeonSupplier;

    /* Keep a reference of the last pose to calculate the speeds */
    private Pose2d m_lastPose = new Pose2d();
    private double lastTime = Utils.getCurrentTimeSeconds();

    /* Accept the swerve drive state and telemeterize it to smartdashboard */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the pose */
        Pose2d pose = state.Pose;

        /* Telemeterize the robot's general speeds */
        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;
        Translation2d distanceDiff = pose.minus(m_lastPose).getTranslation();
        m_lastPose = pose;

        Translation2d velocities = distanceDiff.div(diffTime);

        Logger.recordOutput("Telemetry/Pose", pose);
        Logger.recordOutput("Telemetry/Speed", velocities.getNorm());
        Logger.recordOutput("Telemetry/VelocityX", velocities.getX());
        Logger.recordOutput("Telemetry/VelocityY", velocities.getY());
        Logger.recordOutput("Telemetry/OdometryPeriod", state.OdometryPeriod);

        Logger.recordOutput("Telemetry/Pigeon", pigeonSupplier.get().getRotation3d());
        Logger.recordOutput("Swerve/ModuleStates", state.ModuleStates);
        Logger.recordOutput("Swerve/ModuleTargets", state.ModuleTargets);
    }
}
package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        /** true if a note is at the front of the intake */
        public boolean isNoteAtBottom = false;

        /** true if a note is currently seen between the ground
            feeder, and the main chamber */
        public boolean isNoteAtMiddle = false;

        /** true if a note is currently seen between the main chamber and the
            bender */
        public boolean isNoteAtTop = false;

        public double groundFeedVelocity = 0.0;
        public double groundFeedAppliedVolts = 0.0;
        public double groundFeedCurrentAmps = 0.0;

        public double beltMotorVelocity = 0.0;
        public double beltMotorAppliedVolts = 0.0;
        public double beltMotorCurrentAmps = 0.0;

        public double flywheelMotorVelocity = 0.0;
        public double flywheelMotorAppliedVolts = 0.0;
        public double flywheelMotorCurrentAmps = 0.0;

        public double feedMotorVelocity = 0.0;
        public double feedMotorAppliedVolts = 0.0;
        public double feedMotorCurrentAmps = 0.0;

        public double benderFeedMotorAppliedVolts = 0.0;
        public double benderFeedMotorCurrentAmps = 0.0;
    }

    public enum WhichMotor {
        GROUND_FEED,
        BELT,
        FEED_WHEEL,
        FLYWHEEL,
        BENDER_FEED
    }
    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {
    }

    /** Run the acquirer motor at the specified voltage. */
    public default void setGroundFeedMotorVoltage(double volts) {
    }

    /** Run the belt motor at the specified voltage. */
    public default void setBeltMotorVoltage(double volts) {
    }

    /** Run the flywheel motor at the specified voltage. */
    public default void setFlywheelMotorVoltage(double volts) {
    }

    /** Run the feed wheel motor at the specified voltage. */
    public default void setShooterFeedMotorVoltage(double volts) {
    }

    /** Run the feed wheel motor at the specified voltage. */
    public default void setBenderFeedMotorVoltage(double volts) {
    }

    public default void setBrake(WhichMotor motor) {
    }

    public default void setCoast(WhichMotor motor) {
    }
}

package frc.robot;

public class Constants {

    public static final Mode currentMode = Mode.REAL;

    public static enum Mode {
      /** Running on a real robot. */
      REAL,
  
      /** Running a physics simulator. */
      SIM,
  
      /** Replaying from a log file. */
      REPLAY
    }

    // Swerve constants are in generated/TunerConstants as they were created via the
    // TunerX app

    // Shooter
    public static final int kShooterFeedMotorId = 21; // Falcon 500
    public static final String kShooterFeedMotorBus = "rio";

    public static final int kShooterFlywheelMotorId = 20; // Falcon 500
    public static final String kShooterFlywheelMotorBus = "rio";

    public static final int kShooterBeltMotorId = 22; // Spark Max
    public static final String kShooterBeltMotorBus = "rio";


    public static final int kClimberMotorId = 10; // Falcon
    public static final String kClimberMotorBus = "canivore";

    public static final int kElevatorMotorId = 11; // Falcon
    public static final String kElevatorMotorBus = "rio";

    public static final int kGroundFeedMotorId = 9; // Spark Max
    public static final String kGroundFeedMotorBus = "rio";

    public static final int kBenderTiltMotorId = 7; // Talon SRX
    public static final String kBenderTiltMotorBus = "rio";

    public static final int kBenderFeedMotorId = 15; // Talon SRX
    public static final String kBenderFeedMotorBus = "rio";

    public static final int kLinearActuatorMotorId = 14; // Talon SRX
      public static final String kLinearActuatorMotorBus = "rio";

    // Sensors
    public static final int kGroundFeedSensorId = 0; // Is suppposed to be 0 don't change it
    public static final int kClimberBottomSensorId = 1;
    public static final int kClimberTopSensorId = 2;
    public static final int kElevatorBottomSensorId = 3;
    public static final int kElevatorTopSensorId = 4;
    public static final int kShooterAngleEncoderId = 5;
    public static final int kMiddleFeedSensorId = 6;
    public static final int kTopFeedSensorId = 7;
    public static final int kBenderAngleEncoderPin = 8;
    public static final int kShooterUpperLimitSwitchID = 9;

    // PWM
    public static final int kDriverCameraPWMID = 0;
}

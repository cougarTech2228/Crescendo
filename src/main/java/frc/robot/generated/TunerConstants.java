package frc.robot.generated;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;

public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);
        
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.070292).withKI(0).withKD(0)
        .withKS(0.068222).withKV(2.3048).withKA(0.051634);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 70.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 4.78;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 6.746031746031747;
    private static final double kSteerGearRatio = 21.428571428571427;
    private static final double kWheelRadiusInches = 1.96;

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    public static final String kCANbusName = "canivore";
    public static final int kPigeonId = 61;


    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


    // Front Left
    private static final int kFrontLeftDriveMotorId = 47;
    private static final int kFrontLeftSteerMotorId = 48;
    private static final int kFrontLeftEncoderId = 49;
    private static final double kFrontLeftEncoderOffset = 0.29833984375;

    private static final double kFrontLeftXPosInches = 11.5;
    private static final double kFrontLeftYPosInches = 13.375;

    // Front Right
    private static final int kFrontRightDriveMotorId = 56;
    private static final int kFrontRightSteerMotorId = 57;
    private static final int kFrontRightEncoderId = 58;
    private static final double kFrontRightEncoderOffset = 0.218017578125;

    private static final double kFrontRightXPosInches = 11.5;
    private static final double kFrontRightYPosInches = -13.375;

    // Back Left
    private static final int kBackLeftDriveMotorId = 50;
    private static final int kBackLeftSteerMotorId = 51;
    private static final int kBackLeftEncoderId = 52;
    private static final double kBackLeftEncoderOffset = 0.115966796875;

    private static final double kBackLeftXPosInches = -11.5;
    private static final double kBackLeftYPosInches = 13.375;

    // Back Right
    private static final int kBackRightDriveMotorId = 53;
    private static final int kBackRightSteerMotorId = 54;
    private static final int kBackRightEncoderId = 55;
    private static final double kBackRightEncoderOffset = -0.334228515625;

    private static final double kBackRightXPosInches = -11.5;
    private static final double kBackRightYPosInches = -13.375;


    public static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    public static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);
}

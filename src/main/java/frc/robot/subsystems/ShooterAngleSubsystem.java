package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShooterAngleSubsystem extends PIDSubsystem {

    private DrivebaseSubsystem mDrivebaseSubsystem = DrivebaseSubsystem.getInstance();
    private AprilTagSubsystem mAprilTagSubsystem = AprilTagSubsystem.getInstance();
    private ShuffleboardTab m_sbTab;
    private TalonSRX mLinearActuatorMotor;
    private DutyCycleEncoder mShooterAngleEncoder;
    private double mDistanceToSpeaker = 0;
    private double mAutoAngle = 0;
    private static final double SPEED_UP = -1;
    private static final double SPEED_DOWN = 1;

    private static final double kMotorVoltageLimit = 1;
    private double m_shooterAngle;
    private double mPidOutput = 0;

    private double mGoal = 0;

    /** angle where shooter is able to shoot at the speaker */
    private final static double SHOOT_SPEAKER_SIDE_ANGLE = 393;
    private final static double SHOOT_SPEAKER_FRONT_ANGLE = 375;

    /** angle where shooter is able to shoot at the amp */
    private final static double SHOOT_AMP_ANGLE = 378;

    /** angle where shooter is positioned so we can go under the chain */
    private final static double UNDER_CHAIN_ANGLE = 411;

    /** angle where shooter is in the correct location to load from source */
    private final static double LOAD_SOURCE_HEIGHT = 378;

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.01;
    private static final double kDt = 0.01;
    private static final PIDController pidController = new PIDController(kP, kI, kD, kDt);
    private static final double SHOOTER_ANGLE_THRESHOLD = 1;

    public enum ShooterPosition {
        SHOOT_SPEAKER_SIDE,
        SHOOT_SPEAKER_FRONT,
        SHOOT_AMP,
        LOAD_SOURCE,
        HEIGHT_CHAIN,
        SHOOT_PID
    };

    enum State {
        STOPPED,
        RAISING,
        LOWERING,
    }

    private State mCurrentState = State.STOPPED;

    private ShooterPosition m_currentTargetPosition = ShooterPosition.SHOOT_SPEAKER_FRONT;

    private static ShooterAngleSubsystem mInstance = null;

    public static ShooterAngleSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ShooterAngleSubsystem();
        }
        return mInstance;
    }

    private ShooterAngleSubsystem() {
        super(pidController, 0);

        pidController.setTolerance(SHOOTER_ANGLE_THRESHOLD);
        // pidController.setIZone(kIZone);
        // pidController.setIntegratorRange(ANGLE_MIN, ANGLE_MAX);

        mLinearActuatorMotor = new TalonSRX(Constants.kLinearActuatorMotorId);
        mShooterAngleEncoder = new DutyCycleEncoder(Constants.kShooterAngleEncoderId);
        mLinearActuatorMotor.setNeutralMode(NeutralMode.Brake);

        if (Robot.isDebug) {
            m_sbTab = Shuffleboard.getTab("Shooter Angle (Debug)");

            m_sbTab.addDouble("Current Angle:", new DoubleSupplier() {
                @Override
                public double getAsDouble() {
                    return m_shooterAngle;
                };
            });

            m_sbTab.addDouble("PID goal", new DoubleSupplier() {
                @Override
                public double getAsDouble() {
                    return m_controller.getSetpoint();
                };
            });

            m_sbTab.addDouble("PID calculated output", new DoubleSupplier() {
                @Override
                public double getAsDouble() {
                    return mPidOutput;
                };
            });

            m_sbTab.addDouble("PID Motor output", new DoubleSupplier() {
                @Override
                public double getAsDouble() {
                    return mLinearActuatorMotor.getMotorOutputVoltage();
                };
            });

            m_sbTab.addString("Linear Actuator state", new Supplier<String>() {
                @Override
                public String get() {
                    return mCurrentState.name();
                }
            });

            m_sbTab.addDouble("Distance To speaker", new DoubleSupplier() {
                @Override
                public double getAsDouble() {
                    return mDistanceToSpeaker;
                };
            });

            m_sbTab.addDouble("Auto Angle", new DoubleSupplier() {
                @Override
                public double getAsDouble() {
                    return mAutoAngle;
                };
            });
        }

        new Thread("shooterAngleEncoder") {
            public void run() {
                while (true) {
                    m_shooterAngle = mShooterAngleEncoder.getAbsolutePosition() * 1000;
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            };
        }.start();
    }

    private boolean isDisabled = DriverStation.isDisabled();

    @Override
    public void periodic() {
        super.periodic();

        // only send these commands on change
        if (DriverStation.isDisabled() != isDisabled) {
            isDisabled = DriverStation.isDisabled();
            if (isDisabled) {
                mLinearActuatorMotor.setNeutralMode(NeutralMode.Coast);
            } else {
                mLinearActuatorMotor.setNeutralMode(NeutralMode.Brake);
            }
        }

        if (isDisabled) {
            disable();
            stopMotor();
            return;
        }

        Pose2d currentPose = mDrivebaseSubsystem.getCurrentPose();
        Pose2d speakerPose = mAprilTagSubsystem.aprilTagFieldLayout.getTagPose(7).get().toPose2d();

        mDistanceToSpeaker = currentPose.getTranslation().getDistance(speakerPose.getTranslation());
        
        //y = -65.034x3 + 371.79x2 - 649.68x + 733.47

        mAutoAngle  = (-65.034 * (mDistanceToSpeaker * mDistanceToSpeaker * mDistanceToSpeaker)) +
            (371.79 * (mDistanceToSpeaker * mDistanceToSpeaker)) -
            (649.68 * mDistanceToSpeaker) +
            733.47;

        // pidController.setSetpoint(mAutoAngle);
        // enable();
        
        // if ((m_shooterAngle < SHOOTER_ANGLE_MIN_LIMIT) && (mCurrentState == State.RAISING)) {
        //     mCurrentState = State.STOPPED;
        //     stopMotor();
        // }
        // if ((m_shooterAngle > SHOOTER_ANGLE_MAX_LIMIT) && (mCurrentState == State.LOWERING)) {
        //     mCurrentState = State.STOPPED;
        //     stopMotor();
        // }
    }

    private boolean atGoal() {
        return pidController.atSetpoint();
    }

    public void stopMotor() {
        // Stops shooter motor
        disable();
        mLinearActuatorMotor.set(TalonSRXControlMode.PercentOutput, 0);
        // System.out.println("stopped");
    }

    public void raiseShooter() {
        // Moves the shooter
        System.out.println("called raise shooter");
        disable();
        mLinearActuatorMotor.set(TalonSRXControlMode.PercentOutput, SPEED_UP);
        System.out.println("raising");
    }

    public void lowerShooter() {
        // Moves the shooter down
        System.out.println("called lower shooter");
        disable();
        mLinearActuatorMotor.set(TalonSRXControlMode.PercentOutput, SPEED_DOWN);
        System.out.println("lowering");
    }

    public boolean isInSpeakerLocation_front() {
        return m_currentTargetPosition == ShooterPosition.SHOOT_SPEAKER_FRONT && atGoal();
    }

    public boolean isInSpeakerLocation_side() {
        return m_currentTargetPosition == ShooterPosition.SHOOT_SPEAKER_SIDE && atGoal();
    }

    public boolean isInAmpLocation() {
        return m_currentTargetPosition == ShooterPosition.SHOOT_AMP && atGoal();
    }

    public boolean isInChainLocation() {
        return m_currentTargetPosition == ShooterPosition.HEIGHT_CHAIN && atGoal();
    }

    public boolean isInSourceLocation() {
        return m_currentTargetPosition == ShooterPosition.LOAD_SOURCE && atGoal();
    }

    public void setPosition(ShooterPosition position) {
        m_currentTargetPosition = position;
        switch (m_currentTargetPosition) {
            case SHOOT_SPEAKER_FRONT:
                mGoal = SHOOT_SPEAKER_FRONT_ANGLE;
                break;
            case SHOOT_SPEAKER_SIDE:
                mGoal = SHOOT_SPEAKER_SIDE_ANGLE;
                break;
            case HEIGHT_CHAIN:
                mGoal = UNDER_CHAIN_ANGLE;
                break;
            case LOAD_SOURCE:
                mGoal = LOAD_SOURCE_HEIGHT;
                break;
            case SHOOT_AMP:
                mGoal = SHOOT_AMP_ANGLE;
                break;
            case SHOOT_PID:
                mGoal = mAutoAngle;
                break;
        }

        if (mGoal > m_shooterAngle) {
            mCurrentState = State.RAISING;
        } else if (mGoal < m_shooterAngle) {
            mCurrentState = State.LOWERING;
        } else {
            mCurrentState = State.STOPPED;
        }

        System.out.println("setting shooter angle: " + mGoal);
        this.m_controller.reset();
        pidController.setSetpoint(mGoal);
        enable();
    }

    @Override
    protected double getMeasurement() {
        return m_shooterAngle;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        // clamp the output to a sane range
        mPidOutput = output;
        double val;
        if (output < 0) {
            val = Math.max(-kMotorVoltageLimit, output);
        } else {
            val = Math.min(kMotorVoltageLimit, output);
        }
        mLinearActuatorMotor.set(TalonSRXControlMode.PercentOutput, val);
    }
}
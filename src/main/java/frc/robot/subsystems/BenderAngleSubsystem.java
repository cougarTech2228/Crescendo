package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;

public class BenderAngleSubsystem extends PIDSubsystem {

    public enum BenderPosition {
        SHOOT_SPEAKER,
        SHOOT_AMP,
        LOAD_INTERNAL,
        LOAD_SOURCE,
        PREP_TRAP,
        SHOOT_TRAP
    };

    private ShuffleboardTab m_sbTab;
    private TalonSRX mBenderTiltMotor;
    private BenderState m_benderState = BenderState.stopped;
    private DutyCycleEncoder m_benderAngleEncoder;

    private double m_benderAngle;
    private BenderPosition m_currentTargetPosition = BenderPosition.SHOOT_SPEAKER;

    private static final double kP = 0.08;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kDt = 0.02;

    private static final double kMotorVoltageLimit = 0.5;

    private static final double kIZone = 5;

    private static final double ANGLE_MIN = 5;
    private static final double ANGLE_MAX = 48;

    private final static double BENDER_SPEED = 0.2;

    /** angle where bender is out of the way so we can shoot at the speaker */
    private final static double BENDER_SPEAKER_LOCATION = 51.6;

    /**
     * angle where bender is down so we can load it with a note from internal
     * storage
     */
    private final static double BENDER_INTERNAL_LOAD_NOTE_LOCATION = 27.2;

    /**
     * angle where bender is positioned so we can load it with a note from the
     * source
     */
    private final static double BENDER_LOAD_SOURCE_LOCATION = 25.5;

    /** angle where bender is in the correct location to shoot into the amp */
    private final static double BENDER_SHOOT_AMP_LOCATION = 47.6;

    private final static double BENDER_PREP_TRAP_LOCATION = 72;

    private final static double BENDER_SHOOT_TRAP_lOCATION = 43;

    /** distance away from expected location that we still concider good */
    private final static double BENDER_ANGLE_THRESHOLD = 5;

    private static final PIDController pidController = new PIDController(kP, kI, kD, kDt);

    private enum BenderState {
        stopped,
        raising,
        lowering
    };

    private static BenderAngleSubsystem mInstance = null;

    public static BenderAngleSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new BenderAngleSubsystem();
        }
        return mInstance;
    }

    private BenderAngleSubsystem() {
        super(pidController, 0);

        pidController.setTolerance(BENDER_ANGLE_THRESHOLD);
        pidController.setIZone(kIZone);
        pidController.setIntegratorRange(ANGLE_MIN, ANGLE_MAX);

        mBenderTiltMotor = new TalonSRX(Constants.kBenderTiltMotorId);
        m_benderAngleEncoder = new DutyCycleEncoder(Constants.kBenderAngleEncoderPin);

        mBenderTiltMotor.setNeutralMode(NeutralMode.Brake);

        if (Robot.isDebug) {
            m_sbTab = Shuffleboard.getTab("Bender (Debug)");

            m_sbTab.addDouble("Encoder", new DoubleSupplier() {
                @Override
                public double getAsDouble() {
                    return m_benderAngleEncoder.getAbsolutePosition();
                };
            });

            m_sbTab.addBoolean("PID Enabled", new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    return isEnabled();
                };
            });

            m_sbTab.addBoolean("Lower", new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    return isBenderLowerLimitReached();
                };
            });

            m_sbTab.addBoolean("Upper", new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    return isBenderUpperLimitReached();
                };
            });

            m_sbTab.addString("Target position", new Supplier<String>() {
                @Override
                public String get() {
                    return m_currentTargetPosition.toString();
                }
            });

            m_sbTab.addDouble("PID goal", new DoubleSupplier() {
                @Override
                public double getAsDouble() {
                    return m_controller.getSetpoint();
                };
            });

            m_sbTab.addDouble("PID output", new DoubleSupplier() {
                @Override
                public double getAsDouble() {
                    return mBenderTiltMotor.getMotorOutputVoltage();
                };
            });

            m_sbTab.addDouble("Current Angle:", new DoubleSupplier() {
                @Override
                public double getAsDouble() {
                    return m_benderAngle;
                };
            });

            m_sbTab.add(pidController);
        }

        new Thread("benderAngleEncoder") {
            public void run() {
                while (true) {
                    m_benderAngle = m_benderAngleEncoder.getAbsolutePosition() * 100d;
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
                mBenderTiltMotor.setNeutralMode(NeutralMode.Coast);
            } else {
                mBenderTiltMotor.setNeutralMode(NeutralMode.Brake);
            }
        }

        if (isDisabled) {
            disable();
            stopBender();
            return;
        }
    }

    private boolean atGoal() {
        return pidController.atSetpoint();
    }

    public void stopBender() {
        if (m_benderState != BenderState.stopped) {
            System.out.println("stopping bender");
            m_benderState = BenderState.stopped;
            mBenderTiltMotor.set(TalonSRXControlMode.PercentOutput, 0);
        }
    }

    public void setBenderPosition(BenderPosition position) {
        double angle = 0;
        m_currentTargetPosition = position;
        switch (position) {
            case LOAD_INTERNAL:
                angle = BENDER_INTERNAL_LOAD_NOTE_LOCATION;
                break;
            case LOAD_SOURCE:
                angle = BENDER_LOAD_SOURCE_LOCATION;
                break;
            case SHOOT_AMP:
                angle = BENDER_SHOOT_AMP_LOCATION;
                break;
            case SHOOT_SPEAKER:
                angle = BENDER_SPEAKER_LOCATION;
                break;
            case PREP_TRAP:
                angle = BENDER_PREP_TRAP_LOCATION;
                break;
            case SHOOT_TRAP:
                angle = BENDER_SHOOT_TRAP_lOCATION;
                break;
        }

        if (angle > m_benderAngle) {
            m_benderState = BenderState.raising;
        } else if (angle < m_benderAngle) {
            m_benderState = BenderState.lowering;
        } else {
            m_benderState = BenderState.stopped;
        }

        System.out.println("setting angle: " + angle);
        this.m_controller.reset();
        pidController.setSetpoint(angle);
        enable();
    }

    private boolean isBenderUpperLimitReached() {
        return m_benderAngle > ANGLE_MAX;
    }

    private boolean isBenderLowerLimitReached() {
        return m_benderAngle < ANGLE_MIN;
    }

    @Override
    public void useOutput(double output, double setpoint) {
        // clamp the output to a sane range
        double val;
        if (output < 0) {
            val = Math.max(-kMotorVoltageLimit, output);
        } else {
            val = Math.min(kMotorVoltageLimit, output);
        }
        mBenderTiltMotor.set(TalonSRXControlMode.PercentOutput, -val);
    }

    @Override
    public double getMeasurement() {
        return m_benderAngle;
    }

    /**
     * @return true if the bender is flipped back out of the way so
     *         that a note can be shot at the speaker without hitting the bender
     */
    public boolean isInSpeakerLocation() {
        return m_currentTargetPosition == BenderPosition.SHOOT_SPEAKER && atGoal();
    }

    /**
     * @return true if the bender is flipped forward so that a note can be fed from
     *         the internal chamber into the bender
     */
    public boolean isInInternalLoadingLocation() {
        return m_currentTargetPosition == BenderPosition.LOAD_INTERNAL && atGoal();
    }

    /**
     * @return true if the bender is is the correct location for firing into the amp
     */
    public boolean isInAmpLocation() {
        return m_currentTargetPosition == BenderPosition.SHOOT_AMP && atGoal();
    }

    public boolean isInTrapPrepLocation() {
        return m_currentTargetPosition == BenderPosition.PREP_TRAP && atGoal();
    }

    public void raiseBender() {
        disable();
        System.out.println("called raise elevator");
        if (!isBenderUpperLimitReached()) {
            // mBenderTiltMotor.set(TalonSRXControlMode.PercentOutput, BENDER_SPEED);
            m_benderState = BenderState.raising;
            System.out.println("raising bender");
        }
    }

    public void lowerBender() {
        disable();
        System.out.println("called lower elevator");
        if (!isBenderLowerLimitReached()) {
            mBenderTiltMotor.set(TalonSRXControlMode.PercentOutput, -BENDER_SPEED);
            m_benderState = BenderState.lowering;
            System.out.println("lowering");
        }
    }
}
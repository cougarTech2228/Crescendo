package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class BenderAngleSubsystem extends ProfiledPIDSubsystem {

    public enum BenderPosition {
        SHOOT_SPEAKER,
        SHOOT_AMP,
        LOAD_INTERNAL,
        LOAD_SOURCE
    };

    private ShuffleboardTab m_sbTab;
    private TalonSRX mBenderTiltMotor;
    private BenderState m_benderState = BenderState.stopped;
    private DutyCycleEncoder m_benderAngleEncoder;
    private double m_feedforwardVal = 0;

    private double m_benderAngle;
    private BenderPosition m_currentTargetPosition = BenderPosition.SHOOT_SPEAKER;

    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html
    // Steps for tuning the feedforward values Kg and Kv (leave others at 0)
    // 1. Start by setting Kg and Kv to zero.
    // 2. Increase Kg until the arm can hold its position with as little
    // movement as possible. If the arm moves in the opposite direction,
    // decrease until it remains stationary. You will have to zero in on
    // Kg precisely (at least four decimal places).
    // 3. Increase the velocity feedforward gain Kv until the arm tracks
    // the setpoint during smooth, slow motion. If the arm overshoots,
    // reduce the gain. Note that the arm may "lag" the commanded motion,
    // this is normal, and is fine so long as it moves the correct amount
    // in total.

    private static final double kSVolts = 0;
    private static final double kGVolts = 0;
    private static final double kVVolt = 0;
    private static final double kAVolt = 0;

    private static final double kP = 1.5;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kDt = 0.02;

    private static final double kMaxVelocity = 10.0;
    private static final double kMaxAcceleration = 2.0;

    private static final double kMotorVoltageLimit = 5;

    private static final double ANGLE_MIN = 5;
    private static final double ANGLE_MAX = 48;

    // private final static double BENDER_RAISE_SPEED = 0.2;
    // private final static double BENDER_LOWER_SPEED = -0.2;

    /** angle where bender is out of the way so we can shoot at the speaker */
    private final static double BENDER_SPEAKER_LOCATION = 51.6;

    /** angle where bender is down so we can load it with a note from internal storage */
    private final static double BENDER_INTERNAL_LOAD_NOTE_LOCATION = 25.9;

    /** angle where bender is positioned so we can load it with a note from the source */
    private final static double BENDER_LOAD_SOURCE_LOCATION = BENDER_INTERNAL_LOAD_NOTE_LOCATION;

    /** angle where bender is in the correct location to shoot into the amp */
    private final static double BENDER_SHOOT_AMP_LOCATION = 47.6;

    /** distance away from expected location that we still concider good */
    private final static double BENDER_ANGLE_THRESHOLD = 0.4;

    private static final ProfiledPIDController pidController = new ProfiledPIDController(
            kP, kI, kD,
            new TrapezoidProfile.Constraints(
                    kMaxVelocity,
                    kMaxAcceleration),
            kDt);

    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
            kSVolts, kGVolts,
            kVVolt, kAVolt);

    private enum BenderState {
        stopped,
        raising,
        lowering
    };

    public BenderAngleSubsystem() {
        super(pidController, 0);

        pidController.setTolerance(BENDER_ANGLE_THRESHOLD);

        mBenderTiltMotor = new TalonSRX(Constants.kBenderTiltMotorId);
        m_benderAngleEncoder = new DutyCycleEncoder(Constants.kBenderAngleEncoderPin);

        mBenderTiltMotor.setNeutralMode(NeutralMode.Brake);

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
                return m_controller.getGoal().position;
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

        // m_sbTab.addDouble("FF:", new DoubleSupplier() {
        //     @Override
        //     public double getAsDouble() {
        //         return m_feedforwardVal;
        //     };
        // });
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

    @Override
    public void periodic() {
        super.periodic();

        // // Boundary check the distance sensor's range values
        // if (m_benderAngle > ANGLE_MAX) {
        //     System.out.println("Bender angle sensor exceeded max range limit");
        //     m_benderAngle = ANGLE_MAX;
        // } else if (m_benderAngle < ANGLE_MIN) {
        //     System.out.println("Bender angle sensor exceeded min range limit");
        //     m_benderAngle = ANGLE_MIN;
        // }

        if (DriverStation.isDisabled()) {
            pidController.setGoal(getMeasurement());
            disable();
            mBenderTiltMotor.set(TalonSRXControlMode.PercentOutput, 0);
            //mBenderTiltMotor.setIdleMode(IdleMode.kCoast);
            return;
        }

        // if (isBenderUpperLimitReached() && (m_benderState == BenderState.raising)) {
        //     System.out.println("Bender Upper Limit Reached");
        //     disable();
        //     stopBender();
        // } else if (isBenderLowerLimitReached() && (m_benderState == BenderState.lowering)) {
        //     System.out.println("Bender Lower Limit Reached");
        //     disable();
        //     stopBender();
        // }

        if (pidController.atGoal()) {
            stopBender();
            disable();
        }
    }

    private boolean atGoal() {
        return pidController.atGoal();
    }

    private void stopBender() {
        if (m_benderState != BenderState.stopped) {
            System.out.println("stopping elevator");
            m_benderState = BenderState.stopped;
            mBenderTiltMotor.set(TalonSRXControlMode.PercentOutput, 0);
        }
    }

    public void setBenderPosition(BenderPosition position) {
        double angle = 0;
        m_currentTargetPosition = position;
        switch(position){
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
        }

        if (angle > m_benderAngle) {
            m_benderState = BenderState.raising;
        } else if (angle < m_benderAngle) {
            m_benderState = BenderState.lowering;
        } else {
            m_benderState = BenderState.stopped;
        }

        System.out.println("setting angle: " + angle);
        this.m_controller.reset(getMeasurement());
        pidController.setGoal(angle);
        enable();
    }

    private boolean isBenderUpperLimitReached() {
        return m_benderAngle > ANGLE_MAX;
    }

    private boolean isBenderLowerLimitReached() {
        return m_benderAngle < ANGLE_MIN;
    }

    private boolean isStopped() {
        return (m_benderState == BenderState.stopped);
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        m_feedforwardVal = feedforward;
        double newOutput = output + feedforward;
        // Add the feedforward to the PID output to get the motor output

        // clamp the output to a sane range
        double val;
        if (newOutput < 0) {
            val = Math.max(-kMotorVoltageLimit, newOutput);
        } else {
            val = Math.min(kMotorVoltageLimit, newOutput);
        }
        mBenderTiltMotor.set(TalonSRXControlMode.PercentOutput, -val/12);
    }

    @Override
    public double getMeasurement() {
        return m_benderAngle;
    }

        /**
     *  @return true if the bender is flipped back out of the way so
     *  that a note can be shot at the speaker without hitting the bender
     */
    public boolean isInSpeakerLocation() {
        return m_currentTargetPosition == BenderPosition.SHOOT_SPEAKER && atGoal();
    }

    /**
     * @return true if the bender is flipped forward so that a note can be fed from
     * the internal chamber into the bender
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
}
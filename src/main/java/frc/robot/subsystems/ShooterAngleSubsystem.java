package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterAngleSubsystem extends SubsystemBase {
    
    private ShuffleboardTab m_sbTab;
    private TalonSRX mLinearActuatorMotor;
    private DutyCycleEncoder mShooterAngleEncoder;
    private static final double SPEED_UP = -1;
    private static final double SPEED_DOWN = 1;
    private static final double SPEED_UP_FINE = -0.3;
    private static final double SPEED_DOWN_FINE = 0.3;
    private double m_shooterAngle;

    // private static final double kSVolts = 0;
    // private static final double kGVolts = 0;
    // private static final double kVVolt = 0;
    // private static final double kAVolt = 0;

    private double mGoal = 0;
    private double AUTO_THRESHOLD = 1;
    private double AUTO_THRESHOLD_FINE = 15;
    private boolean mAutoEnabled = false;

    //max up is 36.4
    //min down is 41.1

    /** angle where shooter is able to shoot at the speaker */
    private final static double SHOOT_SPEAKER_ANGLE = 0;

    /** angle where shooter is able to shoot at the amp */
    private final static double SHOOT_AMP_ANGLE = 378;

    /** angle where shooter is positioned so we can go under the chain */
    private final static double UNDER_CHAIN_ANGLE = 411;

    /** angle where shooter is in the correct location to load from source*/
    private final static double LOAD_SOURCE_HEIGHT = 0;

    public enum ShooterPosition {
        SHOOT_SPEAKER,
        SHOOT_AMP,
        LOAD_SOURCE,
        HEIGHT_CHAIN
    };

    enum State {
        STOPPED,
        RAISING,
        LOWERING,
    }
    private State mCurrentState = State.STOPPED;

    private ShooterPosition m_currentTargetPosition = ShooterPosition.SHOOT_SPEAKER;


    
    public ShooterAngleSubsystem() {
        mLinearActuatorMotor = new TalonSRX(Constants.kLinearActuatorLeftMotorId);
        mShooterAngleEncoder = new DutyCycleEncoder(Constants.kShooterAngleEncoderId);
        mLinearActuatorMotor.setNeutralMode(NeutralMode.Brake);

        m_sbTab = Shuffleboard.getTab("Shooter Angle (Debug)");

        m_sbTab.addDouble("Linear Actuator Encoder", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return getMeasurement();
            };
        });

        m_sbTab.addBoolean("Linear Actuator auto Enabled", new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return mAutoEnabled;
            };
        });

        m_sbTab.addDouble("Linear Actuator goal", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return mGoal;
            };
        });

        m_sbTab.addString("Linear Actuator state", new Supplier<String>() {
            @Override
            public String get() {
                return mCurrentState.name();
            }
        });


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

    @Override
    public void periodic() {
        super.periodic();
        if (mAutoEnabled && atGoal()) {
            stopMotor();
            mAutoEnabled = false;
        }

        if (mAutoEnabled) {
            doAutoMovement();
        }        
    }

    public void stopMotor() {
        // Stops shooter motor
        
        mLinearActuatorMotor.set(TalonSRXControlMode.PercentOutput, 0);
        System.out.println("stopped");
    }

    public void raiseShooter() {
        // Moves the shooter
        System.out.println("called raise shooter");
        
        
        mLinearActuatorMotor.set(TalonSRXControlMode.PercentOutput, SPEED_UP);
        System.out.println("raising");
    }

    public void lowerShooter() {
        // Moves the shooter down
        System.out.println("called lower shooter");
        

        mLinearActuatorMotor.set(TalonSRXControlMode.PercentOutput, SPEED_DOWN);
        System.out.println("lowering");
    }

	
	protected double getMeasurement() {
		return m_shooterAngle;
    }


    public boolean isInSpeakerLocation() {
        return m_currentTargetPosition == ShooterPosition.SHOOT_SPEAKER;
    //    return true;
    }

    public boolean isInAmpLocation() {
        return m_currentTargetPosition == ShooterPosition.SHOOT_AMP;
        // return true;
    }

    public boolean isInChainLocation() {
        return m_currentTargetPosition == ShooterPosition.HEIGHT_CHAIN;
        // return true;
    }

    public boolean isInSourceLocation() {
        return m_currentTargetPosition == ShooterPosition.LOAD_SOURCE;
        // return true;
    }

    public void setPosition(ShooterPosition position) {
        m_currentTargetPosition = position;
        switch (m_currentTargetPosition) {
            case SHOOT_SPEAKER:
            mGoal = SHOOT_SPEAKER_ANGLE;
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
        }

        System.out.println("setting shooter angle: " + mGoal);
        mAutoEnabled = true;
        doAutoMovement();
    }

    private boolean atGoal() {
        return Math.abs(mGoal - getMeasurement()) < AUTO_THRESHOLD;
    }

    private void doAutoMovement() {
        if (atGoal()) {
            mAutoEnabled = false;
            return;
        }
        if (getMeasurement() > mGoal) {
            if(Math.abs(getMeasurement() - mGoal) < AUTO_THRESHOLD_FINE){
                System.out.println("Speed up fine");
                mLinearActuatorMotor.set(TalonSRXControlMode.PercentOutput, SPEED_UP_FINE);
            } else {
                System.out.println("Speed up");
                mLinearActuatorMotor.set(TalonSRXControlMode.PercentOutput, SPEED_UP);
            }
            mCurrentState = State.RAISING;
        }
        else if (getMeasurement() < mGoal) {
            if(Math.abs(getMeasurement() - mGoal) < AUTO_THRESHOLD_FINE){
                System.out.println("Speed down fine");
                 mLinearActuatorMotor.set(TalonSRXControlMode.PercentOutput, SPEED_DOWN_FINE);
            } else {
                System.out.println("Speed down");
                mLinearActuatorMotor.set(TalonSRXControlMode.PercentOutput, SPEED_DOWN);
            }
            mCurrentState = State.LOWERING;
        }
    }
}
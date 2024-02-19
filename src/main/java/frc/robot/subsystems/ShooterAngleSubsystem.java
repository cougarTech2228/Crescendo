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
import frc.robot.Constants;

public class ShooterAngleSubsystem extends ProfiledPIDSubsystem {
    
    private ShuffleboardTab m_sbTab;
    private TalonSRX mLinearActuatorMotor;
    private DutyCycleEncoder mShooterAngleEncoder;
    private static final double shooterSpeedUp = -1;
    private static final double shooterSpeedDown = 1;
    private double m_shooterAngle;

    // private static final double kSVolts = 0;
    // private static final double kGVolts = 0;
    // private static final double kVVolt = 0;
    // private static final double kAVolt = 0;

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kDt = 0.02;

    private static final double kMaxVelocity = 1.0;
    private static final double kMaxAcceleration = 2.0;

    private static final double kMotorVoltageLimit = .1;

    //max up is 36.4
    //min down is 41.1

    /** angle where shooter is able to shoot at the speaker */
    private final static double SHOOT_SPEAKER_ANGLE = 0;

    /** angle where shooter is able to shoot at the amp */
    private final static double SHOOT_AMP_ANGLE = 0;

    /** angle where shooter is positioned so we can go under the chain */
    private final static double UNDER_CHAIN_ANGLE = 0;

    /** angle where shooter is in the correct location to load from source*/
    private final static double LOAD_SOURCE_HEIGHT = 0;

    /** distance away from expected location that we still concider good */
    private final static double SHOOTER_ANGLE_THRESHOLD = 0.4;

    public enum ShooterPosition {
        SHOOT_SPEAKER,
        SHOOT_AMP,
        LOAD_SOURCE,
        HEIGHT_CHAIN
    };

    private ShooterPosition m_currentTargetPosition = ShooterPosition.SHOOT_SPEAKER;


    private static final ProfiledPIDController pidController = new ProfiledPIDController(
            kP, kI, kD,
            new TrapezoidProfile.Constraints(
                    kMaxVelocity,
                    kMaxAcceleration),
            kDt);

    
    public ShooterAngleSubsystem() {

        super(pidController, 0);

        pidController.setTolerance(SHOOTER_ANGLE_THRESHOLD);

        
        mLinearActuatorMotor = new TalonSRX(Constants.kLinearActuatorLeftMotorId);
        mShooterAngleEncoder = new DutyCycleEncoder(Constants.kShooterAngleEncoderId);
    
        mLinearActuatorMotor.setNeutralMode(NeutralMode.Brake);
        m_sbTab = Shuffleboard.getTab("Shooter (Debug)");

        m_sbTab.addDouble("Encoder", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return mShooterAngleEncoder.getAbsolutePosition() *100;
            };
        });

        m_sbTab.addBoolean("PID Enabled", new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return isEnabled();
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
                return mLinearActuatorMotor.getMotorOutputVoltage();
            };
        });

        m_sbTab.addDouble("Current Angle:", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return m_shooterAngle;
            };
        });

        new Thread("shooterAngleEncoder") {
            public void run() {
                while (true) {
                    m_shooterAngle = mShooterAngleEncoder.getAbsolutePosition() * 100d;
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

        if (pidController.atGoal()) {
            stopMotor();
            disable();
        }

        if(m_shooterAngle == 0){
            stopMotor();
            disable();
            System.out.println("SHOOTER ANGLE ENCODER MISSING  FIX ME  HEEEEELLLLLPPPPPPPP");
        }
    }

    public void stopMotor() {
        // Stops shooter motor
        disable();
        mLinearActuatorMotor.set(TalonSRXControlMode.PercentOutput, 0);
        System.out.println("stopped");
    }

    public void raiseShooter() {
        // Moves the shooter
        System.out.println("called raise shooter");
        disable();
        
        mLinearActuatorMotor.set(TalonSRXControlMode.PercentOutput, shooterSpeedUp);
        System.out.println("raising");
    }

    public void lowerShooter() {
        // Moves the shooter down
        System.out.println("called lower shooter");
        disable();

        mLinearActuatorMotor.set(TalonSRXControlMode.PercentOutput, shooterSpeedDown);
        System.out.println("lowering");
    }

	@Override
	protected void useOutput(double output, edu.wpi.first.math.trajectory.TrapezoidProfile.State setpoint) {
		// Calculate the feedforward from the sepoint
        // double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        // m_feedforwardVal = feedforward;
        // double newOutput = output + feedforward;
        // Add the feedforward to the PID output to get the motor output

        // clamp the output to a sane range
        double val;
        if (output < 0) {
            val = Math.max(-kMotorVoltageLimit, output);
        } else {
            val = Math.min(kMotorVoltageLimit, output);
        }
        mLinearActuatorMotor.set(TalonSRXControlMode.PercentOutput, val);
    }

	@Override
	protected double getMeasurement() {
		return m_shooterAngle;
    }


    public void setShooterPosition(ShooterPosition position) {
        double angle = 0;
        m_currentTargetPosition = position;
        switch(position){
            case HEIGHT_CHAIN:
                angle = UNDER_CHAIN_ANGLE;
                break;
            case LOAD_SOURCE:
                angle = LOAD_SOURCE_HEIGHT;
                break;
            case SHOOT_AMP:
                angle = SHOOT_AMP_ANGLE;
                break;
            case SHOOT_SPEAKER:
                angle = SHOOT_SPEAKER_ANGLE;
                break;
        }

        System.out.println("Shooter setting angle: " + angle);
        this.m_controller.reset(getMeasurement());
        pidController.setGoal(angle);
        enable();
    }

    public boolean isInSpeakerLocation() {
        return m_currentTargetPosition == ShooterPosition.SHOOT_SPEAKER && pidController.atGoal();
    }

    public boolean shooterIsInAmpLocation() {
        return m_currentTargetPosition == ShooterPosition.SHOOT_AMP && pidController.atGoal();
    }
}
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.BenderAngleSubsystem.BenderPosition;

public class ShooterSubsystem extends SubsystemBase {

    private BenderAngleSubsystem mBenderAngleSubsystem = new BenderAngleSubsystem();
    private ShooterAngleSubsystem mShooterAngleSubsystem = new ShooterAngleSubsystem();
    private TalonFX mShooterFeedMotor;
    private TalonFX mShooterFlywheelMotor;
    private DigitalInput mGroundFeedSensor;
    private DigitalInput mMiddleFeedSensor;
    private DigitalInput mTopFeedSensor;
    private CANSparkMax mGroundFeedMotor;
    private CANSparkMax mShooterBeltMotor;
    private TalonSRX mBenderFeedMotor;
    double timeCheck;
    private enum ShooterState {
        EMPTY,
        ACQUIRINGBOTTOM,
        ACQUIRINGTOP,
        LOADED,
        FIRE_SPEAKER_PREP,
        FIRE_SPEAKER,
        BENDER_LOAD_INTERNAL_PREP,
        BENDER_LOAD_INTERNAL_LOAD_BENDER,
        BENDER_LOAD_INTERNAL_LOAD_BENDER_EXITING_MIDDLE,
        BENDER_LOAD_INTERNAL_LOADED,
        READY_FOR_FIRE_AMP,
        FIRE_AMP
    };
    private ShooterState shooterState = ShooterState.EMPTY;
    private boolean m_isLoaded = false;

    public enum OperatorEvent {
        NONE,
        PREP_AMP,
        PREP_SPEAKER,
        FIRE_SPEAKER,
        FIRE_AMP,
        SPIT
    };
    private OperatorEvent currentEvent;

    private final static double LOAD_SPEED_GROUND = 1;
    private final static double LOAD_SPEED_BELT = -0.2;
    private final static double LOAD_SPEED_SHOOTER_FEED = 0.1;
    private final static double SHOOTER_DELAY = 0.5;
    private final static double SPEAKER_SHOOT_SPEED = 1.0;
    private final static double SPEAKER_SHOOT_BELT_SPEED = -0.2;
    private final static double SPEAKER_FLYWHEEL_SHOOT_SPEED = 1.0;
    private final static double BENDER_SHOOT_SPEED = 1.0;
    private final static double BENDER_FEED_SPEED = -1.0;
    private final static double LINEAR_ACTUATOR_RAISE_SPEED = 0.7;
    private final static double LINEAR_ACTUATOR_LOWER_SPEED = -0.7;

    


    public ShooterSubsystem() {
        mShooterFeedMotor = new TalonFX(Constants.kShooterFeedMotorId);
        mShooterFlywheelMotor = new TalonFX(Constants.kShooterFlywheelMotorId);
        mGroundFeedSensor = new DigitalInput(Constants.kGroundFeedSensorId);
        mMiddleFeedSensor = new DigitalInput(Constants.kMiddleFeedSensorId);
        mTopFeedSensor = new DigitalInput(Constants.kTopFeedSensorId);
        mGroundFeedMotor = new CANSparkMax(Constants.kGroundFeedMotorId, MotorType.kBrushless);
        mShooterBeltMotor = new CANSparkMax(Constants.kShooterBeltMotorId, MotorType.kBrushless);
        
        mBenderFeedMotor = new TalonSRX(Constants.kBenderFeedMotorId);
        mBenderFeedMotor.setNeutralMode(NeutralMode.Brake);
        mShooterFlywheelMotor.setNeutralMode(NeutralModeValue.Brake);
        mShooterFeedMotor.setNeutralMode(NeutralModeValue.Brake);
    }   
    
    public void initStateMachine(boolean preloaded) {
        if (preloaded) {
            changeState(ShooterState.LOADED);
        } else {
            changeState(ShooterState.EMPTY);
        }
    }

    private void changeState(ShooterState newState) {
        System.out.println("** Shooter State: " + shooterState + " --> " + newState);
        shooterState = newState;
    }

    public boolean isHoldingNote() {
        return m_isLoaded;
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        SmartDashboard.putBoolean("GroundFeedSensor", isNoteAtBottom());
        SmartDashboard.putBoolean("MiddleFeedSensor", isNoteAtMiddle());
        SmartDashboard.putBoolean("TopFeedSensor", isNoteAtTop());
        SmartDashboard.putString("Shooter State:", shooterState.name());

        // SmartDashboard.putNumber("Alt Encoder Velocity", mBenderEncoder.getVelocity());
        // SmartDashboard.putNumber("Applied Output", mBenderMotor.getAppliedOutput());
        if(currentEvent == OperatorEvent.SPIT) {
            mGroundFeedMotor.set(-LOAD_SPEED_GROUND);
            return;
        }
        switch (shooterState) {
            case EMPTY:
                stopAllShooterMotors();
                // TODO: if bender is not in default position, shooterState = ShooterState.SETBENDERTODEFAULTPOSITION;
                // else shooterState = ShooterState.
                if (isNoteAtBottom()) {
                    changeState(ShooterState.ACQUIRINGBOTTOM);
                }
                break;
            case ACQUIRINGBOTTOM:
                acquiringBottom();
                if (isNoteAtMiddle()) {
                    changeState(ShooterState.ACQUIRINGTOP);
                }
                break;
            case ACQUIRINGTOP:
                if (!isNoteAtMiddle()) {
                    m_isLoaded = true;
                    changeState(ShooterState.LOADED);
                }
                break;
            case LOADED:
                stopAllShooterMotors();

                if (currentEvent == OperatorEvent.FIRE_SPEAKER) {
                    currentEvent = OperatorEvent.NONE;
                    // ensure bender is out of the way
                    mBenderAngleSubsystem.setBenderPosition(BenderPosition.SHOOT_SPEAKER);
                    changeState(ShooterState.FIRE_SPEAKER_PREP);
                }
                else if (currentEvent == OperatorEvent.PREP_AMP) {
                    currentEvent = OperatorEvent.NONE;
                    mBenderAngleSubsystem.setBenderPosition(BenderPosition.LOAD_INTERNAL);
                    changeState(ShooterState.BENDER_LOAD_INTERNAL_PREP);
                }

                break;
            case FIRE_SPEAKER_PREP:
                mShooterFlywheelMotor.set(SPEAKER_FLYWHEEL_SHOOT_SPEED);
                // TODO: Make this if statement work
                if (flywheelIsAtShootingSpeed() && mBenderAngleSubsystem.benderIsInSpeakerLocation()) {
                    changeState(ShooterState.FIRE_SPEAKER);
                    timeCheck = Timer.getFPGATimestamp();
                }
                break;
            case FIRE_SPEAKER:
                launchSpeaker();
                if (isShooterDelayExpired()) {
                    m_isLoaded = false;
                    changeState(ShooterState.EMPTY);
                }
                break;

            case BENDER_LOAD_INTERNAL_PREP:
                if (mBenderAngleSubsystem.benderIsInInternalLoadingLocation()) {
                    changeState(ShooterState.BENDER_LOAD_INTERNAL_LOAD_BENDER);
                }
                break;
            case BENDER_LOAD_INTERNAL_LOAD_BENDER:
                mBenderFeedMotor.set(ControlMode.PercentOutput, BENDER_FEED_SPEED); // "slow"
                mShooterBeltMotor.set(LOAD_SPEED_BELT);
                mShooterFeedMotor.set(LOAD_SPEED_SHOOTER_FEED);
                mShooterFlywheelMotor.set(LOAD_SPEED_SHOOTER_FEED);
                if (isNoteAtTop()) {
                    changeState(ShooterState.BENDER_LOAD_INTERNAL_LOAD_BENDER_EXITING_MIDDLE);
                }
                break;
            case BENDER_LOAD_INTERNAL_LOAD_BENDER_EXITING_MIDDLE:
                if (!isNoteAtTop()) {
                    mBenderAngleSubsystem.setBenderPosition(BenderPosition.SHOOT_AMP);
                    changeState(ShooterState.BENDER_LOAD_INTERNAL_LOADED);
                }
                break;
            case BENDER_LOAD_INTERNAL_LOADED:
                mBenderFeedMotor.set(ControlMode.PercentOutput, 0);
                mShooterBeltMotor.set(0);
                mShooterFeedMotor.set(0);
                mShooterFlywheelMotor.set(0);
                if(mBenderAngleSubsystem.benderIsInAmpLocation()) {
                    changeState(ShooterState.READY_FOR_FIRE_AMP);
                }
                break;
            case READY_FOR_FIRE_AMP:
                // If amp shooter button is pressed
                // {timeCheck = Timer.getFPGATimestamp(); changeState(ShooterState.BENDERSHOOT;)}
                if (currentEvent == OperatorEvent.FIRE_SPEAKER) {
                    changeState(ShooterState.FIRE_AMP);
                }
                break;
            case FIRE_AMP:
                mBenderFeedMotor.set(ControlMode.PercentOutput, BENDER_SHOOT_SPEED);
                if (isShooterDelayExpired()) {
                    m_isLoaded = false;
                    changeState(ShooterState.EMPTY);
                }
                break;
        }
        
    }

    /**
     *  @return true if a note is at the front of the intake
     */
    private boolean isNoteAtBottom() {
        return !mGroundFeedSensor.get();
    }

    /**
     *  @return true if a note is currently seen between the ground
     * feeder, and the main chamber
     */
    private boolean isNoteAtMiddle() {
        return mMiddleFeedSensor.get();
    }

    /**
     *  @return true if a note is currently seen between the main chamber and the bender
     */
    private boolean isNoteAtTop() {
        return !mTopFeedSensor.get();
    }

    /**
     * @return true if the flywheel is up to speed for shooting in the speaker
     */
    private boolean flywheelIsAtShootingSpeed() {
        System.out.println(mShooterFlywheelMotor.getVelocity().getValue() >= 1.0);
        return true;
        // return mShooterFlywheelMotor.getVelocity().getValue() >= 1.0;
    }

    public void stopFeedShootMotors() {
        mShooterFeedMotor.set(0);
        mShooterFlywheelMotor.set(0);
        mGroundFeedMotor.set(0);
        mShooterBeltMotor.set(0);
    }

    public void stopBottomMotor() {
        mGroundFeedMotor.set(0);
    }

    public void raiseLinearActuator() {
        mShooterAngleSubsystem.raiseShooter();
        System.out.println("raising");
    }

    public void lowerLinearActuator() {
        mShooterAngleSubsystem.lowerShooter();
        System.out.println("lowering");
    }

    public void stopLinearActuator() {
        System.out.println("stop linear actuator shooter subsystem");
        mShooterAngleSubsystem.stopMotor();
    }

    private boolean isShooterDelayExpired() {
        // Returns a boolean, where true indicates that the time between the value of timeCheck and the present time is greater than SHOOTER_DELAY
        if ((Timer.getFPGATimestamp()-timeCheck)>SHOOTER_DELAY) {
            return true;
        } else {
            return false;
        }
    }

    private void acquiringBottom() {
        mGroundFeedMotor.set(LOAD_SPEED_GROUND);
        mShooterBeltMotor.set(LOAD_SPEED_BELT);
        mShooterFeedMotor.set(LOAD_SPEED_SHOOTER_FEED);
    }

    private void launchSpeaker() {
        System.out.println("launchSpeaker");
        mShooterFeedMotor.set(SPEAKER_SHOOT_SPEED);
        mShooterBeltMotor.set(SPEAKER_SHOOT_BELT_SPEED);
    }

    private void stopAllShooterMotors() {
        mGroundFeedMotor.set(0);
        mShooterBeltMotor.set(0);
        mShooterFeedMotor.set(0);
        mShooterFlywheelMotor.set(0);
    }

    /**
     * This method is called to inform the ShooterSubsystem of operator inputs
     */
    public void operatorEvent(OperatorEvent event) {
        currentEvent = event;
    }

    // public void setBender(BenderAngleSubsystem.BenderPosition position) {
    //     mBenderAngleSubsystem.setBenderPosition(position);
    // }
};
package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.bender.BenderAngleSubsystem;
import frc.robot.subsystems.bender.BenderAngleSubsystem.BenderPosition;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.Position;
import frc.robot.subsystems.shooter.ShooterIO.WhichMotor;
import frc.robot.subsystems.shooterAngle.ShooterAngleSubsystem;
import frc.robot.subsystems.shooterAngle.ShooterAngleSubsystem.ShooterPosition;

public class ShooterSubsystem extends SubsystemBase {

    private BenderAngleSubsystem mBenderAngleSubsystem = BenderAngleSubsystem.getInstance();
    private ShooterAngleSubsystem mShooterAngleSubsystem = ShooterAngleSubsystem.getInstance();
    private ElevatorSubsystem mElevatorSubsystem = ElevatorSubsystem.getInstance();

    private final ShooterIO mIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    
    private AddressableLED mLed = new AddressableLED(1);
    private AddressableLEDBuffer mLedBuffer = new AddressableLEDBuffer(147);
    double timeCheck;

    @AutoLogOutput
    private boolean m_isLoaded = false;

    @AutoLogOutput
    public static boolean isShooterLimit = false;

    @AutoLogOutput
    public static boolean isElevatorHome = false;

    public enum OperatorEvent {
        NONE,
        PREP_AMP,
        PREP_SPEAKER_FRONT,
        PREP_SPEAKER_SIDE,
        PREP_SOURCE,
        FIRE,
        SPIT,
        SPIT_STOP,
        PREP_TRAP
    };

    private final static double LOAD_SPEED_GROUND = 12;
    private final static double SPIT_SPEED_GROUND = -12;
    private final static double SPIT_SPEED_SHOOTER_FEED = -6;
    private final static double SPIT_SPEED_SHOOTER_BELT = 6;
    private final static double SPIT_SPEED_SHOOTER_FLYWHEEL = -6;
    private final static double SPIT_SPEED_BENDER = 6;
    private final static double LOAD_SPEED_BELT = -1.8;
    private final static double LOAD_SPEED_SHOOTER_FEED = 0.6;
    private final static double SPEAKER_SHOOT_SPEED = 12;
    private final static double SPEAKER_SHOOT_BELT_SPEED = -12;
    private final static double SPEAKER_FLYWHEEL_SHOOT_SPEED = 12;
    private final static double BENDER_SHOOT_SPEED = 12;
    private final static double BENDER_FEED_AMP_SPEED = -3.6;
    private final static double BENDER_FEED_SOURCE_SPEED = 7.2;
    private final static double SOURCE_FEED_SPEED = -2.4;

    private final static double SPEAKER_SHOOTER_DELAY = 0.5;
    private final static double AMP_PRELOAD_DELAY = 0.25;
    /** Abstract State class */
    private abstract class State {
        private String mName;

        protected State(String name) {
            mName = name;
        }

        public void enterState() {
        };

        public void exitState() {
        };

        public void run() {
        };

        public final void onEvent(OperatorEvent event) {
            System.out.println("Event: " + event + " in state " + mName);
            onEventInternal(event);
        };

        protected void onEventInternal(OperatorEvent event) {
            System.out.println("Ingoring Event " + event + " in state " + mName +
                    ", onEventInernal not overridden!");
        }

        @Override
        public String toString() {
            return mName;
        }
    }

    // State class instances
    private EmptyState mEmptyState = new EmptyState();
    private AcquiringBottomState mAcquiringBottomState = new AcquiringBottomState();
    private AcquiringTopState mAcquiringTopState = new AcquiringTopState();
    private LoadedState mLoadedState = new LoadedState();
    private FireSpeakerPrepState mFireSpeakerPrepState = new FireSpeakerPrepState();
    private FireSpeakerState mFireSpeakerState = new FireSpeakerState();
    private BenderLoadInternalPrepState mBenderLoadInternalPrepState = new BenderLoadInternalPrepState();
    private BenderLoadInternalLoadBenderState mBenderLoadInternalLoadBenderState = new BenderLoadInternalLoadBenderState();
    private BenderLoadInternalBenderExitingMiddleState mBenderLoadInternalBenderExitingMiddleState = new BenderLoadInternalBenderExitingMiddleState();
    private BenderLoadInternalLoadedAmpState mBenderLoadInternalLoadedAmpState = new BenderLoadInternalLoadedAmpState();
    private ReadyForFireAmpState mReadyForFireAmpState = new ReadyForFireAmpState();
    private FireAmpState mFireAmpState = new FireAmpState();
    private SpitState mSpitState = new SpitState();
    private PrepSpeakerFrontEmptyState mPrepSpeakerFrontEmptyState = new PrepSpeakerFrontEmptyState();
    private PrepSpeakerSideEmptyState mPrepSpeakerSideEmptyState = new PrepSpeakerSideEmptyState();
    private PrepSpeakerFrontLoadedState mPrepSpeakerFrontLoadedState = new PrepSpeakerFrontLoadedState();
    private PrepSpeakerSideLoadedState mPrepSpeakerSideLoadedState = new PrepSpeakerSideLoadedState();
    private PrepSourceState mPrepSourceState = new PrepSourceState();
    private SourceLoadingState mSourceLoadingState = new SourceLoadingState();
    private SourceLoadedState mSourceLoadedState = new SourceLoadedState();
    private ReadyForFireTrapState mReadyForFireTrapState = new ReadyForFireTrapState();
    private FireTrapState mFireTrapState = new FireTrapState();
    private BenderLoadInternalLoadedTrapState mBenderLoadInternalLoadedTrapState = new BenderLoadInternalLoadedTrapState();

    private boolean isAmp = true;

    private State currentState = mEmptyState;

    private void setLEDRed() {
        for (int i=0; i< mLedBuffer.getLength(); i++){
            mLedBuffer.setRGB(i, 255, 0, 0);
        }
        mLed.setData(mLedBuffer);
    }

    private void setLEDGreen() {
        for (int i=0; i< mLedBuffer.getLength(); i++){
            mLedBuffer.setRGB(i, 0, 255, 0);
        }
        mLed.setData(mLedBuffer);
    }

    private void setLEDOrange() {
        for (int i=0; i< mLedBuffer.getLength(); i++){
            mLedBuffer.setRGB(i, 166, 160, 50);
        }
        mLed.setData(mLedBuffer);
    }

    private class EmptyState extends State {
        public EmptyState() {
            super("Empty");
        }

        @Override
        public void enterState() {
            stopAllMotors();
            setLEDRed();
            mElevatorSubsystem.setPosition(ElevatorSubsystem.Position.HOME);
        }

        @Override
        public void run() {
            if (inputs.isNoteAtBottom) {
                changeState(mAcquiringBottomState);
            }
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case PREP_SPEAKER_FRONT:
                    changeState(mPrepSpeakerFrontEmptyState);
                    break;
                case PREP_SPEAKER_SIDE:
                    changeState(mPrepSpeakerSideEmptyState);
                    break;
                case SPIT:
                    changeState(mSpitState);
                    break;
                case PREP_SOURCE:
                    changeState(mPrepSourceState);
                default:
                    System.out.println("Ignoring event " + event + " in Empty State");
            }
        }
    }

    private class SpitState extends State {
        public SpitState() {
            super("Spit");
        }

        @Override
        public void enterState() {
            stopAllMotors();
            mIO.setGroundFeedMotorVoltage(SPIT_SPEED_GROUND);
            mIO.setShooterFeedMotorVoltage(SPIT_SPEED_SHOOTER_FEED);
            mIO.setBeltMotorVoltage(SPIT_SPEED_SHOOTER_BELT);
            mIO.setFlywheelMotorVoltage(SPIT_SPEED_SHOOTER_FLYWHEEL);
            mIO.setBenderFeedMotorVoltage(SPIT_SPEED_BENDER);
        }

        @Override
        public void exitState() {
            stopAllMotors();
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case SPIT_STOP:
                    changeState(mEmptyState);
                    break;
                default:
                    System.out.println("Ignoring event " + event + " in " + "Spit State");
            }
        }
    }

    private class AcquiringBottomState extends State {
        public AcquiringBottomState() {
            super("Acquiring Bottom");
        }

        @Override
        public void enterState() {
            mIO.setGroundFeedMotorVoltage(LOAD_SPEED_GROUND);
            mIO.setBeltMotorVoltage(LOAD_SPEED_BELT);
            mIO.setShooterFeedMotorVoltage(LOAD_SPEED_SHOOTER_FEED);
            setLEDOrange();

        }

        @Override
        public void run() {
            if (inputs.isNoteAtMiddle) {
                changeState(mAcquiringTopState);
            }
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case SPIT:
                    changeState(mSpitState);
                    break;
                case PREP_SOURCE:
                    changeState(mPrepSourceState);
                default:
                    System.out.println("Ignoring event " + event + " in Acquiring Bottom State");
            }
        }
    }

    private class AcquiringTopState extends State {
        public AcquiringTopState() {
            super("Acquiring Top");
        }

        @Override
        public void run() {
            if (!inputs.isNoteAtMiddle) {
                m_isLoaded = true;
                changeState(mLoadedState);
                mElevatorSubsystem.setPosition(Position.HOME);
            }
        }
    }

    private class LoadedState extends State {
        public LoadedState() {
            super("Loaded");
        }

        @Override
        public void enterState() {
            setLEDGreen();
            m_isLoaded = true;
            mElevatorSubsystem.setPosition(Position.HOME);
            stopAllMotors();
            // run the gound feed motor backwards while loaded to actively reject
            // new pieces from entering the intake
            // mGroundFeedMotor.set(SPIT_SPEED_GROUND);
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case FIRE:
                    changeState(mFireSpeakerPrepState);
                    break;
                case PREP_AMP:
                    isAmp = true;
                    changeState(mBenderLoadInternalPrepState);
                    break;
                case PREP_SPEAKER_FRONT:
                    changeState(mPrepSpeakerFrontLoadedState);
                    break;
                case PREP_SPEAKER_SIDE:
                    changeState(mPrepSpeakerSideLoadedState);
                    break;
                case PREP_TRAP:
                    isAmp = false;
                    changeState(mBenderLoadInternalPrepState);
                    break;
                case SPIT:
                    changeState(mSpitState);
                    break;
                default:
                    System.out.println("Ignoring event " + event + " in LoadedState");
            }
        }
    }

    private class PrepSpeakerFrontEmptyState extends State {
        public PrepSpeakerFrontEmptyState() {
            super("Prep Speaker Front Empty");
        }

        @Override
        public void enterState() {
            prepSpeakerFront();
        }

        @Override
        public void run() {
            if (mShooterAngleSubsystem.isInSpeakerLocation_front()) {
                changeState(mEmptyState);
            }
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case SPIT:
                    changeState(mSpitState);
                    break;
                default:
                    System.out.println("Ignoring event " + event + " in Acquiring Bottom State");
            }
        }
    }

    private class PrepSpeakerSideEmptyState extends State {
        public PrepSpeakerSideEmptyState() {
            super("Prep Speaker Side Empty");
        }

        @Override
        public void enterState() {
            prepSpeakerSide();
        }

        @Override
        public void run() {
            if (mShooterAngleSubsystem.isInSpeakerLocation_side()) {
                changeState(mEmptyState);
            }
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case SPIT:
                    changeState(mSpitState);
                    break;
                default:
                    System.out.println("Ignoring event " + event + " in Acquiring Bottom State");
            }
        }
    }

    private class PrepSpeakerFrontLoadedState extends State {
        public PrepSpeakerFrontLoadedState() {
            super("Prep Speaker Front Loaded");
        }

        @Override
        public void enterState() {
            prepSpeakerFront();
        }

        @Override
        public void run() {
            if (mShooterAngleSubsystem.isInSpeakerLocation_front()) {
                changeState(mLoadedState);
            }
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case SPIT:
                    changeState(mSpitState);
                    break;
                default:
                    System.out.println("Ignoring event " + event + " in Acquiring Bottom State");
            }
        }
    }

    private class PrepSpeakerSideLoadedState extends State {
        public PrepSpeakerSideLoadedState() {
            super("Prep Speaker Side Loaded");
        }

        @Override
        public void enterState() {
            prepSpeakerSide();
        }

        @Override
        public void run() {
            if (mShooterAngleSubsystem.isInSpeakerLocation_side()) {
                changeState(mLoadedState);
            }
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case SPIT:
                    changeState(mSpitState);
                    break;
                default:
                    System.out.println("Ignoring event " + event + " in Acquiring Bottom State");
            }
        }
    }

    private class FireSpeakerPrepState extends State {
        public FireSpeakerPrepState() {
            super("Fire Speaker Prep");
        }

        @Override
        public void enterState() {
            mBenderAngleSubsystem.setBenderPosition(BenderPosition.SHOOT_SPEAKER);
            mIO.setFlywheelMotorVoltage(SPEAKER_FLYWHEEL_SHOOT_SPEED);
        }

        @Override
        public void run() {
            double v = inputs.flywheelMotorVelocity;
            if (mBenderAngleSubsystem.isInSpeakerLocation() && v >= 95) {
                changeState(mFireSpeakerState);
            }
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case SPIT:
                    changeState(mSpitState);
                    break;
                default:
                    System.out.println("Ignoring event " + event + " in Acquiring Bottom State");
            }
        }
    }

    private class FireSpeakerState extends State {
        private double startTime;

        public FireSpeakerState() {
            super("Fire Speaker");
        }

        @Override
        public void enterState() {
            startTime = Timer.getFPGATimestamp();
            mIO.setShooterFeedMotorVoltage(SPEAKER_SHOOT_SPEED);
            mIO.setBeltMotorVoltage(SPEAKER_SHOOT_BELT_SPEED);
        }

        @Override
        public void exitState() {
            stopAllMotors();
            m_isLoaded = false;
        }

        @Override
        public void run() {
            if ((Timer.getFPGATimestamp() - startTime) > SPEAKER_SHOOTER_DELAY) {
                changeState(mEmptyState);
            }
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case SPIT:
                    changeState(mSpitState);
                    break;
                default:
                    System.out.println("Ignoring event " + event + " in Acquiring Bottom State");
            }
        }
    }

    private class BenderLoadInternalPrepState extends State {
        public BenderLoadInternalPrepState() {
            super("Bender Load Internal - Prep");
        }

        @Override
        public void enterState() {
            prepAmp();
        }

        @Override
        public void run() {
            if (mBenderAngleSubsystem.isInInternalLoadingLocation()) {
                changeState(mBenderLoadInternalLoadBenderState);
            }
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case SPIT:
                    changeState(mSpitState);
                    break;
                default:
                    System.out.println("Ignoring event " + event + " in Acquiring Bottom State");
            }
        }
    }

    private class BenderLoadInternalLoadBenderState extends State {
        public BenderLoadInternalLoadBenderState() {
            super("Bender Load Internal - Load Bender");
        }

        @Override
        public void enterState() {
            mIO.setBenderFeedMotorVoltage(BENDER_FEED_AMP_SPEED);
            mIO.setBeltMotorVoltage(LOAD_SPEED_BELT);
            mIO.setShooterFeedMotorVoltage(LOAD_SPEED_SHOOTER_FEED);
            mIO.setFlywheelMotorVoltage(LOAD_SPEED_SHOOTER_FEED);
        }

        @Override
        public void run() {
            if (inputs.isNoteAtTop) {
                changeState(mBenderLoadInternalBenderExitingMiddleState);
            }
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case SPIT:
                    changeState(mSpitState);
                    break;
                default:
                    System.out.println("Ignoring event " + event + " in Acquiring Bottom State");
            }
        }
    }

    private class BenderLoadInternalBenderExitingMiddleState extends State {
        public BenderLoadInternalBenderExitingMiddleState() {
            super("Bender Load Internal - Exiting Middle");
        }

        @Override
        public void exitState() {
            mIO.setBeltMotorVoltage(0);
            mIO.setShooterFeedMotorVoltage(0);
            mIO.setFlywheelMotorVoltage(0);
        }

        @Override
        public void run() {
            if (!inputs.isNoteAtTop) {
                if(isAmp){
                    changeState(mBenderLoadInternalLoadedAmpState);
                } else{
                    changeState(mBenderLoadInternalLoadedTrapState);
                }
            }
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case SPIT:
                    changeState(mSpitState);
                    break;
                default:
                    System.out.println("Ignoring event " + event + " in Acquiring Bottom State");
            }
        }
    }

    private class BenderLoadInternalLoadedAmpState extends State {
        private double feedTimerStart;

        public BenderLoadInternalLoadedAmpState() {
            super("Bender Load Internal - Loaded Amp");
        }

        @Override
        public void enterState() {
            feedTimerStart = Timer.getFPGATimestamp();
            mElevatorSubsystem.setPosition(ElevatorSubsystem.Position.AMP);
            if (isAmp) {
                mBenderAngleSubsystem.setBenderPosition(BenderPosition.SHOOT_AMP);
                mIO.setBenderFeedMotorVoltage(-BENDER_FEED_AMP_SPEED);
            }
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case FIRE:
                    changeState(mFireAmpState);
                    break;
                case SPIT:
                    changeState(mSpitState);
                    break;
                default:
                    System.out.println("Ignoring event " + event + " in ReadyForFireAmpState");
                    break;
            }
        }

        @Override
        public void run() {
            if ((Timer.getFPGATimestamp() - feedTimerStart) > AMP_PRELOAD_DELAY) {
                mIO.setBenderFeedMotorVoltage(0);
                if (mBenderAngleSubsystem.isInAmpLocation() &&
                        mShooterAngleSubsystem.isInAmpLocation() &&
                        mElevatorSubsystem.isAtAmp()) {
                    changeState(mReadyForFireAmpState);
                }
            }
        }
    }

    private class BenderLoadInternalLoadedTrapState extends State {
        public BenderLoadInternalLoadedTrapState() {
            super("Bender Load Internal - Loaded Trap");
        }

        @Override
        public void enterState() {
            mElevatorSubsystem.setPosition(ElevatorSubsystem.Position.AMP);
            mBenderAngleSubsystem.setBenderPosition(BenderPosition.PREP_TRAP);
        }

        @Override
        public void run() {
            mIO.setBenderFeedMotorVoltage(0);
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case PREP_TRAP:
                    changeState(mReadyForFireTrapState);
                    break;
                case SPIT:
                    changeState(mSpitState);
                    break;
                default:
                    System.out.println("Ignoring event " + event + " in LoadedTrap");
                    break;
            }
        }
    }

    private class ReadyForFireAmpState extends State {
        public ReadyForFireAmpState() {
            super("Ready For Fire Amp");
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case FIRE:
                    changeState(mFireAmpState);
                    break;
                case SPIT:
                    changeState(mSpitState);
                    break;
                default:
                    System.out.println("Ignoring event " + event + " in ReadyForFireAmpState");
                    break;
            }
        }
    }

    private class ReadyForFireTrapState extends State {
        public ReadyForFireTrapState() {
            super("Ready for Fire Trap");
        }

        @Override
        public void enterState() {
            mBenderAngleSubsystem.setBenderPosition(BenderPosition.SHOOT_TRAP);
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case FIRE:
                    changeState(mFireTrapState);
                    break;
                case SPIT:
                    changeState(mSpitState);
                    break;
                default:
                    System.out.println("Ignoring event " + event + " in ReadyForFireTrapState");
                    break;
            }
        }
    }

    private class FireAmpState extends State {
        public FireAmpState() {
            super("Fire Amp");
        }

        @Override
        public void enterState() {
            mIO.setBenderFeedMotorVoltage(BENDER_SHOOT_SPEED);
        }

        @Override
        public void exitState() {
            m_isLoaded = false;
            mElevatorSubsystem.setPosition(ElevatorSubsystem.Position.HOME);
            mBenderAngleSubsystem.setBenderPosition(BenderAngleSubsystem.BenderPosition.SHOOT_SPEAKER);
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case FIRE:
                    mIO.setBenderFeedMotorVoltage(0);
                    changeState(mEmptyState);
                    break;
                case SPIT:
                    changeState(mSpitState);
                    break;
                default:
                    System.out.println("Ignoring event " + event + " in FireAmpState");
                    break;
            }
        }
    }

    private class FireTrapState extends State {
        private FireTrapState() {
            super("Fire Trap");
        }

        @Override
        public void enterState() {
            mIO.setBenderFeedMotorVoltage(BENDER_SHOOT_SPEED);
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case FIRE:
                    mIO.setBenderFeedMotorVoltage(0);
                    changeState(mEmptyState);
                    break;
                case SPIT:
                    changeState(mSpitState);
                    break;
                default:
                    System.out.println("Ignoring event " + event + " in FireTrapState");
                    break;
            }
        }

        @Override
        public void exitState() {
            m_isLoaded = false;
            mBenderAngleSubsystem.setBenderPosition(BenderAngleSubsystem.BenderPosition.SHOOT_SPEAKER);
        }
    }

    private class PrepSourceState extends State {
        public PrepSourceState() {
            super("Prep Source");
        }

        @Override
        public void enterState() {
            setLEDOrange();
            mElevatorSubsystem.setPosition(ElevatorSubsystem.Position.SOURCE);
            mBenderAngleSubsystem.setBenderPosition(BenderPosition.LOAD_SOURCE);
            mShooterAngleSubsystem.setPosition(ShooterPosition.LOAD_SOURCE);
            feedSource();
        }

        @Override
        public void run() {
            if (inputs.isNoteAtTop) {
                changeState(mSourceLoadingState);
            }
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case SPIT:
                    changeState(mSpitState);
                    break;
                default:
                    System.out.println("Ignoring event " + event + " in Acquiring Bottom State");
            }
        }
    }

    private class SourceLoadingState extends State {
        public SourceLoadingState() {
            super("Source Loading");
        }

        @Override
        public void run() {
            if (!inputs.isNoteAtTop) {
                changeState(mSourceLoadedState);
            }
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case SPIT:
                    changeState(mSpitState);
                    break;
                default:
                    System.out.println("Ignoring event " + event + " in Acquiring Bottom State");
            }
        }
    }

    private class SourceLoadedState extends State {
        public SourceLoadedState() {
            super("Source Loaded");
        }

        @Override
        public void run() {
            if (inputs.isNoteAtMiddle) {
                changeState(mLoadedState);
                stopBenderMotor();
                prepSpeakerFront();
            }
        }

        @Override
        public void onEventInternal(OperatorEvent event) {
            switch (event) {
                case SPIT:
                    changeState(mSpitState);
                    break;
                default:
                    System.out.println("Ignoring event " + event + " in Acquiring Bottom State");
            }
        }
    }

    private static ShooterSubsystem mInstance = null;

    public static ShooterSubsystem getInstance() {
        if (mInstance == null) {
            switch (Constants.currentMode) {
                case REAL:
                    mInstance = new ShooterSubsystem(new ShooterIOHW());
                    break;
                case SIM:
                    mInstance = new ShooterSubsystem(new ShooterIOSim());
                    break;
                default:
                    mInstance = new ShooterSubsystem(new ShooterIO(){});
                    break;
            }
        }
        return mInstance;
    }

    private ShooterSubsystem(ShooterIO io) {
        mIO = io;

        io.setBrake(WhichMotor.BENDER_FEED);
        io.setBrake(WhichMotor.FLYWHEEL);
        io.setBrake(WhichMotor.FEED_WHEEL);

        mLed.setLength(mLedBuffer.getLength());
        mLed.setData(mLedBuffer);
        mLed.start();

        ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
        driverTab.addString("Shooter state", new Supplier<String>() {
            @Override
            public String get() {
                return currentState.toString();
            }
        }).withPosition(6, 1)
          .withSize(2, 1);
        mEmptyState.enterState();
    }

    @Override
    public void periodic() {
        mIO.updateInputs(inputs);
        Logger.processInputs("ShooterSubsystem", inputs);
        Logger.recordOutput("ShooterSubsystem/currentState", currentState.mName);
        currentState.run();
    }

    public void initStateMachine(boolean preloaded) {
        if (preloaded) {
            changeState(mLoadedState);
        } else {
            changeState(mEmptyState);
        }
    }

    private void changeState(State newState) {
        System.out.println("** Shooter State: " + currentState + " --> " + newState);
        currentState.exitState();
        newState.enterState();
        currentState = newState;
    }

    public boolean isHoldingNote() {
        return m_isLoaded;
    }

    private void prepAmp() {
        mBenderAngleSubsystem.setBenderPosition(BenderPosition.LOAD_INTERNAL);
        mShooterAngleSubsystem.setPosition(ShooterPosition.SHOOT_AMP);
    }

    private void prepSpeakerFront() {
        mBenderAngleSubsystem.setBenderPosition(BenderPosition.SHOOT_SPEAKER);
        mShooterAngleSubsystem.setPosition(ShooterPosition.SHOOT_SPEAKER_FRONT);
        mElevatorSubsystem.setPosition(Position.HOME);
    }

    private void prepSpeakerSide() {
        mBenderAngleSubsystem.setBenderPosition(BenderPosition.SHOOT_SPEAKER);
        mShooterAngleSubsystem.setPosition(ShooterPosition.SHOOT_SPEAKER_SIDE);
        mElevatorSubsystem.setPosition(Position.HOME);
    }

    public void stopFeedShootMotors() {
        mIO.setShooterFeedMotorVoltage(0);
        mIO.setFlywheelMotorVoltage(0);
        mIO.setGroundFeedMotorVoltage(0);
        mIO.setBeltMotorVoltage(0);
    }

    public void stopBottomMotor() {
        mIO.setGroundFeedMotorVoltage(0);
    }

    public void setLinearActuatorPosition(ShooterAngleSubsystem.ShooterPosition position) {
        mShooterAngleSubsystem.setPosition(position);
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

    private void stopAllMotors() {
        mIO.setGroundFeedMotorVoltage(0);
        mIO.setBeltMotorVoltage(0);
        mIO.setShooterFeedMotorVoltage(0);
        mIO.setFlywheelMotorVoltage(0);
        mIO.setBenderFeedMotorVoltage(0);
    }

    /**
     * This method is called to inform the ShooterSubsystem of operator inputs
     */
    public void operatorEvent(OperatorEvent event) {
        currentState.onEvent(event);
    }

    public void raiseElevator() {
        mElevatorSubsystem.raiseElevator();
    }

    public void stopElevator() {
        mElevatorSubsystem.stopMotor();
    }

    public void lowerElevator() {
        mElevatorSubsystem.lowerElevator();
    }

    /**
     * Is everything in the correct location to shoot the speaker from the side?
     */
    public boolean isReadyToShootSpeakerSide() {
        return (mBenderAngleSubsystem.isInSpeakerLocation() &&
                mShooterAngleSubsystem.isInSpeakerLocation_side());
    }

    /**
     * Is everything in the correct location to shoot the speaker from the front?
     */
    public boolean isReadyToShootSpeakerFront() {
        return (mBenderAngleSubsystem.isInSpeakerLocation() &&
                mShooterAngleSubsystem.isInSpeakerLocation_front());
    }

    public void raiseBender() {
        mBenderAngleSubsystem.setBenderPosition(BenderPosition.SHOOT_SPEAKER);
    }

    public void lowerBender() {
        mBenderAngleSubsystem.setBenderPosition(BenderPosition.LOAD_INTERNAL);
    }

    public void stopBenderMotor() {
        mIO.setBenderFeedMotorVoltage(0);
    }

    public void feedSource() {
        mIO.setBenderFeedMotorVoltage(BENDER_FEED_SOURCE_SPEED);
        mIO.setFlywheelMotorVoltage(SOURCE_FEED_SPEED);
        mIO.setShooterFeedMotorVoltage(SOURCE_FEED_SPEED);
        mIO.setBeltMotorVoltage(-SOURCE_FEED_SPEED);
    }

    public void forceLoaded() {
        changeState(mLoadedState);
    }

    public void forceEmpty() {
        changeState(mEmptyState);
    }
}
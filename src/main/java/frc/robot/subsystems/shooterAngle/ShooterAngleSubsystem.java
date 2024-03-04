package frc.robot.subsystems.shooterAngle;

import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.apriltags.AprilTagSubsystem;
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterAngleSubsystem extends PIDSubsystem {

    private DrivebaseSubsystem mDrivebaseSubsystem = DrivebaseSubsystem.getInstance();
    private AprilTagSubsystem mAprilTagSubsystem = AprilTagSubsystem.getInstance();

    private static final double SPEED_UP = -1;
    private static final double SPEED_DOWN = 1;
    private static final double kMotorVoltageLimit = 1;

    /** angle where shooter is able to shoot at the speaker */
    private final static double SHOOTER_ELEVATOR_LIMIT = 396;
    private final static double SHOOT_SPEAKER_SIDE_ANGLE = 391;
    private final static double SHOOT_SPEAKER_FRONT_ANGLE = 375;

    /** angle where shooter is able to shoot at the amp */
    private final static double SHOOT_AMP_ANGLE = 378;

    /** angle where shooter is positioned so we can go under the chain */
    private final static double UNDER_CHAIN_ANGLE = 411;

    /** angle where shooter is in the correct location to load from source */
    private final static double LOAD_SOURCE_HEIGHT = 370;

    private static final double kP = 0.1;
    private static final double kI = 0.1;
    private static final double kD = 0.01;
    private static final double kDt = 0.01;
    private static final PIDController pidController = new PIDController(kP, kI, kD, kDt);
    private static final double SHOOTER_ANGLE_THRESHOLD = 2;
    private static final double kIZone = 2;

    private final ShooterAngleIO mIO;
    private final ShooterAngleIOInputsAutoLogged inputs = new ShooterAngleIOInputsAutoLogged();

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

    private Pose2d mSpeakerPose = null;
    
    @AutoLogOutput
    private State mCurrentState = State.STOPPED;

    @AutoLogOutput
    private ShooterPosition m_currentTargetPosition = ShooterPosition.SHOOT_SPEAKER_FRONT;

    @AutoLogOutput
    private double mDistanceToSpeaker = 0;

    @AutoLogOutput
    private double mAutoAngle = 0;

    private static ShooterAngleSubsystem mInstance = null;

    public static ShooterAngleSubsystem getInstance() {
        if (mInstance == null) {
            switch (Constants.currentMode) {
                case REAL:
                    mInstance = new ShooterAngleSubsystem(new ShooterAngleIOHW());
                    break;
                case SIM:
                    mInstance = new ShooterAngleSubsystem(new ShooterAngleIOSim());
                    break;
                default:
                    mInstance = new ShooterAngleSubsystem(new ShooterAngleIO(){});
                    break;
            }
        }
        return mInstance;
    }

    private ShooterAngleSubsystem(ShooterAngleIO io) {
        super(pidController, 0);

        mIO = io;
        mIO.setBrakeMode();

        pidController.setTolerance(SHOOTER_ANGLE_THRESHOLD);
        pidController.setIZone(kIZone);

        mIO.setBrakeMode();
    }

    Optional<Alliance> lastAliance = Optional.empty();

    private void updateSpeakerTag() {
        Optional<Alliance> currentAliance = DriverStation.getAlliance();
        if (!currentAliance.equals(lastAliance)) {
            System.out.println ("Alliance Change: " + currentAliance);
            lastAliance = currentAliance;

            int speakerID = 7; // Blue speaker Apriltag ID
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                speakerID = 8;
            }
            mSpeakerPose = mAprilTagSubsystem.aprilTagFieldLayout.getTagPose(speakerID).get().toPose2d();
        }
    }

    @Override
    public void periodic() {
        mIO.updateInputs(inputs);
        super.periodic();

        Logger.processInputs("ShooterAngleSubsystem", inputs);
        Logger.recordOutput("ShooterAngleSubsystem/PID/setpoint", pidController.getSetpoint());
        Logger.recordOutput("ShooterAngleSubsystem/PID/positionError", pidController.getPositionError());
        Logger.recordOutput("ShooterAngleSubsystem/PID/atSetpoint", pidController.atSetpoint());
        
        if (DriverStation.isDisabled()) {
            disable();
            stopMotor();
            return;
        }

        if(getMeasurement() >= SHOOTER_ELEVATOR_LIMIT){
            ShooterSubsystem.isShooterLimit = true;
        }
        else {
            ShooterSubsystem.isShooterLimit = false;
        }

        if(mCurrentState == State.RAISING && inputs.isShooterAtTop){
            System.out.println("AT top and raising, stopping");
            mIO.setOutputPercentage(0);
            mCurrentState = State.STOPPED;
        }

        if (mCurrentState == State.RAISING && getMeasurement() < 380) {
            System.out.println("slow raising");
             mIO.setOutputPercentage(SPEED_UP / 2);
        }
    }


    private boolean atGoal() {
        return pidController.atSetpoint();
    }

    public void stopMotor() {
        // Stops shooter motor
        disable();
        mIO.setOutputPercentage(0);
    }

    public void raiseShooter() {
        // Moves the shooter
        System.out.println("called raise shooter");
        disable();
        if(!inputs.isShooterAtTop){
            if (getMeasurement() < 390) {
                System.out.println ("Slow up");
                mIO.setOutputPercentage( SPEED_UP / 2);
            } else {
                mIO.setOutputPercentage( SPEED_UP);
            }
            System.out.println("raising " + getMeasurement());
            mCurrentState = State.RAISING;
        }
        else{
            System.out.println("At TOP, stopping!");
            mIO.setOutputPercentage( 0);
            mCurrentState = State.STOPPED;
        }
    }

    public void lowerShooter() {
        // Moves the shooter down
        System.out.println("called lower shooter");
        if(ShooterSubsystem.isElevatorHome){
            disable();
            mIO.setOutputPercentage( SPEED_DOWN);
            System.out.println("lowering");
            mCurrentState = State.LOWERING;
        }
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
        double goal = 0;
        switch (m_currentTargetPosition) {
            case SHOOT_SPEAKER_FRONT:
                goal = SHOOT_SPEAKER_FRONT_ANGLE;
                break;
            case SHOOT_SPEAKER_SIDE:
                goal = SHOOT_SPEAKER_SIDE_ANGLE;
                break;
            case HEIGHT_CHAIN:
                goal = UNDER_CHAIN_ANGLE;
                break;
            case LOAD_SOURCE:
                goal = LOAD_SOURCE_HEIGHT;
                break;
            case SHOOT_AMP:
                goal = SHOOT_AMP_ANGLE;
                break;
            case SHOOT_PID:
                {
                    updateSpeakerTag();
                    if (mSpeakerPose != null) {
                        Pose2d currentPose = mDrivebaseSubsystem.getCurrentPose();
                        mDistanceToSpeaker = currentPose.getTranslation().getDistance(mSpeakerPose.getTranslation());

                        //y = -65.034x3 + 371.79x2 - 649.68x + 733.47
                        mAutoAngle  = (-65.034 * (mDistanceToSpeaker * mDistanceToSpeaker * mDistanceToSpeaker)) +
                            (371.79 * (mDistanceToSpeaker * mDistanceToSpeaker)) -
                            (649.68 * mDistanceToSpeaker) +
                            733.47;
                    }
                    goal = mAutoAngle;
                }
                break;
        }

        if (goal > getMeasurement()) {
            mCurrentState = State.RAISING;
        } else if (goal < getMeasurement()) {
            mCurrentState = State.LOWERING;
        } else {
            mCurrentState = State.STOPPED;
        }

        System.out.println("setting shooter angle: " + goal);
        this.m_controller.reset();
        pidController.setSetpoint(goal);
        enable();
    }

    @Override
    protected double getMeasurement() {
        return inputs.currentAngle;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        // clamp the output to a sane range
        Logger.recordOutput("ShooterAngleSubsystem/PID/output", output);
        double val;
        if (output < 0) {
            val = Math.max(-kMotorVoltageLimit, output);
        } else {
            val = Math.min(kMotorVoltageLimit, output);
        }
        if (inputs.isShooterAtTop && val < 0) {
            mIO.setOutputPercentage(0);
        } else {
            mIO.setOutputPercentage(val);
        }
    }
}
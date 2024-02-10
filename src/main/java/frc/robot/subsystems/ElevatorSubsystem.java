package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    
    private TalonFX mElevatorMotor;
    private DigitalInput mElevatorTopSensor;
    private DigitalInput mElevatorBottomSensor;
    private static final double elevatorSpeedUp = -0.3;
    private static final double elevatorSpeedDown = 0.3;
    enum State {
        STOPPED,
        RAISING,
        LOWERING,
    }
    private State currentState = State.STOPPED;
    
    public ElevatorSubsystem() {
        mElevatorMotor = new TalonFX(Constants.kElevatorMotorId);
        mElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        mElevatorTopSensor = new DigitalInput(Constants.kElevatorTopSensorId);
        mElevatorBottomSensor = new DigitalInput(Constants.kElevatorBottomSensorId);
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        SmartDashboard.putBoolean("ElevatorTopSensor", isElevatorAtTop());
        SmartDashboard.putBoolean("ElevatorBottomSensor", isElevatorAtBottom());
        // SmartDashboard.putNumber("Elevator Encoder", mElevatorMotor.getSelectedSensorPosition());
        // SmartDashboard.putNumber("Elevator motor", mElevatorMotor.getMotorOutputPercent());

        if(currentState == State.RAISING && isElevatorAtTop()) {
            stopMotor();
            currentState = State.STOPPED;
        }
        else if(currentState == State.LOWERING && isElevatorAtBottom()) {
            stopMotor();
            currentState = State.STOPPED;
        }

        // SmartDashboard.putNumber("Alt Encoder Velocity", mBenderEncoder.getVelocity());
        // SmartDashboard.putNumber("Applied Output", mBenderMotor.getAppliedOutput());
    }

    public boolean isElevatorAtBottom() {
        // Returns a boolean, opposite of elevator sensor.get
        // because it's inverted, false from sensor = there's something there
        // So we return the opposite, true means elevator at bottom
        return !mElevatorBottomSensor.get();
    }

    public boolean isElevatorAtTop() {
        // Returns a boolean, opposite of elevator sensor.get
        // because it's inverted, false from sensor = there's something there
        // So we return the opposite, true means elevator at top
        return !mElevatorTopSensor.get();
    }

    public void stopMotor() {
        // Stops elevator motor
        mElevatorMotor.set(0);
        currentState = State.STOPPED;
        System.out.println("stopped");
    }

    public void raiseElevator() {
        // Moves the elevator
        System.out.println("called raise elevator");
        if(!isElevatorAtTop()) {
            mElevatorMotor.set(elevatorSpeedUp);
            //mElevatorMotor.set(TalonSRXControlMode.Velocity, 100);
            currentState = State.RAISING;
            System.out.println("raising");
        }
        
    }

    public void lowerElevator() {
        // Moves the elevator down
        System.out.println("called lower elevator");
        if(!isElevatorAtBottom()) {
            mElevatorMotor.set(elevatorSpeedDown);
            currentState = State.LOWERING;
            System.out.println("lowering");
        }
    }

}

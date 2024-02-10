package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase{
    
    private TalonSRX mElevatorMotor;
    private DigitalInput mElevatorTopSensor;
    private DigitalInput mElevatorBottomSensor;
    
    public ElevatorSubsystem() {
        mElevatorMotor = new TalonSRX(Constants.kElevatorMotorId);
        mElevatorTopSensor = new DigitalInput(Constants.kElevatorTopSensorId);
        mElevatorBottomSensor = new DigitalInput(Constants.kElevatorBottomSensorId);
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        SmartDashboard.putBoolean("ElevatorTopSensor", isElevatorAtTop());
        SmartDashboard.putBoolean("ElevatorBottomSensor", isElevatorAtBottom());
        SmartDashboard.putNumber("Elevator Encoder", mElevatorMotor.getSelectedSensorPosition());

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



}

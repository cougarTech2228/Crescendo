package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.OperatorEvent;

public class ButtonBoardSubsystem extends SubsystemBase {
    private ClimberSubsystem climberSubsystem;

    private ElevatorSubsystem elevatorSubsystem;
    private ShooterSubsystem shooterSubsystem;

    private enum ButtonBoardOperationMode {
        Fine,
        Coarse
    }
    private ShooterSubsystem m_shooterSubsystem;


    private final int kJoystickChannel1 = 1;
    private final int kJoystickChannel2 = 2;

    private Joystick m_joystick1;
    private Joystick m_joystick2;

    private ButtonBoardOperationMode m_operationMode;

    public ButtonBoardSubsystem(ClimberSubsystem climberSubsystem, ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem) {
        m_joystick1 = new Joystick(kJoystickChannel1);
        m_joystick2 = new Joystick(kJoystickChannel2);
        this.climberSubsystem = climberSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.shooterSubsystem = shooterSubsystem;

    }
    

    // Joystick #1 Buttons

    private JoystickButton raiseElevatorButton() {
        return new JoystickButton(m_joystick1, 3);
    }

    private JoystickButton lowerElevatorButton() {
        return new JoystickButton(m_joystick1, 5);
    }

    private JoystickButton raiseClimberButton() {
        return new JoystickButton(m_joystick1, 4);
    }
    
    private JoystickButton lowerClimberButton() {
        return new JoystickButton(m_joystick1, 6);
    }

    private JoystickButton raiseLinearActuatorButton() {
        return new JoystickButton(m_joystick1, 2);
    }

    private JoystickButton lowerLinearActuatorButton() {
        return new JoystickButton(m_joystick1, 1);
    }

    private JoystickButton spitButton() {
        return new JoystickButton(m_joystick1, 8);
    }

    private JoystickButton fineCoarseSwitch() {
        return new JoystickButton(m_joystick1, 7);
    }    

    // Joystick #2 Buttons

    private JoystickButton prepSpeakerButton() {
        return new JoystickButton(m_joystick2, 1);
    }

    private JoystickButton shootSpeakerButton() {
        return new JoystickButton(m_joystick2, 2);
    }

    private JoystickButton prepAmpButton() {
        return new JoystickButton(m_joystick2, 3);
    }

    private JoystickButton shootAmpButton() {
        return new JoystickButton(m_joystick2, 4);
    }

    private JoystickButton blank5() {
        return new JoystickButton(m_joystick2, 5);
    }  

    private JoystickButton blank6() {
        return new JoystickButton(m_joystick2, 6);
    }

    private JoystickButton blank7() {
        return new JoystickButton(m_joystick2, 7);
    }

    private JoystickButton blank8() {
        return new JoystickButton(m_joystick2, 8);
    }

    private boolean isFineOperationMode() {
        return (m_operationMode == ButtonBoardOperationMode.Fine);
    }

    private void setOperationMode() {
        if (fineCoarseSwitch().getAsBoolean()) {
            m_operationMode = ButtonBoardOperationMode.Fine;
        } else {
            m_operationMode = ButtonBoardOperationMode.Coarse;
        }
    }

    @Override
    public void periodic() {
    }

    public void configureButtonBindings() {

        raiseElevatorButton().onTrue(
            new InstantCommand(() -> elevatorSubsystem.lowerElevator())
        );

        raiseElevatorButton().onFalse(
            new InstantCommand(() -> elevatorSubsystem.stopMotor())
        );
        
        lowerElevatorButton().onTrue(
            new InstantCommand(() -> elevatorSubsystem.raiseElevator())
        );

        lowerElevatorButton().onFalse(
            new InstantCommand(() -> elevatorSubsystem.stopMotor())
        );

        raiseClimberButton().onTrue(
            new InstantCommand(() -> climberSubsystem.raiseMotors())
        );

        raiseClimberButton().onFalse(
            new InstantCommand(() -> climberSubsystem.stopMotors())
        );

        lowerClimberButton().onTrue(
            new InstantCommand(() -> climberSubsystem.lowerMotors())
        );

        lowerClimberButton().onFalse(
            new InstantCommand(() -> climberSubsystem.stopMotors())
        );  

        raiseLinearActuatorButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("currently raising");
                shooterSubsystem.raiseLinearActuator();
            })
        );

        raiseLinearActuatorButton().onFalse(
            new InstantCommand(() -> shooterSubsystem.stopLinearActuator())
        );

        lowerLinearActuatorButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("currently lowering");
                shooterSubsystem.lowerLinearActuator();
            })
        );

        lowerLinearActuatorButton().onFalse(
            new InstantCommand(() -> shooterSubsystem.stopLinearActuator())
        );

        spitButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("Spit Button Pressed");
                shooterSubsystem.operatorEvent(OperatorEvent.SPIT);
            })
        ); 

        spitButton().onFalse(
            new InstantCommand(() -> {
                System.out.println("Spit Button Released");
                shooterSubsystem.operatorEvent(OperatorEvent.NONE);
            })  
        );

        prepSpeakerButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("Prep Speaker Button Pressed");
                shooterSubsystem.operatorEvent(OperatorEvent.PREP_SPEAKER);
            })
        );

        prepSpeakerButton().onFalse(
            new InstantCommand(() -> {
                System.out.println("Prep Speaker Button Released");
                shooterSubsystem.operatorEvent(OperatorEvent.NONE);
            })
        );

        shootSpeakerButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("Shoot Speaker Button Pressed");
                shooterSubsystem.operatorEvent(OperatorEvent.FIRE_SPEAKER);
            })
        );

        shootSpeakerButton().onFalse(
            new InstantCommand(() -> {
                System.out.println("Shoot Speaker Button Realeased");
                shooterSubsystem.operatorEvent(OperatorEvent.NONE);
            })
        );

        prepAmpButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("Prep Amp Button Pressed");
                shooterSubsystem.operatorEvent(OperatorEvent.PREP_AMP);
            })
        );

        prepAmpButton().onFalse(
            new InstantCommand(() -> {
                System.out.println("Prep Amp Button Realeased");
                shooterSubsystem.operatorEvent(OperatorEvent.NONE);
            })
        );

        shootAmpButton().onTrue(
            new InstantCommand(() -> {
                System.out.println("Shoot Amp Button Pressed");
                shooterSubsystem.operatorEvent(OperatorEvent.FIRE_AMP);
            })
        );

        shootAmpButton().onFalse(
            new InstantCommand(() -> {
                System.out.println("Shoot Amp Button Realeased");
                shooterSubsystem.operatorEvent(OperatorEvent.NONE);
            })
        );    
    }
}
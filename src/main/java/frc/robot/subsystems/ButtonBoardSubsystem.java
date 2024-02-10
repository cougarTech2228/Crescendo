package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class ButtonBoardSubsystem extends SubsystemBase {
    private ClimberSubsystem climberSubsystem;

    private ElevatorSubsystem elevatorSubsystem;

    private enum ButtonBoardOperationMode {
        Fine,
        Coarse
    }

    private final int kJoystickChannel1 = 1;
    private final int kJoystickChannel2 = 2;

    private Joystick m_joystick1;
    private Joystick m_joystick2;

    private ButtonBoardOperationMode m_operationMode;

    public ButtonBoardSubsystem(ClimberSubsystem climberSubsystem, ElevatorSubsystem elevatorSubsystem) {
        m_joystick1 = new Joystick(kJoystickChannel1);
        m_joystick2 = new Joystick(kJoystickChannel2);
        this.climberSubsystem = climberSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

    }
    private JoystickButton raiseClimberButton() {
        return new JoystickButton(m_joystick1, 3);
    }

    private JoystickButton lowerClimberButton() {
        return new JoystickButton(m_joystick1, 4);

    }

    // Joystick #1 Buttons

    private JoystickButton raiseElevatorButton() {
        return new JoystickButton(m_joystick1, 1);
    }

    private JoystickButton lowerElevatorButton() {
        return new JoystickButton(m_joystick1, 2);
    }

    // private JoystickButton getArmHomeButton() {
    //     return new JoystickButton(m_joystick1, 3);
    // }

    // private JoystickButton getArmLowButton() {
    //     return new JoystickButton(m_joystick1, 4);
    // }

    // private JoystickButton getConeFloorButton() {
    //     return new JoystickButton(m_joystick1, 5);
    // }

    // private JoystickButton getArmMiddleButton() {
    //     return new JoystickButton(m_joystick1, 6);
    // }

    // private JoystickButton getPickUpCubeButton() {
    //     return new JoystickButton(m_joystick1, 7);
    // }

    // private JoystickButton getArmHighButton() {
    //     return new JoystickButton(m_joystick1, 8);
    // }

    // // Joystick #2 Buttons

    // private JoystickButton getRotateLeftButton() {
    //     return new JoystickButton(m_joystick2, 1);
    // }

    // private JoystickButton getRotateRightButton() {
    //     return new JoystickButton(m_joystick2, 2);
    // }

    // private JoystickButton getArmUpButton() {
    //     return new JoystickButton(m_joystick2, 3);
    // }

    // private JoystickButton getArmRetractButton() {
    //     return new JoystickButton(m_joystick2, 6);
    // }

    // private JoystickButton getArmDownButton() {
    //     return new JoystickButton(m_joystick2, 5);
    // }

    // private JoystickButton getArmExtendButton() {
    //     return new JoystickButton(m_joystick2, 4);
    // }

    // private JoystickButton getOperationToggleSwitch() {
    //     return new JoystickButton(m_joystick2, 7);
    // }

    // private JoystickButton getToggleGripperButton() {
    //     return new JoystickButton(m_joystick2, 8);
    // }

    // private boolean isFineOperationMode() {
    //     return (m_operationMode == ButtonBoardOperationMode.Fine);
    // }

    // private void setOperationMode() {
    //     if (getOperationToggleSwitch().getAsBoolean()) {
    //         m_operationMode = ButtonBoardOperationMode.Fine;
    //     } else {
    //         m_operationMode = ButtonBoardOperationMode.Coarse;
    //     }
    // }

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
            new InstantCommand(() -> {
                climberSubsystem.raiseMotors();
            })
        );

        raiseClimberButton().onFalse(
            new InstantCommand(() -> {
                climberSubsystem.stopMotors();
            })
        );

        lowerClimberButton().onTrue(
            new InstantCommand(() -> {
                climberSubsystem.lowerMotors();
            })
        );

        lowerClimberButton().onFalse(
            new InstantCommand(() -> {
                climberSubsystem.stopMotors();
            })
        );  


    }
}
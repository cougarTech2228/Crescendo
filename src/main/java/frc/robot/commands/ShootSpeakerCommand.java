package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.OperatorEvent;

public class ShootSpeakerCommand extends Command {

    private ShooterSubsystem mShooter;
    private boolean mIsFront;
    public ShootSpeakerCommand(boolean isFront) {
        mIsFront = isFront;
        mShooter = ShooterSubsystem.getInstance();
    }
    @Override
    public void initialize() {
        System.out.println("initialize()");
        mShooter.operatorEvent(mIsFront ? OperatorEvent.PREP_SPEAKER_FRONT :
            OperatorEvent.PREP_SPEAKER_SIDE);
    }

    @Override
    public void execute() {
        if (mIsFront) {
            if (mShooter.isReadyToShootSpeakerFront()) {
                mShooter.operatorEvent(OperatorEvent.FIRE_SPEAKER);
            }
        } else {
            if (mShooter.isReadyToShootSpeakerSide()) {
                mShooter.operatorEvent(OperatorEvent.FIRE_SPEAKER);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("end()");
    }

    @Override
    public boolean isFinished() {
        boolean finished = !mShooter.isHoldingNote();
        if (finished) {
            System.out.println("Shoot Speaker Command finished!");
        }
        return finished;
    }
}

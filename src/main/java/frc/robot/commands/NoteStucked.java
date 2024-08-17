package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class NoteStucked extends SequentialCommandGroup {
    public NoteStucked(Arm arm, Intake intake) {
        super.addRequirements(arm, intake);

        addCommands(Commands.run(() -> arm.runSetPointProfiled(Arm.NOTE_STUCK_DEG), arm)
            .beforeStarting(() -> arm.runSetPointProfiled(Arm.NOTE_STUCK_DEG))
            .until(arm::inPosition));

        addCommands(Commands.run(intake::runReverse, intake)
            .alongWith(Commands.run(() -> arm.runSetPointProfiled(Arm.NOTE_STUCK_DEG), arm))
        );
    }
}

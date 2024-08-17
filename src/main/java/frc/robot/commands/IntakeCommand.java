package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeCommand extends SequentialCommandGroup {
    private final Intake intake;
    private final Arm arm;

    public IntakeCommand(Intake intake, Arm arm) {
        this.intake = intake;
        this.arm = arm;
        super.addRequirements(intake, arm);

        super.addCommands(
            Commands.run(() -> arm.runSetPointProfiled(Arm.INTAKE_POSITION_DEG), arm)
            .beforeStarting(() -> arm.runSetPointProfiled(Arm.INTAKE_POSITION_DEG))
            .until(arm::inPosition)
        );

        super.addCommands(intake.runIntakeUntilNotePresent().alongWith(Commands.run(() -> arm.runSetPointProfiled(Arm.INTAKE_POSITION_DEG), arm)));
    }
}

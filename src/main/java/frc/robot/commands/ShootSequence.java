package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;

public class ShootSequence extends SequentialCommandGroup {
    public ShootSequence(Pitch pitch, Shooter shooter, Intake intake) {
        addCommands(new AimAtSpeaker(pitch, shooter).withTimeout(1.5));
        addCommands(Commands.run(intake::runShoot).alongWith(new AimAtSpeaker(pitch, shooter)).withTimeout(0.8));
    }
}

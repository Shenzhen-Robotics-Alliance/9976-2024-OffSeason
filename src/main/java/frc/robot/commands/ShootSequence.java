package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootSequence extends SequentialCommandGroup{
    public ShootSequence(Arm arm, Intake intake, Shooter shooter) {
        super.addRequirements(arm, intake, shooter);

        super.addCommands(Commands.run(() -> {
            arm.runSetPointProfiled(Arm.SCORE_POSITION_DEG);
            shooter.runSpeakerShoot();
        }, arm, shooter).withTimeout(2));

        super.addCommands(Commands.run(intake::runShoot, intake).withTimeout(0.5));

        super.addCommands(Commands.run(() -> {
            intake.runIdle();
            arm.runSetPointProfiled(Arm.INTAKE_POSITION_DEG);
            shooter.runIdle();
        }, arm, shooter).withTimeout(2));
    }
}

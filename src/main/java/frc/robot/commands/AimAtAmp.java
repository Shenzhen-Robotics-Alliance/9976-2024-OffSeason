package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;

public class AimAtAmp extends Command {
    private final Pitch pitch;
    private final Shooter shooter;

    public AimAtAmp(Pitch pitch, Shooter shooter) {
        this.pitch = pitch;
        this.shooter = shooter;

        addRequirements(pitch, shooter);
    }

    @Override
    public void execute() {
        pitch.runSetPointProfiled(Pitch.AMP_POSITION_DEG);
        shooter.runAmpShot();
    }
}
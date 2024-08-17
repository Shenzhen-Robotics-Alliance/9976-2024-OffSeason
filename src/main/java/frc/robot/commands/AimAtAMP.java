package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class AimAtAMP extends Command{
    private final Arm arm;
    private final Shooter shooter;
    public AimAtAMP(Arm arm, Shooter shooter) {
        this.arm = arm;
        this.shooter = shooter;
        super.addRequirements(arm, shooter);
    }

    @Override
    public void execute() {
        this.arm.runSetPointProfiled(Arm.AMP_POSITION_DEG);
        this.shooter.runAmpShoot();
    }
    
}

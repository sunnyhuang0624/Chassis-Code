package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake_shoot;

public class intake_cmd extends Command {
    private intake_shoot intake_shooter = new intake_shoot();

    @Override
    public void initialize() {
        // intake_shooter.intake(intake_shooter.getdis());
    }
    
}

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake_shoot;

public class intake_shooter_cmd extends Command {
    final VictorSPX leftshooter = new VictorSPX(29);
    final VictorSPX rightshooter = new VictorSPX(28);
    final VictorSPX leftintake = new VictorSPX(24); // 23
    final VictorSPX rightintake = new VictorSPX(23); // 24

        @Override
        public void initialize() {
            double pretime = Timer.getFPGATimestamp();
            double nowtime = Timer.getFPGATimestamp();        
            while(nowtime - pretime < 2.0){
                rightshooter.set(VictorSPXControlMode.PercentOutput, -1);
                leftshooter.set(VictorSPXControlMode.PercentOutput, 1);
                rightintake.set(VictorSPXControlMode.PercentOutput, 0);
                leftintake.set(VictorSPXControlMode.PercentOutput, 0);
                nowtime = Timer.getFPGATimestamp();
            }
            while (nowtime - pretime > 1.5 && nowtime - pretime < 3.0) {
                rightshooter.set(VictorSPXControlMode.PercentOutput, -1);
                leftshooter.set(VictorSPXControlMode.PercentOutput, 1);
                rightintake.set(VictorSPXControlMode.PercentOutput, -1);
                leftintake.set(VictorSPXControlMode.PercentOutput, 1);
                nowtime = Timer.getFPGATimestamp();
            }
            while (nowtime - pretime > 1.0) {
                rightshooter.set(VictorSPXControlMode.PercentOutput, 0);
                leftshooter.set(VictorSPXControlMode.PercentOutput, 0);
                rightintake.set(VictorSPXControlMode.PercentOutput, 0);
                leftintake.set(VictorSPXControlMode.PercentOutput, 0);
                nowtime = Timer.getFPGATimestamp();
                break;
            }
        }
}

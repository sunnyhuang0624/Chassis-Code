package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake_shoot_prepare extends SubsystemBase {
    final VictorSPX right_handSpx = new VictorSPX(32);
    final VictorSPX left_handSpx = new VictorSPX(33); // 33
    final VictorSPX leftshooter = new VictorSPX(28);
    final VictorSPX rightshooter = new VictorSPX(29);
    final VictorSPX leftintake = new VictorSPX(23); // 23
    final DigitalInput photoSensor = new DigitalInput(9); // 光電感測器
    final VictorSPX leftlift = new VictorSPX(36);
    final VictorSPX rightlift = new VictorSPX(37);
    final Encoder enc = new Encoder(0, 1);
    int mode = 0;
    final Joystick controller2 = new Joystick(1);
    boolean photo = true;

    public void intake(double dist) {
        photo = true;
        right_handSpx.set(VictorSPXControlMode.PercentOutput, 0.02 * (180 - dist));
        left_handSpx.set(VictorSPXControlMode.PercentOutput, -0.02 * (180 - dist));
        if (photoSensor.get() == true && photo == true) {
            photo = true;
            leftintake.set(VictorSPXControlMode.PercentOutput, 0.7);
            if (controller2.getRawButton(8)) {
                rightshooter.set(VictorSPXControlMode.PercentOutput, 0);
                leftshooter.set(VictorSPXControlMode.PercentOutput, 0);
                leftintake.set(VictorSPXControlMode.PercentOutput, 0);
            }
        }

        while (photoSensor.get() == false) {
            photo = false;
            leftintake.set(VictorSPXControlMode.PercentOutput, -0.3);
            while (photoSensor.get() == true && photo == false) {
                stopevery();
            }
            photo = false;
        }
    }

    public void shooter() {
        //right_handSpx.set(VictorSPXControlMode.PercentOutput, 0.02 * (142 - dist));
        //left_handSpx.set(VictorSPXControlMode.PercentOutput, -0.02 * (142 - dist));
        double pretime = Timer.getFPGATimestamp();
            double nowtime = Timer.getFPGATimestamp();
        
            while(nowtime - pretime < 2.0){
                rightshooter.set(VictorSPXControlMode.PercentOutput, -1);
                leftshooter.set(VictorSPXControlMode.PercentOutput, 1);
                leftintake.set(VictorSPXControlMode.PercentOutput, 0);
                nowtime = Timer.getFPGATimestamp();
            }
            while (nowtime - pretime > 1.5 && nowtime - pretime < 3.0) {
                rightshooter.set(VictorSPXControlMode.PercentOutput, -1);
                leftshooter.set(VictorSPXControlMode.PercentOutput, 1);
                leftintake.set(VictorSPXControlMode.PercentOutput, 1);
                nowtime = Timer.getFPGATimestamp();
            }
            while (nowtime - pretime > 1.0) {
                rightshooter.set(VictorSPXControlMode.PercentOutput, 0);
                leftshooter.set(VictorSPXControlMode.PercentOutput, 0);
                leftintake.set(VictorSPXControlMode.PercentOutput, 0);
                nowtime = Timer.getFPGATimestamp();
                break;
            }
        }

    public void stopevery() {
        rightshooter.set(VictorSPXControlMode.PercentOutput, 0);
        leftshooter.set(VictorSPXControlMode.PercentOutput, 0);
        leftintake.set(VictorSPXControlMode.PercentOutput, 0);
    }

    public double getdis() {
        double dist = enc.getDistance();
        double dist_deg = dist * 360.0 / 2048;
        return dist_deg;
    }

    @Override
    public void periodic() {
        
        /*if (controller2.getRawButton(8)) {
            left_handSpx.set(VictorSPXControlMode.PercentOutput, -0.4);
            right_handSpx.set(VictorSPXControlMode.PercentOutput, 0.4);
        } else if (controller2.getRawButton(10)) {
            left_handSpx.set(VictorSPXControlMode.PercentOutput, 0.4);
            right_handSpx.set(VictorSPXControlMode.PercentOutput, -0.4);   
        } else {
            left_handSpx.set(VictorSPXControlMode.PercentOutput, 0);
            right_handSpx.set(VictorSPXControlMode.PercentOutput, 0);        
        } */
        
        double dist = enc.getDistance();
        // double dist2 = enc.getDistancePerPulse();
        double dist_deg = dist * 360.0 / 2048;
        mode = 0; //0
        boolean photo = true;

        if (controller2.getRawButton(5)) {
        // init & shoot
        // 0
        mode = 0;
        }
        if (controller2.getRawButton(6)) {
        // shoot amp
        // -90
        mode = 3;
        }
        if (controller2.getRawButton(4)) {
        // intake
        // 25
        mode = 2;
        }
        if (controller2.getRawButton(3)) {
        // 啟動前
        // 0
        mode = 1;
        }
        if (controller2.getRawButton(8)) {
            // lift
            mode = 4;
        }

        if (mode == 0) { // shoot & default
        double deg = SmartDashboard.getNumber("default_shoot_deg", 60);
        right_handSpx.set(VictorSPXControlMode.PercentOutput, 0.03 * (140 - dist_deg));
        left_handSpx.set(VictorSPXControlMode.PercentOutput, -0.03 * (140 - dist_deg));
        }

        if (mode == 1) { // start
        right_handSpx.set(VictorSPXControlMode.PercentOutput, 0.05 * (0 - dist_deg));
        left_handSpx.set(VictorSPXControlMode.PercentOutput, -0.05 * (0 - dist_deg)); 
        }
        
        if (mode == 2) {
        double deg = SmartDashboard.getNumber("intake_deg", 90);
        right_handSpx.set(VictorSPXControlMode.PercentOutput, 0.02 * (180 - dist_deg));
        left_handSpx.set(VictorSPXControlMode.PercentOutput, -0.02 * (180 - dist_deg));
        }
        if (mode == 3) {
        double deg = SmartDashboard.getNumber("amp_deg", 90);
        right_handSpx.set(VictorSPXControlMode.PercentOutput, 0.05 * (-50 - dist_deg));
        left_handSpx.set(VictorSPXControlMode.PercentOutput, -0.05 * (-50 - dist_deg));
        }

        SmartDashboard.putNumber("deg", dist_deg);

        if (controller2.getRawButton(1)) {
            double pretime = Timer.getFPGATimestamp();
            double nowtime = Timer.getFPGATimestamp();
            while(nowtime - pretime < 2.0){
                rightshooter.set(VictorSPXControlMode.PercentOutput, -1);
                leftshooter.set(VictorSPXControlMode.PercentOutput, 1);
                leftintake.set(VictorSPXControlMode.PercentOutput, 0);
                nowtime = Timer.getFPGATimestamp();
            }
            while (nowtime - pretime > 1.5 && nowtime - pretime < 3.0) {
                rightshooter.set(VictorSPXControlMode.PercentOutput, -1);
                leftshooter.set(VictorSPXControlMode.PercentOutput, 1);
                leftintake.set(VictorSPXControlMode.PercentOutput, 1);
                nowtime = Timer.getFPGATimestamp();
            }
            while (nowtime - pretime > 1.0) {
                rightshooter.set(VictorSPXControlMode.PercentOutput, 0);
                leftshooter.set(VictorSPXControlMode.PercentOutput, 0);
                leftintake.set(VictorSPXControlMode.PercentOutput, 0);
                nowtime = Timer.getFPGATimestamp();
                break;
            }
        }
            

        if (controller2.getRawButton(2)) {        
        photo = true;
        if (photoSensor.get() == true && photo == true) {
            photo = true;
            leftintake.set(VictorSPXControlMode.PercentOutput, 0.4);
            if (controller2.getRawButton(8)) {
                stopevery();
            }
        }

        while (photoSensor.get() == false) {
            photo = false;
            leftintake.set(VictorSPXControlMode.PercentOutput, -0.1);
            while (photoSensor.get() == true && photo == false) {
                stopevery();
            photo = true;
            }
        }
    } else {
        stopevery();
    }

    if (controller2.getRawButton(8)) {
        leftlift.set(ControlMode.PercentOutput, 0.5);
        rightlift.set(ControlMode.PercentOutput, 0.5);
    } else {
        leftlift.set(ControlMode.PercentOutput, 0);
        rightlift.set(ControlMode.PercentOutput, 0);

    }
    }
}

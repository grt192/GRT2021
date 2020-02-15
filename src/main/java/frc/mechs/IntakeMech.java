package frc.mechs;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import frc.gen.BIGData;

public class IntakeMech implements Mech {
    private TalonSRX motor;
    private Solenoid sol;

    public IntakeMech() {
        motor = new TalonSRX(BIGData.getInt("intake_talon_id"));
        motor.configContinuousCurrentLimit(10, 0);
        motor.configPeakCurrentLimit(15, 0);
        motor.configPeakCurrentDuration(100, 0);
        motor.enableCurrentLimit(true);
        //System.out.println(BIGData.getInt("intake_sol_id"));
        sol = new Solenoid(1, BIGData.getInt("intake_sol_id"));
    }

    @Override
    public void update() {
        boolean state = BIGData.getIntakeState();
        boolean disable = BIGData.getDisabled(1);
        // if (disable) {
        //     disable();
        // } else {
        //     motor.set(ControlMode.PercentOutput, state ? BIGData.getDouble("intake_speed") : 0);
        //     sol.set(state);
        // }

        if (state) {
            switch (BIGData.getInt("roller_mode")) {
            case 0:
                motor.set(ControlMode.PercentOutput, 0);
                break;
            case 1:
                motor.set(ControlMode.PercentOutput, BIGData.getDouble("intake_speed"));
                break;
            case 2:
                motor.set(ControlMode.PercentOutput, -BIGData.getDouble("intake_speed"));
                break;
            }
        } else {
            motor.set(ControlMode.PercentOutput, 0);
        }
        sol.set(state);
        //System.out.println(BIGData.getDouble("intake_speed"));
    }

    public void disable() {
        motor.set(ControlMode.Current, 0);
    }
}
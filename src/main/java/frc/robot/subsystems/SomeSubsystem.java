package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Variables;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class SomeSubsystem extends SubsystemBase {
    
    private WPI_TalonSRX motor1 = new WPI_TalonSRX(Constants.TALON_1_ID);
    private WPI_TalonSRX motor2 = new WPI_TalonSRX(Constants.TALON_2_ID);

    private boolean isRunning = false;

    public SomeSubsystem() {
        motor1.configFactoryDefault();
        motor2.configFactoryDefault();
    }

    public void set(double speed) {
        motor1.set(MathUtil.clamp(speed + Variables.driveBackspin, -1, 1));
        motor2.set(MathUtil.clamp(speed + Variables.driveBackspin, -1, 1));

        isRunning = true;
    }

    public void stop() {
        motor1.stopMotor();
        motor2.stopMotor();

        isRunning = false;
    }

    public void updateSpeed(double speed) {
        if (!isRunning) return;
        set(speed);
    }
}

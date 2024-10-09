package frc.robot.utils.motorsupplier;

import org.ejml.simple.UnsupportedOperation;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class TalonPwm extends MotorSupplier<Talon> {
    public TalonPwm(int port){
      super(port);
    }

    @Override
    public MotorSupplier<Talon> withVoltageComp() {
      throw new UnsupportedOperation();
    }

    @Override
    public MotorSupplier<Talon> withBrake(){
      throw new UnsupportedOperation();
    }

    @Override
    public Talon get() {
      if (port < 0) {
        System.out.println("MotorInfo : motor port num < 0, check port is defined : " + port);
        return new Talon(99);
      }
      Talon talon = new Talon(port);
      talon.setInverted(invert);

      
      talon.setSafetyEnabled(safety);
      return talon;
    }


}
  
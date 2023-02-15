package frc.robot.utilities;

import com.revrobotics.CANSparkMax;

public class CANSparkMaxCurrent extends CANSparkMax {

    public CANSparkMaxCurrent(int deviceId, MotorType type) {
        super(deviceId, type);
    }

    public void setCurrent(double amps) {
        //Math.abs(amps)
        this.setSmartCurrentLimit((int) Math.abs(amps));
        this.set(1 * Math.signum(amps));
    }

    public void setTorque(double nm) {

        double percentTorque = nm / 3.28;

        double currentLimit = percentTorque * 181;

        this.setCurrent(currentLimit);

    }

}

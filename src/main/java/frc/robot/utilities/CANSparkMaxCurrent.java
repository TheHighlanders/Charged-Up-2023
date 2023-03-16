package frc.robot.utilities;

import com.revrobotics.CANSparkMax;

public class CANSparkMaxCurrent extends CANSparkMax {

    public CANSparkMaxCurrent(int deviceId, MotorType type) {
        super(deviceId, type);
    }

    public void setCurrent(int amps) {
        //Math.abs(amps)
        this.setSmartCurrentLimit(Math.abs(amps));
        this.set(0.2 * Math.signum(amps));
    }

    public void setTorque(double nm) {

        double percentTorque = nm / 3.28;

        int currentLimit = (int)(percentTorque * 181);

        this.setCurrent(currentLimit);

    }

}

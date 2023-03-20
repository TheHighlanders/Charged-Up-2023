package frc.robot.utilities;

import com.revrobotics.CANSparkMax;

public class CANSparkMaxCurrent extends CANSparkMax {
    public int timeAbove;
    public double currentCurrent;
    public double[] currentBuffer;
    public double index;

    public double limitTo;
    public double spikeMaxAmps;
    public double spikeMaxTime;

    public boolean limiting;

    public int smartLimit;

    public CANSparkMaxCurrent(int deviceId, MotorType type) {
        super(deviceId, type);
        timeAbove = 0;
        index = 0;
        currentCurrent = this.getOutputCurrent();
        currentBuffer = new double[100];
        limiting = false;
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

    public void setSpikeCurrentLimit(double limitTo, double spikeMaxTime, double spikeMaxAmps, int smartLimit){
        this.limitTo = limitTo;
        this.spikeMaxTime = spikeMaxTime;
        this.spikeMaxAmps = spikeMaxAmps;
        this.smartLimit = smartLimit;

        limiting = true;
    }


    public void periodicLimit(){
        currentCurrent = this.getOutputCurrent();
        
        currentBuffer[(int) index] = currentCurrent;
        index++;
        index %= currentBuffer.length;
        
        if(currentCurrent >= spikeMaxAmps){
            limitNow();
        }

        if(timeAbove >= spikeMaxTime){
            limitNow();
        }

        if(currentCurrent >= limitTo){
            timeAbove++;
        } else {
            timeAbove--;
            timeAbove = Math.max(0,timeAbove);
            if(timeAbove == 0){
                this.setSmartCurrentLimit(150);
            }
        }
    }

    public void limitNow(){
        this.setSmartCurrentLimit(smartLimit);
    }
}
package frc.robot.utilities;

import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class SparkMaxPIDControllerSmart{
    private SparkMaxPIDController controller;
    private double setpoint;

    public SparkMaxPIDControllerSmart(SparkMaxPIDController pidController){
        controller = pidController;
    }

    public REVLibError setP(double p){
        return controller.setP(p);
    }

    public REVLibError setI(double i){
        return controller.setI(i);
    }

    public REVLibError setD(double d){
        return controller.setD(d);
    }

    public REVLibError setOutputRange(double min, double max){
        return controller.setOutputRange(min, max);
    }

    public REVLibError setReference(double setpoint, ControlType controlType){
        this.setpoint = setpoint;
        return controller.setReference(setpoint, controlType);
    }

    public REVLibError setMaxI(double max){
        return controller.setIMaxAccum(max, 0);
    }

    public double getSetpoint(){
        return setpoint;
    }
    
}

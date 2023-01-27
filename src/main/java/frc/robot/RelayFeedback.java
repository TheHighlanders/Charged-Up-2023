package frc.robot;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay;

public class RelayFeedback {
    private PIDController pidController;
    private double setpoint;

    public void relayFeedbackTunePID(PIDController pidController, Encoder encoder, double setpoint) {
        pidController.setSetpoint(setpoint);

        while (!isOnTarget(encoder, setpoint)) {
            double currentPosition = encoder.getDistance();
            double error = pidController.getPositionError();

            if (error > 0) {
                if (currentPosition > setpoint) {
                    pidController.setP(pidController.getP() * 0.9);
                    pidController.setI(pidController.getI() * 0.9);
                    pidController.setD(pidController.getD() * 0.9);
                } else {
                    pidController.setP(pidController.getP() * 1.1);
                    pidController.setI(pidController.getI() * 1.1);
                    pidController.setD(pidController.getD() * 1.1);
                }
            }
        }
        pidController.setSetpoint(0);
    }

    public void tunePID(Encoder encoder) {
        pidController.setSetpoint(setpoint);
        while (!isOnTarget(encoder ,setpoint)) {
            pidController.calculate(setpoint);
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        pidController.setSetpoint(0);
    }

    public boolean isOnTarget(Encoder encoder, double setpoint) {
        double tolerance = 0.02;
        double currentPosition = encoder.getDistance();
        if (currentPosition >= setpoint - tolerance && currentPosition <= setpoint + tolerance) {
            return true;
        }
        return false;
    }
}
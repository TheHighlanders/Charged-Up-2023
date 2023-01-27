package frc.robot;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.math.controller.PIDController;

public class StepResponse {
    private PIDController pidController;
    private double setPoint;
    private double ts;
    public double tc;
    public double td;
    private Timer timer;
    private boolean isFinished;
    private boolean isStepInput; // differentiate between a step input and a ramp input

    private double[] processData;

    public StepResponse(PIDController pidController, double setPoint, double ts, boolean isStepInput) {
        this.pidController = pidController;
        this.setPoint = setPoint;
        this.ts = ts;
        this.isStepInput = isStepInput;
        timer = new Timer();
    }

    public void run() {
        pidController.setSetpoint(setPoint);
        double startTime = System.currentTimeMillis();
        isFinished = false;
        timer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                if (isFinished) {
                    timer.cancel();
                } else {
                    double time = System.currentTimeMillis() - startTime;
                    pidController.setSetpoint(setPoint);
                    if (time >= ts) {
                        timer.cancel();
                        pidController.setSetpoint(0);
                        isFinished = true;
                        processData = getProcessData();
                        tc = findTimeConstant(processData);
                        td = findDeadTime(processData);
                    }
                }
            }
        }, 0, 10);
    }

    private double[] getProcessData() {
        // code to get process data
        return processData;
    }
    // Inflection point
    private double findTimeConstant(double[] processData) {
        // Initialize variables to store the time and PV value at the inflection point
        double timeConstant = 0;
        double finalValue = processData[processData.length - 1];
        double PV_63 = finalValue * 0.632;

        // Search for the inflection point
        for (int i = 0; i < processData.length; i++) {
            if (processData[i] >= PV_63) {
                timeConstant = i * ts;
                break;
            }
        }
        return timeConstant;
    }
    // FOPDT model
    private double findDeadTime(double[] processData) {
        // Initialize variables to store the time and PV value at the inflection point
        double deadTime = 0;
        double finalValue = processData[processData.length - 1];
        double timeConstant = findTimeConstant(processData);

        if (timeConstant > 0) {
            deadTime = timeConstant / (1 - 0.632);
        }
        return deadTime;
    }
}
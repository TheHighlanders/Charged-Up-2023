package frc.robot.PID;

import java.util.ArrayList;
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

    private ArrayList<Double> processData;

    public StepResponse(PIDController pidController, double amplitude, double ts, ArrayList<Double> processData) {
        this.pidController = pidController;
        this.setPoint = amplitude;
        this.ts = ts;
        this.processData = processData;
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
                        pidController.setPID(0, 0, 0);
                        isFinished = true;
                        tc = findTimeConstant(processData);
                        td = findDeadTime(processData);
                    }
                }
            }
        }, 0, 10);
    }

    // Inflection point
    private double findTimeConstant(ArrayList<Double> processData) {
        // Initialize variables to store the time and PV value at the inflection point
        double timeConstant = 0;
        double finalValue = processData.get(processData.size() - 1);
        double PV_63 = finalValue * 0.632;

        // Search for the inflection point
        for (int i = 0; i < processData.size(); i++) {
            if (processData.get(i) >= PV_63) {
                timeConstant = i * ts;
                break;
            }
        }
        return timeConstant;
    }
    // FOPDT model
    private double findDeadTime(ArrayList<Double> processData) {
        // Initialize variables to store the time and PV value at the inflection point
        double deadTime = 0;
        double timeConstant = findTimeConstant(processData);

        if (timeConstant > 0) {
            deadTime = timeConstant / (1 - 0.632);
        }
        return deadTime;
    }
}
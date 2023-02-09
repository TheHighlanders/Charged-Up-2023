package frc.robot.PID;

import java.util.ArrayList;
import java.util.ArrayList;
import java.util.Set;
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

    public StepResponse(PIDController pidController, double amplitude, double ts, ArrayList<Double> list) {
        this.pidController = pidController;
        this.setPoint = amplitude;
        this.ts = ts;
        this.processData = list;
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
    private double findTimeConstant(ArrayList<Double> processData2) {
        // Initialize variables to store the time and PV value at the inflection point
        double timeConstant = 0;
        double finalValue = geValueAt(processData2, processData2.size() - 1);
        double PV_63 = finalValue * 0.632;

        // Search for the inflection point
        for (int i = 0; i < processData2.size(); i++) {
            if (geValueAt(processData2, i) >= PV_63) {
                timeConstant = i * ts;
                break;
            }
        }
        return timeConstant;
    }
    // FOPDT model
    private double findDeadTime(ArrayList<Double> processData2) {
        // Initialize variables to store the time and PV value at the inflection point
        double deadTime = 0;
        double timeConstant = findTimeConstant(processData2);

        if (timeConstant > 0) {
            deadTime = timeConstant / (1 - 0.632);
        }
        return deadTime;
    }

    public static double geValueAt(ArrayList<Double> set, int i) {
        int count = 0;
        for (Double value : set) {
            if (count == i) {
                return value;
            }
            count++;
        }
        throw new IndexOutOfBoundsException("Index not found in the set");
    }
}
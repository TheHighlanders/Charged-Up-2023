package frc.robot.PID;

import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.PID.PID;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.Timer;
import java.util.TimerTask;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotBase;

/*
 * PID loops work by continuously measuring the error between the desired angle and the current angle of each wheel
 * they adjust the output of the angle motor to reduce that error
 * 
 * PID loop consists of three components: the proportional, integral, and derivative terms
 * 
 * this can be represented with this function
 * controlOutput = Kp * currentError + Ki * accumulatedError + Kd * errorDerivative
 * -Kp, Ki, and Kd are constants that represent the proportional, integral, and derivative terms
 * -currentError is the difference between the desired angle and the current angle of the wheel
 * -accumulatedError is the sum of all previous errors over time
 * -errorDerivative is the rate of change of the error
 * 
 * in other words
 * the proportional term is used to correct the angle of the wheel in proportion to the error
 * (if the error is large, the angle motor will be commanded to make a large correction)
 * the integral term is used to correct for any residual error that is not corrected by the proportional term.
 * (if the angle of the wheel keeps drifting away from the desired angle, the integral term will gradually increase the output of the angle motor to bring the angle back to the desired setpoint)
 * the derivative term is used to correct for the rate of change of the error
 * (if the angle is moving away from the desired angle at a faster rate, the derivative term will increase the output of the angle motor to slow down the rate of change)
 * 
 * the difference between integral and derivative is that derivative predicts where the wheel is going and compensates for example if the wheel is getting pulled in the other direction
 * while the integral will notice that the wheel isn't in the desired state yet and compensate for example if the wheel has resistance stopping it from moving fast the integral will compensate
 * 
 * ==================================================
 * TUNING (do this on a carpet)
 * ==================================================
 * the two methods i found are "Ziegler-Nichols" and "trial and error", Ziegler-Nichols is a appealing approach that is based on a set of equations that use the process variable and the control variable
 * it gives an approximate value for the constants which then can be tuned using trial and error to get the optimal values
 * 
 * start by measuring the process variable (PV), (current angle of the swerve drive module)
 * start with an initial value of Kp = 0.1, Ki = 0, and Kd = 0
 * set the setpoint (SP), (desired angle of the swerve drive module) you can just use a controller
 * increase the Kp value until the system begins to oscillate(move or swing back and forth at a regular speed), this is called the "ultimate oscillation" method
 * once the system begins to oscillate, note the oscillation period (Tu) and the amplitude (A).
 * 
 * the oscillation period (Tu) is the time it takes for one complete oscillation of the system's response to occur, this would be the time it takes for the angle of the module to oscillate from its setpoint to a maximum deviation from the setpoint, and back to the setpoint again
 * The amplitude (A) is the magnitude of the deviation from the setpoint during one oscillation, this would be the difference in angle between the setpoint and the maximum deviation from the setpoint during one oscillation
 * 
 * to do this collect data on the angle of the swerve module over time as the system is oscillating using a print statement reading the encoders and time ms
 * put his data somewhere you can measure it (or use an FFT, see Sources)  and find the oscillations either by graphing it or just finding the peaks and measuring the time and amplitude of the oscillations identified
 * 
 * after Tu and A are found use these formulas to find the approximate Kp, Ki, and Kd
 * Kp = 0.6 * Tu / A
 * Ki = 2 * Kp / Tu
 * Kd = Kp * Tu / 8
 * 
 * tune these values to perfection afterwards and also try increasing the setpoint or load on the system to see how the PID loop responds
 * 
 * Sources: https://en.wikipedia.org/wiki/PID_controller, https://www.controleng.com/articles/understanding-derivative-in-pid-control/, https://www.controleng.com/articles/why-are-pid-loops-so-difficult-to-master/, https://en.wikipedia.org/wiki/Fast_Fourier_transform
 */

/*copy and paste for implementation 
   import frc.robot.PID.PID;

   SmartDashboard.putString("Theorteical optimal values", PID.calculateOptimalPIDValuesZN(pidController)); //ZN
   SmartDashboard.putString("Theorteical optimal values", PID.calculateOptimalPIDValuesTL(pidController)); //TL
   
*/
public class PID {
    private static double total;
    public static ArrayList<Double> list = new ArrayList<>(); // dynamic data structure
    private static PIDController pidController;

    public static void setPID(PIDController Controller) {
        pidController = Controller;
    }

    // Modified Ziegler-Nichols
    public static String calculateOptimalPIDValuesZN() {
        sortAndRemoveDuplicates(list);
        double amplitude = calculateAmplitude();

        // Obtain the process variable (PV) and control variable (CV) values 
        // CV value will depend in the pid you are testing, in this case I made it for the heading pid

        double pv = pidController.getSetpoint();
        DriverStation.reportWarning(list.toString(), false);

        // Calculate the ultimate gain (Ku) and period (Tu)
        double ku = (4 * amplitude) / pv;
        double tu = calculatePeriod();

        // Calculate the optimal PID values
        double kp = 0.6 * ku;
        double ki = 2 * kp / tu;
        double kd = kp * tu / 8;

        return kp + " " + ki + " " + kd + " " + tu + " " + ku + " " + amplitude;
    }

    // Tyreus-Luyben
    public static String calculateOptimalPIDValuesTL() {
        double amplitude = calculateAmplitude();
        StepResponse stepBro = new StepResponse(pidController, amplitude, 0.5, list);

        // setPoint is the setpoint to test 
        // ts is the time step of the step input signal (amount of time that elapses between each step change in the input signal)
        stepBro.run();

        // Obtain the process variable (PV), control variable (CV), time constant (Tc), and dead time (Td) values
        double td = stepBro.td;
        double tc = stepBro.tc;
        double pv = pidController.getSetpoint();

        // Calculate the ultimate gain (Ku) and period (Tu)
        double ku = (4 * amplitude) / pv;
        double tu = pidController.getPeriod();

        // Calculate the optimal PID values
        double kp = (ku * td) / (tc * 3 * (tu + td));
        double ki = kp / (tu + 2 * td);
        double kd = kp * (tu + 2 * td) / 8;

        return kp + " " + ki + " " + kd;
    }

    public static String calculateOscillatingPIDValuesZN() {
        // Obtain the process variable (PV) and control variable (CV) values
        double pv = pidController.getSetpoint();

        // Calculate the ultimate gain (Ku) and period (Tu)
        double amplitude = calculateAmplitude();
        double ku = amplitude / pv;
        double tu = calculatePeriod();

        // Calculate the PID values
        double kp = 0.5 * ku;
        double ki = kp / (tu * 0.5);
        double kd = kp * tu / 8;

        return kp + " " + ki + " " + kd;
    }

    public static double testNoiseLevel(int samples) {
        // accuracy of tests
        AnalogInput input = new AnalogInput(0);
        total = 0;
        timer.scheduleAtFixedRate(new TimerTask() {
            int i = 0;

            public void run() {
                total += input.getVoltage();
                i++;
                if (i >= samples) {
                    timer.cancel();
                }
            }
        }, 0, 10);
        double average = total / samples;
        return average;
    }

    /*
     * exaple data:{ 1.0, 2.0, 3.0, 2.0, 1.0, 0.0, -1.0, -2.0, -3.0, -2.0, -1.0 } returns 3.0
     * because the maximum value in the array is 3.0 and the minimum value is -3.0 
     * and the amplitude is calculated as the difference between the maximum and minimum values divided by 2, which is (3.0 - (-3.0)) / 2 = 3.0
    */
    public static double calculateAmplitude() {
        // large amplitude means higher instability
        double max = Double.MIN_VALUE;
        double min = Double.MAX_VALUE;
        for (double d : list) {
            max = Math.max(max, d);
            min = Math.min(min, d);
        }
        return (max - min) / 2;
    }

    public static void updatePeriods(double period) {
        
        list.add(period);
    }

    public static double calculatePeriod() {
        // long periods mean slow responce
        double[] diffs = new double[list.size() - 1];
        for (int i = 0; i < list.size() - 1; i++) {
            diffs[i] = StepResponse.geValueAt(list, i + 1) - StepResponse.geValueAt(list, i);
        }
        int start = 0;
        for (int i = 1; i < diffs.length; i++) {
            if (diffs[i] <= 0 && diffs[i - 1] > 0) {
                start = i;
                break;
            }
        }
        int end = start;
        for (int i = start + 1; i < diffs.length; i++) {
            if (diffs[i] >= 0 && diffs[i - 1] < 0) {
                end = i;
                break;
            }
        }
        return end - start;
    }

    public static void relayFeedbackTunePID(Encoder encoder, Relay relay,
            double setpoint) {
        pidController.setSetpoint(setpoint); // set the desired setpoint for the PID controller
        double error;
        boolean isClosed = false; // variable to keep track of relay state
        while (!isOnTarget(encoder, setpoint)) { // loop until the PID controller reaches the setpoint
            error = encoder.getDistance() - setpoint; // calculate the error
            if (error > 0.02) { // check if the error is greater than the tolerance
                if (!isClosed) { // if the relay is open
                    relay.set(Relay.Value.kForward); // close the relay
                    isClosed = true;
                }
            } else { // if the error is within the tolerance
                if (isClosed) { // if the relay is closed
                    relay.set(Relay.Value.kOff); // open the relay
                    isClosed = false;
                }
            }
            try {
                Thread.sleep(10); // wait for a bit before checking the error again
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        pidController.setPID(0, 0, 0); // disable the PID controller once it reaches the setpoint
        relay.set(Relay.Value.kOff); // open the relay once the setpoint is reached
    }

    public static boolean isOnTarget(Encoder encoder, double setpoint) {
        double tolerance = Constants.ModuleConstants.kAngleTolerance;
        double currentPosition = encoder.getDistance();
        if (currentPosition >= setpoint - tolerance && currentPosition <= setpoint + tolerance) {
            return true;
        }
        return false;
    }

    public static void sortAndRemoveDuplicates(ArrayList<Double> data) {
        for (int i = 1; i < data.size(); i++) {
            if (data.get(i) == data.get(i - 1)) {
                data.remove(i-1);
            }
        }
        for (int i = data.size() - 1; i > 0; i--) {
            if (data.get(i-1) == data.get(i)) {
                data.remove(i);
            }
        }
    }

}

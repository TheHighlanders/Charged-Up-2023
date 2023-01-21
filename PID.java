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
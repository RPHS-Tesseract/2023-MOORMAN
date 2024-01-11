package org.firstinspires.ftc.teamcode;

public class controllerPID {
    public class PIDController {
        private double kp;  // Proportional gain
        private double ki;  // Integral gain
        private double kd;  // Derivative gain

        private double setpoint;
        private double integral;
        private double previousError;

        public PIDController(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            this.setpoint = 0.0;
            this.integral = 0.0;
            this.previousError = 0.0;
        }

        public void setSetpoint(double setpoint) {
            this.setpoint = setpoint;
        }

        public double calculate(double processVariable) {
            double error = setpoint - processVariable;

            // Proportional term
            double proportional = kp * error;

            // Integral term
            integral += ki * error;

            // Derivative term
            double derivative = kd * (error - previousError);

            // Calculate the total output
            double output = proportional + integral + derivative;

            // Update previous error
            previousError = error;

            return output;
        }
    }
}

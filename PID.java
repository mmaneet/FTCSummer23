package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;


public class PID {

    private double val;
    private double target;
    private double error;

    private static final double Kp = 0.0004375;
    private static final double Ki = 0.0002;
    private static final double Kd = 0;
    private double last_error;

    private double integralSum = 0;
    private static final double CONTROL_LOOP_TIME_STEP = 0.2;

    private static double MAX_INTEGRAL_TERM = 800;

    public PID(double Val, double Target, ElapsedTime timer){
        this.val = Val;
        this.target = Target;
        this.error = this.target - this.val;
        this.last_error = this.error;
        this.integralSum += error * timer.seconds();
    }

    public double updatePID(double Val, double Target, ElapsedTime timer, double power){
        this.val = Val;
        this.target = Target;
        error = this.target - this.val;
        this.integralSum += error * CONTROL_LOOP_TIME_STEP;

        if (integralSum > MAX_INTEGRAL_TERM) {
            integralSum = MAX_INTEGRAL_TERM;
        } else if (integralSum < -MAX_INTEGRAL_TERM) {
            integralSum = -MAX_INTEGRAL_TERM;
        }

        if (Math.abs(target) <= 30) {
            return 0;
        } else if (Math.abs(error) < 30){
            return power;
        } else {
            return get_new_velocity(timer);
        }
    }

    private double get_new_velocity(ElapsedTime timer){

        double p = Kp * target;
        double i = Ki * integralSum;
        double d = Kd * ((error - last_error)/CONTROL_LOOP_TIME_STEP);

        double sum = p + i + d;
        last_error = error;
        return sum;
    }

}

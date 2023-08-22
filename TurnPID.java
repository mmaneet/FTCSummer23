package org.firstinspires.ftc.teamcode;

public class TurnPID {

    private double directionMag;
    private String direction;
    private final String wheel;
    private static final double target = 5;

    private static final double Kp = 0.0075;
    private static final double Ki = 0.001;
    private static final double Kd = 0;

    private double error;
    private double last_error;
    private double integralSum = 0;

    private static final double CONTROL_LOOP_TIME_STEP = 0.2;
    private static final double MAX_INTEGRAL_TERM = 150;

    public TurnPID(double directionMag, String direction, String wheel){
        this.directionMag = directionMag;
        this.direction = direction;
        this.wheel = wheel;
        error = Math.abs(target - this.directionMag);
        last_error = error;
        integralSum += error * CONTROL_LOOP_TIME_STEP;
    }

    public double updatePID(double directionMag, String direction){
        this.directionMag = directionMag;
        this.direction = direction;
        error = Math.abs(target - this.directionMag);
        integralSum += error * CONTROL_LOOP_TIME_STEP;

        if (integralSum > MAX_INTEGRAL_TERM) {
            integralSum = MAX_INTEGRAL_TERM;
        } else if (integralSum < -MAX_INTEGRAL_TERM) {
            integralSum = -MAX_INTEGRAL_TERM;
        }

        return get_new_velocity();
    }

    private double get_new_velocity(){

        final double p = Kp * error;
        final double i = Ki * integralSum;
        final double d = Kd * ((error - last_error)/CONTROL_LOOP_TIME_STEP);
        last_error = error;

        final double sum = p + i + d;
        if (sum < 0.001){
            return 0.4;
        }

        if (wheel.equals("left")){
            if (direction.equals("left")){
                return -sum;
            }
            else{
                return sum;
            }
        } else{
            if (direction.equals("left")){
                return sum;
            }
            else{
                return -sum;
            }
        }
    }

}

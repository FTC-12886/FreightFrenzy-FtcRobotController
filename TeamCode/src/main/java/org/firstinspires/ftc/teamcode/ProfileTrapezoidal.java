/* Trapezoidal Profiler
 * Jay Jasper
 */ 

package org.firstinspires.ftc.teamcode;

public class ProfileTrapezoidal {
    private double profile_acceleration;
    private double profile_speed;
    private double current_velocity;
    private double current_position; 
    private double current_acceleration;

    public ProfileTrapezoidal(double profile_speed_, double profile_acceleration_){
        profile_speed = profile_speed_;
        profile_acceleration = profile_acceleration_;

        current_velocity = 0;
        current_position = 0;
        current_acceleration = 0;
    }

    private double profile_trap_sign(double value){
        return (1.0 - 2.0*(value < 0));
    }

    private double profile_trap_abs(double value){
        return value*profile_trap_sign(value);
    }

    private double profile_trap_limit(double value, double low, double high){
        value = value > high ? high : value;
        value = value < low ? low : value;
        return value;
    }

    // given new raw input and time delta since last call (loop time), spit out a new smoothed value
    public double smooth(double goal, double dt){
        // close to goal - stay!
        if ((profile_trap_abs(goal - current_position) < 0.5*profile_acceleration*dt*dt)
            & (profile_trap_abs(current_velocity) <= profile_acceleration*dt)) {
            current_position = goal;
            current_velocity = 0;
            current_acceleration = 0;
        }

        // far from goal - move! 
        else{
            // distance and direction to get to end goal
            double direction_to_goal = profile_trap_sign(goal - current_position);

            // where do you end up if you try to stop right now?  
            double stop_time = profile_trap_abs(current_velocity) / profile_acceleration;
            double stop_position = 0.5*(-1.0*direction_to_goal*profile_acceleration)*stop_time*stop_time
                + current_velocity*stop_time + current_position;

            // perform correction for quantized time. more important at coarse quantization
            double stop_steps = stop_time / dt;
            double stop_distance_correction = stop_steps*0.5*profile_acceleration*dt*dt;
            stop_position += profile_trap_sign(current_velocity)*stop_distance_correction;

            double stop_position_error = stop_position - goal;

            // accelerate = -1 on final approach, +1 if falling short of the goal
            current_acceleration = -1 + 2*(stop_position_error*direction_to_goal < 0);

            // adjust velocity, limited
            current_velocity = profile_trap_limit(current_velocity + profile_acceleration*direction_to_goal*current_acceleration*dt, 
                                                        -profile_speed, profile_speed);
        
            // increment position target
            current_position = current_position + current_velocity*dt;
        }

        return current_position;
    }

}
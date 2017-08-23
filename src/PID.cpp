#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void PID::UpdateError(double cte) {//, double delta_t) {
    d_error = (cte - p_error); // /delta_t;
    p_error = cte;
    i_error += cte;// * delta_t;
    //Twiddle(cte);
}

double PID::TotalError() {
    double steer_value = -Kp * p_error - Ki * i_error - Kd * d_error; 
    if (steer_value > 1.) {
        return 1.;
    } else if (steer_value < -1.) {
        return -1.;
    }
    return steer_value;
}
/*
void PID::Twiddle(double cte) {
    tol = 0.2;
    double params[] = {p_error, i_error, d_error};

    if ((p_error + i_error + d_error) > tol) {
        return;
    }

    for (p_index = 0; p_index < 3; p_index++) {
        params[p_index] += params_d[p_index];
    }
    // cte = run()
    if (best_cte > cte) {
        best_cte = cte;
        params_d[p_index] *= 1.3;
    } else {
        params[p_index] -= 2 * params_d[p_index];
        //run
    }
}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws){
    std::string reset_msg = "42[\"reset\",{}]";
    ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
  }


    it = 0;
    while sum(dp) > tol:
        print("Iteration {}, best error = {}, p={}, dp={}".format(it, best_err, p, dp))
        
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)
            
            if best_err > err:
                best_err = err
                dp[i] *= 1.3
            else:
                p[i] -= 2*dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)
                
                if best_err > err:
                    best_err = err
                    dp[i] *= 1.3
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.7
                    
        it += 1
    return p, best_err
*/

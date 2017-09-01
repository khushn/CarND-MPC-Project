### Reflections on the project


#### Approach
We use a MPC controller, which is in simple terms curve fitting. 

##### Transformation of values to car co-ordinates
This is of paramount importance. Else we struggle with the CTE. The CTE depends on the difference between y coordinate and the refernce path (returned by the polyfit described below). 

(NTS: I struggled a fair bit, because of earlier thinking, I can just do it for display. Lessons learned the hard way). 

So we do the transformation of all the x,y values using the function shown below: 

<blockquote>
vector<double> transform(double mapx, double mapy, double vehx, double vehy, double psi) {

  vector<double> ret;

  double dx = mapx - vehx;

  double dy = mapy - vehy;

  double r = sqrt(dx*dx + dy*dy);

  double angle2 = atan2(mapy-vehy, mapx - vehx);

  double angle = angle2-psi;

  ret.push_back(r*cos(angle));

  ret.push_back(r*sin(angle));

  return ret;

}
</blockquote>

##### polyfit
We use the given set of x,y values and use the 
<code>auto coeffs = polyfit(xvals, yvals, 4); </code> 
to get a 4th order equation. 4th oder equation should take care of all kinds of crazy curves, as learnt in an earlier Math lesson. Also if the actual need is of a lesser order function, I believe the <code> polyfit() </code> method should take care of it, by returning high order coefficients as zero values.

##### State
The state comprises of 6 fields, which are: 

<code>
px_transformed, py_transformed, psi_transformed, v, cte, epsi;
</code>

The *_transformed names imply that they have been transformed to car coordinates.


##### Actuators
The actuators of the system are acceleration and steering angle. These are the main outputs of the curve fitting using <code> IPOPT/CppAD </code> libraries which are used in the MPC.cpp. 

Techincally the code of these libraries does curve fitting using the model, constraints and cost defined, and predicts the next N steps, and also gives the actuators at each step (so they are N - 1 in count)

##### Update equations
Are the ones described in class. Shown below: 

<blockquote>
fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);

fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);

fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);

fg[1 + v_start + t] = v1 - (v0 + a0 * dt);

fg[1 + cte_start + t] =
  cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));

fg[1 + epsi_start + t] =
  epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
</blockquote>

##### Cost 
The cost specification is the critical part of MPC. I used all the things suggested in the class - CTE, ePsy, velocity reference delta, acceleration (absolute and its change rate), steering angle (absolute and its change rate). 

The code for that is added in the <code> () </code> operator of FG_eval class.

##### Constraints 
Other constraints of the model are specificed in the <code> MPC::Solve() </code> method. 

##### Hyper parameters
The values to tune and N and dt (T is just N* dt). I was able to reach max velocity of 70 miles/hour . For N = 6 and dt = 0.072 seconds (72 milli seconds). I am checking in the code with the same values. 

The value of dt is particularly very important, as it approximates the next step of the vehicle, before we come back to the calculation again. And so it should plan for roughly around 100 msec (which is the latency time). Keeping it too short, results in Car dancing left/right on the track. Having it too high, is meaningless, as ideally it should be on the lower side for accuracy.

Another intersting point is. Its not necessary that if you have a higher N, then its better. As the math in IPOPT/CppAD just reduces the area between the curves, so with higher N, its possible it leaves a wide gap for beginning points and closes it for the farther points. Which actually happened with me, with my car going offroad just by increasing N, and everything else the same! 

Bottom line both N and dt have to be carefully selected.

##### Latency
It was advided to use a latency value of 100 milli seconds. This is to similate real world conditions, where actuators - Gas and steering wheel changes take some time to take effect. The effect of latency is incorporated, by adjusting the values got from the simulator by advancing all values  (px, py, psi. a, v) for a time of 100 msec. This is done using the same update equations.

##### Summary
I was able to reach a max speed of 70 mph. In which the Car was able to do multiple rounds while staying on the driveable portion. Albiet, the driving is a bit risque. Its pretty stable at speeds below or around 50 mph. I tried for 80 but was not successful. 

My main learning is that, this is primarily curve fitting. That is, trying to draw a curve parallel to the reference curve, having as less a gap in between them, but with some goals and constraints. The goals are things like having the car move at all, and at good speed. The constraints come in the form of real world issues e.g. steering angle within limit, driving to be bit smooth etc. etc. And also coupled with latency, the problem becomes indeed very difficult. 



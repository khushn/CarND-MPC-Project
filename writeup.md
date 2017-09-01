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

<code>
fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);

fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);

fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);

fg[1 + v_start + t] = v1 - (v0 + a0 * dt);

fg[1 + cte_start + t] =
  cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
  
fg[1 + epsi_start + t] =
  epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
</code>
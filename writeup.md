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
</blockquote>>

##### polyfit
We use the given set of x,y values and use the 
<code>auto coeffs = polyfit(xvals, yvals, 4); </code> 
to get a 4th order equation. 4th oder equation should take care of all kinds of crazy curves, as learnt in an earlier Math lesson. Also if the actual need is of a lesser order function, I believe the <code> polyfit() </code> method should take care of it, by returning high order coefficients as zero values.


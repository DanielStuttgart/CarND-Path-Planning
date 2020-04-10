# CarND-Path-Planning
Path-Planning part of Nanodegree "Self-driving car engineer"

## Getting Started
For getting in touch with term3-simulator I implemented a constant velocity of 0,5 m / 20 ms = 25 m/s: 
```c++
// "getting started" --> move with 50 km/h in heading direction
double dist_inc = 0.5;        // every frame, add 0.2 m to x; 0.5 m / 20 ms = 25 m/s = 90 km/h = 56 mph
for (int i = 0; i < 50; ++i) {
    next_x_vals.push_back(car_x + (dist_inc * i) * cos(deg2rad(car_yaw)));
    next_y_vals.push_back(car_y + (dist_inc * i) * sin(deg2rad(car_yaw)));
}
```
The second given example introduces a steering movement by PI / 100: 
```c++
// "more complex paths"
double pos_x;
double pos_y;
double angle;
int path_size = previous_path_x.size();

for (int i = 0; i < path_size; ++i) {
  next_x_vals.push_back(previous_path_x[i]);        // new path = old path
  next_y_vals.push_back(previous_path_y[i]);
}

if (path_size == 0) {                                 // if old path empty, first position = car_position
  pos_x = car_x;
  pos_y = car_y;
  angle = deg2rad(car_yaw);
}
else {
  pos_x = previous_path_x[path_size - 1];           // if old path exists, first position = last_position from old path
  pos_y = previous_path_y[path_size - 1];

  double pos_x2 = previous_path_x[path_size - 2];
  double pos_y2 = previous_path_y[path_size - 2];
  angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
}

double dist_inc = 0.5;
for (int i = 0; i < 50 - path_size; ++i) {            // for new path, add PI/100
  next_x_vals.push_back(pos_x + (dist_inc)*cos(angle + (i + 1) * (pi() / 100)));
  next_y_vals.push_back(pos_y + (dist_inc)*sin(angle + (i + 1) * (pi() / 100)));
  pos_x += (dist_inc)*cos(angle + (i + 1) * (pi() / 100));
  pos_y += (dist_inc)*sin(angle + (i + 1) * (pi() / 100));
}
```
Next step to stay in lane by using s, d - coordinates from loaded map (needed to change path `string map_file_ = "../../../data/highway_map.csv";`: 
```c++
// keep in lane example
double dist_inc = 0.5; 
for (int i = 0; i < 50; ++i) {
  double next_s = car_s + dist_inc * (i+1);           // car moves forward by dist_inc
  double next_d = 6;                                // car starts in middle lane = 1,5 lanes from middle line = 1,5 * 4 m = 6 m              
  vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  next_x_vals.push_back(xy[0]);
  next_y_vals.push_back(xy[1]);
}         
```
## Car following
```c++
// initialization within main()
double ref_vel = 0;           // start at 0.0 in order to avoid high gradients in acceleration

// car-following within onMessage()
// car-following part beneath --> set reference velocity (video until 48:56 minutes)

bool too_close = false;         // variable if target object gets too close (30 m)

// find ref_v to use --> no car ahead: ref_vel; or car_velocity --> look for relevant vehicles in my lane
for (int i = 0; i < sensor_fusion.size(); i++) {
  // sensor_fusion:
  // 0: car id  
  // 1: position in x
  // 2: position in y
  // 3: relative velocity in x-dir
  // 4: relative velocity in y-dir
  // 5: car's s-position (longitudinal)
  // 6: car's d-position (lateral)
  float d = sensor_fusion[i][6];        

  // if car is in my lane (lane width = 4 m
  if (d < (2 + 4 * lane + 2) && d >(2 + 4 * lane - 2)) {
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx * vx + vy * vy);
        double check_car_s = sensor_fusion[i][5];

        check_car_s += (double)prev_size * 0.02 * check_speed;    // if using previous points can project s value out

        // check s values greater than mine and s gap lower than 30 m
        if ((check_car_s > car_s) && (check_car_s - car_s) < 30) {                                                
            //ref_vel = 29.5;       // only sets speed stupidly to 29.5 mph, if object ahead is detected
            too_close = true;                 

            // if an object is detected and we are on one of the right lanes, go to lane 0
            // lane change is done in 30 m (seen in waypoints)
            if (lane > 0)
            {
                lane = 0;
            }
        }                    
  }
}

// if too close to object ahead, slow down; if far enough away, speed up
double max_vel = 49.5;
if (too_close) {          // if a car is ahead, decelerate
  ref_vel -= .224;        // 0.224 mph = 0.01 m/s
}
else if (ref_vel < max_vel) {
  ref_vel += .224;        // if no car is ahead, accelerate
}
```
Example List
* Part 1
  * Subpart 1a
  * Subpart 1b
* Part 2
* Part 3
  * Subpart 3a
  * Subpart 3b

*italic*
**bold**
_**bold and italic**_

Inline Code `code example`

Code Cells
```python
# function to find highes value in array
def max_val(array):
  max_val = array[0]
  max_id = 0
  for i in range(len(array)): 
    if array[i] > max_val: 
      max_val = array[i]
      max_id = i
```

Task List
- [X] Get familiar with markdown
- [ ] Write code
- [ ] Write markdown

Table
Description | Header | Date
------------|--------|------
Hello World | First Program | 29.03.2020
Snipper | Snipping Tool | 19.03.2020

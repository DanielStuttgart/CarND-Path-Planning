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

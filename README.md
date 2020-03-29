# CarND-Path-Planning
Path-Planning part of Nanodegree "Self-driving car engineer"

## Getting Started
For getting in touch with term3-simulator I implemented a constant velocity of 0,5 m / 20 ms = 5 m/s: 
```c++
// "getting started" --> move with 50 km/h in heading direction
double dist_inc = 0.5;        // every frame, add 0.2 m to x; 0.5 m / 20 ms = 25 m/s = 90 km/h = 56 mph
for (int i = 0; i < 50; ++i) {
    next_x_vals.push_back(car_x + (dist_inc * i) * cos(deg2rad(car_yaw)));
    next_y_vals.push_back(car_y + (dist_inc * i) * sin(deg2rad(car_yaw)));
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

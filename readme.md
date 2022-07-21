### Original source code

[Original: flappy-automation-test, in ROS-kinetic and python2](https://github.com/JohsBL/flappy_automation_test)

### My Demo



### My score

+ #### "Good performance with strong robustness!"
+ In 20 **consecutive** tests, the scores are as follows:

| Test No.   | 1  |  2  | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 
| ----   | :--:  | :--:  |:--:  |:--:  |:--:  |:--:  |:--:  |:--:  |:--:  |:--:  |
| Score   |  114 |   118 | 116| 116| 109|111| 110 | 114 |117| 112|


| Test No.   | 11  |  12  | 13 | 14 | 15 | 16 | 17 | 18 | 19 | 20 |
| ----   | :--:  | :--:  |:--:  |:--:  |:--:  |:--:  |:--:  |:--:  |:--:  |:--:  |
| Score   | 116  | 112 | 116| 115| 117| 113| 116 | 118 | 112| 115|

### My solution
1. Position integral --> get current position
	+ Exact Euler method works better than forward Euler method
2. Y-axis control --> move the bird to `self.goal_y`
	+ At first I use bang-bang control
		+ Problem: oscillation
	+ Then I add PD control when it's close to `self.goal_y`
		+ Problem: hard to estimate time cost `= self.goal_y_reach_count * DELTA_T`
	+ Because the game is discretized in 30 FPS, I finally choose MPC for motion control
		+ Benefit: faster response, more accurate time cost
3. Obstacle update --> find the gate to pass
	+ Set `self.pos_up` and `self.pos_low` for upper and lower bound of the gate
	+ Apply two 1-D-arrays (0/1:free/stone) to represent forward and backward obstacles it can scan
	+ Data processing:
		+ Raw processing --> Clustering:
			+ only forward; backward + forward; only backward
4. Bird status
	+ The bird has 4 status: `init`, `free`, `constrained`, `forecast`
	+ Set current status
	+ Take different actions for different status:
		+ `init`: y-axis: None; x-axis: Speed up
		+ `free`: y-axis: Move to the goal; x-axis: Adjust Vx
		+ `constrained`: y-axis: None; x-axis: slow down to `VX_SAFE`
		+ `forecast`: y-axis: None; x-axis: adjust vx to `Vx_desire`
5. Find the gate `self.goal_y`
	+ The gate width should be 0.5m
	+ Possible gate: detected free space width is in `0.5m < d < 0.5m(1+10%)`
		+ if only one gate --> pass
		+ if multiple gates --> go to the closest one
		+ if no such gates --> go to maximum free(unobserved) space

6. Publish Accelerations and save delta-x,y

# FloorSegmentation

![KITTI Point Cloud](https://github.com/SmallMunich/FloorSegmentation/blob/master/image/origin.jpg)

This Code Is From The Paper: [Fast segmentation of 3D point clouds: A paradigm on LiDAR data for autonomous vehicle applications(2017 IEEE ICRA)](https://ieeexplore.ieee.org/document/7989591/)

Implementation of the ground segmentation algorithm proposed in 
```
@inproceedings{D. Zermas2017,
  title={Fast segmentation of 3D point clouds: A paradigm on LiDAR data for autonomous vehicle applications},
  author={D. Zermas, I. Izzat and N. Papanikolopoulos},
  booktitle={International Conference on Robotics and Automation (ICRA), 2017 IEEE},
  pages={5067--5073},
  year={2017},
  organization={IEEE}
}
```

## How To Build 

```bash
 cd build 
 cmake ..
 make 

```

It's All.

## How To Run 

```bash
./test_floorseg 
```

And Then, You Can Find new pcd Files In The Build File.

### Visualize Floor & Not-Floor Point Cloud.

![Segmentation Floor Point Cloud](https://github.com/SmallMunich/FloorSegmentation/blob/master/image/floor.jpg)

![Segmentation No Floor Point Cloud](https://github.com/SmallMunich/FloorSegmentation/blob/master/image/nofloor.jpg)

### Ground Plane Fitting
- **sensor_height**  Sensor height above ground.
- **sensor_model**  Sensors Model Lines.
- **num_seg**  the number of segments.
- **num_iter**  the number of iterations.
- **num_seg**  the num of segments.
- **num_lpr**  the number points of seeds.
- **th_seeds**  the seeds threshold.
- **th_dist**  the max dist for floor points.


### More Details Write In Blog Address

* analyze this algorithm: 
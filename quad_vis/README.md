# quad_vis
This package has robot model, DH parameters and launch file that displays it in RVIZ.

# Usage
To generate URDF model:
```
cd quad_vis/
python3 dh2urdf.py
```
It will generate complete robot model given DH parameters in `quad_vis/legs_dh.csv`.

Then you can run `ros2 launch quad_vis quad_vis.launch.py` to display model.
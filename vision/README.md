## hardware
- OAK-D Pro W  detection + depth
- Raspberry Pi 5 host 

## to run
```bash
source ~/oak_env/bin/activate
python3 scripts/yolo_depth.py
```

## Output per detection
- Class name and OSHA waste category
- 3D position (X,Y,Z) in metres
- Principal axis for gripper orientation
- Grasp confidence score 0-1
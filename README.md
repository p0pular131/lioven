# lioven(lioSAM HEven)

## 1. Initial Match 
* 먼저 livoen params.yaml에서 globalMap, keyPose path 설정

* 시작 위치 잡는거면 useOdom -> 0, 중간에 다시 localization 하는거면 useOdom -> 1

```
roslaunch lioven initialMatch.launch
```

* 실행한 결과로 나온 gicp의 결과값 대입

```
[INFO] [1727957990.710005]: Saved first_scan.pcd
100%|██████████████████████████████████████████████| 360/360 [00:02<00:00, 167.12it/s]
intial_x: 1.2305603872255186
intial_y: 2.070963878512688
intial_z: 0.054528479681173664
intial_roll: 0.01939314451226561
intial_pitch: -0.015228127411551773
intial_yaw: 0.8433777031570235
```

## 2. Run Localization

```
roslaunch lioven run.launch
```
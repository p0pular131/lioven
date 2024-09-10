# lioven(lioSAM HEven)

## 1. Initial Match 
  * 먼저 src로 가서 initialmatch.py의 본선, 예선, 학교 map 중에서 어떤거 load할지 설정.

```
cd erp_ws/src/heven_m_m/lioven/src

# initialmatch.py안의 두 경로 확인
global_map_np = read_pcd_file("../data/bonsun/cloudGlobal.pcd")
key_poses = read_kitty_format_poses("../data/bonsun/optimized_poses.txt")
```
* 그 후 해당 directory에서 두 파이썬 파일 실행

```
python3 initialMatch.py
```

  * 실행한 결과로 나온 gicp의 결과값 대입

```
100%|███████████| 100/100 [00:00<00:00, 156.65it/s]
result: 6.3478284469361 12.009480060347185 -0.07002332106903895 0.005328962403867491 -0.009412060731663896 1.025610375710519
```
  * 위 예시에서나온 결과인 x=6.3478284469361, y=12.009480060347185, z=-0.07002332106903895 roll=0.005328962403867491, pitch =-0.009412060731663896, yaw=1.025610375710519를 아래와같이 대입하면 됩니다.
    - intial_x: 6.3478284469361
    - intial_y: 12.009480060347185
    - intial_z: -0.07002332106903895
    - intial_roll: 0.005328962403867491
    - intial_pitch: -0.009412060731663896
    - initial_yaw: 1.025610375710519

## 2. Run Localization
* param.yaml에서 마찬가지로 본선, 예선, 학교 map 경로 확인

```
  PathCornerMap: "/home/heven/erp_ws/src/heven_m_m/lioven/data/bonsun/cloudCorner.pcd" 
  PathSurfMap: "/home/heven/erp_ws/src/heven_m_m/lioven/data/bonsun/cloudSurf.pcd"
```
* 런치파일 실행
```
roslaunch lioven run.launch
```
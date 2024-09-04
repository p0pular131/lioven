# lioven(lioSAM HEven)

## 1. Initial Match 
  * gps값을 통해서 대략적인 현재 <-> map 간의 원점 차이값 match_x, match_y에 넣어주기

```
roslaunch lio_sam sensor.launch
roslaunch lioven initialMatch.launch
```

  * 실행한 결과로 나온 gicp의 결과값 중, 점수가 가장 좋은 걸로 initial값 선정

```
[ INFO] [1724392065.489579823]: ----> Loading for Map . . .
[ INFO] [1724392068.749525996]: Global map loaded with 2226270 points ! ! 
[ INFO] [1724392099.629827834]: First Scan Sub !
[ INFO] [1724392103.425851837]: GICP Score at direction [x=6.800000, y=1.500000, z=0.000000]: 0.150382
[ INFO] [1724392107.147062246]: GICP Score at direction [x=7.100000, y=1.500000, z=0.000000]: 0.150573
[ INFO] [1724392110.925442639]: GICP Score at direction [x=6.800000, y=1.600000, z=0.000000]: 0.150564
[ INFO] [1724392114.716214568]: GICP Score at direction [x=7.100000, y=1.600000, z=0.000000]: 0.151188
[ INFO] [1724392118.579231784]: GICP Score at direction [x=7.300000, y=1.500000, z=0.000000]: 0.343354
[ INFO] [1724392122.181038304]: GICP Score at direction [x=6.800000, y=1.600000, z=0.000000]: 0.150564
[ INFO] [1724392125.956187441]: GICP Score at direction [x=7.300000, y=1.600000, z=0.000000]: 0.365687
[ INFO] [1724392125.962752922]: Best GICP Score: 0.150382
[ INFO] [1724392125.962780655]: Best Transformation Matrix:
  0.988493    0.151238 -0.00299957   -0.106475
  -0.151215    0.987424  -0.0461216   -0.169805
-0.00401349   0.0460444    0.998931   -0.121885
          0           0           0           1
[ INFO] [1724392125.962786709]: Best Transformation: [Roll, Pitch, Yaw, X, Y, Z]: [0.046138, -0.003000, -0.151821, -0.106475, -0.169805, -0.121885]
```
  * 위 예시에서는 GICP Score: 0.150382인  x=6.800000, y=1.500000, z=0.000000 에다가 gicp 결과값인 -0.106475, -0.169805, -0.121885을 더해서 아래의 값들을 대입
    - intial_x: 6.6935325
    - intial_y: 1.430195
    - intial_z: -0.121885

## 2. Run Localization

```
roslaunch lioven run.launch
```
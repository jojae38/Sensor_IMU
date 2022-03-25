# Sensor_IMU

# E2BOX EBIMU-9DOFV5 센서 (데이터 취득 성공)

## 1. 결선 data 받아보기
### 1-1. Windows 에서 hyperterminal 사용하여 매뉴얼에 나온 명령어 입력
*  \<sor30> 을 통해 30Hz로 데이터를 받아옴 
*  \<sof2> 를 통해 Quaternion으로 데이터를 받아옴 ( 초기값 Euler )
* Gyroscope과 Accelerometer가 가장 중요. min 10Hz ~ 15Hz max 30Hz 로 데이터 받아올 수 있도록
* rosparam으로 특정값을 입력할 수 있을지
* Quaternion은 gyroscope과 accelerometer로 계산해서 얻을 수 있지만, IMU에서 쏴주는 데이터도 얻을 수 있게 하면 좋을 듯  +  timestamp도 마찬가지 (secondary) 
* 30Hz 출력속도를 위해 \<sor33> 커맨드 사용. 정확히 30Hz는 아닙니다.

## 2. serial.h 클래스를 사용하여 imu 센서 데이터를 처음 상태에서 세팅 후 값을 읽어오기
### 2-1. serial_imu 노드를 생성하여 초기 세팅값을 파라미터로 받아옴
* \<sor0> 명령어를 통해 polling 모드로 진입하게 합니다. 이를 토대로 while(ros::ok()) 루프의 rate를 param으로 설정하여 사용자가 원하는 속도로 센서 데이터를 출력할 수 있습니다.
* \<sof2> 명령어를 통해 오일러 각도 단위를 쿼터니언으로 변환하여 출력하도록 합니다.
* \<sog1> 명령어를 통해 자이로 센서 데이터를 출력하도록 설정합니다.
* \<soa1> 명령어를 통해 가속도 센서 데이터를 출력하도록 설정합니다.
<img src="https://cogaplex.synology.me/jkang/EBIMU-9DOFV5/initial_value.png" width="40%"/>
* 센서의 초기에는 오일러 각도데이터 roll, pitch, yaw만 10hz로 출력하는데, 위의 명령어 설정을 통해 원하는 값을 출력하도록 비휘발성 메모리에 저장합니다. 이후 노드가 켜질때마다 명령어가 실행되기는 하나, 기본적으로 처음 세팅이 되면 메모리에 남아있게 됩니다.

### 2-2. polling 모드
<img src="https://cogaplex.synology.me/jkang/EBIMU-9DOFV5/polling_mode.png" />
* "* "라는 헤더를 write하여 센서로부터 즉시 데이터를 취득합니다.

### 2-3. set output format -> Quarternion
<img src="https://cogaplex.synology.me/jkang/EBIMU-9DOFV5/set_output_format.png"/>
* 공장 초기 설정이 오일러 각도로 되어있어, sensor_msgs/Imu 메시지로 퍼블리시 하기 위해 quarternion으로 변경하였습니다.

### 2-4. set output gyro
<img src="https://cogaplex.synology.me/jkang/EBIMU-9DOFV5/set_output_gyro.png" />
* 자이로 센서 값을 출력하도록 설정합니다. 

### 2-5. set output accelero
<img src="https://cogaplex.synology.me/jkang/EBIMU-9DOFV5/set_output_accelero.png" />
* 가속도 센서 값을 출력하도록 설정합니다.

### 2-6. 결과 출력
<img src="https://cogaplex.synology.me/jkang/EBIMU-9DOFV5/result_terminal.png" width="60%"/>
* 읽어온 값을 sensor_msgs/Imu 메시지로 퍼블리시 합니다
<img src="https://cogaplex.synology.me/jkang/EBIMU-9DOFV5/rostopic_echo.png" width="60%"/>

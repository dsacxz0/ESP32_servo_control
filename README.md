# 使用uROS控制servo
## 步驟
1. 將 ESP32 以有線方式連接電腦

2. 將此 repo clone 進 WSL2 並使用 VScode 開啟

3. 待 PlatformIO IDE 設定完成後修改 ```src/main.cpp``` 的變數
    - ```ssid``` 為欲連接 WiFi 名稱
    - ```password``` 為欲連接 WiFi 密碼
    - ```agent_ip``` 為 micro-ROS agent (電腦) 的 ip，可透過 ```ipconfig``` 指令查看

4. 修改後將程式燒錄進 ESP32

5. 開啟 Docker Desktop 並在此專案目錄下輸入
    ```bash
    cd servo_publisher
    bash run_agent.sh
    ```
    在輸入完成後 micro-ROS agent 將會啟動

6. 檢查 ESP32 使否有與 micro-ROS agent 連接上，若沒有可按 EN 按鈕重新嘗試連接，若仍不行可以透過 monitor logging 查看問題所在，若連接成功 micro-ROS agent logging 會出現以下畫面

   ![ros_agent_logging](https://github.com/Steven0811/ESP32_servo_control/blob/uROS_STA_connection_control/.github/assets/ros_agent_logging.png)

7. 在 ESP32 連接成功後在此專案目錄下開啟另一個 bash 並輸入
   ```bash
   cd servo_publisher
   bash run_publisher.sh
   ```
   在輸入完成後會進入到一個 Docker container 裡面

8. 輸入
   ```bash
   export ROS_DOMAIN_ID=0
   ```
   將 ROS domain ID 設為 0
   
9. 輸入
   ```bash
   ros2 topic list
   ```
   查看所有的 ROS Topic，會出現以下畫面

   ![ros_topic_list](https://github.com/Steven0811/ESP32_servo_control/blob/uROS_STA_connection_control/.github/assets/ros_topic_list.png)

   若沒有出現 ```/servo_trajectory``` 代表 ESP32 仍未與 micro-ROS agent 連接上

10. 輸入
    ```bash
    r
    ros2 run servo_publisher servo_trajectory_pub
    ```
    即可控制舵機

    ![control_servo](https://github.com/Steven0811/ESP32_servo_control/blob/uROS_STA_connection_control/.github/assets/control_servo.png)
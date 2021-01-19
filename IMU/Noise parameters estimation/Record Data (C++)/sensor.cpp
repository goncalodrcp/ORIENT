#include "sensor.h"

Sensor::Sensor() {
    manager = LpmsSensorManagerFactory();
    rotm = Mat::zeros(cv::Size(3,3), DataType<double>::type);
    quat = Mat::zeros(cv::Size(4,1), DataType<double>::type);
    eul = Mat::zeros(cv::Size(3,1), DataType<double>::type);
    acc = Mat::zeros(cv::Size(3,1), DataType<double>::type);
    gyro = Mat::zeros(cv::Size(3,1), DataType<double>::type);
    chrono::high_resolution_clock::time_point timestamp; 
    float sensorTime;
}

bool Sensor::Connect(const char* _id, int _type) {
    int count = 0;
    sensor = manager->addSensor(_type, _id);
    while(1) {
        cout << YELLOW << "STATUS : " << RESET << "Sensor is connecting ..." << endl;
        if(count > 5)
            return false;
        if(sensor->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED) {
            return true;
        }
        this_thread::sleep_for(std::chrono::seconds(5));
        count ++;
    }
}

bool Sensor::GetData() {
    if(sensor->getConnectionStatus() != SENSOR_CONNECTION_CONNECTED){
        cout << RED << "IMU not connected" << endl;
        cout << RESET << "SENSOR STATUS:" << sensor->getConnectionStatus() << endl;
        return false;
    }
    if(!sensor->hasImuData()){
        cout << RED << "IMU doesn't have data!" << endl;
        return false;
    }
    
    // Get data from the IMU
    data = sensor->getCurrentData();

    // Rotation matrix data
    int count = 0;
    for(int i = 1; i < 3; i++) {
        for(int j = 1; j < 3; j++) {
            rotm.at<double>(i,j) = data.rotationM[count];
            count ++;
        }
    }
    // Quaternion data
    for(int i = 0; i < 4; i++) {
        quat.at<double>(i) = data.q[i];
    }
    // Euler angle data
    eul.at<double>(0) = data.r[2];
    eul.at<double>(1) = data.r[1];
    eul.at<double>(2) = data.r[0];
    // Extract raw data from the sensor
    // What is the difference between this and the calibrated raw data?
    for(int i = 0; i < 3; i++){
        acc.at<double>(i)= data.aRaw[i];
        gyro.at<double>(i) = data.gRaw[i];
    }

    // Get timestamps
    timestamp = chrono::high_resolution_clock::now();
    sensorTime = data.timeStamp;

    return true;
}


void Sensor::Disconnect() {
    manager->removeSensor(sensor);
    delete manager;
}



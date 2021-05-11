#include "camera.h"
#include "image.h"
#include "sensor.h"
#include "defs.h"

#define online 0

void ThrowError(string what) {
    cout << RED << "(main.cpp) ERROR : " << RESET << what << endl;
    exit(EXIT_FAILURE);
}

int main() {

    // Variables
    Sensor mySensor;
    Mat quat;
    bool ret;
    string dir;
    Mat acc, gyro;
    chrono::high_resolution_clock::time_point timestamp;
    
    dir = "/media/goncalopereira/DATA/IST/ORIENT_repos/Tests/Mariana/C++examples/SensorRecordings/";
    ofstream out(dir+"quaternion.data");
    ofstream raw(dir+"raw.data");

    // Connect sensor
    ret = mySensor.Connect("A5014194", DEVICE_LPMS_U);
    if (!ret) ThrowError("Sensor not connected");
    cout << YELLOW << "STATUS : " << RESET << "Sensor connected" << endl;

    getchar();

    auto start = std::chrono::steady_clock::now();

    while(1) {

        //TODO: Add raw data record
        // Change sampling rates, filter constants, etc
        // Get orientation from IMU
        ret = mySensor.GetData();
        if (!ret) ThrowError("Sensor was not able to get orientation");
        cout << YELLOW << "STATUS : " << RESET << "Orientation obtained" << endl;
        mySensor.quat.copyTo(quat);
        mySensor.acc.copyTo(acc);
        mySensor.gyro.copyTo(gyro);
        timestamp = mySensor.timestamp;
        auto instant = chrono::duration_cast<chrono::nanoseconds>(timestamp.time_since_epoch()).count();
        cout << BLUE << "IMU (degrees)" << RESET << quat << endl;
        cout << RED << "Timestamp:" << RESET << instant << endl;
        out << instant << ", " << format(quat, Formatter::FMT_CSV) << endl;
        raw << instant << ", " << format(acc, Formatter::FMT_CSV) << ", " << format(gyro, Formatter::FMT_CSV) << endl;
        // Print sampling time of sensor
        // cout << GREEN << mySensor.sensorTime << endl;

        
        auto end = std::chrono::steady_clock::now();
        auto duration = chrono::duration_cast<std::chrono::minutes>(end - start).count();
        if(duration >= 240){
            cout << GREEN << "Time's up!" << endl;
            cout << RESET << "Elapsed time:" << duration << " minutes." << endl;
            break;
        }

        this_thread::sleep_for(chrono::milliseconds(10));

    }

    out.close();
    raw.close();
    return 0;
}

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>

class ThermalCamera {
private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
    std::vector<uint8_t> dataBuffer;

public:
    ThermalCamera(const std::string& port) : serial(io, port) {
        serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
        serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    }

    void configureModule() {
        writeData({ 0xA5, 0x25, 0x01, 0xCB });
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        writeData({ 0xA5, 0x35, 0x02, 0xDC });
    }

    void startDataCollection() {
        while (true) {
            readData(1544);
            processFrame();
            displayImage();
            checkKeyPress();
        }
    }

    void stopDataCollection() {
        writeData({ 0xA5, 0x35, 0x01, 0xDB });
        serial.close();
        cv::destroyAllWindows();
        std::cout << "Stopped" << std::endl;
    }

private:
    void writeData(const std::vector<uint8_t>& data) {
        boost::asio::write(serial, boost::asio::buffer(data));
    }

    void readData(std::size_t dataSize) {
        dataBuffer.resize(dataSize);
        boost::asio::read(serial, boost::asio::buffer(dataBuffer));
    }

    void processFrame() {
        double Tmin = 20.0;
        double Tmax = 40.0;

        double Ta = (byte2int(dataBuffer[1540]) + byte2int(dataBuffer[1541]) * 256) / 100.0;
        std::vector<int16_t> tempArray;
        for (std::size_t i = 4; i < 1540; i += 2) {
            int16_t value = byte2int(dataBuffer[i]) + byte2int(dataBuffer[i + 1]) * 256;
            tempArray.push_back(value);
        }

        cv::Mat taImg(24, 32, CV_8UC1);
        for (std::size_t i = 0; i < tempArray.size(); ++i) {
            uint8_t norm = static_cast<uint8_t>((tempArray[i] / 100.0 - Tmin) * 255.0 / (Tmax - Tmin));
            taImg.at<uint8_t>(i / 32, i % 32) = norm;
        }

        cv::Mat img;
        cv::applyColorMap(taImg, img, cv::COLORMAP_JET);
        cv::resize(img, img, cv::Size(320, 240), cv::INTER_CUBIC);
        cv::flip(img, img, 1);

        std::string text = "Tmin = " + std::to_string(*std::min_element(tempArray.begin(), tempArray.end()) / 100.0) + 
          " Tmax = " + std::to_string(*std::max_element(tempArray.begin(), tempArray.end()) / 100.0) +
          " FPS = " + std::to_string(1.0 / getElapsedTime());
        cv::putText(img, text, cv::Point(5, 15), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 0), 1);
          cv::imshow("Output", img);
}

void displayImage() {
    cv::waitKey(1);
}

void checkKeyPress() {
    int key = cv::waitKey(1);
    if (key == 's') {
        std::string filename = "pic_" + getCurrentTimestamp() + ".jpg";
        cv::imwrite(filename, img);
        std::cout << "Saving image " << filename << std::endl;
    }
}

int byte2int(uint8_t b) {
    return static_cast<int>(b);
}

double getElapsedTime() {
    static std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    double elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0;
    t0 = t1;
    return elapsedTime;
}

std::string getCurrentTimestamp() {
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&now_c);
    char timestamp[20];
    std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H-%M-%S", now_tm);
    return std::string(timestamp);
}

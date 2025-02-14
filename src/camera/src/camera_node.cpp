#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

std::string device;
int width, height, frame_rate;
std::string bayer_format;

// Kamera ayarlarını değiştirme fonksiyonu
bool setControl(int fd, int id, int value) {
    struct v4l2_control control = {};
    control.id = id;
    control.value = value;
    return ioctl(fd, VIDIOC_S_CTRL, &control) != -1;
}

// Kamera ayarlarını yapılandır
void configureCamera(const YAML::Node& config) {
    device = config["camera"]["device"].as<std::string>();
    width = config["camera"]["width"].as<int>();
    height = config["camera"]["height"].as<int>();
    frame_rate = config["camera"]["frame_rate"].as<int>();
    bayer_format = config["camera"]["bayer_format"].as<std::string>("rggb");

    int fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        ROS_ERROR("Kamera cihazı açılamadı!");
        return;
    }

    YAML::Node controls = config["controls"];
    
    // Tüm V4L2 kontrol ayarları
    setControl(fd, V4L2_CID_BRIGHTNESS, controls["brightness"].as<int>());
    setControl(fd, V4L2_CID_RED_BALANCE, controls["red_balance"].as<int>());
    setControl(fd, V4L2_CID_BLUE_BALANCE, controls["blue_balance"].as<int>());
    setControl(fd, V4L2_CID_GAIN, controls["gain"].as<int>());
    setControl(fd, V4L2_CID_WHITE_BALANCE_TEMPERATURE, controls["white_balance_temperature"].as<int>());
    setControl(fd, V4L2_CID_AUTO_WHITE_BALANCE, controls["white_balance_auto"].as<int>());
    setControl(fd, V4L2_CID_EXPOSURE_AUTO, controls["auto_exposure"].as<int>());
    setControl(fd, V4L2_CID_EXPOSURE_ABSOLUTE, controls["exposure_absolute"].as<int>());
    
    // Özel TIS kamera ayarları
    setControl(fd, 0x0199e201, controls["exposure_time_us"].as<int>());
    setControl(fd, 0x0199e202, controls["auto_shutter"].as<int>());
    setControl(fd, 0x0199e203, controls["auto_exposure_reference"].as<int>());
    setControl(fd, 0x0199e205, controls["gain_auto"].as<int>());
    setControl(fd, 0x0199e208, controls["trigger_mode"].as<int>());
    setControl(fd, 0x0199e211, controls["strobe_enable"].as<int>());
    setControl(fd, 0x0199e250, controls["white_balance_temperature"].as<int>());
    setControl(fd, 0x0199e251, controls["flip_horizontal"].as<int>());
    setControl(fd, 0x0199e252, controls["flip_vertical"].as<int>());
    setControl(fd, 0x0199e253, controls["tone_mapping"].as<int>());
    setControl(fd, 0x0199e254, controls["exposure_auto_upper_limit_auto"].as<int>());
    setControl(fd, 0x0199e255, controls["exposure_auto_lower_limit_us"].as<int>());
    setControl(fd, 0x0199e256, controls["exposure_auto_upper_limit_us"].as<int>());
    setControl(fd, 0x0199e259, controls["gain_auto_lower_limit"].as<int>());
    setControl(fd, 0x0199e260, controls["gain_auto_upper_limit"].as<int>());
    
    close(fd);
}

// Kamera başlat ve görüntü al
int initCamera(int &fd, void** buffer, int &buffer_length) {
    fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        ROS_ERROR("Kamera açılamadı!");
        return -1;
    }

    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SRGGB8;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        ROS_ERROR("Kamera formatı ayarlanamadı!");
        close(fd);
        return -1;
    }

    struct v4l2_requestbuffers req = {};
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        ROS_ERROR("Bellek ayırma başarısız!");
        close(fd);
        return -1;
    }

    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;

    if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
        ROS_ERROR("Buffer sorgulama başarısız!");
        close(fd);
        return -1;
    }

    *buffer = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    buffer_length = buf.length;

    if (*buffer == MAP_FAILED) {
        ROS_ERROR("Buffer mmap başarısız!");
        close(fd);
        return -1;
    }

    if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
        ROS_ERROR("Buffer kuyruğa alınamadı!");
        close(fd);
        return -1;
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        ROS_ERROR("Kamera yayını başlatılamadı!");
        close(fd);
        return -1;
    }

    return 0;
}

// Bayer formatını RGB'ye çevir
void convertBayerToRGB(cv::Mat &bayer, cv::Mat &rgb) {
    if (bayer_format == "rggb") {
        cv::cvtColor(bayer, rgb, cv::COLOR_BayerBG2BGR);
    } else if (bayer_format == "grbg") {
        cv::cvtColor(bayer, rgb, cv::COLOR_BayerGB2BGR);
    } else if (bayer_format == "gbrg") {
        cv::cvtColor(bayer, rgb, cv::COLOR_BayerGR2BGR);
    } else if (bayer_format == "bggr") {
        cv::cvtColor(bayer, rgb, cv::COLOR_BayerRG2BGR);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tis_camera_node");
    ros::NodeHandle nh;
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/camera/image_raw", 1);

    YAML::Node config = YAML::LoadFile("/home/alper/tis/src/camera/config/camera_config.yaml");
    configureCamera(config);

    int fd;
    void* buffer;
    int buffer_length;
    
    if (initCamera(fd, &buffer, buffer_length) < 0) {
        return -1;
    }

    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    ros::Rate loop_rate(frame_rate);

    while (ros::ok()) {
        if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
            ROS_WARN("Görüntü alınamadı!");
            continue;
        }

        cv::Mat bayer(height, width, CV_8UC1, buffer);
        cv::Mat rgb;
        convertBayerToRGB(bayer, rgb);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb).toImageMsg();
        image_pub.publish(msg);

        ioctl(fd, VIDIOC_QBUF, &buf);
        ros::spinOnce();
        loop_rate.sleep();
    }

    close(fd);
    return 0;
}
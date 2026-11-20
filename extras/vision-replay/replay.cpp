#include "System.h"
#include "Tracking.h"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using Clock = std::chrono::steady_clock;
using OrbSystem = ORB_SLAM3::System;
using Measurement = ORB_SLAM3::IMU::Point;

struct ImagePair {
    long long nanoseconds;
    std::string left;
    std::string right;
};
struct InertialSample {
    long long nanoseconds;
    Measurement measurement;
};

std::vector<std::string> split(std::string line) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    std::vector<std::string> fields;
    std::istringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(field);
    return fields;
}

std::vector<ImagePair> readImages(const std::string& folder) {
    std::ifstream stream(folder + "/frames.csv");
    std::vector<ImagePair> images;
    std::string row;
    std::getline(stream, row);
    while (std::getline(stream, row)) {
        auto f = split(row);
        if (f.size() != 4) throw std::runtime_error("Bad image manifest row");
        const auto ns = std::stoll(f[1]);
        if (!images.empty() && ns <= images.back().nanoseconds) continue;
        images.push_back({ns, folder + "/" + f[2], folder + "/" + f[3]});
    }
    if (images.empty()) throw std::runtime_error("No image pairs in " + folder);
    return images;
}

std::vector<InertialSample> readImu(const std::string& folder) {
    std::ifstream stream(folder + "/imu.csv");
    std::vector<InertialSample> imu;
    std::string row;
    std::getline(stream, row);
    while (std::getline(stream, row)) {
        auto f = split(row);
        if (f.size() != 7) throw std::runtime_error("Bad IMU manifest row");
        const auto ns = std::stoll(f[0]);
        if (!imu.empty() && ns <= imu.back().nanoseconds) continue;
        imu.push_back({ns, Measurement(std::stof(f[4]), std::stof(f[5]), std::stof(f[6]),
                                     std::stof(f[1]), std::stof(f[2]), std::stof(f[3]), ns * 1e-9)});
    }
    if (imu.empty()) throw std::runtime_error("No IMU samples in " + folder);
    return imu;
}

void writePose(std::ostream& out, const Sophus::SE3f& pose) {
    const auto& p = pose.translation();
    const auto& q = pose.unit_quaternion();
    out << ',' << p.x() << ',' << p.y() << ',' << p.z()
        << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w();
}

int main(int argc, char** argv) {
    if (argc != 7) {
        std::cerr << "Usage: replay VOCAB SETTINGS DATASET CSV mapping|restart FRAME_STRIDE\n";
        return 2;
    }
    cv::setNumThreads(1);
    const auto images = readImages(argv[3]);
    const auto imu = readImu(argv[3]);
    const bool restart = std::string(argv[5]) == "restart";
    const unsigned int stride = std::stoi(argv[6]);
    if (stride == 0) throw std::runtime_error("Invalid image stride");
    OrbSystem system(argv[1], argv[2], OrbSystem::IMU_STEREO, false);
    auto* oldAnchor = restart ? system.BenchmarkFindAnchor() : nullptr;
    if (restart && !oldAnchor) throw std::runtime_error("No original keyframe in loaded atlas");
    std::ofstream output(argv[4]);
    output << std::setprecision(13)
           << "timestamp_ns,state,map_id,contains_old_anchor,imu_ready,latency_ms,processing_ms,input_late_ms,imu_samples,"
           << "x,y,z,qx,qy,qz,qw,anchor_id,anchor_time,ax,ay,az,aqx,aqy,aqz,aqw\n";
    std::size_t cursor = 0;
    const auto anchor = Clock::now();
    for (std::size_t index=0;index<images.size();index+=stride) {
        const auto& frame=images[index];
        const auto due=anchor+std::chrono::nanoseconds(frame.nanoseconds-images.front().nanoseconds);
        std::this_thread::sleep_until(due);
        const auto begin = Clock::now();
        auto left=cv::imread(frame.left,cv::IMREAD_GRAYSCALE);
        auto right=cv::imread(frame.right,cv::IMREAD_GRAYSCALE);
        if (left.empty() || right.empty()) throw std::runtime_error("Missing stereo image");
        std::vector<Measurement> packets;
        while (cursor<imu.size() && imu[cursor].nanoseconds<=frame.nanoseconds) {
            packets.push_back(imu[cursor++].measurement);
        }
        const auto trackingStart=Clock::now();
        const auto twc=system.TrackStereo(left,right,frame.nanoseconds*1e-9,packets).inverse();
        const auto receipt=Clock::now();
        int state=system.GetTrackingState();
        bool old=false,ready=false;
        unsigned long anchorId=0;
        double anchorTime;
        Sophus::SE3f anchorTwc;
        const auto mapId=system.BenchmarkMapInfo(oldAnchor,old,ready,anchorId,anchorTime,anchorTwc);
        using Milliseconds=std::chrono::duration<double,std::milli>;
        output << frame.nanoseconds << ',' << state << ',' << mapId << ',' << old << ',' << ready
               << ',' << Milliseconds(receipt-due).count() << ',' << Milliseconds(receipt-trackingStart).count()
               << ',' << Milliseconds(begin-due).count() << ',' << packets.size();
        writePose(output,twc);
        output << ',' << anchorId << ',' << anchorTime;
        writePose(output,anchorTwc);
        output << '\n';
        output.flush();
        if (index%200==0) std::cout << "FRAME " << index << '/' << images.size() << " STATE " << state << " MAP " << mapId << " OLD " << old << std::endl;
    }
    // Let mapping jobs finish before taking the final map snapshot.
    std::this_thread::sleep_for(std::chrono::seconds(2));
    auto* mapAnchor = restart ? nullptr : system.BenchmarkFindAnchor();
    system.Shutdown();
    if (!restart) {
        if (!mapAnchor) {
            std::cerr << "No keyframes initialized during replay" << std::endl;
            return 3;
        }
        std::ofstream ref(std::string(argv[4])+".anchor");
        bool old,ready;
        unsigned long id;
        double time;
        Sophus::SE3f pose;
        const auto map = system.BenchmarkMapInfo(mapAnchor,old,ready,id,time,pose);
        ref << std::setprecision(13) << map << ',' << id << ',' << time << ',' << ready;
        writePose(ref,pose);
        ref << '\n';
    }
    std::cout << "REPLAY_COMPLETE" << std::endl;
}

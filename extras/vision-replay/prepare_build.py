from pathlib import Path

src = Path('/orb_work/ORB_SLAM3')

def replace(path, old, new, marker=None):
    p = src / path
    s = p.read_text()
    if (marker or new) in s:
        return
    if s.count(old) != 1:
        raise RuntimeError(f'Unexpected upstream context in {path}')
    p.write_text(s.replace(old, new))

# Ubuntu 20.04 in our benchmark image supplies OpenCV 4.2. The application
# uses functions available in that version. Keep the upstream source unchanged.
replace('CMakeLists.txt', 'find_package(OpenCV 4.4)', 'find_package(OpenCV 4.2)')
# Rectified configurations intentionally have no independent Camera 2 object.
replace('src/Settings.cc', 'if(settings.sensor_ == System::STEREO || settings.sensor_ == System::IMU_STEREO){\n            output << "\\t-Camera 2 parameters (";',
        'if((settings.sensor_ == System::STEREO || settings.sensor_ == System::IMU_STEREO) &&\n           settings.cameraType_ != Settings::Rectified){\n            output << "\\t-Camera 2 parameters (";')
# The no-loop setting used by our comparison must still consume the queue.
# In the early-return version, Run() reads an unset mpCurrentKF.
replace('src/LoopClosing.cc', '''    // To deactivate placerecognition. No loopclosing nor merging will be performed
    if(!mbActiveLC)
        return false;

''', '    // Consume and index queued keyframes even with loop detection disabled.\n')
replace('src/LoopClosing.cc', 'if(mpLastMap->IsInertial() && !mpLastMap->GetIniertialBA2())',
        'if(!mbActiveLC || (mpLastMap->IsInertial() && !mpLastMap->GetIniertialBA2()))')
# EraseObservation can erase the current point from mspMapPoints. Iterate
# over a pointer snapshot during archive cleanup to preserve the iterator.
replace('src/Map.cc', '    int nMPWithoutObs = 0;\n    for(MapPoint* pMPi : mspMapPoints)',
        '    int nMPWithoutObs = 0;\n    const auto pointsBeforeCleanup = GetAllMapPoints();\n    for(MapPoint* pMPi : pointsBeforeCleanup)')
replace('include/System.h', '    int GetTrackingState();', '''    int GetTrackingState();
    // Read-only benchmark probes for identifying the loaded map after a merge.
    KeyFrame* BenchmarkFindAnchor();
    long BenchmarkMapInfo(KeyFrame* oldAnchor, bool& oldMap, bool& imuReady,
                          unsigned long& originId, double& originTime,
                          Sophus::SE3f& originTwc);''')
replace('src/System.cc', 'void System::Shutdown()\n{', '''KeyFrame* System::BenchmarkFindAnchor()
{
    KeyFrame* first = nullptr;
    for (Map* map : mpAtlas->GetAllMaps()) {
        for (KeyFrame* kf : map->GetAllKeyFrames()) {
            if (kf && !kf->isBad() && (!first || kf->mnId < first->mnId)) first = kf;
        }
    }
    return first;
}

long System::BenchmarkMapInfo(KeyFrame* oldAnchor, bool& oldMap, bool& imuReady,
                              unsigned long& originId, double& originTime,
                              Sophus::SE3f& originTwc)
{
    Map* map = mpAtlas->GetCurrentMap();
    std::unique_lock<std::mutex> lock(map->mMutexMapUpdate);
    oldMap = oldAnchor && oldAnchor->GetMap() == map;
    imuReady = map->isImuInitialized();
    KeyFrame* origin = oldMap ? oldAnchor : map->GetOriginKF();
    originTime = -1.;
    originId = 0;
    originTwc = Sophus::SE3f();
    if (origin && !origin->isBad()) {
        originId = origin->mnId;
        originTime = origin->mTimeStamp;
        originTwc = origin->GetPoseInverse();
    }
    return static_cast<long>(map->GetId());
}

void System::Shutdown()
{''', 'KeyFrame* System::BenchmarkFindAnchor()')
replace('src/System.cc', 'KeyFrame* origin = oldMap ? oldAnchor : map->GetOriginKF();',
        'KeyFrame* origin = oldMap ? oldAnchor : (map->KeyFramesInMap() ? map->GetOriginKF() : nullptr);')
replace('src/System.cc', '    // Wait until all thread have effectively stopped', '''    // Drain workers before serializing the map in headless replay.
    if (!mStrSaveAtlasToFile.empty()) {
        while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
            usleep(5000);
    }
    // Wait until all thread have effectively stopped''')

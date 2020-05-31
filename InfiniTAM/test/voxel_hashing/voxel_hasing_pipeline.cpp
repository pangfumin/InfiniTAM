#include <iostream>
#include "Engine/UIEngine.h"
#include "Engine/ImageSourceEngine.h"
#include "Utils/temporal_buffer.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace InfiniTAM::Engine;


void LoadImages(const string dataset_path, const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(dataset_path + "/" + sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(dataset_path + "/" + sD);

        }
    }
}

typedef  Eigen::Matrix<double,7,1> Vec7;
void LoadGroundTruth(const string &strGroundTruthFilename,
                     TemporalBuffer<Vec7>& groundTruthBuffer) {
    ifstream fAssociation;
    fAssociation.open(strGroundTruthFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            if (s[0] == '#') {
//                std::cout <<s << std::endl;
                continue;
            }
            stringstream ss;
            ss << s;
            double timestamp, tx, ty, tz, qx, qy, qz, qw;
            ss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

            Vec7 vec;
            vec << tx , ty , tz , qx , qy , qz , qw;


            int64_t  ts_ns = timestamp * 1e9;
            groundTruthBuffer.addValue(ts_ns, vec);

        }
    }
}

void FindCorrespondingPose(vector<double> &vQueryTimestamps, const TemporalBuffer<Vec7>& groundTruthBuffer,
                               std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& poses) {
    for (auto ts : vQueryTimestamps) {
        int64_t  ts_ns = ts * 1e9;
        Vec7 vec;
        //        std::cout <<ts_ns << std::endl;
        int64_t  timestamp_ns_of_value;
        groundTruthBuffer.getNearestValueToTime(ts_ns, &vec);
        //        std::cout << "vec: " << vec.transpose() << std::endl;
        Eigen::Matrix<double,4,4>  T = Eigen::Matrix4d::Identity();
        Eigen::Vector3d t = vec.head<3>();
        Eigen::Quaterniond q;
        q.coeffs() = vec.tail(4);
        T.topLeftCorner(3,3) = q.toRotationMatrix();
        T.topRightCorner(3,1) = t;
        poses.push_back(T);
    }

}

int main() {

    std::string datasetPath = "/home/pang/dataset/tum/rgbd_dataset_freiburg1_desk";
    std::string associationFile = datasetPath + "/associated.txt";
    std::string strGroundTruthFilename = datasetPath + "/groundtruth.txt";
    std::string  calib_file = datasetPath + "/calib.txt";


    std::cout << "associationFile: " << associationFile << std::endl;
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;

    LoadImages(datasetPath, associationFile, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
    std::cout << "vstrImageFilenamesRGB: " << vstrImageFilenamesRGB.size() << std::endl;


    TemporalBuffer<Vec7> groundTruthBuffer;
    LoadGroundTruth(strGroundTruthFilename, groundTruthBuffer);
    std::cout << "groundTruthBuffer: " << groundTruthBuffer.size() << std::endl;

    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> camera_poses;
    FindCorrespondingPose(vTimestamps, groundTruthBuffer, camera_poses);
    std::cout << "camera_pose: " << camera_poses.size() << std::endl;

    Vector2i imageSize(640, 480);

    printf("initialising ...\n");
    ImageSourceEngine *imageSource = new ImageFileReader1(calib_file.c_str(), imageSize, imageSize,
                                                          &vstrImageFilenamesRGB, &vstrImageFilenamesD, &camera_poses);

    int cnt = 0;

    while (imageSource->hasMoreImages()) {
       ITMUChar4Image rgb(imageSize, true, true);
//       ITMShortImage *rawDepth, Eigen::Matrix4d* pose
        std::cout << cnt ++ << std::endl;
       imageSource->getImages(&rgb, NULL, NULL);
//        rgb.UpdateHostFromDevice();

       cv::Mat cv_color(imageSize[1], imageSize[0], CV_8UC4, rgb.GetData(MEMORYDEVICE_CPU));
       cv::imshow("image", cv_color);
       cv::waitKey();
    }


    return 0;
}
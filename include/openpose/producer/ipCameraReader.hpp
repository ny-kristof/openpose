#ifndef OPENPOSE_PRODUCER_IP_CAMERA_READER_HPP
#define OPENPOSE_PRODUCER_IP_CAMERA_READER_HPP

#include <atomic>
#include <mutex>
#include <openpose/core/common.hpp>
#include <openpose/producer/videoCaptureReader.hpp>

namespace op
{
    /**
     * IpCameraReader is a wrapper of the cv::VideoCapture class for IP camera streaming.
     */
    class OP_API IpCameraReader : public VideoCaptureReader
    {
    public:
        /**
         * Constructor of IpCameraReader. It opens the IP camera as a wrapper of cv::VideoCapture.
         * @param cameraPath const std::string parameter with the full camera IP link.
         */
        explicit IpCameraReader(const std::string& cameraPath, const Point<int>& ipcamResolution = Point<int>{}, const std::string& cameraParameterPath = "",
                                const bool undistortImage = false);

        virtual ~IpCameraReader();

        std::string getNextFrameName();

        inline bool isOpened() const
        {
            return VideoCaptureReader::isOpened();
        }

        double get(const int capProperty);

        void set(const int capProperty, const double value);

    private:
        const std::string mPathName;
        const bool mWebcamStarted;
        long long mFrameNameCounter;
        bool mThreadOpened;
        std::mutex mBufferMutex;
        Matrix mBuffer;
        std::atomic<bool> mCloseThread;
        std::thread mThread;
        // Detect camera unplugged
        double mLastNorm;
        std::atomic<int> mDisconnectedCounter;
        Point<int> mResolution;

        Matrix getRawFrame();

        std::vector<Matrix> getRawFrames();

        void bufferingThread();

        bool reset();

        DELETE_COPY(IpCameraReader);
    };
}

#endif // OPENPOSE_PRODUCER_IP_CAMERA_READER_HPP

// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ITMLib/ITMLib.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
namespace InfiniTAM
{
	namespace Engine
	{
		class ImageSourceEngine
		{
		public:
			ITMRGBDCalib calib;

			ImageSourceEngine(const char *calibFilename);
			virtual ~ImageSourceEngine() {}

			virtual bool hasMoreImages(void) = 0;
			virtual void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth) = 0;
			virtual void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth, Eigen::Matrix4d* pose ) = 0;
			virtual Vector2i getDepthImageSize(void) = 0;
			virtual Vector2i getRGBImageSize(void) = 0;
		};

		class ImageFileReader : public ImageSourceEngine
		{
		private:
			static const int BUF_SIZE = 2048;
			char rgbImageMask[BUF_SIZE];
			char depthImageMask[BUF_SIZE];

			ITMUChar4Image *cached_rgb;
			ITMShortImage *cached_depth;

			void loadIntoCache();
			int cachedFrameNo;
			int currentFrameNo;
		public:

			ImageFileReader(const char *calibFilename, const char *rgbImageMask, const char *depthImageMask);

			~ImageFileReader();

			bool hasMoreImages(void);
			void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);
            void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth, Eigen::Matrix4d* pose ) {

            }
			Vector2i getDepthImageSize(void);
			Vector2i getRGBImageSize(void);
		};

        class ImageFileReader1 : public ImageSourceEngine
        {
        private:
            const std::vector<std::string> * rgbImageLists_;
            const std::vector<std::string> * depthImageLists_;
            const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> *cameraPoseLists_;

            Vector2i depthImageSize_, rgbImageSize_;
            int currentFrameNo;
        public:

            ImageFileReader1(const char *calibFilename,
                            const Vector2i rgbImageSize,
                            const Vector2i depthImageSize,
                            const std::vector<std::string> *rgbImageLists,
                            const std::vector<std::string> *depthImageLists,
                            const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> *cameraPoseLists);
            ~ImageFileReader1();
            bool hasMoreImages(void);
            void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);
            void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth, Eigen::Matrix4d* pose );
            Vector2i getDepthImageSize(void);
            Vector2i getRGBImageSize(void);
        };

		class CalibSource : public ImageSourceEngine
		{
		private:
			Vector2i imgSize;
			void ResizeIntrinsics(ITMIntrinsics &intrinsics, float ratio);

		public:
			CalibSource(const char *calibFilename, Vector2i setImageSize, float ratio);
			~CalibSource() { }

			bool hasMoreImages(void) { return true; }
			void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth) { }
            void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth, Eigen::Matrix4d* pose ){

			}
			Vector2i getDepthImageSize(void) { return imgSize; }
			Vector2i getRGBImageSize(void) { return imgSize; }
		};

		class RawFileReader : public ImageSourceEngine
		{
		private:
			static const int BUF_SIZE = 2048;
			char rgbImageMask[BUF_SIZE];
			char depthImageMask[BUF_SIZE];

			ITMUChar4Image *cached_rgb;
			ITMShortImage *cached_depth;

			void loadIntoCache();
			int cachedFrameNo;
			int currentFrameNo;

			Vector2i imgSize;
			void ResizeIntrinsics(ITMIntrinsics &intrinsics, float ratio);

		public:
			RawFileReader(const char *calibFilename, const char *rgbImageMask, const char *depthImageMask, Vector2i setImageSize, float ratio);
			~RawFileReader() { }

			bool hasMoreImages(void);
			void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);
            void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth, Eigen::Matrix4d* pose ){}

			Vector2i getDepthImageSize(void) { return imgSize; }
			Vector2i getRGBImageSize(void) { return imgSize; }
		};
	}
}


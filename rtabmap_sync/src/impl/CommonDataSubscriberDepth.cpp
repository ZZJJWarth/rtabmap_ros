/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rtabmap_sync/CommonDataSubscriber.h>

#include <rtabmap/utilite/UConversion.h>

#include <cmath>
#include <cstring>
#include <opencv2/core.hpp>
#include <std_msgs/msg/string.hpp>
#include <zc_shm.hpp>

namespace rtabmap_sync {

namespace {

std::string normalizeZcTopic(std::string topic)
{
	if(!topic.empty() && topic[0] == '/')
	{
		topic.erase(0, 1);
	}
	return topic;
}

void eraseFromProcessingQueue(size_t subscriberId, size_t messageId)
{
	if(subscriberId == 0 || shm == 0)
	{
		return;
	}

	std::string procName = "ShmBlockingProcessing_" + std::to_string(subscriberId);
	ShmBlockingProcessing * proc = shm->find<ShmBlockingProcessing>(procName.c_str()).first;
	if(proc == 0)
	{
		return;
	}

	scoped_lock<interprocess_mutex> procLock(proc->mutex);
	auto iter = proc->myMessage.find(messageId);
	if(iter == proc->myMessage.end())
	{
		return;
	}
	if(iter->second <= 1)
	{
		proc->myMessage.erase(iter);
	}
	else
	{
		--iter->second;
	}
}

void copyPointCloud2FromZc(const ShmPointCloud2 & src, sensor_msgs::msg::PointCloud2 & dst)
{
	dst.header.stamp.sec = src.header.stamp.sec;
	dst.header.stamp.nanosec = src.header.stamp.nanosec;
	dst.header.frame_id = src.header.frame_id.c_str();
	dst.height = src.height;
	dst.width = src.width;
	dst.fields.clear();
	dst.fields.reserve(src.fields.size());
	for(const auto & field : src.fields)
	{
		sensor_msgs::msg::PointField outField;
		outField.name = field.name.c_str();
		outField.offset = field.offset;
		outField.datatype = field.datatype;
		outField.count = field.count;
		dst.fields.push_back(std::move(outField));
	}
	dst.is_bigendian = src.is_bigendian != 0;
	dst.point_step = src.point_step;
	dst.row_step = src.row_step;
	dst.data.resize(src.data.size());
	if(!src.data.empty())
	{
		std::memcpy(dst.data.data(), src.data.data(), src.data.size());
	}
	dst.is_dense = src.is_dense != 0;
}

}  // namespace

void CommonDataSubscriber::rgbZcCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
	ShmImage * image = shm?shm->find<ShmImage>(msg->data.c_str()).first:0;
	if(image == 0)
	{
		return;
	}

	{
		std::lock_guard<std::mutex> lock(latestRgbZcMutex_);
		if(latestRgbZcImage_ && latestRgbZcImage_ != image)
		{
			releaseRgbZcImage(latestRgbZcImage_);
		}
		latestRgbZcImage_ = image;
	}
	if(subscribedToDepthZc_ || subscribedToScan3dZc_)
	{
		tryDispatchRgbdZcFrame();
	}
}

void CommonDataSubscriber::depthZcStringCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
	ShmImage * image = shm?shm->find<ShmImage>(msg->data.c_str()).first:0;
	if(image == 0)
	{
		return;
	}

	{
		std::lock_guard<std::mutex> lock(latestRgbZcMutex_);
		if(latestDepthZcImage_ && latestDepthZcImage_ != image)
		{
			releaseZcImage(latestDepthZcImage_, depthZcSubscriberId_);
		}
		latestDepthZcImage_ = image;
	}
	tryDispatchRgbdZcFrame();
}

void CommonDataSubscriber::scan3dZcStringCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
	ShmPointCloud2 * cloud = shm?shm->find<ShmPointCloud2>(msg->data.c_str()).first:0;
	if(cloud == 0)
	{
		return;
	}

	{
		std::lock_guard<std::mutex> lock(latestRgbZcMutex_);
		if(latestScan3dZcCloud_ && latestScan3dZcCloud_ != cloud)
		{
			releaseZcPointCloud2(latestScan3dZcCloud_, scan3dZcSubscriberId_);
		}
		latestScan3dZcCloud_ = cloud;
	}
	tryDispatchRgbdZcFrame();
}

void CommonDataSubscriber::cameraInfoZcCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
	{
		std::lock_guard<std::mutex> lock(latestRgbZcMutex_);
		latestZcCameraInfo_ = msg;
	}
	tryDispatchRgbdZcFrame();
}

void CommonDataSubscriber::odomZcCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
	{
		std::lock_guard<std::mutex> lock(latestRgbZcMutex_);
		latestZcOdom_ = msg;
	}
	tryDispatchRgbdZcFrame();
}

void CommonDataSubscriber::releaseRgbZcImage(ShmImage * image)
{
	releaseZcImage(image, rgbZcSubscriberId_);
}

void CommonDataSubscriber::releaseZcImage(ShmImage * image, size_t subscriberId)
{
	if(image == 0 || manager_ == 0 || shm == 0 || subscriberId == 0)
	{
		return;
	}

	eraseFromProcessingQueue(subscriberId, image->myId);
	manager_->releaseMessage(image, shm);
}

void CommonDataSubscriber::releaseZcPointCloud2(ShmPointCloud2 * cloud, size_t subscriberId)
{
	if(cloud == 0 || manager_ == 0 || shm == 0 || subscriberId == 0)
	{
		return;
	}

	eraseFromProcessingQueue(subscriberId, cloud->myId);
	manager_->releaseMessage(cloud, shm);
}

cv_bridge::CvImageConstPtr CommonDataSubscriber::takeRgbZcImage(
		const rclcpp::Time & stamp,
		std::function<void()> & release)
{
	ShmImage * image = 0;
	{
		std::lock_guard<std::mutex> lock(latestRgbZcMutex_);
		if(latestRgbZcImage_ == 0)
		{
			return cv_bridge::CvImageConstPtr();
		}

		rclcpp::Time rgbStamp(latestRgbZcImage_->header.stamp.sec, latestRgbZcImage_->header.stamp.nanosec, stamp.get_clock_type());
		const double dt = std::fabs((rgbStamp - stamp).seconds());
		if(dt > rgbZcStampTolerance_)
		{
			return cv_bridge::CvImageConstPtr();
		}

		image = latestRgbZcImage_;
		latestRgbZcImage_ = 0;
	}

	std_msgs::msg::Header header;
	header.stamp.sec = image->header.stamp.sec;
	header.stamp.nanosec = image->header.stamp.nanosec;
	header.frame_id = image->header.frame_id.c_str();

	int type = cv_bridge::getCvType(image->encoding.c_str());
	cv::Mat mat(image->height, image->width, type, image->data.data(), image->step);
	release = [this, image]() {
		this->releaseRgbZcImage(image);
	};

	return cv_bridge::CvImageConstPtr(
		new cv_bridge::CvImage(header, image->encoding.c_str(), mat),
		[release](const cv_bridge::CvImage * ptr) mutable {
			delete ptr;
			if(release)
			{
				release();
			}
		});
}

cv_bridge::CvImageConstPtr CommonDataSubscriber::makeCvImageFromZc(
		ShmImage * image,
		size_t subscriberId)
{
	if(image == 0)
	{
		return cv_bridge::CvImageConstPtr();
	}

	std_msgs::msg::Header header;
	header.stamp.sec = image->header.stamp.sec;
	header.stamp.nanosec = image->header.stamp.nanosec;
	header.frame_id = image->header.frame_id.c_str();

	int type = cv_bridge::getCvType(image->encoding.c_str());
	cv::Mat mat(image->height, image->width, type, image->data.data(), image->step);

	return cv_bridge::CvImageConstPtr(
		new cv_bridge::CvImage(header, image->encoding.c_str(), mat),
		[this, image, subscriberId](const cv_bridge::CvImage * ptr) {
			delete ptr;
			this->releaseZcImage(image, subscriberId);
		});
}

void CommonDataSubscriber::tryDispatchRgbdZcFrame()
{
	if(!subscribedToDepthZc_)
	{
		return;
	}

	ShmImage * rgb = 0;
	ShmImage * depth = 0;
	ShmPointCloud2 * scan3d = 0;
	sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo;
	nav_msgs::msg::Odometry::ConstSharedPtr odom;

	{
		std::lock_guard<std::mutex> lock(latestRgbZcMutex_);
		if(latestRgbZcImage_ == 0 || (subscribedToDepthZc_ && latestDepthZcImage_ == 0))
		{
			return;
		}
		if(subscribedToScan3d_ && subscribedToScan3dZc_ && latestScan3dZcCloud_ == 0)
		{
			return;
		}
		if(!latestZcCameraInfo_)
		{
			return;
		}
		if(subscribedToOdom_ && !latestZcOdom_)
		{
			return;
		}

		rclcpp::Time rgbStamp(
				latestRgbZcImage_->header.stamp.sec,
				latestRgbZcImage_->header.stamp.nanosec,
				RCL_ROS_TIME);
		rclcpp::Time depthStamp(
				latestDepthZcImage_->header.stamp.sec,
				latestDepthZcImage_->header.stamp.nanosec,
				RCL_ROS_TIME);
		const double imageDt = std::fabs((rgbStamp - depthStamp).seconds());
		if(imageDt > rgbZcStampTolerance_)
		{
			return;
		}
		if(subscribedToScan3d_ && subscribedToScan3dZc_)
		{
			rclcpp::Time scanStamp(
					latestScan3dZcCloud_->header.stamp.sec,
					latestScan3dZcCloud_->header.stamp.nanosec,
					RCL_ROS_TIME);
			const double scanDt = std::fabs((scanStamp - depthStamp).seconds());
			if(scanDt > rgbZcStampTolerance_)
			{
				return;
			}
		}

		rgb = latestRgbZcImage_;
		depth = latestDepthZcImage_;
		scan3d = latestScan3dZcCloud_;
		cameraInfo = latestZcCameraInfo_;
		odom = latestZcOdom_;
		latestRgbZcImage_ = 0;
		latestDepthZcImage_ = 0;
		if(subscribedToScan3d_ && subscribedToScan3dZc_)
		{
			latestScan3dZcCloud_ = 0;
		}
	}

	cv_bridge::CvImageConstPtr rgbImage = makeCvImageFromZc(rgb, rgbZcSubscriberId_);
	cv_bridge::CvImageConstPtr depthImage = makeCvImageFromZc(depth, depthZcSubscriberId_);
	if(!rgbImage.get() || !depthImage.get())
	{
		return;
	}

	sensor_msgs::msg::LaserScan scan2dMsg;
	sensor_msgs::msg::PointCloud2 scan3dMsg;
	if(scan3d)
	{
		copyPointCloud2FromZc(*scan3d, scan3dMsg);
		releaseZcPointCloud2(scan3d, scan3dZcSubscriberId_);
	}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg;
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg;
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(rgbImage->header.stamp);}
	commonSingleCameraCallback(odom, userDataMsg, rgbImage, depthImage, *cameraInfo, *cameraInfo, scan2dMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::depthZcCommonCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr & depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cameraInfoMsg,
		const sensor_msgs::msg::LaserScan & scanMsg,
		const sensor_msgs::msg::PointCloud2 & scan3dMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr & odomInfoMsg)
{
	std::function<void()> release;
	cv_bridge::CvImageConstPtr imageMsg = takeRgbZcImage(depthMsg->header.stamp, release);
	if(!imageMsg.get())
	{
		return;
	}
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg;
	commonSingleCameraCallback(odomMsg, userDataMsg, imageMsg, cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::depthZcCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg;
	sensor_msgs::msg::LaserScan scanMsg;
	sensor_msgs::msg::PointCloud2 scan3dMsg;
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg;
	depthZcCommonCallback(odomMsg, depthMsg, cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::depthZcScan2dCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg;
	sensor_msgs::msg::PointCloud2 scan3dMsg;
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg;
	depthZcCommonCallback(odomMsg, depthMsg, cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::depthZcScan3dCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg;
	sensor_msgs::msg::LaserScan scan2dMsg;
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg;
	depthZcCommonCallback(odomMsg, depthMsg, cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}

void CommonDataSubscriber::depthZcInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg;
	sensor_msgs::msg::LaserScan scan2dMsg;
	sensor_msgs::msg::PointCloud2 scan3dMsg;
	depthZcCommonCallback(odomMsg, depthMsg, cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::depthZcScan2dInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg;
	sensor_msgs::msg::PointCloud2 scan3dMsg;
	depthZcCommonCallback(odomMsg, depthMsg, cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::depthZcScan3dInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg;
	sensor_msgs::msg::LaserScan scan2dMsg;
	depthZcCommonCallback(odomMsg, depthMsg, cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}

void CommonDataSubscriber::depthZcOdomCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	sensor_msgs::msg::LaserScan scan2dMsg;
	sensor_msgs::msg::PointCloud2 scan3dMsg;
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg;
	depthZcCommonCallback(odomMsg, depthMsg, cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::depthZcOdomScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	sensor_msgs::msg::PointCloud2 scan3dMsg;
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg;
	depthZcCommonCallback(odomMsg, depthMsg, cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::depthZcOdomScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	sensor_msgs::msg::LaserScan scan2dMsg;
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg;
	depthZcCommonCallback(odomMsg, depthMsg, cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}

void CommonDataSubscriber::depthZcOdomInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	sensor_msgs::msg::LaserScan scan2dMsg;
	sensor_msgs::msg::PointCloud2 scan3dMsg;
	depthZcCommonCallback(odomMsg, depthMsg, cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::depthZcOdomScan2dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	sensor_msgs::msg::PointCloud2 scan3dMsg;
	depthZcCommonCallback(odomMsg, depthMsg, cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}

void CommonDataSubscriber::depthZcOdomScan3dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	sensor_msgs::msg::LaserScan scan2dMsg;
	depthZcCommonCallback(odomMsg, depthMsg, cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}

// RGB + Depth
void CommonDataSubscriber::depthCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScan2dCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScan3dCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScanDescCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
void CommonDataSubscriber::depthInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScan2dInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScan3dInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScanDescInfoCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}

// RGB + Depth + Odom
void CommonDataSubscriber::depthOdomCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScanDescCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
void CommonDataSubscriber::depthOdomInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScan2dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScan3dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScanDescInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg; // Null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}

#ifdef RTABMAP_SYNC_USER_DATA
// RGB + Depth + User Data
void CommonDataSubscriber::depthDataCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // Null
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScan2dCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScan3dCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScanDescCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
void CommonDataSubscriber::depthDataInfoCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScan2dInfoCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScan3dInfoCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScanDescInfoCallback(
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	nav_msgs::msg::Odometry::ConstSharedPtr odomMsg; // null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}

// RGB + Depth + Odom + User Data
void CommonDataSubscriber::depthOdomDataCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	sensor_msgs::msg::LaserScan scanMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScan2dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScan3dCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScanDescCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg; // null
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
void CommonDataSubscriber::depthOdomDataInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScan2dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	sensor_msgs::msg::PointCloud2 scan3dMsg; // null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScan3dInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	sensor_msgs::msg::LaserScan scan2dMsg; // Null
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScanDescInfoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
		const rtabmap_msgs::msg::UserData::ConstSharedPtr userDataMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
		const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg,
		const rtabmap_msgs::msg::ScanDescriptor::ConstSharedPtr scanMsg,
		const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	if(syncDiagnostic_.get()) {syncDiagnostic_->tickInput(imageMsg->header.stamp);}
	std::vector<rtabmap_msgs::msg::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleCameraCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
#endif

void CommonDataSubscriber::setupDepthZcCallbacks(
		rclcpp::Node& node,
		const rclcpp::SubscriptionOptions & options,
		bool subscribeOdom,
		bool subscribeScan2d,
		bool subscribeScan3d,
		bool subscribeOdomInfo)
{
	RCLCPP_INFO(node.get_logger(), "Setup depth callback with Robonix RGB zero-copy input");

	if(!rgbZcShmInitialized_)
	{
		shm_init(rgbZcShmName_.c_str(), static_cast<size_t>(rgbZcShmSize_));
		rgbZcShmInitialized_ = true;
	}

	std::string shmTopic = normalizeZcTopic(rgbZcTopic_);
	rgbZcSubscriberId_ = manager_->add_subscriber(shmTopic.c_str(), shm);
	rgbZcSub_ = node.create_subscription<std_msgs::msg::String>(
			rgbZcTopic_,
			rclcpp::QoS(topicQueueSize_).reliability(qosImage_),
			std::bind(&CommonDataSubscriber::rgbZcCallback, this, std::placeholders::_1),
			options);

	if(subscribedToDepthZc_ || subscribedToScan3dZc_)
	{
		if(!subscribedToDepthZc_)
		{
			RCLCPP_WARN(node.get_logger(), "Robonix scan_cloud ZC requires depth ZC in this RTAB-Map path. Falling back to legacy RGB ZC path.");
		}
		else
		{
			std::string depthShmTopic = normalizeZcTopic(depthZcTopic_);
			depthZcSubscriberId_ = manager_->add_subscriber(depthShmTopic.c_str(), shm);
			depthZcStringSub_ = node.create_subscription<std_msgs::msg::String>(
					depthZcTopic_,
					rclcpp::QoS(topicQueueSize_).reliability(qosImage_),
					std::bind(&CommonDataSubscriber::depthZcStringCallback, this, std::placeholders::_1),
					options);
			cameraInfoZcSub_ = node.create_subscription<sensor_msgs::msg::CameraInfo>(
					"rgb/camera_info",
					rclcpp::QoS(topicQueueSize_).reliability(qosCameraInfo_),
					std::bind(&CommonDataSubscriber::cameraInfoZcCallback, this, std::placeholders::_1),
					options);
			if(subscribeOdom)
			{
				odomZcSub_ = node.create_subscription<nav_msgs::msg::Odometry>(
						"odom",
						rclcpp::QoS(topicQueueSize_).reliability(qosOdom_),
						std::bind(&CommonDataSubscriber::odomZcCallback, this, std::placeholders::_1),
						options);
			}
			if(subscribeScan3d && subscribedToScan3dZc_)
			{
				std::string scanShmTopic = normalizeZcTopic(scan3dZcTopic_);
				scan3dZcSubscriberId_ = manager_->add_subscriber(scanShmTopic.c_str(), shm);
				scan3dZcStringSub_ = node.create_subscription<std_msgs::msg::String>(
						scan3dZcTopic_,
						rclcpp::QoS(topicQueueSize_).reliability(qosScan_),
						std::bind(&CommonDataSubscriber::scan3dZcStringCallback, this, std::placeholders::_1),
						options);
			}
			RCLCPP_INFO(
					node.get_logger(),
					"ROBONIX_RTABMAP_RGBD_SCAN_ZC active: rgb='%s' depth='%s' scan3d='%s' shm='%s'",
					rgbZcTopic_.c_str(),
					depthZcTopic_.c_str(),
					(subscribeScan3d && subscribedToScan3dZc_)?scan3dZcTopic_.c_str():"<disabled>",
					rgbZcShmName_.c_str());
			return;
		}
	}

	std::string depthTopic = node.get_node_topics_interface()->resolve_topic_name("depth/image");
	zcDepthSub_.subscribe(&node, depthTopic, RCLCPP_QOS(topicQueueSize_, qosImage_), options);
	cameraInfoSub_.subscribe(&node, "rgb/camera_info", RCLCPP_QOS(topicQueueSize_, qosCameraInfo_), options);

	if(subscribeOdom)
	{
		odomSub_.subscribe(&node, "odom", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
		if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(&node, "scan", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
				SYNC_DECL5(CommonDataSubscriber, depthZcOdomScan2dInfo, approxSync_, syncQueueSize_, odomSub_, zcDepthSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(CommonDataSubscriber, depthZcOdomScan2d, approxSync_, syncQueueSize_, odomSub_, zcDepthSub_, cameraInfoSub_, scanSub_);
			}
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(&node, "scan_cloud", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
				SYNC_DECL5(CommonDataSubscriber, depthZcOdomScan3dInfo, approxSync_, syncQueueSize_, odomSub_, zcDepthSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(CommonDataSubscriber, depthZcOdomScan3d, approxSync_, syncQueueSize_, odomSub_, zcDepthSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
			SYNC_DECL4(CommonDataSubscriber, depthZcOdomInfo, approxSync_, syncQueueSize_, odomSub_, zcDepthSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL3(CommonDataSubscriber, depthZcOdom, approxSync_, syncQueueSize_, odomSub_, zcDepthSub_, cameraInfoSub_);
		}
	}
	else if(subscribeScan2d)
	{
		subscribedToScan2d_ = true;
		scanSub_.subscribe(&node, "scan", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
		if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
			SYNC_DECL4(CommonDataSubscriber, depthZcScan2dInfo, approxSync_, syncQueueSize_, zcDepthSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL3(CommonDataSubscriber, depthZcScan2d, approxSync_, syncQueueSize_, zcDepthSub_, cameraInfoSub_, scanSub_);
		}
	}
	else if(subscribeScan3d)
	{
		subscribedToScan3d_ = true;
		scan3dSub_.subscribe(&node, "scan_cloud", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
		if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
			SYNC_DECL4(CommonDataSubscriber, depthZcScan3dInfo, approxSync_, syncQueueSize_, zcDepthSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL3(CommonDataSubscriber, depthZcScan3d, approxSync_, syncQueueSize_, zcDepthSub_, cameraInfoSub_, scan3dSub_);
		}
	}
	else if(subscribeOdomInfo)
	{
		subscribedToOdomInfo_ = true;
		odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
		SYNC_DECL3(CommonDataSubscriber, depthZcInfo, approxSync_, syncQueueSize_, zcDepthSub_, cameraInfoSub_, odomInfoSub_);
	}
	else
	{
		SYNC_DECL2(CommonDataSubscriber, depthZc, approxSync_, syncQueueSize_, zcDepthSub_, cameraInfoSub_);
	}

	subscribedTopicsMsg_ += uFormat("\n   %s (Robonix RGB ZC, shm=%s)",
			rgbZcTopic_.c_str(),
			rgbZcShmName_.c_str());
	RCLCPP_INFO(
			node.get_logger(),
			"ROBONIX_RTABMAP_RGB_ZC active: topic='%s' shm='%s' subscriber_id=%zu",
			rgbZcTopic_.c_str(),
			rgbZcShmName_.c_str(),
			rgbZcSubscriberId_);
}

void CommonDataSubscriber::setupDepthCallbacks(
		rclcpp::Node& node,
		const rclcpp::SubscriptionOptions & options,
		bool subscribeOdom,
#ifdef RTABMAP_SYNC_USER_DATA
		bool subscribeUserData,
#else
		bool,
#endif
		bool subscribeScan2d,
		bool subscribeScan3d,
		bool subscribeScanDesc,
		bool subscribeOdomInfo)
{
	RCLCPP_INFO(node.get_logger(), "Setup depth callback");

	std::string rgbTopic = node.get_node_topics_interface()->resolve_topic_name("rgb/image"); // Humble/Jazzy don't resolve base topic, fixed by https://github.com/ros-perception/image_common/commit/ea7589ae8c1f7ecb83d6aab7b4c890c2d630d27a
	std::string depthTopic = node.get_node_topics_interface()->resolve_topic_name("depth/image"); // Humble/Jazzy don't resolve base topic, fixed by https://github.com/ros-perception/image_common/commit/ea7589ae8c1f7ecb83d6aab7b4c890c2d630d27a
#ifdef PRE_ROS_LYRICAL
	image_transport::TransportHints rgbHints(&node); // using "image_transport" parameter
	image_transport::TransportHints depthHints(&node, "raw", "depth_transport");
	imageSub_.subscribe(&node, rgbTopic, rgbHints.getTransport(), rclcpp::QoS(topicQueueSize_).reliability(qosImage_).get_rmw_qos_profile(), options);
	imageDepthSub_.subscribe(&node, depthTopic, depthHints.getTransport(), rclcpp::QoS(topicQueueSize_).reliability(qosImage_).get_rmw_qos_profile(), options);
#else
	image_transport::TransportHints rgbHints(node); // using "image_transport" parameter
	image_transport::TransportHints depthHints(node, "raw", "depth_transport");
	imageSub_.subscribe(node, rgbTopic, rgbHints.getTransport(), rclcpp::QoS(topicQueueSize_).reliability(qosImage_), options);
	imageDepthSub_.subscribe(node, depthTopic, depthHints.getTransport(), rclcpp::QoS(topicQueueSize_).reliability(qosImage_), options);
#endif
	cameraInfoSub_.subscribe(&node, "rgb/camera_info", RCLCPP_QOS(topicQueueSize_, qosCameraInfo_), options);

#ifdef RTABMAP_SYNC_USER_DATA
	if(subscribeOdom && subscribeUserData)
	{
		odomSub_.subscribe(&node, "odom", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
		userDataSub_.subscribe(&node, "user_data", RCLCPP_QOS(topicQueueSize_, qosUserData_), options);

		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(&node, "scan_descriptor", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
				SYNC_DECL7(CommonDataSubscriber, depthOdomDataScanDescInfo, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL6(CommonDataSubscriber, depthOdomDataScanDesc, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_);
			}
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(&node, "scan", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
				SYNC_DECL7(CommonDataSubscriber, depthOdomDataScan2dInfo, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL6(CommonDataSubscriber, depthOdomDataScan2d, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
			}
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(&node, "scan_cloud", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
				SYNC_DECL7(CommonDataSubscriber, depthOdomDataScan3dInfo, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL6(CommonDataSubscriber, depthOdomDataScan3d, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
			SYNC_DECL6(CommonDataSubscriber, depthOdomDataInfo, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL5(CommonDataSubscriber, depthOdomData, approxSync_, syncQueueSize_, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_);
		}
	}
	else 
#endif
	if(subscribeOdom)
	{
		odomSub_.subscribe(&node, "odom", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);

		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(&node, "scan_descriptor", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
				SYNC_DECL6(CommonDataSubscriber, depthOdomScanDescInfo, approxSync_, syncQueueSize_, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(CommonDataSubscriber, depthOdomScanDesc, approxSync_, syncQueueSize_, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_);
			}
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(&node, "scan", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
				SYNC_DECL6(CommonDataSubscriber, depthOdomScan2dInfo, approxSync_, syncQueueSize_, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(CommonDataSubscriber, depthOdomScan2d, approxSync_, syncQueueSize_, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
			}
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(&node, "scan_cloud", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
				SYNC_DECL6(CommonDataSubscriber, depthOdomScan3dInfo, approxSync_, syncQueueSize_, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(CommonDataSubscriber, depthOdomScan3d, approxSync_, syncQueueSize_, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
			SYNC_DECL5(CommonDataSubscriber, depthOdomInfo, approxSync_, syncQueueSize_, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL4(CommonDataSubscriber, depthOdom, approxSync_, syncQueueSize_, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_);
		}
	}
#ifdef RTABMAP_SYNC_USER_DATA
	else if(subscribeUserData)
	{
		userDataSub_.subscribe(&node, "user_data", RCLCPP_QOS(topicQueueSize_, qosUserData_), options);

		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(&node, "scan_descriptor", RCLCPP_QOS(topicQueueSize_, qosScan_), options);

			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
				SYNC_DECL6(CommonDataSubscriber, depthDataScanDescInfo, approxSync_, syncQueueSize_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(CommonDataSubscriber, depthDataScanDesc, approxSync_, syncQueueSize_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_);
			}
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(&node, "scan", RCLCPP_QOS(topicQueueSize_, qosScan_), options);

			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
				SYNC_DECL6(CommonDataSubscriber, depthDataScan2dInfo, approxSync_, syncQueueSize_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(CommonDataSubscriber, depthDataScan2d, approxSync_, syncQueueSize_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
			}
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(&node, "scan_cloud", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
				SYNC_DECL6(CommonDataSubscriber, depthDataScan3dInfo, approxSync_, syncQueueSize_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(CommonDataSubscriber, depthDataScan3d, approxSync_, syncQueueSize_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
			SYNC_DECL5(CommonDataSubscriber, depthDataInfo, approxSync_, syncQueueSize_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL4(CommonDataSubscriber, depthData, approxSync_, syncQueueSize_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_);
		}
	}
#endif
	else
	{
		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(&node, "scan_descriptor", RCLCPP_QOS(topicQueueSize_,qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
				SYNC_DECL5(CommonDataSubscriber, depthScanDescInfo, approxSync_, syncQueueSize_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(CommonDataSubscriber, depthScanDesc, approxSync_, syncQueueSize_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_);
			}
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(&node, "scan", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
				SYNC_DECL5(CommonDataSubscriber, depthScan2dInfo, approxSync_, syncQueueSize_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(CommonDataSubscriber, depthScan2d, approxSync_, syncQueueSize_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
			}
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(&node, "scan_cloud", RCLCPP_QOS(topicQueueSize_, qosScan_), options);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
				SYNC_DECL5(CommonDataSubscriber, depthScan3dInfo, approxSync_, syncQueueSize_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(CommonDataSubscriber, depthScan3d, approxSync_, syncQueueSize_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(&node, "odom_info", RCLCPP_QOS(topicQueueSize_, qosOdom_), options);
			SYNC_DECL4(CommonDataSubscriber, depthInfo, approxSync_, syncQueueSize_, imageSub_, imageDepthSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL3(CommonDataSubscriber, depth, approxSync_, syncQueueSize_, imageSub_, imageDepthSub_, cameraInfoSub_);
		}
	}
}

} /* namespace rtabmap_sync */

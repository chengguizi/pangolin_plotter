#ifndef STEREO_PROCESSOR_H_
#define STEREO_PROCESSOR_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
// #include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>


#include <tbb/tbb.h>

/**
 * This is an abstract base class for stereo image processing nodes.
 * It handles synchronization of input topics (approximate or exact)
 * and checks for sync errors.
 * To use this class, subclass it and implement the imageCallback() method.
 */
class StereoProcessor
{

public:
	struct Parameters {
		int queue_size;

		std::string left_topic, right_topic;
		std::string left_info_topic, right_info_topic;
	};

	static constexpr size_t N = 2;

	struct SyncedData {
		ros::Time timestamp;
		std::array<sensor_msgs::Image::ConstPtr, N> image_ptrs;
		std::array<sensor_msgs::CameraInfo::ConstPtr, N> info_ptrs;
	};

	tbb::concurrent_bounded_queue<SyncedData> multi_camera_buffer;

private:

	Parameters param;

	// subscriber
	image_transport::SubscriberFilter left_sub_, right_sub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_, right_info_sub_;
	

	// for sync checking
	ros::WallTimer check_synced_timer_;
	int left_received_, right_received_, left_info_received_, right_info_received_, all_received_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactPolicy;
	typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
	ExactSync exact_sync_;

	// for sync checking
	static void increment(int* value){ ++(*value); }

	// this callback is only called when all messages are synced at one time
	void dataCb(	const sensor_msgs::ImageConstPtr& l_image_msg,
					const sensor_msgs::ImageConstPtr& r_image_msg,
					const sensor_msgs::CameraInfoConstPtr& l_info_msg,
					const sensor_msgs::CameraInfoConstPtr& r_info_msg){	
		//int _seq = l_image_msg->header.seq;
		all_received_++;

		// imageCallback(l_image_msg, r_image_msg, l_info_msg, r_info_msg);

		SyncedData data;

		data.timestamp = l_image_msg->header.stamp;
		data.image_ptrs[0] = l_image_msg;
		data.image_ptrs[1] = r_image_msg;
		data.info_ptrs[0] = l_info_msg;
		data.info_ptrs[1] = r_info_msg;

		multi_camera_buffer.push(data);
	}

	void checkInputsSynchronized()
	{
		if (!all_received_)
			return;
		int threshold = 3 * all_received_;
		if (left_received_ >= threshold || right_received_ >= threshold || 
				left_info_received_ >= threshold || right_info_received_ >= threshold) {
			ROS_WARN("[stereo_processor] Low number of synchronized left/right/left_info/right_info tuples received.\n"
								"Left images received:       %d (topic '%s')\n"
								"Right images received:      %d (topic '%s')\n"
								"Left camera info received:  %d (topic '%s')\n"
								"Right camera info received: %d (topic '%s')\n"
								"Synchronized tuples: %d\n"
								"Possible issues:\n"
								"\t* stereo_image_proc is not running.\n"
								"\t  Does `rosnode info %s` show any connections?\n"
								"\t* The cameras are not synchronized.\n"
								"\t  Try restarting the node with parameter _approximate_sync:=True\n"
								"\t* The network is too slow. One or more images are dropped from each tuple.\n"
								"\t  Try restarting the node, increasing parameter 'queue_size' (currently %d)",
								left_received_, left_sub_.getTopic().c_str(),
								right_received_, right_sub_.getTopic().c_str(),
								left_info_received_, left_info_sub_.getTopic().c_str(),
								right_info_received_, right_info_sub_.getTopic().c_str(),
								all_received_, ros::this_node::getName().c_str(), param.queue_size);
		}
	}


public:
	
	/**
	 * Constructor, subscribes to input topics using image transport and registers
	 * callbacks.
	 * \param transport The image transport to use
	 */

	StereoProcessor(int queue_size, const std::string& left_topic, const std::string& right_topic, 
					const std::string& left_info_topic, const std::string& right_info_topic ) : StereoProcessor( Parameters{queue_size,left_topic,right_topic, left_info_topic, right_info_topic})
	{
	}

	StereoProcessor(const Parameters& param) :  param(param), 
		left_received_(0), right_received_(0), left_info_received_(0), right_info_received_(0), all_received_(0), 
		exact_sync_(ExactPolicy(param.queue_size), left_sub_, right_sub_, left_info_sub_, right_info_sub_)
	{
		// Read local parameters
		ros::NodeHandle local_nh("~");

		// Resolve topic names
		ros::NodeHandle nh;

		// Subscribe to four input topics.
		ROS_INFO("viso2_ros: Subscribing to:\n\t* %s\n\t* %s\n\t* %s\n\t* %s", 
				param.left_topic.c_str(), param.right_topic.c_str(),
				param.left_info_topic.c_str(), param.right_info_topic.c_str());

		image_transport::ImageTransport it(nh);
		//image_transport::TransportHints hints(transport,ros::TransportHints().tcpNoDelay());
		left_sub_.subscribe(it, param.left_topic, 10); //, hints); // http://docs.ros.org/diamondback/api/image_transport/html/classimage__transport_1_1TransportHints.html
		right_sub_.subscribe(it, param.right_topic, 10); //, hints);
		left_info_sub_.subscribe(nh, param.left_info_topic, 10); //, ros::TransportHints().tcpNoDelay());
		right_info_sub_.subscribe(nh, param.right_info_topic, 10); //,  ros::TransportHints().tcpNoDelay());

		// Complain every 15s if the topics appear unsynchronized
		left_sub_.registerCallback(boost::bind(StereoProcessor::increment, &left_received_));
		right_sub_.registerCallback(boost::bind(StereoProcessor::increment, &right_received_));
		left_info_sub_.registerCallback(boost::bind(StereoProcessor::increment, &left_info_received_));
		right_info_sub_.registerCallback(boost::bind(StereoProcessor::increment, &right_info_received_));
		check_synced_timer_ = nh.createWallTimer(ros::WallDuration(15.0),
				boost::bind(&StereoProcessor::checkInputsSynchronized, this));

		// Synchronize input topics.
		exact_sync_.registerCallback(boost::bind(&StereoProcessor::dataCb, this, _1, _2, _3, _4));
	}

	/**
	 * Implement this method in sub-classes 
	 */
	// virtual void imageCallback(	const sensor_msgs::ImageConstPtr l_image_msg,
	// 							const sensor_msgs::ImageConstPtr r_image_msg,
	// 							const sensor_msgs::CameraInfoConstPtr l_info_msg,
	// 							const sensor_msgs::CameraInfoConstPtr r_info_msg) = 0;

};

#endif


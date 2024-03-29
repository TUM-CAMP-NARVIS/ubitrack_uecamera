/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

/**
 * @ingroup driver_components
 * @file
 * The player component for playback of recorded ue camera measurements
 * Based on standard ubitrack player component
 *
 * 
 */

// Ubitrack
#include <utUtil/OS.h>
#include <utUtil/CalibFile.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/Timestamp.h>
#include <utDataflow/Module.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/PullSupplier.h>

// std
#include <deque> // for image queue
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <map>
#include <regex>
#include <stdint.h>
#include <mutex>
#include <condition_variable>

#ifdef _WIN32
#include <filesystem>
#else
#include <experimental/filesystem>
#endif

#if __cplusplus < 201703L // with C++ version less than 17, filesystem was in experimental
namespace fs = std::experimental::filesystem;
#else
namespace fs = std::filesystem;
#endif

// Boost
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp>
#include <boost/shared_ptr.hpp>

#include <utVision/Image.h>
#include <opencv/highgui.h>
#include "nlohmann/json.hpp"

using json = nlohmann::json;

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Drivers.UEPlayer" ) );

namespace Ubitrack { namespace Drivers {

// forward decls
class UECameraPlayerComponentBase;


/**
 * Component key for UECameraPlayerProducer/Consumer components.
 */

MAKE_DATAFLOWCONFIGURATIONATTRIBUTEKEY( UECameraPlayerComponentKey, std::string, "file")

/**
 * Module used by UECameraPlayer components, maintains a single main loop for all UECameraPlayer components
 */
class UECameraPlayerModule
	: public Dataflow::Module< Dataflow::SingleModuleKey, UECameraPlayerComponentKey, UECameraPlayerModule, UECameraPlayerComponentBase >
{
public:
	/** simplifies our life afterwards */
	typedef Dataflow::Module< Dataflow::SingleModuleKey, UECameraPlayerComponentKey, UECameraPlayerModule, UECameraPlayerComponentBase > BaseClass;

	UECameraPlayerModule( const Dataflow::SingleModuleKey& key, boost::shared_ptr< Graph::UTQLSubgraph >, FactoryHelper* fh )
		: BaseClass( key, fh )
		, m_bStop( false )
	{
		LOG4CPP_INFO( logger, "created UECameraPlayerModule" );
	}

	~UECameraPlayerModule()
	{
		// stop main loop
		m_bStop = true;

		// wait for thread
		if ( m_pMainLoopThread )
			m_pMainLoopThread->join();

		LOG4CPP_INFO( logger, "destroyed UECameraPlayerModule" );
	}

	void startThread()
	{
		LOG4CPP_DEBUG( logger, "starting thread" );

		// start mainloop
		if ( !m_pMainLoopThread )
			m_pMainLoopThread.reset( new boost::thread( boost::bind( &UECameraPlayerModule::mainloop, this ) ) );
	}

protected:

	/** the main loop thread */
	boost::shared_ptr< boost::thread > m_pMainLoopThread;

	/** stop the main loop? */
	bool m_bStop;

	/** method that runs the main loop */
	void mainloop();

	/** create new components */
	boost::shared_ptr< UECameraPlayerComponentBase > createComponent( const std::string& type, const std::string& name,
		boost::shared_ptr< Graph::UTQLSubgraph > pConfig, const ComponentKey& key, UECameraPlayerModule* pModule );
};


/**
 * Base class for all UECameraPlayer components.
 */
class UECameraPlayerComponentBase
	: public UECameraPlayerModule::Component
{
public:
	UECameraPlayerComponentBase( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph >, const UECameraPlayerComponentKey& key,
		UECameraPlayerModule* module )
		: UECameraPlayerModule::Component( name, key, module )
	{}

	virtual ~UECameraPlayerComponentBase()
	{}

	/** returns the timestamp of the first event */
	virtual Measurement::Timestamp getFirstTime() const
	{ assert( false ); return 0; }

	/** return real time of the next measurement to be played or 0 if no events */
	virtual Measurement::Timestamp getNextTime( Measurement::Timestamp recordStart, Measurement::Timestamp playbackStart )
	{ assert( false ); return 0; }

	/** send the next event with the given offset */
	virtual void sendNext( Measurement::Timestamp recordStart, Measurement::Timestamp playbackStart )
	{ assert( false ); }
	
	virtual void sendFinished() {

		//assert( false );
		// @todo how to gracefully exit the UECameraPlayer ?
		LOG4CPP_WARN( logger, "UECameraPlayer reached end of recording..");
	}

	virtual void start()
	{
		// for some reason, the default startModule mechanism does not work here...
		UECameraPlayerModule::Component::start();
		getModule().startThread();
	}
};



class UECameraPlayerComponentImage
	: public UECameraPlayerComponentBase
{
public:
	/** loads the file */
	UECameraPlayerComponentImage( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > pConfig, const UECameraPlayerComponentKey& key,
		UECameraPlayerModule* module )
		: UECameraPlayerComponentBase( name, pConfig, key, module )
		, m_tsFile( "" )
		, m_offset( 0 )
		, m_speedup( 1.0 )
		, m_outPortColor( "OutputColor", *this )
		, m_outPortDepth( "OutputDepth", *this )
		, m_outPortPoseColor( "OutputPoseColor", *this, boost::bind(&UECameraPlayerComponentImage::getCameraPoseColor, this, _1) )
		, m_outPortPoseDepth( "OutputPoseDepth", *this, boost::bind(&UECameraPlayerComponentImage::getCameraPoseDepth, this, _1) )
		, m_initialTS( 1000LL )
		, m_outPortIntrinsicsColor( "OutputIntrinsicsColor", *this, boost::bind(&UECameraPlayerComponentImage::getCameraModelColor, this, _1) )
		, m_outPortIntrinsicsDepth( "OutputIntrinsicsDepth", *this, boost::bind(&UECameraPlayerComponentImage::getCameraModelDepth, this, _1) )
        , readingFilesDone(false)
        , m_maxSizeQueue(32)
	{
		LOG4CPP_INFO( logger, "Created UECameraPlayerComponentImage using file \"" << key.get() << "\"." );

		pConfig->m_DataflowAttributes.getAttributeData("offset", m_offset);
		pConfig->m_DataflowAttributes.getAttributeData("speedup", m_speedup);
		std::string metadataFile = pConfig->m_DataflowAttributes.getAttributeString("file");

	    std::ifstream infile(metadataFile);
		metadata = json::parse(infile);

		basePath = metadataFile.substr(0, metadataFile.length() - std::string("Metadata.json").length());
		// load all image paths
		std::regex rex{R"###(image_number_([0-9]*)\.depth16)###"};
		std::smatch matches;

		std::vector<std::string> allFiles;
		for (const auto & entry : fs::directory_iterator(basePath))
		{
			LOG4CPP_INFO( logger, "Found image " << entry.path().u8string() );
			
			std::string fileName = entry.path().u8string();
			if (std::regex_search(fileName, matches, rex))
			{
				std::string numbers = matches[1];
				allImagesBasenames[std::stoi(numbers)] = basePath + "image_number_" + numbers;
			}
		}		
		
		// start loading images in a thread
		if ( !m_pLoadThread )
			m_pLoadThread.reset( new boost::thread( boost::bind( &UECameraPlayerComponentImage::loadImages, this ) ) );
			
		// before starting the component let the tread do some work on loading 
		Util::sleep( 300 );
	}
	
    void loadImages()
    {
        std::string sWidth = metadata["width"];
        std::string sHeight = metadata["height"];
        std::string sFPS = metadata["fps"];
		int32_t width = std::stoi(sWidth);
		int32_t height = std::stoi(sHeight);
		float fps = std::stof(sFPS);
		int64_t nanoseconds = 1000LL * 1000LL * 1000LL;

		LOG4CPP_INFO( logger, "Total images " << allImagesBasenames.size() );

		for (auto entry = allImagesBasenames.begin(); entry != allImagesBasenames.end(); ++entry)
		{            
			std::string nameColor = entry->second + ".bgr8";
			std::string nameDepth = entry->second + ".depth16";

			cv::Mat imgColor(height, width, CV_8UC4);
			cv::Mat imgDepth(height, width, CV_16UC1);
			cv::Mat imgDepth8bit(height, width, CV_8UC1);
			
			int32_t fileSizeColor, fileSizeDepth;
			
            if (!
                (readFile(nameColor, fileSizeColor, reinterpret_cast<char*>(imgColor.data)) &&
                 readFile(nameDepth, fileSizeDepth, reinterpret_cast<char*>(imgDepth.data)))
                )
            {
                LOG4CPP_INFO(logger, "Skipped image " << entry->second << " due to file reading errors.");
                continue;
            }
/*
			uint16_t maxValue = 4096;
			uint16_t minValue = 0;

			for (int index = 0; index < width * height; ++index)
			{
				uint16_t value = imgDepth.at<uint16_t>(index / width, index % width);
				double result = static_cast<double>(value - minValue) * 255 / (maxValue - minValue);
				if (result > 255.0)
				    result = 255.0;

				uint8_t res8bit = static_cast<uint8_t>(result);
				imgDepth8bit.at<uint8_t>(index / width, index % width) = res8bit;
			}

			cv::imwrite(entry->second + ".pgm", imgDepth8bit);*/

			if (fileSizeColor != 2 * fileSizeDepth && fileSizeColor != width * height * 4)
			{
				// Skip this image, something is wrong
                LOG4CPP_INFO(logger, "Skipped image " << entry->second << " due to filesize errors.");
				continue;
			}
						
            {
                std::unique_lock <std::mutex> guardColor(queueAccessColor);
                if (imagesColor.size() == m_maxSizeQueue)
                {
                    // Wait for mutex conditional signal to wake up
                    queueAccessColorCV.wait(guardColor);
                }
                imagesColor.push(imgColor);
            }
			
            {
                std::unique_lock <std::mutex> guardDepth(queueAccessDepth);
                if (imagesDepth.size() == m_maxSizeQueue)
                {
                    // Wait for mutex conditional signal to wake up
                    queueAccessDepthCV.wait(guardDepth);
                }
                imagesDepth.push(imgDepth);
            }
            
			//LOG4CPP_INFO(logger, "Loaded next images, basename " << entry->second);
			//LOG4CPP_INFO(logger, "Timestamp: " << m_initialTS + entry->first * nanoseconds / fps);
            m_events.push_back(Measurement::Timestamp(m_initialTS + entry->first * nanoseconds / fps));
            eventsAccessCV.notify_one();
		}			

		// read file with timestamps and other data
		
		// generated for this example
        
        readingFilesDone = true;
        LOG4CPP_INFO( logger, "Done loading " << m_events.size() << " images defined in file \"" << m_tsFile << "\"." );
	}

	Measurement::Timestamp getFirstTime() const
	{
		if (!m_events.empty()) 
        {
			// TODO: provide the first/earliest timestamp 
			Measurement::Timestamp ts = *m_events.begin();
			return ts + 1000000LL * m_offset;
		}			
		else
			return 0;
	}

	/** return time of the next measurement to be played or 0 if no events */
	Measurement::Timestamp getNextTime( Measurement::Timestamp recordStart, Measurement::Timestamp playbackStart )
	{
        if (!(readingFilesDone && m_events.empty()))
        {
            if (m_events.empty())
            {
                std::unique_lock<std::mutex> guard(eventsAccess);
                eventsAccessCV.wait(guard);
            }
			// TODO: provide the next timestamp 
			Measurement::Timestamp ts = *m_events.begin();
			return recordTimeToReal(ts, recordStart, playbackStart);
		}
			
		else
			return 0;
	}

	/** send the next event */
	void sendNext( Measurement::Timestamp recordStart, Measurement::Timestamp playbackStart )
	{
		if( !m_events.empty() )
		{
			Measurement::Timestamp ts = recordTimeToReal(*m_events.begin(), recordStart, playbackStart);
			unsigned long long int timestamp = *m_events.begin();
			LOG4CPP_INFO(logger, "Getting timestamp: " << static_cast<double>(timestamp / 1000000) << " from camera " << basePath);
			Vision::Image::ImageFormatProperties fmt;

			fmt.imageFormat = Vision::Image::BGRA;
			fmt.channels = 4;
			fmt.bitsPerPixel = 32;

            {
                std::unique_lock <std::mutex> guardColor(queueAccessColor);
                boost::shared_ptr<Vision::Image> colorImage = boost::shared_ptr<Vision::Image>(new Vision::Image(imagesColor.front(), fmt));
                m_outPortColor.send(Measurement::ImageMeasurement(ts, colorImage));
            }

			fmt.imageFormat = Vision::Image::DEPTH;
			fmt.channels = 1;
			fmt.bitsPerPixel = 16;

            {
                std::unique_lock <std::mutex> guardColor(queueAccessDepth);
                boost::shared_ptr<Vision::Image> depthImage = boost::shared_ptr<Vision::Image>(new Vision::Image(imagesDepth.front(), fmt));
                m_outPortDepth.send(Measurement::ImageMeasurement(ts, depthImage));
            }

            imagesColor.pop();
            queueAccessColorCV.notify_one();
            
            imagesDepth.pop();
            queueAccessDepthCV.notify_one();

			m_events.pop_front();
		}
	}

    Measurement::Pose getCameraPoseColor(Measurement::Timestamp t)
    {
	    std::string sQx = metadata["color_image"]["rot_X"];
	    std::string sQy = metadata["color_image"]["rot_Y"];
    	std::string sQz = metadata["color_image"]["rot_Z"];
        std::string sQw = metadata["color_image"]["rot_W"];

	    double qx = std::stod(sQx);
	    double qy = std::stod(sQy);
        double qz = std::stod(sQz);
        double qw = std::stod(sQw);

        std::string sXpos = metadata["color_image"]["pos_x"];
        std::string sYpos = metadata["color_image"]["pos_y"];
        std::string sZpos = metadata["color_image"]["pos_z"];

        double xPos  = std::stod(sXpos);
		double yPos  = std::stod(sYpos);
		double zPos  = std::stod(sZpos);
		
		Math::Pose pose = ubitrackPoseFromUEPose(xPos, yPos, zPos, qx, qy, qz, qw);

		//LOG4CPP_INFO(logger, "Getting color pose: " << pose);
			
		return Measurement::Pose(t, pose);
	}

	Measurement::Pose getCameraPoseDepth(Measurement::Timestamp t)
	{
        std::string sQx = metadata["depth_image"]["rot_X"];
        std::string sQy = metadata["depth_image"]["rot_Y"];
        std::string sQz = metadata["depth_image"]["rot_Z"];
        std::string sQw = metadata["depth_image"]["rot_W"];

        double qx = std::stod(sQx);
        double qy = std::stod(sQy);
	    double qz = std::stod(sQz);
	    double qw = std::stod(sQw);

	    std::string sXpos = metadata["depth_image"]["pos_x"];
	    std::string sYpos = metadata["depth_image"]["pos_y"];
	    std::string sZpos = metadata["depth_image"]["pos_z"];

		double xPos  = std::stod(sXpos);
		double yPos  = std::stod(sYpos);
		double zPos  = std::stod(sZpos);
		
		Math::Pose pose = ubitrackPoseFromUEPose(xPos, yPos, zPos, qx, qy, qz, qw);

		//LOG4CPP_INFO(logger, "Getting depth pose: " << pose);

		return Measurement::Pose(t, pose);
	}

	Measurement::CameraIntrinsics getCameraModelColor(Measurement::Timestamp t)
    {
        std::string sWidth = metadata["width"];
        std::string sHeight = metadata["height"];
		std::size_t width = std::stoi(sWidth);
		std::size_t height = std::stoi(sHeight);
		int32_t iWidth = static_cast<int32_t>(width);
		int32_t iHeight = static_cast<int32_t>(height);
		const double PI = 3.1415926;
        std::string sFOV = metadata["fov"];
		
        double field_of_view = std::stod(sFOV);

		// TODO: provide intrinsic properties
		// done
		Math::Matrix< double, 3, 3 > intrinsicMatrix;
		double focal_length_X = (width / 2) / std::tan(PI * (field_of_view / 2.0) / 180.0);
		double focal_length_Y = (height / 2) / std::tan(PI * (field_of_view / 2.0) / 180.0);

		// row , col
		intrinsicMatrix(0, 0) = focal_length_X; //fx
		intrinsicMatrix(0, 1) = 0;
		intrinsicMatrix(0, 2) = -iWidth / 2; //cx
		intrinsicMatrix(1, 0) = 0;
		intrinsicMatrix(1, 1) = focal_length_Y; //fy
		intrinsicMatrix(1, 2) = -iHeight / 2; //cy
		intrinsicMatrix(2, 0) = 0;
		intrinsicMatrix(2, 1) = 0;
		intrinsicMatrix(2, 2) = -1;

		Math::Vector< double, 2 > radial;
		radial(0) = 0;
		radial(1) = 0;
		Math::Vector< double, 2 > tangential;
		tangential(0) = 0;
		tangential(1) = 0;

		Math::CameraIntrinsics<double> intrinsics(intrinsicMatrix, radial, tangential, width, height);
        return Measurement::CameraIntrinsics(t, intrinsics);
    }

	Measurement::CameraIntrinsics getCameraModelDepth(Measurement::Timestamp t)
	{
		return getCameraModelColor(t);
	}
	
protected:
		
	std::unique_ptr<char[]> readFile(std::string filename, int32_t& fileSize)
	{
		std::ifstream file(filename, std::ios::binary);
		file.unsetf(std::ios::skipws);
		file.seekg(0, std::ios::end);
		fileSize = static_cast<int32_t>(file.tellg());
		file.seekg(0, std::ios::beg);
		std::unique_ptr<char[]> buffer(new char[fileSize]);
		file.read(buffer.get(), fileSize);
		file.close();
		return buffer;
	}

    bool readFile(std::string filename, int32_t& fileSize, char* buffer)
    {
        std::ifstream file(filename, std::ios::binary);
        if (!file.good())
        {
            return false;
        }
        
        file.unsetf(std::ios::skipws);
        file.seekg(0, std::ios::end);
        fileSize = static_cast<int32_t>(file.tellg());
        file.seekg(0, std::ios::beg);
        file.read(buffer, fileSize);
        file.close();
        return true;
    }

	Math::Pose ubitrackPoseFromUEPose(double xPos, double yPos, double zPos, double qx, double qy, double qz, double qw)
	{
		xPos /= 100.0;
		yPos /= 100.0;
		zPos /= 100.0;
		
		Math::Quaternion rotation(qx,qy,qz,qw);
		Math::Vector3d position(xPos, yPos, zPos);                                                                 
	    Math::Pose ueCameraPose(rotation, position);


		// rotate 90° around Z Axis
		Math::Pose p1 = Math::Pose(Math::Quaternion(0., 0., -0.7071067811865476, 0.7071067811865476), Math::Vector3d(.0, .0, .0));

		// rotate 90° around x Axis, evtl. -90° Math::Pose(-0.7071067811865476, 0, 0, 0.7071067811865476);
		Math::Pose p2 = Math::Pose(Math::Quaternion(-0.7071067811865476, 0., 0., 0.7071067811865476), Math::Vector3d(.0, .0, .0));
	
		Math::Pose orientUECameraToUbitrackCameraOrientation = p2*p1;  // rotate camera, still lefthanded

		Math::Pose ueCameraInUbitrack = orientUECameraToUbitrackCameraOrientation*ueCameraPose;  
	
		// now ueCameraInUbitrack is oriented like ubitrack expects the camera but still left 	handed
		Math::Quaternion rot = ueCameraInUbitrack.rotation();  

		// convert left to right handed by mirroring along the x-y Plane
		// just flip z
		Math::Vector3d pos = ueCameraInUbitrack.translation();
		Math::Vector3d posRightHanded = Math::Vector3d(pos(0),pos(1), -pos(2));  // flip z and then invert

		// why invert: When we mirror on x-y plane the origin is now somewhere else hence we have to invert otherwise
		// flip z: z = -z
		// invert: x = -x, y = -y, z=-z, w=w
		// --> just invert x and y
		Math::Quaternion tmp2 = ~orientUECameraToUbitrackCameraOrientation.rotation();
		rot = rot * tmp2;
		Math::Quaternion rotRightHanded = Math::Quaternion(-rot.x(), -rot.y(), rot.z(), rot.w());

		Math::Pose finalPose = Math::Pose(rotRightHanded,posRightHanded);
        
		return finalPose;
	}

	/**
	 * converts a recorded time to a real time.
	 * @param t timestamp to convert
	 * @param recordStart time the recording was started
	 * @param realStart time the playback was started
	 */
	Measurement::Timestamp recordTimeToReal( Measurement::Timestamp t, Measurement::Timestamp recordStart, Measurement::Timestamp playbackStart )
	{ return static_cast< Measurement::Timestamp >( ( t - recordStart + m_offset * 1000000LL ) / m_speedup + playbackStart ); }

	/** file which defines timestamps and images */
	std::string m_tsFile;
	
	/** offset if the event should be sent at some other time than its timestamp */
	int m_offset;

	/** speedup factor */
	double m_speedup;
	
	/** a thread for loading images */
	boost::shared_ptr< boost::thread > m_pLoadThread;

	/** output port */
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPortColor;
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPortDepth;
	Dataflow::PullSupplier< Measurement::Pose > m_outPortPoseColor;
	Dataflow::PullSupplier< Measurement::Pose > m_outPortPoseDepth;
	Dataflow::PullSupplier< Measurement::CameraIntrinsics > m_outPortIntrinsicsColor;
	Dataflow::PullSupplier< Measurement::CameraIntrinsics > m_outPortIntrinsicsDepth;

	/** queue for the images being loaded, @todo add mutex for accessing m_events */
	// TODO: store all data
	uint64_t m_initialTS;
    
    const int32_t m_maxSizeQueue;
    std::atomic_bool readingFilesDone;
    std::mutex queueAccessColor;
    std::mutex queueAccessDepth;
    std::mutex eventsAccess;

    std::condition_variable queueAccessColorCV;
    std::condition_variable queueAccessDepthCV;
    std::condition_variable eventsAccessCV;

    std::queue<cv::Mat> imagesColor;
	std::queue<cv::Mat> imagesDepth;
	std::map<int, std::string> allImagesBasenames;

	std::string basePath;

	json metadata;
	std::deque< Measurement::Timestamp > m_events;
};



void UECameraPlayerModule::mainloop()
{
	// find time of first recorded event in queue
	Measurement::Timestamp recordStart( 0 );
	{
		ComponentList l( getAllComponents() );
		for ( ComponentList::iterator it = l.begin(); it != l.end(); it++ )
		{
			Measurement::Timestamp t = (*it)->getFirstTime();
			if ( t && ( recordStart == 0 || t < recordStart ) )
				recordStart = t;
		}
	}
	LOG4CPP_DEBUG( logger, "recordStart = " << recordStart );

	// delay start for 2s to allow other components to start
	Measurement::Timestamp playbackStart( Measurement::now() + 2000000000LL );
	LOG4CPP_DEBUG( logger, "playbackStart = " << playbackStart );

	// find playback time of first event in queue
	Measurement::Timestamp nextEventTime( 0 );
	{
		ComponentList l( getAllComponents() );
		for ( ComponentList::iterator it = l.begin(); it != l.end(); it++ )
		{
			Measurement::Timestamp t = (*it)->getNextTime( recordStart, playbackStart );
			if ( t && ( nextEventTime == 0 || t < nextEventTime ) )
				nextEventTime = t;
		}
	}
	LOG4CPP_INFO( logger, "Starting main loop" );
	LOG4CPP_DEBUG( logger, "Starting main loop at " << nextEventTime );

	// main loop
	while ( !m_bStop && nextEventTime )
	{
		LOG4CPP_DEBUG( logger, "nextEventTime = " << nextEventTime );

		// sleep until next event
		Measurement::Timestamp now( Measurement::now() );
		long long int sleepdur( nextEventTime - now );
		if ( sleepdur > 0 )
		{
			LOG4CPP_DEBUG( logger, "sleeping " << sleepdur / 1000000 << "ms" );
			Util::sleep( int( sleepdur / 1000000 ), int( sleepdur % 1000000 ) );
		}

		now = Measurement::now();
		nextEventTime = 0;

		// iterate all components
		ComponentList l( getAllComponents() );
		int count = 0;
		for ( ComponentList::iterator it = l.begin(); it != l.end(); it++ )
		{
			// send all events due until now
			Measurement::Timestamp t = (*it)->getNextTime( recordStart, playbackStart );
			
			for ( ; t && t <= now; )
			{
				//LOG4CPP_DEBUG( logger,  (*it)->getName() << "sending " << t  );
				(*it)->sendNext( recordStart, playbackStart );
				t = (*it)->getNextTime( recordStart, playbackStart );
				if ( !t ){
					LOG4CPP_NOTICE( logger, (*it)->getName() << " reached end of recording" );
					(*it)->sendFinished();
				}
				count++;
			}
			

			// update next event time
			if ( t && ( nextEventTime == 0 || t < nextEventTime ) )
				nextEventTime = t;
		}
		//LOG4CPP_NOTICE(logger, "count of send events:"<< count);
	}
}


// has to be here, after all class declarations
boost::shared_ptr< UECameraPlayerComponentBase > UECameraPlayerModule::createComponent( const std::string& type, const std::string& name,
	boost::shared_ptr< Graph::UTQLSubgraph > pConfig, const UECameraPlayerModule::ComponentKey& key, UECameraPlayerModule* pModule )
{
	if ( type == "UECameraPlayerImage" )
		return boost::shared_ptr< UECameraPlayerComponentBase >( new UECameraPlayerComponentImage( name, pConfig, key, pModule ) );


	UBITRACK_THROW( "Class " + type + " not supported by UECameraPlayer module" );
}


} } // namespace Ubitrack::Drivers


UBITRACK_REGISTER_COMPONENT( Ubitrack::Dataflow::ComponentFactory* const cf ) {
	// create list of supported types
	std::vector< std::string > UECameraPlayerComponents;

	UECameraPlayerComponents.push_back( "UECameraPlayerImage" );

	cf->registerModule< Ubitrack::Drivers::UECameraPlayerModule > ( UECameraPlayerComponents );
}

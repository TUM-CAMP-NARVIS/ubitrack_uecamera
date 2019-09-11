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
		, m_currentIndex( 0 )
		, m_outPortIntrinsicsColor( "OutputIntrinsicsColor", *this, boost::bind(&UECameraPlayerComponentImage::getCameraModelColor, this, _1) )
		, m_outPortIntrinsicsDepth( "OutputIntrinsicsDepth", *this, boost::bind(&UECameraPlayerComponentImage::getCameraModelDepth, this, _1) )
	{
		LOG4CPP_INFO( logger, "Created UECameraPlayerComponentImage using file \"" << key.get() << "\"." );

		pConfig->m_DataflowAttributes.getAttributeData("offset", m_offset);
		pConfig->m_DataflowAttributes.getAttributeData("speedup", m_speedup);
		std::string metadataFile = pConfig->m_DataflowAttributes.getAttributeString("file");

	    std::ifstream infile(metadataFile);
		metadata = json::parse(infile);

		std::string basePath = metadataFile.substr(0, metadataFile.length() - std::string("Metadata.json").length());
		// load all image paths
		std::regex rex{R"###(image_number_([0-9]*)\.raw32f)###"};
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
        //boost::filesystem::path tsFile( m_tsFile );
        //if( !boost::filesystem::exists( tsFile ) )
        //	UBITRACK_THROW( "file with timestamps does not exist, please check the path: \"" + m_tsFile + "\"");
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
			
			int32_t fileSizeColor, fileSizeDepth;
			std::unique_ptr<char[]> bufferColor = readFile(nameColor, fileSizeColor);
			std::unique_ptr<char[]> bufferDepth = readFile(nameDepth, fileSizeDepth);
			
			if (fileSizeColor != fileSizeDepth && fileSizeColor != width * height * 4)
			{
				// Skip this image, something is wrong
				LOG4CPP_INFO( logger, "Skipped image " << entry->second << " due to filesize errors." );
				continue;
			}
			
			m_events.push_back(Measurement::Timestamp(m_initialTS + entry->first * nanoseconds / fps));
			LOG4CPP_INFO( logger, "Loaded image " << entry->second );

			std::memcpy(imgColor.data, bufferColor.get(), fileSizeColor);
			std::memcpy(imgDepth.data, bufferDepth.get(), fileSizeDepth);

			imagesColor.push_back(imgColor);
			imagesDepth.push_back(imgDepth);
		}			

		// read file with timestamps and other data
		
		// generated for this example
		
		LOG4CPP_INFO( logger, "Done loading " << m_events.size() << " images defined in file \"" << m_tsFile << "\"." );
		
	}

	Measurement::Timestamp getFirstTime() const
	{
		if (!m_events.empty()) {
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
		if (!m_events.empty()) {
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

			// TODO send all data with the same timestamp

			Vision::Image::ImageFormatProperties fmt;

			fmt.imageFormat = Vision::Image::BGRA;
			fmt.channels = 4;
			fmt.bitsPerPixel = 32;

			// boost::shared_ptr<Vision::Image> colorImage = boost::shared_ptr<Vision::Image>(new Vision::Image(640, 480, 3, CV_8U, 0));
			boost::shared_ptr<Vision::Image> colorImage = boost::shared_ptr<Vision::Image>(new Vision::Image(imagesColor[m_currentIndex], fmt));
			m_outPortColor.send( Measurement::ImageMeasurement( ts, colorImage ) );

			fmt.imageFormat = Vision::Image::DEPTH;
			fmt.channels = 1;
			fmt.bitsPerPixel = 16;

			// QUESTION. Is this supposed to be m_outPortDepth?
			//boost::shared_ptr<Vision::Image> depthImage = boost::shared_ptr<Vision::Image>(new Vision::Image(640, 480, 1, CV_16U, 0));

			boost::shared_ptr<Vision::Image> depthImage = boost::shared_ptr<Vision::Image>(new Vision::Image(imagesDepth[m_currentIndex], fmt));
			m_outPortDepth.send(Measurement::ImageMeasurement(ts, depthImage));
			
			LOG4CPP_INFO( logger, "Send image " << m_currentIndex << "\"." );

			m_currentIndex++;
			m_events.pop_front();
		}
	}

    Measurement::Pose getCameraPoseColor(Measurement::Timestamp t)
    {
        std::string sPitch = metadata["color_image"]["rot_pitch"];
        std::string sRoll = metadata["color_image"]["rot_roll"];
        std::string sYaw = metadata["color_image"]["rot_yaw"];

        double pitch = std::stod(sPitch);
        double yaw = std::stod(sYaw);
        double roll = std::stod(sRoll);

        std::string sXpos = metadata["color_image"]["pos_x"];
        std::string sYpos = metadata["color_image"]["pos_y"];
        std::string sZpos = metadata["color_image"]["pos_z"];

		double xPos  = std::stod(sXpos);
		double yPos  = std::stod(sYpos);
		double zPos  = std::stod(sZpos);
		
		Math::Quaternion rotation = getQuaternionFromYPR(yaw, pitch, roll);

        Math::Vector3d position(xPos, yPos, zPos);                                                                 
        Math::Pose pose(rotation, position);
		return Measurement::Pose(t, pose);
	}

	Measurement::Pose getCameraPoseDepth(Measurement::Timestamp t)
	{
        std::string sPitch = metadata["depth_image"]["rot_pitch"];
        std::string sRoll = metadata["depth_image"]["rot_roll"];
        std::string sYaw = metadata["depth_image"]["rot_yaw"];

        double pitch = std::stod(sPitch);
        double yaw = std::stod(sYaw);
        double roll = std::stod(sRoll);

        std::string sXpos = metadata["depth_image"]["pos_x"];
        std::string sYpos = metadata["depth_image"]["pos_y"];
        std::string sZpos = metadata["depth_image"]["pos_z"];

        double xPos = std::stod(sXpos);
        double yPos = std::stod(sYpos);
        double zPos = std::stod(sZpos);

		Math::Quaternion rotation = getQuaternionFromYPR(yaw, pitch, roll);

        Math::Vector3d position(xPos, yPos, zPos);                                                                 
        Math::Pose pose(rotation, position);
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

	Math::Quaternion getQuaternionFromYPR(double yaw, double pitch, double roll)
	{
		const double PI = 3.14159265359;

		double cy = std::cos(PI * yaw * 0.5 / 180.0);
		double sy = std::sin(PI * yaw * 0.5 / 180.0);
		double cp = std::cos(PI * pitch * 0.5 / 180.0);
		double sp = std::sin(PI * pitch * 0.5 / 180.0);
		double cr = std::cos(PI * roll * 0.5 / 180.0);
		double sr = std::sin(PI * roll * 0.5 / 180.0);
		
		Math::Quaternion rotation(
			cy * cp * cr + sy * sp * sr,
			cy * cp * sr - sy * sp * cr,
			sy * cp * sr + cy * sp * cr,
			sy * cp * cr - cy * sp * sr);

		return rotation;
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
	std::vector<cv::Mat> imagesColor;
	std::vector<cv::Mat> imagesDepth;
	std::map<int, std::string> allImagesBasenames;
	int32_t m_currentIndex;
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

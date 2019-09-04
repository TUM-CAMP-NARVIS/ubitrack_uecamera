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

// Boost
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp>

#include <utVision/Image.h>
#include <opencv/highgui.h>

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Drivers.UEPlayer" ) );

namespace Ubitrack { namespace Drivers {

// forward decls
class UECameraPlayerComponentBase;


/**
 * Component key for UECameraPlayerProducer/Consumer components.
 */
class UECameraPlayerComponentKey
	: public Dataflow::EdgeAttributeKey< std::string >
{
public:
	/** extract the "file" parameter of the edge config. */
	UECameraPlayerComponentKey( boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::EdgeAttributeKey< std::string >( pConfig, "Output", "file" )
	{}
};


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
		, m_outPortPose( "OutputPose", *this )
		, m_outPortIntrinsics( "OutputPose", *this, boost::bind(&UECameraPlayerComponentImage::getCameraModel, this, _1) )
	{
		LOG4CPP_INFO( logger, "Created UECameraPlayerComponentImage using file \"" << key.get() << "\"." );

		// read configuration
		pConfig->getEdge( "Output" )->getAttributeData( "offset", m_offset );
		pConfig->getEdge( "Output" )->getAttributeData( "speedup", m_speedup );

		// get the file which describes the timestamps and filenames
		pConfig->getEdge( "Output" )->getAttributeData( "file", m_tsFile );
		
		// start loading images in a thread
		if ( !m_pLoadThread )
			m_pLoadThread.reset( new boost::thread( boost::bind( &UECameraPlayerComponentImage::loadImages, this ) ) );
			
		// before starting the component let the tread do some work on loading 
		Util::sleep( 300 );
	}
	
	void loadImages()
	{
		boost::filesystem::path tsFile( m_tsFile );
		if( !boost::filesystem::exists( tsFile ) )
			UBITRACK_THROW( "file with timestamps does not exist, please check the path: \"" + m_tsFile + "\"");
			

		// read file with timestamps and filenames
		std::ifstream ifs( m_tsFile.c_str() );
		if( !ifs.is_open ( ) )
			UBITRACK_THROW( "Could not open file \"" + m_tsFile  + "\". This file should contain the timestamps and filenames of the images." );

		LOG4CPP_INFO( logger, "Starting to load images defined in file \"" << m_tsFile << "\"." );

		while( ifs.good() ) // && this->isRunning() ) <- "isRunning" does not work here, but not really necessary
		{
			// read contents line by line
			// read the image file and add the image and the timestamp to an event queue
			std::string temp;
			getline( ifs, temp );
			
			LOG4CPP_TRACE( logger, "Loading image file for log line " <<  temp );

			// parsing line with stringstreams
			// we have the fileformat "1272966027407 CameraRaw01249.jpg"
			std::stringstream ss ( std::stringstream::in | std::stringstream::out );
			ss << temp; // parse the line
			
			// declare and initialize the variable for the timestamp 
			Measurement::Timestamp timeStamp = 0;
			ss >> timeStamp;
			
			// declare and initialize the variable for the name of the image 
			std::string fileName;
			ss >> fileName;
			
			if ( fileName.empty() )
				continue;

		  // mh: "front()" is c++11 standard
		  // if ( fileName.front() == '"' ) {

			if ( *(fileName.begin()) == '"' )
			{
				fileName.erase( 0, 1 ); // erase the first character
				fileName.erase( fileName.size() - 1 ); // erase the last character
			}
  		  
		 	// check for length of timestamp to decide if timestamp is ms or ns
			// (we just need ms here, no need to be more accurate)
			//if( timeStamp > 1e13 )
			//	timeStamp = static_cast< Measurement::Timestamp >( timeStamp * 1e-06 );
			
			boost::filesystem::path file( fileName );
			boost::filesystem::file_status fstatus( boost::filesystem::status( file ) );
			
			//check if path to file is given absolute
			if( !boost::filesystem::exists( fstatus ) )
			{
				//no absolute path, check relative path to timestampfile:
				file = boost::filesystem::path( tsFile.parent_path().string() + "/" + fileName );
				if( !boost::filesystem::exists( file ) )
					continue;
			}
			
			//logging
			LOG4CPP_TRACE( logger, "loading image file " <<  file.string() << " for frame " <<  timeStamp );

			// Load the image
			// assigning the NULL avoids memory leaks
			cv::Mat img;

			
			try
			{
				if (file.extension() == ".BoostBinary") {
					Vision::Image tmp;
					Util::readBinaryCalibFile(file.string(), tmp);
					img = tmp.Mat();
				}
				else {
					img = cv::imread(file.string(), CV_LOAD_IMAGE_UNCHANGED);
				}
				
			}
			catch( std::exception& e )
			{
				LOG4CPP_ERROR( logger, "loading image file \"" << file.string() << "\" failed: " << e.what() );
				continue;
			}
			catch ( ... )
			{
				LOG4CPP_ERROR(logger, "loading image file \"" << file.string() << "\" failed: other reason");
				continue;
			}
			
			if( img.total() == 0 )
			{
				LOG4CPP_ERROR( logger, "loading image file \"" <<  file.string() << "\" failed." );
				continue;
			}

			// @todo: define ImageFormatProperties here to match the decoded images.

			// convert loaded image into the required pImage class
			boost::shared_ptr< Vision::Image > pImage( new Vision::Image( img ) );

			// @todo make imgformat configurable on image UECameraPlayer ??
			
			// Building the event and packing timestamp and image into it
			Measurement::ImageMeasurement e( static_cast< Measurement::Timestamp>(  timeStamp), pImage );

			// Store it
			m_events.push_back( e );
		}
		LOG4CPP_INFO( logger, "Done loading " << m_events.size() << " images defined in file \"" << m_tsFile << "\"." );
		
	}

	Measurement::Timestamp getFirstTime() const
	{
		if ( !m_events.empty() )
			return m_events.begin()->time() + 1000000LL * m_offset;
		else
			return 0;
	}

	/** return time of the next measurement to be played or 0 if no events */
	Measurement::Timestamp getNextTime( Measurement::Timestamp recordStart, Measurement::Timestamp playbackStart )
	{
		if( !m_events.empty() )
			return recordTimeToReal( m_events.begin()->time(), recordStart, playbackStart );
		else
			return 0;
	}

	/** send the next event */
	void sendNext( Measurement::Timestamp recordStart, Measurement::Timestamp playbackStart )
	{
		if( !m_events.empty() )
		{
			m_outPort.send( Measurement::ImageMeasurement( recordTimeToReal( m_events.begin()->time(), recordStart, playbackStart ), m_events.front() ) );
			m_events.pop_front();
		}
	}

	Measurement::CameraIntrinsics getCameraModel(Measurement::Timestamp t)
    {
		
        return Measurement::CameraIntrinsics(t, );
    }
protected:

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
	Dataflow::PushSupplier< Measurement::Pose > m_outPortPose;
	Dataflow::PullSupplier< Measurement::CameraIntrinsics > m_outPortIntrinsics;

	/** queue for the images being loaded, @todo add mutex for accessing m_events */
	std::deque< Measurement::ImageMeasurement > m_events;
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
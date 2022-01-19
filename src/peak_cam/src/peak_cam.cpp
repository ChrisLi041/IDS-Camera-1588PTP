
// Copyright (c) 2020, Sherif Nekkah 
// All rights reserved. 
// 
// DISCLAMER:
//
//
// This package was created and used within an academic project and should
// be considered as experimental code. There may be bugs and deficiencies in the
// software. Feel free for suggestions, pull requests or any possible issue. 
//
//
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met: 
// 
//  * Redistributions of source code must retain the above copyright notice, 
//    this list of conditions and the following disclaimer. 
//  * Redistributions in binary form must reproduce the above copyright 
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the distribution. 
//  * Neither the name of  nor the names of its contributors may be used to 
//    endorse or promote products derived from this software without specific 
//    prior written permission. 
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE. 



#include "peak_cam.hpp"


namespace peak_cam
{

Peak_Cam::Peak_Cam(ros::NodeHandle nh) : nh_private(nh)
{
  std::string camera_topic;
  nh_private.getParam("camera_topic", camera_topic);

  ROS_INFO("Setting parameters to:");
  ROS_INFO("  camera_topic: %s", camera_topic.c_str());

  image_publisher = nh.advertise<sensor_msgs::Image>(camera_topic, 1);
 
  f = boost::bind(&Peak_Cam::reconfigureRequest, this, _1, _2);
  server.setCallback(f);

  peak::Library::Initialize();

  openDevice();
}

Peak_Cam::~Peak_Cam()
{
  ROS_INFO("Shutting down");

  // closing camera und peak library
  closeDevice();
  peak::Library::Close();

  ROS_INFO("Peak library closed");
  ros::shutdown();
}

void Peak_Cam::openDevice()
{
    ROS_INFO("Opening device");
    auto& deviceManager = peak::DeviceManager::Instance();  
    //Select Device and set Parameters Once
    while (!acquisitionLoop_running)
    {
        try
        {
            // update the device manager
            deviceManager.Update();
           
 	    // exit program if no device was found
            if (deviceManager.Devices().empty())
            {
               ROS_INFO("No device found. Exiting program");
               // close library before exiting program
               peak::Library::Close();
               return;
            }
            
	    // list all available devices
            size_t i = 0;
            ROS_INFO_ONCE("Devices available: ");
            for (const auto& deviceDescriptor : deviceManager.Devices())
            {
                ROS_INFO("%lu: %s", i, deviceDescriptor->DisplayName().c_str());
                ++i;
            }
            
	    // set i back to 0
            i = 0;
	    size_t selectedDevice = 0;
            for (const auto& deviceDescriptor : deviceManager.Devices())
            {
                if (peak_params.selectedDevice == deviceDescriptor->SerialNumber())
                {
                    ROS_INFO_ONCE("SELECTING NEW DEVICE: %lu", i);
		    selectedDevice = i;
		}
                ++i;
            }
	    
	    // get vector of device descriptors
            auto deviceDesrciptors = deviceManager.Devices();

            // open the selected device
            m_device = deviceManager.Devices().at(selectedDevice)->OpenDevice(peak::core::DeviceAccessType::Control);
            ROS_INFO_STREAM("[PEAK_CAM]: " << m_device->ModelName() << " found");

            // get the remote device node map
            m_nodeMapRemoteDevice = m_device->RemoteDevice()->NodeMaps().at(0); 

	    std::vector<std::shared_ptr<peak::core::nodes::Node>> nodes = m_nodeMapRemoteDevice->Nodes();

            // sets Acquisition Parameters of the camera -> see yaml
            setDeviceParameters();

            // open the first data stream
            m_dataStream = m_device->DataStreams().at(0)->OpenDataStream(); 
            // get payload size
            auto payloadSize = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("PayloadSize")->Value();
            
            // get number of buffers to allocate
            // the buffer count depends on your application, here the minimum required number for the data stream
            auto bufferCountMax = m_dataStream->NumBuffersAnnouncedMinRequired();
            
            // allocate and announce image buffers and queue them
            for (uint64_t bufferCount = 0; bufferCount < bufferCountMax; ++bufferCount)
            {
                auto buffer = m_dataStream->AllocAndAnnounceBuffer(static_cast<size_t>(payloadSize), nullptr);
                m_dataStream->QueueBuffer(buffer);
            }    

            // start the data stream
            m_dataStream->StartAcquisition(peak::core::AcquisitionStartMode::Default, peak::core::DataStream::INFINITE_NUMBER);
            // start the device

            //Chris DEBUG PTP
            //m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStart")->Execute();

            ROS_INFO_STREAM("[PEAK_CAM]: " << m_device->ModelName() << " connected");

            acquisitionLoop_running = true;

        }
        catch (const std::exception& e)
        {
            ROS_ERROR_STREAM_ONCE("[PEAK_CAM]: EXCEPTION: " << e.what());
            ROS_ERROR_STREAM("[PEAK_CAM]: Device at port " << peak_params.selectedDevice << " not connected or must run as root!");
            ROS_ERROR_STREAM("[PEAK_CAM]: Check if IP address is correct and make sure IDS Peak is closed.");

        }
    }
}

void Peak_Cam::setDeviceParameters()
{   
    try
    {
        int maxWidth, maxHeight = 0;
        
        //YZ DEBUG ADD PTP Parameters: 
        //_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("SignalMultiplierEnable")->SetValue(peak_params.SignalMultiplierEnable);
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("PtpEnable")->SetValue(peak_params.PtpEnable);
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("PtpSlaveOnly")->SetValue(peak_params.PtpSlaveOnly);

        //Chris DEBUG
        //bool value = nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("PtpSlaveOnly")->Value();
        //nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("PtpSlaveOnly")->SetValue(true);
        //nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("SignalMultiplierEnable")->SetValue(false);
        ROS_INFO_STREAM("[PEAK_CAM]: SignalMultiplierEnable is set to '" << peak_params.SignalMultiplierEnable << "'");
        ROS_INFO_STREAM("[PEAK_CAM]: PtpEnable is set to '" << peak_params.PtpEnable << "'");
        ROS_INFO_STREAM("[PEAK_CAM]: PtpSlaveOnly is set to '" << peak_params.PtpSlaveOnly << "'");
        std::string PtpStatus_val = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PtpStatus")->CurrentEntry()->SymbolicValue();
        ROS_INFO_STREAM("[PEAK_CAM]: PtpStatus is set to '" << PtpStatus_val << "'");

        //Chris DEBUG
        //timestamping
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("ChunkModeActive")->SetValue(true);
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("ChunkSelector")->SetCurrentEntry("Timestamp");
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("ChunkEnable")->SetValue(true);
        // Determine the current status of ChunkModeActive
        bool chunk_md_act = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("ChunkModeActive")->Value();
        ROS_INFO_STREAM("[PEAK_CAM]: ChunkModeActive is set to '" << chunk_md_act << "'");
        std::string chunk_selector = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("ChunkSelector")->CurrentEntry()->SymbolicValue();
        ROS_INFO_STREAM("[PEAK_CAM]: ChunkSelector is set to '" << chunk_selector << "'");
        bool chunk_en = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("ChunkEnable")->Value();
        ROS_INFO_STREAM("[PEAK_CAM]: ChunkEnable is set to '" << chunk_en << "'");
        //set offset
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("OffsetX")->SetValue(0);
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("OffsetY")->SetValue(0);

        //
        maxWidth = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("WidthMax")->Value();
        ROS_INFO_STREAM("[PEAK_CAM]: maxWidth '" << maxWidth << "'");
        maxHeight = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("HeightMax")->Value();
        ROS_INFO_STREAM("[PEAK_CAM]: maxHeight '" << maxHeight << "'");
        // Set Width, Height
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Width")->SetValue(peak_params.ImageWidth);
        ROS_INFO_STREAM("[PEAK_CAM]: ImageWidth is set to '" << peak_params.ImageWidth << "'");
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Height")->SetValue(peak_params.ImageHeight);
        ROS_INFO_STREAM("[PEAK_CAM]: ImageHeight is set to '" << peak_params.ImageHeight << "'");
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("OffsetX")->SetValue((maxWidth - peak_params.ImageWidth) / 2);
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("OffsetY")->SetValue((maxHeight - peak_params.ImageHeight) / 2);
	
        //Set GainAuto Parameter
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainAuto")->SetCurrentEntry(peak_params.GainAuto);
        ROS_INFO_STREAM("[PEAK_CAM]: GainAuto is set to '" << peak_params.GainAuto << "'");

        //Set GainSelector Parameter
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")->SetCurrentEntry(peak_params.GainSelector);
        ROS_INFO_STREAM("[PEAK_CAM]: GainSelector is set to '" << peak_params.GainSelector << "'");

        //Set ExposureAuto Parameter
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("ExposureAuto")->SetCurrentEntry(peak_params.ExposureAuto);
        ROS_INFO_STREAM("[PEAK_CAM]: ExposureAuto is set to '" << peak_params.ExposureAuto << "'");


        //Set ExposureTime Parameter
        if(peak_params.ExposureAuto == "Off")
        {
            m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("ExposureTime")->SetValue(peak_params.ExposureTime);
            ROS_INFO_STREAM("[PEAK_CAM]: ExposureTime is set to " << peak_params.ExposureTime << " microseconds");
        }

        //Set DeviceLinkThroughputLimit Parameter
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("DeviceLinkThroughputLimit")->SetValue(peak_params.DeviceLinkThroughputLimit);
        float linkRate = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("DeviceLinkAcquisitionFrameRateLimit")->Value();
        ROS_INFO_STREAM("[PEAK_CAM]: DeviceLinkThroughputLimit is set to " << peak_params.DeviceLinkThroughputLimit << " Bps allowing for " << linkRate << " Hz");
        
        //Configure Trigger and set AcquisitionFrameRate Parameter
        
        std::cout << "TRIGGER SOURCE: "<<peak_params.TriggerSource << std::endl;
        if (peak_params.TriggerSource == "Off")
        {
            //m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")->SetValue(peak_params.AcquisitionFrameRate);
            //ROS_INFO_STREAM("[PEAK_CAM]: AcquisitionFrameRate is set to " << peak_params.AcquisitionFrameRate << " Hz");



            
            if(linkRate < peak_params.AcquisitionFrameRate){
                ROS_ERROR_STREAM("[PEAK_CAM]: AcquisitionFrameRate is higher than DeviceLinkAcquisitionFrameRateLimit! Expect latency and buffer overflows!");
            }
            
        } else {


            //Chris DEBUG
            //m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("AcquisitionFrameRateTargetEnable")->SetValue(false);
            //m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRateTarget")->SetValue(10.0);
            //ROS_INFO_STREAM("[PEAK_CAM]: AcquisitionFrameRateTarget is set to '" << peak_params.SignalMultiplierEnable << "'");
            


            std::cout << "SET THE AC FRAME RATE TARGET" << std::endl;

            //YZ ADDING THE LINE TRIGGER FEATURE
            m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("LineSelector")->SetCurrentEntry("Line2");
            m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("LineMode")->SetCurrentEntry("Output");
            m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("LineSource")->SetCurrentEntry("PPS");

            m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TimerSelector")->SetCurrentEntry("Timer0");
            m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("TimerDuration")->SetValue(990000.0);
            // m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("TimerDelay")->SetValue(91000);
            m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TimerTriggerSource")->SetCurrentEntry("Line2");
            m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TimerTriggerActivation")->SetCurrentEntry("RisingEdge");


            m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PWMSelector")->SetCurrentEntry("PWM0");
            m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PWMTriggerSource")->SetCurrentEntry("Timer0Active");
            m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("PWMFrequency")->SetValue(10.0);
            m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PWMTriggerActivation")->SetCurrentEntry("LevelHigh");
            m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("PWMDutyCycle")->SetValue(50);

            // m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerMode")->SetCurrentEntry("On");
            m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSource")->SetCurrentEntry("PWM0");
            m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerActivation")->SetCurrentEntry("RisingEdge");
            m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("TriggerDelay")->SetValue(91000);
            m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSelector")->SetCurrentEntry("ExposureEnd");
            

            //m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerMode")->SetCurrentEntry("Off");
            //m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerActivation")->SetCurrentEntry(peak_params.TriggerActivation);
            //m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("TriggerDivider")->SetValue(peak_params.TriggerDivider);
            //m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSource")->SetCurrentEntry(peak_params.TriggerSource);
            ROS_INFO_STREAM("[PEAK_CAM]: No AcquisitionFrameRate is set, camera is expected to be externally triggered by " << peak_params.TriggerSource << " on " << peak_params.TriggerActivation);
            ROS_INFO_STREAM("[PEAK_CAM]: TriggerDivider is set to " << peak_params.TriggerDivider);
            ROS_WARN_STREAM("[PEAK_CAM]: Make sure resulting trigger rate stays below " << linkRate << " Hz to avoid latency and buffer overflows!");
        }
        

        //Set Line1 (flash output) signal source
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("LineSelector")->SetCurrentEntry("Line1");
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("LineSource")->SetCurrentEntry("ExposureActive");
        ROS_INFO_STREAM("[PEAK_CAM]: Flash output configured to " << peak_params.Line1Source);

        //Set Gamma Parameter
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gamma")->SetValue(peak_params.Gamma);
        ROS_INFO_STREAM("[PEAK_CAM]: Gamma is set to " << peak_params.Gamma);

        //Set PixelFormat Parameter
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat")->SetCurrentEntry(peak_params.PixelFormat);
        ROS_INFO_STREAM("[PEAK_CAM]: PixelFormat is set to '" << peak_params.PixelFormat << "'");
        
        //Set Parameters for ROS Image
        if (peak_params.PixelFormat == "Mono8")
        {
            pixel_format_name = peak::ipl::PixelFormatName::Mono8;
            image_for_encoding.encoding = sensor_msgs::image_encodings::MONO8;
        
        }
        else if(peak_params.PixelFormat == "RGB8")
        {
            pixel_format_name = peak::ipl::PixelFormatName::RGB8;
            image_for_encoding.encoding = sensor_msgs::image_encodings::RGB8;
        
        }        
        else if(peak_params.PixelFormat == "BGR8")
        {
            pixel_format_name = peak::ipl::PixelFormatName::BGR8;
            image_for_encoding.encoding = sensor_msgs::image_encodings::BGR8;
        
        } 
             
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("[PEAK_CAM]: EXCEPTION: " << e.what());
        ROS_ERROR("[PEAK_CAM]: Could not set all Parameters");
    }
    
}



void Peak_Cam::acquisitionLoop()
{
    int flag = 0;
    int banner = 0;
    while (acquisitionLoop_running)
    {
        try
        {   
            //Chris DEBUG PTP
            ROS_INFO_ONCE("[PEAK_CAM]: Waiting for Acquisition ... ...");
            ROS_INFO_STREAM("[PEAK_CAM]: PtpStatus is now '" << m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PtpStatus")->CurrentEntry()->SymbolicValue() << "'");
            if (m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PtpStatus")->CurrentEntry()->SymbolicValue() == "Slave")
            //Acquisition can start iff PtpStatus is in "Slave" mode
            {
                if(flag == 0)
                {
                    flag = 1;
                    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStart")->Execute();
                    ROS_INFO_STREAM("[PEAK_CAM]: PTP connected successfully! Acquisition started!");
                    double frame_val = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRateTarget")->Value();
                    ROS_INFO_STREAM("[PEAK_CAM]: AcquisitionFrameRateTarget is now '" << frame_val << "'");
                    //int64_t timestmp = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("ChunkExposureTriggerTimestamp")->Value();
                    //ROS_INFO_STREAM("[PEAK_CAM]: Timestamp is '" << timestmp << "'");
                    
                    // bool chunk_md_act = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("ChunkModeActive")->Value();
                    // ROS_INFO_STREAM("[PEAK_CAM]: ChunkModeActive is set to '" << chunk_md_act << "'");
                    // std::string chunk_selector = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("ChunkSelector")->CurrentEntry()->SymbolicValue();
                    // ROS_INFO_STREAM("[PEAK_CAM]: ChunkSelector is set to '" << chunk_selector << "'");
                    // bool chunk_en = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("ChunkEnable")->Value();
                    // ROS_INFO_STREAM("[PEAK_CAM]: ChunkEnable is set to '" << chunk_en << "'");
                }
                else
                {
                    ROS_INFO_STREAM("[PEAK_CAM]: Camera running... ...");
                }
                
            }
            else if (m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PtpStatus")->CurrentEntry()->SymbolicValue() == "Faulty")
            {
                m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("PtpEnable")->SetValue(peak_params.PtpEnable);
                m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("PtpSlaveOnly")->SetValue(peak_params.PtpSlaveOnly);
            }
            else if (m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PtpStatus")->CurrentEntry()->SymbolicValue() == "Listening")
            {
                ROS_INFO_STREAM("[PEAK_CAM]: Waiting for PTP");
            }
            else if (m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PtpStatus")->CurrentEntry()->SymbolicValue() == "Uncalibrated")
            {
                ROS_INFO_STREAM("[PEAK_CAM]: PTP enabled, waiting for connection");
            }
            else if (m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PtpStatus")->CurrentEntry()->SymbolicValue() == "Disabled")
            {
                ROS_INFO_STREAM("[PEAK_CAM]: Trying to reinitialize PTP connection...");
                m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("PtpEnable")->SetValue(peak_params.PtpEnable);
                m_nodeMapRemoteDevice->FindNode<peak::core::nodes::BooleanNode>("PtpSlaveOnly")->SetValue(peak_params.PtpSlaveOnly);
            }
            // get buffer from data stream and process it
            auto buffer = m_dataStream->WaitForFinishedBuffer(15000); //was 5000
            
            // auto ts = 0;
            if (buffer->HasChunks()&&m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PtpStatus")->CurrentEntry()->SymbolicValue() != "")
            {
                m_nodeMapRemoteDevice->UpdateChunkNodes(buffer); 
                auto ts = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("ChunkTimestamp")->Value();
                ROS_INFO_STREAM("[PEAK_CAM]: Timestamp is '" << ts << "'");
            };
                
            //.
            // Create IDS peak IPL image for debayering and convert it to RGBa8 format
            //auto image = peak::BufferTo<peak::ipl::Image>(buffer);
            //image = image.ConvertTo(peak::ipl::PixelFormatName::BGRa8);
        
            // Queue buffer so that it can be used again
            //m_dataStream->QueueBuffer(buffer);
            //

            // buffer processing start
            auto image = peak::BufferTo<peak::ipl::Image>(buffer).ConvertTo(pixel_format_name);

            cv::Mat cvImage;
            if (peak_params.PixelFormat == "Mono8")
                cvImage = cv::Mat::zeros(image.Height(), image.Width(), CV_8UC1);
            else
                cvImage = cv::Mat::zeros(image.Height(), image.Width(), CV_8UC3);

            int sizeBuffer = static_cast<int>(image.ByteCount());

            // Device buffer is being copied into cv_bridge format
            std::memcpy(cvImage.data, image.Data(), static_cast<size_t>(sizeBuffer));

            // cv_bridge Image is converted to sensor_msgs/Image to publish on ROS Topic
            cv_bridge::CvImage cvBridgeImage;
            int nano = 1e9;
            // ROS_INFO_STREAM("[PEAK_CAM]: type is '" << typeid(ts).name() << "'");
            cvBridgeImage.header.stamp.sec = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("ChunkTimestamp")->Value() / nano;
            cvBridgeImage.header.stamp.nsec = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("ChunkTimestamp")->Value() % nano;
            cvBridgeImage.header.frame_id = peak_params.selectedDevice;
            cvBridgeImage.encoding = image_for_encoding.encoding; 
            cvBridgeImage.image = cvImage; 

            image_publisher.publish(cvBridgeImage.toImageMsg());    

            ROS_INFO_STREAM_ONCE("[PEAK_CAM]: Publishing data");

            // queue buffer
            m_dataStream->QueueBuffer(buffer);
            
        }
        catch (const std::exception& e)
        {
            // if (banner == 1)
            // {
            ROS_ERROR_STREAM("[PEAK_CAM]: EXCEPTION: " << e.what());
            ROS_ERROR("[PEAK_CAM]: Acquisition loop stopped, device may be disconnected!");
            ROS_ERROR("[PEAK_CAM]: No device reset available");
            ROS_ERROR("[PEAK_CAM]: Restart peak cam node!");
            acquisitionLoop_running = false;
        
            // }
            // else
            // {
            //     banner = 1;
            //     ROS_INFO_STREAM_ONCE("[PEAK_CAM]: Waiting for PTP ... ...");
            // };
            // ROS_ERROR_STREAM("[PEAK_CAM]: EXCEPTION: " << e.what());
            // ROS_ERROR("[PEAK_CAM]: Acquisition loop stopped, device may be disconnected!");
            // ROS_ERROR("[PEAK_CAM]: No device reset available");
            // ROS_ERROR("[PEAK_CAM]: Restart peak cam node!");
            

        }        
    }
}

void Peak_Cam::closeDevice()
{
    // if device was opened, try to stop acquisition
    if (m_device)
    {
        try
        {
            m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStop")->Execute();
            ROS_INFO("Executing 'AcquisitionStop'");
            acquisitionLoop_running = false;
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_STREAM("EXCEPTION: " << e.what());
        }
    }

    // if data stream was opened, try to stop it and revoke its image buffers
    if (m_dataStream)
    {
        try
        {
            m_dataStream->KillWait(); //->KillOneWait();
            m_dataStream->StopAcquisition(peak::core::AcquisitionStopMode::Default);
            m_dataStream->Flush(peak::core::DataStreamFlushMode::DiscardAll);

            for (const auto& buffer : m_dataStream->AnnouncedBuffers())
            {
                m_dataStream->RevokeBuffer(buffer);
            }

            ROS_INFO("'AcquisitionStop' Succesful");
            acquisitionLoop_running = false;
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_STREAM("EXCEPTION: " << e.what());
        }
    }
}

void Peak_Cam::reconfigureRequest(const Config &config, uint32_t level)
{
    //Chris DEBUG PTP
    peak_params.SignalMultiplierEnable = config.SignalMultiplierEnable;
    peak_params.PtpEnable = config.PtpEnable;
    peak_params.PtpSlaveOnly = config.PtpSlaveOnly;

    peak_params.ExposureTime = config.ExposureTime;
    peak_params.TriggerSource = config.TriggerSource;
    peak_params.TriggerActivation = config.TriggerActivation;
    peak_params.TriggerDivider = config.TriggerDivider;
    peak_params.Line1Source = config.Line1Source;
    peak_params.AcquisitionFrameRate = config.AcquisitionFrameRate;
    peak_params.Gamma = config.Gamma;

    peak_params.selectedDevice = config.selectedDevice;

    peak_params.GainAuto = config.GainAuto;
    peak_params.GainSelector = config.GainSelector;    
    peak_params.ExposureAuto = config.ExposureAuto;
    peak_params.PixelFormat = config.PixelFormat;

    peak_params.DeviceLinkThroughputLimit = config.DeviceLinkThroughputLimit;
    
    peak_params.ImageHeight = config.ImageHeight;
    peak_params.ImageWidth = config.ImageWidth;
}

} // namespace peak_cam

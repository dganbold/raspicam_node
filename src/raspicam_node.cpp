/*
Copyright (c) 2013, Broadcom Europe Ltd
Copyright (c) 2013, James Hughes
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
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

// We use some GNU extensions (basename)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#define VCOS_ALWAYS_WANT_LOGGING

#define VERSION_STRING "v1.2"

extern "C" {
#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

#include "host_applications/linux/apps/raspicam/RaspiCamControl.h"
#include "host_applications/linux/apps/raspicam/RaspiCLI.h"
}

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/SetCameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"

#include <semaphore.h>

//----------------------------------------------------------------------
// Private macros 
//----------------------------------------------------------------------

// Camera number to use - we only have one camera, indexed from 0.
#define CAMERA_NUMBER	0

// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT	0
#define MMAL_CAMERA_VIDEO_PORT		1
#define MMAL_CAMERA_CAPTURE_PORT	2

// Video format information
#define VIDEO_FRAME_RATE_NUM		30
#define VIDEO_FRAME_RATE_DEN		1

// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM	3

/// Interval at which we check for an failure abort during capture

int mmal_status_to_int(MMAL_STATUS_T status);

//----------------------------------------------------------------------
// Private types 
//----------------------------------------------------------------------

typedef struct
{
	int isInit;
	int width;						/// Requested width of image
	int height;						/// requested height of image
	int framerate;					/// Requested frame rate (fps)
	int quality;
	RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters

	MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
	MMAL_COMPONENT_T *encoder_component;   /// Pointer to the encoder component
	MMAL_CONNECTION_T *preview_connection; /// Pointer to the connection from camera to preview
	MMAL_CONNECTION_T *encoder_connection; /// Pointer to the connection from camera to encoder

	MMAL_POOL_T *camera_pool;		/// Pointer to the pool of buffers used by video output port
	ros::Publisher *image_pub;

} RASPIVID_STATE;

typedef struct
{
	unsigned char *buffer[2];		/// File handle to write buffer data to.
	RASPIVID_STATE *pstate;			/// pointer to our state in case required in callback
	int abort;						/// Set to 1 in callback if an error occurs to attempt to abort the capture
	int frame;
	int id;

} PORT_USERDATA;

//----------------------------------------------------------------------
// Persistent variables 
//----------------------------------------------------------------------

RASPIVID_STATE	state_srv;

ros::Publisher	image_pub;
ros::Publisher	camera_info_pub;
std::string		  tf_prefix;

sensor_msgs::CameraInfo c_info;

//----------------------------------------------------------------------
//  
//----------------------------------------------------------------------

// Checks if specified port is valid and enabled, then disables it
static void check_disable_port(MMAL_PORT_T *port)
{
	if(port && port->is_enabled){
		mmal_port_disable(port);
	}
}

// Checks if specified component is valid and enabled, then disables it
static void check_disable_component(MMAL_COMPONENT_T *camera)
{
	if(camera && camera->is_enabled){
		mmal_component_disable(camera);
	}
}

// Checks if specified component is valid, then destroy it
static void check_destroy_component(MMAL_COMPONENT_T *camera)
{
	if(camera){
		mmal_component_destroy(camera);
		camera = NULL;
	}
}

// Assign a default set of parameters to the state passed in
static void get_status(RASPIVID_STATE *state)
{
	int temp;
	std::string str;
	if (!state){
		vcos_assert(0);
		return;
	}
	// Default everything to zero
	memset(state, 0, sizeof(RASPIVID_STATE));

	// Parse parameter width
	if(ros::param::get("~width", temp)){
		if(temp > 0 && temp <= 1920){	
			state->width = temp;
		} else {
			state->width = 640;
		}
	} else {
		state->width = 640;
		ros::param::set("~width", 640);
	}
	// Parse parameter height
	if(ros::param::get("~height", temp)){
		if(temp > 0 && temp <= 1080){	
			state->height = temp;
		} else {
			state->height = 480;
		}
	} else {
		state->height = 480;
		ros::param::set("~height", 480);
	}
	// Parse parameter quality
	if(ros::param::get("~quality", temp )){
		if(temp > 0 && temp <= 100){
			state->quality = temp;
		} else {
			state->quality = 80;
		}
	} else {
		state->quality = 80;
		ros::param::set("~quality", 80);
	}
	// Parse parameter framerate
	if(ros::param::get("~framerate", temp )){
		if(temp > 0 && temp <= 90){
			state->framerate = temp;
		} else {
			state->framerate = 30;
		}
	} else {
		state->framerate = 30;
		ros::param::set("~framerate", 30);
	}
	// Parse parameter tf_prefix
	if(ros::param::get("~tf_prefix",  str)){
		tf_prefix = str;
	} else {
		tf_prefix = "";
		ros::param::set("~tf_prefix", "");
	}

	// Set up the camera_parameters to default
	raspicamcontrol_set_defaults(&state->camera_parameters);
	
	// Parse parameter videoStabilisation
	if(ros::param::get("~videoStabilisation", temp )){
		if(temp == 1){
			state->camera_parameters.videoStabilisation = 1;
		} else {
			state->camera_parameters.videoStabilisation = 0;
		}
	}
  // Parse parameter exposureMode
	if(ros::param::get("~exposureMode", temp )){
    switch(temp){
      case 0:  state->camera_parameters.exposureMode = MMAL_PARAM_EXPOSUREMODE_OFF;          break;
      case 1:  state->camera_parameters.exposureMode = MMAL_PARAM_EXPOSUREMODE_AUTO;         break;
      case 2:  state->camera_parameters.exposureMode = MMAL_PARAM_EXPOSUREMODE_NIGHT;        break;
      case 3:  state->camera_parameters.exposureMode = MMAL_PARAM_EXPOSUREMODE_NIGHTPREVIEW; break;
      case 4:  state->camera_parameters.exposureMode = MMAL_PARAM_EXPOSUREMODE_BACKLIGHT;    break;
      case 5:  state->camera_parameters.exposureMode = MMAL_PARAM_EXPOSUREMODE_SPOTLIGHT;    break;
      case 6:  state->camera_parameters.exposureMode = MMAL_PARAM_EXPOSUREMODE_SPORTS;       break;
      case 7:  state->camera_parameters.exposureMode = MMAL_PARAM_EXPOSUREMODE_BEACH;        break;
      case 8:  state->camera_parameters.exposureMode = MMAL_PARAM_EXPOSUREMODE_VERYLONG;     break;
      case 9:  state->camera_parameters.exposureMode = MMAL_PARAM_EXPOSUREMODE_FIXEDFPS;     break;
      case 10: state->camera_parameters.exposureMode = MMAL_PARAM_EXPOSUREMODE_ANTISHAKE;    break;
      case 11: state->camera_parameters.exposureMode = MMAL_PARAM_EXPOSUREMODE_FIREWORKS;    break;
      default: state->camera_parameters.exposureMode = MMAL_PARAM_EXPOSUREMODE_OFF;          break;
    }
	}
	// Parse parameter rotation
	if(ros::param::get("~rotation", temp )){
		if(temp > 0 && temp <= 360){
			state->camera_parameters.rotation = temp;
		} else {
			state->camera_parameters.rotation = 0;
		}
	}
  // Parse parameter brightness
	if(ros::param::get("~brightness", temp )){
		if(temp > 0 && temp <= 100){
			state->camera_parameters.brightness = temp;
		} else {
			state->camera_parameters.brightness = 0;
		}
	}
  // Parse parameter saturation
	if(ros::param::get("~saturation", temp )){
		if(temp > -100 && temp <= 100){
			state->camera_parameters.saturation = temp;
		} else {
			state->camera_parameters.saturation = 0;
		}
	}
	// Parse parameter shutter_speed
	if(ros::param::get("~shutter_speed", temp )){
		if(temp > 0 && temp <= 100000){
			state->camera_parameters.shutter_speed = temp;
		} else {
			state->camera_parameters.shutter_speed = 0;
		}
	}
 	// Parse parameter ISO
	if(ros::param::get("~iso", temp )){
		if(temp > 0 && temp <= 800){
			state->camera_parameters.ISO = temp;
		} else {
			state->camera_parameters.ISO = 0;
		}
	}
  // Parse parameter drc_level
	if(ros::param::get("~drc_level", temp )){
    switch(temp){
      case 0:  state->camera_parameters.drc_level = MMAL_PARAMETER_DRC_STRENGTH_OFF;     break;
      case 1:  state->camera_parameters.drc_level = MMAL_PARAMETER_DRC_STRENGTH_LOW;     break;
      case 2:  state->camera_parameters.drc_level = MMAL_PARAMETER_DRC_STRENGTH_MEDIUM;  break;
      case 3:  state->camera_parameters.drc_level = MMAL_PARAMETER_DRC_STRENGTH_HIGH;    break;
      default: state->camera_parameters.drc_level = MMAL_PARAMETER_DRC_STRENGTH_OFF;     break;
    }
	}
	//
	state->isInit = 0;
	// Setup preview window defaults
	//raspipreview_set_defaults(&state->preview_parameters);
}

//----------------------------------------------------------------------
//  
//----------------------------------------------------------------------

// Callback will dump buffer data to the specific file
static void camera_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	MMAL_BUFFER_HEADER_T *new_buffer;
	
	// We pass our file handle and other stuff in via the userdata field.
	PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
	if(pData && pData->pstate->isInit){

		int bytes_written = buffer->length;
		if(buffer->length){
			mmal_buffer_header_mem_lock(buffer);
			memcpy(&(pData->buffer[pData->frame & 1][pData->id]), buffer->data, buffer->length);
			pData->id += bytes_written;
			mmal_buffer_header_mem_unlock(buffer);
		}

		if(bytes_written != buffer->length){
			vcos_log_error("Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
			pData->abort = 1;
		}
		if(buffer->flags & (MMAL_BUFFER_HEADER_FLAG_FRAME_END | MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED)){
			sensor_msgs::Image msg;
			msg.header.seq = pData->frame;
			msg.header.frame_id = tf_prefix;
			msg.header.frame_id.append("/camera");
			msg.header.stamp = ros::Time::now();
			msg.height = pData->pstate->height;
			msg.width = pData->pstate->width;
			msg.encoding = "bgr8";
			msg.is_bigendian = 0;
			msg.step = pData->pstate->width*3;
			msg.data.insert( msg.data.end(), pData->buffer[pData->frame & 1], &(pData->buffer[pData->frame & 1][pData->id]) );
			image_pub.publish(msg);
			c_info.header.seq = pData->frame;
			c_info.header.stamp = msg.header.stamp;
			c_info.header.frame_id = msg.header.frame_id;
			camera_info_pub.publish(c_info);
			pData->frame++;
			pData->id = 0;	
		}
	} else {
		vcos_log_error("Received a encoder buffer callback with no state");
	}
	// release buffer back to the pool
	mmal_buffer_header_release(buffer);
	
	// and send one back to the port (if still open)
	if(port->is_enabled){
		MMAL_STATUS_T status;
		new_buffer = mmal_queue_get(pData->pstate->camera_pool->queue);

		if(new_buffer){
			status = mmal_port_send_buffer(port, new_buffer);
		}
		if(!new_buffer || status != MMAL_SUCCESS){
			vcos_log_error("Unable to return a buffer to the encoder port");
		}
	} else {
		ROS_INFO("oups");
	}
}

//----------------------------------------------------------------------
//  
//----------------------------------------------------------------------

// Create the camera component, set up its ports
static MMAL_COMPONENT_T *create_camera_component(RASPIVID_STATE *state)
{
	MMAL_COMPONENT_T	*camera		= NULL;
	MMAL_ES_FORMAT_T	*format		= NULL;
	MMAL_PORT_T			*video_port = NULL;
	MMAL_POOL_T			*pool		= NULL;
	MMAL_STATUS_T		status;
	
	// Create the component
	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
	if(status != MMAL_SUCCESS){
		vcos_log_error("Failed to create camera component");
		check_destroy_component(camera);
		return NULL;
	}
	// Check the output port
	if(!camera->output_num){
		vcos_log_error("Camera doesn't have output ports");
		check_destroy_component(camera);
		return NULL;
	}
	video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];

	// Set up the camera configuration
	{
		MMAL_PARAMETER_CAMERA_CONFIG_T cam_config;
		cam_config.hdr.id = MMAL_PARAMETER_CAMERA_CONFIG;
		cam_config.hdr.size = sizeof(cam_config);
		cam_config.max_stills_w = state->width;
		cam_config.max_stills_h = state->height;
		cam_config.stills_yuv422 = 0;
		cam_config.one_shot_stills = 0;
		cam_config.max_preview_video_w = state->width;
		cam_config.max_preview_video_h = state->height;
		cam_config.num_preview_video_frames = 3;
		cam_config.stills_capture_circular_buffer_height = 0;
		cam_config.fast_preview_resume = 0;
		cam_config.use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC;

		mmal_port_parameter_set(camera->control, &cam_config.hdr);
	}
	// Set up the port encode format on the video port
	format = video_port->format;
	format->encoding			= MMAL_ENCODING_RGB24;
	format->encoding_variant	= MMAL_ENCODING_RGB24;

	if(state->camera_parameters.shutter_speed > 6000000){
		MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},{ 50, 1000 }, {166, 1000}};
		mmal_port_parameter_set(video_port, &fps_range.hdr);
	} else if(state->camera_parameters.shutter_speed > 1000000){
		MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},{ 167, 1000 }, {999, 1000}};
		mmal_port_parameter_set(video_port, &fps_range.hdr);
	}

	format->es->video.width				= state->width;
	format->es->video.height			= state->height;
	format->es->video.crop.x			= 0;
	format->es->video.crop.y			= 0;
	format->es->video.crop.width		= state->width;
	format->es->video.crop.height		= state->height;
	format->es->video.frame_rate.num	= state->framerate;
	format->es->video.frame_rate.den	= VIDEO_FRAME_RATE_DEN;

	status = mmal_port_format_commit(video_port);
	if(status){
		vcos_log_error("camera video format couldn't be set");
		check_destroy_component(camera);
		return NULL;
	}

	// Ensure there are enough buffers to avoid dropping frames
	if(video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM){
		video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;
	}
	video_port->buffer_num = video_port->buffer_num_recommended;

	// Enable component
	status = mmal_component_enable(camera);
	if(status){
		vcos_log_error("camera component couldn't be enabled");
		check_destroy_component(camera);
		return NULL;
	}

	raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);
	
	// Create pool of buffer headers for the output port to consume
	pool = mmal_port_pool_create(video_port, video_port->buffer_num, video_port->buffer_size);
    if(!pool){
       vcos_log_error("Failed to create buffer header pool for camera port %s", video_port->name);
	}
    state->camera_pool		= pool;
    state->camera_component = camera;

	ROS_INFO("Camera component done\n");
	return camera;
}

// Connect two specific ports together
static MMAL_STATUS_T connect_ports(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection)
{
	MMAL_STATUS_T status;

	status =  mmal_connection_create(connection, output_port, input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
	if(status == MMAL_SUCCESS){
		status = mmal_connection_enable(*connection);
		if(status != MMAL_SUCCESS){
			mmal_connection_destroy(*connection);
		}
	}
	return status;
}

//----------------------------------------------------------------------
// Handlers 
//----------------------------------------------------------------------

//
static void signal_handler(int signal_number)
{
	// Going to abort on all signals
	vcos_log_error("Aborting program\n");

	// TODO : Need to close any open stuff...how?
	exit(255);
}

//----------------------------------------------------------------------
// Private functions 
//----------------------------------------------------------------------

//
int init_cam(RASPIVID_STATE *state)
{
	// Our main data storage vessel..
	MMAL_STATUS_T status;
	MMAL_PORT_T *camera_video_port = NULL;

	bcm_host_init();
	get_status(state);
	
	// Register our application with the logging system
	vcos_log_register("RaspiVid", VCOS_LOG_CATEGORY);

	signal(SIGINT, signal_handler);
   
	// Create camera componenet
	if(!create_camera_component(state)){
	   ROS_INFO("%s: Failed to create camera component", __func__);
	} else {
      
		PORT_USERDATA * callback_data = (PORT_USERDATA *) malloc (sizeof(PORT_USERDATA));
		camera_video_port = state->camera_component->output[MMAL_CAMERA_VIDEO_PORT];
		
		callback_data->buffer[0] = (unsigned char *) malloc ( state->width * state->height * 8 );
		callback_data->buffer[1] = (unsigned char *) malloc ( state->width * state->height * 8 );
		
		// Set up our userdata - this is passed though to the callback where we need the information.
		callback_data->pstate	= state;
		callback_data->abort	= 0;
		callback_data->id		= 0;
		callback_data->frame	= 0;
		
		camera_video_port->userdata = (struct MMAL_PORT_USERDATA_T *) callback_data;
		PORT_USERDATA *pData = (PORT_USERDATA *)camera_video_port->userdata;
		// Enable the video output port and tell it its callback function
		status = mmal_port_enable(camera_video_port, camera_buffer_callback);
		
		if(status != MMAL_SUCCESS){
			ROS_INFO("Failed to setup encoder output");
			return 1;
		}
		state->isInit = 1;
	}
	return 0;
}

//
int start_capture(RASPIVID_STATE *state){

	if(!(state->isInit)){
		// Initialize camera
		init_cam(state);
	}
	// Starting component connection stage
	MMAL_PORT_T *camera_video_port = state->camera_component->output[MMAL_CAMERA_VIDEO_PORT];

	// Set the value of a boolean parameter
    if(mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS){
		return 1;
	} else {
		ROS_INFO("Starting video capture (%d,%d,%d,%d)\n", state->width, state->height, state->quality, state->framerate);
	}
    // Send all the buffers to the video port
    {
	 	int num = mmal_queue_length(state->camera_pool->queue);
	 	for(int q=0; q<num; q++)
	 	{
	      	MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state->camera_pool->queue);

	      	if (!buffer){
	        	vcos_log_error("Unable to get a required buffer %d from pool queue", q);
			}
	       	
			if (mmal_port_send_buffer(camera_video_port, buffer)!= MMAL_SUCCESS){
	           	vcos_log_error("Unable to send a buffer to video output port (%d)", q);
			}
	 	}
	}
	ROS_INFO("Video capture started\n");
	
	return 0;
}

//
int close_cam(RASPIVID_STATE *state){
	
	if(state->isInit){
		state->isInit = 0;
		
		MMAL_COMPONENT_T *camera = state->camera_component;
		MMAL_PORT_T *camera_video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
		PORT_USERDATA * pData = (PORT_USERDATA *)camera_video_port->userdata;

		// Disable video port
		check_disable_port(camera_video_port);

		// Disable camera components
		check_disable_component(camera);

		// Destroy video component
		if(state->camera_pool){
			// Get rid of any port buffers first
			mmal_port_pool_destroy(camera_video_port, state->camera_pool);
		}
		free(pData->buffer[0]);
		free(pData->buffer[1]);

		// Destroy camera component
		check_destroy_component(camera);
		return 0;
	} else { 
		return 1;
	}
}

//----------------------------------------------------------------------
// Service callbacks
//----------------------------------------------------------------------

bool serv_start_cap( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res )
{
	start_capture(&state_srv);
	return true;
}


bool serv_stop_cap( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res )
{
	close_cam(&state_srv);
	return true;
}

bool serv_set_camera_info( std_srvs::Empty::Request &req, std_srvs::Empty::Response &res )
{
	return true;
}

//----------------------------------------------------------------------
// Main
//----------------------------------------------------------------------

int main(int argc, char **argv){
    /**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line. For programmatic
	* remappings you can use a different version of init() which takes remappings
	* directly, but for most command-line programs, passing argc and argv is the easiest
	* way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "raspicam_node");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;
	
	/* Info manager */
	camera_info_manager::CameraInfoManager c_info_man (n, "camera", "package://raspicam_ros/calibrations/camera.yaml");
	get_status(&state_srv);

	if(!c_info_man.loadCameraInfo ("package://raspicam_ros/calibrations/camera.yaml")){
		ROS_INFO("Calibration file missing. Camera not calibrated");
	} else {
		c_info = c_info_man.getCameraInfo ();
		ROS_INFO("Camera successfully calibrated");
	}

	/* Publisher to the topic */
	image_pub = n.advertise<sensor_msgs::Image>("camera/image_raw", 1);
	camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("camera/camera_info", 1);
	
	/* Parameter service */
	ros::ServiceServer start_cam = n.advertiseService("camera/start_capture",	serv_start_cap);
	ros::ServiceServer stop_cam  = n.advertiseService("camera/stop_capture",	serv_stop_cap);
	ros::ServiceServer info_cam  = n.advertiseService("camera/set_camera_info",	serv_set_camera_info);

	ros::spin();

	close_cam(&state_srv);
	return 0;
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------

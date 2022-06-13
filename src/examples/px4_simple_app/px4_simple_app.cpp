/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING INs
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <uORB/topics/gs.h>

#include <uORB/uORB.h>

#include <stdio.h>
#include <string.h>
#include <math.h>



#include<iostream>
#include<list>
#include <unistd.h>   // sleep function head file

using namespace std;

extern "C" __EXPORT int px4_simple_app_main(int argc, char *argv[]);


int px4_simple_app_main(int argc, char *argv[])
{

    PX4_INFO("This is gps setpoint test");
    list<float>GPS;
    list<float>::iterator iter;

    float latitude;
    float longitude;	
    float altitude;



    
    /*定义话题结构*/
    struct gs_s gps_setpoint;
    /*初始化数据*/
    memset(&gps_setpoint, 0, sizeof(gps_setpoint));

    /*公告主题*/
    orb_advert_t gps_sp_pub = orb_advertise(ORB_ID(gs), &gps_setpoint);


    /*数据赋值*/
  

	
    	cout<<"Please input desired latitude:"<<endl;
    	cin>>latitude;
        GPS.push_back(latitude);
	gps_setpoint.latitude = GPS.back();
        cin.clear();
	cin.ignore();

	

    	cout<<"Please input desired longitude:"<<endl;
	cin>>longitude;
        GPS.push_back(longitude);
	gps_setpoint.longitude = GPS.back();
        cin.clear();


    	cout<<"Please input desired altitude:"<<endl;
	cin>>altitude;
        GPS.push_back(altitude);
	gps_setpoint.altitude = GPS.back();




    /*发布数据*/
    gps_setpoint.timestamp = hrt_absolute_time();
    orb_publish(ORB_ID(gs), gps_sp_pub, &gps_setpoint);


    /*订阅数据，在copy之前，必须要订阅

    int gps_sp_sub_fd = orb_subscribe(ORB_ID(gs));

    struct gs_s data_copy;

    /copy数据
    orb_copy(ORB_ID(gs), gps_sp_sub_fd, &data_copy);

    打印
    PX4_WARN("subscriber received from gs topic::::\nlatitude:%f\nlongitude:%f\naltitude:%f\n",
    (double)data_copy.latitude,
    (double)data_copy.longitude,
    (double)data_copy.altitude);
	*/

    return 0;
}


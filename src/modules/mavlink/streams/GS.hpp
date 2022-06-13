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
 * THIS SOFTWAREIS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef GS_HPP
#define GS_HPP 

#include <uORB/SubscriptionInterval.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/gs.h>

class  MavlinkStreamMymessage : public MavlinkStream
{
public:
    const char *get_name() const override
    {
        return MavlinkStreamMymessage::get_name_static();
    }
    static const char *get_name_static()
    {
        return "GS_PARAM_INMAVLINK";
    }
    static uint16_t get_id_static()
    {
    	return MAVLINK_MSG_ID_GS_PARAM_INMAVLINK;
    }
    uint16_t get_id() override
    {
        return get_id_static();
    }

    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamMymessage(mavlink);
    }
    
    unsigned get_size() override
    {
        return MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }
private:
    explicit MavlinkStreamMymessage(Mavlink *mavlink) : MavlinkStream(mavlink){}
    uORB::Subscription _sub{ORB_ID(gs)}; 


    //发送函数
    bool send() override 
    {
        struct gs_s _gs;    
        if (_sub.update(&_gs)) 
        {
            mavlink_gs_param_inmavlink_t msg;

            msg.longitude= _gs.longitude;
            msg.altitude= _gs.altitude;
            msg.latitude= _gs.latitude;
            mavlink_msg_gs_param_inmavlink_send_struct(_mavlink->get_channel(), &msg);
            return true;
        }
        return false;


    }
};

#endif //GS_HPP


#pragma once
// MESSAGE GS_PARAM_INMAVLINK PACKING

#define MAVLINK_MSG_ID_GS_PARAM_INMAVLINK 227


typedef struct __mavlink_gs_param_inmavlink_t {
 float latitude; /*<  latitude value*/
 float longitude; /*<  longitude value*/
 float altitude; /*<  altitude value*/
} mavlink_gs_param_inmavlink_t;

#define MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_LEN 12
#define MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_MIN_LEN 12
#define MAVLINK_MSG_ID_227_LEN 12
#define MAVLINK_MSG_ID_227_MIN_LEN 12

#define MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_CRC 129
#define MAVLINK_MSG_ID_227_CRC 129



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GS_PARAM_INMAVLINK { \
    227, \
    "GS_PARAM_INMAVLINK", \
    3, \
    {  { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gs_param_inmavlink_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gs_param_inmavlink_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gs_param_inmavlink_t, altitude) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GS_PARAM_INMAVLINK { \
    "GS_PARAM_INMAVLINK", \
    3, \
    {  { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gs_param_inmavlink_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gs_param_inmavlink_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gs_param_inmavlink_t, altitude) }, \
         } \
}
#endif

/**
 * @brief Pack a gs_param_inmavlink message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param latitude  latitude value
 * @param longitude  longitude value
 * @param altitude  altitude value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gs_param_inmavlink_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float latitude, float longitude, float altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, altitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_LEN);
#else
    mavlink_gs_param_inmavlink_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GS_PARAM_INMAVLINK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_MIN_LEN, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_LEN, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_CRC);
}

/**
 * @brief Pack a gs_param_inmavlink message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param latitude  latitude value
 * @param longitude  longitude value
 * @param altitude  altitude value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gs_param_inmavlink_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float latitude,float longitude,float altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, altitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_LEN);
#else
    mavlink_gs_param_inmavlink_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GS_PARAM_INMAVLINK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_MIN_LEN, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_LEN, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_CRC);
}

/**
 * @brief Encode a gs_param_inmavlink struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gs_param_inmavlink C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gs_param_inmavlink_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gs_param_inmavlink_t* gs_param_inmavlink)
{
    return mavlink_msg_gs_param_inmavlink_pack(system_id, component_id, msg, gs_param_inmavlink->latitude, gs_param_inmavlink->longitude, gs_param_inmavlink->altitude);
}

/**
 * @brief Encode a gs_param_inmavlink struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gs_param_inmavlink C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gs_param_inmavlink_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gs_param_inmavlink_t* gs_param_inmavlink)
{
    return mavlink_msg_gs_param_inmavlink_pack_chan(system_id, component_id, chan, msg, gs_param_inmavlink->latitude, gs_param_inmavlink->longitude, gs_param_inmavlink->altitude);
}

/**
 * @brief Send a gs_param_inmavlink message
 * @param chan MAVLink channel to send the message
 *
 * @param latitude  latitude value
 * @param longitude  longitude value
 * @param altitude  altitude value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gs_param_inmavlink_send(mavlink_channel_t chan, float latitude, float longitude, float altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, altitude);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK, buf, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_MIN_LEN, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_LEN, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_CRC);
#else
    mavlink_gs_param_inmavlink_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK, (const char *)&packet, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_MIN_LEN, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_LEN, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_CRC);
#endif
}

/**
 * @brief Send a gs_param_inmavlink message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gs_param_inmavlink_send_struct(mavlink_channel_t chan, const mavlink_gs_param_inmavlink_t* gs_param_inmavlink)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gs_param_inmavlink_send(chan, gs_param_inmavlink->latitude, gs_param_inmavlink->longitude, gs_param_inmavlink->altitude);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK, (const char *)gs_param_inmavlink, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_MIN_LEN, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_LEN, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_CRC);
#endif
}

#if MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gs_param_inmavlink_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float latitude, float longitude, float altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, altitude);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK, buf, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_MIN_LEN, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_LEN, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_CRC);
#else
    mavlink_gs_param_inmavlink_t *packet = (mavlink_gs_param_inmavlink_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->altitude = altitude;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK, (const char *)packet, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_MIN_LEN, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_LEN, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_CRC);
#endif
}
#endif

#endif

// MESSAGE GS_PARAM_INMAVLINK UNPACKING


/**
 * @brief Get field latitude from gs_param_inmavlink message
 *
 * @return  latitude value
 */
static inline float mavlink_msg_gs_param_inmavlink_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field longitude from gs_param_inmavlink message
 *
 * @return  longitude value
 */
static inline float mavlink_msg_gs_param_inmavlink_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field altitude from gs_param_inmavlink message
 *
 * @return  altitude value
 */
static inline float mavlink_msg_gs_param_inmavlink_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a gs_param_inmavlink message into a struct
 *
 * @param msg The message to decode
 * @param gs_param_inmavlink C-struct to decode the message contents into
 */
static inline void mavlink_msg_gs_param_inmavlink_decode(const mavlink_message_t* msg, mavlink_gs_param_inmavlink_t* gs_param_inmavlink)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gs_param_inmavlink->latitude = mavlink_msg_gs_param_inmavlink_get_latitude(msg);
    gs_param_inmavlink->longitude = mavlink_msg_gs_param_inmavlink_get_longitude(msg);
    gs_param_inmavlink->altitude = mavlink_msg_gs_param_inmavlink_get_altitude(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_LEN? msg->len : MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_LEN;
        memset(gs_param_inmavlink, 0, MAVLINK_MSG_ID_GS_PARAM_INMAVLINK_LEN);
    memcpy(gs_param_inmavlink, _MAV_PAYLOAD(msg), len);
#endif
}

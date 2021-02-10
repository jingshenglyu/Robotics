// Generated by gencpp from file rbo_create/SensorPacket.msg
// DO NOT EDIT!


#ifndef RBO_CREATE_MESSAGE_SENSORPACKET_H
#define RBO_CREATE_MESSAGE_SENSORPACKET_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace rbo_create
{
template <class ContainerAllocator>
struct SensorPacket_
{
  typedef SensorPacket_<ContainerAllocator> Type;

  SensorPacket_()
    : header()
    , wheeldropCaster(false)
    , wheeldropLeft(false)
    , wheeldropRight(false)
    , bumpLeft(false)
    , bumpRight(false)
    , wall(false)
    , cliffLeft(false)
    , cliffFronLeft(false)
    , cliffFrontRight(false)
    , cliffRight(false)
    , virtualWall(false)
    , infraredByte(0)
    , advance(false)
    , play(false)
    , distance(0)
    , angle(0)
    , chargingState(0)
    , voltage(0)
    , current(0)
    , batteryTemperature(0)
    , batteryCharge(0)
    , batteryCapacity(0)
    , wallSignal(0)
    , cliffLeftSignal(0)
    , cliffFrontLeftSignal(0)
    , cliffFrontRightSignal(0)
    , cliffRightSignal(0)
    , homeBase(false)
    , internalCharger(false)
    , songNumber(0)
    , songPlaying(0)  {
    }
  SensorPacket_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , wheeldropCaster(false)
    , wheeldropLeft(false)
    , wheeldropRight(false)
    , bumpLeft(false)
    , bumpRight(false)
    , wall(false)
    , cliffLeft(false)
    , cliffFronLeft(false)
    , cliffFrontRight(false)
    , cliffRight(false)
    , virtualWall(false)
    , infraredByte(0)
    , advance(false)
    , play(false)
    , distance(0)
    , angle(0)
    , chargingState(0)
    , voltage(0)
    , current(0)
    , batteryTemperature(0)
    , batteryCharge(0)
    , batteryCapacity(0)
    , wallSignal(0)
    , cliffLeftSignal(0)
    , cliffFrontLeftSignal(0)
    , cliffFrontRightSignal(0)
    , cliffRightSignal(0)
    , homeBase(false)
    , internalCharger(false)
    , songNumber(0)
    , songPlaying(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _wheeldropCaster_type;
  _wheeldropCaster_type wheeldropCaster;

   typedef uint8_t _wheeldropLeft_type;
  _wheeldropLeft_type wheeldropLeft;

   typedef uint8_t _wheeldropRight_type;
  _wheeldropRight_type wheeldropRight;

   typedef uint8_t _bumpLeft_type;
  _bumpLeft_type bumpLeft;

   typedef uint8_t _bumpRight_type;
  _bumpRight_type bumpRight;

   typedef uint8_t _wall_type;
  _wall_type wall;

   typedef uint8_t _cliffLeft_type;
  _cliffLeft_type cliffLeft;

   typedef uint8_t _cliffFronLeft_type;
  _cliffFronLeft_type cliffFronLeft;

   typedef uint8_t _cliffFrontRight_type;
  _cliffFrontRight_type cliffFrontRight;

   typedef uint8_t _cliffRight_type;
  _cliffRight_type cliffRight;

   typedef uint8_t _virtualWall_type;
  _virtualWall_type virtualWall;

   typedef uint8_t _infraredByte_type;
  _infraredByte_type infraredByte;

   typedef uint8_t _advance_type;
  _advance_type advance;

   typedef uint8_t _play_type;
  _play_type play;

   typedef int16_t _distance_type;
  _distance_type distance;

   typedef int16_t _angle_type;
  _angle_type angle;

   typedef uint8_t _chargingState_type;
  _chargingState_type chargingState;

   typedef uint16_t _voltage_type;
  _voltage_type voltage;

   typedef int16_t _current_type;
  _current_type current;

   typedef int8_t _batteryTemperature_type;
  _batteryTemperature_type batteryTemperature;

   typedef uint16_t _batteryCharge_type;
  _batteryCharge_type batteryCharge;

   typedef uint16_t _batteryCapacity_type;
  _batteryCapacity_type batteryCapacity;

   typedef uint16_t _wallSignal_type;
  _wallSignal_type wallSignal;

   typedef uint16_t _cliffLeftSignal_type;
  _cliffLeftSignal_type cliffLeftSignal;

   typedef uint16_t _cliffFrontLeftSignal_type;
  _cliffFrontLeftSignal_type cliffFrontLeftSignal;

   typedef uint16_t _cliffFrontRightSignal_type;
  _cliffFrontRightSignal_type cliffFrontRightSignal;

   typedef uint16_t _cliffRightSignal_type;
  _cliffRightSignal_type cliffRightSignal;

   typedef uint8_t _homeBase_type;
  _homeBase_type homeBase;

   typedef uint8_t _internalCharger_type;
  _internalCharger_type internalCharger;

   typedef uint8_t _songNumber_type;
  _songNumber_type songNumber;

   typedef uint8_t _songPlaying_type;
  _songPlaying_type songPlaying;





  typedef boost::shared_ptr< ::rbo_create::SensorPacket_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rbo_create::SensorPacket_<ContainerAllocator> const> ConstPtr;

}; // struct SensorPacket_

typedef ::rbo_create::SensorPacket_<std::allocator<void> > SensorPacket;

typedef boost::shared_ptr< ::rbo_create::SensorPacket > SensorPacketPtr;
typedef boost::shared_ptr< ::rbo_create::SensorPacket const> SensorPacketConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rbo_create::SensorPacket_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rbo_create::SensorPacket_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::rbo_create::SensorPacket_<ContainerAllocator1> & lhs, const ::rbo_create::SensorPacket_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.wheeldropCaster == rhs.wheeldropCaster &&
    lhs.wheeldropLeft == rhs.wheeldropLeft &&
    lhs.wheeldropRight == rhs.wheeldropRight &&
    lhs.bumpLeft == rhs.bumpLeft &&
    lhs.bumpRight == rhs.bumpRight &&
    lhs.wall == rhs.wall &&
    lhs.cliffLeft == rhs.cliffLeft &&
    lhs.cliffFronLeft == rhs.cliffFronLeft &&
    lhs.cliffFrontRight == rhs.cliffFrontRight &&
    lhs.cliffRight == rhs.cliffRight &&
    lhs.virtualWall == rhs.virtualWall &&
    lhs.infraredByte == rhs.infraredByte &&
    lhs.advance == rhs.advance &&
    lhs.play == rhs.play &&
    lhs.distance == rhs.distance &&
    lhs.angle == rhs.angle &&
    lhs.chargingState == rhs.chargingState &&
    lhs.voltage == rhs.voltage &&
    lhs.current == rhs.current &&
    lhs.batteryTemperature == rhs.batteryTemperature &&
    lhs.batteryCharge == rhs.batteryCharge &&
    lhs.batteryCapacity == rhs.batteryCapacity &&
    lhs.wallSignal == rhs.wallSignal &&
    lhs.cliffLeftSignal == rhs.cliffLeftSignal &&
    lhs.cliffFrontLeftSignal == rhs.cliffFrontLeftSignal &&
    lhs.cliffFrontRightSignal == rhs.cliffFrontRightSignal &&
    lhs.cliffRightSignal == rhs.cliffRightSignal &&
    lhs.homeBase == rhs.homeBase &&
    lhs.internalCharger == rhs.internalCharger &&
    lhs.songNumber == rhs.songNumber &&
    lhs.songPlaying == rhs.songPlaying;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::rbo_create::SensorPacket_<ContainerAllocator1> & lhs, const ::rbo_create::SensorPacket_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace rbo_create

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::rbo_create::SensorPacket_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rbo_create::SensorPacket_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rbo_create::SensorPacket_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rbo_create::SensorPacket_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rbo_create::SensorPacket_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rbo_create::SensorPacket_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rbo_create::SensorPacket_<ContainerAllocator> >
{
  static const char* value()
  {
    return "56f92e8d70d283e7e15aa47855e73ea7";
  }

  static const char* value(const ::rbo_create::SensorPacket_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x56f92e8d70d283e7ULL;
  static const uint64_t static_value2 = 0xe15aa47855e73ea7ULL;
};

template<class ContainerAllocator>
struct DataType< ::rbo_create::SensorPacket_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rbo_create/SensorPacket";
  }

  static const char* value(const ::rbo_create::SensorPacket_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rbo_create::SensorPacket_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"bool wheeldropCaster\n"
"bool wheeldropLeft\n"
"bool wheeldropRight\n"
"bool bumpLeft\n"
"bool bumpRight\n"
"bool wall\n"
"bool cliffLeft\n"
"bool cliffFronLeft\n"
"bool cliffFrontRight\n"
"bool cliffRight\n"
"bool virtualWall\n"
"uint8 infraredByte\n"
"bool advance\n"
"bool play\n"
"int16 distance\n"
"int16 angle\n"
"uint8 chargingState\n"
"uint16 voltage\n"
"int16 current\n"
"int8 batteryTemperature\n"
"uint16 batteryCharge\n"
"uint16 batteryCapacity\n"
"uint16 wallSignal\n"
"uint16 cliffLeftSignal\n"
"uint16 cliffFrontLeftSignal\n"
"uint16 cliffFrontRightSignal\n"
"uint16 cliffRightSignal\n"
"bool homeBase\n"
"bool internalCharger\n"
"uint8 songNumber\n"
"uint8 songPlaying\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::rbo_create::SensorPacket_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rbo_create::SensorPacket_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.wheeldropCaster);
      stream.next(m.wheeldropLeft);
      stream.next(m.wheeldropRight);
      stream.next(m.bumpLeft);
      stream.next(m.bumpRight);
      stream.next(m.wall);
      stream.next(m.cliffLeft);
      stream.next(m.cliffFronLeft);
      stream.next(m.cliffFrontRight);
      stream.next(m.cliffRight);
      stream.next(m.virtualWall);
      stream.next(m.infraredByte);
      stream.next(m.advance);
      stream.next(m.play);
      stream.next(m.distance);
      stream.next(m.angle);
      stream.next(m.chargingState);
      stream.next(m.voltage);
      stream.next(m.current);
      stream.next(m.batteryTemperature);
      stream.next(m.batteryCharge);
      stream.next(m.batteryCapacity);
      stream.next(m.wallSignal);
      stream.next(m.cliffLeftSignal);
      stream.next(m.cliffFrontLeftSignal);
      stream.next(m.cliffFrontRightSignal);
      stream.next(m.cliffRightSignal);
      stream.next(m.homeBase);
      stream.next(m.internalCharger);
      stream.next(m.songNumber);
      stream.next(m.songPlaying);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SensorPacket_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rbo_create::SensorPacket_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rbo_create::SensorPacket_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "wheeldropCaster: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.wheeldropCaster);
    s << indent << "wheeldropLeft: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.wheeldropLeft);
    s << indent << "wheeldropRight: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.wheeldropRight);
    s << indent << "bumpLeft: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.bumpLeft);
    s << indent << "bumpRight: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.bumpRight);
    s << indent << "wall: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.wall);
    s << indent << "cliffLeft: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cliffLeft);
    s << indent << "cliffFronLeft: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cliffFronLeft);
    s << indent << "cliffFrontRight: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cliffFrontRight);
    s << indent << "cliffRight: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cliffRight);
    s << indent << "virtualWall: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.virtualWall);
    s << indent << "infraredByte: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.infraredByte);
    s << indent << "advance: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.advance);
    s << indent << "play: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.play);
    s << indent << "distance: ";
    Printer<int16_t>::stream(s, indent + "  ", v.distance);
    s << indent << "angle: ";
    Printer<int16_t>::stream(s, indent + "  ", v.angle);
    s << indent << "chargingState: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.chargingState);
    s << indent << "voltage: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.voltage);
    s << indent << "current: ";
    Printer<int16_t>::stream(s, indent + "  ", v.current);
    s << indent << "batteryTemperature: ";
    Printer<int8_t>::stream(s, indent + "  ", v.batteryTemperature);
    s << indent << "batteryCharge: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.batteryCharge);
    s << indent << "batteryCapacity: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.batteryCapacity);
    s << indent << "wallSignal: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.wallSignal);
    s << indent << "cliffLeftSignal: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.cliffLeftSignal);
    s << indent << "cliffFrontLeftSignal: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.cliffFrontLeftSignal);
    s << indent << "cliffFrontRightSignal: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.cliffFrontRightSignal);
    s << indent << "cliffRightSignal: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.cliffRightSignal);
    s << indent << "homeBase: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.homeBase);
    s << indent << "internalCharger: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.internalCharger);
    s << indent << "songNumber: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.songNumber);
    s << indent << "songPlaying: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.songPlaying);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RBO_CREATE_MESSAGE_SENSORPACKET_H
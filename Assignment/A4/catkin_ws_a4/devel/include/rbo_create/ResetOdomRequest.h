// Generated by gencpp from file rbo_create/ResetOdomRequest.msg
// DO NOT EDIT!


#ifndef RBO_CREATE_MESSAGE_RESETODOMREQUEST_H
#define RBO_CREATE_MESSAGE_RESETODOMREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rbo_create
{
template <class ContainerAllocator>
struct ResetOdomRequest_
{
  typedef ResetOdomRequest_<ContainerAllocator> Type;

  ResetOdomRequest_()
    : reset_odom(false)  {
    }
  ResetOdomRequest_(const ContainerAllocator& _alloc)
    : reset_odom(false)  {
  (void)_alloc;
    }



   typedef uint8_t _reset_odom_type;
  _reset_odom_type reset_odom;





  typedef boost::shared_ptr< ::rbo_create::ResetOdomRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rbo_create::ResetOdomRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ResetOdomRequest_

typedef ::rbo_create::ResetOdomRequest_<std::allocator<void> > ResetOdomRequest;

typedef boost::shared_ptr< ::rbo_create::ResetOdomRequest > ResetOdomRequestPtr;
typedef boost::shared_ptr< ::rbo_create::ResetOdomRequest const> ResetOdomRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rbo_create::ResetOdomRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rbo_create::ResetOdomRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::rbo_create::ResetOdomRequest_<ContainerAllocator1> & lhs, const ::rbo_create::ResetOdomRequest_<ContainerAllocator2> & rhs)
{
  return lhs.reset_odom == rhs.reset_odom;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::rbo_create::ResetOdomRequest_<ContainerAllocator1> & lhs, const ::rbo_create::ResetOdomRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace rbo_create

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::rbo_create::ResetOdomRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rbo_create::ResetOdomRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rbo_create::ResetOdomRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rbo_create::ResetOdomRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rbo_create::ResetOdomRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rbo_create::ResetOdomRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rbo_create::ResetOdomRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "75845a070de97808c5df63dd2cc872a3";
  }

  static const char* value(const ::rbo_create::ResetOdomRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x75845a070de97808ULL;
  static const uint64_t static_value2 = 0xc5df63dd2cc872a3ULL;
};

template<class ContainerAllocator>
struct DataType< ::rbo_create::ResetOdomRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rbo_create/ResetOdomRequest";
  }

  static const char* value(const ::rbo_create::ResetOdomRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rbo_create::ResetOdomRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool reset_odom\n"
;
  }

  static const char* value(const ::rbo_create::ResetOdomRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rbo_create::ResetOdomRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.reset_odom);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ResetOdomRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rbo_create::ResetOdomRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rbo_create::ResetOdomRequest_<ContainerAllocator>& v)
  {
    s << indent << "reset_odom: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reset_odom);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RBO_CREATE_MESSAGE_RESETODOMREQUEST_H
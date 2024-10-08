// Generated by gencpp from file swarm_controller/location_msg.msg
// DO NOT EDIT!


#ifndef SWARM_CONTROLLER_MESSAGE_LOCATION_MSG_H
#define SWARM_CONTROLLER_MESSAGE_LOCATION_MSG_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace swarm_controller
{
template <class ContainerAllocator>
struct location_msg_
{
  typedef location_msg_<ContainerAllocator> Type;

  location_msg_()
    : x(0.0)
    , y(0.0)  {
    }
  location_msg_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;





  typedef boost::shared_ptr< ::swarm_controller::location_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::swarm_controller::location_msg_<ContainerAllocator> const> ConstPtr;

}; // struct location_msg_

typedef ::swarm_controller::location_msg_<std::allocator<void> > location_msg;

typedef boost::shared_ptr< ::swarm_controller::location_msg > location_msgPtr;
typedef boost::shared_ptr< ::swarm_controller::location_msg const> location_msgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::swarm_controller::location_msg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::swarm_controller::location_msg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::swarm_controller::location_msg_<ContainerAllocator1> & lhs, const ::swarm_controller::location_msg_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::swarm_controller::location_msg_<ContainerAllocator1> & lhs, const ::swarm_controller::location_msg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace swarm_controller

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::swarm_controller::location_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::swarm_controller::location_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::swarm_controller::location_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::swarm_controller::location_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::swarm_controller::location_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::swarm_controller::location_msg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::swarm_controller::location_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff8d7d66dd3e4b731ef14a45d38888b6";
  }

  static const char* value(const ::swarm_controller::location_msg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xff8d7d66dd3e4b73ULL;
  static const uint64_t static_value2 = 0x1ef14a45d38888b6ULL;
};

template<class ContainerAllocator>
struct DataType< ::swarm_controller::location_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "swarm_controller/location_msg";
  }

  static const char* value(const ::swarm_controller::location_msg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::swarm_controller::location_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x\n"
"float32 y\n"
;
  }

  static const char* value(const ::swarm_controller::location_msg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::swarm_controller::location_msg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct location_msg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::swarm_controller::location_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::swarm_controller::location_msg_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SWARM_CONTROLLER_MESSAGE_LOCATION_MSG_H

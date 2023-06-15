// Generated by gencpp from file qingzhou_cloud/roadLine.msg
// DO NOT EDIT!


#ifndef QINGZHOU_CLOUD_MESSAGE_ROADLINE_H
#define QINGZHOU_CLOUD_MESSAGE_ROADLINE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace qingzhou_cloud
{
template <class ContainerAllocator>
struct roadLine_
{
  typedef roadLine_<ContainerAllocator> Type;

  roadLine_()
    : lineX()
    , lineY()
    , lineStatus(0)  {
    }
  roadLine_(const ContainerAllocator& _alloc)
    : lineX(_alloc)
    , lineY(_alloc)
    , lineStatus(0)  {
  (void)_alloc;
    }



   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _lineX_type;
  _lineX_type lineX;

   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _lineY_type;
  _lineY_type lineY;

   typedef uint8_t _lineStatus_type;
  _lineStatus_type lineStatus;





  typedef boost::shared_ptr< ::qingzhou_cloud::roadLine_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::qingzhou_cloud::roadLine_<ContainerAllocator> const> ConstPtr;

}; // struct roadLine_

typedef ::qingzhou_cloud::roadLine_<std::allocator<void> > roadLine;

typedef boost::shared_ptr< ::qingzhou_cloud::roadLine > roadLinePtr;
typedef boost::shared_ptr< ::qingzhou_cloud::roadLine const> roadLineConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::qingzhou_cloud::roadLine_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::qingzhou_cloud::roadLine_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::qingzhou_cloud::roadLine_<ContainerAllocator1> & lhs, const ::qingzhou_cloud::roadLine_<ContainerAllocator2> & rhs)
{
  return lhs.lineX == rhs.lineX &&
    lhs.lineY == rhs.lineY &&
    lhs.lineStatus == rhs.lineStatus;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::qingzhou_cloud::roadLine_<ContainerAllocator1> & lhs, const ::qingzhou_cloud::roadLine_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace qingzhou_cloud

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::qingzhou_cloud::roadLine_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::qingzhou_cloud::roadLine_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::qingzhou_cloud::roadLine_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::qingzhou_cloud::roadLine_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qingzhou_cloud::roadLine_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qingzhou_cloud::roadLine_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::qingzhou_cloud::roadLine_<ContainerAllocator> >
{
  static const char* value()
  {
    return "979b1a9d0eae9e69bcac1c5b8eb2ff08";
  }

  static const char* value(const ::qingzhou_cloud::roadLine_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x979b1a9d0eae9e69ULL;
  static const uint64_t static_value2 = 0xbcac1c5b8eb2ff08ULL;
};

template<class ContainerAllocator>
struct DataType< ::qingzhou_cloud::roadLine_<ContainerAllocator> >
{
  static const char* value()
  {
    return "qingzhou_cloud/roadLine";
  }

  static const char* value(const ::qingzhou_cloud::roadLine_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::qingzhou_cloud::roadLine_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32[] lineX\n"
"float32[] lineY\n"
"uint8 lineStatus\n"
;
  }

  static const char* value(const ::qingzhou_cloud::roadLine_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::qingzhou_cloud::roadLine_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.lineX);
      stream.next(m.lineY);
      stream.next(m.lineStatus);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct roadLine_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::qingzhou_cloud::roadLine_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::qingzhou_cloud::roadLine_<ContainerAllocator>& v)
  {
    s << indent << "lineX[]" << std::endl;
    for (size_t i = 0; i < v.lineX.size(); ++i)
    {
      s << indent << "  lineX[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.lineX[i]);
    }
    s << indent << "lineY[]" << std::endl;
    for (size_t i = 0; i < v.lineY.size(); ++i)
    {
      s << indent << "  lineY[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.lineY[i]);
    }
    s << indent << "lineStatus: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.lineStatus);
  }
};

} // namespace message_operations
} // namespace ros

#endif // QINGZHOU_CLOUD_MESSAGE_ROADLINE_H

#ifndef BEHAVIORTREE_ROS2__BT_SERVICE_ERROR_CODE_HPP_
#define BEHAVIORTREE_ROS2__BT_SERVICE_ERROR_CODE_HPP_

namespace BT
{
enum ServiceNodeErrorCode
{
  SERVICE_UNREACHABLE,
  SERVICE_TIMEOUT,
  INVALID_REQUEST,
  SERVICE_ABORTED
};

inline const char* toStr(const ServiceNodeErrorCode& err)
{
  switch (err)
  {
    case SERVICE_UNREACHABLE: return "SERVICE_UNREACHABLE";
    case SERVICE_TIMEOUT: return "SERVICE_TIMEOUT";
    case INVALID_REQUEST: return "INVALID_REQUEST";
    case SERVICE_ABORTED: return "SERVICE_ABORTED";
  }
  return nullptr;
}
} // namespace BT

#endif  // BEHAVIORTREE_ROS2__BT_SERVICE_ERROR_CODE_HPP_

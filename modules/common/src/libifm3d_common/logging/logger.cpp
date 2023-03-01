#include <ifm3d/common/logging/logger.h>

ifm3d::Logger&
ifm3d::Logger::Get()
{
  static Logger instance;
  return instance;
}
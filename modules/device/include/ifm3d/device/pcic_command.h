#ifndef IFM3D_DEVICE_PCIC_COMMAND_H
#define IFM3D_DEVICE_PCIC_COMMAND_H

#include <cstdint>
#include <string>
#include <vector>

namespace ifm3d
{
  class PCICCommand
  {
  public:
    PCICCommand() = default;
    virtual ~PCICCommand() = default;

    /**
     * @brief Serialize the command into a sequence of bytes for PCIC
     * transmission.
     *
     * @return A vector of uint8_t representing the serialized command data.
     */
    virtual std::vector<std::uint8_t> SerializeData() const = 0;
  };
} // namespace ifm3d

#endif // IFM3D_DEVICE_PCIC_COMMAND_H

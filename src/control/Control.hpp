#ifndef CONTROL_CONTROL_HPP_
#define CONTROL_CONTROL_HPP_

#include <aidrive/Types.hpp>

namespace aidrive
{
namespace control
{

///
class Controller
{
public:
    Controller();

    std::vector<Vector3f> optimize(const std::vector<Vector3f>& reference);

private:
    static constexpr size_t OPT_STEPS = 50;
    float64_t m_cmd[OPT_STEPS]{};

}; // class Controller

} // namespace control
} // namespace aidrive

#endif // CONTROL_CONTROL_HPP_
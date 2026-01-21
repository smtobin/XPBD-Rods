#pragma once

#include "config/Config.hpp"

#include <string>

namespace Config
{

class JointConfig : public Config_Base
{
public:
    explicit JointConfig()
        : Config_Base()
    {}

    explicit JointConfig(const YAML::Node& node)
        : Config_Base(node)
    {
        _extractParameter("body1", node, _body1);
        _extractParameter("body1-pos-offset", node, _body1_pos_offset);
        _extractParameter("body1-rot-offset", node, _body1_rot_offset);

        _extractParameter("body2", node, _body2);
        _extractParameter("body2-pos-offset", node, _body2_pos_offset);
        _extractParameter("body2-rot-offset", node, _body2_rot_offset);
    }

    std::string body1() const { return _body1.value; }
    Vec3r body1PositionalOffset() const { return _body1_pos_offset.value; }
    Vec3r body1RotationalOffset() const { return _body1_rot_offset.value; }

    std::optional<std::string> body2() const { return _body2.value; }
    Vec3r body2PositionalOffset() const { return _body2_pos_offset.value; }
    Vec3r body2RotationalOffset() const { return _body2_rot_offset.value; }

protected:
    ConfigParameter<std::string> _body1 = ConfigParameter<std::string>("");
    ConfigParameter<Vec3r> _body1_pos_offset = ConfigParameter<Vec3r>(Vec3r::Zero());
    ConfigParameter<Vec3r> _body1_rot_offset = ConfigParameter<Vec3r>(Vec3r::Zero());

    ConfigParameter<std::optional<std::string>> _body2;
    ConfigParameter<Vec3r> _body2_pos_offset = ConfigParameter<Vec3r>(Vec3r::Zero());
    ConfigParameter<Vec3r> _body2_rot_offset = ConfigParameter<Vec3r>(Vec3r::Zero());
};

} // namespace Config
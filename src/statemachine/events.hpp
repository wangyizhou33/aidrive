#ifndef EVENTS_HPP_
#define EVENTS_HPP_

#include "tinyfsm.hpp"


struct UserEvent : tinyfsm::Event {};
struct ModuleEvent : tinyfsm::Event {};
struct SystemEvent : tinyfsm::Event {};


struct Disengage : UserEvent {};
struct EngageACC : UserEvent {};
struct EngageICA : UserEvent {};
struct EngageNDA : UserEvent {};
struct OverrideThrottle : UserEvent {};
struct BrakeThrottle : UserEvent {};

struct AEBFiring : ModuleEvent {};
struct WaitTimeout : ModuleEvent {};
struct LonReady : ModuleEvent {};
struct LonNotReady : ModuleEvent {};
struct LatReady : ModuleEvent {};
struct LatNotReady : ModuleEvent {};


#endif // EVENTS_HPP_
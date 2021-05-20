#include "tinyfsm.hpp"
#include <iostream>

struct Top;
struct Lon;
struct Lat;
struct BP;
struct Mission;

// Set up state machines
using fsm_list = tinyfsm::FsmList<Top, Lon, Lat, BP, Mission>;

template <typename E>
void send_event(E const& event)
{
    fsm_list::template dispatch<E>(event);
}

// state enum
enum class TopEnum
{
    INVALID,
    MANUAL,
    L1,
    L2,
    L3,
    L4
};

enum class LonEnum
{
    INVALID,
    FAULT,
    READY,
    CONTROLLING,
    OVERRIDEN,
    WAITING,
    HOLDING
};

enum class LatEnum
{
    INVALID,
    FAULT,
    READY,
    CONTROLLING,
    OVERRIDEN
};

enum class BPEnum
{
    INVALID,
    FAULT,
    ACC,
    ACCLK,
    L3inTransit,
    L3
};

enum class MissionEnum
{
    INVALID,
    OFF,
    ESTABLISHED,
    ACTIVATED,
    ACTIVATED_NEW_ESTABLISHED
};

// UI event
struct EngageL1 : tinyfsm::Event
{
};
struct EngageL2 : tinyfsm::Event
{
};
struct EngageL3 : tinyfsm::Event
{
};
struct EngageL4 : tinyfsm::Event
{
};
struct Disengage : tinyfsm::Event
{
};

// module event
struct ReportLonReady : tinyfsm::Event
{
};
struct ReportLatReady : tinyfsm::Event
{
};
struct ReportL3Ready : tinyfsm::Event
{
};
struct ReportL3Fail : tinyfsm::Event
{
};
struct L1OnEvent : tinyfsm::Event
{
};
struct L2OnEvent : tinyfsm::Event
{
};
struct L3Requested : tinyfsm::Event // Top sends this event
{
};
struct BPL3onEvent : tinyfsm::Event // BP sends this event
{
};
struct ThrottleOverride : tinyfsm::Event
{
};
struct BrakeOverride : tinyfsm::Event
{
};
struct StopThrottleOverride : tinyfsm::Event
{
};
struct StopSteerOverride : tinyfsm::Event
{
};
struct SteerOverride : tinyfsm::Event
{
};
struct WaitingOnEvent : tinyfsm::Event
{
};
struct WaitingOffEvent : tinyfsm::Event
{
};
struct HoldingOnEvent : tinyfsm::Event
{
};
struct Resume : tinyfsm::Event
{
};
struct EstablishMission : tinyfsm::Event
{
};
struct ActivateMission : tinyfsm::Event
{
};
struct DeactivateMission : tinyfsm::Event
{
};

struct Top : public tinyfsm::Fsm<Top>
{
    virtual void react(EngageL1 const&) { std::cout << "EngageL1 event ignored" << std::endl; };
    virtual void react(EngageL2 const&) { std::cout << "EngageL2 event ignored" << std::endl; };
    virtual void react(EngageL3 const&) { std::cout << "EngageL3 event ignored" << std::endl; };
    virtual void react(EngageL4 const&) { std::cout << "L4 disabled in configuration in this release" << std::endl; };
    virtual void react(Disengage const&) { std::cout << "Disengage event ignored" << std::endl; };
    virtual void react(BPL3onEvent const&){};
    void react(tinyfsm::Event const&){};

    virtual void entry(void){};
    void exit(void){};

    static TopEnum getCurrentTopState() { return m_current; };

protected:
    static TopEnum m_current;
};

TopEnum Top::m_current = TopEnum::INVALID;

struct Manual : public Top
{
    void entry() override
    {
        send_event(Disengage());
        m_current = TopEnum::MANUAL;
    };
    virtual void react(EngageL1 const&);
    virtual void react(EngageL2 const&);
    virtual void react(EngageL3 const&);
};

struct L1 : public Top
{
    void entry() override
    {
        send_event(L1OnEvent());
        m_current = TopEnum::L1;
    };

    virtual void react(EngageL2 const&);
    virtual void react(EngageL3 const&);
    virtual void react(Disengage const&);
};

struct L2 : public Top
{
    void entry() override
    {
        send_event(L2OnEvent());
        m_current = TopEnum::L2;
    };
    virtual void react(EngageL1 const&);
    virtual void react(EngageL3 const&);
    virtual void react(Disengage const&);
    virtual void react(BPL3onEvent const&);
};

struct L3 : public Top
{
    void entry() override
    {
        m_current = TopEnum::L3;
    };
    virtual void react(EngageL1 const&);
    virtual void react(EngageL2 const&);
    virtual void react(Disengage const&);
};

struct L4 : public Top
{
    void entry() override
    {
        m_current = TopEnum::L4;
    };
};

struct Lon : public tinyfsm::Fsm<Lon>
{
    virtual void entry(void){};
    void exit(void){};

    virtual void react(tinyfsm::Event const&){};
    virtual void react(ReportLonReady const&) { std::cout << "no op on ReportLonReady event" << std::endl; };
    virtual void react(L2OnEvent const&){};
    virtual void react(L1OnEvent const&){};
    virtual void react(Disengage const&){};
    virtual void react(ThrottleOverride const&){};
    virtual void react(BrakeOverride const&);
    virtual void react(StopThrottleOverride const&){};
    virtual void react(WaitingOnEvent const&){};
    virtual void react(WaitingOffEvent const&){};
    virtual void react(HoldingOnEvent const&){};
    virtual void react(Resume const&){};

    static LonEnum getCurrentLonState() { return m_current; };

protected:
    static LonEnum m_current;
};
LonEnum Lon::m_current = LonEnum::INVALID;

struct LonFault : public Lon
{
    void entry() override
    {
        m_current = LonEnum::FAULT;
    };
    void react(ReportLonReady const&) override;
};

struct LonReady : public Lon
{
    void entry() override
    {
        m_current = LonEnum::READY;
    };
    void react(L2OnEvent const&) override;
    void react(L1OnEvent const&) override;
};

struct LonControlling : public Lon
{
    void entry() override
    {
        m_current = LonEnum::CONTROLLING;
    };
    void react(Disengage const&) override;
    void react(ThrottleOverride const&) override;
    void react(WaitingOnEvent const&) override;
};

struct LonOverriden : public Lon
{
    void entry() override
    {
        m_current = LonEnum::OVERRIDEN;
    };
    void react(Disengage const&) override;
    void react(StopThrottleOverride const&);
};

struct LonWaiting : public Lon
{
    void entry() override
    {
        m_current = LonEnum::WAITING;
    };
    void react(HoldingOnEvent const&) override;
    void react(WaitingOffEvent const&) override;
};

struct LonHolding : public Lon
{
    void entry() override
    {
        m_current = LonEnum::HOLDING;
    };
    void react(Resume const&) override;
    void react(ThrottleOverride const&) override;
};

struct Lat : public tinyfsm::Fsm<Lat>
{
    virtual void entry(void){};
    void exit(void){};

    virtual void react(tinyfsm::Event const&){};
    virtual void react(ReportLatReady const&) { std::cout << "no op on ReportLatReady event" << std::endl; };
    virtual void react(L2OnEvent const&){};
    virtual void react(L1OnEvent const&){};
    virtual void react(Disengage const&){};
    virtual void react(SteerOverride const&){};
    virtual void react(StopSteerOverride const&){};

    static LatEnum getCurrentLatState() { return m_current; };

protected:
    static LatEnum m_current;
};
LatEnum Lat::m_current = LatEnum::INVALID;

struct LatFault : public Lat
{
    void entry() override
    {
        m_current = LatEnum::FAULT;
    };
    void react(ReportLatReady const&) override;
};

struct LatReady : public Lat
{
    void entry() override
    {
        m_current = LatEnum::READY;
    };
    void react(L2OnEvent const&) override;
};

struct LatControlling : public Lat
{
    void entry() override
    {
        m_current = LatEnum::CONTROLLING;
    };
    void react(Disengage const&) override;
    void react(L1OnEvent const&) override;
    void react(SteerOverride const&) override;
};

struct LatOverriden : public Lat
{
    void entry() override
    {
        m_current = LatEnum::OVERRIDEN;
    };
    void react(Disengage const&) override;
    void react(L1OnEvent const&) override;
    void react(StopSteerOverride const&) override;
};

struct BP : public tinyfsm::Fsm<BP>
{
    virtual void entry(void){};
    void exit(void){};

    virtual void react(tinyfsm::Event const&){};
    virtual void react(L3Requested const&){};
    virtual void react(L2OnEvent const&){};
    virtual void react(L1OnEvent const&){};
    virtual void react(Disengage const&){};
    virtual void react(ReportL3Ready const&){};
    virtual void react(ReportL3Fail const&){};

    static BPEnum getCurrentState() { return m_current; };

protected:
    static BPEnum m_current;
};
BPEnum BP::m_current = BPEnum::INVALID;

struct BPFault : public BP
{
    void entry() override
    {
        m_current = BPEnum::FAULT;
    };
    void react(L2OnEvent const&) override;
    void react(L1OnEvent const&) override;
};

struct BPACC : public BP
{
    void entry() override
    {
        m_current = BPEnum::ACC;
    };
    void react(L2OnEvent const&) override;
    void react(Disengage const&) override;
};

struct BPACCLK : public BP
{
    void entry() override
    {
        m_current = BPEnum::ACCLK;
    };
    void react(L3Requested const&) override;
    void react(L1OnEvent const&) override;
    void react(Disengage const&) override;
};

struct BPL3inTransit : public BP
{
    void entry() override
    {
        m_current = BPEnum::L3inTransit;
    };
    void react(ReportL3Ready const&) override;
    void react(ReportL3Fail const&) override;
    void react(L2OnEvent const&) override;
    void react(L1OnEvent const&) override;
    void react(Disengage const&) override;
};

struct BPL3 : public BP
{
    void entry() override
    {
        m_current = BPEnum::L3;
    };
    void react(L2OnEvent const&) override;
    void react(L1OnEvent const&) override;
    void react(Disengage const&) override;
};

struct MissionOff;
struct Mission : public tinyfsm::Fsm<Mission>
{
    virtual void entry(void){};
    void exit(void){};

    virtual void react(tinyfsm::Event const&){};
    virtual void react(EstablishMission const&) { std::cout << "EstablishMission event ignored" << std::endl; };
    virtual void react(ActivateMission const&) { std::cout << "ActivateMission event ignored" << std::endl; };
    virtual void react(DeactivateMission const&) { transit<MissionOff>(); };

    static MissionEnum getCurrentState() { return m_current; };

protected:
    static MissionEnum m_current;
};
MissionEnum Mission::m_current = MissionEnum::INVALID;

struct MissionOff : public Mission
{
    void entry() override
    {
        m_current = MissionEnum::OFF;
    };
    void react(EstablishMission const&) override;
};
struct MissionEst : public Mission
{
    void entry() override
    {
        m_current = MissionEnum::ESTABLISHED;
    };
    void react(ActivateMission const&) override;
};
struct MissionAct : public Mission
{
    void entry() override
    {
        m_current = MissionEnum::ACTIVATED;
    };
    void react(EstablishMission const&) override;
};
struct MissionActNewEst : public Mission
{
    void entry() override
    {
        m_current = MissionEnum::ACTIVATED_NEW_ESTABLISHED;
    };
    void react(ActivateMission const&) override;
};

void Manual::react(EngageL1 const&)
{
    if (Lon::getCurrentLonState() == LonEnum::READY)
    {
        transit<L1>();
    }
    else
    {
        std::cout << "EngageL1 event is rejected" << std::endl;
    }
};

void Manual::react(EngageL2 const&)
{
    if (Lon::getCurrentLonState() == LonEnum::READY &&
        Lat::getCurrentLatState() == LatEnum::READY)
    {
        transit<L2>();
    }
    else
    {
        std::cout << "EngageL2 event is rejected" << std::endl;
    }
};

void Manual::react(EngageL3 const&)
{
    react(EngageL2());
    if (m_current == TopEnum::L2)
    {
        send_event(L3Requested());
    }
}

void L1::react(EngageL2 const&)
{
    if (Lat::getCurrentLatState() == LatEnum::READY)
    {
        transit<L2>();
    }
    else
    {
        std::cout << "EngageL2 event is rejected" << std::endl;
    }
};
void L1::react(EngageL3 const&)
{
    react(EngageL2());
    if (m_current == TopEnum::L2)
    {
        send_event(L3Requested());
    }
}
void L1::react(Disengage const&)
{
    transit<Manual>();
};
void L2::react(EngageL1 const&)
{
    transit<L1>();
};
void L2::react(EngageL3 const&)
{
    send_event(L3Requested());
}
void L2::react(Disengage const&)
{
    transit<Manual>();
};
void L2::react(BPL3onEvent const&)
{
    transit<L3>();
}
void L3::react(EngageL1 const&)
{
    transit<L1>();
};
void L3::react(EngageL2 const&)
{
    transit<L2>();
}
void L3::react(Disengage const&)
{
    transit<Manual>();
};

void Lon::react(BrakeOverride const&)
{
    send_event(Disengage());
    transit<LonFault>();
}

void LonFault::react(ReportLonReady const&)
{
    transit<LonReady>();
}
void LonReady::react(L2OnEvent const&)
{
    react(L1OnEvent());
}
void LonReady::react(L1OnEvent const&)
{
    transit<LonControlling>();
}
void LonControlling::react(Disengage const&)
{
    transit<LonFault>();
}
void LonControlling::react(ThrottleOverride const&)
{
    transit<LonOverriden>();
}
void LonControlling::react(WaitingOnEvent const&)
{
    transit<LonWaiting>();
}
void LonOverriden::react(Disengage const&)
{
    transit<LonFault>();
}
void LonOverriden::react(StopThrottleOverride const&)
{
    transit<LonControlling>();
}
void LonWaiting::react(HoldingOnEvent const&)
{
    transit<LonHolding>();
}
void LonWaiting::react(WaitingOffEvent const&)
{
    transit<LonControlling>();
}
void LonHolding::react(Resume const&)
{
    transit<LonControlling>();
}
void LonHolding::react(ThrottleOverride const&)
{
    transit<LonOverriden>();
}

void LatFault::react(ReportLatReady const&)
{
    transit<LatReady>();
}
void LatReady::react(L2OnEvent const&)
{
    transit<LatControlling>();
}
void LatControlling::react(Disengage const&)
{
    transit<LatFault>();
}
void LatControlling::react(L1OnEvent const&)
{
    react(Disengage());
}
void LatControlling::react(SteerOverride const&)
{
    transit<LatOverriden>();
}
void LatOverriden::react(Disengage const&)
{
    transit<LatFault>();
}
void LatOverriden::react(L1OnEvent const&)
{
    react(Disengage());
}
void LatOverriden::react(StopSteerOverride const&)
{
    transit<LatControlling>();
}

void BPFault::react(L2OnEvent const&)
{
    transit<BPACCLK>();
}
void BPFault::react(L1OnEvent const&)
{
    transit<BPACC>();
}
void BPACC::react(L2OnEvent const&)
{
    transit<BPACCLK>();
}
void BPACC::react(Disengage const&)
{
    transit<BPFault>();
}
void BPACCLK::react(L1OnEvent const&)
{
    transit<BPACC>();
}
void BPACCLK::react(Disengage const&)
{
    transit<BPFault>();
}
void BPACCLK::react(L3Requested const&)
{
    transit<BPL3inTransit>();
}

void BPL3inTransit::react(L1OnEvent const&)
{
    transit<BPACC>();
}
void BPL3inTransit::react(L2OnEvent const&)
{
    transit<BPACCLK>();
}
void BPL3inTransit::react(Disengage const&)
{
    transit<BPFault>();
}
void BPL3inTransit::react(ReportL3Ready const&)
{
    transit<BPL3>();
    send_event(BPL3onEvent());
}
void BPL3inTransit::react(ReportL3Fail const&)
{
    transit<BPACCLK>();
}
void BPL3::react(L1OnEvent const&)
{
    transit<BPACC>();
}
void BPL3::react(L2OnEvent const&)
{
    transit<BPACCLK>();
}
void BPL3::react(Disengage const&)
{
    transit<BPFault>();
}

void MissionOff::react(EstablishMission const&)
{
    transit<MissionEst>();
}
void MissionEst::react(ActivateMission const&)
{
    transit<MissionAct>();
}
void MissionAct::react(EstablishMission const&)
{
    transit<MissionActNewEst>();
}
void MissionActNewEst::react(ActivateMission const&)
{
    transit<MissionAct>();
}

// ----------------------------------------------------------------------------
// Initial state definition
//
FSM_INITIAL_STATE(Top, Manual)
FSM_INITIAL_STATE(Lon, LonFault)
FSM_INITIAL_STATE(Lat, LatFault)
FSM_INITIAL_STATE(BP, BPFault)
FSM_INITIAL_STATE(Mission, MissionOff)
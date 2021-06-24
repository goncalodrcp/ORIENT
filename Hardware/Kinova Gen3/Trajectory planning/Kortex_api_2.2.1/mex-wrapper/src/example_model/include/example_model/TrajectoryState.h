#ifndef TRAJECTORY_STATE_H
#define TRAJECTORY_STATE_H

/**
 * Copyright Kinova Inc. 2019
 */

#include <Base.pb.h>

using namespace Kinova::Api;

class TrajectoryState
{
public:

    enum State
    {
        eIdle,
        eRunning,
        ePaused,
        ePrecomputedTrajectoryProcessing,
    };

    struct StateInformation
    {
        StateInformation()
            : state(State::eIdle)
            , action_handle(0)
            , timestamp(0)
            , reason()
        {}

        explicit StateInformation(State &state)
            : state(state)
            , action_handle(0)
            , timestamp(0)
            , reason()
        {}

        unsigned int action_handle;
        unsigned int timestamp;
        std::string  reason;
        State        state;

        friend std::ostream& operator<<(std::ostream& out, StateInformation const& rhs)
        {
            out << "Handle " << rhs.action_handle << "\nTimestamp: " << rhs.timestamp << "\nReason "
                     << rhs.reason << "\nState " << static_cast<unsigned int>(rhs.state);
            return out;
        }
    };

    class StateChangeListener
    {
        public:
            virtual void onStateChangeReceived(TrajectoryState::StateInformation &info) = 0;
    };

    TrajectoryState();
    explicit TrajectoryState(State initial_state);

    StateInformation GetState();

    void UpdateState(Base::ActionNotification const &notif);

    void addListener(StateChangeListener* listener);
    void notifyListeners();

    bool operator== (TrajectoryState const& rhs);
    bool operator!= (TrajectoryState const& rhs);

private:

    StateInformation                                    m_current_state;
    StateInformation                                    m_previous_state;
    std::vector<StateChangeListener*>                   m_listeners;
};


#endif //TRAJECTORY_STATE_H

#ifndef TRAJECTORY_NOTIFICATIONMANAGER_H
#define TRAJECTORY_NOTIFICATIONMANAGER_H

/**
 * Copyright Kinova Inc. 2019
 */

#include <memory>
#include <thread>
#include <atomic>

#include "TrajectoryState.h"
#include "SimplifiedApi.h"

using namespace Kinova::ApiHelper;

class TrajectoryNotificationManager : public TrajectoryState::StateChangeListener,
                                      public ArmBaseApi::ApiStateListener
{
public:

    TrajectoryNotificationManager() = delete;
    explicit TrajectoryNotificationManager(std::weak_ptr<ArmBaseApi> api_ref);
    ~TrajectoryNotificationManager();

    void Initialize();

    TrajectoryState PollTrajectoryStatus();

    void onStateChangeReceived(TrajectoryState::StateInformation &info) override;
    void onApiStateChanged(ArmBaseApi::ApiState const& new_state) override;

    uint32_t GetLastError();

private:
    void Release();
    void Callback(Base::ActionNotification const &notification_data);
    void Subscribe(Common::NotificationHandle const& handle);

    std::weak_ptr<ArmBaseApi>                       m_pApi;

    TrajectoryState                                 m_state;
    std::vector<Common::NotificationHandle>         m_subscriptions;

    uint32_t m_lastError;
};


#endif //TRAJECTORY_NOTIFICATIONMANAGER_H

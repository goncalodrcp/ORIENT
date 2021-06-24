#ifndef SIMPLIFIED_K_API
#define SIMPLIFIED_K_API

#include <string>
#include <memory>
#include "ActionNotificationManager.h"
#include "TrajectoryNotificationManager.h"

namespace Kinova
{
    namespace ApiHelper
    {
        class ArmBaseApi;
    }
}

class ActionNotificationManager;
class TrajectoryNotificationManager;

class ArmBaseSimplifiedApi
{
private:
    std::shared_ptr<TrajectoryNotificationManager>     m_pTrajectoryNotificationManager;
    std::shared_ptr<ActionNotificationManager>         m_pActionNotificationManager;
    std::shared_ptr<Kinova::ApiHelper::ArmBaseApi>     m_pArmApi;

    bool m_is_subscribed_to_action = { false };

    uint32_t m_lastError;

public:
    explicit ArmBaseSimplifiedApi(const std::string& ipAddress);
    ~ArmBaseSimplifiedApi();

    std::shared_ptr<Kinova::ApiHelper::ArmBaseApi> GetApiRef();
    std::shared_ptr<ActionNotificationManager>     GetActionNotificationRef();
    std::shared_ptr<TrajectoryNotificationManager> GetTrajectoryManagerRef();

    uint32_t GetLastError();
};

#endif


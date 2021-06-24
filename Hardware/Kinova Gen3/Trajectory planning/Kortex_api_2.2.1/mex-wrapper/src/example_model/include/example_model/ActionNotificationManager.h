#ifndef _ACTION_NOTIFICATION_MANAGER_H
#define _ACTION_NOTIFICATION_MANAGER_H

#include <Base.pb.h>

#include "ArmBaseApi.h"

using namespace Kinova::Api;
using namespace Kinova::ApiHelper;

class ActionNotificationManager : public ArmBaseApi::ApiStateListener
{
public:
    explicit ActionNotificationManager(std::weak_ptr<ArmBaseApi> api_ref);
    ~ActionNotificationManager();

    void Initialize();
    void Release();
    void Callback(Base::ActionNotification const &notification_data);

    Base::ActionNotification FirstNotification();
    Base::ActionNotification LastNotification();

    void onApiStateChanged(ArmBaseApi::ApiState const& new_state) override;

private:

    void PushNotification(Base::ActionNotification val);
    void PopNotification();
    bool Subscribe(Kinova::Api::Common::NotificationHandle const& handle);

    std::vector<Kinova::Api::Common::NotificationHandle> m_subscriptions;
    std::queue<Base::ActionNotification>                 m_notification_queue;
    Base::ActionNotification                             m_last_notification;
    std::mutex                                           m_mutex;

    std::weak_ptr<ArmBaseApi>       m_api;
};


#endif //SIMPLIFIEDAPI_ACTIONNOTIFICATIONMANAGER_H

#ifndef _ARMBASE_API_H_
#define _ARMBASE_API_H_

#include <Frame.pb.h>
#include <Base.pb.h>
#include <BaseCyclic.pb.h>
#include <Session.pb.h>
#include <VisionConfig.pb.h>

#include <KDetailedException.h>
#include <HeaderInfo.h>

#include <TransportClientUdp.h>
#include <TransportClientTcp.h>
#include <RouterClient.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <VisionConfigClientRpc.h>
#include <DeviceManagerClientRpc.h>

#include <functional>
#include <vector>

#include "IArmBaseApi.h"
#include "KError.h"

#include <cstdio>

using namespace Kinova;
using namespace Api;

namespace Kinova
{
namespace ApiHelper
{

    enum error_ArmBaseApi
    {
        SUCCESS = 1,
        ERROR_DETECTED = 2,
        INIT_ERROR = 3
    };

    enum api_config 
    {
        PORT_HIGHLEVEL = 10000,
        PORT_LOWLEVEL  = 10001,
    };

    class ArmBaseApi : public IArmBaseApi
    {
        
    public:

        enum ApiState
        {
            Connected,
            SessionCreated,
            Closed,
        };

        class ApiStateListener
        {
        public:
            virtual void onApiStateChanged(ApiState const& new_state) = 0;
        };

        ArmBaseApi(std::string const& ipAddress, std::function<void(Kinova::Api::KError)> callback);
        explicit ArmBaseApi(std::string const& ipAddress);

        ~ArmBaseApi() override;

        bool IsInitialized() override;
        session_parameters GetSessionParameters() override;

        uint32_t CloseSession() override;
        uint32_t OpenSession(session_parameters const& params) override;
        uint32_t InitVision();

        Base::BaseClient* BaseServices();
        BaseCyclic::BaseCyclicClient* BaseCyclicServices();

        VisionConfig::VisionConfigClient* VisionConfigServices();
        DeviceManager::DeviceManagerClient* DeviceManagerServices();

        bool HasVisionModule();
        uint32_t GetVisionModuleID();

        void AddApiStateListener(std::weak_ptr<ApiStateListener> listener);

    private:
        uint32_t InitAPI(std::string const& ipAddress) override;
        uint32_t CloseAPI() override;
        void Release();

        void SetApiState(ApiState const& new_state);
        void notify();

    private:
        function<void(Kinova::Api::KError)> m_errorCallback;

        std::string m_ipAddress;

        bool m_isInitialized;
        bool m_isSessionOpened;
        bool m_HasVision;

        uint32_t m_VisionID;

        session_parameters m_SessionParameters;

        ITransportClient* m_pTransport_HighLevel{};
        RouterClient* m_pRouterClient_HighLevel{};
        ITransportClient* m_pTransport_LowLevel{};
        RouterClient* m_pRouterClient_LowLevel{};
        
        Base::BaseClient* m_pBase{};
        BaseCyclic::BaseCyclicClient* m_pBaseCyclic{};
        
        SessionManager* m_pSessionMng_HighLevel{};
        SessionManager* m_pSessionMng_LowLevel{};

        VisionConfig::VisionConfigClient* m_pVisionConfig{};
        DeviceManager::DeviceManagerClient * m_pDeviceMng{};

        std::vector<std::weak_ptr<ApiStateListener>>      m_listeners;

        ApiState         m_state;
    };

}
}

#endif // _ARMBASE_API_H_

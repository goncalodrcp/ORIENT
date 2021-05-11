#ifndef _I_ARM_BASE_API_
#define _I_ARM_BASE_API_

#include <string>
#include <functional>
#include "KError.h"

using namespace std;

namespace Kinova
{
namespace ApiHelper
{

    /*
    * This contains parameters to initiate a session
    */
    struct session_parameters
    {
        std::string username;
        std::string password;
        
        uint32_t sessionTimeout;
        uint32_t connectionTimeout;
    };

    class IArmBaseApi
    {
    protected:
        /* Initialize the API. (in the child those one will be used in constructor/destructor) */
        virtual uint32_t InitAPI(std::string const& ipAddress) = 0;
        
        /* Close and clear the ressources properly. (in the child those one will be used in constructor/destructor) */
        virtual uint32_t CloseAPI() = 0;

    public:
        virtual ~IArmBaseApi() {}

        /* Get the status of the initialization. This method should be used as a protection to avoid calling a target that has not been initialized.*/
        virtual bool IsInitialized() = 0;

        /* This allows the child class to specify the some parameters needed by the session manager. Each element of the vector represents a single session.*/
        virtual session_parameters GetSessionParameters() = 0;

        /* This allow the child class to open/close a session (logIn/logOut) */
        virtual uint32_t OpenSession(session_parameters const& params) = 0;
        virtual uint32_t CloseSession() = 0;
    };

}
}

#endif // _I_ARM_BASE_API_




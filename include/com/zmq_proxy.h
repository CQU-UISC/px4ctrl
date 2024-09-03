#include <zmq.hpp>
#include <json.hpp>
#include <thread>

namespace px4ctrl{
namespace gcs{

class ZmqProxy{
public:
    ZmqProxy(std::string base_dir);
    ~ZmqProxy();
    void join();
    void close();
private:
    void run();
    std::thread proxy_thread;

    zmq::context_t ctx;
    // Init XSUB socket
    zmq::socket_t xsub_socket;
    std::string xsub_endpoint;

    // Init XPUB socket
    zmq::socket_t xpub_socket;
    std::string xpub_endpoint;
};
}
}

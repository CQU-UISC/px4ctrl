#include <spdlog/spdlog.h>
#include <zmq.hpp>
#include <zmq_addon.hpp>

#include "server.h"
#include "datas.h"


namespace px4ctrl
{
namespace ui
{
ZmqProxy::ZmqProxy(zmq::context_t& ctx, const ZmqParas& paras)
    : paras(paras),
      xsub_endpoint(paras.xsub_bind), 
      xpub_endpoint(paras.xpub_bind),
      xsub_socket(ctx, zmq::socket_type::xsub),
      xpub_socket(ctx, zmq::socket_type::xpub)
{
    try
    {
        // proxy(ctx,paras.xsub_bind,paras.xpub_bind),
        spdlog::info("bind xsub:{}, xpub:{}", xsub_endpoint, xpub_endpoint);
        xsub_socket.bind(xsub_endpoint);
        xpub_socket.bind(xpub_endpoint);
    }
    catch (zmq::error_t& e)
    {
        spdlog::error("unable bind xsub/xpub, err:{}", e.what());
        throw e;
    }
    proxy_thread = std::thread(&ZmqProxy::run, this);
    return;
}

void ZmqProxy::run()
{
    try{
        zmq::proxy(xsub_socket, xpub_socket);
    }
    catch(zmq::error_t& e){
        if (e.num() == ETERM)
        {
            spdlog::info("proxy thread exited");
            xpub_socket.close();
            xsub_socket.close();
            return;
        }
        spdlog::error("proxy error:{}",e.what());
    }
    return;
}

void ZmqProxy::join()
{
    if (proxy_thread.joinable())
    {
        proxy_thread.join();
        spdlog::info("zmq proxy thread exited");
    }
}

ZmqProxy::~ZmqProxy(){
    join();
}

Px4Server::Px4Server(
    zmq::context_t& ctx,
    const ZmqParas& paras):
paras(paras),
server_pub(ctx,zmq::socket_type::pub),
client_sub(ctx,zmq::socket_type::sub)
{
    try {
        server_pub.connect(paras.xsub_url);
        client_sub.connect(paras.xpub_url);
        client_sub.set(zmq::sockopt::subscribe, paras.client_topic);
    } catch (const zmq::error_t &e) {
        spdlog::error("unable bind server/pub, err:{}", e.what());
        throw e;
    }
    recv_thread = std::thread(&Px4Server::recv_t, this);
    return;
}

void Px4Server::pub(const ServerPayload &payload){
    if(!ok){
        spdlog::error("zmq socket has shutdown");
        return;
    }
    server_pub.send(zmq::buffer(paras.server_topic), zmq::send_flags::sndmore);
    server_pub.send(zmq::buffer(&payload, sizeof(ServerPayload)));
    return;
}

void Px4Server::recv_t(){
    while(ok){
        try {
            std::vector<zmq::message_t> msgs;
            zmq::recv_result_t result = zmq::recv_multipart(client_sub, std::back_inserter(msgs));
            if (!result)
            {
                spdlog::error("recv client command failed");
                continue;
            }
            spdlog::info("recv client command");
            ClientPayload payload;
            unpack(msgs[1],payload);
            client_data.post(payload);
        } catch (const zmq::error_t &e) {
            if(e.num() == ETERM){
                spdlog::info("server recv thread exited");
                ok = false;
                server_pub.close();
                client_sub.close();
            }
            spdlog::error("server recv error:{}",e.what());
        }
    }
    return;
}

Px4Server::~Px4Server(){
    ok = false;
    recv_thread.join();
    spdlog::info("server closed");
    return;
}

} // namespace ui

} // namespace px4ctrl
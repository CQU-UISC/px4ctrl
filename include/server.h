#pragma once
#include <functional>
#include <mutex>
#include <spdlog/common.h>
#include <spdlog/sinks/base_sink.h>
#include <sstream>
#include <string>
#include <thread>
#include <zmq.hpp>

#include "types.h"
#include "datas.h"

namespace px4ctrl
{
namespace ui
{
class ZmqProxy{
    public:
        ZmqProxy(zmq::context_t& ctx, const ZmqParas& paras);
        ~ZmqProxy();
    private:
        const ZmqParas& paras;

        void run();
        void join();

        std::thread proxy_thread;
        std::string xsub_endpoint;
        std::string xpub_endpoint;

        zmq::socket_t xsub_socket;
        zmq::socket_t xpub_socket;
    };

class Px4Server{
    public:
        Px4Server(zmq::context_t& ctx, const ZmqParas& paras);
        ~Px4Server();

        void pub(const ServerPayload &payload);
        Px4Data<ClientPayload> client_data;

    private:
        const ZmqParas &paras;
        zmq::socket_t server_pub, client_sub;
        std::thread recv_thread;
        bool ok = true;
        void recv_t();
};

template <typename Mutex>
class zmq_sink : public spdlog::sinks::base_sink<Mutex>
{
public:
    zmq_sink(zmq::context_t& ctx, const ZmqParas& paras)
        :paras(paras), server_log_pub(ctx, zmq::socket_type::pub)
    {
        try {
            server_log_pub.connect(paras.xsub_url);
        } catch (zmq::error_t& e) {
            spdlog::error("unable bind server/pub, err:{}", e.what());
            throw e;
        }
    }

protected:
    const ZmqParas &paras;
    zmq::socket_t server_log_pub;
    bool exit_flag = false;
    inline void pub(const std::string& log){
        try {
            if (exit_flag) {
                return;
            }
            server_log_pub.send(zmq::buffer(paras.log_topic), zmq::send_flags::sndmore);
            server_log_pub.send(zmq::buffer(log.c_str(), log.size()));
        } catch (zmq::error_t& e) {
            if (e.num() == ETERM) {
                exit_flag = true;
                server_log_pub.close();
                return;
            }
        }
    }

    inline void sink_it_(const spdlog::details::log_msg& msg) override
    {
        spdlog::memory_buf_t formatted;
        spdlog::sinks::base_sink<Mutex>::formatter_->format(msg, formatted);

        std::stringstream ss;
        ss.write(formatted.data(),
                static_cast<std::streamsize>(formatted.size()));
        pub(ss.str());
    };

    inline void flush_() override{};
};

using zmq_sink_mt = zmq_sink<std::mutex>;
using zmq_sink_st = zmq_sink<spdlog::details::null_mutex>;
} // namespace ui

} // namespace px4ctrl
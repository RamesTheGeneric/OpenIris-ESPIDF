#pragma once
#ifndef RESTAPI_HPP
#define RESTAPI_HPP
#include <mongoose.h>
#include <CommandManager.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <cstdint>

#include "esp_log.h"

#define JSON_RESPONSE "Content-Type: application/json\r\n"

class UDPStream;

struct RequestContext
{
    mg_connection* connection;
    std::string method;
    std::string body;
};

struct RequestBaseData
{
    std::string allowed_method;
    CommandType command_type;
    int success_code;
    int error_code;
    RequestBaseData(std::string allowed_method, CommandType command_type, int success_code, int error_code)
        : allowed_method(allowed_method), command_type(command_type), success_code(success_code), error_code(error_code) {};
};

class RestAPI
{
    typedef std::unordered_map<std::string, RequestBaseData> route_map;
    std::string url;
    route_map routes;

    mg_mgr mgr;
    std::shared_ptr<CommandManager> command_manager;
    UDPStream* _udpStream = nullptr;

   private:
    void handle_endpoint_command(RequestContext* context, std::string allowed_method, CommandType command_type, int success_code, int error_code);
    void handle_udp_start(struct mg_connection* connection, struct mg_http_message* message);
    void handle_udp_stop(struct mg_connection* connection, struct mg_http_message* message);

   public:
    RestAPI(std::string url, std::shared_ptr<CommandManager> command_manager);
    void setUdpStream(UDPStream* stream) { _udpStream = stream; }
    void begin();
    void handle_request(struct mg_connection* connection, int event, void* event_data);
    void poll();
};

namespace RestAPIHelpers
{
void event_handler(struct mg_connection* connection, int event, void* event_data);
};

void HandleRestAPIPollTask(void* pvParameter);

#endif
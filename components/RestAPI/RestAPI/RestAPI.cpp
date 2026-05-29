#include "RestAPI.hpp"
#include <UDPStream.hpp>

#include <utility>

#define PATCH_METHOD "PATCH"
#define POST_METHOD "POST"
#define GET_METHOD "GET"
#define DELETE_METHOD "DELETE"

bool getIsSuccess(const nlohmann::json& response)
{
    if (!response.contains("result"))
    {
        return false;
    }

    return response.at("result").at("status").get<std::string>() == "success";
}

RestAPI::RestAPI(std::string url, std::shared_ptr<CommandManager> commandManager) : command_manager(commandManager)
{
    this->url = std::move(url);
    // updates via PATCH
    routes.emplace("/api/update/wifi/", RequestBaseData(PATCH_METHOD, CommandType::UPDATE_WIFI, 200, 400));
    routes.emplace("/api/update/device/mode/", RequestBaseData(PATCH_METHOD, CommandType::SWITCH_MODE, 200, 400));
    routes.emplace("/api/update/camera/", RequestBaseData(PATCH_METHOD, CommandType::UPDATE_CAMERA, 200, 400));
    routes.emplace("/api/update/ota/credentials", RequestBaseData(PATCH_METHOD, CommandType::UPDATE_OTA_CREDENTIALS, 200, 400));
    routes.emplace("/api/update/ap/", RequestBaseData(PATCH_METHOD, CommandType::UPDATE_AP_WIFI, 200, 400));
    routes.emplace("/api/update/led_duty_cycle/", RequestBaseData(PATCH_METHOD, CommandType::SET_LED_DUTY_CYCLE, 200, 400));

    // POST will set the data
    routes.emplace("/api/set/pause/", RequestBaseData(POST_METHOD, CommandType::PAUSE, 200, 400));
    routes.emplace("/api/set/wifi/", RequestBaseData(POST_METHOD, CommandType::SET_WIFI, 200, 400));
    routes.emplace("/api/set/mdns/", RequestBaseData(POST_METHOD, CommandType::SET_MDNS, 200, 400));
    routes.emplace("/api/set/config/save/", RequestBaseData(POST_METHOD, CommandType::SAVE_CONFIG, 200, 400));
    routes.emplace("/api/set/wifi/connect/", RequestBaseData(POST_METHOD, CommandType::CONNECT_WIFI, 200, 400));

    // resets via POST as well
    routes.emplace("/api/reset/config/", RequestBaseData(POST_METHOD, CommandType::RESET_CONFIG, 200, 400));

    // gets via GET
    routes.emplace("/api/get/config/", RequestBaseData(GET_METHOD, CommandType::GET_CONFIG, 200, 400));
    routes.emplace("/api/get/mdns/", RequestBaseData(GET_METHOD, CommandType::GET_MDNS_NAME, 200, 400));
    routes.emplace("/api/get/led_duty_cycle/", RequestBaseData(GET_METHOD, CommandType::GET_LED_DUTY_CYCLE, 200, 400));
    routes.emplace("/api/get/serial_number/", RequestBaseData(GET_METHOD, CommandType::GET_SERIAL, 200, 400));
    routes.emplace("/api/get/led_current/", RequestBaseData(GET_METHOD, CommandType::GET_LED_CURRENT, 200, 400));
    routes.emplace("/api/get/who_am_i/", RequestBaseData(GET_METHOD, CommandType::GET_WHO_AM_I, 200, 400));

    // deletes via DELETE
    routes.emplace("/api/delete/wifi", RequestBaseData(DELETE_METHOD, CommandType::DELETE_NETWORK, 200, 400));

    // reboots via POST
    routes.emplace("/api/reboot/device/", RequestBaseData(GET_METHOD, CommandType::RESTART_DEVICE, 200, 500));

    // heartbeat via GET
    routes.emplace("/api/ping/", RequestBaseData(GET_METHOD, CommandType::PING, 200, 400));
}

void RestAPI::begin()
{
    mg_log_set(MG_LL_DEBUG);
    mg_mgr_init(&mgr);
    // every route is handled through this class, with commands themselves by a command manager
    // hence we pass a pointer to this in mg_http_listen
    mg_http_listen(&mgr, this->url.c_str(), (mg_event_handler_t)RestAPIHelpers::event_handler, this);
}

void RestAPI::handle_request(struct mg_connection* connection, int event, void* event_data)
{
    if (event == MG_EV_HTTP_MSG)
    {
        auto const* message = static_cast<struct mg_http_message*>(event_data);
        auto const uri = std::string(message->uri.buf, message->uri.len);

        // Handle UDP stream endpoints directly (need client IP from connection)
        if (uri == "/api/stream/udp/start" && std::string(message->method.buf, message->method.len) == "POST")
        {
            handle_udp_start(connection, const_cast<struct mg_http_message*>(message));
            return;
        }
        if (uri == "/api/stream/udp/stop" && std::string(message->method.buf, message->method.len) == "POST")
        {
            handle_udp_stop(connection, const_cast<struct mg_http_message*>(message));
            return;
        }

        if (this->routes.find(uri) == this->routes.end())
        {
            mg_http_reply(connection, 404, "", "Wrong URL");
            return;
        }

        auto const base_request_params = this->routes.at(uri);

        auto* context = new RequestContext{
            .connection = connection,
            .method = std::string(message->method.buf, message->method.len),
            .body = std::string(message->body.buf, message->body.len),
        };
        this->handle_endpoint_command(context, base_request_params.allowed_method, base_request_params.command_type, base_request_params.success_code,
                                      base_request_params.error_code);
    }
}

void RestAPIHelpers::event_handler(struct mg_connection* connection, int event, void* event_data)
{
    auto* rest_api_handler = static_cast<RestAPI*>(connection->fn_data);
    rest_api_handler->handle_request(connection, event, event_data);
}

void RestAPI::poll()
{
    mg_mgr_poll(&mgr, 100);
}

void HandleRestAPIPollTask(void* pvParameter)
{
    auto* rest_api_handler = static_cast<RestAPI*>(pvParameter);
    while (true)
    {
        rest_api_handler->poll();
        vTaskDelay(1000);
    }
}

// ---------------------------------------------------------------------------
// UDP stream start/stop handlers (bypass CommandManager, need client IP)
// ---------------------------------------------------------------------------
void RestAPI::handle_udp_start(struct mg_connection* connection, struct mg_http_message* message)
{
    if (!_udpStream) {
        mg_http_reply(connection, 500, JSON_RESPONSE, "{\"status\":\"error\",\"message\":\"UDP stream not available\"}");
        return;
    }

    // Extract client IP from connection peer address
    char peer_str[48];
    mg_snprintf(peer_str, sizeof(peer_str), "%M", mg_print_ip, &connection->rem);
    uint32_t client_ip = 0;
    // Parse dotted-decimal IP
    unsigned int b[4] = {0};
    if (sscanf(peer_str, "%u.%u.%u.%u", &b[0], &b[1], &b[2], &b[3]) == 4) {
        client_ip = (b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3];
    }

    if (client_ip == 0) {
        mg_http_reply(connection, 400, JSON_RESPONSE, "{\"status\":\"error\",\"message\":\"Could not determine client IP\"}");
        return;
    }

    // Parse optional port from JSON body
    uint16_t port = CONFIG_UDP_STREAM_PORT;
    if (message->body.len > 0) {
        double port_val;
        if (mg_json_get_num(message->body, "$.port", &port_val)) {
            int parsed_port = (int)port_val;
            if (parsed_port > 0 && parsed_port <= 65535) {
                port = (uint16_t)parsed_port;
            }
        }
    }

    if (!_udpStream->start(client_ip, port)) {
        mg_http_reply(connection, 409, JSON_RESPONSE,
            "{\"status\":\"error\",\"message\":\"UDP stream already active or failed to start\"}");
        return;
    }

    mg_http_reply(connection, 200, JSON_RESPONSE,
        "{\"status\":\"success\",\"data\":{\"port\":%u,\"client_ip\":\"%s\"}}", port, peer_str);
}

void RestAPI::handle_udp_stop(struct mg_connection* connection, struct mg_http_message* message)
{
    (void)message;
    if (!_udpStream) {
        mg_http_reply(connection, 500, JSON_RESPONSE, "{\"status\":\"error\",\"message\":\"UDP stream not available\"}");
        return;
    }

    _udpStream->stop();

    mg_http_reply(connection, 200, JSON_RESPONSE, "{\"status\":\"success\"}");
}

void RestAPI::handle_endpoint_command(RequestContext* context, std::string allowed_method, CommandType command_type, int success_code, int error_code)
{
    if (context->method != allowed_method)
    {
        mg_http_reply(context->connection, 401, JSON_RESPONSE, "{%m:%m}", MG_ESC("error"), "Method not allowed");
        return;
    }

    const nlohmann::json result = command_manager->executeFromType(command_type, context->body);
    const auto code = getIsSuccess(result) ? success_code : error_code;
    mg_http_reply(context->connection, code, JSON_RESPONSE, result.dump().c_str());
}
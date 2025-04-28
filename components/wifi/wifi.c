#include "wifi.h"
#include "wifi_config.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "navigation.h"

static const char *TAG = "WiFi";
static httpd_handle_t server = NULL;

static add_waypoint_cb_t add_waypoint_cb = NULL;
static remove_waypoint_cb_t remove_waypoint_cb = NULL;
static start_navigation_cb_t start_navigation_cb = NULL;
static stop_navigation_cb_t stop_navigation_cb = NULL;

// HTTP请求处理函数
static esp_err_t add_waypoint_handler(httpd_req_t *req)
{
    if(req->content_len <= 0 || req->content_len > 100) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid content length");
        return ESP_FAIL;
    }

    char buf[100] = {0};
    int ret = httpd_req_recv(req, buf, sizeof(buf));
    if(ret <= 0) {
        return ESP_FAIL;
    }

    double lat, lon;
    if(sscanf(buf, "lat=%lf&lon=%lf", &lat, &lon) != 2) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid parameters");
        return ESP_FAIL;
    }

    if(!add_waypoint_cb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    add_waypoint_cb(lat, lon);
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t remove_waypoint_handler(httpd_req_t *req)
{
    char buf[100];
    int ret, remaining = req->content_len;
    
    while (remaining > 0) {
        if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            return ESP_FAIL;
        }
        remaining -= ret;
    }
    
    int index;
    if (sscanf(buf, "index=%d", &index) == 1) {
        if (remove_waypoint_cb) {
            remove_waypoint_cb(index);
            httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        }
    }
    
    httpd_resp_send_500(req);
    return ESP_FAIL;
}

static esp_err_t start_navigation_handler(httpd_req_t *req)
{
    if (start_navigation_cb) {
        start_navigation_cb();
        httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    httpd_resp_send_500(req);
    return ESP_FAIL;
}

static esp_err_t stop_navigation_handler(httpd_req_t *req)
{
    if(!stop_navigation_cb) {
        ESP_LOGE(TAG, "Stop navigation callback not set");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    stop_navigation_cb();
    ESP_LOGI(TAG, "Navigation stopped by HTTP request");
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static const httpd_uri_t add_waypoint_uri = {
    .uri = "/add_waypoint",
    .method = HTTP_POST,
    .handler = add_waypoint_handler,
    .user_ctx = NULL
};

static const httpd_uri_t remove_waypoint_uri = {
    .uri = "/remove_waypoint",
    .method = HTTP_POST,
    .handler = remove_waypoint_handler,
    .user_ctx = NULL
};

static const httpd_uri_t start_navigation_uri = {
    .uri = "/start_navigation",
    .method = HTTP_POST,
    .handler = start_navigation_handler,
    .user_ctx = NULL
};

static const httpd_uri_t stop_navigation_uri = {
    .uri = "/stop_navigation",
    .method = HTTP_POST,
    .handler = stop_navigation_handler,
    .user_ctx = NULL
};

static void wifi_event_handler(void* arg, esp_event_base_t event_base, 
                             int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Station " MACSTR " joined, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Station " MACSTR " left, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

// 完善 read_waypoints_handler 函数
static esp_err_t read_waypoints_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    if(!root) {
        httpd_resp_send_500(req);
        ESP_LOGE(TAG, "Failed to create JSON object");
        return ESP_FAIL;
    }

    // 从导航模块获取实际数据
    cJSON *waypoints = cJSON_CreateArray();
    waypoint_node_t *node = read_all_waypoints();
    while (node) {
        cJSON *waypoint = cJSON_CreateObject();
        cJSON_AddNumberToObject(waypoint, "latitude", node->data.latitude);
        cJSON_AddNumberToObject(waypoint, "longitude", node->data.longitude);
        cJSON_AddItemToArray(waypoints, waypoint);
        node = node->next;
    }
    cJSON_AddItemToObject(root, "waypoints", waypoints);
    cJSON_AddNumberToObject(root, "count", waypoint_count);
    cJSON_AddStringToObject(root, "status", "ok");

    char *json_str = cJSON_PrintUnformatted(root);
    if (!json_str) {
        httpd_resp_send_500(req);
        ESP_LOGE(TAG, "Failed to print JSON string");
        cJSON_Delete(root);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));
    
    cJSON_free(json_str);
    cJSON_Delete(root);
    return ESP_OK;
}

// 完善 wifi_init 函数中的 STA 模式事件处理
void wifi_init(wifi_mode_t mode)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    if(mode == WIFI_MODE_AP) {
        esp_netif_create_default_wifi_ap();
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
        
        wifi_config_t wifi_config = {
            .ap = {
                .ssid = DEFAULT_SSID,
                .ssid_len = strlen(DEFAULT_SSID),
                .channel = 6,
                .password = DEFAULT_PASSWORD,
                .max_connection = MAX_CLIENTS,
                .authmode = WIFI_AUTH_WPA_WPA2_PSK
            }
        };
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    } 
    else {
        esp_netif_create_default_wifi_sta();
        // 添加 STA 模式的事件处理
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

        wifi_config_t wifi_config = {
            .sta = {
                .ssid = DEFAULT_SSID,
                .password = DEFAULT_PASSWORD,
                .reconnect_enable = true,
                .reconnect_interval = 10
            }
        };
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    }
    
    ESP_ERROR_CHECK(esp_wifi_start());
    start_http_server();
}

static const httpd_uri_t finish_waypoint_uri = {
    .uri = "/finish_waypoint",
    .method = HTTP_POST,
    .handler = finish_waypoint_handler,
    .user_ctx = NULL
};

static const httpd_uri_t read_waypoints_uri = {
    .uri = "/read_waypoints",
    .method = HTTP_GET,
    .handler = read_waypoints_handler,
    .user_ctx = NULL
};

void start_http_server() {
    if(server) {
        ESP_LOGE(TAG, "HTTP server already running");
        return;
    }

    if(!add_waypoint_cb || !remove_waypoint_cb || 
       !start_navigation_cb || !stop_navigation_cb) {
        ESP_LOGE(TAG, "Callback functions not set");
        return;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 12;
    config.stack_size = 8192;  // 增加栈大小
    
    if(httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return;
    }
    httpd_register_uri_handler(server, &add_waypoint_uri);
    httpd_register_uri_handler(server, &remove_waypoint_uri);
    httpd_register_uri_handler(server, &start_navigation_uri);
    httpd_register_uri_handler(server, &stop_navigation_uri);
    httpd_register_uri_handler(server, &finish_waypoint_uri); // 注册新的处理函数
    httpd_register_uri_handler(server, &read_waypoints_uri);  // 注册新的处理函数
    ESP_LOGI(TAG, "HTTP服务器启动成功");
}

void stop_http_server()
{
    if (server) {
        httpd_stop(server);
        server = NULL;
        ESP_LOGI(TAG, "HTTP服务器已停止");
    }
}

void set_add_waypoint_cb(add_waypoint_cb_t cb)
{
    add_waypoint_cb = cb;
}

void set_remove_waypoint_cb(remove_waypoint_cb_t cb)
{
    remove_waypoint_cb = cb;
}

void set_start_navigation_cb(start_navigation_cb_t cb)
{
    start_navigation_cb = cb;
}

void set_stop_navigation_cb(stop_navigation_cb_t cb)
{
    stop_navigation_cb = cb;
}


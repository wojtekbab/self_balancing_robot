/**
 * @file web_server.hpp
 * @author Wojciech Babicki (wojciech.babicki@op.pl)
 * @brief 
 * @version 0.1
 * @date 2024-07-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef WEB_SERVER_HPP
#define WEB_SERVER_HPP

#include <ESPAsyncWebServer.h>

/**
 * @brief string contains html page
 * 
 */
extern const char index_html[];
/**
 * @brief functions set callbacks when server send data, starts server
 * 
 */
void setupWebServer();

#endif // WEB_SERVER_HPP

// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2018, SICK AG, Waldkirch
*  Copyright (C) 2018, FZI Forschungszentrum Informatik, Karlsruhe, Germany
*
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*    http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

*/

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!
 * \file AsyncUDPClient.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------


#include <communication/AsyncUDPClient.h>

namespace sick {
namespace communication {
AsyncUDPClient::AsyncUDPClient(const PacketHandler& packet_handler,
                               boost::asio::io_service& io_service,
                               const uint16_t& local_port)
  : m_packet_handler(packet_handler)
  , m_io_service(io_service)
{
  // Keep io_service busy
  m_io_work_ptr = std::make_shared<boost::asio::io_service::work>(m_io_service);
  try
  {
    auto endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), local_port);
    m_socket_ptr  = std::make_shared<boost::asio::ip::udp::socket>(m_io_service, endpoint);
  }
  catch (const std::exception& e)
  {
    printf("Exception while creating socket: %s\n", e.what());
  }
  printf("UDP client is setup\n");
}

AsyncUDPClient::AsyncUDPClient(const PacketHandler& packet_handler,
                               boost::asio::io_service& io_service,
                               const boost::asio::ip::address_v4 host_ip,
                               const boost::asio::ip::address_v4 interface_ip,
                               const uint16_t& local_port)
  : m_packet_handler(packet_handler)
  , m_io_service(io_service)
{
  if (!host_ip.is_multicast())
  {
    printf("Provided Host IP is not in the Multicast range!\n");
    exit(-1);
  }
  if (interface_ip.is_unspecified())
  {
    // TODO better error handling
    printf("Provided Interface IP is unspecified! Set it to the Interface IP which receives the "
              "multicast packages.\n");
    printf("Shutting down node\n");
    exit(-1);
  }
  // Keep io_service busy
  m_io_work_ptr = std::make_shared<boost::asio::io_service::work>(m_io_service);
  try
  {
    auto endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), local_port);
    m_socket_ptr  = std::make_shared<boost::asio::ip::udp::socket>(m_io_service, endpoint);
    m_socket_ptr->set_option(boost::asio::ip::multicast::join_group(host_ip, interface_ip));
  }
  catch (const std::exception& e)
  {
    printf("Exception while creating socket: %s\n", e.what());
  }
  printf("UDP client is setup\n");
}

AsyncUDPClient::~AsyncUDPClient()
{
  m_io_service.stop();
}

void AsyncUDPClient::startReceive()
{
  m_socket_ptr->async_receive_from(boost::asio::buffer(m_recv_buffer),
                                   m_remote_endpoint,
                                   [this](boost::system::error_code ec, std::size_t bytes_recvd) {
                                     this->handleReceive(ec, bytes_recvd);
                                   });
}

void AsyncUDPClient::handleReceive(const boost::system::error_code& error,
                                   const std::size_t& bytes_transferred)
{
  if (!error)
  {
    sick::datastructure::PacketBuffer packet_buffer(m_recv_buffer, bytes_transferred);
    m_packet_handler(packet_buffer);
  }
  else
  {
    printf("Error in UDP handle receive: %i\n", error.value());
  }
  startReceive();
}


void AsyncUDPClient::runService()
{
  startReceive();
}

unsigned short AsyncUDPClient::getLocalPort()
{
  if (m_socket_ptr)
  {
    return m_socket_ptr->local_endpoint().port();
  }
  return 0;
}

} // namespace communication
} // namespace sick

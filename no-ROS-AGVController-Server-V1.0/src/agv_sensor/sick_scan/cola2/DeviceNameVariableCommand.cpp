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
 * \file DeviceNameVariableCommand.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-17
 */
//----------------------------------------------------------------------

#include <cola2/DeviceNameVariableCommand.h>

#include <cola2/Cola2Session.h>
#include <cola2/Command.h>

namespace sick {
namespace cola2 {

DeviceNameVariableCommand::DeviceNameVariableCommand(Cola2Session& session,
                                                     datastructure::DeviceName& device_name)
  : VariableCommand(session, 17)
  , m_device_name(device_name)
{
  m_device_name_parser_ptr = std::make_shared<sick::data_processing::ParseDeviceName>();
}

bool DeviceNameVariableCommand::canBeExecutedWithoutSessionID() const
{
  return true;
}

bool DeviceNameVariableCommand::processReply()
{
  if (!base_class::processReply())
  {
    return false;
  }

  m_device_name_parser_ptr->parseTCPSequence(getDataVector(), m_device_name);


  return true;
}


} // namespace cola2
} // namespace sick

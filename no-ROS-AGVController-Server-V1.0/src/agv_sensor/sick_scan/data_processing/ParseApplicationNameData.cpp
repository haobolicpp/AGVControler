// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2019, SICK AG, Waldkirch
*  Copyright (C) 2019, FZI Forschungszentrum Informatik, Karlsruhe, Germany
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
 * \file ParseApplicationNameData.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-07-22
 */
//----------------------------------------------------------------------

#include <data_processing/ParseApplicationNameData.h>

#include <cola2/Command.h>

namespace sick {
namespace data_processing {

ParseApplicationNameData::ParseApplicationNameData() {}


bool ParseApplicationNameData::parseTCPSequence(
  const datastructure::PacketBuffer& buffer,
  sick::datastructure::ApplicationName& application_name) const
{
  // Keep our own copy of the shared_ptr to keep the iterators valid
  const std::shared_ptr<std::vector<uint8_t> const> vec_ptr = buffer.getBuffer();
  std::vector<uint8_t>::const_iterator data_ptr             = vec_ptr->begin();
  application_name.setVersionCVersion(readVersionIndicator(data_ptr));
  application_name.setVersionMajorVersionNumber(readMajorNumber(data_ptr));
  application_name.setVersionMinorVersionNumber(readMinorNumber(data_ptr));
  application_name.setVersionReleaseNumber(readReleaseNumber(data_ptr));
  application_name.setNameLength(readNameLength(data_ptr));
  application_name.setApplicationName(readApplicationName(data_ptr));
  return true;
}

std::string
ParseApplicationNameData::readVersionIndicator(std::vector<uint8_t>::const_iterator data_ptr) const
{
  std::string result;
  result.push_back(read_write_helper::readUint8(data_ptr + 0));
  return result;
}

uint8_t
ParseApplicationNameData::readMajorNumber(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 1);
}

uint8_t
ParseApplicationNameData::readMinorNumber(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 2);
}

uint8_t
ParseApplicationNameData::readReleaseNumber(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint8(data_ptr + 3);
}

uint32_t
ParseApplicationNameData::readNameLength(std::vector<uint8_t>::const_iterator data_ptr) const
{
  return read_write_helper::readUint32LittleEndian(data_ptr + 4);
}


std::string
ParseApplicationNameData::readApplicationName(std::vector<uint8_t>::const_iterator data_ptr) const
{
  uint32_t name_length = read_write_helper::readUint32LittleEndian(data_ptr + 4);
  std::string name;
  for (uint8_t i = 0; i < name_length; i++)
  {
    name.push_back(read_write_helper::readUint8(data_ptr + 8 + i));
  }
  return name;
}


} // namespace data_processing
} // namespace sick

#pragma once

/*
 * Copyright (C) <SUB><year> LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <iostream>
#include <vector>

namespace DSRCApplication
{

  /**
   * \brief Stuct containing the algorithm configuration values for ros2_dsrc_driver
   */
  struct Config
  {
    //! Example parameter
    std::string dsrc_address = "169.254.1.1";
    int dsrc_listening_port = 1516;
    int listening_port = 5398;

    // Stream operator for this config
    // TODO for USER: Update prints for the added parameters
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "ros2_dsrc_driver::Config { " << std::endl
           << "listening_port: " << c.listening_port << std::endl
           << "dsrc_listening_port: " << c.dsrc_listening_port << std::endl
           << "dsrc_address: " << c.dsrc_address << std::endl
           << "}" << std::endl;
    
      return output;
    }
  };

}

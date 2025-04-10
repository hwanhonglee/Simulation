/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// HH_250327 
#ifndef __POINTCLOUDXYZIRCAEDT_H
#define __POINTCLOUDXYZIRCAEDT_H

#include <pcl/point_cloud.h>
#include <velodyne_pointcloud/datacontainerbase.h>

#include <velodyne_pointcloud/point_types.h>

namespace velodyne_pointcloud
{
class PointcloudXYZIRCAEDT : public velodyne_rawdata::DataContainerBase
{
public:
  pcl::PointCloud<velodyne_pointcloud::PointXYZIRCAEDT>::Ptr pc;

  PointcloudXYZIRCAEDT() : pc(new pcl::PointCloud<velodyne_pointcloud::PointXYZIRCAEDT>) {}

  // HH_250409
  virtual void addPoint(
    const float & x, const float & y, const float & z,
    const uint8_t & intensity, const uint8_t & return_type, const uint16_t & channel,
    const float & azimuth, const float & elevation, const float & distance,
    const uint32_t & time_stamp) override;
};
}  // namespace velodyne_pointcloud
#endif

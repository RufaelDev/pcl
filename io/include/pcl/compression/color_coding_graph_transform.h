/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2015, Centrum Wiskunde Informatica.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Centrum Wiskunde Informatica nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef COLOR_COMPRESSION_GRAPHTF_H
#define COLOR_COMPRESSION_GRAPHTF_H

#include <iterator>
#include <iostream>
#include <vector>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <string.h>

#include <pcl/compression/octree_pointcloud_compression.h>

namespace pcl
{
  namespace octree
  { 
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b ColorCodingGraphTransform class
     "Point Cloud Attribute Compression With Graph Transform" 
      Cha Zhang, Dinei Florencio and Charles Loop 
      IEEE International conference on image processing 2014 (ICIP'14), Paris France
     *  \note This class encodes 8-bit color information for octree-based point cloud compression.
     *  \note
     *  \note typename: PointT: type of point used in pointcloud
     *  \author Rufael Mekuria (rufael.mekuria@cwi.nl) */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    template<typename PointT, 
        typename LeafT = OctreeContainerPointIndices,
        typename BranchT = OctreeContainerEmpty,
        typename OctreeT = Octree2BufBase<LeafT, BranchT>>
    class colorCodingGraphTF
    {
      // octree point cloud
      public:
        // public typedefs
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

        void 
        encodeColors(OctreeT &octree_in, PointCloud &cloud_in,  int level, int coding_profile);

        void
        decodeColors(OctreeT &octree_in, PointCloud &cloud_inout, int level, int coding_profile);
    };
  }
}

#endif

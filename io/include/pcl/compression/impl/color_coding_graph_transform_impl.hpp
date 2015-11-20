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
#ifndef COLOR_COMPRESSION_GRAPHTF_HPP
#define COLOR_COMPRESSION_GRAPHTF_HPP

#include <pcl/compression/color_coding_graph_transform.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <Eigen/dense>
#include<pcl/common/eigen.h>

namespace pcl
{
  namespace octree
  { 
    
    // adjacency list
    struct gftAdjacencyListEntry{
      int index;
      int weight;
    };

    // adjacency matrix
    struct gftAdjacencyMatrix
    {
      int height;
      int width;
      std::vector<float> data;
    };

    struct RGBVal
    {
      uint8_t r;
      uint8_t g;
      uint8_t b;
    };

    typedef vector<gftAdjacencyListEntry> gftAdjacancyList; 

    typedef vector<gftAdjacencyMatrix> gftAdjacancyMatrices;

    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void 
      colorCodingGraphTF<PointT, LeafT, BranchT, OctreeT>::encodeColors(OctreeT &octree_in, PointCloud &cloud_in,  int level, int coding_profile)
    {
      int levels_in_tree = octree_in.getTreeDepth();
      int leaf_count_per_level = 0;
      int cum_leaf_count = 0;

      // level keys
      vector<OctreeKey> levelKeys;     //! all levelN keys in a vector

      // level content
      vector<vector<OctreeKey>> levelContent; //! all contents of LevelN
      vector<vector<RGBVal>> levelColors;

      // temp
      vector<OctreeKey> tmp_currentLevelContent;
      OctreeKey tmp_currentLevelKey;
      vector<RGBVal> temp_rgb;

      OctreeDepthFirstIterator<OctreeT> it = octree_in.begin();
      OctreeDepthFirstIterator<OctreeT> it_end = octree_in.end();        

      // iterate the tree to find the levelN entries
      for (; it != it_end; ++it)
      {

        if(it.isLeafNode())
        {
          tmp_currentLevelContent.push_back(it.getCurrentOctreeKey());
          std::vector<int> &leaf_count = it.getLeafContainer().getPointIndicesVector();
          
          if(leaf_count.size()){
          
            int av_color_val_r=0; 
            int av_color_val_g=0; 
            int av_color_val_b=0; 
            
            for(int i=0;i<leaf_count.size();i++)
            {
              av_color_val_r+= cloud_in.points[leaf_count[i]].r;
              av_color_val_g+= cloud_in.points[leaf_count[i]].g;
              av_color_val_b+= cloud_in.points[leaf_count[i]].b;
            }

            av_color_val_r = av_color_val_r/leaf_count.size();
            av_color_val_g = av_color_val_g/leaf_count.size();
            av_color_val_b = av_color_val_b/leaf_count.size();
            RGBVal clr={};

            clr.r = (uint8_t) av_color_val_r;
            clr.g = (uint8_t) av_color_val_g;
            clr.b = (uint8_t) av_color_val_b;

            temp_rgb.push_back(clr);
          }
        }
        if(it.getCurrentOctreeDepth() == (levels_in_tree - level))
        {

          if(!tmp_currentLevelContent.empty())
          {
            levelKeys.push_back(tmp_currentLevelKey);
            levelContent.push_back(tmp_currentLevelContent);
            levelColors.push_back(temp_rgb);

            temp_rgb.clear();
            tmp_currentLevelContent.clear();
          }

          tmp_currentLevelKey = it.getCurrentOctreeKey();
        }
      }

      if(!tmp_currentLevelContent.empty())
      {
        levelKeys.push_back(tmp_currentLevelKey);
        levelContent.push_back(tmp_currentLevelContent);
        levelColors.push_back(temp_rgb);
        temp_rgb.clear();
      }

      // adjacency lists for each level, each entry has a vector
      vector<vector<gftAdjacancyList>> levelAdjacencies;
      levelAdjacencies.resize(levelContent.size());

      // for each level or each entry compute adjacency list 
      for(int i=0; i<levelContent.size();++i)
      {
        // allocate adjacency list for each of the entries in the level 
        levelAdjacencies[i].resize(levelContent[i].size());

        //
        for(int j=0; j < levelContent[i].size(); j++)
        {
          // for each entry j
          OctreeKey &jKey = levelContent[i][j];

          // search for adjacencies in this level
          for(int k=0; k <levelContent[i].size();k++)
          {
            OctreeKey &kKey = levelContent[i][k];

            int diff[3]={};
            int diffsqr=0; 

            diff[0] = (int) jKey.x  - (int) kKey.x;
            diff[1] = (int) jKey.y  - (int) kKey.y;
            diff[2] = (int) jKey.z  - (int) kKey.z;

            diffsqr = diff[0] * diff[0] + diff[1]* diff[1] + diff[2] * diff[2]; 

            if((diffsqr > 0) &&  (diffsqr <= 3))
            {
              gftAdjacencyListEntry a; 
              a.index  = k;
              a.weight = 1/std::sqrt( (double)diffsqr);
              levelAdjacencies[i][j].push_back(a);
            }
          }
        }
      }

      // Construct the precision matrices for each level
      gftAdjacancyMatrices lPrecisionMatrices(levelContent.size());

      for(int i=0; i<levelContent.size();++i)
      {
        int matrix_width = levelContent[i].size();
        lPrecisionMatrices[i].width = matrix_width;
        lPrecisionMatrices[i].data.resize(matrix_width * matrix_width);

        for(int j=0; j <levelContent[i].size(); j++)
        {
          double d_weights_sum=0;

          for(int k=0; k<levelAdjacencies[i][j].size();k++)
          {
            lPrecisionMatrices[i].data[j*matrix_width + levelAdjacencies[i][j][k].index] 
            = -1 * levelAdjacencies[i][j][k].weight;
            d_weights_sum+=levelAdjacencies[i][j][k].weight;
          }

          lPrecisionMatrices[i].data[j*matrix_width + j] = d_weights_sum;
        }
      }


      // compute the eigenvalue decomposition of each precision matrix
      std::vector<boost::shared_ptr<Eigen::MatrixXd>> diag_eigen_vals(levelContent.size());
      std::vector<boost::shared_ptr<Eigen::MatrixXd>> eigen_vecs(levelContent.size());

      for(int i=0; i<levelContent.size();++i)
      {
        int matrix_width = lPrecisionMatrices[i].width;
        Eigen::MatrixXd eAMatrix(matrix_width, matrix_width);

        for(int n=0;n<matrix_width;n++)
          for(int m=0;m<matrix_width;m++)
            eAMatrix(n,m) = lPrecisionMatrices[i].data[n*matrix_width + m];

        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(eAMatrix);

        std::cout << "The eigenvalues of A are:" << endl << solver.eigenvalues() << endl;
        std::cout << "The matrix of eigenvectors, V, is:" << endl << solver.eigenvectors() << endl << endl;

        diag_eigen_vals[i] = boost::shared_ptr<Eigen::MatrixXd>(new Eigen::MatrixXd(matrix_width,matrix_width));
        eigen_vecs[i] = boost::shared_ptr<Eigen::MatrixXd> (new Eigen::MatrixXd(matrix_width,matrix_width));

        *diag_eigen_vals[i] = solver.eigenvalues().asDiagonal();
        *eigen_vecs[i] = solver.eigenvectors();
      }

      // colors resulting from the graph transform
      std::vector<boost::shared_ptr<Eigen::MatrixXd>> gft_results(levelContent.size());

      // for each of the levels compute the color on the eigen values
      for(int i=0; i<levelContent.size();++i)
      {
        int matrix_width = lPrecisionMatrices[i].width;

        //compute the color vectors
        Eigen::MatrixXd color_matrix(matrix_width,3);
        for(int j=0; j < matrix_width;j++){
          color_matrix(j,0) = (double) levelColors[i][j].r;
          color_matrix(j,1) = (double) levelColors[i][j].g;
          color_matrix(j,2) = (double) levelColors[i][j].b;
        }

        // do the graph transform
        gft_results[i] = boost::shared_ptr<Eigen::MatrixXd>(new Eigen::MatrixXd(matrix_width,3));
        *gft_results[i] = *eigen_vecs[i] * color_matrix;
      }


      /*
      // with the adjacency lists we can create the adjacency matrices, for now just output the adjacency list
      for(int i=0; i<levelContent.size();++i)
      {
      // print the level key
      std::cout 
      << " key of the level is: " << " x " 
      << levelKeys[i].x << " y " 
      << levelKeys[i].y << " z "
      << levelKeys[i].z 
      << std::endl;

      // print adjacency list
      std::cout << " level " << i << " adjacency lists " << std::endl;
      for(int j=0; j <levelContent[i].size(); j++)
      {
      std::cout << " node " << j << " : "; 
      for(int k=0; k<levelAdjacencies[i][j].size();k++)
      {
      std::cout << "  "  << levelAdjacencies[i][j][k].index << " ";
      }
      std::cout << std::endl;
      }

      } 
      */
    };

    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void 
      colorCodingGraphTF<PointT, LeafT, BranchT, OctreeT>::decodeColors(OctreeT &octree_in, PointCloud &cloud_inout, int level, int coding_profile)
    {

    };
  }
}
#endif
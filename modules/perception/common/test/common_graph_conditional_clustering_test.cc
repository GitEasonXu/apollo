/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <gtest/gtest.h>
#include <limits>
#include "modules/perception/base/point_cloud_types.h"
#include "modules/perception/common/geometry/basic.h"
#include "modules/perception/common/graph/conditional_clustering.h"
namespace apollo {
namespace perception {
namespace common {

int easy_candidate(const base::PointF& p, void* vg,
                   std::vector<int>* nn_indices) {
  nn_indices->push_back(1);
  return nn_indices->size();
}

int easy_candidate_all(const base::PointF& p, void* vg,
                       std::vector<int>* nn_indices) {
  for (int i = 0; i < 300; i++) {
    nn_indices->push_back(i);
  }
  return nn_indices->size();
}

int easy_candidate_error(const base::PointF& p, void* vg,
                         std::vector<int>* nn_indices) {
  return -1;
}

bool easy_condition(const base::PointF& p1, const base::PointF& p2, void* vg) {
  float dist = 5.0f;
  if (CalculateEuclidenDist(p1, p2) <= dist) {
    return true;
  }
  return false;
}

bool easy_condition_false(const base::PointF& p1, const base::PointF& p2,
                          void* vg) {
  return false;
}

TEST(ConditionalClusteringTest, conditional_clustering_test) {
  ConditionClustering<base::PointF> condition_clustering =
      ConditionClustering<base::PointF>(false);
  base::PolygonFTypePtr polygon_in_ptr =
      base::PolygonFTypePtr(new base::PolygonFType(16, 16, base::PointF()));
  base::PolygonFType polygon_out;
  base::PointF tmp_pt;
  size_t i, j;
  for (i = 0; i < 8; i++) {
    for (j = 0; j < 8; j++) {
      tmp_pt.x = 0.5f * i;
      tmp_pt.y = 0.5f * j;
      tmp_pt.z = 0;
      *(polygon_in_ptr->at(i, j)) = tmp_pt;
    }
  }
  for (i = 0; i < 16; i++) {
    for (j = 0; j < 16; j++) {
      tmp_pt.x = 0.5f * i + 100;
      tmp_pt.y = 0.5f * j + 100;
      tmp_pt.z = 0;
      *(polygon_in_ptr->at(i, j)) = tmp_pt;
    }
  }
  condition_clustering.set_cloud(polygon_in_ptr);
  condition_clustering.set_candidate_function(easy_candidate);
  condition_clustering.set_condition_function(easy_condition);
  condition_clustering.set_min_cluster_size(1);
  condition_clustering.set_max_cluster_size(10);
  IndicesClusters indices_clusters;
  EXPECT_EQ(indices_clusters.size(), 0);
  condition_clustering.Segment(&indices_clusters);
  EXPECT_EQ(indices_clusters.size(), 256);
  indices_clusters.resize(0);
  condition_clustering.set_min_cluster_size(100);
  condition_clustering.set_max_cluster_size(1000);
  condition_clustering.Segment(&indices_clusters);
  EXPECT_EQ(indices_clusters.size(), 0);
  indices_clusters.resize(0);
  condition_clustering.set_candidate_function(easy_candidate_all);
  condition_clustering.set_min_cluster_size(1);
  condition_clustering.set_max_cluster_size(256);
  condition_clustering.Segment(&indices_clusters);
  EXPECT_EQ(indices_clusters.size(), 1);
  indices_clusters.resize(0);

  indices_clusters.resize(0);
  condition_clustering.set_candidate_function(easy_candidate_error);
  condition_clustering.Segment(&indices_clusters);
  EXPECT_EQ(indices_clusters.size(), 256);

  indices_clusters.resize(0);
  condition_clustering.set_candidate_function(easy_candidate);
  condition_clustering.set_condition_function(easy_condition_false);
  condition_clustering.set_min_cluster_size(1);
  condition_clustering.set_max_cluster_size(256);
  condition_clustering.Segment(&indices_clusters);
  EXPECT_EQ(indices_clusters.size(), 256);
}

TEST(ConditionalClusteringTest, conditional_clustering_test1) {
  ConditionClustering<base::PointF> condition_clustering =
      ConditionClustering<base::PointF>(true);
  base::PolygonFTypePtr polygon_in_ptr =
      base::PolygonFTypePtr(new base::PolygonFType(16, 16, base::PointF()));
  base::PolygonFType polygon_out;
  base::PointF tmp_pt;
  size_t i, j;
  for (i = 0; i < 8; i++) {
    for (j = 0; j < 8; j++) {
      tmp_pt.x = 0.5f * i;
      tmp_pt.y = 0.5f * j;
      tmp_pt.z = 0;
      *(polygon_in_ptr->at(i, j)) = tmp_pt;
    }
  }
  for (i = 0; i < 16; i++) {
    for (j = 0; j < 16; j++) {
      tmp_pt.x = 0.5f * i + 100;
      tmp_pt.y = 0.5f * j + 100;
      tmp_pt.z = 0;
      *(polygon_in_ptr->at(i, j)) = tmp_pt;
    }
  }
  condition_clustering.set_cloud(polygon_in_ptr);
  condition_clustering.set_candidate_function(easy_candidate);
  condition_clustering.set_condition_function(easy_condition);
  condition_clustering.set_min_cluster_size(1);
  condition_clustering.set_max_cluster_size(10);
  IndicesClusters indices_clusters;
  EXPECT_EQ(indices_clusters.size(), 0);
  condition_clustering.Segment(&indices_clusters);
  EXPECT_EQ(indices_clusters.size(), 256);
  indices_clusters.resize(0);
  condition_clustering.set_min_cluster_size(100);
  condition_clustering.set_max_cluster_size(1000);
  condition_clustering.Segment(&indices_clusters);
  EXPECT_EQ(indices_clusters.size(), 0);
  indices_clusters.resize(0);
  condition_clustering.set_candidate_function(easy_candidate_all);
  condition_clustering.set_min_cluster_size(1);
  condition_clustering.set_max_cluster_size(256);
  condition_clustering.Segment(&indices_clusters);
  EXPECT_EQ(indices_clusters.size(), 1);
  indices_clusters.resize(0);

  indices_clusters.resize(0);
  condition_clustering.set_candidate_function(easy_candidate_error);
  condition_clustering.Segment(&indices_clusters);
  EXPECT_EQ(indices_clusters.size(), 256);

  indices_clusters.resize(0);
  condition_clustering.set_candidate_function(easy_candidate);
  condition_clustering.set_condition_function(easy_condition_false);
  condition_clustering.set_min_cluster_size(1);
  condition_clustering.set_max_cluster_size(256);
  condition_clustering.Segment(&indices_clusters);
  EXPECT_EQ(indices_clusters.size(), 256);
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
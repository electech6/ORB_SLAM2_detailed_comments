/**
 * File: FeatureVector.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: feature vector
 * License: see the LICENSE.txt file
 *
 */

#include "FeatureVector.h"
#include <map>
#include <vector>
#include <iostream>

namespace DBoW2 {

// ---------------------------------------------------------------------------

FeatureVector::FeatureVector(void)
{
}

// ---------------------------------------------------------------------------

FeatureVector::~FeatureVector(void)
{
}

// ---------------------------------------------------------------------------

/**
 * @brief 把node id下所有的特征点的索引值归属到它的向量里
 * 
 * @param[in] id              节点ID，内部包含很多单词
 * @param[in] i_feature       特征点在图像中的索引
 */
void FeatureVector::addFeature(NodeId id, unsigned int i_feature)
{
  // 返回指向大于等于id的第一个值的位置
  FeatureVector::iterator vit = this->lower_bound(id);
  // 将同样node id下的特征点索引值放在一个向量里
  if(vit != this->end() && vit->first == id)
  {
    // 如果这个node id已经创建，可以直接插入特征点索引
    vit->second.push_back(i_feature);
  }
  else
  {
    // 如果这个node id还未创建，创建后再插入特征点索引
    vit = this->insert(vit, FeatureVector::value_type(id, std::vector<unsigned int>() ));
    vit->second.push_back(i_feature);
  }
}

// ---------------------------------------------------------------------------

std::ostream& operator<<(std::ostream &out, 
  const FeatureVector &v)
{
  if(!v.empty())
  {
    FeatureVector::const_iterator vit = v.begin();
    
    const std::vector<unsigned int>* f = &vit->second;

    out << "<" << vit->first << ": [";
    if(!f->empty()) out << (*f)[0];
    for(unsigned int i = 1; i < f->size(); ++i)
    {
      out << ", " << (*f)[i];
    }
    out << "]>";
    
    for(++vit; vit != v.end(); ++vit)
    {
      f = &vit->second;
      
      out << ", <" << vit->first << ": [";
      if(!f->empty()) out << (*f)[0];
      for(unsigned int i = 1; i < f->size(); ++i)
      {
        out << ", " << (*f)[i];
      }
      out << "]>";
    }
  }
  
  return out;  
}

// ---------------------------------------------------------------------------

} // namespace DBoW2

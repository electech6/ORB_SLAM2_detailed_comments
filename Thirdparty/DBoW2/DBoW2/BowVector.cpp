/**
 * File: BowVector.cpp
 * Date: March 2011
 * Author: Dorian Galvez-Lopez
 * Description: bag of words vector
 * License: see the LICENSE.txt file
 *
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "BowVector.h"

namespace DBoW2 {

// --------------------------------------------------------------------------

BowVector::BowVector(void)
{
}

// --------------------------------------------------------------------------

BowVector::~BowVector(void)
{
}

// --------------------------------------------------------------------------

/**
 * @brief 更新BowVector中的单词权重
 * 
 * @param[in] id    单词的ID
 * @param[in] v     单词的权重
 */
void BowVector::addWeight(WordId id, WordValue v)
{
  // 返回指向大于等于id的第一个值的位置
  BowVector::iterator vit = this->lower_bound(id);
  
  
  // http://www.cplusplus.com/reference/map/map/key_comp/
  if(vit != this->end() && !(this->key_comp()(id, vit->first)))
  {
    // 如果id = vit->first, 说明是同一个Word，权重更新 
    vit->second += v;
  }
  else
  {
    // 如果该Word id不在BowVector中，新添加进来
    this->insert(vit, BowVector::value_type(id, v));
  }
}

// --------------------------------------------------------------------------

void BowVector::addIfNotExist(WordId id, WordValue v)
{
  BowVector::iterator vit = this->lower_bound(id);
  
  if(vit == this->end() || (this->key_comp()(id, vit->first)))
  {
    this->insert(vit, BowVector::value_type(id, v));
  }
}

// --------------------------------------------------------------------------

void BowVector::normalize(LNorm norm_type)
{
  double norm = 0.0; 
  BowVector::iterator it;

  if(norm_type == DBoW2::L1)
  {
    for(it = begin(); it != end(); ++it)
      norm += fabs(it->second);
  }
  else
  {
    for(it = begin(); it != end(); ++it)
      norm += it->second * it->second;
		norm = sqrt(norm);  
  }

  if(norm > 0.0)
  {
    for(it = begin(); it != end(); ++it)
      it->second /= norm;
  }
}

// --------------------------------------------------------------------------

std::ostream& operator<< (std::ostream &out, const BowVector &v)
{
  BowVector::const_iterator vit;
  std::vector<unsigned int>::const_iterator iit;
  unsigned int i = 0; 
  const unsigned int N = v.size();
  for(vit = v.begin(); vit != v.end(); ++vit, ++i)
  {
    out << "<" << vit->first << ", " << vit->second << ">";
    
    if(i < N-1) out << ", ";
  }
  return out;
}

// --------------------------------------------------------------------------

void BowVector::saveM(const std::string &filename, size_t W) const
{
  std::fstream f(filename.c_str(), std::ios::out);
  
  WordId last = 0;
  BowVector::const_iterator bit;
  for(bit = this->begin(); bit != this->end(); ++bit)
  {
    for(; last < bit->first; ++last)
    {
      f << "0 ";
    }
    f << bit->second << " ";
    
    last = bit->first + 1;
  }
  for(; last < (WordId)W; ++last)
    f << "0 ";
  
  f.close();
}

// --------------------------------------------------------------------------

} // namespace DBoW2


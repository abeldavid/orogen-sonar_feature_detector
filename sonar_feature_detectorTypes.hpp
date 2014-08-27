#ifndef sonar_feature_detector_TYPES_HPP
#define sonar_feature_detector_TYPES_HPP

#include <vector>
#include "base/Eigen.hpp"
#include "base/Time.hpp"

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace sonar_feature_detector {
  
  struct SonarFeature{
    base::Vector2d position;
    base::Vector2d span;
    double avg_confidence;
    double sum_confidence;
    double confidence;
    int number_of_cells;
    
    /**
     * < operator for sorting of features. ATTENTION: operator is reversed, to achive a reversed sorting
     */
    bool operator< ( SonarFeature const& rhs) const
    { 
      return ( confidence > rhs.confidence  ); 
    }    
    
    
  };
  
  struct SonarFeatures{
    
    base::Time time;
    std::vector<SonarFeature> features;
    
  };
  
   
}

#endif


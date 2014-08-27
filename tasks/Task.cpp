/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <stack>
#include <algorithm>

using namespace sonar_feature_detector;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine, initial_state)
{
}

Task::~Task()
{
}


bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    
    uw_localization::SimpleGrid grid;
    
    if(_grid_maps.readNewest(grid) == RTT::NewData){
      
      sonar_detectors::SonarFeatures features = processMap(grid);
      normFeatures( features);
      sortFeatures( features);
      _features.write(features);
      
    }
    
    
}

sonar_detectors::SonarFeatures Task::processMap(uw_localization::SimpleGrid &grid){
  
  //Set borders for the graph search -> use a minimal distance to the wall
  
  bottom_left_corner = (-1.0 * _map_origin.get()) + base::Vector2d(_minimum_wall_distance.get(), _minimum_wall_distance.get() );
  upper_right_corner = (-1.0 * _map_origin.get()) + _map_span.get()
    - base::Vector2d(_minimum_wall_distance.get(), _minimum_wall_distance.get() );
  
  std::stack<base::Vector2d> points_todo;
  uw_localization::SimpleGridElement elem;
  sonar_detectors::SonarFeatures features;
  features.time = grid.time;
  
  //Perform graph search
  
  //search for begining of regions
  for( double x = bottom_left_corner.x() ; x < upper_right_corner.x(); x += grid.resolution){
   
    for( double y = bottom_left_corner.y(); y < upper_right_corner.y(); y += grid.resolution){
      
      if(checkObstacle(grid, x, y) ){
        
        base::Vector2d sum_pos(0.0, 0.0);
        double sum_weight = 0.0;
        base::Vector2d max(-INFINITY, -INFINITY);
        base::Vector2d min(INFINITY, INFINITY);
        int count_cells = 0;
        
        points_todo.push(base::Vector2d(x,y));
          
        while(!points_todo.empty()){
            
          base::Vector2d act_pos = points_todo.top();
          points_todo.pop();
          grid.getCell(act_pos.x(), act_pos.y(), elem);
          elem.flag = true;
          grid.setCell(act_pos.x(), act_pos.y(), elem);
          
          sum_pos += act_pos * elem.obstacle_conf;
          sum_weight += elem.obstacle_conf;
          count_cells++;
          
          //Correct minimum and maximum coordinates
          if(act_pos.x() > max.x()){
            max.x() = act_pos.x();
          }
          
          if(act_pos.y() > max.y()){
            max.y() = act_pos.y();
          }
          
          if(act_pos.x() < min.x()){
            min.x() = act_pos.x();
          }
          
          if(act_pos.y() < min.y()){
            min.y() = act_pos.y();
          }          
          
          //Check neighborhood and add to the stack          
          for(double x_offset = - grid.resolution; x_offset <= grid.resolution; x_offset += grid.resolution){
            
            for(double y_offset = -grid.resolution; y_offset <= grid.resolution; y_offset += grid.resolution){
              
              //This is no neighbor, this is our actual position -> ignore this one
              if(x_offset == 0.0 && y_offset == 0.0){
                continue;
              }
              
              //Check neighbor-cell and add it to the stack
              if(checkObstacle(grid, act_pos.x() + x_offset, act_pos.y() + y_offset) ){

                points_todo.push( base::Vector2d(act_pos.x() + x_offset, act_pos.y() + y_offset) );
                
              }              
              
            }      
            
          }      
         
                    
          
        }      
        
        if(sum_weight > 0.0 && count_cells >= _minimum_object_cells.get()){
          
          sonar_detectors::SonarFeature feature;
          feature.position = sum_pos * (1.0 / sum_weight);
          feature.sum_confidence = sum_weight;
          feature.avg_confidence = sum_weight / count_cells;
          feature.span = max - min + base::Vector2d(1.0, 1.0); 
          feature.number_of_cells = count_cells;          
          
          double size = feature.span.norm();
          feature.confidence = feature.avg_confidence * ( 1.0 / ( std::fabs(size - _optimal_object_size.get() )  ) );
          
          
          features.features.push_back(feature);
        }
        
        
      }  
      
      
    }    
    
  }
  
  return features;
  
}

bool Task::checkObstacle(uw_localization::SimpleGrid &grid, double x, double y){
  
  if(checkCoordinate(base::Vector2d(x,y) ) == false){
    return false;
  }
  
  uw_localization::SimpleGridElement elem;
  
  if(grid.getCell(x, y, elem) ){
   
    if(elem.obstacle && elem.obstacle_conf > 0.0 && (!elem.flag)){
      return true;
    }
    
  }    
  
  return false;
}


bool Task::checkCoordinate(base::Vector2d pos){
 
  if(pos.x() < bottom_left_corner.x() || pos.x() > upper_right_corner.x()){
    return false;
  }
  
  if(pos.y() < bottom_left_corner.y() || pos.y() > upper_right_corner.y()){
    return false;
  }
    
  return true;  
}

void Task::normFeatures(sonar_detectors::SonarFeatures &features){
  
  double sum_confidence = 0.0;
  
  for(std::vector<sonar_detectors::SonarFeature>::iterator it = features.features.begin(); it != features.features.end(); it++){
    sum_confidence += it->confidence;
    
  }
  
  if(sum_confidence != 0.0){
  
    for(std::vector<sonar_detectors::SonarFeature>::iterator it = features.features.begin(); it != features.features.end(); it++){
      it->confidence *= (1.0 / sum_confidence);
      
    }
    
  }
  
  
}

void Task::sortFeatures(sonar_detectors::SonarFeatures &features){

  std::sort( features.features.begin(), features.features.end());
  
}

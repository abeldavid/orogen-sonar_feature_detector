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
    
    servoing_mode = false;
    fixed_map = false;
    servoing_finished = false;
    base::Vector2d middle = (-1.0 * _map_origin.get()) + (0.5 * _map_span.get());
    lastRBS.position = base::Vector3d(middle.x(), middle.y(), 0.0); //Init position in the middle of the map
    
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    
    uw_localization::SimpleGrid grid;
    
    if(_grid_maps.readNewest(grid) == RTT::NewData){
      
      
      if(!servoing_mode){
        
        if(state() != BUILDING_MAP){
          state(BUILDING_MAP);
        }
        
        features = processMap(grid);
        sortFeatures( features);
        normFeatures( features);
        sortFeaturesOptimalRoute( features);
        
        if(fixed_map && features.features.size() > _minimum_number_of_targets.get()){
          servoing_mode = true;
          target_features = features;
          _next_target.write(base::Vector3d(target_features.features[0].position.x(), target_features.features[0].position.y(), 0.0) );
          _next_target_feature.write(target_features.features[0]);
          
          base::LinearAngular6DCommand cmd;
          cmd.time = grid.time;
          cmd.linear(0) = target_features.features[0].position.x();
          cmd.linear(1) = target_features.features[0].position.y();
          cmd.linear(2) = _servoing_depth.get();
          cmd.angular(0) = 0.0;
          cmd.angular(1) = 0.0;
          cmd.angular(2) = 0.0;
          _next_target_command.write(cmd);
          
          state(TARGET_SERVOING);
        }
        else if(fixed_map){ //We want to fix the map, but we have not ennough features
          
          state(NOT_ENOUGH_TARGETS);
          fixed_map = false;
          
        }
        
      }
      _features.write(features);
      
    }
    
    
    if(_pose_samples.readNewest(lastRBS) == RTT::NewData){
    
      
      if(servoing_mode && (!servoing_finished) ){
      
        if(target_features.features.size() > 0){
          
          sonar_detectors::SonarFeature f = target_features.features[0];
          
          //Vehicle reached the position of the feature -> select new feature
          if( std::fabs(lastRBS.position.x() - f.position.x()) < f.span.x() * 0.5 &&
            std::fabs(lastRBS.position.y() - f.position.y()) < f.span.y() * 0.5){
            
            target_features.features.erase( target_features.features.begin());
            
            state(REACHED_TARGET);
          
            if(target_features.features.size() > 0){
              
              _next_target.write(base::Vector3d( target_features.features[0].position.x(), target_features.features[0].position.y(), 0.0 ) );
              _next_target_feature.write( target_features.features[0] );
              
              state(TARGET_SERVOING);  
            
              
            }else{
               servoing_finished = true;
               state(SERVOING_FINISHED);
               return; 
            }
          
            
          }
          
           base::LinearAngular6DCommand cmd;
           cmd.time = lastRBS.time;
           cmd.linear(0) = target_features.features[0].position.x();
           cmd.linear(1) = target_features.features[0].position.y();
           cmd.linear(2) = _servoing_depth.get();
           cmd.angular(0) = 0.0;
           cmd.angular(1) = 0.0;
           cmd.angular(2) = std::atan2(target_features.features[0].position.y() - lastRBS.position.y(),
                                       target_features.features[0].position.x() - lastRBS.position.x());
           _next_target_command.write(cmd); 
          
          
        }else{
          servoing_finished = true;
          state(SERVOING_FINISHED);
          
        } 
        
      
      }
      
    }    
    
    
}

sonar_detectors::SonarFeatures Task::processMap(uw_localization::SimpleGrid &grid){
  
  //Set borders for the graph search -> use a minimal distance to the wall
  
  bottom_left_corner = (-1.0 * grid.origin) + base::Vector2d(_minimum_wall_distance.get(), _minimum_wall_distance.get() );
  upper_right_corner = (-1.0 * grid.origin) + grid.span
    - base::Vector2d(_minimum_wall_distance.get(), _minimum_wall_distance.get() );
    
  bottom_left_wall = (-1.0 * grid.origin);
  upper_right_wall = (-1.0 * grid.origin) + grid.span;
  
  std::stack<base::Vector2d> points_todo;
  uw_localization::SimpleGridElement elem;
  sonar_detectors::SonarFeatures features;
  features.time = grid.time;
  
  //Perform graph search
  
  //search for begining of regions
  for( double x = bottom_left_wall.x() ; x < upper_right_wall.x(); x += grid.resolution){
   
    for( double y = bottom_left_wall.y(); y < upper_right_wall.y(); y += grid.resolution){
      
      if(checkObstacle(grid, x, y) ){
        
        base::Vector2d sum_pos(0.0, 0.0);
        double sum_weight = 0.0;
        base::Vector2d max(-INFINITY, -INFINITY);
        base::Vector2d min(INFINITY, INFINITY);
        int count_cells = 0;
        bool touch_border = false;
        
        points_todo.push(base::Vector2d(x,y));
          
        while(!points_todo.empty()){
            
          base::Vector2d act_pos = points_todo.top();
          points_todo.pop();
          grid.getCell(act_pos.x(), act_pos.y(), elem);
          elem.flag = true;
          grid.setCell(act_pos.x(), act_pos.y(), elem);
          
          if(!checkCoordinate(act_pos)){
            
            if(_filter_border_structures.get()){
              touch_border = true;
            }
            else{
              continue;
            }
            
          }else{
          
            sum_pos += act_pos * elem.obstacle_conf;
            sum_weight += elem.obstacle_conf;
            count_cells++;
          }
          
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
        
        if(sum_weight > 0.0 && count_cells >= _minimum_object_cells.get() && (!touch_border) ){
          
          sonar_detectors::SonarFeature feature;
          feature.position = sum_pos * (1.0 / sum_weight);
          feature.sum_confidence = sum_weight;
          feature.avg_confidence = sum_weight / count_cells;
          feature.span = max - min + base::Vector2d(grid.resolution, grid.resolution); 
          feature.number_of_cells = count_cells;          
          
          double size = feature.span.norm();
          feature.confidence = feature.avg_confidence * ( 1.0 / ( std::fabs(size - _optimal_object_size.get() + 1.0 )  ) );
          
          
          features.features.push_back(feature);
        }
        
        
      }  
      
      
    }    
    
  }
  
  return features;
  
}

bool Task::checkObstacle(uw_localization::SimpleGrid &grid, double x, double y){
  
  if(checkInsideWalls(base::Vector2d(x,y) ) == false){
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

bool Task::checkInsideWalls(base::Vector2d pos){
  
  if(pos.x() < bottom_left_wall.x() || pos.x() > upper_right_wall.x()){
    return false;
  }
  
  if(pos.y() < bottom_left_wall.y() || pos.y() > upper_right_wall.y()){
    return false;
  }
  
  return true;
  
}


void Task::normFeatures(sonar_detectors::SonarFeatures &features){
  
  //we have no features -> do nothing
  if(features.features.size() == 0){
    return;
  }
  
  double best_confidence = features.features.begin()->confidence;
  
  if(best_confidence != 0.0){
  
    for(std::vector<sonar_detectors::SonarFeature>::iterator it = features.features.begin(); it != features.features.end(); it++){
      it->confidence *= (1.0 / best_confidence);
      
    }
    
  }
  
  
}

void Task::sortFeatures(sonar_detectors::SonarFeatures &features){

  std::sort( features.features.begin(), features.features.end());
  
}

void Task::sortFeaturesOptimalRoute(sonar_detectors::SonarFeatures &features){
  
  if(features.features.size() > 0){
  
    std::vector<sonar_detectors::SonarFeature> cp = features.features;
    features.features.clear();
    
    base::Vector2d pos(lastRBS.position.x(), lastRBS.position.y());
    sonar_detectors::SonarFeature f;
    
    while(cp.size() > 0){
      
      double min = INFINITY;
      
      std::vector<sonar_detectors::SonarFeature>::iterator min_it = cp.begin();
      
      for(std::vector<sonar_detectors::SonarFeature>::iterator it = cp.begin(); it != cp.end(); it++){
        
        double dist = (it->position - pos).norm() * ( 1.0 +  ( _confidence_weight.get() * (1.0 - it->confidence) ) ) ;
        
        if(dist < min){
          min = dist;
          min_it = it;
        }
        
        
      }      
      
      f = *min_it;
      pos = f.position;
      features.features.push_back(f);
      cp.erase(min_it);
      
      
    }
  
  }
  
}


void Task::fix_map(){
  fixed_map = true;
}  
  

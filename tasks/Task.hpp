/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef SONAR_FEATURE_DETECTOR_TASK_TASK_HPP
#define SONAR_FEATURE_DETECTOR_TASK_TASK_HPP

#include "sonar_feature_detector/TaskBase.hpp"
#include "uw_localization/types/map.hpp"
#include "sonar_detectors/SonarDetectorTypes.hpp"
#include "base/samples/RigidBodyState.hpp"

namespace sonar_feature_detector {


    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * import_types_from "sonar_feature_detectorTypes.hpp"
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','sonar_feature_detector::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

      base::Vector2d upper_right_corner;
      base::Vector2d bottom_left_corner;
      
      /**
       * Perform the graph-serch-algorithm on the map and find conected components
       */
      sonar_detectors::SonarFeatures processMap(uw_localization::SimpleGrid &grid);
      
      /**
       * Check, if there is a obstacle at a given position without a flag
       */
      bool checkObstacle(uw_localization::SimpleGrid &grid, double x, double y);
      
      /**
       * Check if a coordinate is inside our map-boundaries
       */
      bool checkCoordinate(base::Vector2d pos);
      
      /**
       * Normalize the confidence of all features, so the confidence sum is 1.0
       */
      void normFeatures(sonar_detectors::SonarFeatures &features);
      
      /**
       * Sort the features, based on their confidence
       */
      void sortFeatures(sonar_detectors::SonarFeatures &features);
      
      /**
       * Sort the particles, based on their confidences and the optimal path
       */
      void sortFeaturesOptimalRoute(sonar_detectors::SonarFeatures &features);      
      
      bool servoing_mode;
      bool fixed_map;
      bool servoing_finished;
      sonar_detectors::SonarFeatures features;
      sonar_detectors::SonarFeatures target_features;


    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "sonar_feature_detector::Task", TaskCore::TaskState initial_state = Stopped);

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

        /** Default deconstructor of Task
         */
	~Task();


        bool startHook();


        void updateHook();
        
        void fix_map();
        
    };
}

#endif

